import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped, Pose, PoseStamped, Twist
from std_msgs.msg import Int64
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
import tf_transformations
import tf2_ros
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from quadropted_msgs.msg import RobotVelocity, RobotFootContact, HighState, HighCmd
from ForwardKinematics import robot_FK
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from collections import deque

class ConversionNode(Node):
    def __init__(self):
        super().__init__('conversion_node')

        #for debugging
        self.declare_parameter('verbose', False)
        self.verbose = self.get_parameter('verbose').get_parameter_value().bool_value
        if self.verbose:
            self.get_logger().info(f"Verbose mode: {self.verbose}")

        # basic parameters taken form QuadrupedOdometryNode.py
        self.declare_parameter('publish_rate', 50)
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().integer_value
        if self.verbose:
            self.get_logger().info(f"Publish rate: {publish_rate} Hz")

        self.declare_parameter('clock_topic', '/clock')
        clock_topic = self.get_parameter('clock_topic').get_parameter_value().string_value
        if self.verbose:
            self.get_logger().info(f"Clock Topic: {clock_topic}")

        # QoS Profiles from QuadrupedOdometryNode.py
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
            history=HistoryPolicy.KEEP_LAST
        )
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
            history=HistoryPolicy.KEEP_LAST
        )

        self.high_cmd_msg = HighCmd()

        # high_cmd_msg.head = msg.header           # questionable if needed

        self.high_cmd_msg.mode = 0                   # idle
        # high_cmd_msg.mode = 1                   # standing
        # high_cmd_msg.mode = 2                   # walking, following target velocity

        self.high_cmd_msg.gait_type = 0                # idle
        # high_cmd_msg.gait_type = 1                # trot walking
        # high_cmd_msg.gait_type = 2                # trot running
        # high_cmd_msg.gait_type = 3                # stair climbing
        # high_cmd_msg.gait_type = 4                # trot obstacle

        self.high_cmd_msg.speed_level = 0              # low speed
        # high_cmd_msg.speed_level = 1              # medium speed
        # high_cmd_msg.speed_level = 2              # high speed

        self.high_cmd_msg.foot_raise_height = 0.08       # in meters, default value

        self.high_cmd_msg.body_height = 0.28             # in meters, default value

        self.latest_goal = None
        self.latest_velocity = None            
        self.latest_yaw_speed = None                

        # Publishers
        self.nav2_uni = self.create_publisher(HighCmd, 'conv', qos_reliable)
        self.uni_nav2 = self.create_publisher(Odometry, 'conv', qos_reliable)

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            RobotVelocity,              
            'robot1/robot_velocity',           
            self.cmd_vel_callback,      
            qos_reliable                
        )

        self.nav_pose_sub = self.create_subscription(
            PoseStamped,
            '/robot1/goal_pose',
            self.nav_pose_callback,
            qos_reliable
        )

        self.robo_state_sub = self.create_subscription(
            HighState,                  
            'high_state',                                       # check topic name                    
            self.robo_state_callback,        
            qos_reliable                
        )
        
        # Publishers
        self.foot_contact_pub = self.create_publisher(RobotFootContact, '/robot1/foot_contact', 10)
        self.imu_pub = self.create_publisher(Imu, '/robot1/imu_plugin/out', 10)
        self.joint_cmd_pub = self.create_publisher(Float64MultiArray, '/robot1/joint_group_controller/commands', 10)
        self.robot_velocity_pub = self.create_publisher(RobotVelocity, '/robot1/robot_velocity', 10)

def cmd_vel_callback(self, msg):
    self.latest_velocity[0] = msg.linear.x     # desired forward velocity
    self.latest_velocity[1] = msg.linear.y     # desired lateral velocity
    self.latest_yaw_speed = msg.angular.z      # desired yaw rate

    self.publish_high_cmd()

def nav_pose_callback(self, msg):
    self.latest_goal_.pose.position.x = msg.position.x    # desired position x
    self.latest_goal_.pose.position.y = msg.position.y    # desired position y
    self.latest_goal_.pose.position.z = msg.position.z    # desired position z

    q = msg.pose.orientation
    quaternion = [q.x, q.y, q.z, q.w]  
    roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)

    self.latest_goal_.pose.orientation[0] = roll              # desired position roll
    self.latest_goal_.pose.orientation[1] = pitch             # desired position pitch
    self.latest_goal_.pose.orientation[2] = yaw               # desired position yaw

    self.publish_high_cmd()

def rob_state_callback(self, msg): 
    foot_contact_msg = RobotFootContact()
    foot_force = msg.foot_forces
    foot_contact_msg.contact_states = [1 if force > 5.0 else 0 for force in foot_force]     # threshold at 5.0 N, adjust as needed

    imu_msg = Imu()
    imu_msg = msg.imu                                # directly use the imu data from HighState

    joint_cmd_msg = Float64MultiArray()
    joint_cmd_msg.data = msg.motor_states[:12]       # directly use the motor positions from HighState, only first 12 values are relevant

    robot_velocity_msg = RobotVelocity()             # directly use the velocity data from HighState       
    robot_velocity_msg.linear.x = msg.velocity[1]
    robot_velocity_msg.linear.y = msg.velocity[2]
    robot_velocity_msg.angular.z = msg.yaw_speed

    self.foot_contact_pub.publish(foot_contact_msg)
    self.imu_pub.publish(imu_msg)
    self.joint_cmd_pub.publish(joint_cmd_msg)
    self.robot_velocity_pub.publish(robot_velocity_msg)

def publish_high_cmd(self):
    if self.latest_velocity is None or self.latest_goal is None:
        return

    # Update high_cmd_msg with the latest data
    self.high_cmd_msg.velocity[0] = self.latest_velocity.linear.x           # desired forward velocity
    self.high_cmd_msg.velocity[1] = self.latest_velocity.linear.y           # desired lateral velocity
    self.high_cmd_msg.yaw_speed = self.latest_velocity.angular.z            # desired yaw rate

    self.high_cmd_msg.position[0] = self.latest_goal.pose.position.x        # desired position x
    self.high_cmd_msg.position[1] = self.latest_goal.pose.position.y        # desired position y

    self.high_cmd_msg.euler[0] = self.latest_goal.pose.orientation[0]       # desired roll angle
    self.high_cmd_msg.euler[1] = self.latest_goal.pose.orientation[1]       # desired pitch angle        
    self.high_cmd_msg.euler[2] = self.latest_goal.pose.orientation[2]       # desired yaw angle

    # Publish the updated high_cmd_msg
    self.nav2_uni.publish(self.high_cmd_msg)

    if self.verbose:
        self.get_logger().info(f"Published HighCmd: pos=({self.high_cmd_msg.position}), vel=({self.high_cmd_msg.velocity}), yaw_rate={self.high_cmd_msg.yaw_speed}")

def main(args=None):
    rclpy.init(args=args)
    conversion_node = ConversionNode()
    rclpy.spin(conversion_node)
    conversion_node.destroy_node()
    rclpy.shutdown()