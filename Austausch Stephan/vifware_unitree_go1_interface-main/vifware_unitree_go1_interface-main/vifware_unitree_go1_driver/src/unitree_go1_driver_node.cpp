// Copyright VIF

#include <rclcpp/rclcpp.hpp>

#include <memory>

// components
#include <vifware_unitree_go1_driver/unitree_go1_driver.hpp>

#include <autoware_interface/autoware_interface.hpp>


int main(int argc, char ** argv)
{
    // Force flush of the stdout buffer.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Initialize any global resources needed by the middleware and the client library.
    // This will also parse command line arguments one day (as of Beta 1 they are not used).
    // You must call this before using any other part of the ROS system.
    // This should be called once per process.
    rclcpp::init(argc, argv);

    // Create an executor that will be responsible for execution of callbacks for a set of nodes.
    // With this version, all callbacks will be called from within this thread (the main one).
    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;

    // Add some nodes to the executor which provide work for the executor during its "spin" function.
    // An example of available work is executing a subscription callback, or a timer callback.
    auto vehicle = std::make_shared<vifware_unitree_go1_interface::UnitreeGo1Driver>(options);
    exec.add_node(vehicle->get_node_base_interface());

    // auto autoware_interface = std::make_shared<vifware_vehicle_interfaces::AutowareInterface>(options);
    // exec.add_node(autoware_interface);

    // spin will block until work comes in, execute work as it becomes available, and keep blocking.
    // It will only be interrupted by Ctrl-C.
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
