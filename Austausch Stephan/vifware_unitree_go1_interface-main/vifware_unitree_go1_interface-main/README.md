# vifware_unitree_go1_interface

todo docu

## in case of a native build

install following apt packages

ros-humble-realtime-tools
ros-humble-autoware-adapi-v1-msgs
ros-humble-autoware-control-msgs
ros-humble-autoware-common-msgs
ros-humble-autoware-vehicle-msgs



## clone

 tier4_autoware_msgs
 tinyfsm_ros2
 vifware_vehicle_interfaces


## fetch dependencies

```vcs import < unitree_go1.repos```

### install dependencies

sudo apt install liblcm-dev (version 1.3)

or follow instructions from unitree


```sudo apt install openjdk-8-jdk
sudo update-alternatives --config javac
# choose the java-8-openjdk option
cd lcm-1.4.0
mkdir build
cd build
cmake ..
make
sudo make install # this will install lcm
sudo ldconfig -v # updates the shared library cache
# after this, revert back to the previous version of openjdk:
sudo update-alternatives --config javac # and choose whichever version was selected previously (typically java-11-openjdk)
```
## Troubleshooting

In case you get the error "[Warning]: RouDi not found - waiting ..." run in another terminal:
```
$iox-roudi
```