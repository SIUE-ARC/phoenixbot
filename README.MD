# Phoenixbot

## Installation

### Install udev rules
```
sudo ln -s $(pwd)/99-phoenixbot.rules /etc/udev/rules.d/99-phoenixbot.rules
sudo udevadm control --reload
sudo udevadm trigger
```

## Running in gazebo

### MRDC Autonomous Simulation
Runs phoenixbot in the 2018 MRDC world with the autonomous behavior coordinator.
```
roslaunch phoenixbot_launch phoenixbot_auto2018.launch sim:=true
```

### Teleop Simulation
Runs phoenixbot in an empty world, with teleop_twist_joy and the
phoenixbot_teleop node.
```
roslaunch phoenixbot_launch phoenixbot_teleop.launch sim:=true
```

### SLAM Simulation
Runs phoenixbot in the Willow Garage world using SLAM as a map source instead
of a static map. Starts navigation stack but provides no other control
methods.
```
roslaunch phoenixbot_launch phoenixbot_slam.launch sim:=true
```

### View Simulation Model
Launches low level simulation nodes to test simulation models.
```
roslaunch phoenixbot_launch phoenixbot_model.launch
roslaunch phoenixbot_launch phoenixbot_model.launch world:=worlds/to_test.world
```

## Running on hardware
It is critically important that the simon arm be resting on the top bar before
powering on the arduino, otherwise the position of the arm will not be initialized
correctly.

## Connect to phonenixbot
ssh arc@phoenixbot.local
ROS_MASTER_URI="http://phoenixbot.local:11311" do this for every terminals
to disconnect: sudo shutdown -h now

