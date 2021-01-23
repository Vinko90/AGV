# How-to manually run Nav2d test

1. To be able to launch the testing environment  you need to install the following dependencies
```
sudo apt-get install libsuitesparse-dev
sudo apt-get install ros-kinetic-joy
```

2. Launch roscore
```
roscore
```

3. Set parameters "WORKING_FOLDER" and "OUTPUT_FOLDER"
```
rosparam set WORKING_FOLDER:={directory_of_repository}
rosparam set OUTPUT_FOLDER:={directoy_destination_results}
```

4. Launch the test node
```
rosrun agv_exploration_test agv_exploration_test
```

# How-to run one map

1. Launch roscore
```
roscore
```

2. Launch map launch file
```
roslaunch <map>.launch
```

3. Start mapping and exploration from agv-exploration node
```
rosrun agv-exploration agv-exploration
```
