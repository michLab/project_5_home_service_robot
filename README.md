# Map my world

A fourth project from Udacity Robotics Software Engineer Nanodegree.
A robot performs RTAB-MAP SLAM using odometry and laser scans.

<p align="center">
  <img width="920" height="600" src="images/rtab-slam.gif">
  <br>Robot performing RTAB-Map Slam
</p>


## How to use

In order to run this package follow this steps:
* Make directory for catkin workspace:
```sh
mkdir -p path_to_my_workspace/workspace_name/src
```
* Go to the src directory and init catkin workspace:
```sh
cd path_to_my_workspace/workspace_name/src
catkin_init_workspace
```
* Go to `workspace_name` directory and pull this repository:
```sh
git init
git remote add origin link_to_this_repo
git pull
git checkout master
```
* Make catkin:
```sh
cd path_to_my_workspace/workspace_name
catkin_make
source devel/setup.bash
```
* Run ROS:
```sh
export GAZEBO_MODEL_PATH=path_to_my_workspace/workspace_name/src/my_robot/models
roslaunch my_robot world.launch
```
* In another terminal run rtab-map:
```sh
cd path_to_my_workspace/workspace_name
source devel/setup.bash
roslaunch my_robot mapping.launch
```
* If you want to use Teleop in third terminal write:
```sh
cd path_to_my_workspace/workspace_name
source devel/setup.bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
* Final map (database) is stored in default location:
```sh
~/.ros/rtabmap.db
```

## World
A single floor with many details (furniture, plants, portraits, boxes etc.)

<p align="center">
  <img width="460" height="300" src="images/world_global.png">
  <br>World - a global overview
</p>

<p align="center">
  <img width="460" height="300" src="images/world_local_1.png">
  <br>World - local view
</p>

<p align="center">
  <img width="460" height="300" src="images/world_local_2.png">
  <br>World - local view #2
</p>


## Nodes
Three nodes are essential for where am I:
* `rtabmap` - performs slam  ([RTAB-Map](http://wiki.ros.org/rtabmap_ros))

## Params
In order to tune rtab map use parameters from package wiki and paste them to package launch file (mapping.launch).

## rtabmap-databaseViewer
It allows for complete analysis of mapping session.

<p align="center">
  <img width="460" height="300" src="images/rtab-map-database-viewer.png">
  <br>Mapping results
</p>


## License
The contents of this repository are covered under the [MIT License](./LICENSE.txt)


## Contributing

1. Fork it (<https://github.com/michLab/project_4_mapping.git>)
2. Create your feature branch (`git checkout -b feature/fooBar`)
3. Commit your changes (`git commit -am 'Add some fooBar'`)
4. Push to the branch (`git push origin feature/fooBar`)
5. Create a new Pull Request
