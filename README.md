# Home service robot

A fifth and final project from Udacity Robotics Software Engineer Nanodegree.
A robot moves in environment and pick virtual objects, from pick-up zone to drop-off zone.


<p align="center">
  <img width="920" height="600" src="images/home_service.gif">
  <br>Robot 
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

* In every script change path to gazebo models to: path_to_my_workspace/workspace_name/src/my_robot/models
 and set path to world file (path_to_my_workspace/workspace_name/src/my_robot/worlds/my_world.world)


## Scripts
The project contains 5 scripts:
* 'test_slam.sh' to test SLAM algorithm (the robot is controled via teleop)
* 'test_navigation.sh' which is used to set 2D nav goals for robot in rviz
* 'pick_objects.sh' which is used to automatically move robot between two positions: pick-up zone and drop-off zone. 
* 'add_makrers.sh' which is used to display virtual objects in rviz
* 'home_service.sh' which is used to perform algorithm of picking of objects and moving from pick-up zone to drop-off zone.

** To run scripts remember to set proper path as described in the end of the _How to use_ section **

## Picking algorithm (in home_service.sh)

* Initially show the marker at the pickup zone
* Hide the marker once your robot reaches the pickup zone
* Wait 5 seconds to simulate a pickup
* Show the marker at the drop off zone once your robot reaches it
* When robot leaves drop off zone show marker once again in the pick up zone
* Go to beginning


## License
The contents of this repository are covered under the [MIT License](./LICENSE.txt)


## Contributing

1. Fork it (<https://github.com/michLab/project_5_home_service_robot.git>)
2. Create your feature branch (`git checkout -b feature/fooBar`)
3. Commit your changes (`git commit -am 'Add some fooBar'`)
4. Push to the branch (`git push origin feature/fooBar`)
5. Create a new Pull Request
