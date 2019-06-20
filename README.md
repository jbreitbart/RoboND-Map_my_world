# Udacity where am I?

Based on my repository for RoboND Go Chase it

* To launch gazebo and rviz: `roslaunch my_robot world.launch`
* To launch amcl for localazation: `roslaunch my_robot amcl.launch`
* To launch the teleop package: `rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

## RViz

Config should be loaded on `roslaunch`, but you may still have to set `export LC_NUMERIC="en_US.UTF-8"` if visualization is not showing correctly.
