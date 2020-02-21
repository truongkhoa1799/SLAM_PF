To start your particle filter run following commands in a console

1) roslaunch stage_ros stage.launch
This starts the stage simulator and its visualization

2) roslaunch laser_based_localization_pf laser_based_localization_pf.launch
This starts Rviz and the keyboard node (that you can drive around with your robot)

3) rosrun laser_based_localization_pf laser_based_localization_pf
This starts your particle filter
