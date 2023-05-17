# Set up
- clone ```https://github.com/AndrejOrsula/panda_ign_moveit2``` to your ros2 src folder
- to specify a controller: 
  - you must change the ```ros2_control_command_interface``` at line 44 in ```panda.urdf.xacro```
  - and you must change the yaml file at line 48 from the launch file
- go into the config folder from this package and run ```xacro panda.urdf.xacro > panda.urdf```