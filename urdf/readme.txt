
URDF can not specify the pose of the robot itself within a world. It is also not a universal description format since it cannot specify joint loops (parallel linkages), and it lacks friction and other properties. Additionally, it cannot specify things that are not robots, such as lights, heightmaps, etc.

SDF has been tried to specify friction in the model, but it turns out friction can be determined with a gazebo plugin. Check the urdf file of conveyor belt for details.


