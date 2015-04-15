# ros_planar_2d
ROS implementation of QUT robot arm project.

Out of the box MoveIt! does not support a 2dof arm in it's IK solver.
Installed openRAVE on Ubuntu 14.04 following these instructions: http://www.aizac.info/installing-openrave0-9-on-ubuntu-trusty-14-04-64bit/

openRAVE usage via the following: http://moveit.ros.org/wiki/Kinematics/IKFast

In order to create the file planar_2d.dae, needed to spin up an AWS server running 12.04 and ROS hydro, then scp files between host ROS machine and AWS.

To successfully use custom ikfast 2D solver, must clone the source for package moveit_ikfast, and update line that specifies behavior for given IK fast solver type. In this case had to modify for the case of TranslationXY2D. 

Once all configuration is done, and a fake trajectory can be solved and executed, and we can turn off fake trajectories. See launch file for part one to see how this is achieved. 

Next steps involve creating a joint controller for our robot see joint_trajectory_controller package.