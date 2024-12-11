# robot_protocols

The idea of this module is to make robots hot-swappable. You can
create a new robot protocol, specify it in configuration under a `launch-profile`,
and then be able to use your way of communicating with the robot.

This is helpful for having `MockRobot` for testing, or for having a `RealFollower` 
and `RealLeader` robots.

Just implement the `BaseRobotProtocol` and you're good to go.