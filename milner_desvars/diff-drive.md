# how a differential-drive robot works and how to control it

A differential-drive robot is a wheeled robot with two controllable wheels, these wheels only can turn by their axis, so rotation of the robot is accomplished by rotating the wheels in opposite directions.

You can control the robot by controlling the velocity and direction in which the left and right wheels turn, you can use control theory to move the robot by giving the general velocity and direction you want it to move, and by that you calculate the velocities of both wheels to accomplish that objective.

# Unicycle and Differential Drive Models

For the kinematics, first we going to check the Unicycle Model,  since you only move in the plane x-y, you only care about 3 states.

~~~~
x - position on the x-axis

y - position on the y-axis 

φ - phi - angle of the unicycle
~~~~

For a unicycle model, 2 inputs affect these 3 states of the robot:

~~~~
v = forward velocity (ie meters per second)

w = angular velocity (ie radians per second)
~~~~

But in the real world we are trying to control a Differential Drive robot, that as previously said, have 2 inputs to control the movement:

~~~~
v_r = clockwise angular velocity of right wheel (ie radians per second)

v_l = counter-clockwise angular velocity of left wheel (ie radians per second)
~~~~

Because the Unicycle model is easy to design around it, and fortunately for us, we can convert unicycle inputs into differential inputs, for that we have to use some values of the geometry of the wheeled robot, this values are:

~~~~
L = wheelbase (in meters per radian)

R = wheel radius (in meters per radian
~~~~

Because R is a measurement of the radius of a wheel, it makes sense to think of R as meters per radian.

For L, it is not as intuitive. In the differential drive kinematics model, you can think of L as the radius of the circle drawn by one wheel spinning while holding the other wheel still. So L is also in the units of meters per radian where the radius is of that circle’s.

we can use the kinematics for a unicycle model and kinematics for a differential drive model to come up with the following equations to convert unicycle v and w inputs into v_r and v_l differential drive inputs for our ROSbots robot.

~~~~
v_r = ((2 * v) + (w * L)) / (2 * R)

v_l = ((2 * v) - (w * L)) / (2 * R)
~~~~

The numerator for both equations are in meters per second. The denominator is in meters per radian. Both v_r and v_l result in radians per second, clock-wise and counter-clock-wise respectively— what we would expect.


# Unicycle model and Twist message

The most common way to send movement commands to the robot is with use of geometry_msgs/Twist message type. Then motor driver node should use data stored in them to control the motor.

The geometry_msgs/Twist message express velocity in free space and consists of two fields:

* Vector3 linear - represents linear part of velocity [m/s]
* Vector3 angular - represents angular part of velocity [rad/s]

The ROS Twist message can encode motion in 6 degrees of freedom (DOF)- 3 as linear motion and 3 as angular. We only need 2 DOF's - linear.x which is our v and angular.z which is our w.

So with the Twist message type, we can send to the robot with detail the two necessary inputs we need to control the movement.

