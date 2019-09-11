# Differential-drive robots

## Definition

A differential-drive robot (DDR from now) is a robot which has a two-wheel mechanism for allowing it to move. There is a third wheel which prevents the robot from falling and helps it balance, but which has not active role in the movement of the robot.  
This mechanism for moving the robot is based on the difference between the two wheels angular velocities, which is why it is called _differential_-drive.

Before starting to describe how it works, it's important to notice that there are four velocities involved: The two velocities that correspond to the wheels and the angular and linear velocity of the robot.

## Functioning-principles

The moving direction in a two-dimensional plane can be described by two parameters/ways of movement:

- Traslation
- Rotation

We'll define both parameters relative to the middle of the two wheels (that is, at the center of the axis that passes through both wheels).  
We'll start by explaining the simple cases: when each of the parameters appears alone. That is, without the presence of the other parameter.
**Pure traslation** in the DDR occurs only when the two wheels angular velocities have the same module and sign. Similarly, **pure rotation** occurs in the DDR only when the two wheels have the same angular-velocity module but different sign.  
In the rest of the cases, both parameters are present together. So, in an intuitive way, we can see that when the angular-velocities will have the same sign but different modules, the robot will move in the same direction as the direction of the angular velocities sign, but with a rotational movement, which becomes more predominant as the difference between the modules grows. The extreme case of this scenario is when one of the wheels has no angular velocity at all, in which case the robot rotates along the Z-axis that passes through the wheel that has no velocity (supposing that the plane in which the DDR moves is in the X-Y plane). That is, it has rotation and traslation with our definition of the reference as the center of the robot, but pure rotation if we define the reference in the zero-velocity wheel.

Besides the angular velocity of the wheels, there are two more factors that define the pose variation (position and orientation) of the robot. These are the **wheels radius** and the **distance between the two wheels**.  
Intuitively, its easy to see that if we make longer the distance between the wheels, greater angular velocities of the wheels will be needed for getting the same angular velocity of the robot as with shorter wheel distance. This is because as the wheels distance becomes larger, the circumference which will be involved in the angular velocity of the robot will be bigger, which makes the rotational velocity of the robot lower.  
With the wheels radius the opposite effect takes place. As the radius becomes bigger, with constant wheel angular velocity, the velocity of the robot increases.

## Mathematical model

![Reference graphic](media/reference.png)

Mathematically, all that we explained before can be described with the following equations, which define the **kynematic model** of the DDR:

- V_robot = (Vr + Vl)/2
- V_x = V_robot * cos(theta)
- V_y = V_robot * sin(theta)
- w_robot = (r/L)*(Vr - Vl)  
    ('L' being the distance between the wheels, 'r' the radius of the wheels and 'theta' the angle of orientation of the robot)

From these equations, we can rearrange them such that we can define both wheels velocities as function of the linear and angular velocity desired for the robot. We'll want to do that because the system is moved controlling the wheel velocities, but the real parameters that we want to control are the linear and angular velocities _of the robot_. After rearranging, the equations are:

- V_r = (V_robot + (L/2)w_robot)
- V_l = (V_robot - (L/2)w_robot)

Finally, there are a few physical considerations that can introduce error to the equations described before, or can make the model for movement more complex. These simplifications are:

- Both wheels velocities are constant when they are present. That is, there is no transient state in the velocity of the wheels.
- The wheels do not slip
- The surface is flat
- The wheels have a fixed position, and can't move in any way except for their angular velocity. (As opposed to the case of, for example, a mecanum wheel)
- Dynamic properties. This model only considers the kinematics of the DDR (that is, positions and velocities), and not takes into account the effect of friction, torques and acceleration.

Ignoring these considerations helps us find a simple model for movement, but reduces the cases in which we can use the equations described above. So it's important to know whether we can apply these equations or not every time we're considering using a DDR.

### References

- <https://pdfs.semanticscholar.org/edde/fa921e26efbbfd6c65ad1e13af0bbbc1b946.pdf>
- <http://planning.cs.uiuc.edu/node659.html>
- <https://www.omicsonline.org/open-access/dynamic-modelling-of-differentialdrive-mobile-robots-using-lagrange-and-newtoneuler-methodologies-a-unified-framework-2168-9695.1000107.php?aid=19201>
