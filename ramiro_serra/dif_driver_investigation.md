# Differential wheeled robots

A differential wheeled robot is a mobile robot whose movement is based on two separately driven wheels placed on either side of the robot body. It can thus change its direction by varying the relative rate of rotation of its wheels and hence does not require an additional steering motion.

![Robot\](https://www.google.com/url?sa=i&rct=j&q=&esrc=s&source=images&cd=&ved=2ahUKEwjQgfuB67TkAhUbIrkGHcxLCWUQjRx6BAgBEAQ&url=http://planning.cs.uiuc.edu/node659.html&psig=AOvVaw2lkNnIE3pIZqVMvjR70tXw&ust=1567606381458775 )](http://planning.cs.uiuc.edu/img5518.gif)

###### Differential wheeled robot

### How it works?

If both the wheels are driven in the same direction and speed, the robot will go in a straight line. If both wheels are turned with equal speed in opposite directions, as is clear from the diagram shown, the robot will rotate about the central point of the axis. Otherwise, depending on the speed of rotation and its direction, the center of rotation may fall anywhere on the line defined by the two contact points of the tires. While the robot is traveling in a straight line, the center of rotation is an infinite distance from the robot. Since the direction of the robot is dependent on the rate and direction of rotation of the two driven wheels, these quantities should be sensed and controlled precisely. 

To construct a simple model of the constraints that arise from the differential drive, only the distance ![ L](http://planning.cs.uiuc.edu/img83.gif) between the two wheels, and the wheel radius, ![ r](http://planning.cs.uiuc.edu/img165.gif), are necessary. The action vector ![ u = (u_r,u_l)](http://planning.cs.uiuc.edu/img5520.gif) directly specifies the two angular wheel velocities (e.g., in radians per second). Consider how the robot moves as different actions are applied.. If ![ u_l = u_r > 0](http://planning.cs.uiuc.edu/img5521.gif), then the robot moves forward in the direction that the wheels are pointing. The speed is proportional to ![ r](http://planning.cs.uiuc.edu/img165.gif). In general, if ![ u_l = u_r](http://planning.cs.uiuc.edu/img5522.gif), then the distance traveled over a duration ![t](http://planning.cs.uiuc.edu/img1142.gif) of time is ![ r t u_l](http://planning.cs.uiuc.edu/img5523.gif) (because ![ t u_l](http://planning.cs.uiuc.edu/img5524.gif) is the total angular displacement of the wheels). If ![u_l = -u_r \not = 0](http://planning.cs.uiuc.edu/img5525.gif), then the robot rotates clockwise because the wheels are turning in opposite directions. This motivates the placement of the body-frame origin at the center of the axle between the wheels. By this assignment, no translation occurs if the wheels rotate at the same rate but in opposite directions.

Based on these observations, the configuration transition equation is

![\begin{displaymath}\begin{split}{\dot x}& = \frac{r}{2} (u_l + u_r) \cos \theta ...
...theta  {\dot \theta}& = \frac{r}{L} (u_r - u_l) . \end{split}\end{displaymath}](http://planning.cs.uiuc.edu/img5526.gif)

The translational part contains ![ \cos \theta](http://planning.cs.uiuc.edu/img1449.gif) and ![ \sin \theta](http://planning.cs.uiuc.edu/img1450.gif) parts, just like the simple car because the differential drive moves in the direction that its drive wheels are pointing. 

The translation speed depends on the average of the angular wheel velocities. To see this, consider the case in which one wheel is fixed and the other rotates. This initially causes the robot to translate at '1/2' of the speed in comparison to both wheels rotating. The rotational speed ![ {\dot \theta}](http://planning.cs.uiuc.edu/img5488.gif) is proportional to the change in angular wheel speeds. The robot's rotation rate grows linearly with the wheel radius but reduces linearly with respect to the distance between the wheels

## How to control it?

#### The unicycle aproach

Dealing with the displacement and velocities of the two wheels of a differential drive robot is messy. A preferred model is that of a unicycle, where we can think of the robot as having one wheel that can move with a desired velocity (V) at a specified heading Phi. Having the equations to translate between the unicycle model and our wheel velocities is what allows us to simplify the robot with the unicycle model. We have seen how to take measured wheel displacements to calculate the new robot pose. Now we do the reverse and calculate the desired wheel velocities from the unicycle model.

In the global coordinate frame, we can represent the velocity of the unicycle robot as:

![equation](http://faculty.salina.k-state.edu/tim/robotics_sg/_images/math/e046ca1e68836f8e625d09ea0f176463ad4c6045.png)

Then, by equaling ![xdot](http://latex.codecogs.com/gif.latex?%5Cdot%7Bx%7D) with the previous ![xdot](http://latex.codecogs.com/gif.latex?%5Cdot%7Bx%7D), ![ydot](http://latex.codecogs.com/gif.latex?%5Cdot%7By%7D) with the previous ![ydot](http://latex.codecogs.com/gif.latex?%5Cdot%7By%7D), and ![phidot](http://latex.codecogs.com/gif.latex?%5Cdot%7B%5Ctheta%7D) with the previous ![phidot](http://latex.codecogs.com/gif.latex?%5Cdot%7B%5Ctheta%7D) and resolving, we get the the next equations.

![equation](http://faculty.salina.k-state.edu/tim/robotics_sg/_images/math/2e3995d40e5d1670a32e4cbbfe33976197314804.png)

![equation](http://faculty.salina.k-state.edu/tim/robotics_sg/_images/math/bc15602adc2e28b3b4b22539d068feee0aff81ea.png)


### What did we accomplish?

We accomplished to control the robot inputs (Wheel velocities), with a much more natural approach, which is a linear velocity, and an angular velocity. Just like we used on the technical challenge.


*References:* 
* http://planning.cs.uiuc.edu/node659.html
* https://en.wikipedia.org/wiki/Differential_wheeled_robot
* http://faculty.salina.k-state.edu/tim/robotics_sg/Control/kinematics/unicycle.html


                             
                             
