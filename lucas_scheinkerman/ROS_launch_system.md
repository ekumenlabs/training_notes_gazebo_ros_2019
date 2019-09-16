# ROS Launch system

One of this week's assignment was an investigation about the ROS launch system, which will be presented in this file.

## Architecture

### <include>

### <group>

### <machine>

## Launching nodes in remote machines



## Differences between `rosrun` and `roslaunch`

The main difference between the two is that `roslaunch` can launch more than one node at the same time, because it 
executes a launchfile which can have the commands for executing more than one node, and they can be from different packages. Opposed to this, `rosrun` only can launch a single node.

Another difference is that `roslaunch` executes the `roscore` command if there haven't been one already executed, which is responsible for running nodes and programs which are required in order to run a ROS session. `rosrun` doesn't run it, so if there wasn't an active ROS session before the `rosrun` command, nothing happens when you run it.
