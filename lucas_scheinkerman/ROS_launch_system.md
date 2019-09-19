# ROS Launch system

One of this week's assignment was an investigation about the ROS launch system, which will be presented in this file.

## Useful tags

### `<include>`

The `<include>` tag allows to import a roslaunch file into another roslaunch file. It's important to mention that all the content of the imported roslaunch file will be present in the importing file, except for the `<master>` tag. This tag will be defined by the top-level file.

### `<remap>`

The `<remap>` tag allows to pass remapping arguments to nodes inside roslaunch files. The tag applies to all subsequent declarations inside the scope in which `<remap>` was used.

### `<group>`

The `<group>` tag exists for the sole purpose of making easier applying settings to a group of nodes. It supports all the tags available for the `<launch>` command because they are equivalent to each other, with the exception that `<launch>` works as the root element of a roslaunch file (that is, the root container of a roslaunch file), and the `<group>` tag acts as a container inside the `<launch>` container.

### `<machine>`

The `<machine>` tag allows declaring a machine on which node can be executed. This allows to run ROS nodes on remote machines.

Once a machine has been declared, the way of executing nodes in that machine is the following:

```XML
<node machine="machine_name" name="node_name" pkg="node_package" type="node_type">
```

## Differences between `rosrun` and `roslaunch`

The main difference between the two is that `roslaunch` can launch more than one node at the same time, because it executes a launchfile which can have the commands for executing more than one node, and they can be from different packages. Opposed to this, `rosrun` only can launch a single node.

Another difference is that `roslaunch` executes the `roscore` command if there haven't been one already executed, which is responsible for running nodes and programs which are required in order to run a ROS session. `rosrun` doesn't run it, so if there wasn't an active ROS session before the `rosrun` command, nothing happens when you run it.

Here is an example where `rosrun` is executed without an existing master:

```console
create@galatea:/create_ws$ rosrun turtlesim turtlesim_node 
[ERROR] [1568727408.945640807]: [registerPublisher] Failed to contact master at [localhost:11311].  Retrying...