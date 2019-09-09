# Notes

## Client and server separation

When you run Gazebo you are running to different executables. In one hand 'gzserver' and in the other 'gzclient'.

The 'gzserver' executable runs the physics update-loop and sensor data generation. This is the core of Gazebo and it can be used independently of a graphical interface. 

The 'gzclient' executable runs a QT based user interface. This application provides a nice visualization of simulation and convenient controls over various simulation properties.

Gazebo works running both 'gzserver' and 'gzclient', however, you can use  'gzserver' without 'gzclient' in situations where the user interface is not needed, ex: on a cloud computer.

## World files

It contains the elements of the simulation, robots, sensors, etc. This file is formatted using SDF (Simulation Description Format), and typically have a .world extension.

The Gazebo server reads this file to generate and populate a world.

Ex. 'can_population.world'

## Model files

A model file uses the same SDF format as world files, but should only contain a single <model> ... <model>('<model>' is one of the elements you can have in an SDF file, others for example are 'model' and 'actor').The purpose of these files is to facilitate model reuse and simplify world files.

'Ex. can.sdf' 

## Environment Variables

Gazebo uses several environment variables to locate files and set up communications between the server and clients. Default values that work for most cases are compiled in. Some environment variables are:

*GAZEBO_MODE_PATH
*GAZEBO_RESOURCE_PATH
*GAZEBO_MASTER_URI
*GAZEBO_PLUGIN_PATH
*GAZEBO_MODEL_DATABASE_URI

## Plugins

Plugins provide a simple and convenient mechanism to interface with Gazebo. Plugins can either be loaded on the command line, or specified in an SDF file.

Plugins specified on the command line are loaded fist, then plugins specified in the SDF files are loaded. Some plugins are loaded by the server, such as plugins that affect physics properties, while other plugins are loaded by the graphical client to facilitate custom GUI generation.

'Ex. libRestWebPlugin.so'

## Communication Between Processes

Gzserver and Gzclient communicate using the gazebo communication library.

The communication library currently uses the open-source Google Protobuf for the message serialization and boost::ASIO for the transport mechanism. It supports the publish/subscribe communication paradigm.

ROS on the other hand uses nodes to communicate in certain topics controlled by a master who have a record of all the components who participated in the communication.

## System

### Gazebo Master

This is essentially a topic name server. It provides name lookup and topic management. A single master can handle multiple physics simulations, sensor generators, and GUIs.

### Communication Library

Used by almost all subsequent libraries. Acts as the communication and transport mechanism for Gazebo.

### Physics Library

Provides a simple and generic interface to fundamental simulation components, such as rigid bodies, collision shapes, joints, etc...

### Rendering Library

Uses OGRE to provide a simple interface for rendering 3D scenes to both the GUI and sensor libraries.

### Sensor Generation

Implements all the various types of sensors, listens to the world state updates from a physics simulator and produces output specified by the instantiated sensors.

### GUI (Graphical User Interface)

The GUI library uses QT to create graphical widgets for users to interact with the simulation. The user may control the flow of time by pausing or changing time step size via GUI widgets.

### Plugins

The physics, sensor, and rendering libraries support plugins. These plugins provide users with access to the respective libraries without using the communication system.