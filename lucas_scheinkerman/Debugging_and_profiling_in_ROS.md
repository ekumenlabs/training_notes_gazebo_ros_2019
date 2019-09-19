# Debugging and profiling in ROS

In this file we'll se a few things about debugging and profiling in ROS.

## A common error

A common error we stumbled upon a few times was not being able to set breakpoints in `gdb` and `pdb` debuggers.

In the next example, we'll show what happens when we try to run `gdb` with the `send_goal.launch` launch file, from the package `ca_move_base`.
We added to the launch file the attribute `launch-prefix="xterm -e gdb --args"` for running `gdb` automatically with that launch file. The `xterm` part makes the gdb print on a new terminal.

When compiling initially with `catkin_make` alone, the terminal shows:

```console
GNU gdb (Ubuntu 7.11.1-0ubuntu1~16.5) 7.11.1
Copyright (C) 2016 Free Software Foundation, Inc.
License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>
This is free software: you are free to change and redistribute it.
There is NO WARRANTY, to the extent permitted by law.  Type "show copying"
and "show warranty" for details.
This GDB was configured as "x86_64-linux-gnu".
Type "show configuration" for configuration details.
For bug reporting instructions, please see:
<http://www.gnu.org/software/gdb/bugs/>.
Find the GDB manual and other documentation resources online at:
<http://www.gnu.org/software/gdb/documentation/>.
For help, type "help".
Type "apropos word" to search for commands related to "word"...
Reading symbols from /create_ws/devel/lib/ca_move_base/send_goal...(no debugging symbols found)...done
```

This text is the normal initial printing of `gdb`, except for the "no debugging symbols found" part. This is related to the fact that we didn't compile with the `DCMAKE_BUILD_TYPE=Debug` flag. What this flag does is to compile all compilable code with associated symbols which can be later used for debugging. If they're not compiled with these symbols, `gdb` can't create breakpoints because it doesn't has a reference for creating them:

```console
(gdb) b 2
No symbol table is loaded.  Use the "file" command.
```

When compiling everything again with `catkin_make -DCMAKE_BUILD_TYPE=Debug`, when trying to set breakpoints with `gdb`:

```console
GNU gdb (Ubuntu 7.11.1-0ubuntu1~16.5) 7.11.1
Copyright (C) 2016 Free Software Foundation, Inc.
License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>
This is free software: you are free to change and redistribute it.
There is NO WARRANTY, to the extent permitted by law.  Type "show copying"
and "show warranty" for details.
This GDB was configured as "x86_64-linux-gnu".
Type "show configuration" for configuration details.
For bug reporting instructions, please see:
<http://www.gnu.org/software/gdb/bugs/>.
Find the GDB manual and other documentation resources online at:
<http://www.gnu.org/software/gdb/documentation/>.
For help, type "help".
Type "apropos word" to search for commands related to "word"...
Reading symbols from /create_ws/devel/lib/ca_move_base/send_goal...done.
(gdb)
```

We can see that now it can read all the correct symbols from our `.cpp` file. Let's try setting breakpoints:

```console
(gdb) break 12
Breakpoint 1 at 0x47e5f4: file /create_ws/src/create_autonomy/navigation/ca_move_base/src/send_robot_goal.cpp, line 12.
(gdb) break 16
Breakpoint 2 at 0x47e623: file /create_ws/src/create_autonomy/navigation/ca_move_base/src/send_robot_goal.cpp, line 16.
(gdb) break 18
Breakpoint 3 at 0x47e689: file /create_ws/src/create_autonomy/navigation/ca_move_base/src/send_robot_goal.cpp, line 18.
(gdb) 
```

No problems were printed. Let's run the program:

```console
(gdb) r
Starting program: /create_ws/devel/lib/ca_move_base/send_goal 1 1 2 __name:=nav __log:=/home/create/.ros/log/ba42386a-da4a-11e9-81ae-902b346723a7/create1-nav-2.log
[Thread debugging (gdb) r
Starting program: /create_ws/devel/lib/ca_move_base/send_goal 1 1 2 __name:=nav __log:=/home/create/.ros/log/ba42386a-da4a-11e9-81ae-902b346723a7/create1-nav-2.log
[Thread debugging using libthread_db enabled]
Using host libthread_db library "/lib/x86_64-linux-gnu/libthread_db.so.1".

Breakpoint 1, main (argc=6, argv=0x7fffffffe178) at /create_ws/src/create_autonomy/navigation/ca_move_base/src/send_robot_goal.cpp:12
12      int main(int argc, char** argv){
(gdb) next
14              ros::init(argc, argv, NODE_NAME);
(gdb) next

Breakpoint 2, main (argc=4, argv=0x7fffffffe178) at /create_ws/src/create_autonomy/navigation/ca_move_base/src/send_robot_goal.cpp:16
16              MoveBaseClient ac("move_base", true);
(gdb) next
[New Thread 0x7ffff1636700 (LWP 6886)]
[New Thread 0x7ffff0e35700 (LWP 6887)]
[New Thread 0x7fffebfff700 (LWP 6888)]
[New Thread 0x7fffeb7fe700 (LWP 6893)]
[New Thread 0x7fffeaffd700 (LWP 6894)]

Thread 1 "send_goal" hit Breakpoint 3, main (argc=4, argv=0x7fffffffe178) at /create_ws/src/create_autonomy/navigation/ca_move_base/src/send_robot_goal.cpp:18
18              while(!ac.waitForServer(ros::Duration(5.0))){using libthread_db enabled]
Using host libthread_db library "/lib/x86_64-linux-gnu/libthread_db.so.1".

Breakpoint 1, main (argc=6, argv=0x7fffffffe178) at /create_ws/src/create_autonomy/navigation/ca_move_base/src/send_robot_goal.cpp:12
12      int main(int argc, char** argv){
(gdb) next
14              ros::init(argc, argv, NODE_NAME);
(gdb) next

Breakpoint 2, main (argc=4, argv=0x7fffffffe178) at /create_ws/src/create_autonomy/navigation/ca_move_base/src/send_robot_goal.cpp:16
16              MoveBaseClient ac("move_base", true);
(gdb) next
[New Thread 0x7ffff1636700 (LWP 6886)]
[New Thread 0x7ffff0e35700 (LWP 6887)]
[New Thread 0x7fffebfff700 (LWP 6888)]
[New Thread 0x7fffeb7fe700 (LWP 6893)]
[New Thread 0x7fffeaffd700 (LWP 6894)]

Thread 1 "send_goal" hit Breakpoint 3, main (argc=4, argv=0x7fffffffe178) at /create_ws/src/create_autonomy/navigation/ca_move_base/src/send_robot_goal.cpp:18
18              while(!ac.waitForServer(ros::Duration(5.0))){
```

