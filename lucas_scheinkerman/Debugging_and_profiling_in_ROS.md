# Debugging and profiling in ROS

In this file we'll se a few issues about debugging and profiling in ROS with different tools.

## Error when trying to set breakpoints to ROS launch files with `gdb`

A common error we stumbled upon a few times was not being able to set breakpoints in the `gdb` debugger.

In the next example, we'll show what happens when we try to run `gdb` with the `send_goal.launch` launch file, from the package `ca_move_base`.
We added to the launch file the attribute `launch-prefix="xterm -e gdb --args"` for running `gdb` automatically with that launch file. The `xterm` part makes the gdb print on a new terminal.

When compiling initially with `catkin_make` alone, the terminal shows:

```console
GNU gdb (Ubuntu 7.11.1-0ubuntu1~16.5) 7.11.1
Copyright (C) 2016 Free Software Foundation, Inc.
License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>
(...)
Reading symbols from /create_ws/devel/lib/ca_move_base/send_goal...(no debugging symbols found)...done
```

This text is the normal initial printing of `gdb`, except for the "no debugging symbols found" part. This is related to the fact that we didn't compile with the `DCMAKE_BUILD_TYPE=Debug` flag. **What this flag does is to compile all compilable code with associated symbols which can be later used for debugging. If they're not compiled with these symbols, `gdb` can't create breakpoints because it doesn't has a reference for creating them**:

```console
(gdb) b 2
No symbol table is loaded.  Use the "file" command.
```

When compiling everything again with `catkin_make -DCMAKE_BUILD_TYPE=Debug`, when trying to set breakpoints with `gdb`:

```console
GNU gdb (Ubuntu 7.11.1-0ubuntu1~16.5) 7.11.1
Copyright (C) 2016 Free Software Foundation, Inc.
License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>
(...)
Reading symbols from /create_ws/devel/lib/ca_move_base/send_goal...done.
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

It can be seen that the `.cpp` program was executed without any issues regarding to the use of the `gdb` program.

## Using Valgrind with ROS

Another well-known tool is `Valgrind`, which is used mostly for memory leaking detection and profiling. Let's see how to use it with ROS launch files.

By adding the attribute `launch-prefix="valgrind"` to a node, we will be ordering to run the node together with valgrind. [Additional parameters](<http://valgrind.org/docs/manual/manual-core.html#manual-core.basicopts>) can be provided between the quotes, for managing the amount and type of information that the tool will show.

As a brief example, we added the `Valgrind` attribute to the `calibrate.launch` file in the 'navigation' package:

```XML
<launch>
    <!-- Remap the raspicam frames -->
    <include file="$(find ca_visual_odometry)/launch/cam_mapping.launch"/>

    (...)

    <node pkg="camera_calibration" type="cameracalibrator.py" name="$(arg cam_name)_calibration" output="screen"
        args="--size $(arg chessboard_x)x$(arg chessboard_y) --square $(arg square_size_m) image:=$(arg cam_topic)" launch-prefix="valgrind"/>
</launch>

```

When running this launch file, there was a lot of output. Just as an example, here is the summay of it that appears at the end of the output:

```console
==6865== HEAP SUMMARY:
==6865==     in use at exit: 6,475,124 bytes in 14,187 blocks
==6865==   total heap usage: 121,291 allocs, 107,104 frees, 4,348,015,376 bytes allocated
==6865==
==6865== LEAK SUMMARY:
==6865==    definitely lost: 1,958 bytes in 21 blocks
==6865==    indirectly lost: 296 bytes in 5 blocks
==6865==      possibly lost: 492,966 bytes in 226 blocks
==6865==    still reachable: 5,979,904 bytes in 13,935 blocks
==6865==                       of which reachable via heuristic:
==6865==                         newarray           : 1,536 bytes in 16 blocks
==6865==         suppressed: 0 bytes in 0 blocks
==6865== Rerun with --leak-check=full to see details of leaked memory
==6865== 
==6865== For counts of detected and suppressed errors, rerun with: -v
==6865== Use --track-origins=yes to see where uninitialised values come from
==6865== ERROR SUMMARY: 5494 errors from 136 contexts (suppressed: 0 from 0)
``

For more information about how to interpret the output of this tool, [see this](<http://valgrind.org/docs/manual/quick-start.html#quick-start.interpret>) and [page 7 of this]<https://aleksander.es/data/valgrind-memcheck.pdf>.
