# Debug and profiling notes
The whole document is based on [this note](http://wiki.ros.org/roslaunch/Tutorials/Roslaunch%20Nodes%20in%20Valgrind%20or%20GDB), so you can check it first.

## Using Valgrind for profiling

[This other article](http://wiki.ros.org/roslaunch/Tutorials/Profiling%20roslaunch%20nodes) talks about profiling.
We'll use the [create_autonomy]() repo, in order to make some examples.

First, we open the docker launchfile, [see here](), then we stall `valgrind` and `kcachegrind`, we will make use of that later.
Then, we compile our project, `catkin_make DCMAKE_BUILD_TYPE=Debug`.
Then, we source devel/setup.bash `source devel/setup.bash`
Now, we will decide which node to profile, in our case `send_goal`, a node located in `ca_move_base` pkg.
In order to profile that node, we have to include a `launch-prefix` tag, into the desired `node` tag into the `launch file`.
So, to locate the launchfile `roscd ca_move_base/launch`.
We can edit the `send_goal.launch` file with whatever editor we feel like.

This is how my `send_goal.launch` looked like after the changes I made.
...

Note i added some default values to the required arguments, only to make the usage of the launchfile more straight-forward.
We can launch now the file with `roslaunch ca_move_base send_goal.launch`, we can let it run a little and then `C-c` to stop it.
Valgrind will automatically create a log located in `~/.ros/`, let's check it:
    
    cd ~/.ros/
    ls

Then we can open it with `kcachegrind`, that we installed earlier
    
    kcachegrind callgrind.sendgoal.COMPLETE
*The appended number may very with the PID associated.*

Now a window shows up, with a lot of info of the program, if you want to profundize more on the topic, you can see [here](http://valgrind.org/docs/), and learn how to interpret the data.
INSERT IMAGE

## Debugging with gdb

The process is pretty similar to the previous case, but in this one, the `launch-prefix` differs a little, let's see:

We have some options depending on how we want to launch gdb, in my case, I chose the *run your node in a gdb in a separate xterm window, manually type run to start it* way.
So, we have to change the launch prefix of `send_goal` to match with the info on the wiki page.
This is how my `send_goal.launch` file looks like:

INSERT IMAGE

We can launch now the file with `roslaunch ca_move_base send_goal.launch`, and a `xterm` window will popup.
We can now use every functionality that `gdb` offers, I recommend seeing [this](https://darkdust.net/files/GDB%20Cheat%20Sheet.pdf) for some useful info.

Let's see a quick example, first set a setpoint at line 10 of the file, `b 10`.
Then run the program with `run`.
We can see the program stopped at the checkpoint we set previously.






