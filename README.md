# Welcome to ros_bt_py!

This is a [Behavior
Tree](https://en.wikipedia.org/wiki/Behavior_tree_(artificial_intelligence,_robotics_and_control))
library meant to be an alternative to
[SMACH](http://wiki.ros.org/smach),
[FlexBE](http://wiki.ros.org/flexbe) and the like.

It includes a ReactJS-based web GUI and all the building blocks you
need to build moderately advanced mission control Behavior Trees
without writing a single line of code!

Its claim to fame is the ability to *shove*, or transparently
remote-execute, parts of a Behavior Tree.

## Documentation

The [main documentation effort nowadays is found in the repositories wiki](https://ids-git.fzi.de/ros/ros_bt_py/-/wikis/home).

All of this (and more) is explained in the Master Thesis "Distributed
Execution of Behavior Trees using Heterogeneous Robot Teams" by Nils Berg (link
coming as soon as I'm certain where I can post the PDF).

There is also a growing amount of Sphinx documentation in the `doc`
folder - simply execute the following commands in your shell to get
browsable HTML documentation, including some tutorials:

```bash
$ cd ros_bt_py/doc
$ make html
$ cd build
$ python -m SimpleHTTPServer & xdg-open http://localhost:8000/html
```

## Installation

To actually start using ros_bt_py, you need to install its dependencies first:

```bash
$ cd catkin_workspace
$ rosdep install --from-paths src --ignore-src -r -y
```

**Warning**
If you get the following error:
```bash
ERROR: the following packages/stacks could not have their rosdep keys resolved to system dependencies:
ros_bt_py: Cannot locate rosdep definition for [web_server]
```
You probably need to add the [web_server](https://ids-git.fzi.de/ros/web_server) package to your workspace as well:
```bash
git clone https://ids-git.fzi.de/ros/web_server
```

**Warning**
rosapi <=0.11.9 has issues with service calls with non empty requests on python3 (ROS >= noetic).
The following error in the terminal window where your started ros_bt_py is a indication of this issue:
` Error processing request: field services must be a list or tuple type`
As of August 2020, mitigating this means using the latest git version of the rosbridge_suite, of which rosapi is a part of.
```bash
git clone https://github.com/RobotWebTools/rosbridge_suite.git
```

**Warning**
If `pip install tornado` returns anything higher than tornado-5.1.1 you need to uninstall the conflicting tornado package,
otherwise you might run into a nasty bug due to rosbridge silently closing websocket connections, making the web editor unusable.
On Ubuntu 16.04 do this by (assuming it is installed in /usr/local and need to be removed with sudo):

```bash
sudo apt remove ros-kinetic-rosbridge-suite
sudo -i
pip uninstall tornado
exit
sudo apt install ros-kinetic-rosbridge-suite
```

After installing the dependencies, simply run `catkin_make` and you're
good to go! The command

```bash
$ roslaunch ros_bt_py ros_bt_py.launch
```

will start a BT server and the rosbridge and webserver needed for the
GUI. Afterwards, you can open
http://localhost:8085/ros_bt_py/editor.html to use the editor.
