rqt dot
=======

This is a simple rqt-based ROS GUI for visualizing graphviz dotcode
published over a ROS `std_msgs/String` topic.

## Usage Example

First, open the `rqt_dot` GUI:

```
rosrun rqt_dot rqt_dot
```

Second, publish some dotcode from somewhere, like the command line:

```
rostopic pub -r 1 /dotcode std_msgs/String "data: digraph foo { a; b; a -> b; }"
```

Third, subscribe to a ROS topic by entering it by name and clicking
the `Subscribe` button, and see the result:

![](doc/rqt_dot.png)

You can stop updates by clicking again on the `Subscribe` button.
