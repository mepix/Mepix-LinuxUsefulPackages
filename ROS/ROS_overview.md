#ROS Introduction
ROS is an open-source software framework for Robotics Development

Provides a mean to talk to drivers and communicate between nodes.  Also has package management system
Analysis packages.

##History of ROS
- 2000 -> at Stanford
- 2007 -> WIllow Garrage
- 2013 -> Opensource Robotics Foundation

##Applications
- Drones
- Arms
- Bi-pedal Robotics

##High Level Steps
- Perception
- Decision Making
- Actuation

Each one of these is broken down into one or more ROS Nodes
There is a ROS master node that handles communication

Master node also hosts a parameter server


###Topics & Messages
Publish/Subscribe (Pub-sub) architecture where messages passed via topics (published and subscribed) between Nodes


Use predefined messages (200+) or define our own


*Physical Quantities*
- Positions
- Velocities
- Accelerations
- Durations

*Sensor Readings*
- Laser Scans
- Images
- Point Clouds
- Inertial Measurements

#### Common Messages
- [geometry_msgs](http://docs.ros.org/en/api/geometry_msgs/html/index-msg.html)

Can contain any type of data!

###Services
Request/Response Communication
- NO BUS
- NO Publishers or Subscribers

Direct interaction in a 1-to-1 basis

###Compute Graph
Shows how nodes are connected... visualized by RQT Graph

##WHY Turtles
- 1940's William Grey Walter refered to his early robots as "turtles"
- 1960's Dr. Seymour Papert (MIT) made turtle drawing turtle robots
- [Turtle Art Wikipedia](https://en.wikipedia.org/wiki/Turtle_graphics)
- [Turtle Art Gallery](http://turtleart.org/gallery/index.html)
- ROS Releases still have turtle names

To launch **turtlesim:**
1. `roscore`
1. `rosrun turtlesim turtlesim_node`
1. `rosrun turtlesim turtle_teleop_key`

## Commands
- `roscore` initializes ROS environment by starting the Master Process
- `rosnode list` prints a list of *all* active ROS nodes
- `rostopic list` prints a list of *all* active ROS topics
- `rostopic info [NAME]` details the topic, including who is publishing and who is subscribing to the topic
- `rosmsg show [PACKAGE NAME]/[MESSAGE NAME]` reveals the datatype transmitted by the message
- `rosed [PACKAGE NAME]/[MESSAGE NAME]` opens an [editor](http://wiki.ros.org/ROS/Tutorials/UsingRosEd) to get even more information about the message
- `rostopic echo [MESSAGE NAME]` prints the message in realtime in the terminal window
- `roslaunch` starts several rosnodes.
- `rosdep chceck [PACKAGE NAME]` checks for runtime dependencies
- `rosdep install -i [PACKAGE NAME]` installs dependent packages

##Publishers
`pub1 = rospy.Publisher("/topic_name", message_type, queue_size=size)`

The `"/topic_name"` indicates which topic the publisher will be publishing to. The message_type is the type of message being published on "/topic_name".

ROS publishing can be either **synchronous** or **asynchronous**:

- **Synchronous** publishing means that a publisher will attempt to publish to a topic but may be blocked if that topic is being published to by a different publisher. In this situation, the second publisher is blocked until the first publisher has serialized all messages to a buffer and the buffer has written the messages to each of the topic's subscribers. This is the default behavior of a rospy.Publisher if the queue_size parameter is not used or set to None.
- **Asynchronous** publishing means that a publisher can store messages in a queue until the messages can be sent. If the number of messages published exceeds the size of the queue, the oldest messages are dropped. The queue size can be set using the queue_size parameter.

Once the publisher has been created as above, a message with the specified data type can be published as follows: `pub1.publish(message)`

##Scripts (NODES)
````
cd ~/catkin_ws/src/simple_arm/
mkdir scripts

cd scripts
echo '#!/bin/bash' >> hello
echo 'echo Hello World' >> hello
chmod u+x hello

cd ~/catkin_ws
catkin_make
source devel/setup.bash
rosrun simple_arm hello

````

[rospy Documentation](http://docs.ros.org/kinetic/api/rospy/html/)

##Service
###Definition
- Allows request/response communication between nodes.
- Within the node providing the service, request messages are handled by functions or methods. Once the requests have been handled successfully, the node providing the service sends a message back to the requester node.
###Creation
In Python, a ROS service can be created using the following definition format:

`service = rospy.Service('service_name', serviceClassName, handler)`

- `service_name` is the name given to the service. Other nodes will use this name to specify which service they are sending requests to.
- `serviceClassName` comes from the file name where the service definition exists in an .srv file
- `handler` is the name of the function or method that handles the incoming service message. This function is called each time the service is called, and the message from the service call is passed to the handler as an argument. The handler should return an appropriate service response message.

###Using Services
1. Services can be called directly from the command line.

1. Within another node through a ServiceProxy, which provides the interface for sending messages to the service: `service_proxy = rospy.ServiceProxy('service_name', serviceClassName)`

One way the ServiceProxy can then be used to send requests is as follows:
````
msg = serviceClassNameResponse()
#update msg attributes here to have correct data
response = service_proxy(msg)
````
In the code above, a new service message is created by calling the `serviceClassNameResponse()` method. This method is provided by rospy, and its name is given by appending `Response()` to the name used for `serviceClassName`. Since the message is new, the message attributes should be updated to have the appropriate data. Next, the `service_proxy` can be called with the message, and the response stored.

For other ways to pass data to `service_proxy`, see the ROS documentation [here](http://wiki.ros.org/rospy/Overview/Services).

##ROS Subscribers
- Enables node to read messages from a topic, allowing useful data to be streamed into the node!

In Python:
````
sub1 = rospy.Subscriber("/topic_name", message_type, callback_function)

````
- `"/topic_name"` indicates which topic the Subscriber should listen to.
- `message_type` is the type of message being published on `"/topic_name"`.
- `callback_function` is the name of the function that should be called with each incoming message. Each time a message is received, it is passed as an argument to `callback_function`. Not required to return anything.

## Timing in ROS

ROS is very particular with timing and uses it's own methods to actuate the rates and callbacks.

### Behavior with C++

`ros::spin()` blocks the main thread from exiting until ROS invokes a shutdown - via a `Ctrl + C` for example. They are written as the last line of code of the main thread of the program.

```cpp
// ros::spin()
ros::init(argc, argv, "my_node");
ros::NodeHandle nh;
ros::Subscriber sub = nh.subscribe(...);
...
ros::spin();
```

`ros::spinOnce()` will allow the loop to execute once  with the desired rate.

```cpp
// ros::spinOnce()
ros::Rate r(10); // 10 hz
while (should_continue)
{
  // ... do some work, publish some messages, etc. ...
  ros::spinOnce();
  r.sleep();
}
```


### Behavior with Python


For ROS Python, `rospy.spin()` can be replaced by a while loop, so it is not mandatory to use it when working with subscribers.

`rospy.is_shutdown()` when you want to publish to a topic at a certain rate, but in that case you use `rospy.Rate.sleep()`

```py
while not rospy.is_shutdown():
  #DO SOMETHING
  rospy.Rate.sleep()
```

See [this page](https://get-help.robotigniteacademy.com/t/what-is-rospy-spin-ros-spin-ros-spinonce-and-what-are-they-for/58) for a more detailed explanation.

## ROS with Different Python Versions

ROS can work with two different versions of python as long as they are "sandboxed" within a node. For example, if the primary version of ROS uses Python2.7 (Pre-Melodic), Python 3 can also be used by installing the following dependencies:

```sh
sudo apt-get install python3-catkin-pkg-modules
sudo apt-get install python3-rospkg-modules
```

**Note:** Remember  to specify the version of python necessary to run the node in the [shebang](https://en.wikipedia.org/wiki/Shebang_(Unix)).
