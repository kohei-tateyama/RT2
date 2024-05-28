
# RT 2nd Assignment
1. Create a node that implements an action client, allowing the user to set a target (x, y) or to cancel it. Try to use the
feedback/status of the action server to know when the target has been reached. The node also publishes the
robot position and velocity as a custom message (x,y, vel_x, vel_z), by relying on the values published on the
topic /odom;
2. Create a service node that, when called, returns the coordinates of the last target sent by the user
3. Create another service node that subscribes to the robot’s position and velocity (using the custom message) and
implements a server to retrieve the distance of the robot from the target and the robot’s average speed.


# Installing and running
## 1. Installing
In order to run this project,
```bash
git clone https://github.com/kohei-tateyama/Assignment_2_2023.git
```

## 2. Run
Start the ROS master:
```bash
roscore
```

In a separate terminal, go to the ros workspace:
```bash
cd ros_ws
```

Build the workspace using catkin:
```bash
catkin_make
```

go to the scripts directory and make all the files executable:

```bash
cd src/assignment_2_2023/scripts
chmod +x *
```

source the ROS setup:
```bash
source ../../../devel/setup.bash
```

Launch the assignment:
```bash
roslaunch rt_assignment_2 assignment_1.launch
```


The user may also run all the nodes seperately using `rosrun`.

# How to move the robot

## 1. Controls

### Set new target
In order to set the new target, simply type in two floats x,y in the terminal like below

```bash
5,6
```

### Cancel the goal
To cancel the already-set goal, type 'c' in the terminal


### Get the last target (service)
To get the last target set by the user, type in the command below.
If there are no targets that are set, it will output 0,0

```bash
rosservice call /LastTarget
```


### Get target distance (service)
To get the distance of the robot from the target, and also the average velocity of the target, type in the command below.
The output is dist, av_speed_x, av_speed_y. If the target is not set, the dist will be 100

```bash
rosservice call /GetDist
```


## 2. Pseudocode
Written below is the pseudocode of the action_client node.
```
initialize ROS node

create action client for '/reaching_goal' action server
create publisher for '/goal_topic' with message type Goal
subscribe to '/odom' for robot odometry updates
subscribe to '/reaching_goal/status' for goal status updates

while ROS is running:
    get user input for new goal position

    if user input is 'c':
        cancel the goal
        print "Goal cancelled"
    else:
        try:
            extract x, y from user input
            set new goal position
            publish goal position on '/goal_topic'
            send new goal to action server
        except ValueError:
            print "ERROR! Enter the new goal position x,y"

    wait for callbacks and handle updates

cleanup and exit

```