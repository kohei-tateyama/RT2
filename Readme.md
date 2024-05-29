
# RT2 Assignments
Kohei Tateyama (S6427214)
### Assignment 1
Comment the second assignment of RT1 using doxygen or sphinx. The html files are uploaded in docs folder. The comments can be seen in the python code in scripts folder.

### Assignment 2
1. Create an interface to assign (or cancel) goals to the robot using jupyter notebook
2. Create a plot with the robot's position and targets' positions in the environment
3. Create a plot for the number of reached/not-reached targets

Explanation on how to run the code below. Jupyter notebook uploaded in scripts folder.

### Assignment 3
Statistical analysis of the first assigment of RT1. Assignment uploaded with the name RT_Assignment3.pdf




# Installing and running
## 1. Installing
In order to run this project, in the ros workspace, under the src directory, run
```bash
git clone https://github.com/kohei-tateyama/assignment_2_2023.git
```

## 2. Run
Start the ROS master:
```bash
roscore
```

Build the workspace using catkin:
```bash
catkin_make
```

go to the scripts directory and make all the files executable:
```bash
cd src/scripts
chmod +x *
```

Launch the assignment:
```bash
roslaunch assignment_2_2023 assignment_1.launch
```

Open up jupyter notebook:
```bash
jupyter notebook --allow-root --ip 0.0.0.0
```


# How to move the robot
In the jupyter notebook, there are widgets to set the x and y values of the robot. Set these two values and press the button "send goal position". To cancel the goal, simply press the cancel button.

# Plots
The plots should be shown in the very bottom of the code. The left plot shows the goal position as "x" and the drone position as "o". Once the goal is set, and the drone is moving, the drone position should be live plotted, and should show the trajectory of the drone.

The plot on the right shows how many goals are reached/ not reached in a bar graph. This graph is also live plotted, so it should be updated everytime a new goal is set, and everytime the goal is reached.
