# MIE443 Contest 2: Finding Objects of Interest in an Environment - Group 18
## Group Members
**Farhan Wadia - 1003012606**

**Henry Cueva Barnuevo - 1003585122**

**Yilin Huang - 1003145232**

## Execution Commands
1. Place this repository in the `catkin_ws/src` folder of the file system.

2. Launch the simulated world in Gazebo by entering the below command in a terminal window:
```bash
roslaunch mie443_contest2 turtlebot_world.launch world:=practice
```
3. Run the AMCL and obstacle avoidance algorithm with the path to the map. An example for `map_practice.yaml` is given below:
```bash
roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/turtlebot/catkin_ws/src/mie443_contest2/maps/map_practice.yaml
```
4. Launch RVIZ to visualize the map in a separate terminal window. Align the 2D Pose Estimate in RVIZ after it opens:
```bash
roslaunch turtlebot_rviz_launchers view_navigation.launch
```
5. Run the contest2.cpp file by entering the below command in a separate terminal window:
```bash
rosrun mie443_contest2 contest2
```
6. At the end of the contest, open the file called `Group 18 Tags - YYYY-MM-DD HH-MM-SS.csv` that was created by the contest code. The file will be located in the `/catkin_ws/src/mie443_contest2/` directory. 
The file can be opened using gedit, or imported as a spreadsheet using LibreOffice Calc. Rows are appended to the file after discovering each box rather than all at the end. An example of what the file would look like after discovering the first box in the practice environment is given below: 
```bash
"Order Visited", "Box ID", "Tag File", "Discovered as Duplicate", "Location (x, y, angle)"
"1", "5", "tag_14.jpg", "False", "(0.183132, 0.590897, -3.03299)"
```
