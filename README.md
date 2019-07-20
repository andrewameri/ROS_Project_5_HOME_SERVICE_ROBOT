# Home Service Robot
This is the final project of Udacity Robotics Software Engineer Nano Degree. The project goal is to enable a turtle bot to autonomously navigate through a custom designed world while carrying an object between two coordinates. Various ROS packages have been used to accomplish this task. The hierarchy of the project is as follows:
- src
  - turtlebot_simulator (online package)
  - turtlebot_interactions (online package)
  - turtlebot_apps (online package)
  - turtlebot (online package)
  - turtlebot_msgs (online package)
  - slam_gmapping (online package)
  - pgm_map_creator (online package)
  - pick_objects (Custom package)
  - add_markers (Custom package)
  - home_service_robot (Custom package)
  - scripts (Custom scripts)
  - CMakeList

### Purpose of packages 
- turtlebot_simulator: We use turtlebot_gazebo meta package launch files in different stages of this project. Specifically, "amcl_demo" launch file is used for AMCL localization; "gmapping_demo" launch file is used for mapping the custom world; and "turtlebot_world" is used for laoding the turtle bot with necessary sensors in the custom world. 
- turtlebot_interactions: We use turtlebot_rviz_launchers meta package launch files for importing sufficient information into rviz. We call "view_navigation" launch everytime for different stages of this project whenever we need visualization. 
- turtlebot_apps: We use turtlebot_navigation meta package launch files for amcl_demo. Moreover, the 3D scanner sensors .launch.xml files will be used from "includes" directory of this package. 
- turtlebot: We use turtlebot_teleop meta package launch file "keyboard_teleop" for manually controlling the robot. 
- turtlebot_msgs: This package is installed as dependency of other packages 
- slam_gmapping: We use this package for mapping purposes. 
- pgm_map_creator: The map that is generated using this package is not used in the final solution. However, this package is included for the sake of learning and practicing.
- pick_objects: "pick_objects_node" node is written in this package for picking up and dropping off a fake object. A client is defined to send goal requests to the move_base server through a SimpleActionClient. We will be sending two separate coordinates through this method; first coordinate is for pickup location and the second coordinate is for dropoff location. Moreover, a new topic "/robot_status" is also defined in this node to publish the current status of the robot. The status can be either START, PICKUP, DROPOFF, or ERROR. From the point the simulation starts till the moment that robot arrives at pickup location the robot status is START. As soon as it reaches to pickup location the robot status is PICKUP; After arrving at dropoff location the robot status is DROPOFF; and if any difficulty happens during the process that stop the robot, the status would be ERROR. These messages will be transmitted over /robot_status topic to add_markers node which will be explained next.
- add_markers: This package is written to simulate the appearance and disappearance of the object while robot is navigating in the world. The node in this package publishes Marker messages to map. As the result we will see a purple cube at pickup and dropoff location. This process is tied with processing messages that it receives by being a subscriber to /robot_status. 
- home_service_robot: This package has the following items as the final manifastation of this project:
  - "world" directory: which has the custom world file for running this project in.
  - "rviz" directory: which has the rviz configuration file. It's worth to mention that for almost all of the scripts, rviz file is being pulled from turtlebot_interactions package and not this folder.
  - "model" directory: which has the model of the building that we use in our world file. 
  - "map" directory: which has two sets of .pgm and .yaml files. The main pair of files that is used in home_service script is AndrewOffice2 which is the generated map from test_slam script. However, for the sake of practicing I also included the artificial generated map from pgm_map_creator. 
  - "launch" directory: a launch file is created for calling rviz. This launch file is redundant in this solution, because I use the rviz launch file from "turtlebot_rviz_launchers" package. 
  -  scripts: There are five scripts in this fodler.
      - test_navigation.sh: which is responsible for loading the turtlebot in the custom world, running the amcl localication, and running rviz for visualization.
      - test_slam.sh: which is responsible for loading the turtlebot in the custom world, running the gmapping algorithm for mapping the custom world, running the rviz for visualization and running the teleop node for manually controlling the robot. 
      - pick_objects.sh: which is responsible for loading the turtlebot in the custom world, running the amcl localization algorithm, running the rviz for visualization, and running the pick_objects node for simulating the pickup/dropoff actions.
      - add_marker.sh: which is responsible for loading the turtlebot in the custom world, running the amcl localization algorithm, running the rviz for visualization, and showing a purple box as a fake object that the robot is supposedly carrying. 
      - home_service.sh: which is responsible for loading the turtlebot in the custom world, running the amcl localization algorithm, running the rviz for visualization, and running the pick_objects node for simulating the pickup/dropoff actions, and showing a purple box as a fake object that the robot is supposedly carrying.

