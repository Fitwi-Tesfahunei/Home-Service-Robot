# Home-Service-Robot
This robot moves towards the pickup goal and picks up a virtual object. Once it picks the virtual object, it waits for 5 seconds and moves towards the drop-off goal. Once it reaches the drop-off position, it drops the virtual object.


In this project, there are several packages (with various nodes) working in tandem. They are launched with the help of home_service.sh executable file. It launches various nodes within the turtlebot_gazebo package, pick_objects package, add_virtual_objects package, and turtlebot_rviz_launchers package. The turtlebot_world.launch file found within the turtlebot_gazebo package launches the UdacityOffice.world file in gazebo along with the turtlebot robot. It reads the world file from the map_and_world package. The view_navigation.launch file found within the turtlebot_rviz_launchers package launches rviz and sets up the map, robot, camera, and laser scans within rviz.

The amcl_demo.launch file found within the turtlebot_gazebo package launches the map server node, Adaptive Monte Carlo Localization (AMCL) node, and move base node. The map server node provides map data as a ROS service. It takes the map data from the map_and_world package. The AMCL node takes in odometry and laser scan data to do  AMCL  localization. And finally, the move base node utilizes a cost map where each part of the map is divided into which area is occupied and which area is unoccupied. As the robot moves around, a local cost map, in relation to the global cost map, keeps getting updated allowing the package a continuous path for the robot to move along.

The pick_objects.launch file found within the pick_objects package launches the pick_objects node that informs the robot to move from point A to point B. The frame ID is the same map used in the  AMCL  launch file. It sends successive goals for the robot to reach. It creates a path for the robot based on Dijkstra's algorithm (a variant of the Uniform Cost Search algorithm) while avoiding obstacles on its path.

The add_virtual_objects.launch file found within the add_virtual_objects package launches the add_virtual_objects node that adds markers of a cubic shape to the map. The frame ID is the same map used in the  AMCL  launch file. It subscribes to the odometry of the robot and based on the robot's pose, adds or removes markers.


YouTube: https://youtu.be/gIrM1uhsVy8
