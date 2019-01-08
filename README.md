# Drone-Project-FLAP
The code repository for the drone project FLAP done at Luleå University of Technology for educational purposes.

INTRODUCTION
------------
The code when run operates a tailsitter drone and interacts with a kfly chip as well as a vicon system to autonomously hover the tailsitter at a specified point.

REQUIREMENTS
------------
No special software requirements outside of the code provided, given that ROS kinetic is being used to build it.

INSTALLATION
------------
1. Move the provided folders, excluding the bagfiles folder, into your ROS workspace src folder
2. To build the system move to your ROS workspace and use the command "catkin build"

CONFIGURATION
-------------
Available configuration for the running system can be found in the services provided.

SetGains - Sets the gains for the controller hosted by the control node
SetLimits - Sets the saturation limits for the controller hosted by the control node
SetPositionReference - Sets the position reference for the controller hosted by the control node

MAINTAINERS
-----------

Current maintainers:
Kalle Löfgren - kallfg-3@student.ltu.se

CREDIT
-----------
Credit goes to Emil Fresk and Elias Small for the existing code which were modified and used in this project, specifially the kfly communication as well as the vicon communication used at the FROST lab at Luleå University of Technology.
