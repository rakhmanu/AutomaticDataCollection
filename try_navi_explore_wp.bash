#!/bin/bash

# load waypoints into parameter server
rosparam load `rospack find try_navi`/config/waypoints.yaml;

# let RoadmapServer know that wps are available in param server
rosservice call /rosplan_roadmap_server/load_waypoints;

# call turtlebot_explore_common.bash script
bash `rospack find try_navi`/scripts/try_navi.bash
