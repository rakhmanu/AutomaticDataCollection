#!/bin/bash
rosparam load `rospack find try_navi`/config/waypoints.yaml;
echo " Loading own PDDL file is completed";
echo " Calling planner interface.";
rosservice call /rosplan_planner_interface/planning_server;

echo " Calling plan parser.";
rosservice call /rosplan_parsing_interface/parse_plan;

echo " Calling plan dispatcher.";
rosservice call /rosplan_plan_dispatcher/dispatch_plan;

echo " Finished!";
