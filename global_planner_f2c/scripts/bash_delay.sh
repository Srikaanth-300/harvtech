#!/bin/bash
sleep 20
rosrun global_planner_f2c goal_initializer.py

# This bash script waits for 20 secs (not ros time) and then runs the python script for publishing a fake goal
