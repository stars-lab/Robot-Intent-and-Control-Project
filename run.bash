#!/bin/bash
rosrun turtlebot_code odom_server.py
rosrun turtlebot_code cmd_queue.py
rosrun turtlebot_code mover.py
rosrun turtlebot_code text_parser.py