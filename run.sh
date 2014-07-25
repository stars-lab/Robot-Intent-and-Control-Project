#!/bin/bash
python nodes/odom_server.py &
python nodes/cmd_queue.py &
python nodes/mover.py &
python nodes/text_parser.py &
