#!/bin/bash

set -e

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------
roscore &
sleep 5
rosrun analyzeFreq_package cra2_ex1_node.py
