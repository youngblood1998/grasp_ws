#!/bin/bash

echo "###############   START collect_img   ###############" 
{
gnome-terminal -t "collect_img" -x bash -c "rosrun grasp_pointcloud collect_img.py"
}&

echo "###############   START collect_point_cloud   ###############" 
{
gnome-terminal -t "collect_point_cloud" -x bash -c "rosrun grasp_pointcloud collect_point_cloud"
}&

echo "###############   START collect_data_move   ###############" 
{
gnome-terminal -t "collect_data_move" -x bash -c "rosrun grasp_pointcloud collect_data_move.py"
}&