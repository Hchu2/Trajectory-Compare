#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time : 2020.7.24
# @Author : LiShengyu
# @File : trajectory.py
import trajectory
import os
import configparser
import sys

#trajs=trajectory.traj_align('C:\\Users\\one\\source\\repos\\PythonApplication2\\PythonApplication2\\stamped_groundtruth.txt',
#                            'C:\\Users\\one\\source\\repos\\PythonApplication2\\PythonApplication2\\stamped_traj_estimate.txt',
#                            'C:\\Users\\one\\source\\repos\\PythonApplication2\\PythonApplication2\\','Normal','Normal','Adjust_Yaw',True,True,True)

#trajs=trajectory.traj_align('C:\\Users\\one\\source\\repos\\PythonApplication2\\PythonApplication2\\LC_combined_smoothed.txt',
#                            'C:\\Users\\one\\source\\repos\\PythonApplication2\\PythonApplication2\\data0715_03_uzhrpg.txt',
#                           'C:\\Users\\one\\source\\repos\\PythonApplication2\\PythonApplication2\\','XYZ','XYZ','Adjust_Yaw',True,True,True)

#cd C:\Users\one\source\repos\PythonApplication2\PythonApplication2
#python main.py C:\\Users\\one\\source\\repos\\PythonApplication2\\PythonApplication2\\example2_config.ini

def main():
    #trajs=trajectory.traj_align('C:\\Users\\one\\source\\repos\\PythonApplication2\\PythonApplication2\\example_config.ini')
    trajs=trajectory.traj_align(sys.argv[1])

if __name__=="__main__":
    main()