#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time : 2020.7.24
# @Author : LiShengyu
# @File : trajectory.py

import utils
import math
import numpy as np

# 传入索引，默认是从开始到末尾的全段拟合
def pos_estimator(groundtruth,estimator,index_start,index_end):
    N=index_end-index_start+1
    p1=np.mat(np.zeros((3,1)))
    p2=np.mat(np.zeros((3,1)))
    #center of mass
    for i in range(index_start,index_end+1):
        p1+=groundtruth[i].align_pos
        p2+=estimator[i].align_pos
    p1*=(1.0/N)
    p2*=(1.0/N)
    #remove the center
    q1=[]
    q2=[]
    for j in range(N):
        q1.append(groundtruth[j].align_pos-p1)
        q2.append(estimator[j].align_pos-p2)
    W=np.mat(np.zeros((3,3)))
    for k in range(N):
        W+=q1[k]*q2[k].T
    U,sigma,VT=np.linalg.svd(W)
    R=U*VT
    t=p1-R*p2
    return R,t
   


def pos_estimator_best_yaw(groundtruth,estimator,index_start,index_end):
    N=index_end-index_start+1
    p1=np.mat(np.zeros((3,1)))
    p2=np.mat(np.zeros((3,1)))
    #center of mass
    for i in range(index_start,index_end+1):
        p1+=groundtruth[i].align_pos
        p2+=estimator[i].align_pos
    p1*=(1.0/N)
    p2*=(1.0/N)
    #remove the center
    q1=[]
    q2=[]
    groundtruth_p=np.mat(np.zeros((N,3)))
    estimator_p=np.mat(np.zeros((N,3)))
    for j in range(N):
        q1.append(groundtruth[j].align_pos-p1)
        q2.append(estimator[j].align_pos-p2)
        groundtruth_p[j]=(groundtruth[j].align_pos-p1).T
        estimator_p[j]=(estimator[j].align_pos-p2).T
    #计算航向角
    best_R=estimator_p.T*groundtruth_p
    A=best_R[0,1]-best_R[1,0]
    B=best_R[0,0]+best_R[1,1]
    best_yaw=math.atan2(B,A)-math.pi/2.0
    str="best yaw for the trajectory is %f°" %(best_yaw*180.0/math.pi)
    print(str)
    R=utils.Euler2Rotation_IE(np.array([[best_yaw],
                                        [0],
                                        [0]]))
    t=p1-R*p2
    return R,t

  
  