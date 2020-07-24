#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time : 2020.7.24
# @Author : LiShengyu
# @File : trajectory.py
import numpy as np
import math
import re

#所有的算法均基于np
#四元数顺序w x y z!!!

class traj:
    #def __init__(self,time,position,velocity,yaw,pitch,roll):
    def __init__(self):
        self.time=0.0
        self.position=np.mat(np.zeros((3,1)))
        self.velocity=np.mat(np.zeros((3,1)))
        self.q=np.mat(np.zeros((4,1)))
        #姿态单位需要重新确认一下
        self.yaw=0.0
        self.pitch=0.0
        self.roll=0.0

        #getmatching 之后才会生成的变量
        self.distance=0.0
        self.align_pos=np.mat(np.zeros((3,1)))
        self.align_velocity=np.mat(np.zeros((3,1)))
        self.align_q=np.mat(np.zeros((4,1)))

#记录相对误差的，里面是列表，一个位置上的计算相对误差得到一个rel_error
#之后在这里补充注释！！！！！！！！！！！！！
class rel_error:
    def __init__(self):
        self.time=[]
        self.distance=[]
        self.rel_tran=[]
        self.rel_tran_percentage=[]
        self.yaw=[]
        self.pitch=[]
        self.roll=[]
        self.angle=[]

#计算全局误差时使用，得到全段拟合后的各项误差
#类型为list
class abs_error:
    def __init__(self):
        self.time=[]
        self.distance=[]
        self.dpos=[]
        self.dvel=[]
        self.datt=[]
        self.sqrt_pos=[]
        self.sqrt_att=[]

#差分轨迹，用于画轨迹图
class diff_traj:
    def __init__(self):
        self.time=[]
        self.distance=[]
        self.ground_pos=[]
        self.align_estimator_pos=[]

        

class chart_info:
    def __init__(self):
        self.Length=0.0     #总里程
        self.t_rel=0.0      # %
        self.t_abs=0.0      # m
        self.r_rel=0.0      # deg/m
       


def skewSymmetric(v):
     return np.array([[0, -v[2], v[1]],
                        [v[2], 0, -v[0]],
                        [-v[1], v[0], 0]], dtype=numpy.float64)


def XYZ2BLH(XYZ):
    a1=6378137
    e2=0.00669437999013
    L=math.atan(XYZ[1,0]/XYZ[0,0])
    if XYZ[0,0]<0 and XYZ[1,0]>0:
        L+=math.pi
    if XYZ[0,0]<0 and XYZ[1,0]<0:
        L-=math.pi
    B1=math.atan(XYZ[2,0]/math.sqrt(XYZ[0,0]*XYZ[0,0]+XYZ[1,0]*XYZ[1,0]))
    N=a1/math.sqrt(1-e2*math.sin(B1)*math.sin(B1))
    B2=math.atan((XYZ[2,0] + N * e2*math.sin(B1)) / math.sqrt(XYZ[0,0]*XYZ[0,0] + XYZ[1,0]*XYZ[1,0]))
    #迭代法求大地纬度
    while abs(B1-B2)>(0.001/3600.*math.pi/180.0):
        B1=B2
        N=a1/math.sqrt(1-e2*math.sin(B1)*math.sin(B1))
        B2=math.atan((XYZ[2,0] + N * e2*math.sin(B1)) / math.sqrt(XYZ[0,0]*XYZ[0,0] + XYZ[1,0]*XYZ[1,0]))
    B=B2
    H=XYZ[2,0]/math.sin(B2)-N*(1-e2)
    return np.array([[B],[L],[H]])

def BLH2XYZ(BLH):
    a1=6378137
    e2=0.00669437999013
    B=BLH[0]
    L=BLH[1]
    H=BLH[2]
    N=a1/math.sqrt(sqrt-e2*math.sin(B)*math.sin(B))
    X=(N+H)*math.cos(B)*math.cos(L)
    Y=(N+H)*math.cos(B)*math.sin(L)
    Z=(N*(1-e2)+H)*math.sin(B)
    return np.array([[X],[Y],[Z]])


#传入的是BLH ECEF到ENU的转换矩阵
def R_ECEF_ENU(BLH):
    sinB=math.sin(BLH[0,0])
    cosB=math.cos(BLH[0,0])
    sinL=math.sin(BLH[1,0])
    cosL=math.cos(BLH[1,0])
    return np.array([[-sinL,cosL,0],
                     [-cosL*sinB,-sinL*sinB,cosB],
                     [cosL*cosB,sinL*cosB,sinB]])

#对T赋值R,t,1
def linear_translation(trans,R,t):
    trans[0,0]=R[0,0]
    trans[0,1]=R[0,1]
    trans[0,2]=R[0,2]

    trans[1,0]=R[1,0]
    trans[1,1]=R[1,1]
    trans[1,2]=R[1,2]
    
    trans[2,0]=R[2,0]
    trans[2,1]=R[2,1]
    trans[2,2]=R[2,2]
                  
    trans[0,3]=t[0,0]
    trans[1,3]=t[1,0]
    trans[2,3]=t[2,0]
    trans[3,3]=1
    return trans

#对T赋值R
def linear(trans,R):
    trans[0,0]=R[0,0]
    trans[0,1]=R[0,1]
    trans[0,2]=R[0,2]

    trans[1,0]=R[1,0]
    trans[1,1]=R[1,1]
    trans[1,2]=R[1,2]
    
    trans[2,0]=R[2,0]
    trans[2,1]=R[2,1]
    trans[2,2]=R[2,2]

    trans[3,3]=1
    return trans

#得到R
def get_linear(trans):
    return np.array([[trans[0,0],trans[0,1],trans[0,2]],
                     [trans[1,0],trans[1,1],trans[1,2]],
                     [trans[2,0],trans[2,1],trans[2,2]]])

#得到t
def get_translation(trans):
    return np.array([[trans[0,3]],
                     [trans[1,3]],
                     [trans[2,3]]])

def QuaternionNormalize(q):
    return q/np.linalg.norm(q)

def sign(R):

    if R>0:
       return 1
    elif R<0:
        return -1
    else:
        return 0
   
#四元数顺序是w x y z!!!!
def Quaternion2Rotation(q):
    qq=QuaternionNormalize(q)
    R=np.mat(np.zeros((3,3)))
    q0=qq[0,0]
    q1=qq[1,0]
    q2=qq[2,0]
    q3=qq[3,0]
    R[0,0]=q0*q0+q1*q1-q2*q2-q3*q3
    R[0,1]=2*(q1*q2-q0*q3)
    R[0,2]=2*(q1*q3+q0*q2)
       
    R[1,0]=2*(q1*q2+q0*q3)
    R[1,1]=q0*q0-q1*q1+q2*q2-q3*q3
    R[1,2]=2*(q2*q3-q0*q1)
       
    R[2,0]=2*(q1*q3-q0*q2)
    R[2,1]=2*(q2*q3+q0*q1)
    R[2,2]=q0*q0-q1*q1-q2*q2+q3*q3
    return R

def Rotation2Quaternion(R):
    q=np.mat(np.zeros((4,1)))
    q[0,0]=math.sqrt(abs(1.0+R[0,0]+R[1,1]+R[2,2]))/2.0
    q[1,0]=sign(R[2,1]-R[1,2])*math.sqrt(abs(1.0+R[0,0]-R[1,1]-R[2,2]))/2.0
    q[2,0]=sign(R[0,2]-R[2,0])*math.sqrt(abs(1.0-R[0,0]+R[1,1]-R[2,2]))/2.0
    q[3,0]=sign(R[1,0]-R[0,1])*math.sqrt(abs(1.0-R[0,0]-R[1,1]+R[2,2]))/2.0
    return q


#uzh开源数据评定工具所使用的旋转矩阵转欧拉角
#结果的单位为弧度，顺序为ypr
def Rotation2Euler_uzh(R):
    i=0
    j=1
    k=2
    cy=math.sqrt(R[i,i]*R[i,i]+R[j,i]*R[j,i])
    ax=math.atan2(R[k,j],R[k,k])
    ay=math.atan2(-R[k,i],cy)
    az=math.atan2(R[j,i],R[i,i])
    tmp=ax
    ax=az
    az=tmp
    return np.array([[ax],
                     [ay],
                     [az]])

#欧拉角，单位为度，修正大于300度的角
def correct_ypr(t):
    if t[0]>=300:
        t[0]-=360
    if t[0]<=-300:
        t[0]+=360
    if t[1]>=300:
        t[1]-=360
    if t[1]<=-300:
        t[1]+=360
    if t[2]>=300:
        t[2]-=360
    if t[2]<=-300:
        t[2]+=360
    return t

#仅仅在求adjust_yaw时使用
def Euler2Rotation_IE(R):
  alpha=R[0]
  beta=R[2]
  gama=R[1]
  ans=np.mat(np.zeros((3,3)))
  ans[0,0]=math.cos(alpha)*math.cos(beta) - math.sin(alpha)*math.sin(beta)*math.sin(gama)
  ans[0,1]=math.sin(alpha)*math.cos(beta) + math.cos(alpha)*math.sin(beta)*math.sin(gama)
  ans[0,2]=-math.sin(beta)*math.cos(gama)
  ans[1,0]=-math.sin(alpha)*math.cos(gama)
  ans[1,1]=math.cos(alpha)*math.cos(gama)
  ans[1,2]=math.sin(gama)
  ans[2,0]=math.cos(alpha)*math.sin(beta) + math.sin(alpha)*math.cos(beta)*math.sin(gama)
  ans[2,1]=math.sin(alpha)*math.sin(beta) - math.cos(alpha)*math.cos(beta)*math.sin(gama)
  ans[2,2]=math.cos(beta)*math.cos(gama)
  return ans
    
def Rotation2Euler_IE(R):
    ay=math.asin(R[1,2])
    ax=math.atan2(-R[1,0]/math.cos(ay),R[1,1]/math.cos(ay))
    az=math.atan2(-R[0,2],R[2,2])
    

    return np.array([[ax],
                     [ay],
                     [az]])



def is_number(num):
  pattern = re.compile(r'^[-+]?[-0-9]\d*\.\d*|[-+]?\.?[0-9]\d*$')
  result = pattern.match(num)
  if result:
    return True
  else:
    return False