#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time : 2020.7.24
# @Author : LiShengyu
# @File : trajectory.py
import os
import numpy as np
import utils
import math
import os
import configparser
#做了假设 
#只有三种情况
#1. pos
#2. pos vel
#3. pos vel (yaw,pitch,roll)


def load_data(result_dir,start_time,end_time,Is_use_pos,Is_use_vel,Is_use_att,pos_mode,att_mode,time_index,pos_index,vel_index,att_index,interval):
    trajs=[]
    f=open(result_dir)
    lines=f.readlines()
    for line in lines:
        if line[0]=='#':
            continue
        if interval==',':
            list=line.strip('\n').split(",")
        else:
            list=line.strip('\n').split()
        if len(list)==0:
            continue
        if utils.is_number(list[0])==False:
            continue
        #for i in range(len(list)):
            #print(list[i])
        #input()
        if Is_use_pos:
            tmp_traj=utils.traj()
            tmp_traj.time=float(list[int(time_index[0])])
            if tmp_traj.time>1e17:
                tmp_traj.time=tmp_traj.time/1e9
            if tmp_traj.time<start_time or tmp_traj.time>end_time:
                continue
            new_pos=np.array([[float(list[int(pos_index[0])])],[float(list[int(pos_index[1])])],[float(list[int(pos_index[2])])]])
            if pos_mode=='ECEF':
                if abs(new_pos[0,0])<1e4 or abs(new_pos[0,0])<1e4:#BLH
                    if abs(new_pos[0,0]>math.pi/2.0 or abs(new_pos[1,0])>math.pi):# BL是度
                        new_pos[0,0]=new_pos[0,0]*math.pi/180.0
                        new_pos[1,0]=new_pos[1,0]*math.pi/180.0
                        tmp_traj.position=utils.BLH2XYZ(new_pos)
                    else:
                        tmp_traj.position=utils.BLH2XYZ(new_pos)
                else:
                    tmp_traj.position=new_pos
            else:
                 tmp_traj.position=new_pos
            #print(tmp_traj.position)
            if Is_use_vel:
                tmp_traj.velocity=np.array([[float(list[int(vel_index[0])])],[float(list[int(vel_index[1])])],[float(list[int(vel_index[2])])]])
            if Is_use_att:
                if att_mode=='Euler':#格式为欧拉角,且单位为度
                    att=np.array([[float(list[int(att_index[0])])],[float(list[int(att_index[1])])],[float(list[int(att_index[2])])]])*math.pi/180.0
                    tmp_traj.q=utils.Rotation2Quaternion(utils.Euler2Rotation_IE(att))
                else: #格式为四元数
                    tmp_q=np.array([[float(list[int(att_index[0])])],
                                         [float(list[int(att_index[1])])],
                                         [float(list[int(att_index[2])])],
                                         [float(list[int(att_index[3])])]])  #输出为 xyzw 读入为 wxyz
                    tmp_traj.q=utils.QuaternionNormalize(tmp_q);

            trajs.append(tmp_traj)
        else:
            print("No pos infomation")

        
    return trajs

def load_iedata(result_dir,start_time,end_time,Is_use_pos,Is_use_vel,Is_use_att,pos_mode,att_mode,time_index,pos_index,vel_index,att_index,interval,fixed_diffs):
    trajs=[]
    f=open(result_dir)
    lines=f.readlines()
    for line in lines:
        if line[0]=='#':
            continue
        if interval==',':
            list=line.strip('\n').split(",")
        else:
            list=line.strip('\n').split()
        if len(list)==0:
            continue
        if utils.is_number(list[0])==False:
            continue
        #for i in range(len(list)):
            #print(list[i])
        #input()
        if Is_use_pos:
            tmp_traj=utils.traj()
            tmp_traj.time=float(list[int(time_index[0])])
            if tmp_traj.time>1e17:
                tmp_traj.time=tmp_traj.time/1e9
            if tmp_traj.time<start_time or tmp_traj.time>end_time:
                continue
            new_pos=np.array([[float(list[int(pos_index[0])])],[float(list[int(pos_index[1])])],[float(list[int(pos_index[2])])]])
            if pos_mode=='ECEF':
                if abs(new_pos[0,0])<1e4 or abs(new_pos[0,0])<1e4:#BLH
                    if abs(new_pos[0,0]>math.pi/2.0 or abs(new_pos[1,0])>math.pi):# BL是度
                        new_pos[0,0]=new_pos[0,0]*math.pi/180.0
                        new_pos[1,0]=new_pos[1,0]*math.pi/180.0
                        tmp_traj.position=utils.BLH2XYZ(new_pos)
                    else:
                        tmp_traj.position=utils.BLH2XYZ(new_pos)
                else:
                    tmp_traj.position=new_pos
            else:
                tmp_traj.position=new_pos
            tmp_traj.position=np.array([
                [new_pos[0,0] -float(fixed_diffs[0]) ],
                [new_pos[1,0]-float(fixed_diffs[1]) ],
                [new_pos[2,0]-float(fixed_diffs[2])]])
            
            if Is_use_vel:
                tmp_traj.velocity=np.array([[float(list[int(vel_index[0])])],[float(list[int(vel_index[1])])],[float(list[int(vel_index[2])])]])
            if Is_use_att:
                if att_mode=='Euler':#格式为欧拉角,且单位为度
                    att=np.array([[float(list[int(att_index[0])])],[float(list[int(att_index[1])])],[float(list[int(att_index[2])])]])*math.pi/180.0
                    tmp_traj.q=utils.Rotation2Quaternion(utils.Euler2Rotation_IE(att))
                else: #格式为四元数
                    tmp_q=np.array([[float(list[int(att_index[0])])],
                                         [float(list[int(att_index[1])])],
                                         [float(list[int(att_index[2])])],
                                         [float(list[int(att_index[3])])]])  #输出为 xyzw 读入为 wxyz
                    tmp_traj.q=utils.QuaternionNormalize(tmp_q);

            trajs.append(tmp_traj)
        else:
            print("No pos infomation")

        
    return trajs


def read_uzh_result(traj_dir):
    trajs=[]
    f=open(traj_dir)
    lines=f.readlines()
    for line in lines:
        #line_size=len(line)
        list=line.strip('\n').split(' ')
        tmp_traj=utils.traj()
        tmp_traj.time=float(list[0])
        tmp_traj.position=np.array([[float(list[1])],[float(list[2])],[float(list[3])]])
        #tmp_traj.velocity=np.array([[float(list[4])],[float(list[5])],[float(list[6])]])
        tmp_traj.q=np.array([[float(list[7])],[float(list[4])],[float(list[5])],[float(list[6])]])
        trajs.append(tmp_traj)
    return trajs


#暂时读publish输出的 以验证程序的正确性
def read_gnut_result(traj_dir):
    trajs=[]
    f=open(traj_dir)
    lines=f.readlines()
    for line in lines:
        list=line.strip('\n').split()
        tmp_traj=utils.traj()
        tmp_traj.time=float(list[0])
        tmp_traj.position=np.array([[float(list[1])],[float(list[2])],[float(list[3])]])
        tmp_traj.velocity=np.array([[float(list[4])],[float(list[5])],[float(list[6])]])
        #tmp_traj.q=np.array([[float(list[13])],[float(list[10])],[float(list[11])],[float(list[12])]])  #输出为 xyzw 读入为 wxyz
        tmp_traj.yaw=float(list[7])*math.pi/180.0
        tmp_traj.pitch=float(list[8])*math.pi/180.0
        tmp_traj.roll=float(list[9])*math.pi/180.0
        tmp_traj.q=utils.Rotation2Quaternion(utils.Euler2Rotation_IE(np.mat(np.array([[tmp_traj.yaw],[tmp_traj.pitch],[tmp_traj.roll]]))))
        trajs.append(tmp_traj)
        
    return trajs
    
#暂时定义用于读取IE数据
def read_IE_result(traj_dir):
    trajs=[]
    f=open(traj_dir)
    lines=f.readlines()
    for line in lines:
        if line[0]!='2':
            continue
        list=line.strip('\n').split()
        tmp_traj=utils.traj()
        tmp_traj.time=float(list[1])
        tmp_traj.position=np.array([[float(list[2])],[float(list[3])],[float(list[4])]])
        
        tmp_traj.velocity=np.array([[float(list[12])],[float(list[13])],[float(list[14])]])
        att=np.array([[float(list[21])],[float(list[22])],[float(list[23])]])*math.pi/180.0
        tmp_traj.q=utils.Rotation2Quaternion(utils.Euler2Rotation_IE(att))  #输出为 xyzw 读入为 wxyz
        trajs.append(tmp_traj)

        
    return trajs


def get_config_values(section, option):
    """
    根据传入的section获取对应的value
    :param section: ini配置文件中用[]标识的内容
    :return:
    """
    config = configparser.ConfigParser()
    config.read(configFilePath)
    # return config.items(section=section)
    return config.get(section=section, option=option)