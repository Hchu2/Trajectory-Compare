#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time : 2020.7.24
# @Author : LiShengyu
# @File : trajectory.py
import numpy as np
import matplotlib.pyplot as plt
import utils

def plot_traj(plot_title,result_dir,diff_traj,ground_coordinatemode,estimator_coordinatemode):
    font1 = {'family' : 'Times New Roman','weight' : 'normal','size' : 15,}
    if ground_coordinatemode=='ECEF' or estimator_coordinatemode=='ECEF':
        plt.xlabel('East [m]')
        plt.ylabel('North [m]')
    else:
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
    number=0
    gt_x=[]
    gt_y=[]
    es_x=[]
    es_y=[]
    for i,j in zip(diff_traj.ground_pos,diff_traj.align_estimator_pos):
        number+=1
        A=[i[0,0],j[0,0]]
        B=[i[1,0],j[1,0]]
        gt_x.append(i[0,0])
        gt_y.append(i[1,0])
        es_x.append(j[0,0])
        es_y.append(j[1,0])
        if number%10==1:
            plt.plot(A,B,'gray',linewidth=0.8,linestyle='--')
    plt.plot(gt_x,gt_y,'darkgreen',linewidth=2.5,label='Groundtruth')
    plt.plot(es_x,es_y,'lightseagreen',linewidth=1,label='Estimator')

    #指定图例位置，1右上角，2左上角，3右下角，4左下角，0自动适应图像
    plt.legend(loc=1,ncol=1)

    #plt.title('Error in DataSet08',font1)
    plt.grid(linestyle='--')
    plt.tight_layout()
    #plt.show()
    str=result_dir+'traj.pdf'
    #print(str)
    plt.savefig(str)


def plot_abs_error(plot_title,xlabel_kind,result_dir,abs_error,ground_coordinatemode,estimator_coordinatemode):
    dx=[]
    dy=[]
    dz=[]
    dvel_x=[]
    dvel_y=[]
    dvel_z=[]
    dyaw=[]
    dpitch=[]
    droll=[]
    for i,j,k in zip(abs_error.dpos,abs_error.dvel,abs_error.datt):
        dx.append(i[0,0])
        dy.append(i[1,0])
        dz.append(i[2,0])
        dvel_x.append(j[0,0])
        dvel_y.append(j[1,0])
        dvel_z.append(j[2,0])
        dyaw.append(k[0,0])
        dpitch.append(k[1,0])
        droll.append(k[2,0])
    
    #首先画出位置
    plt.figure()
    plt.subplot(3,1,1)
    if xlabel_kind=='Time':
        plt.plot(abs_error.time,dyaw,'purple',linewidth=2)
        plt.xlabel('time [s]')
    else:
        plt.plot(abs_error.distance,dyaw,'purple',linewidth=2)
        plt.xlabel('distance [m]')
    plt.ylabel('yaw error [deg]')
    #plt.xlim(0,)
    #plt.ylim(0,)
    #plt.title('Error in DataSet08',font1)
    plt.grid(linestyle='--')

    plt.subplot(3,1,2)
    if xlabel_kind=='Time':
        plt.plot(abs_error.time,dpitch,'gold',linewidth=2)
        plt.xlabel('time [s]')
    else:
        plt.plot(abs_error.distance,dpitch,'gold',linewidth=2)
        plt.xlabel('distance [m]')
    plt.ylabel('pitch error [deg]')
    #plt.xlim(0,)
    #plt.ylim(0,)
    plt.grid(linestyle='--')
    #plt.tight_layout(pad=5,rect=(0,0,1,1))

    #roll方向
    plt.subplot(3,1,3)
    if xlabel_kind=='Time':
        plt.plot(abs_error.time,droll,'deepskyblue',linewidth=2)
        plt.xlabel('time [s]')
    else:
        plt.plot(abs_error.distance,droll,'deepskyblue',linewidth=2)
        plt.xlabel('distance [m]')
    plt.ylabel('roll error [deg]')
    #plt.xlim(0,)
    #plt.ylim(0,)
    plt.grid(linestyle='--')
    #自动调整
    plt.tight_layout()
    str=result_dir+'rotation_error.pdf'
    #print(str)
    plt.savefig(str)

    #绘制位置误差图
    plt.figure()
    #E方向或者x方向
    plt.subplot(3,1,1)
    if xlabel_kind=='Time':
        plt.plot(abs_error.time,dx,'purple',linewidth=2)
        plt.xlabel('time [s]')
    else:
        plt.plot(abs_error.distance,dx,'purple',linewidth=2)
        plt.xlabel('distance [m]')
    if ground_coordinatemode=='ECEF' or estimator_coordinatemode=='ECEF':
        plt.ylabel('E error [m]')
    else:
        plt.ylabel('x error [m]')
    #plt.xlim(0,)
    #plt.ylim(0,)
    #plt.title('Error in DataSet08',font1)
    plt.grid(linestyle='--')
    #plt.tight_layout()
    
    #e方向
    plt.subplot(3,1,2)
    if xlabel_kind=='Time':
        plt.plot(abs_error.time,dy,'gold',linewidth=2)
        plt.xlabel('time [s]')
    else:
        plt.plot(abs_error.distance,dy,'gold',linewidth=2)
        plt.xlabel('distance [m]')
    if ground_coordinatemode=='ECEF' or estimator_coordinatemode=='ECEF':
        plt.ylabel('N error [m]')
    else:
        plt.ylabel('y error [m]')
    #plt.xlim(0,)
    #plt.ylim(0,)
    plt.grid(linestyle='--')
    #plt.tight_layout(pad=5,rect=(0,0,1,1))
    
    #u方向
    plt.subplot(3,1,3)
    if xlabel_kind=='Time':
        plt.plot(abs_error.time,dz,'deepskyblue',linewidth=2)
        plt.xlabel('time [s]')
    else:
        plt.plot(abs_error.distance,dz,'deepskyblue',linewidth=2)
        plt.xlabel('distance [m]')
    if ground_coordinatemode=='ECEF' or estimator_coordinatemode=='ECEF':
        plt.ylabel('U error [m]')
    else:
        plt.ylabel('z error [m]')
    #plt.xlim(0,)
    #plt.ylim(0,)
    plt.grid(linestyle='--')
    #自动调整
    plt.tight_layout()
    str=result_dir+'translation_error.pdf'
    #print(str)
    plt.savefig(str)

    #绘制速度误差图
    plt.figure()
    #E方向或者x方向
    plt.subplot(3,1,1)
    if xlabel_kind=='Time':
        plt.plot(abs_error.time,dvel_x,'purple',linewidth=2)
        plt.xlabel('time [s]')
    else:
        plt.plot(abs_error.distance,dvel_x,'purple',linewidth=2)
        plt.xlabel('distance [m]')
    if ground_coordinatemode=='ECEF' or estimator_coordinatemode=='ECEF':
        plt.ylabel('E error [m/s]')
    else:
        plt.ylabel('x error [m/s]')
    #plt.xlim(0,)
    #plt.ylim(0,)
    #plt.title('Error in DataSet08',font1)
    plt.grid(linestyle='--')
    #plt.tight_layout()
    
    #e方向
    plt.subplot(3,1,2)
    if xlabel_kind=='Time':
        plt.plot(abs_error.time,dvel_y,'gold',linewidth=2)
        plt.xlabel('time [s]')
    else:
        plt.plot(abs_error.distance,dvel_y,'gold',linewidth=2)
        plt.xlabel('distance [m]')
    if ground_coordinatemode=='ECEF' or estimator_coordinatemode=='ECEF':
        plt.ylabel('N error [m/s]')
    else:
        plt.ylabel('y error [m/s]')
    #plt.xlim(0,)
    #plt.ylim(0,)
    plt.grid(linestyle='--')
    #plt.tight_layout(pad=5,rect=(0,0,1,1))
    
    #u方向
    plt.subplot(3,1,3)
    if xlabel_kind=='Time':
         plt.plot(abs_error.time,dvel_z,'deepskyblue',linewidth=2)
         plt.xlabel('time [s]')
    else:
         plt.plot(abs_error.distance,dvel_z,'deepskyblue',linewidth=2)
         plt.xlabel('distance [m]')
    if ground_coordinatemode=='ECEF' or estimator_coordinatemode=='ECEF':
        plt.ylabel('U error [m/s]')
    else:
        plt.ylabel('z error [m/s]')
    #plt.xlim(0,)
    #plt.ylim(0,)
    plt.grid(linestyle='--')
    #自动调整
    plt.tight_layout()
    str=result_dir+'velocity_error.pdf'
    #print(str)
    plt.savefig(str)



    #绘制旋转平移误差图
    plt.figure()
    #translation
    plt.subplot(2,1,1)
    plt.plot(abs_error.distance,abs_error.sqrt_pos,'purple',linewidth=2)
    plt.xlabel('distance [m]')
    plt.ylabel('translation error [m]')
    #plt.xlim(0,)
    #plt.ylim(0,)
    #plt.title('Error in DataSet08',font1)
    plt.grid(linestyle='--')
    #plt.tight_layout()
    
    #pitch方向
    plt.subplot(2,1,2)
    plt.plot(abs_error.distance,abs_error.sqrt_att,'gold',linewidth=2)
    plt.xlabel('distance [m]')
    plt.ylabel('rotation error [deg]')
    #plt.xlim(0,)
    #plt.ylim(0,)
    plt.grid(linestyle='--')
    #plt.tight_layout(pad=5,rect=(0,0,1,1))
    plt.tight_layout()
    str=result_dir+'rotation_translation_error.pdf'
    #print(str)
    plt.savefig(str)



def plot_blox(plot_title,result_dir,result_name,xlabel,ylabel,length,task_list):
    # 标题格式
    font1 = {'family' : 'Times New Roman',
    'weight' : 'bold',
    'size' : 18,}
    font2 = {'family' : 'Times New Roman',
     'weight' : 'normal',
    'size' : 15,}
    
    plt.rcParams['xtick.direction'] = 'in' 
    plt.rcParams['ytick.direction'] = 'in' 
    
    fig = plt.figure(figsize=(8, 4))
    bplt=plt.boxplot(task_list, notch=False, sym='', vert=True, patch_artist=True)
    plt.grid(axis='y',which='major',linewidth=0.75,linestyle='-',color='0.75')
    
    ax = plt.gca()
    ax.spines['top'].set_linewidth(2)
    ax.spines['bottom'].set_linewidth(2)
    ax.spines['left'].set_linewidth(2)
    ax.spines['right'].set_linewidth(2)
    
    for compnent  in bplt.keys():
        for line in bplt[compnent]:
            line.set_color('blue')
            line.set_linewidth(2)
    
    for pacthes in bplt['boxes']:
        pacthes.set_facecolor('lightblue')
    
    
    plt.xticks([x+1 for x in range(len(task_list))], length)
    #plt.title('box plot',font1)
    plt.xlabel(xlabel,font2)
    plt.ylabel(ylabel,font2)
    plt.yticks(fontproperties = 'Times New Roman', size = 14)
    plt.xticks(fontproperties = 'Times New Roman', size = 14)
    str=result_dir+result_name
    #print(str)
    plt.savefig(str)

def plot_bloxs(plot_title,result_dir,rel_error1,rel_error2,rel_error3,rel_error4,rel_error5,length):
    #rel_translation_error
    xlabel='Distance [m]'
    ylabel='Translation error [m]'
    tang_error=[rel_error1.rel_tran,rel_error2.rel_tran,rel_error3.rel_tran,rel_error4.rel_tran,rel_error5.rel_tran]
    plot_blox(plot_title,result_dir,'rel_translation_error.pdf',xlabel,ylabel,length,tang_error)

    #rel_translation_error_perc
    xlabel='Distance [m]'
    ylabel='Translation error [%]'
    tang_error=[rel_error1.rel_tran_percentage,rel_error2.rel_tran_percentage,rel_error3.rel_tran_percentage,rel_error4.rel_tran_percentage,rel_error5.rel_tran_percentage]
    plot_blox(plot_title,result_dir,'rel_translation_error_perc.pdf',xlabel,ylabel,length,tang_error)

    #rel_yaw_error
    xlabel='Distance [m]'
    ylabel='Yaw error [deg]'
    tang_error=[rel_error1.yaw,rel_error2.yaw,rel_error3.yaw,rel_error4.yaw,rel_error5.yaw]
    plot_blox(plot_title,result_dir,'rel_yaw_error.pdf',xlabel,ylabel,length,tang_error)

    #rel_yaw_error
    xlabel='Distance [m]'
    ylabel='Pitch error [deg]'
    tang_error=[rel_error1.pitch,rel_error2.pitch,rel_error3.pitch,rel_error4.pitch,rel_error5.pitch]
    plot_blox(plot_title,result_dir,'rel_pitch_error.pdf',xlabel,ylabel,length,tang_error)

    #rel_yaw_error
    xlabel='Distance [m]'
    ylabel='Roll error [deg]'
    tang_error=[rel_error1.roll,rel_error2.roll,rel_error3.roll,rel_error4.roll,rel_error5.roll]
    plot_blox(plot_title,result_dir,'rel_roll_error.pdf',xlabel,ylabel,length,tang_error)