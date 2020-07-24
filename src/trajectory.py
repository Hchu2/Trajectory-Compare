#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time : 2020.7.24
# @Author : LiShengyu
# @File : trajectory.py


import numpy as np
import utils
import trajectory_loading as tl
import plot
import math
import svd
import configparser

#该类只负责存储数据




        
class traj_align:
    #def __init__(self,groundtruth_dir='',estimator_dir='',result_dir='',ground_coordinatemode='Normal',
    #             estimator_coordinatemode='Normal',comparemode='Adjust',use_pos=True,use_velocity=False,use_q=False):
    def __init__(self,config_dir=''):
        

        #python中不能使用bool(xx)来进行强制类型转换
        true_str='True'
        #参考轨迹的路径
        self.config_dir=config_dir
        config = configparser.ConfigParser()
        config.read(self.config_dir)
        #真实轨迹路径 [Dir]所属下的groundtruth_dir
        self.groundtruth_dir=config.get('Dir', 'groundtruth_dir')
        #估计轨迹路径
        self.estimator_dir=config.get('Dir', 'estimator_dir')
        #结果文件路径
        self.result_dir=config.get('Dir', 'result_dir')
        #参考轨迹位置坐标的形式 Normal XYZ
        self.ground_coordinatemode=config.get('Mode', 'groundtruth_coordinatemode')
        #估计轨迹位置坐标的形式
        self.estimator_coordinatemode=config.get('Mode', 'estimator_coordinatemode')
        #比较的形式 Adjust Adjust_yaw Translate
        self.comparemode=config.get('Mode', 'compare_mode')
        #开始时间
        self.start_time=float(config.get('Mode','start_time'))
        #结束时间
        self.end_time=float(config.get('Mode','end_time'))
        #全段拟合百分比
        self.adjust_percentage=config.get('Mode','Adjust_percentage')
        #是否使用位置
        self.use_pos=true_str == config.get('Status', 'Is_use_pos')  
        #是否使用速度
        self.use_velocity=true_str == config.get('Status', 'Is_use_vel')          
        #是否使用姿态
        self.use_q=true_str == config.get('Status', 'Is_use_att')
        #姿态的载入模式
        self.gt_att_mode=config.get('Status', 'gt_att_mode')
        self.es_att_mode=config.get('Status', 'es_att_mode')
        #速度的载入模式
        self.gt_vel_mode=config.get('Status','gt_vel_mode')
        self.es_vel_mode=config.get('Status','es_vel_mode')
        #真实轨迹-间隔符号
        self.gt_interval=config.get('Status','gt_interval')
        #估计轨迹-间隔符号
        self.es_interval=config.get('Status','es_interval')

        #真实轨迹-时间索引
        self.gt_time_index=config.get('Index', 'gt_time_index').split(",")
        #真实轨迹-位置索引
        self.gt_pos_index=config.get('Index', 'gt_pos_index').split(",")
        #真实轨迹-速度索引
        self.gt_vel_index=config.get('Index', 'gt_vel_index').split(",") 
        #真实轨迹-姿态索引
        self.gt_att_index=config.get('Index', 'gt_att_index').split(",") 
        
        #估计轨迹-时间索引
        self.es_time_index=config.get('Index', 'es_time_index').split(",")
        #估计轨迹-位置索引
        self.es_pos_index=config.get('Index', 'es_pos_index').split(",") 
        #估计轨迹-速度索引
        self.es_vel_index=config.get('Index', 'es_vel_index').split(",")  
        #估计轨迹-姿态索引
        self.es_att_index=config.get('Index', 'es_att_index').split(",")
     
        #画图的横坐标
        self.xlabel=config.get('Plot','xlabel')
        self.fixed_diffs=config.get('Plot', 'fixed_differences').split(",") 


        print("---------------------------------------------")
        print("The groundtruth dir is %s" %(self.groundtruth_dir))
        print("The estimator dir is %s" %(self.estimator_dir))
        print()
        print("The groundtruth coordinatemode is %s" %(self.ground_coordinatemode))
        print("The estimator coordinatemode is %s" %(self.estimator_coordinatemode))
        print("The compare mode is %s" %(self.comparemode))
        print()
        if self.use_pos:
            print("The state of compare the position is True")
        else:
            print("The state of compare the position is False")
        if self.use_velocity:
            print("The state of compare the velocity is True")
        else:
            print("The state of compare the velocity is False")
        if self.use_q:
            print("The state of compare the attitude is True")
            print("The attitude mode of groundtruth of %s" %(self.gt_att_mode))
            print("The attitude mode of estimator of %s" %(self.es_att_mode))
        else:
            print("The state of compare the attitude is  False")
        print("---------------------------------------------")
        print()

        

        #用于原始数据的存储
        self.groundtruth=[]
        self.estimator=[]


        #对齐并进行了位置、速度转换的数据
        self.aligned_groundtruth=[]
        self.aligned_estimator=[]

        #对齐的索引
        self.indexs=[]

        #记录下参考轨迹和估计轨迹第一个的坐标
        self.init_groundtruth_pos=np.mat(np.zeros((3,1)))
        self.init_estimator_pos=np.mat(np.zeros((3,1)))
        #记录下参考轨迹和估计轨迹的第一个的R_e_n
        self.groundtruth_R_e_n=np.mat(np.zeros((3,1)))
        self.estimator_R_e_n=np.mat(np.zeros((3,1)))
        
        #分段里程
        self.subtraj_length=[]


        #记录相对误差 用于画图
        self.rel_error_1=utils.rel_error()
        self.rel_error_2=utils.rel_error()
        self.rel_error_3=utils.rel_error()
        self.rel_error_4=utils.rel_error()
        self.rel_error_5=utils.rel_error()
        #用于做表格的数据
        self.chart_info=utils.chart_info()
        self.abs_error=utils.abs_error()
        #用于画轨迹图
        self.diffs_traj=utils.diff_traj()
        
        #录入数据
        self.load_data()
        #对齐轨迹
        self.align_trajectory()
       
        str="the length of align_groundtruth is %d" %(len(self.aligned_groundtruth))
        print(str)
        str="the length of align_estimator is %d" %(len(self.aligned_estimator))
        print(str)
        print()
        
        #进行比较
        if self.comparemode=='Adjust' or self.comparemode=='Adjust_Yaw':
            self.AdjustTrajectory()
        else:
            self.TranslateTrajectory()

        #画相关图
        self.plot_info()
        
        #输出
        self.result_write()

    #读取轨迹数据
    def load_data(self):
        print('Loading trajectory data...')
        print()

        #self.groundtruth=tl.read_IE_result(self.groundtruth_dir)
        #self.groundtruth=tl.read_uzh_result(self.groundtruth_dir)
        self.groundtruth=tl.load_iedata(self.groundtruth_dir,self.start_time,self.end_time,self.use_pos,self.use_velocity,self.use_q,
                                      self.ground_coordinatemode,self.gt_att_mode,self.gt_time_index,self.gt_pos_index,self.gt_vel_index,self.gt_att_index,self.gt_interval,self.fixed_diffs)
        str="the length of groundtruth is %d" %(len(self.groundtruth))
        print(str)
        
        #self.estimator=tl.read_gnut_result(self.estimator_dir)
        #self.estimator=tl.read_uzh_result(self.estimator_dir)
        self.estimator=tl.load_data(self.estimator_dir,self.start_time,self.end_time,self.use_pos,self.use_velocity,self.use_q,
                                    self.estimator_coordinatemode,self.es_att_mode,self.es_time_index,self.es_pos_index,self.es_vel_index,self.es_att_index,self.es_interval)
        str="the length of estimator is %d" %(len(self.estimator))
        print(str)
        print()
        

    #轨迹对齐，将对齐的筛选出来，并对e系下的坐标进行处理
    def align_trajectory(self):
        print("Aligning the trajectory data...")
        print()
        #获取匹配的列表信息
        i=0
        j=0
        dt=1000000000.0 #dt 为NAN
        while i<len(self.groundtruth) and j<len(self.estimator):
            if self.groundtruth[i].time<self.estimator[j].time:
                if self.estimator[j].time-self.groundtruth[i].time<dt+0.00001:
                    dt=self.estimator[j].time-self.groundtruth[i].time
                    i+=1
                    continue
                else:
                    self.indexs.append([i,j-1])
                    i+=1
                    dt=1000000000
                    continue

            else:
                if self.groundtruth[i].time-self.estimator[j].time<dt+0.00001:
                    dt=self.groundtruth[i].time-self.estimator[j].time
                    j+=1
                    continue
                else:
                    self.indexs.append([i-1,j])
                    j+=1
                    dt=1000000000
                    continue
        str="the trajectory matching number is %d" %(len(self.indexs))
        print(str)

        #筛选出来 存入aligned_groundtruth和aligned_estimator
        #首先对参考轨迹处理，并计算里程
        first_ground_traj=True
        last_pos=np.mat(np.zeros((3,1)))
        distance=0.0
        for i in self.indexs:
            ground_traj=self.groundtruth[i[0]]
            if first_ground_traj:
                self.init_groundtruth_pos=ground_traj.position
                first_ground_traj=False
                last_pos=ground_traj.position
                #XYZ模式
                if(self.ground_coordinatemode=='ECEF'):
                    self.groundtruth_R_e_n=utils.R_ECEF_ENU(utils.XYZ2BLH(self.init_groundtruth_pos))
                    #print(self.groundtruth_R_e_n)
            #转至第一个点的n系下
            if self.gt_vel_mode=='ENU':
                cur_R_e_n=utils.R_ECEF_ENU(utils.XYZ2BLH(ground_traj.position))
                tmp_vel=cur_R_e_n.transpose()*ground_traj.velocity
                ground_traj.velocity=tmp_vel
            #添加 必须在是adjust的情况下才这么做
            if self.ground_coordinatemode=='ECEF' and self.comparemode!='Translate':
                ground_traj.align_pos=self.groundtruth_R_e_n@(ground_traj.position-self.init_groundtruth_pos)#不是简单的点乘！！！
                ground_traj.align_velocity=self.groundtruth_R_e_n@ground_traj.velocity
                ground_traj.align_q=ground_traj.q
            #Normal
            else:
                ground_traj.align_pos=ground_traj.position
                ground_traj.align_velocity=ground_traj.velocity
                ground_traj.align_q=ground_traj.q
            #计算里程
            dpos=ground_traj.position-last_pos
            distance+=math.sqrt(dpos[0,0]*dpos[0,0]+dpos[1,0]*dpos[1,0]+dpos[2,0]*dpos[2,0])
            last_pos=ground_traj.position
            ground_traj.distance=distance
            self.aligned_groundtruth.append(ground_traj)

        
        #对estimator进行操作
        first_estimator_traj=True
        for j in self.indexs:
            estimator_traj=self.estimator[j[1]]
            if first_estimator_traj:
                self.init_estimator_pos=estimator_traj.position
                #print(estimator_traj.position)
                first_estimator_traj=False
                last_pos=estimator_traj.position
                #XYZ模式
                if(self.estimator_coordinatemode=='ECEF') :
                    self.estimator_R_e_n=utils.R_ECEF_ENU(utils.XYZ2BLH(self.init_estimator_pos))
                    
            #转至第一个点的n系下
            if self.es_vel_mode=='ENU':
                cur_R_e_n=utils.R_ECEF_ENU(utils.XYZ2BLH(estimator_traj.position))
                tmp_vel=cur_R_e_n.transpose()*estimator_traj.velocity
                estimatro_traj.velocity=tmp_vel
            if self.estimator_coordinatemode=='ECEF'and self.comparemode!='Translate':
                estimator_traj.align_pos=self.groundtruth_R_e_n@(estimator_traj.position-self.init_estimator_pos)
                estimator_traj.align_velocity=self.groundtruth_R_e_n@estimator_traj.velocity
                estimator_traj.align_q=estimator_traj.q
            #Normal
            else:
                estimator_traj.align_pos=estimator_traj.position
                estimator_traj.align_velocity=estimator_traj.velocity
                estimator_traj.align_q=estimator_traj.q
                #print(estimator_traj.align_pos)		
            self.aligned_estimator.append(estimator_traj)
       
    
    def compute_relative_error(self):
        all_mileages=int(self.aligned_groundtruth[len(self.aligned_groundtruth)-1].distance)
        str="the all mileage is %d m" %(all_mileages)
        print(str)
        print()

        print("Computint relative errors... ")
        print()
        for i in range(1,6):
            #计算分段里程 0.1 0.2 0.3 0.4 0.5 作为两点求相对位姿变换的界限值
            self.subtraj_length.append(int((i)/10.0*all_mileages))
            
        #分段开始处理
        for i in range(5):   #0,1,2,3,4
            subsection_index=[]
            max_dist_diff=0.2*self.subtraj_length[i]
            #遍历全段每个数据，寻找在0.2倍距离之内的点
            for j in range(len(self.aligned_groundtruth)-2):#<
                front=self.aligned_groundtruth[j]
                for k in range(j+1,len(self.aligned_groundtruth)-1):#<=
                    back_one=self.aligned_groundtruth[k]
                    back_two=self.aligned_groundtruth[k+1]
                    dpos1=back_two.distance-(front.distance+self.subtraj_length[i])
                    dpos2=front.distance+self.subtraj_length[i]-back_one.distance
                    if dpos1>=0 and dpos2>=0:
                        if dpos1>dpos2 and dpos2<=max_dist_diff:#前面的更近
                            subsection_index.append([j,k])
                            continue
                        if dpos2>dpos1 and dpos1<=max_dist_diff:#后面的更近
                            subsection_index.append([j,k+1])
                            continue
                    if k==len(self.aligned_groundtruth)-2 and dpos1<0 and abs(dpos1)<=max_dist_diff:
                        subsection_index.append([j,k+1])
                        continue
            str="The trajectory at %dm have %d matching points... " %(self.subtraj_length[i],len(subsection_index))
            print(str)
            

            #开始计算相对误差
            T_gt_front=np.mat(np.zeros((4,4)))
            T_gt_back=np.mat(np.zeros((4,4)))
            T_gt_12=np.mat(np.zeros((4,4)))
            T_es_front=np.mat(np.zeros((4,4)))
            T_es_back=np.mat(np.zeros((4,4)))
            T_es_12=np.mat(np.zeros((4,4)))
            T_error=np.mat(np.zeros((4,4)))
            T_errors=[]
            for ii in subsection_index:
                ground_traj_front=self.aligned_groundtruth[ii[0]]
                ground_traj_back=self.aligned_groundtruth[ii[1]]
                estimator_traj_front=self.aligned_estimator[ii[0]]
                estimator_traj_back=self.aligned_estimator[ii[1]]
                
                utils.linear_translation(T_gt_front,utils.Quaternion2Rotation(ground_traj_front.align_q),ground_traj_front.align_pos)
                utils.linear_translation(T_gt_back,utils.Quaternion2Rotation(ground_traj_back.align_q),ground_traj_back.align_pos)
                T_gt_12=T_gt_front.I*T_gt_back
                
                utils.linear_translation(T_es_front,utils.Quaternion2Rotation(estimator_traj_front.align_q),estimator_traj_front.align_pos)
                utils.linear_translation(T_es_back,utils.Quaternion2Rotation(estimator_traj_back.align_q),estimator_traj_back.align_pos)
                T_es_12=T_es_front.I*T_es_back

                T_c2_rot=np.mat(np.zeros((4,4)))
                #T_c2_rot=T_es_back.linear()
                utils.linear(T_c2_rot,utils.Quaternion2Rotation(estimator_traj_back.align_q))
                T_error=T_c2_rot*T_gt_12.I*T_es_12*T_c2_rot.I          
                T_errors.append(T_error)
            #对误差进行处理，存储到对应的rel_error中 用于后续画图，输出并进行比较
            for kk in range(len(T_errors)):
                gt_traj=self.aligned_groundtruth[kk]
                error=T_errors[kk]
                rel_trans=utils.get_translation(error)
                rel_rot=utils.get_linear(error)
                #rel_ypr=utils.Rotation2Euler_uzh(rel_rot)*180.0/math.pi #单位为度
                rel_ypr=utils.Rotation2Euler_IE(rel_rot)*180.0/math.pi #单位为度
                rel_trans_sqrt=math.sqrt(rel_trans[0]*rel_trans[0]+rel_trans[1]*rel_trans[1]+rel_trans[2]*rel_trans[2])
               
                utils.correct_ypr(rel_ypr)
                if i==0:
                    self.rel_error_1.time.append(gt_traj.time)
                    self.rel_error_1.distance.append(gt_traj.distance)
                    self.rel_error_1.rel_tran.append(rel_trans_sqrt)
                    self.rel_error_1.rel_tran_percentage.append(rel_trans_sqrt/self.subtraj_length[i]*100)
                    self.rel_error_1.yaw.append(abs(rel_ypr[0]))
                    self.rel_error_1.pitch.append(abs(rel_ypr[1]))
                    self.rel_error_1.roll.append(abs(rel_ypr[2]))
                    self.rel_error_1.angle.append(math.sqrt(rel_ypr[0]*rel_ypr[0]+rel_ypr[1]*rel_ypr[1]+rel_ypr[2]*rel_ypr[2]))
                if i==1:
                    self.rel_error_2.time.append(gt_traj.time)
                    self.rel_error_2.distance.append(gt_traj.distance)
                    self.rel_error_2.rel_tran.append(rel_trans_sqrt)
                    self.rel_error_2.rel_tran_percentage.append(rel_trans_sqrt/self.subtraj_length[i]*100)
                    self.rel_error_2.yaw.append(abs(rel_ypr[0]))
                    self.rel_error_2.pitch.append(abs(rel_ypr[1]))
                    self.rel_error_2.roll.append(abs(rel_ypr[2]))
                    self.rel_error_2.angle.append(math.sqrt(rel_ypr[0]*rel_ypr[0]+rel_ypr[1]*rel_ypr[1]+rel_ypr[2]*rel_ypr[2]))
                if i==2:
                    self.rel_error_3.time.append(gt_traj.time)
                    self.rel_error_3.distance.append(gt_traj.distance)
                    self.rel_error_3.rel_tran.append(rel_trans_sqrt)
                    self.rel_error_3.rel_tran_percentage.append(rel_trans_sqrt/self.subtraj_length[i]*100)
                    self.rel_error_3.yaw.append(abs(rel_ypr[0]))
                    self.rel_error_3.pitch.append(abs(rel_ypr[1]))
                    self.rel_error_3.roll.append(abs(rel_ypr[2]))
                    self.rel_error_3.angle.append(math.sqrt(rel_ypr[0]*rel_ypr[0]+rel_ypr[1]*rel_ypr[1]+rel_ypr[2]*rel_ypr[2]))
                if i==3:
                    self.rel_error_4.time.append(gt_traj.time)
                    self.rel_error_4.distance.append(gt_traj.distance)
                    self.rel_error_4.rel_tran.append(rel_trans_sqrt)
                    self.rel_error_4.rel_tran_percentage.append(rel_trans_sqrt/self.subtraj_length[i]*100)
                    self.rel_error_4.yaw.append(abs(rel_ypr[0]))
                    self.rel_error_4.pitch.append(abs(rel_ypr[1]))
                    self.rel_error_4.roll.append(abs(rel_ypr[2]))
                    self.rel_error_4.angle.append(math.sqrt(rel_ypr[0]*rel_ypr[0]+rel_ypr[1]*rel_ypr[1]+rel_ypr[2]*rel_ypr[2]))
                if i==4:
                    self.rel_error_5.time.append(gt_traj.time)
                    self.rel_error_5.distance.append(gt_traj.distance)
                    self.rel_error_5.rel_tran.append(rel_trans_sqrt)
                    self.rel_error_5.rel_tran_percentage.append(rel_trans_sqrt/self.subtraj_length[i]*100)
                    self.rel_error_5.yaw.append(abs(rel_ypr[0]))
                    self.rel_error_5.pitch.append(abs(rel_ypr[1]))
                    self.rel_error_5.roll.append(abs(rel_ypr[2]))
                    self.rel_error_5.angle.append(math.sqrt(rel_ypr[0]*rel_ypr[0]+rel_ypr[1]*rel_ypr[1]+rel_ypr[2]*rel_ypr[2]))
            str="The relative error at %dm have been computed... " %(self.subtraj_length[i])
            print(str)
            print()
    
    def compute_absolute_error(self):
        print("Computint absolute errors... ")
        R=np.mat(np.zeros((3,3)))
        t=np.mat(np.zeros((3,1)))
        if self.comparemode=='Adjust':
            R,t=svd.pos_estimator(self.aligned_groundtruth,self.aligned_estimator,0,int(float(self.adjust_percentage)*(len(self.aligned_groundtruth)-1)))
        else:#Adjust_Yaw
            R,t=svd.pos_estimator_best_yaw(self.aligned_groundtruth,self.aligned_estimator,0,int(float(self.adjust_percentage)*(len(self.aligned_groundtruth)-1)))
        
        print("R")
        print(R)
        print("t")
        print(t)
        #输出误差 和轨迹
        t_abs=0.0
        for i in range(len(self.aligned_groundtruth)):
            ground_traj=self.aligned_groundtruth[i]
            ground_pos=ground_traj.align_pos
            ground_vel=ground_traj.align_velocity
            ground_q=ground_traj.align_q

            estimator_traj=self.aligned_estimator[i]
            estimator_pos=estimator_traj.align_pos
            estimator_vel=estimator_traj.align_velocity
            estimator_q=estimator_traj.align_q

            #误差计算为真值减去估计值
            dpos=ground_pos-R*estimator_pos-t
            dvel=ground_vel-R*estimator_vel
            t_abs+=dpos[0]*dpos[0]+dpos[1]*dpos[1]+dpos[2]*dpos[2]
            dR=R*utils.Quaternion2Rotation(estimator_q)*(utils.Quaternion2Rotation(ground_q)).T
            #datt=utils.Rotation2Euler_uzh(dR)*180.0/math.pi
            datt=utils.Rotation2Euler_IE(dR)*180.0/math.pi
            utils.correct_ypr(datt)

            #填充绝对误差
            self.abs_error.time.append(ground_traj.time)
            self.abs_error.distance.append(ground_traj.distance)
            self.abs_error.dpos.append(dpos)
            self.abs_error.dvel.append(dvel)
            self.abs_error.datt.append(datt)
            self.abs_error.sqrt_pos.append(math.sqrt(dpos[0]*dpos[0]+dpos[1]*dpos[1]+dpos[2]*dpos[2]))
            self.abs_error.sqrt_att.append(math.sqrt(datt[0]*datt[0]+datt[1]*datt[1]+datt[2]*datt[2]))
            #填充轨迹信息
            self.diffs_traj.time.append(ground_traj.time)
            self.diffs_traj.distance.append(ground_traj.distance)
            self.diffs_traj.ground_pos.append(ground_pos)
            self.diffs_traj.align_estimator_pos.append(ground_pos-dpos)
            if i==len(self.aligned_groundtruth)-1:
                self.chart_info.Length=ground_traj.distance
                self.chart_info.t_rel=math.sqrt(dpos[0]*dpos[0]+dpos[1]*dpos[1]+dpos[2]*dpos[2])/ground_traj.distance*100.0
                self.chart_info.r_rel=math.sqrt(datt[0]*datt[0]+datt[1]*datt[1]+datt[2]*datt[2])/ground_traj.distance*100.0
        self.chart_info.t_abs=math.sqrt(t_abs/len(self.aligned_groundtruth))
        
        

    def AdjustTrajectory(self,scale=1.0):
        self.compute_relative_error()
        self.compute_absolute_error()

    def TranslateTrajectory(self):
        t_abs=0.0
        for i in range(len(self.aligned_groundtruth)):
            ground_traj=self.aligned_groundtruth[i]
            ground_pos=ground_traj.align_pos
            ground_vel=ground_traj.align_velocity
            ground_q=ground_traj.align_q

            estimator_traj=self.aligned_estimator[i]
            estimator_pos=estimator_traj.align_pos
            estimator_vel=estimator_traj.align_velocity
            estimator_q=estimator_traj.align_q

            #误差计算为真值减去估计值
            #添加，分为XYZ XYZ和NORMAL NORMAL
            #只有两种情况
            if self.ground_coordinatemode=='ECEF' and self.estimator_coordinatemode=='ECEF':
                dpos=self.groundtruth_R_e_n@(ground_pos-estimator_pos)
                dvel=self.groundtruth_R_e_n@(ground_vel-estimator_vel)
            else:
                dpos=ground_pos-estimator_pos
                dvel=ground_vel-estimator_vel
            t_abs+=dpos[0]*dpos[0]+dpos[1]*dpos[1]+dpos[2]*dpos[2]
            dR=utils.Quaternion2Rotation(estimator_q)*(utils.Quaternion2Rotation(ground_q)).T
            datt=utils.Rotation2Euler_IE(dR)*180.0/math.pi
            #datt=utils.Rotation2Euler_uzh(dR)*180.0/math.pi
            utils.correct_ypr(datt)
            #填充绝对误差
            self.abs_error.time.append(ground_traj.time)
            self.abs_error.distance.append(ground_traj.distance)
            self.abs_error.dpos.append(dpos)
            self.abs_error.dvel.append(dvel)
            self.abs_error.datt.append(datt)
            self.abs_error.sqrt_pos.append(math.sqrt(dpos[0]*dpos[0]+dpos[1]*dpos[1]+dpos[2]*dpos[2]))
            self.abs_error.sqrt_att.append(math.sqrt(datt[0]*datt[0]+datt[1]*datt[1]+datt[2]*datt[2]))
            #填充轨迹信息
            self.diffs_traj.time.append(ground_traj.time)
            self.diffs_traj.distance.append(ground_traj.distance)
            if self.ground_coordinatemode=='ECEF' and self.estimator_coordinatemode=='ECEF':
                #给出N系下的偏差
                self.diffs_traj.ground_pos.append(self.groundtruth_R_e_n@(ground_pos-self.init_groundtruth_pos))
                self.diffs_traj.align_estimator_pos.append(self.groundtruth_R_e_n@(ground_pos-self.init_groundtruth_pos)-dpos)
                #print(self.groundtruth_R_e_n@(ground_pos-self.init_groundtruth_pos))
            else:
                #否则就是xyz模式
                self.diffs_traj.ground_pos.append(ground_pos)
                self.diffs_traj.align_estimator_pos.append(ground_pos-dpos)
            if i==len(self.aligned_groundtruth)-1:
                self.chart_info.Length=ground_traj.distance
                #拟合没有 t_rel和 r_rel
        self.chart_info.t_abs=math.sqrt(t_abs/len(self.aligned_groundtruth))
        

    def plot_info(self):
        print("Start ploting trajs...")
        print()
        print("Ploting trajectory diagram..." )
        print()
        plot.plot_traj('',self.result_dir,self.diffs_traj,self.ground_coordinatemode,self.estimator_coordinatemode)

        print("Ploting absolute error...")
        print()
        plot.plot_abs_error('',self.xlabel,self.result_dir,self.abs_error,self.ground_coordinatemode,self.estimator_coordinatemode)

        if self.comparemode!='Translate':
            print("Ploting blox of relative error...")
            print()
            plot.plot_bloxs('',self.result_dir,self.rel_error_1,self.rel_error_2,self.rel_error_3,self.rel_error_4,self.rel_error_5,self.subtraj_length)


    def result_write(self):
        print("Starting write infomation to file(txt)...")
        print()
        str=self.result_dir+'chart_info.txt'
        f=open(str,'w')
        
        f.write("Length is %fm" %(self.chart_info.Length))
        f.write('\n')
        f.write("t_rel is %f%%" %(self.chart_info.t_rel))
        f.write('\n')
        f.write("r_rel is %fdeg/100m" %(self.chart_info.r_rel))
        f.write('\n')
        f.write("t_abs ims %fm" %(self.chart_info.t_abs))
        f.write('\n')
        f.close()

        #输出全段误差信息 
        #格式为 time pos vel att
        str=self.result_dir+'absolute_error.txt'
        f=open(str,'w')
        f.write("#time dpos dvel datt\n")

        for time,dpos,dvel,datt,mileage in zip(self.abs_error.time,self.abs_error.dpos,self.abs_error.dvel,self.abs_error.datt,self.abs_error.distance):
            f.write("%.4f  " %(time))
            f.write("%.4f  " %(mileage))
            f.write("%.5f  %.5f  %.5f  " %(dpos[0,0],dpos[1,0],dpos[2,0]))
            f.write("%.5f  %.5f  %.5f  " %(dvel[0,0],dvel[1,0],dvel[2,0]))
            f.write("%.5f  %.5f  %.5f  " %(datt[0,0],datt[1,0],datt[2,0]))
            f.write('\n')


        f.close()