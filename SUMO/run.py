import numpy as np
import traci
import pandas as pd
import Model
def run0(time_step):
    for veh_id in traci.simulation.getDepartedIDList():
        #86:位置，64：速度，114：加速度，191
        traci.vehicle.subscribe(veh_id, [86,64,114,68])
    positions = traci.vehicle.getAllSubscriptionResults()
    df = pd.DataFrame(positions).T
    df.rename(columns = {86: "position",64: "speed",114:'acc',68:'length'}, inplace = True)
    df['time'] =traci.simulation.getTime()-time_step
    return df
def run_cy(L,df):
    df.sort_values('position', inplace=True, ignore_index=False)
    veh_id = df.index.values
    position = df['position'].values
    speed = df['speed'].values
    position0 = position[1:]
    speed0 = speed[1:]
    position1 = np.append(position0,position[0]+L)
    speed1 = np.append(speed0,speed[0])
    headway = position1-position
    delta_v = speed1 - speed
    df['headway'] = headway
    df['delta_v'] = delta_v
    return veh_id,speed,position
def run_open(df):
    veh_id = df.index.values
    position = df['position'].values
    speed = df['speed'].values
    position0 = position[1:]
    speed0 = speed[1:]
    position1 = position[:len(position)-1]
    speed1 = speed[:len(speed)-1]
    headway = position1-position0
    delta_v = speed0-speed1
    headway = np.append(headway,None)
    delta_v = np.append(delta_v,None)
    df['headway'] = headway
    df['delta_v'] = delta_v
    return speed1,veh_id
class Run:
    def __init__(self,end_time,time_step,L,borders,output_df = False,leader_speed=0):
        self.end_time = end_time
        self.time_step = time_step
        self.L = L
        self.output_df = output_df
        self.borders = borders
        self.leader_speed = leader_speed

    def IDM(self,alpha,beta,v0,s0,T):
        for step in np.arange(0,self.end_time,self.time_step):
            df = run0(self.time_step)
            if self.borders == 0:
                if len(df)!=0:
                    veh_id,speed,position = run_cy(self.L,df)
                    a = Model.Cf(df)
                    acc = a.IDM(alpha,beta,v0,s0,T)
                    speed_new = speed+acc*0.1
                    position_new = position + 0.5*(speed+speed_new)*self.time_step
                    for i in range(len(df)):
                        if position_new[i]>self.L:
                            traci.vehicle.moveTo(str(veh_id[i]),'edgeR-1-2_0',position[i]-self.L)
                        traci.vehicle.setSpeedMode(str(veh_id[i]),0)
                        traci.vehicle.setSpeed(str(veh_id[i]),speed_new[i])
            else :
                if len(df)!=0:
                    speed1,veh_id = run_open(df)
                    a = Model.Cf(df)
                    acc = a.IDM(alpha,beta,v0,s0,T)
                    print(speed1)
                    print(acc)
                    speed_new = speed1+acc*0.1
                    speed_new = np.append(speed_new,self.leader_speed)
                    for i in range(len(df)):
                        traci.vehicle.setSpeedMode(str(veh_id[i]),0)
                        traci.vehicle.setSpeed(str(veh_id[i]),speed_new[i])
            if self.output_df:
                print(df)
            traci.simulationStep(step+0.1)
    def OV(self,alpha,L_OV,v_max):
        for step in np.arange(0,self.end_time,self.time_step):
            df = run0(self.time_step)
            if self.borders == 0:
                if len(df)!=0:
                    veh_id,speed,position = run_cy(self.L,df)
                    a = Model.Cf(df)
                    acc = a.OV(alpha,L_OV,v_max)
                    speed_new = speed+acc*0.1
                    position_new = position + 0.5*(speed+speed_new)*self.time_step
                    for i in range(len(df)):
                        if position_new[i]>self.L:
                            traci.vehicle.moveTo(str(veh_id[i]),'edgeR-1-2_0',position[i]-self.L)
                        traci.vehicle.setSpeedMode(str(veh_id[i]),0)
                        traci.vehicle.setSpeed(str(veh_id[i]),speed_new[i])
            else :
                if len(df)!=0:
                    speed1,veh_id= run_open(df)
                    a = Model.Cf(df)
                    acc = a.OV(alpha,L_OV,v_max)
                    speed_new = speed1+acc*0.1
                    speed_new = np.append(speed_new,self.leader_speed)
                    for i in range(len(df)):
                        traci.vehicle.setSpeedMode(str(veh_id[i]),0)
                        traci.vehicle.setSpeed(str(veh_id[i]),speed_new[i])
            if self.output_df:
                print(df)
            traci.simulationStep(step+0.1)
            
    def FVD(self,alpha,kappa,L_OV,v_max):
        for step in np.arange(0,self.end_time,self.time_step):
            df = run0(self.time_step)
            if self.borders == 0:
                if len(df)!=0:
                    veh_id,speed,position = run_cy(self.L,df)
                    a = Model.Cf(df)
                    acc = a.FVD(alpha,kappa,L_OV,v_max)
                    speed_new = speed+acc*0.1
                    position_new = position + 0.5*(speed+speed_new)*self.time_step
                    for i in range(len(df)):
                        if position_new[i]>self.L:
                            traci.vehicle.moveTo(str(veh_id[i]),'edgeR-1-2_0',position[i]-self.L)
                        traci.vehicle.setSpeedMode(str(veh_id[i]),0)
                        traci.vehicle.setSpeed(str(veh_id[i]),speed_new[i])
            else :
                if len(df)!=0:
                    speed1,veh_id= run_open(df)
                    a = Model.Cf(df)
                    acc = a.FVD(alpha,kappa,L_OV,v_max)
                    speed_new = speed1+acc*0.1
                    speed_new = np.append(speed_new,self.leader_speed)
                    for i in range(len(df)):
                        traci.vehicle.setSpeedMode(str(veh_id[i]),0)
                        traci.vehicle.setSpeed(str(veh_id[i]),speed_new[i])
            if self.output_df:
                print(df)
            traci.simulationStep(step+0.1)
