import random
import numpy as np
import pandas as pd
np.set_printoptions(suppress=True)


# 0ID,1时间，2位置，3速度，4加速度，5类型(0HDV，1CAV)，6车道,7路径(0主道出，1匝道出,8检测器信息（（检测器ID，当前时间）)
def add_car(lane, t, num, p_add, p_auto, v, veh_list, route_p):
    pos = 0
    new_veh = []
    if len(veh_list) != 0:
        pos = min(veh_list[-1][2]-15, 0)
    if random.random() <= p_add:
        new_veh = [num, t, pos, v, 0, 0, lane, 0,[]]
        num += 1
        if random.random() <= p_auto:
            new_veh[5] = 1
        if random.random() <= route_p:
            new_veh[7] = 1
    return new_veh, num


def veh_disappear(veh_list, length, disappear_num,pass_time):
    while True:
        if len(veh_list) != 0:
            if veh_list[0][2] > length:
                if len(veh_list[0][8])!=0:
                    pass_time.append(veh_list[0][8])
                del veh_list[0]
                disappear_num += 1
            else:
                break
        else:
            break
    return veh_list, disappear_num,pass_time


def IDM_HV(v, v_lead, x_lead, x,lane_id ,tao=0.4):
    alpha = 3
    s0 = 2
    T = 1.5
    beta = 1.5
    v0 = speed_limit(lane_id, x)
    veh_len = 5
    delta_v = v_lead - v
    s = x_lead - x - veh_len
    s_star = s0 + v*tao + max(v * T - v * delta_v / 2 / (alpha * beta) ** 0.5, 0)
    acc = alpha * (1 - (v / v0) ** 4 - (s_star / s) ** 2)
    return acc

def IDM_CAV(v, v_lead1, v_lead2, x, x_lead1, x_lead2, lane_id):
    alpha = 3
    s0 = 2
    T = 1.5
    beta = 1.5
    v0 = speed_limit(lane_id, x)
    veh_len = 5
    gamma1 = 0.7
    gamma2 = 0.3
    delta_v = gamma1*(v_lead1-v) + gamma2*(v_lead2 - v_lead1)
    s = gamma1*(x_lead1-x-veh_len) + gamma2*(x_lead2 - x_lead1 - veh_len)
    s_star = s0 + max(v * T - v * delta_v / 2 / (alpha * beta) ** 0.5, 0)
    acc = alpha * (1 - (v / v0) ** 4 - (s_star / s) ** 2)
    return acc
def change_lane_identify(target_lane,pos):
    if target_lane == 3:
        return 3
    else:
        if target_lane == 1:
            if pos<14000:
                return 1.1
            else:
                return 1.2
        elif target_lane == 2:
            if pos<13000:
                return 2.1
            else:
                return 2.2

def LC(veh_list1_1,veh_list1_2,veh_list2_1, veh_list2_2, p, a_bias,no_DLC,target_lane):
    veh_list1 = []
    veh_list2 = []
    veh_list1.extend(veh_list1_2)
    veh_list1.extend(veh_list1_1)
    veh_list2.extend(veh_list2_2)
    veh_list2.extend(veh_list2_1)
    s0 = 2
    tao = 0.4
    veh_len = 5
    change = []
    judge = 0
    T = 1
    DSRC = 100
    for i in range(len(veh_list1)):
        if veh_list1[i][2] < 0:
            continue
        if (veh_list1[i][7]==1)&(veh_list1[i][2]<=no_DLC[1])&(veh_list1[i][2]>=no_DLC[0]):
            continue
        pos_range = (veh_list1[i][2] - veh_len - s0 - veh_list1[i][3] * (tao+T) * (1 - veh_list1[i][5]),
                     veh_list1[i][2] + veh_len + s0 + veh_list1[i][3] * (tao+T) * (1 - veh_list1[i][5]))
        for j in range(len(veh_list2)):
            if (veh_list2[j][2] < pos_range[1]) & (veh_list2[j][2] > pos_range[0]):
                judge = 1
                break
        if judge == 1:
            continue
        lead = 'no_lead'
        for k in range(len(veh_list2)):
            s_l = veh_list2[k][2] - veh_list1[i][2]
            if s_l > 0:
                lead = k
            else:
                break
        if lead == 'no_lead':
            if len(veh_list2) == 0:
                acc_lag_ = 0
                acc_lag = 0
            else:
                if veh_list2[0][5] == 0:
                    acc_lag = IDM_HV(veh_list2[0][3], 0, veh_list2[0][2]+1000, veh_list2[0][2],veh_list2[0][6])
                    acc_lag_ = IDM_HV(veh_list2[0][3], veh_list1[i][3], veh_list1[i][2], veh_list2[0][2], veh_list2[0][6])
                else:
                    acc_lag = IDM_HV(veh_list2[0][3], 0, veh_list2[0][2]+1000, veh_list2[0][2], veh_list2[0][6], tao=0)
                    acc_lag_ = IDM_HV(veh_list2[0][3], veh_list1[i][3], veh_list1[i][2], veh_list2[0][2], veh_list2[0][6], tao=0)
        elif lead == len(veh_list2) - 1:
            acc_lag = 0
            acc_lag_ = 0
        else:
            if (veh_list2[lead+1][5] == 0):
                acc_lag = IDM_HV(veh_list2[lead + 1][3], veh_list2[lead][3], veh_list2[lead][2], veh_list2[lead + 1][2], veh_list2[lead + 1][6])
            else:
                if lead == 0:
                    acc_lag = IDM_HV(veh_list2[lead+1][3], veh_list2[lead][3], veh_list2[lead][2], veh_list2[lead+1][2], veh_list2[lead+1][6], tao=0)
                else:
                    if (veh_list2[lead-1][2]-veh_list2[lead+1][2]) > DSRC:
                        acc_lag = IDM_HV(veh_list2[lead+1][3], veh_list2[lead][3], veh_list2[lead][2], veh_list2[lead+1][2], veh_list2[lead+1][6], tao=0)
                    else:
                        acc_lag = IDM_CAV(veh_list2[lead+1][3], veh_list2[lead][3], veh_list2[lead-1][3], veh_list2[lead+1][2], veh_list2[lead][2], veh_list2[lead-1][2], veh_list2[lead + 1][6])
            if (veh_list2[lead + 1][5] == 0):
                acc_lag_ = IDM_HV(veh_list2[lead + 1][3], veh_list1[i][3], veh_list1[i][2], veh_list2[lead + 1][2], veh_list2[lead + 1][6])
            else:
                if lead == 'no_lead':
                    acc_lag_ = IDM_HV(veh_list2[lead + 1][3], veh_list1[i][3], veh_list1[i][2], veh_list2[lead + 1][2], veh_list2[lead + 1][6], tao=0)
                else:
                    if (veh_list2[lead][2] - veh_list2[lead+1][2]) > DSRC:
                        acc_lag_ = IDM_HV(veh_list2[lead + 1][3], veh_list1[i][3], veh_list1[i][2],
                                          veh_list2[lead + 1][2], veh_list2[lead + 1][6], tao=0)
                    else:
                        acc_lag_ = IDM_CAV(veh_list2[lead + 1][3], veh_list1[i][3], veh_list2[lead][3], veh_list2[lead+1][2], veh_list1[i][2], veh_list2[lead][2], veh_list2[lead + 1][6])
        if acc_lag_ < -2:
            continue
        if lead != 'no_lead':
            if (veh_list1[i][5] == 0):
                acc_ = IDM_HV(veh_list1[i][3], veh_list2[lead][3], veh_list2[lead][2], veh_list1[i][2], change_lane_identify(target_lane,veh_list1[i][2]))
            else:
                if lead == 0:
                    acc_ = IDM_HV(veh_list1[i][3], veh_list2[lead][3], veh_list2[lead][2], veh_list1[i][2], change_lane_identify(target_lane, veh_list1[i][2]), tao=0)
                else:
                    if (veh_list2[lead-1][2]-veh_list1[i][2]) > DSRC:
                        acc_ = IDM_HV(veh_list1[i][3], veh_list2[lead][3], veh_list2[lead][2], veh_list1[i][2], change_lane_identify(target_lane, veh_list1[i][2]), tao=0)
                    else:
                        acc_ = IDM_CAV(veh_list1[i][3], veh_list2[lead][3], veh_list2[lead-1][3], veh_list1[i][2], veh_list2[lead][2], veh_list2[lead-1][2], change_lane_identify(target_lane, veh_list1[i][2]))
        else:
            acc_ = IDM_HV(veh_list1[i][3], 0, veh_list1[i][2] + 1000, veh_list1[i][2], change_lane_identify(target_lane, veh_list1[i][2]))
        if i != 0:
            if (veh_list1[i][5]==0):
                acc = IDM_HV(veh_list1[i][3], veh_list1[i - 1][3], veh_list1[i - 1][2], veh_list1[i][2], veh_list1[i][6])
            else:
                if i == 1:
                    acc = IDM_HV(veh_list1[i][3], veh_list1[i - 1][3], veh_list1[i - 1][2], veh_list1[i][2], veh_list1[i][6], tao=0)
                else:
                    if (veh_list1[i-2][2]-veh_list1[i][2]) > DSRC:
                        acc = IDM_HV(veh_list1[i][3], veh_list1[i - 1][3], veh_list1[i - 1][2], veh_list1[i][2], veh_list1[i][6], tao=0)
                    else:
                        acc = IDM_CAV(veh_list1[i][3], veh_list1[i-1][3], veh_list1[i-2][3], veh_list1[i][2], veh_list1[i-1][2], veh_list1[i-2][2], veh_list1[i][6])
        else:
            acc = IDM_HV(veh_list1[i][3], 0, veh_list1[i][2] + 1000, veh_list1[i][2], veh_list1[i][6])
        if i != len(veh_list1) - 1:
            if (veh_list1[i+1][5] == 0):
                acc_fol = IDM_HV(veh_list1[i + 1][3], veh_list1[i][3], veh_list1[i][2], veh_list1[i + 1][2], veh_list1[i+1][6])
            else:
                if i == 0:
                    acc_fol = IDM_HV(veh_list1[i + 1][3], veh_list1[i][3], veh_list1[i][2], veh_list1[i + 1][2], veh_list1[i+1][6], tao=0)
                elif i > 0:
                    if (veh_list1[i-1][2] - veh_list1[i+1][2]) > DSRC:
                        acc_fol = IDM_HV(veh_list1[i + 1][3], veh_list1[i][3], veh_list1[i][2], veh_list1[i + 1][2], veh_list1[i+1][6]
                                         ,tao=0)
                    else:
                        acc_fol = IDM_CAV(veh_list1[i+1][3], veh_list1[i][3], veh_list1[i-1][3], veh_list1[i+1][2], veh_list1[i][2], veh_list1[i-1][2], veh_list1[i+1][6])
            if i != 0:
                if (veh_list1[i+1][5] == 0):
                    acc_fol_ = IDM_HV(veh_list1[i + 1][3], veh_list1[i - 1][3], veh_list1[i - 1][2], veh_list1[i + 1][2], veh_list1[i + 1][6])
                else:
                    if i == 1:
                        acc_fol_ = IDM_HV(veh_list1[i + 1][3], veh_list1[i - 1][3], veh_list1[i - 1][2],
                                          veh_list1[i + 1][2], veh_list1[i + 1][6], tao=0)
                    elif i > 1:
                        if (veh_list1[i - 2][2] - veh_list1[i + 1][2]) > DSRC:
                            acc_fol_ = IDM_HV(veh_list1[i + 1][3], veh_list1[i - 1][3], veh_list1[i - 1][2],
                                              veh_list1[i + 1][2],veh_list1[i + 1][6], tao=0)
                        else:
                            acc_fol_ = IDM_CAV(veh_list1[i+1][3], veh_list1[i-1][3], veh_list1[i-2][3], veh_list1[i+1][2], veh_list1[i-1][2], veh_list1[i-2][2],veh_list1[i+1][6])
            else:
                acc_fol_ = IDM_HV(veh_list1[i + 1][3], 0, veh_list1[i + 1][2] + 1000, veh_list1[i + 1][2], veh_list1[i + 1][6])
        else:
            acc_fol = 0
            acc_fol_ = 0
        if acc_ - acc + p * (acc_lag_ - acc_lag + acc_fol_ - acc_fol) > 0.1 + a_bias:
            change.append(veh_list1[i])
    return change


def MLC(veh_list1, veh_list2, a_bias, limit_pos, mandatory_pos_now, target_lane, mandatory_pos_change='no', limit_route=1):
    s0 = 2
    tao = 0.4
    veh_len = 5
    change = []
    judge = 0
    T = 1
    DSRC = 100
    for i in range(len(veh_list1)):
        if (veh_list1[i][2] <= limit_pos) | (veh_list1[i][7] != limit_route):
            continue
        pos_range = (veh_list1[i][2] - veh_len - s0 - veh_list1[i][3] * (tao+T) * (1 - veh_list1[i][5]),
                     veh_list1[i][2] + veh_len + s0 + veh_list1[i][3] * (tao+T) * (1 - veh_list1[i][5]))
        for j in range(len(veh_list2)):
            judge = 0
            if (veh_list2[j][2] >= pos_range[0]) & (veh_list2[j][2] <= pos_range[1]):
                judge = 1
                break
        if judge == 1:
            continue
        lead = 'no_lead'
        for k in range(len(veh_list2)):
            s_l = veh_list2[k][2] - veh_list1[i][2]
            if s_l > 0:
                lead = k
            else:
                break
        if lead == 'no_lead':
            if len(veh_list2) == 0:
                acc_lag_ = 0
            else:
                if veh_list2[0][5] == 0:
                    acc_lag_ = IDM_HV(veh_list2[0][3], veh_list1[i][3], veh_list1[i][2], veh_list2[0][2], veh_list2[0][6])
                else:
                    acc_lag_ = IDM_HV(veh_list2[0][3], veh_list1[i][3], veh_list1[i][2], veh_list2[0][2], veh_list2[0][6], tao=0)
        elif lead == len(veh_list2) - 1:
            acc_lag_ = 0
        else:
            if (veh_list2[lead + 1][5] == 0):
                acc_lag_ = IDM_HV(veh_list2[lead + 1][3], veh_list1[i][3], veh_list1[i][2], veh_list2[lead + 1][2], veh_list2[lead + 1][6])
            else:
                if lead == 'no_lead':
                    acc_lag_ = IDM_HV(veh_list2[lead + 1][3], veh_list1[i][3], veh_list1[i][2], veh_list2[lead + 1][2], veh_list2[lead + 1][6]
                                      , tao=0)
                else:
                    if (veh_list2[lead][2] - veh_list2[lead + 1][2]) > DSRC:
                        acc_lag_ = IDM_HV(veh_list2[lead + 1][3], veh_list1[i][3], veh_list1[i][2],
                                          veh_list2[lead + 1][2], veh_list2[lead + 1][6], tao=0)
                    else:
                        acc_lag_ = IDM_CAV(veh_list2[lead + 1][3], veh_list1[i][3], veh_list2[lead][3],
                                           veh_list2[lead + 1][2], veh_list1[i][2], veh_list2[lead][2], veh_list2[lead + 1][6])
        if acc_lag_ < -2:
            continue
        if lead != 'no_lead':
            if (veh_list1[i][5] == 0):
                acc_ = IDM_HV(veh_list1[i][3], veh_list2[lead][3], veh_list2[lead][2], veh_list1[i][2], change_lane_identify(target_lane, veh_list1[i][2]))
            else:
                if lead == 0:
                    acc_ = IDM_HV(veh_list1[i][3], veh_list2[lead][3], veh_list2[lead][2], veh_list1[i][2], change_lane_identify(target_lane, veh_list1[i][2]), tao=0)
                elif lead>0:
                    if (veh_list2[lead - 1][2] - veh_list1[i][2]) > DSRC:
                        acc_ = IDM_HV(veh_list1[i][3], veh_list2[lead][3], veh_list2[lead][2], veh_list1[i][2], change_lane_identify(target_lane, veh_list1[i][2]), tao=0)
                    else:
                        acc_ = IDM_CAV(veh_list1[i][3], veh_list2[lead][3], veh_list2[lead - 1][3], veh_list1[i][2],
                                       veh_list2[lead][2], veh_list2[lead - 1][2], change_lane_identify(target_lane, veh_list1[i][2]))
        else:
            ##这块问题较大
            if veh_list1[i][6] == 2.1:
                acc_ = max(-1.5, IDM_HV(veh_list1[i][3], 0, mandatory_pos_change, veh_list1[i][2], change_lane_identify(target_lane, veh_list1[i][2])))
            else:
                acc_ = IDM_HV(veh_list1[i][3], 0, veh_list1[i][2]+1000, veh_list1[i][2], change_lane_identify(target_lane, veh_list1[i][2]))
        if i != 0:
            if (veh_list1[i][5] == 0):
                acc = IDM_HV(veh_list1[i][3], veh_list1[i - 1][3], veh_list1[i - 1][2], veh_list1[i][2], veh_list1[i][6])
            else:
                if i == 1:
                    acc = IDM_HV(veh_list1[i][3], veh_list1[i - 1][3], veh_list1[i - 1][2], veh_list1[i][2], veh_list1[i][6], tao=0)
                elif i>0:
                    if (veh_list1[i - 2][2] - veh_list1[i][2]) > DSRC:
                        acc = IDM_HV(veh_list1[i][3], veh_list1[i - 1][3], veh_list1[i - 1][2], veh_list1[i][2], veh_list1[i][6], tao=0)
                    else:
                        acc = IDM_CAV(veh_list1[i][3], veh_list1[i - 1][3], veh_list1[i - 2][3], veh_list1[i][2],
                                      veh_list1[i - 1][2], veh_list1[i - 2][2], veh_list1[i][6])
        else:
            acc = max(-1.5, IDM_HV(veh_list1[i][3], 0, mandatory_pos_now, veh_list1[i][2], veh_list1[i][6]))
        if acc_ - acc > 0.1 + a_bias:
            change.append(veh_list1[i])
    return change


def veh_change(change_list, veh_list1, veh_list2, lane_id):
    for i in range(len(change_list)):
        change_list[i][6] = lane_id
    veh_list2.extend(change_list)
    veh_list1_new = []
    for i in range(len(veh_list1)):
        if veh_list1[i] not in change_list:
            veh_list1_new.append(veh_list1[i])
    return veh_list1_new, veh_list2

def lane_identify(veh_list, connect_pos, lane_id):
    veh_list_1 = []
    veh_list_2 = []
    for i in range(len(veh_list)):
        if veh_list[i][2] > connect_pos:
            veh_list[i][6] = lane_id + 0.2
            veh_list_2.append(veh_list[i])
        else:
            veh_list[i][6] = lane_id + 0.1
            veh_list_1.append(veh_list[i])
    return veh_list_1, veh_list_2

def IDM_new(veh_list, ramp_limit, route_limit, limit_pos=10000, connect_list=[]):
    DSRC = 100
    for i in range(len(veh_list)):
        if i == 0:
            if ramp_limit:
                if veh_list[i][5] == 0:
                    acc = IDM_HV(veh_list[i][3], 0, limit_pos, veh_list[i][2], veh_list[i][6])
                else:
                    acc = IDM_HV(veh_list[i][3], 0, limit_pos, veh_list[i][2], veh_list[i][6], tao=0)
            elif route_limit &(veh_list[i][7] == 1):
                if veh_list[i][5] == 0:
                    acc = IDM_HV(veh_list[i][3], 0, limit_pos, veh_list[i][2], veh_list[i][6])
                else:
                    acc = IDM_HV(veh_list[i][3], 0, limit_pos, veh_list[i][2], veh_list[i][6], tao=0)
                acc = max(acc, -1.5)
            else:
                if (veh_list[i][6] == 1.1) | (veh_list[i][6] == 2.1):
                    if (veh_list[i][5] == 0) & (len(connect_list) > 0):
                        acc = IDM_HV(veh_list[i][3], connect_list[-1][3], connect_list[-1][2], veh_list[i][2], veh_list[i][6])
                    elif (veh_list[i][5] == 1) & (len(connect_list) > 0):
                        if len(connect_list) == 1:
                            acc = IDM_HV(veh_list[i][3], connect_list[-1][3], connect_list[-1][2], veh_list[i][2], veh_list[i][6], tao=0)
                        elif len(connect_list) > 1:
                            if (connect_list[-2][2]-veh_list[i][2]) > DSRC:
                                acc = IDM_HV(veh_list[i][3], connect_list[-1][3], connect_list[-1][2], veh_list[i][2], veh_list[i][6]
                                             , tao=0)
                            else:
                                acc = IDM_CAV(veh_list[i][3], connect_list[-1][3], connect_list[-2][3], veh_list[i][2], connect_list[-1][2], connect_list[-2][2], veh_list[i][6])
                    else:
                        acc = IDM_HV(veh_list[i][3], veh_list[i][3], veh_list[i][2] + 1000, veh_list[i][2], veh_list[i][6])
                else:
                    acc = IDM_HV(veh_list[i][3], veh_list[i][3], veh_list[i][2]+1000, veh_list[i][2], veh_list[i][6])
        elif (i == 1) & (veh_list[i][5] == 1):
            acc = IDM_HV(veh_list[i][3], veh_list[i-1][3], veh_list[i-1][2], veh_list[i][2], veh_list[i][6], tao=0)
        elif (veh_list[i][5] == 1) & (veh_list[i-2][2]-veh_list[i][2] > DSRC):
            acc = IDM_HV(veh_list[i][3], veh_list[i - 1][3], veh_list[i - 1][2], veh_list[i][2], veh_list[i][6], tao=0)
        elif veh_list[i][5] == 1:
            acc = IDM_CAV(veh_list[i][3], veh_list[i-1][3], veh_list[i-2][3], veh_list[i][2], veh_list[i-1][2], veh_list[i-2][2], veh_list[i][6])
        else:
            acc = IDM_HV(veh_list[i][3], veh_list[i - 1][3], veh_list[i - 1][2], veh_list[i][2], veh_list[i][6])
        veh_list[i][4] = acc
    return veh_list



# def speed_limit(lane_id,pos,detect_speed):
#     if lane_id == 0:
#         if pos<8500:
#             # 40
#             v0 = 11.11
#         else:
#             # 60
#             v0 = 16.67
#     elif lane_id == 1.1:
#         if  pos<=5000:
#             # 100
#             v0 = 27.78
#         elif 5000<pos<=7000:
#             # 80
#             v0 = 22.22
#         elif 7000<pos<=9000:
#             # 60
#             v0 = 16.67
#         elif 9000<pos<=11000:
#             # 80
#             v0 = 22.22
#         elif 11000 < pos:
#             # 60
#             v0 = 16.67
#     elif lane_id == 1.2:
#         v0 = 27.78
#     elif lane_id == 2.1:
#         if pos <= 9000:
#             # 100
#             v0 = 27.78
#         elif 9000 < pos <= 11000:
#             # 80
#             v0 = 22.22
#         elif 11000 < pos:
#             # 60
#             v0 = 16.67
#     elif lane_id == 2.2:
#         v0 = 27.78
#     else:
#         if 14000 > pos > 13500:
#             # 60
#             v0 = 16.67
#         else:
#             # 40
#             v0 = 11.11
#     return v0


def speed_limit(lane_id,pos):
    if lane_id == 0:
        if pos < 7500:
            # 40
            v0 = 11.11
        else:
            # 60
            v0 = 16.67
    elif lane_id == 1.1:
        if pos <= 5000:
            # 100
            v0 = 27.78
        elif 5000 < pos <= 7000:
            # 80
            v0 = 22.22
        elif 7000 < pos <= 9000:
            # 60
            v0 = 16.67
        elif 9000 < pos <= 11000:
            # 80
            v0 = 22.22
        elif 11000 < pos:
            # 60
            v0 = 16.67
    elif lane_id == 1.2:
        v0 = 27.78
    elif lane_id == 2.1:
        if pos <= 9000:
            # 100
            v0 = 27.78
        elif 9000 < pos <= 11000:
            # 80
            v0 = 22.22
        elif 11000 < pos:
            # 60
            v0 = 16.67
    elif lane_id == 2.2:
        v0 = 27.78
    else:
        if 13000 > pos > 12000:
            # 60
            v0 = 16.67
        else:
            # 40
            v0 = 11.11
    return v0


def IDM(veh_list, limit, limit_pos=10000):
    alpha = 3
    v0 = 33
    pos1 = 0
    pos2 = 0
    speed1 = 0
    speed2 = 0
    s0 = 2
    beta = 1.5
    T = 1
    tao = 0.4
    veh_lengh = 5
    DSRC = 100
    for i in range(len(veh_list)):
        # 自由流
        if i == 0:
            if limit:
                s_star = s0 + max(0, T * veh_list[i][3] - veh_list[i][3] * (-veh_list[i][3]) / 2 / (
                    alpha * beta) ** 0.5)
                veh_list[i][4] = alpha * (
                        1 - (veh_list[i][3] / v0) ** 4 - (s_star / (limit_pos - veh_list[i][2] - veh_lengh)) ** 2)
            else:
                veh_list[i][4] = alpha * (1 - (veh_list[i][3] / v0) ** 4)
            pos1 = pos2 = veh_list[i][2]
            speed1 = speed2 = veh_list[i][3]
        else:
            if veh_list[i][5] == 0:
                s_star = s0 + tao * veh_list[i][3]+max(0, T*veh_list[i][3] - veh_list[i][3] * (speed1 - veh_list[i][3]) / 2 / (
                    alpha * beta) ** 0.5)
                veh_list[i][4] = alpha * (
                        1 - (veh_list[i][3] / v0) ** 4 - (s_star / (pos1 - veh_list[i][2] - veh_lengh)) ** 2)
                pos2 = pos1
                pos1 = veh_list[i][2]
                speed2 = speed1
                speed1 = veh_list[i][3]
            else:
                if (pos2-veh_list[i][2] > DSRC) | (i == 1):
                    gamma1 = 1
                    gamma2 = 0
                else:
                    gamma1 = 0.7
                    gamma2 = 0.3
                s_star = s0 + max(0, T * veh_list[i][3] - veh_list[i][3] * (
                    gamma1 * (speed1-veh_list[i][3]) + gamma2 * (speed2-speed1)) / 2 / (
                             alpha * beta) ** 0.5)
                veh_list[i][4] = alpha * (
                    1 - (veh_list[i][3] / v0) ** 4 - (
                        s_star / (gamma1 * (pos1 - veh_list[i][2]-veh_lengh) + gamma2 * (pos2-pos1 - veh_lengh))) ** 2)
                pos2 = pos1
                pos1 = veh_list[i][2]
                speed2 = speed1
                speed1 = veh_list[i][3]
    return veh_list


def veh_move(veh_list):
    for i in range(len(veh_list)):
        veh_list[i][2] += veh_list[i][3] * 0.1 + veh_list[i][4] * 0.5 * 0.01
        veh_list[i][3] += veh_list[i][4] * 0.1
        veh_list[i][1] += 1
    return veh_list


def list_sort(veh_list):
    if len(veh_list) != 0:
        veh_ar = np.array(veh_list)
        veh_ar = veh_ar[veh_ar[:, 2].argsort()[::-1]]
        veh_list = veh_ar.tolist()
    return veh_list


def detector_run(detector, id, veh_list, pos, length=5):
    for i in range(len(veh_list)):
        repeat = 0
        if (veh_list[i][2] >= pos) & (veh_list[i][2] <= pos + length):
            for j in range(len(veh_list[i][8])):
                if veh_list[i][8][j][0] == id:
                    repeat += 1
            if repeat == 0:
                veh_list[i][8].append((id, veh_list[i][1]))
                detector.append(veh_list[i])
        if veh_list[i][2] < pos:
            break
    return detector, veh_list


# def detector_output(detector, volume_list, speed_list, times, time_interval=600):
#     id_record = []
#     speed = []
#     if times % time_interval == 0:
#         if len(detector) != 0:
#             for i in range(len(detector)):
#                 if detector[i][0] not in id_record:
#                     id_record.append(detector[i][0])
#                     speed.append(detector[i][3])
#             volume_list.append(len(id_record))
#             speed_list.append(np.mean(speed))
#             detector = []
#         else:
#             volume_list.append(0)
#             speed_list.append(0)
#     return volume_list, speed_list, detector

def detector_output(detector, volume_list, speed_list, times, time_interval=600):
    speed = []
    if times % time_interval == 0:
        if len(detector) != 0:
            for i in range(len(detector)):
                speed.append(detector[i][3])
            volume_list.append(len(detector))
            speed_list.append(np.mean(speed))
            detector = []
        else:
            volume_list.append(0)
            speed_list.append(0)
    return volume_list, speed_list, detector

def lane_connect(veh_list_1, veh_list_2, connect_pos):
    veh_list_1_new = []
    for i in range(len(veh_list_1)):
        if veh_list_1[i][2] > connect_pos:
            veh_list_2.append(veh_list_1[i])
        else:
            veh_list_1_new.append(veh_list_1[i])
    return veh_list_1_new, veh_list_2

def end_pass_time(veh_list, pass_time):
    for i in range(len(veh_list)):
        if len(veh_list[i][8]) != 0:
            pass_time.append(veh_list[i][8])
    return pass_time

ramp_out_range = (12000, 13000)
ramp_out_2to1_range = (11000, 13000)
ramp_in_range = (7500, 8000)
noDLC_range = (11000, 14000)
global_p = 0.2
lane_len = 16000
ramp_out_len = 16000
global_CAV_p = 1
global_ramp_out_p = 0.2
lane_add_p = 0.06
ramp_in_add_p = 0.03
add_v = 0
sim_time = 18000

ramp_in = []
ramp_out = []
lane1_1 = []
lane2_1 = []
lane1_2 = []
lane2_2 = []

disappear1 = 0
disappear2 = 0
disappear3 = 0
# ________________________________________________________________________
detector1_1 = []
volume1_1 = []
speed1_1 = []
detector2_1 = []
volume2_1 = []
speed2_1 = []
detector3_1 = []
volume3_1 = []
speed3_1 = []
detector4_1 = []
volume4_1 = []
speed4_1 = []

detector1_2 = []
volume1_2 = []
speed1_2 = []
detector2_2 = []
volume2_2 = []
speed2_2 = []
detector3_2 = []
volume3_2 = []
speed3_2 = []
detector4_2 = []
volume4_2 = []
speed4_2 = []
sign = 0
# ________________________________________________________________________
total_list1 = []
total_list2 = []
total_list_out = []
num = 0
total = []
pass_time_list = []
for t in range(sim_time):
    lane1 = []
    lane2 = []
    # if sign == 1:
    #     break
    # 车辆换道
    change_ramp_out = MLC(lane1_1, ramp_out, 0, ramp_out_range[0], ramp_out_range[1], 3)
    change_ramp_out_2to1 = MLC(lane2_1, lane1_1, 0, ramp_out_2to1_range[0], ramp_out_2to1_range[1], 1, mandatory_pos_change=ramp_out_range[1])
    change_ramp_in = MLC(ramp_in, lane1_1, 0, ramp_in_range[0], ramp_in_range[1], 1)
    change1 = LC(lane1_1, lane1_2, lane2_1, lane2_2, global_p, -0.2, noDLC_range, 2)
    change2 = LC(lane2_1, lane2_2, lane1_1, lane1_2, global_p, 0.2, noDLC_range, 1)
    #
    # change1_new = []
    # change2_new = []
    # for i in range(len(change1)):
    #     if change1[i] not in change_ramp_out:
    #         change1_new.append(change1[i])
    # for i in range(len(change2)):
    #     if change2[i] not in change_ramp_out_2to1:
    #         change2_new.append(change2[i])
    change2.extend(change_ramp_out_2to1)
    lane1_1, ramp_out = veh_change(change_ramp_out, lane1_1, ramp_out, 3)
    ramp_in, lane1_1 = veh_change(change_ramp_in, ramp_in, lane1_1, 1.1)
    lane1.extend(lane1_2)
    lane1.extend(lane1_1)
    lane2.extend(lane2_2)
    lane2.extend(lane2_1)
    lane2, lane1 = veh_change(change2, lane2, lane1, 1)
    lane1, lane2 = veh_change(change1, lane1, lane2, 2)
    lane1_1, lane1_2 = lane_identify(lane1, ramp_out_range[1], 1)
    lane2_1, lane2_2 = lane_identify(lane2, ramp_out_2to1_range[1], 2)


    ####
    # 车辆排序
    lane1_1 = list_sort(lane1_1)
    lane2_1 = list_sort(lane2_1)
    lane1_2 = list_sort(lane1_2)
    lane2_2 = list_sort(lane2_2)
    ramp_out = list_sort(ramp_out)
    # 车辆运行

    lane1_1 = IDM_new(lane1_1, False, True, limit_pos=ramp_out_range[1], connect_list=lane1_2)
    lane2_1 = IDM_new(lane2_1, False, True, limit_pos=ramp_out_2to1_range[1], connect_list=lane2_2)
    lane1_2 = IDM_new(lane1_2, False, False)
    lane2_2 = IDM_new(lane2_2, False, False)
    ramp_in = IDM_new(ramp_in, True, False, limit_pos=ramp_in_range[1])
    ramp_out = IDM_new(ramp_out, False, False)

    lane1_1, lane1_2 = lane_connect(lane1_1, lane1_2, ramp_out_range[1])
    lane2_1, lane2_2 = lane_connect(lane2_1, lane2_2, ramp_out_2to1_range[1])

    # for i in range(len(lane1)):
    #     if abs(lane1[i][4]) > 10:
    #         print('error')
    #         sign = 1
    # for i in range(len(lane2)):
    #     if abs(lane2[i][4]) > 10:
    #         print('error')
    #         sign = 1

    lane1_1 = veh_move(lane1_1)
    lane1_2 = veh_move(lane1_2)
    lane2_1 = veh_move(lane2_1)
    lane2_2 = veh_move(lane2_2)
    ramp_in = veh_move(ramp_in)
    ramp_out = veh_move(ramp_out)
    #_________________________________________
    detector1_1, lane1_1 = detector_run(detector1_1, 1, lane1_1, 2500)
    detector2_1, lane1_1 = detector_run(detector2_1, 2, lane1_1, 6000)
    detector3_1, lane1_1 = detector_run(detector3_1, 3, lane1_1, 10000)
    detector4_1, lane1_2 = detector_run(detector4_1, 4, lane1_2, 15000)
    detector1_2, lane2_1 = detector_run(detector1_2, 5, lane2_1, 2500)
    detector2_2, lane2_1 = detector_run(detector2_2, 6, lane2_1, 6000)
    detector3_2, lane2_1 = detector_run(detector3_2, 7, lane2_1, 10000)
    detector4_2, lane2_2 = detector_run(detector4_2, 8, lane2_2, 15000)
    #_________________________________________
    # 车辆消失
    lane1_2, disappear1, pass_time_list = veh_disappear(lane1_2, lane_len, disappear1, pass_time_list)
    lane2_2, disappear2, pass_time_list = veh_disappear(lane2_2, lane_len, disappear2, pass_time_list)
    ramp_out, disappear3, pass_time_list = veh_disappear(ramp_out, ramp_out_len, disappear3, pass_time_list)
    # 添加车辆
    new_veh1, num = add_car(1.1, t, num, lane_add_p, global_CAV_p, add_v, lane1_1, global_ramp_out_p)
    if len(new_veh1) != 0:
        lane1_1.append(new_veh1)
    new_veh2, num = add_car(2.1, t, num, lane_add_p, global_CAV_p, add_v, lane2_1, global_ramp_out_p)
    if len(new_veh2) != 0:
        lane2_1.append(new_veh2)
    new_veh_ramp_in, num = add_car(0, t, num, ramp_in_add_p, global_CAV_p, add_v, ramp_in, global_ramp_out_p)
    if len(new_veh_ramp_in) != 0:
        ramp_in.append(new_veh_ramp_in)

    #_______________________________________________________________________________
    volume1_1, speed1_1, detector1_1 = detector_output(detector1_1, volume1_1, speed1_1, t)
    volume2_1, speed2_1, detector2_1 = detector_output(detector2_1, volume2_1, speed2_1, t)
    volume3_1, speed3_1, detector3_1 = detector_output(detector3_1, volume3_1, speed3_1, t)
    volume4_1, speed4_1, detector4_1 = detector_output(detector4_1, volume4_1, speed4_1, t)
    volume1_2, speed1_2, detector1_2 = detector_output(detector1_2, volume1_2, speed1_2, t)
    volume2_2, speed2_2, detector2_2 = detector_output(detector2_2, volume2_2, speed2_2, t)
    volume3_2, speed3_2, detector3_2 = detector_output(detector3_2, volume3_2, speed3_2, t)
    volume4_2, speed4_2, detector4_2 = detector_output(detector4_2, volume4_2, speed4_2, t)
    # _______________________________________________________________________________
    print(t)
#     lane1 = []
#     lane2 = []
#     lane1.extend(lane1_2)
#     lane1.extend(lane1_1)
#     lane2.extend(lane2_2)
#     lane2.extend(lane2_1)
#     total_list1.extend(np.array(lane1).flatten().tolist())
#     total_list2.extend(np.array(lane2).flatten().tolist())
#     total_list_out.extend(np.array(ramp_out).flatten().tolist())
#
# total_ar1 = np.array(total_list1).reshape(-1, 9)
# total_ar2 = np.array(total_list2).reshape(-1, 9)
# total_ar_out = np.array(total_list_out).reshape(-1, 9)

pass_time_list = end_pass_time(lane1_1, pass_time_list)
pass_time_list = end_pass_time(lane1_2, pass_time_list)
pass_time_list = end_pass_time(lane2_1, pass_time_list)
pass_time_list = end_pass_time(lane2_2, pass_time_list)
pass_time_list = end_pass_time(ramp_in, pass_time_list)
pass_time_list = end_pass_time(ramp_out, pass_time_list)
print(volume1_1, speed1_1)
print(volume2_1, speed2_1)
print(volume3_1, speed3_1)
print(volume4_1, speed4_1)
print(volume1_2, speed1_2)
print(volume2_2, speed2_2)
print(volume3_2, speed3_2)
print(volume4_2, speed4_2)
print(pass_time_list)

text1 = pd.DataFrame(volume1_1)
text2 = pd.DataFrame(speed1_1)
text1.columns = ['quantity']
text2.columns = ['speed']
writer1 = pd.ExcelWriter('main1.xls')
text1.to_excel(writer1, sheet_name='sheet1')
text2.to_excel(writer1, sheet_name='sheet2')
writer1.save()

text3 = pd.DataFrame(volume2_1)
text4 = pd.DataFrame(speed2_1)
text3.columns = ['quantity']
text4.columns = ['speed']
writer2 = pd.ExcelWriter('main2.xls')
text3.to_excel(writer2, sheet_name='sheet1')
text4.to_excel(writer2, sheet_name='sheet2')
writer2.save()

text5 = pd.DataFrame(volume3_1)
text6 = pd.DataFrame(speed3_1)
text5.columns = ['quantity']
text6.columns = ['speed']
writer3 = pd.ExcelWriter('main3.xls')
text5.to_excel(writer3, sheet_name='sheet1')
text6.to_excel(writer3, sheet_name='sheet2')
writer3.save()

text7 = pd.DataFrame(volume4_1)
text8 = pd.DataFrame(speed4_1)
text7.columns = ['quantity']
text8.columns = ['speed']
writer4 = pd.ExcelWriter('main4.xls')
text7.to_excel(writer4, sheet_name='sheet1')
text8.to_excel(writer4, sheet_name='sheet2')
writer4.save()

text9 = pd.DataFrame(volume1_2)
text10 = pd.DataFrame(speed1_2)
text9.columns = ['quantity']
text10.columns = ['speed']
writer5 = pd.ExcelWriter('main5.xls')
text9.to_excel(writer5, sheet_name='sheet1')
text10.to_excel(writer5, sheet_name='sheet2')
writer5.save()

text11 = pd.DataFrame(volume2_2)
text12 = pd.DataFrame(speed2_2)
text11.columns = ['quantity']
text12.columns = ['speed']
writer6 = pd.ExcelWriter('main6.xls')
text11.to_excel(writer6, sheet_name='sheet1')
text12.to_excel(writer6, sheet_name='sheet2')
writer6.save()

text13 = pd.DataFrame(volume3_2)
text14 = pd.DataFrame(speed3_2)
text13.columns = ['quantity']
text14.columns = ['speed']
writer7 = pd.ExcelWriter('main7.xls')
text13.to_excel(writer7, sheet_name='sheet1')
text14.to_excel(writer7, sheet_name='sheet2')
writer7.save()

text15 = pd.DataFrame(volume4_2)
text16 = pd.DataFrame(speed4_2)
text15.columns = ['quantity']
text16.columns = ['speed']
writer8 = pd.ExcelWriter('main8.xls')
text15.to_excel(writer8, sheet_name='sheet1')
text16.to_excel(writer8, sheet_name='sheet2')
writer8.save()