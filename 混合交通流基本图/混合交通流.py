import numpy as np
import matplotlib.pyplot as plt
list_num = 6
veh_length = 5
veh_num = 100
lane_length = 2000
sim_time = 20000
time_step = 0.1
gamma0 = 0.7
gamma1 = 0.3
veh_ar = np.zeros([veh_num, list_num])
veh_ar[:, 0] = np.arange(0, veh_num)
veh_ar[:, 2] = lane_length * veh_ar[:, 0] / veh_num + np.random.normal(0, 2, veh_num)
veh_ar[:, 3] = 15 + np.random.normal(0, 2, veh_num)
veh_ar[:,5] = np.random.choice([0,1],veh_num,p=[0,1])


def headway_identify(ar):
    pos0 = ar[:, 2]
    laggest_veh_pos = pos0[0] + lane_length
    pos1 = np.append(pos0, laggest_veh_pos)
    pos1 = pos1[1:]
    headway = pos1 - pos0
    for i in range(len(headway)):
        if ar[i][5] == 1:
            if i == len(headway)-1:
                headway[i] = gamma0*headway[i]+gamma1*headway[0]
            else:
                headway[i] = gamma0*headway[i]+gamma1*headway[i+1]
    return headway


def delta_v_identift(ar):
    speed0 = ar[:, 3]
    laggest_veh_speed = speed0[0]
    speed1 = np.append(speed0, laggest_veh_speed)
    speed1 = speed1[1:]
    delta_v = speed1 - speed0
    for i in range(len(delta_v)):
        if ar[i][5] == 1:
            if i == len(delta_v)-1:
                delta_v[i] = gamma0*delta_v[i]+gamma1*delta_v[0]
            else:
                delta_v[i] = gamma0*delta_v[i]+gamma1*delta_v[i+1]
    return delta_v


def IDM(veh_speed, veh_headway, veh_delta_v, alpha, beta, v0, s0, T):
    spacing = veh_headway - veh_length
    s_star = s0 + T * veh_speed - ((veh_speed * veh_delta_v) / 2 / (alpha * beta) ** 0.5)
    veh_acc = alpha * (1 - (veh_speed / v0) ** 4 - (s_star / spacing) ** 2)
    return veh_acc


def speed_update(veh_speed, veh_acc):
    speed = veh_speed + veh_acc * time_step
    return speed


def pos_update(veh_pos, veh_spped, veh_acc):
    pos = (veh_pos + veh_spped * time_step + 0.5 * veh_acc * time_step ** 2) % lane_length
    return pos


veh_total = []
for t in range(sim_time):
    headway = headway_identify(veh_ar)
    delta_v = delta_v_identift(veh_ar)
    acc = IDM(veh_ar[:, 3], headway, delta_v, 1, 1.5, 30, 2, 1)
    speed = speed_update(veh_ar[:, 3], acc)
    pos = pos_update(veh_ar[:, 2], veh_ar[:, 3], acc)
    if (t>500)&(t<=505):
        acc[0]+=0.1
    if (t>505)&(t<=510):
        acc[0]-=0.1
    veh_ar[:, 4] = acc
    veh_ar[:, 3] = speed
    veh_ar[:, 2] = pos
    veh_ar[:, 1] += 1
    veh_ar = veh_ar[veh_ar[:, 2].argsort()]
    if (t > 500)&(t<2000):
        veh_total.extend(veh_ar.flatten().tolist())
veh_total = np.array(veh_total).reshape(-1, list_num)
plt.scatter(veh_total[:,2],veh_total[:,1],s=0.3)
plt.show()
