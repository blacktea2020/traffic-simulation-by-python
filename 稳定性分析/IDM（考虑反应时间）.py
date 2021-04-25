# ar[0]:车辆ID
# ar[1]:帧数
# ar[2]:车辆位置
# ar[3]:车辆速度
# ar[4]:车辆加速度
# ar[5]:上时刻速度
# ar[6]:上时刻加速度
# ar[7]:与前车间距
# ar[8]:与前车速度差
# ar[9]:tao时刻前的速度
# ar[10]:tao时刻前的车间距
# ar[11]:tao时刻前的速度差
# 导入包
import numpy as np
import matplotlib.pyplot as plt
import time

np.set_printoptions(threshold=np.inf)
np.set_printoptions(suppress=True)
# np.set_printoptions(precision=8)
# 遥控器
v0 = 100 / 3  # 期望速度
s0 = 2  # 最小间距
beta = 1.5  # 舒适减速度
delta_t = 0.1  # 步长
# alpha = 1  # 最大加速度
l = 5  # 车辆长度
N = 50  # 车辆数
tao = 5


# IDM模型
def IDM(v, s, delta_v, T):
    a = alpha * (1 - (v / v0) ** 4 - (S_star(v, delta_v, T) / s) ** 2)
    return a


def S_star(v, delta_v, T):
    s_star = s0 + T * v - v * delta_v / (2 * (alpha * beta) ** 0.5)
    return s_star


# 初始值
def initialization(ar):
    ar[:, 0] = np.arange(0, N)
    ar[:, 2] = ar[:, 0] * headway0
    ar[:, 3] = np.ones(N, ) * 15
    ar = Spacing_identify(ar)
    ar[:, 9] = np.ones(N, ) * 15
    ar[:, 10] = ar[:, 7]
    return ar


# 确定与前车间距
def Spacing_identify(ar):
    ar = ar[ar[:, 2].argsort()]
    position = ar[:, 2]
    position1 = np.delete(position, 0)
    position2 = np.delete(position, -1)
    position_new = (position1 - position2) - l
    position_new = np.append(position_new, None)
    ar[:, 7] = position_new
    return ar


# 确定速度差
def Speed_identify(ar):
    ar = ar[ar[:, 2].argsort()]
    speed = ar[:, 3]
    speed1 = np.delete(speed, 0)
    speed2 = np.delete(speed, -1)
    speed_new = speed1 - speed2
    speed_new = np.append(speed_new, None)
    ar[:, 8] = speed_new
    return ar


# 加速度更新
def Acc_upgrade():
    ar[:, 6] = ar[:, 4]
    ACC = ar[ar[:, 0] != N - 1]
    ACC[:, 4] = IDM(ACC[:, 9], ACC[:, 10], ACC[:, 11], T)
    ar[ar[:, 0] != N - 1] = ACC

    ar[N - 1][4] = 0


# 速度更新
def Speed_upgrade():
    ar[:, 5] = ar[:, 3]
    ar[:, 3] += 0.5 * (ar[:, 6] + ar[:, 4]) * delta_t


# 位置更新
def Position_upgrade():
    ar[:, 2] += ar[:, 5] * delta_t + 0.5 * ar[:, 6] * delta_t ** 2


def Disturbance(ar, time_left, time, disturbance):
    arr = ar[(ar[:, 1] >= time_left) & (ar[:, 1] < time_left + time) & (ar[:, 0] == N - 1)]
    arr[:, 4] = -1 * disturbance
    ar[(ar[:, 1] >= time_left) & (ar[:, 1] < time_left + time) & (ar[:, 0] == N - 1)] = arr
    arr = ar[(ar[:, 1] >= time_left + time) & (ar[:, 1] < time_left + 2 * time) & (ar[:, 0] == N - 1)]
    arr[:, 4] = disturbance
    ar[(ar[:, 1] >= time_left + time) & (ar[:, 1] < time_left + 2 * time) & (ar[:, 0] == N - 1)] = arr

    # 定义初始值


np.set_printoptions(suppress=True)
# __main__
for alpha in np.linspace(0.1,4,40):
    for T in np.linspace(0.1,4,40):
        s_star0 = S_star(15, 0, T)
        headway0 = (s_star0 ** 2 / (1 - 0.45 ** 4)) ** 0.5 + l
        ar = np.zeros([N, 12])
        ar = initialization(ar)
        start = time.time()
        list0 = []
        list = []
        for i in np.arange(1, 1000):
            ar = Spacing_identify(ar)
            ar = Speed_identify(ar)
            Acc_upgrade()
            Disturbance(ar, 600, 30, 1)
            Speed_upgrade()
            Position_upgrade()
            ar[:, 1] += 1
            list.append(ar)
            arr_list = np.array(list).reshape(-1, 12)
            if i > 5:
                ar[:, 9] = arr_list[arr_list[:, 1] == ar[0, 1] - tao][:, 3]
                ar[:, 10] = arr_list[arr_list[:, 1] == ar[0, 1] - tao][:, 7]
                ar[:, 11] = arr_list[arr_list[:, 1] == ar[0, 1] - tao][:, 8]
        for id in range(0, N):
            ar_id = arr_list[(arr_list[:, 0] == id)&(arr_list[:,1]>=500)]
            minSpeed = ar_id[:, 3].min()
            list0.append(minSpeed)
        list0_arr = 15 - np.array(list0)
        list0_arr = list0_arr.round(8)
        list1_arr = np.sort(list0_arr)
        judge = (list0_arr == list1_arr)
        if not judge.all():
            plt.scatter(alpha, T, c='k', s=0.5)
        end = time.time()
        print(T,alpha)
        print(end - start)
plt.title('String stability region')
plt.xlabel('alpha')
plt.ylabel('T')
plt.show()
# plt.plot(np.arange(0, N), list0_arr)
# plt.title('T = %s,alpha = %s' % (T, alpha))
# plt.show()
print('The End')
