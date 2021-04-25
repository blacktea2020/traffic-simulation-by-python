# 导入包
import numpy as np
import matplotlib.pyplot as plt
import time
np.set_printoptions(threshold=np.inf)
# 遥控器
p = 0.3  # 随机慢化概率
L = 1000  # 道路长度
v_max = 5  # 最大速度
delta_t = 1  # 步长
T = 20000  # 持续时间
l = 1  # 车辆长度


# ar[0]:车辆ID
# ar[1]:帧数
# ar[2]:车辆位置
# ar[3]:车辆速度
# ar[4]:与前车间距


# 初始值
def initialization(n):
    global ar
    N = int(Range[times] * L)
    headway = int(L / N)
    ar = np.zeros([N, 5])
    ar[:, 0] = np.arange(0, N)
    ar[:, 2] = ar[:, 0] * headway
    ar[:, 4] = headway
    return ar


# 确定与前车间距
def Spacing_front_identify():
    global ar
    ar = ar[ar[:, 2].argsort()]
    position = ar[:, 2]
    position1 = np.append(position, position[0] + L)
    position1 = np.delete(position1, 0)
    ar[:, 4] = position1 - position


# 速度更新
def Speed_upgrade():
    # 加速
    speed = ar[ar[:, 3] < v_max]
    speed[:, 3] += 1
    ar[ar[:, 3] < v_max] = speed
    # 减速
    speed = ar[ar[:, 3] > (ar[:, 4] - l)]
    speed[:, 3] = speed[:, 4] - l
    ar[ar[:, 3] > (ar[:, 4] - l)] = speed
    # 随机慢化
    speed = ar[ar[:, 3] > 0]
    speed[:, 3] -= np.random.choice([0, 1], len(speed), p=[2 / 3, 1 / 3])
    ar[ar[:, 3] > 0] = speed


# 位置更新
def Position_upgrade():
    ar[:, 2] = (ar[:, 2] + ar[:, 3]) % L



# 定义初始值
basemap = np.zeros([50,3])
np.set_printoptions(suppress=True)
# __main__
point1 = np.linspace(0, 0.7, 140, endpoint=False)
point2 = np.linspace(0.7, 1, 20)
Range = np.concatenate((point1, point2))
for times in np.arange(1,50):
    ar = initialization(times)  # 第times次仿真
    list0 = []
    N = int(Range[times] * L)
    print('Times:', times)
    start = time.time()
    for i in np.arange(1, 10001):
        ar[:, 1] += 1
        Position_upgrade()
        Spacing_front_identify()
        Speed_upgrade()
        if i>=5000:
            list0.extend(ar.flatten().tolist())

    arr_list = np.array(list0).reshape(-1, 5)
    end = time.time()
    print(end - start)
    speed_mean = arr_list[:, 3].mean()
    Density = N / L
    Flow = Density * speed_mean
    basemap[times][0] = speed_mean
    basemap[times][1] = Density
    basemap[times][2] = Flow
plt.plot(basemap[:,1], basemap[:,2])
plt.show()
print('The End')
