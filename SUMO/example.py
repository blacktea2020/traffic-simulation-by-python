import net
import traci
import rou
import environment
import sumocfg as sc
import run

#创建nod.xml，道路长度为1005.
net.Simple.nod(1005)
#创建edg.xml，道路限速为100.
net.Simple.edg(100)
#将nod.xml,edg.xml合成为net.xml
net.Simple.netconvert(r'D:\APPs\SUMO\bin')
#生成sumocfg文件，仿真步长为0.1.
sc.cfg(0.1)
#检查SUMO_HOME环境
sumoCmd = environment.main()
#开始仿真
traci.start(sumoCmd)
#实例化车辆对象，车长为4m。
veh0 = rou.HeadwayDistribution(5)
#车辆服从均匀分布，数量为10，初始间距为100m，初始速度为20m/s。
veh0.uniform(20,20,10)
#实例化仿真对象，仿真时长1000s，周期性边界，输出DataFrame
run0 = run.Run(1000,0.1,1000,0,output_df=True)
#IDM模型参数alpha=3，beta=2，v0=50，s0=5，T=3
run0.FVD(3,2,20,15)
#结束仿真并关闭SUMO.GUI
traci.close()

