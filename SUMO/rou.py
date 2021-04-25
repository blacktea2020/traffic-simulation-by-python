import traci
import numpy as np
import random
class Initialization:
    def __init__(self,l):
        self.l = l
        traci.route.add(routeID='route0', edges=['edgeR-1-2'])
        traci.vehicletype.copy('DEFAULT_VEHTYPE','Vtype0')
        traci.vehicletype.setLength('Vtype0',l)
class TimeDistribution(Initialization):
    def uniform(self,N,t):
        veh_id = np.arange(0,N)
        for n in veh_id:
            traci.vehicle.add(routeID='route0', typeID='Vtype0',vehID=str(n),depart = n*t,departPos=0)
    def random_distribution(self,N,start,end):
        veh_id = np.arange(0,N)
        veh_time = np.array(random.sample(range(start*10,end*10), N))/10
        veh_time = np.sort(veh_time)
        for n in veh_id:
            traci.vehicle.add(routeID='route0', typeID='Vtype0',vehID=str(n),depart = veh_time[n],departPos=0)
        
class HeadwayDistribution(Initialization):
    def uniform(self,N,h,speed):
        veh_id = np.arange(0,N)
        for n in veh_id:
            traci.vehicle.add(routeID='route0', typeID='Vtype0',vehID=str(n),depart = 0 ,departPos=n*h,departSpeed=speed)
    def normal(self,N,mu,sigma,speed):
        veh_id = np.arange(0,N)
        veh_headway = np.random.normal(mu,sigma,N)
        veh_headway[veh_headway<self.l] == self.l
        veh_position = np.cumsum(veh_headway)
        for n in veh_id:
            traci.vehicle.add(routeID='route0', typeID='Vtype0',vehID=str(n),depart = 0 ,departPos=veh_position[n],departSpeed=speed)