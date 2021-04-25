# -*- coding: utf-8 -*-
"""
Created on Tue Jan 19 00:41:56 2021

@author: 46981
"""
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt


road_network = nx.generators.classic.circulant_graph(25, [1,2])
pos_road_network = nx.circular_layout(road_network)
nx.draw(road_network, pos_road_network, with_labels=False, node_size=30, edge_color='b', alpha=0.3)
plt.show() 
adjacency_road=np.array(nx.adjacency_matrix(road_network).todense())
incidence_road=np.array(nx.incidence_matrix(road_network).todense())

demand_network = nx.random_graphs.barabasi_albert_graph(25,4,seed = 1)
nx.draw(demand_network, pos_road_network, with_labels=False, node_size=30, edge_color='b', alpha=0.3)
plt.show() 
adjacency_demand=np.array(nx.adjacency_matrix(demand_network).todense())
incidence_demand=np.array(nx.incidence_matrix(demand_network).todense())
    
road_list = []
demand_list = []
for i in range(25):
    for j in range(25):
        if adjacency_demand[i][j] == 1:
            demand_list.append((i,j))
        if adjacency_road[i][j]==1:
            road_list.append((i,j))

node_lane = np.zeros([len(road_list),3])
node_lane[:,0] = np.arange(len(road_list))
for i in range(len(road_list)):
    node_lane[i][1] = road_list[i][0]
    node_lane[i][2] = road_list[i][1]
    
time_step = 600
delta_t = 10
speed_free = 54
weav_speed = 21.6
capacity = 5
cell_length = 150
cell_capacity = 60
veh_id = 0

lane_adjacency = np.zeros([len(road_list),len(road_list)])
lane_link = np.zeros([len(road_list),3])
for i in range(len(road_list)):
    for j in range(len(road_list)):
        if (node_lane[i][2] == node_lane[j][1])&(node_lane[i][1]!=node_lane[j][2]):
            lane_adjacency[i][j]=1
            if lane_link[i][0] == 0:
                lane_link[i][0] = j
            elif lane_link[i][1] == 0:
                lane_link[i][1] = j
            else:
                lane_link[i][2] = j
     
ava_congestion_scale = []
congestion_scale_ar = np.zeros([500,8])
line_num = []
for veh_interval in range(5,6):
    lane_ar = np.zeros([adjacency_road.sum()*5,3])
    node_ar = np.zeros([adjacency_road.sum(),4])
    node_ar[:,0] = np.arange(len(road_list))
    for i in range(len(lane_ar)):
        lane_ar[i][0] = i//5
        lane_ar[i][1] = i%5
    def BPR(node_ar,lane_ar):
        weight_list = adjacency_road.copy()
        weight_list[weight_list==0] = 10000
        for i in range(len(weight_list)):
            weight_list[i][i] = 0
        for i in range(len(road_list)):
            lane_flow = lane_ar[i][2]
            node_flow = node_ar[i][1]+node_ar[i][2]+node_ar[i][3]
            flow = lane_flow+node_flow
            weight = 1+flow
            weight_list[road_list[i][0]][road_list[i][1]] = weight
    
        def floyd(d):
            p = np.zeros(d.shape)
            for i in range(len(p)):
                p[i,:] = i
            for n in range(len(d)):
                for i in range(len(d)):
                    for j in range(len(d)):
                        if (i == n) or (j == n):
                            continue
                        if d[i][j] > d[i][n] + d[n][j]:
                            d[i][j] = d[i][n] + d[n][j]
                            p[i][j] = p[n][j]
            return p
        weight_ar = floyd(weight_list.copy())
        short_path = {}
        for i in range(len(weight_ar)):
            short_path[i] = {}
        for i in range(len(weight_ar)):
            for j in range(len(weight_ar)):
                if i == j:
                    path = [i]
                else:
                    path = [j]
                    k = j
                    while True:
                        if weight_ar[i][k] == i:
                            path.append(i)
                            break
                        else:
                            k = int(weight_ar[i][k])
                            path.append(k)
                short_path[i][j] = path[::-1]
        return short_path

    congestion_scale_list = [0]
    veh_ar = np.zeros([25*veh_interval,20])
    for t in range(500):    
        veh_ar_view = veh_ar.copy()
        lane_ar_view0 = lane_ar.copy()
        node_ar_view0 = node_ar.copy()
        #进入节点
        if t != 0:
            signal = np.zeros([len(lane_adjacency),3])
            zone_ar = np.zeros([len(lane_adjacency),3])
            node_flow_ar = np.zeros([len(lane_adjacency),3])
            for i in range(len(lane_adjacency)):
                linked = np.where(lane_adjacency[:,i]==1)[0]
                zone_ar[i] = np.array(linked)
                zone_list = []
                for j in linked:
                    zones = np.where(lane_link[j]==i)[0][0]
                    zone_list.append(zones)
                node_flow = []
                for k in range(len(linked)):
                    node_flow.append(node_ar[linked[k]][zone_list[k]+1])
                node_flow_ar[i] = np.array(node_flow)
                signal[i] = np.array(node_flow)/np.array(node_flow).sum()
            signal[signal!=signal] = 0
            free_flow = 60 - lane_ar[lane_ar[:,1]==0][:,2]
            free_flow_ar = np.zeros([len(lane_adjacency),3])
            for i in range(3):
                free_flow_ar[:,i] = free_flow
            transit_flow = np.minimum(np.minimum(node_flow_ar,5),(signal*21.6*free_flow_ar/54).astype(int))
            lane_start = lane_ar[lane_ar[:,1]==0]
            for i in range(len(lane_adjacency)):
                lane_start[i][2] +=  transit_flow[i].sum()
            lane_ar_view0[lane_ar[:,1]==0] = lane_start
            for i in range(len(lane_adjacency)):
                for j in range(3):
                    lane_index0 = zone_ar[i][j]
                    lane_index1 = np.where(lane_link[int(lane_index0)]==i)[0][0]
                    node_ar_view0[int(lane_index0)][int(lane_index1)+1] -= transit_flow[i][j]
                    veh_transit = veh_ar[(veh_ar[:,2]==5)&(veh_ar[:,3]==int(lane_index0))&(veh_ar[:,4]==i)]
                    if len(veh_transit)!=0:
                        veh_transit[:int(transit_flow[i][j]),1]=t
                        veh_transit[:int(transit_flow[i][j]),2]=0
                        path_ = veh_transit[:int(transit_flow[i][j]),3:]
                        path_ = path_[:,1:]
                        path_ = np.append(path_,np.zeros([len(path_),1]),axis =1)
                        path_[:,-1] = np.nan
                        veh_transit[:int(transit_flow[i][j]),3:] = path_
                    veh_ar_view[(veh_ar[:,2]==5)&(veh_ar[:,3]==int(lane_index0))&(veh_ar[:,4]==i)] = veh_transit
            lane_diff0 = lane_ar_view0[:,2] - lane_ar[:,2]
            node_diff0 = node_ar_view0 - node_ar
            lane_ar_view1 = lane_ar.copy()
            node_ar_view1 = node_ar.copy()
        #渠化区域
            cell40 = []
            cell41 = []
            cell42 = []
            cell_leave = []
            cell4 = lane_ar[lane_ar[:,1]==4]
            for i in range(len(lane_link)):
                cell40.append(len(veh_ar[(veh_ar[:,2]==4)&(veh_ar[:,3]==i)&(veh_ar[:,4]==lane_link[i][0])]))
                cell41.append(len(veh_ar[(veh_ar[:,2]==4)&(veh_ar[:,3]==i)&(veh_ar[:,4]==lane_link[i][1])]))
                cell42.append(len(veh_ar[(veh_ar[:,2]==4)&(veh_ar[:,3]==i)&(veh_ar[:,4]==lane_link[i][2])]))
                cell_leave.append(len(veh_ar[(veh_ar[:,2]==4)&(veh_ar[:,3]==i)&(veh_ar[:,4]!=veh_ar[:,4])]))
            cell40 = np.array(cell40)
            cell41 = np.array(cell41)
            cell42 = np.array(cell42)
            cell_leave = np.array(cell_leave)
            cell50 = node_ar[:,1]
            cell51 = node_ar[:,2]
            cell52 = node_ar[:,3]
            n50 = np.minimum(np.minimum(cell40,5),(21.6*(20-cell50)/54).astype(int))
            n51 = np.minimum(np.minimum(cell41,5),(21.6*(20-cell51)/54).astype(int))
            n52 = np.minimum(np.minimum(cell42,5),(21.6*(20-cell52)/54).astype(int))
            n_leave = cell_leave
            node_ar_view1[:,1] = cell50 + n50
            node_ar_view1[:,2] = cell51 + n51
            node_ar_view1[:,3] = cell52 + n52 
            cell4_new = cell4[:,2]
            cell4_new -= n50+n51+n52+n_leave
            cell4[:,2] = cell4_new
            lane_ar_view1[lane_ar[:,1]==4] = cell4
            
            lane_diff1 = lane_ar_view1[:,2] - lane_ar[:,2]
            node_diff1 = node_ar_view1 - node_ar
            
            for i in range(len(lane_link)):
                veh40 = veh_ar[(veh_ar[:,2]==4)&(veh_ar[:,3]==i)&(veh_ar[:,4]==lane_link[i][0])]
                veh41 = veh_ar[(veh_ar[:,2]==4)&(veh_ar[:,3]==i)&(veh_ar[:,4]==lane_link[i][1])]
                veh42 = veh_ar[(veh_ar[:,2]==4)&(veh_ar[:,3]==i)&(veh_ar[:,4]==lane_link[i][2])]
                veh_leave = veh_ar[(veh_ar[:,2]==4)&(veh_ar[:,3]==i)&(veh_ar[:,4]!=veh_ar[:,4])]
                veh40[:int(n50[i]),2] = 5
                veh40[:int(n50[i]),1] = t
                veh41[:int(n51[i]),2] = 5
                veh41[:int(n51[i]),1] = t
                veh42[:int(n52[i]),2] = 5
                veh42[:int(n52[i]),1] = t
                veh_leave[:int(n_leave[i]),2] = 10
                veh_leave[:int(n_leave[i]),1] = t
                veh_ar_view[(veh_ar[:,2]==4)&(veh_ar[:,3]==i)&(veh_ar[:,4]==lane_link[i][0])] = veh40
                veh_ar_view[(veh_ar[:,2]==4)&(veh_ar[:,3]==i)&(veh_ar[:,4]==lane_link[i][1])] = veh41
                veh_ar_view[(veh_ar[:,2]==4)&(veh_ar[:,3]==i)&(veh_ar[:,4]==lane_link[i][2])] = veh42
                veh_ar_view[(veh_ar[:,2]==4)&(veh_ar[:,3]==i)&(veh_ar[:,4]!=veh_ar[:,4])] = veh_leave
            
            #路段
            lane_ar_view2 = lane_ar.copy()
            node_ar_view2 = node_ar.copy()
            
            veh_num = lane_ar[:,2]
            veh_num_copy = veh_num.copy() 
            cell0 = lane_ar[lane_ar[:,1]==0]
            cell1 = lane_ar[lane_ar[:,1]==1]
            cell2 = lane_ar[lane_ar[:,1]==2]
            cell3 = lane_ar[lane_ar[:,1]==3]
            cell4 = lane_ar[lane_ar[:,1]==4]
            n1 = np.minimum(np.minimum(cell0[:,2],15),(21.6*(60-cell1[:,2])/54).astype(int))
            n2 = np.minimum(np.minimum(cell1[:,2],15),(21.6*(60-cell2[:,2])/54).astype(int))
            n3 = np.minimum(np.minimum(cell2[:,2],15),(21.6*(60-cell3[:,2])/54).astype(int))
            n4 = np.minimum(np.minimum(cell3[:,2],15),(21.6*(60-cell4[:,2])/54).astype(int))
            cell0_new =cell0[:,2] - n1
            cell1_new =cell1[:,2] + n1 - n2
            cell2_new =cell2[:,2] + n2 - n3
            cell3_new =cell3[:,2] + n3 - n4
            cell4_new =cell4[:,2] + n4
            cell0[:,2] = cell0_new
            cell1[:,2] = cell1_new
            cell2[:,2] = cell2_new
            cell3[:,2] = cell3_new              
            cell4[:,2] = cell4_new
            lane_ar_view2[lane_ar[:,1]==0] = cell0
            lane_ar_view2[lane_ar[:,1]==1] = cell1
            lane_ar_view2[lane_ar[:,1]==2] = cell2
            lane_ar_view2[lane_ar[:,1]==3] = cell3
            lane_ar_view2[lane_ar[:,1]==4] = cell4
            veh_ar_copy = veh_ar.copy()
            for i in range(len(road_list)):
                veh_i0 = veh_ar[(veh_ar[:,2]==0)&(veh_ar[:,3]==i)]
                veh_i1 = veh_ar[(veh_ar[:,2]==1)&(veh_ar[:,3]==i)]
                veh_i2 = veh_ar[(veh_ar[:,2]==2)&(veh_ar[:,3]==i)]
                veh_i3 = veh_ar[(veh_ar[:,2]==3)&(veh_ar[:,3]==i)]
                veh_i0[:int(n1[i]),2] = 1
                veh_i0[:int(n1[i]),1] = t
                veh_i1[:int(n2[i]),2] = 2
                veh_i1[:int(n2[i]),1] = t
                veh_i2[:int(n3[i]),2] = 3
                veh_i2[:int(n3[i]),1] = t
                veh_i3[:int(n4[i]),2] = 4
                veh_i3[:int(n4[i]),1] = t
                veh_ar_view[(veh_ar_copy[:,2]==0)&(veh_ar[:,3]==i)] = veh_i0
                veh_ar_view[(veh_ar_copy[:,2]==1)&(veh_ar[:,3]==i)] = veh_i1
                veh_ar_view[(veh_ar_copy[:,2]==2)&(veh_ar[:,3]==i)] = veh_i2
                veh_ar_view[(veh_ar_copy[:,2]==3)&(veh_ar[:,3]==i)] = veh_i3
                
            lane_diff2 = lane_ar_view2[:,2] - lane_ar[:,2]
            node_diff2 = node_ar_view2 - node_ar
            
            veh_ar = veh_ar_view
            lane_ar[:,2] += lane_diff0+lane_diff1+lane_diff2
            node_ar += node_diff0+node_diff1+node_diff2
            #车辆离开
            veh_ar = veh_ar[veh_ar[:,2]!=10]
            congestion_scale = len(lane_ar[lane_ar[:,2]>=54])+len(node_ar[(node_ar[:,1]>=18)|(node_ar[:,2]>=18)|(node_ar[:,3]>=18)])
            congestion_scale_list.append(congestion_scale)
            line_num.append(len(veh_ar[veh_ar[:,2] == -1]))
            #排队
            short_path = BPR(node_ar,lane_ar)
            lane_start = lane_ar[lane_ar[:,1]==0]
            cell0_capacity = 60-lane_start[:,2]
            line_up = veh_ar[veh_ar[:,2] == -1]
            O = line_up[:,3].copy()
            D = np.zeros(len(O))
            for i in range(len(line_up)):
                line_up_i = line_up[i]
                D[i]=line_up_i[line_up_i==line_up_i][-1]
            line_up[:,3:] = np.nan
            for i in range(len(line_up)):
                O_node = node_lane[int(O[i])][1]
                D_node = node_lane[int(D[i])][2]
                path_node = np.array(short_path[int(O_node)][int(D_node)])
                path_lane = []
                for j in range(len(path_node)-1):
                    path_lane.append(int(node_lane[(node_lane[:,1]==path_node[j])&(node_lane[:,2]==path_node[j+1])][0][0])) 
                    for k in range(len(path_lane)):
                        line_up[i,3+k] = path_lane[k]
            for i in range(len(lane_start)):
                veh_line_up = line_up[line_up[:,3]==i]
                veh_line_up[:int(cell0_capacity[i]),2] = 0
                line_up[line_up[:,3]==i] = veh_line_up
                lane_start[i][2]+=np.minimum(cell0_capacity[i],len(veh_line_up))
            lane_ar[lane_ar[:,1]==0] = lane_start
            veh_ar[veh_ar[:,2] == -1] = line_up
        #生成车辆
        if t <100:
            short_path = BPR(node_ar,lane_ar)
            if t != 0:
                veh_ar_new = np.zeros([25*veh_interval,20]) 
                veh_ar_new[:,0] = np.arange(veh_id,veh_id+25*veh_interval)
                veh_ar_new[:,1] = t
                veh_ar_new[:,3:] = None
                veh_id += 25*veh_interval 
                veh_ar = np.vstack((veh_ar,veh_ar_new))
            else:
                veh_ar[:,0 ] = np.arange(veh_id,veh_id+25*veh_interval)
                veh_ar[:,3:] = None
                veh_id += 25*veh_interval
            veh_choose = np.zeros(25*veh_interval)
            for i in range(25):
                choose  = np.where(adjacency_demand[i]==1)[0]
                pr = np.ones(len(choose))*(1/len(choose))
                veh_choose[veh_interval*i:veh_interval*(i+1)] = np.random.choice(choose,veh_interval,p=pr)
            for i in range(25*veh_interval):
                path_node = short_path[i//veh_interval][int(veh_choose[i])]
                path_lane = []
                for j in range(len(path_node)-1):
                    path_lane.append(int(node_lane[(node_lane[:,1]==path_node[j])&(node_lane[:,2]==path_node[j+1])][0][0])) 
                for k in range(len(path_lane)):
                    veh_ar_ = veh_ar[(veh_ar[:,0]>=veh_id-25*veh_interval+veh_interval*i)&(veh_ar[:,0]<veh_id-25*veh_interval+veh_interval*(i+1))]
                    veh_ar_[:,k+3] = path_lane[k]
                    veh_ar[(veh_ar[:,0]>=veh_id-25*veh_interval+veh_interval*i)&(veh_ar[:,0]<veh_id-25*veh_interval+veh_interval*(i+1))] = veh_ar_
            lane_start = lane_ar[lane_ar[:,1]==0]
            cell0_capacity = 60-lane_start[:,2]
            veh_ar_new = veh_ar[(veh_ar[:,0]>=veh_id-25*veh_interval)&(veh_ar[:,0]<veh_id)]
            for i in range(len(lane_start)):
                veh_line_up = veh_ar_new[veh_ar_new[:,3]==i]
                if cell0_capacity[i] <= len(veh_line_up):
                    veh_line_up[int(cell0_capacity[i]):,2] = -1
                veh_ar_new[veh_ar_new[:,3]==i] = veh_line_up
            veh_ar[(veh_ar[:,0]>=veh_id-25*veh_interval)&(veh_ar[:,0]<veh_id)] = veh_ar_new 
            for i in range(len(lane_start)):
                lane_start[i][2] = len(veh_ar[(veh_ar[:,3]==i)&(veh_ar[:,2]==0)])
            lane_ar[lane_ar[:,1]==0] = lane_start
        veh_ar = veh_ar[veh_ar[:,1].argsort()]
        print(t)

    ava_congestion_scale.append(sum(congestion_scale_list)/500)
    congestion_scale_ar[:,veh_interval-1] = np.array(congestion_scale_list)

