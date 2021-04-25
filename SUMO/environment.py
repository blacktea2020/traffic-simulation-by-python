import os
import sys
from sumolib import checkBinary
def main(sumocfgFile='sumo.sumocfg'):
    if 'SUMO_HOME' in os.environ:
        tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
        sys.path.append(tools)
    else:
        sys.exit("please declare environment variable 'SUMO_HOME'")
    
    if_show_gui = True
    
    if not if_show_gui:
        sumoBinary = checkBinary("sumo")
    else:
        sumoBinary = checkBinary("sumo-gui")
    sumoCmd = [sumoBinary, "-c", sumocfgFile]
    return sumoCmd