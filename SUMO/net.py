'''
实现自动创建net.xml
created on Sat Nov 14 13:19:38 2020
Simple.nod(length)
导出nod.xml，length为单行道长度
Simple.edg(speed)
导出edg.xml，speed为道路限速
Simple.netconvert(Path)
导出net.xml，Path为SUMO文件根路径/bin
例： r'D\APPs\SUMO\bin'
'''
from xml.dom.minidom import Document
import os
class Simple:
    def nod(x):
        doc = Document()
        nodes = doc.createElement('nodes')
        doc.appendChild(nodes)
        node = doc.createElement('node')
        nodes.appendChild(node)
        node.setAttribute('id','node1')
        node.setAttribute('x','0')
        node.setAttribute('y','0')
        node.setAttribute('type','priority')
        node = doc.createElement('node')
        nodes.appendChild(node)
        node.setAttribute('id','node2')
        node.setAttribute('x',str(x))
        node.setAttribute('y','0')
        node.setAttribute('type','priority')
        with open('nod.xml', 'wb+') as f:
            f.write(doc.toprettyxml(indent='\t', encoding='utf-8'))
        return
    def edg(speed):
        doc = Document()
        edges = doc.createElement('edges')
        doc.appendChild(edges)
        edge = doc.createElement('edge')
        edges.appendChild(edge)
        edge.setAttribute('id','edgeR-1-2')
        edge.setAttribute('from','node1')
        edge.setAttribute('to','node2')
        edge.setAttribute('priority','75')
        edge.setAttribute('priority','75')
        edge.setAttribute('numLanes','1')
        edge.setAttribute('speed',str(speed))
        with open('edg.xml', 'wb+') as f:
            f.write(doc.toprettyxml(indent='\t', encoding='utf-8'))
        return
    def netconvert(path_sumo):
        path_root = os.path.abspath('.')
        path_sumo_disk = path_sumo[0]
        copy0 = 'copy ' + path_root + r'\nod.xml ' + path_sumo + ' /y'
        copy1 = 'copy ' + path_root + r'\edg.xml ' + path_sumo + ' /y'
        cd_sumo = 'cd ' + '/' + path_sumo_disk + ' ' + path_sumo
        os.system(copy0)
        os.system(copy1)
        netconvert = 'netconvert --node-files=nod.xml --edge-files=edg.xml --output-file=net.xml'
        os.system(cd_sumo + ' && ' + netconvert)
        copy2 = 'copy ' + path_sumo + r'\net.xml ' + path_root + ' /y'
        os.system(copy2)
