from xml.dom.minidom import Document

def cfg(stepLength):
    doc = Document()
    configuration = doc.createElement('configuration')
    doc.appendChild(configuration)
    configuration.setAttribute('xmlns:xsi',
                               'http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/sumoConfiguration.xsd')
    
    input = doc.createElement('input')
    configuration.appendChild(input)
    net_file = doc.createElement('net-file')
    input.appendChild(net_file)
    net_file.setAttribute('value','net.xml')
    
    time = doc.createElement('time')
    configuration.appendChild(time)
    step_length = doc.createElement('step-length')
    time.appendChild(step_length)
    step_length.setAttribute('value',str(stepLength))
    
    processing = doc.createElement('processing')
    configuration.appendChild(processing)
    step_method = doc.createElement('step-method.ballistic')
    processing.appendChild(step_method)
    step_method.setAttribute('value','true')
    
    Gui_only = doc.createElement('Gui-only')
    configuration.appendChild(Gui_only)
    start = doc.createElement('start')
    Gui_only.appendChild(start)
    start.setAttribute('value',"t")
    quit_on_end = doc.createElement('quit-on-end')
    Gui_only.appendChild(quit_on_end)
    quit_on_end.setAttribute('value',"t")
    
    
    with open('sumo.sumocfg', 'wb+') as f:
        f.write(doc.toprettyxml(indent='\t', encoding='utf-8'))
    return