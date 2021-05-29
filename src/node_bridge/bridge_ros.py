#!/usr/bin/env python
import rospy
from node_bridge_ros import msg
from node_bridge.bridge import NodeBridge
from node_bridge import protocol
from node_bridge.protocol import NodeBridgeProtocol
import std_msgs
from munch import munchify
from queue import Queue

class NodeBridgeROS():
    def __init__(self, port_type='serial', port=0):
        #ros
        rospy.init_node('node_bridge', anonymous=True)
        rate = rospy.Rate(125) 
        #bridge
        print('node bridge listen on /dev/usbTTY')
        bridge = NodeBridge('serial', port='/dev/usbTTY')
        self._protocol_data = NodeBridgeProtocol()
        self._send_queue = Queue()
        #service
        self._init_service()
        while not rospy.is_shutdown():
            bridge.open(self._on_receive, send_queue=self._send_queue)

    def _on_receive(self, new_byte):
        result = self._protocol_data.update(new_byte)
        if 'result' in result:
            result = result['result']
            #ros
            pub = rospy.Publisher('node_bridge/'+result['name'], getattr(msg, result['name']), queue_size=10)
            data = getattr(msg, result['name'])()
            for (key,value) in result['data'].items():
                if hasattr(data, key):
                    setattr(data, key, value)
            del result['data']['seq']
            data = munchify(result['data'])
            pub.publish(**data)

    def _init_service(self):
        serice_list = [x for x,y in protocol.protocol_info_table.items() if y['service']]
        for service_name in serice_list:
            s = rospy.Service(service_name, getattr(msg, service_name), self._handle_service)

    def _handle_service(self, req):
        # for (key,value) in rqt:
        rospy.loginfo(dir(req))
        return std_msgs.msg.String('Sent.')

if __name__ == '__main__':
    try:
        NodeBridgeROS()
    except rospy.ROSInterruptException:
        pass