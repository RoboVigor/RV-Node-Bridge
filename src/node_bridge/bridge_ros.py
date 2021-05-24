#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from node_bridge_ros import msg
from node_bridge.bridge import NodeBridge
from node_bridge.protocol import NodeBridgeProtocol
from munch import munchify

def talker():
    #ros
    rospy.init_node('node_bridge', anonymous=True)
    rate = rospy.Rate(125) 
    #bridge
    bridge = NodeBridge('serial', port=0)
    protocol_data = NodeBridgeProtocol()
    def on_receive(new_byte):
        result = protocol_data.update(new_byte)
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
    while not rospy.is_shutdown():
        bridge.open(on_receive)
        # rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass