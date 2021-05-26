import sys
import os
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__))+'/../node_bridge/')
import protocol, bridge
import time

bridge = bridge.NodeBridge('serial', port='COM7')
protocol_data = protocol.NodeBridgeProtocol()


def send(vx, vy):
    global i
    test_data['vx'] = vx
    test_data['vy'] = vy
    test_packet = protocol.pack('chassisData', test_data, seq=i)
    print(test_packet)
    bridge.send(test_packet)
    time.sleep(0.1)


test_data = protocol.create_protocol_data('chassisData')
speed = 0.2
for i in range(20):
    send(speed, 0)
for i in range(20):
    send(0, speed)
for i in range(20):
    send(-speed, 0)
for i in range(20):
    send(0, -speed)