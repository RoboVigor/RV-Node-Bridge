import sys
import os
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__))+'/../node_bridge/')
import protocol, bridge
import time

bridge = bridge.NodeBridge('serial', port='/dev/ttyTHS2')
protocol_data = protocol.NodeBridgeProtocol()


def send(yaw_diff, pitch_diff):
    test_data['yaw_angle_diff'] = yaw_diff
    test_data['pitch_angle_diff'] = pitch_diff
    test_packet = protocol.pack('autoaimData', test_data, seq=i)
    print(test_packet)
    bridge.send(test_packet)
    time.sleep(0.1)


test_data = protocol.create_protocol_data('autoaimData')
angle = 1
for i in range(10):
    send(0, angle/2)
for i in range(20):
    send(0, -angle/2)
for i in range(10):
    send(0, angle/2)
time.sleep(2)
for i in range(10):
    send(angle, 0)
for i in range(20):
    send(-angle, 0)
for i in range(10):
    send(angle, 0)
