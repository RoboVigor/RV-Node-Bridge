import sys
import os
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__))+'/../node_bridge/')
import protocol, bridge
import time

bridge = bridge.NodeBridge('serial', port='/dev/ttyTHS2')
protocol_data = protocol.NodeBridgeProtocol()

def on_receive_test(new_byte):
    result = protocol_data.update(new_byte, verbose=True)
    if 'result' in result:
        return False
    return True
while True:
    bridge.open(on_receive_test)