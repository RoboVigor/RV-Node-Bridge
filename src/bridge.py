# -*- coding: utf-8 -*-

import serial
from serial.tools import list_ports
import helpers


class NodeBridge():
    
    def __init__(self, type='serial', port=0):
        self.type = type
        if type=='serial':
            # Set default port
            port_list = list(list_ports.comports())
            self.default_port = ''
            if len(port_list) > 0:
                self.default_port = port_list[port].device

    def send(self, packet, port=None):
        if not port:
            port = self.default_port
        with serial.Serial(port, 115200, timeout = 0.1) as ser:
            ser.write(packet)
            return True

    def on_receive(self, callback, port=None):
        if not port:
            port = self.default_port
        with serial.Serial(port, 115200, timeout=0.1) as ser:
            while callback(ser.read(1)):
                pass

if __name__ == '__main__':
        import protocol
        bridge = NodeBridge('serial', port=1)
        protocol_data = protocol.NodeBridgeProtocol()
        def on_receive_test(new_byte):
            result = protocol_data.update(new_byte, verbose=True)
            if 'result' in result:
                return False
            return True

        # send
        # while True:
        #     for i in range(10):
        #         test_data = protocol.create_protocol_data('autoaimData')
        #         test_data['fire'] = True
        #         test_data['pitch_angle_diff'] = 1.11
        #         test_data['yaw_angle_diff'] = 2.22
        #         test_packet = protocol.pack('autoaimData', test_data)
        #         bridge.send(test_packet)

        # receive
        # while True:
        #     bridge.on_receive(on_receive_test)

        # send while receive
        # @todo: 减少开关端口次数
        i=0
        while True:
            i = (i+1)%256
            test_data = protocol.create_protocol_data('heartbeat')
            # test_data['data1'] = i
            test_packet = protocol.pack('heartbeat', test_data)
            bridge.send(test_packet)
            bridge.on_receive(on_receive_test)


