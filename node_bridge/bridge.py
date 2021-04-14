# -*- coding: utf-8 -*-

import serial
from serial.tools import list_ports
import helpers
from queue import Queue
import time


class NodeBridge():

    def __init__(self, port_type='serial', port=0):
        self.port_type = port_type
        if port_type == 'serial':
            # Set default port
            port_list = list(list_ports.comports())
            print('Available ports: ', [x.device for x in port_list])
            self.default_port = ''
            if type(port) == str:
                self.default_port = port
            else:
                if len(port_list) > 0:
                    self.default_port = port_list[port].device
        print('Communication port: ', self.default_port)

    def send(self, packet, port=None):
        if not port:
            port = self.default_port
        with serial.Serial(port, 115200, timeout=0.1) as ser:
            ser.write(packet)
            return True

    def open(self, on_receive, send_queue=None, port=None):
        if not port:
            port = self.default_port
        if not send_queue:
            send_queue = Queue()
        with serial.Serial(port, 115200, timeout=0.1) as ser:
            while True:
                while on_receive(ser.read(1)):
                    pass
                while not send_queue.empty():
                    a = send_queue.get()
                    ser.write(a)


if __name__ == '__main__':
    import protocol
    bridge = NodeBridge('serial', port='COM7')
    protocol_data = protocol.NodeBridgeProtocol()

    # send
    # while True:
    test_data = protocol.create_protocol_data('autoaimData')

    def send(yaw_diff, pitch_diff):
        test_data['yaw_angle_diff'] = yaw_diff
        test_data['pitch_angle_diff'] = pitch_diff
        test_packet = protocol.pack('autoaimData', test_data, seq=i)
        print(test_packet)
        bridge.send(test_packet)
        time.sleep(0.1)
    angle = 0.2
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

    # send
    # while True:
    # test_data = protocol.create_protocol_data('chassisData')

    # def sendvw(vw):
    #     test_data['vw'] = vw
    #     test_packet = protocol.pack('chassisData', test_data, seq=i)
    #     print(test_packet)
    #     bridge.send(test_packet)
    #     time.sleep(0.1)
    # speed = 0.3
    # for i in range(10):
    #     sendvw(speed)
    # for i in range(20):
    #     sendvw(-1*speed)
    # for i in range(10):
    #     sendvw(speed)
    # for i in range(10):
    #     sendvw(0)

    # receive
    # def on_receive_test(new_byte):
    #     result = protocol_data.update(new_byte, verbose=True)
    #     if 'result' in result:
    #         return False
    #     return True
    # while True:
    #     bridge.open(on_receive_test)

    # send while receive
    # i = 0
    # queue_test = Queue()

    # def on_receive_test(new_byte):
    #     global i, queue_test
    #     # add heartbeat to queue
    #     if i % 5 == 0:
    #         test_data = protocol.create_protocol_data('heartbeat')
    #         test_packet = protocol.pack('heartbeat', test_data, seq=i)
    #         queue_test.put(test_packet)
    #     i = (i+1) % 256
    #     # receive protocol
    #     result = protocol_data.update(new_byte, verbose=True)
    #     if 'result' in result:
    #         return False
    #     return True
    # bridge.open(on_receive_test, send_queue=queue_test)
