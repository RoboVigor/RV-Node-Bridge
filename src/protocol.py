# -*- coding: utf-8 -*-
# CRC-8

import serial
from serial.tools import list_ports
import bitstring
import math
import re
import helpers

# Set default port
port_list = list(list_ports.comports())
default_port = ''
if len(port_list) > 0:
    default_port = port_list[0].device

protocol_info_table = {}

def unpack_data(name, packet):
    data = {}
    format_string = ''
    for (key, info) in protocol_info_table[name]['struct'].items():
        new_data = []
        for i in range(info.get('arraysize',1)):
            if 'struct' in info:
                size = protocol_info_table[info['struct']]['size']
                new_data += [unpack_data(info['struct'], packet)]
            else:
                size = math.ceil(int(re.match('.*:(?P<bitsize>\d*)', info['format']).group('bitsize'))/8)
                new_data += bitstring.BitArray(bytes=packet).unpack(info['format'])
            packet = packet[size:]
        if 'arraysize' in info:
            data[key] = new_data
        else:
            data[key] = new_data[0]
    return data

def unpack(packet, skip_crc8=False):
    if not packet[0]==0xa5 or len(packet)<8:
        return None
    protocol_data = {}
    # validate
    crc16_pass = helpers.get_crc16_checksum(packet[0:-2])==packet[-2]+(packet[-1]<<8)
    crc8_pass = skip_crc8 or helpers.get_crc8_checksum(packet[0:4])==packet[4]
    if not crc16_pass or not crc8_pass:
        return None
    # header
    protocol_data['data_length'] = packet[1]+(packet[2]<<8)
    protocol_data['seq'] = packet[3]
    protocol_data['id'] = packet[5-skip_crc8]+(packet[6-skip_crc8]<<8)
    # data
    name = next(x for x,y in protocol_info_table.items() if y['id']==protocol_data['id'])
    protocol_data['data'] = unpack_data(name, packet[7-skip_crc8:-2])
    return protocol_data

def pack_data(name, data):
    formats = []
    data_items = []
    for (key, info) in protocol_info_table[name]['struct'].items():
        if 'arraysize' in info:
            data_list = data[key]
        else:
            data_list = [data[key]]
        for new_data in data_list:
            if 'struct' in info:
                info['format'], new_data = pack_data(info['struct'], new_data)
                data_items += new_data
            else:
                data_items += [new_data]
            formats += [info['format']]
    return (",".join(formats), data_items)

def pack(name, data, seq=0, skip_crc8=False):
    protocol_info = protocol_info_table[name]
    packet = bitstring.pack('<BHB', *[0xa5, protocol_info['size'], seq])
    if not skip_crc8:
        packet += bytes([helpers.get_crc8_checksum(packet.tobytes())])
    packet += bitstring.pack('<H', protocol_info['id'])
    format_string, data_items = pack_data(name, data)
    packet += bitstring.pack(format_string, *data_items)
    packet += bitstring.pack('<H', helpers.get_crc16_checksum(packet.tobytes()))
    return packet.tobytes()


def send(data, feedback=False, port=default_port):
    with serial.Serial(port, 115200, timeout = 0.1) as ser:
        ser.write(bytearray(data))
        if feedback:
            x=ser.read(1000)
            print([i for i in x])
            print(x)
            return x
        else:
            return True


if __name__ == '__main__':
    protocol_info_table = {
        'autoaim': {'id': 0x401, 'description':'for test', 'size':24, 'struct':{
            'x':   {'format':'floatle:32'},
            'y':   {'format':'floatle:32'},
            'z':   {'format':'floatle:32', 'arraysize':2},
            'test':{'struct': 'Test_Type', 'arraysize':2}
        }},
        'Test_Type': {'size':4, 'struct':{
            'x':   {'format':'floatle:32'}
        }},
    }

    # pack & unpack
    test_data = {'x':1.11,'y':2.22,'z':[3.33,4.44],'test':[{'x':5.55},{'x':6.66}]}
    test_packet = pack('autoaim', test_data)
    print('pack result: ', test_packet)
    print('unpack result: ', unpack(test_packet))

    # unpack
    # unpacker=Unpacker()
    # with serial.Serial('COM6', 115200, timeout=0.1) as ser:
    #     while True:
    #         byte=ser.read(1)
    #         # print('Unpack: {:02X}'.format(byte))
    #         # info=unpacker.send(int.from_bytes(byte, byteorder = 'little'))
    #         if info['state'] == 'EOF':
    #             print('id: 0x{:04X}'.format(info['id']))
    #             print('data: ', info['packet'])
