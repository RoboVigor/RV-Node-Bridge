# -*- coding: utf-8 -*-
import yaml
import bitstring
import math
import re
import helpers
import os

# load protocol info
with open(os.path.split(os.path.realpath(__file__))[0]+'/../dist/protocol.yaml', 'r', encoding="utf-8") as file:
    file_data = file.read()
    protocol_info_table = yaml.safe_load(file_data)


def packet_to_dict(name, packet):
    data = {}
    format_string = ''
    for item in protocol_info_table[name]['struct']:
        new_data = []
        for i in range(item.get('arraysize', 1)):
            if 'struct' in item:
                size = protocol_info_table[item['struct']]['size']
                new_data += [packet_to_dict(item['struct'], packet)]
            else:
                size = math.ceil(
                    int(re.match('.*:(?P<bitsize>\d*)', item['format']).group('bitsize'))/8)
                new_data += bitstring.BitArray(
                    bytes=packet).unpack(item['format'])
            packet = packet[size:]
        if 'arraysize' in item:
            data[item['key']] = new_data
        else:
            data[item['key']] = new_data[0]
    return data


def create_protocol_data(name):
    return packet_to_dict(name, bytes([0]*protocol_info_table[name]['size']))


def dict_to_packet(name, data):
    def generate_format_sequence(name, data):
        formats = []
        data_items = []
        for item in protocol_info_table[name]['struct']:
            if 'arraysize' in item:
                data_list = data[item['key']]
            else:
                data_list = [data[item['key']]]
            for new_data in data_list:
                if 'struct' in item:
                    item['format'], new_data = generate_format_sequence(
                        item['struct'], new_data)
                    data_items += new_data
                else:
                    data_items += [new_data]
                formats += [item['format']]
        return (",".join(formats), data_items)
    format_string, data_items = generate_format_sequence(name, data)
    return bitstring.pack(format_string, *data_items)


def unpack(packet, skip_crc8=False):
    # find SOF
    unused_packet = packet
    try:
        start_pos = packet.index(0xa5)
        packet = packet[start_pos:]
    except ValueError:
        return {'unused_packet': unused_packet}
    # check crc8
    if len(packet) < 5-skip_crc8:
        return {'unused_packet': unused_packet}
    if not skip_crc8 and helpers.get_crc8_checksum(packet[0:4]) != packet[4]:
        return {'unused_packet': packet[5:], 'error': 'CRC8 Not Correct'}
    # check crc16
    protocol_data = {}
    protocol_data['data_length'] = packet[1]+(packet[2] << 8)
    packet_correct_size = 9-skip_crc8+protocol_data['data_length']
    if len(packet) < packet_correct_size:
        return {'unused_packet': unused_packet}
    packet, unused_packet = (
        packet[0:packet_correct_size], packet[packet_correct_size:])
    if not helpers.get_crc16_checksum(packet[0:-2]) == packet[-2]+(packet[-1] << 8):
        return {'unused_packet': unused_packet[packet_correct_size:], 'error': 'CRC16 Not Correct'}
    # parse header
    protocol_data['seq'] = packet[3]
    protocol_data['id'] = packet[5-skip_crc8]+(packet[6-skip_crc8] << 8)
    # parse data
    protocol_data['name'] = next(x for x, y in protocol_info_table.items(
    ) if y.get('id', '') == protocol_data['id'])
    protocol_data['data'] = packet_to_dict(
        protocol_data['name'], packet[7-skip_crc8:-2])
    return {'unused_packet': unused_packet, 'result': protocol_data}


def pack(name, data, seq=1, skip_crc8=False):
    protocol_info = protocol_info_table[name]
    packet = bitstring.pack('<BHB', *[0xa5, protocol_info['size'], seq])
    if not skip_crc8:
        packet += bytes([helpers.get_crc8_checksum(packet.tobytes())])
    packet += bitstring.pack('<H', protocol_info['id'])
    packet += dict_to_packet(name, data)
    packet += bitstring.pack('<H',
                             helpers.get_crc16_checksum(packet.tobytes()))
    return packet.tobytes()


class NodeBridgeProtocol():
    def __init__(self, source_list=None):
        self.data = {}
        self.seq = 0
        self.packet_buffer = bytes()

    def update(self, packet, verbose=False):
        self.packet_buffer += packet
        unpack_result = unpack(self.packet_buffer)
        self.packet_buffer = unpack_result['unused_packet']
        if 'result' in unpack_result:
            result = unpack_result['result']
            result['data']['seq'] = result['seq']
            self.seq = result['seq']
            self.data[result['name']] = result['data']
        if verbose:
            if 'error' in unpack_result:
                print('ERR Protocol: ', unpack_result['error'])
            elif 'result' in unpack_result:
                print(
                    'LOG Protocol: ', unpack_result['result']['name'], unpack_result['result']['data'])
        return unpack_result


if __name__ == '__main__':
    # pack & unpack
    test_data = create_protocol_data('autoaimData')
    test_data['fire'] = True
    test_data['pitch_angle_diff'] = 1.11
    test_data['yaw_angle_diff'] = 2.22
    test_packet = pack('autoaimData', test_data)
    print('pack result: ', test_packet)
    # test with some useless bytes
    print('unpack result: ', unpack(
        bytes([1, 2, 3])+test_packet+bytes([4, 5, 6])))
