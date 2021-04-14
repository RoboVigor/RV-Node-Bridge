# -*- coding: utf-8 -*-
import yaml
import re
import math
import os

class NodeBridgeCompiler():

    def __init__(self, source_list=None):
        self.source_list = source_list

    def load(self, source, type):
        if type=='yaml':
            with open(source, 'r', encoding="utf-8") as file:
                file_data = file.read()
                data = yaml.safe_load(file_data)
                data['source'] = [os.path.basename(source)]
                data['version'] = [data['version']]
                return data
        elif type=='plaintext':
            with open(source, 'r', encoding="utf-8") as file:
                return file.read()
        else:
            return None

    type_info_table = {
        "uint8_t":  {"size":1, "python":"uintle", "ros":"uint8"},
        "uint16_t": {"size":2, "python":"uintle", "ros":"uint16"},
        "uint32_t": {"size":4, "python":"uintle", "ros":"uint32"},
        "uint64_t": {"size":8, "python":"uintle", "ros":"uint64"},
        "int":      {"size":2, "python":"intle", "ros":"int16"},
        "int8_t":   {"size":1, "python":"intle", "ros":"int8"},
        "int16_t":  {"size":2, "python":"intle", "ros":"int16"},
        "int32_t":  {"size":4, "python":"intle", "ros":"int32"},
        "int64_t":  {"size":8, "python":"intle", "ros":"int64"},
        "float":    {"size":4, "python":"floatle", "ros":"float32"},
        "double":   {"size":8, "python":"floatle", "ros":"float64"},
        "char":     {"size":1, "python":"uintle", "ros":"uint8"},
    }

    def add_type(self, type_name, size, **args):
        self.type_info_table[type_name] = {"size":size}
        self.type_info_table[type_name].update(args)

    def merge_data(self, data_list):
        data = data_list[0]
        for data_merge in data_list[1:]:
            for key in ['source', 'version', 'structs', 'protocols']:
                data[key] += data_merge[key]
        return data

    def parse_struct(self, struct_string, register_struct_name=''):
        r = re.compile('\n*?(?P<type>\S*?)\s*?(?P<key>\S*?)(?:\[(?P<arraysize>\d*?)\])?(?:\s?:\s?(?P<bitsize>\d*?))?;')
        items = [m.groupdict() for m in r.finditer(struct_string)]
        bitsize = 0
        for item in items:
            item['arraysize'] = int(item['arraysize']) if item['arraysize'] else None
            item['typeinfo'] = self.type_info_table[item['type']]
            bitsize += int(item['bitsize'] or '0') or (item['typeinfo']['size'] * int(item['arraysize'] or '1') * 8)
        return {"size": math.ceil(bitsize/8), "items": items}

    def compile(self, target):
        source_list = self.source_list
        compile_functions_table = {
            'stm32':     {'parse': self.parse_stm32, 'generate': self.generate_stm32},
            'python': {'parse': self.parse_python, 'generate': self.generate_python},
            'ros':       {'parse': self.parse_ros, 'generate': self.generate_ros},
        }
        compile_functions = compile_functions_table[target]
        if source_list:
            data_list = []
            for source in source_list:
                data = self.load(source, 'yaml')
                compile_functions['parse'](data)
                data_list += [data]
            data = self.merge_data(data_list)
            compile_functions['generate'](data)

    def parse_python(self, data):
        # parse structs
        for struct in data['structs']:
            struct.update(self.parse_struct(struct['struct']))
            self.add_type(struct['name'], struct['size'], python="bits")
        # parse protocols
        for protocol in data['protocols']:
            protocol.update(self.parse_struct(protocol['struct']))
        # generate bitstring format
        for struct in data['structs']+data['protocols']:
            for item in struct['items']:
                item['format'] = "{}:{}".format(item['typeinfo']['python'], (item['bitsize'] or item['typeinfo']['size']*8))
        return data

    def generate_python(self, data):
        def generate_item_struct(item):
            if len([x for x in data['structs'] if x['name']==item['type']])>0:
                dict_items = [('key',item['key']), ('struct',item['type']), ('arraysize',item['arraysize'])]
            else:
                dict_items = [('key',item['key']), ('format',item['format']), ('arraysize', item['arraysize'])]
            return dict([x for x in dict_items if x[1]])

        protocol_info_table = {}
        for struct in data['structs']:
            protocol_info_table[struct['name']] = {
                'size': struct['size'],
                'struct': [generate_item_struct(x) for x in struct['items']],
            }
        for protocol in data['protocols']:
            for (interface_key,interface_id) in [y for x in protocol['interface'] for y in x.items()]:
                protocol_info_table[interface_key] = {
                    'id': interface_id,
                    'size': protocol['size'],
                    'description': protocol['description'],
                    'struct': [generate_item_struct(x) for x in protocol['items']],
                }
        with open(os.path.dirname(__file__)+'/../dist/protocol.yaml', 'w', encoding='utf-8') as file:
            yaml.dump(protocol_info_table, file, default_flow_style=False, encoding='utf-8', allow_unicode=True)

    def parse_ros(self, data):
        # parse structs
        for struct in data['structs']:
            struct.update(self.parse_struct(struct['struct']))
            self.add_type(struct['name'], struct['size'], ros=struct['name'])
        # parse protocols
        for protocol in data['protocols']:
            protocol.update(self.parse_struct(protocol['struct']))

    def generate_ros(self, data):
        def generate_msg(name, items, description='', id=''):
            msg_content = '# protocol_name: {}'.format(name)
            msg_content += '\n# protocol_id: {:#06X}'.format(id) if id else ''
            msg_content += '\n# protocol_description: {}'.format(description) if description else ''
            msg_content += '\n'
            for item in items:
                arraysize = '[{}]'.format(item['arraysize']) if item['arraysize'] else ''
                msg_content += '\n{}{} {}'.format(item['typeinfo']['ros'], arraysize, item['key'])
            with open(os.path.dirname(__file__)+'/../dist/msg/{}.msg'.format(name), 'w', encoding='utf-8') as file:
                file.write(msg_content)
        msg_files = []
        for struct in data['structs']:
            msg_files += [struct['name']]
            generate_msg(struct['name'], struct['items'])
        for protocol in data['protocols']:
            for (interface_key,interface_id) in [y for x in protocol['interface'] for y in x.items()]:
                msg_files += [interface_key]
                generate_msg(interface_key, protocol['items'], id=interface_id, description=protocol['description'])
        with open(os.path.dirname(__file__)+'/../dist/CMakeLists.txt', 'w', encoding='utf-8') as file:
            file.write('add_message_files(\n  FILES\n'+'\n'.join(['  {}.msg'.format(x) for x in msg_files])+'\n)')

    def parse_stm32(self, data):
        # parse structs
        for struct in data['structs']:
            struct.update(self.parse_struct(struct['struct']))
            self.add_type(struct['name'], struct['size'])
        # parse protocols
        data['protocols'] = [x for x in data['protocols'] if x['stm32_receive'] or x['stm32_send']]
        for protocol in data['protocols']:
            protocol['name'] = protocol['name'] + '_t'
            protocol.update(self.parse_struct(protocol['struct']))
        return data

    def generate_stm32(self, data):
        template_data = {}
        # concat data
        template_data['config_version'] = "\n".join([' * @version {} v{}'.format(*x) for x in zip(data['source'], data['version'])])
        # load template
        template_protocol = self.load(os.path.dirname(__file__)+'/../template/protocol.h.txt', 'plaintext')
        template_union = self.load(os.path.dirname(__file__)+'/../template/union.txt', 'plaintext')
        template_struct = self.load(os.path.dirname(__file__)+'/../template/struct.txt', 'plaintext')
        # generate content
        protocol_info = [list(y.values())+[x['size'], int(x['stm32_receive'])] for x in data['protocols'] for y in x['interface']]
        protocol_data_size = sum([x['size']*len(x['interface']) for x in data['protocols']])
        protocol_data_struct = "\n".join(["{} {}; // {:#06X} {}".format(x['name'], list(y.keys())[0], list(y.values())[0], x['description']) for x in data['protocols'] for y in x['interface']])
        protocol_data = {'size':protocol_data_size, 'name':'ProtocolData_Type', 'struct':protocol_data_struct}
        # fix indent
        for info in [(data['structs'], 4), (data['protocols'], 8), ([protocol_data], 8)]:
            for item in info[0]:
                item['struct'] = item['struct'].replace('\n', '\n'+' ' * info[1])
        # fill template
        template_data['structs'] = "\n\n".join([template_struct.format(**x) for x in data['structs']])
        template_data['protocols'] = "\n\n".join([template_union.format(**x) for x in data['protocols']])
        template_data['protocol_info'] = ",".join(["{{{:#06X},{},{}}}".format(*x) for x in protocol_info])
        template_data['protocol_data'] = template_union.format(**protocol_data)
        with open(os.path.dirname(__file__)+'/../dist/protocol.h', 'w', encoding="utf-8") as file:
            file.write(template_protocol.format(**template_data))


if __name__ == '__main__':
    ws = os.path.dirname(__file__)+'/../'
    compiler = NodeBridgeCompiler([ws+'config/judge.yml', ws+'config/host.yml', ws+'config/user.yml'])
    compiler.compile('python')
    compiler.compile('stm32')
    compiler.compile('ros')
