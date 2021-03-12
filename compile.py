import importlib
import re
import math
import os

class NodeBridge():
    
    def __init__(self, source_list=None):
        if source_list:
            data_list = []
            for source in source_list:
                data = self.load(source, 'yaml')
                self.parse_stm32(data)
                data_list += [data]
            data = self.merge_data(data_list)
            self.generate_stm32(data)

    def load(self, source, type):
        if type=='yaml':
            with open(source, 'r', encoding="utf-8") as file:
                yaml = importlib.import_module('yaml')
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

    typeInfoTable = {
        "uint8_t":{"ros":"", "size":1},
        "uint16_t":{"ros":"", "size":2},
        "uint32_t":{"ros":"", "size":4},
        "uint64_t":{"ros":"", "size":8},
        "int":{"ros":"", "size":2},
        "int8_t":{"ros":"", "size":1},
        "int16_t":{"ros":"", "size":2},
        "int32_t":{"ros":"", "size":4},
        "int64_t":{"ros":"", "size":8},
        "float":{"ros":"", "size":4},
        "char":{"ros":"", "size":1},
    }
    
    def add_type(self, type_name, size, ros=""):
        self.typeInfoTable[type_name] = {"ros":ros, "size":size}

    def merge_data(self, data_list):
        data = data_list[0]
        for data_merge in data_list[1:]:
            for key in ['source', 'version', 'structs', 'protocols']:
                data[key] += data_merge[key]
        return data

    def parse_stm32(self, data):
        offset = 0
        structs = []
        protocols = []
        # parse structs
        for struct in data['structs']:
            struct.update(self.parse_struct_stm32(struct['struct']))
            self.add_type(struct['name'], struct['size'])
        # parse protocols
        data['protocols'] = [x for x in data['protocols'] if x['stm32_receive'] or x['stm32_send']]
        for protocol in data['protocols']:
            protocol['name'] = protocol['name'] + '_t'
            protocol.update(self.parse_struct_stm32(protocol['struct']))
    
    def parse_struct_stm32(self, struct_string, register_struct_name=''):
        r = re.compile('\n*?(?P<type>\S*?)\s*?(?P<key>\S*?)\[?(?P<arraysize>\d*?)\]?\s*?:?\s*?(?P<bitsize>\d*?);')
        struct = [m.groupdict() for m in r.finditer(struct_string)]
        bitsize = 0
        for item in struct:
            item['typeinfo'] = self.typeInfoTable[item['type']]
            bitsize += int(item['bitsize'] or '0') or (item['typeinfo']['size'] * int(item['arraysize'] or '1') * 8)
        return {"size": math.ceil(bitsize/8)}

    def generate_stm32(self, data):
        template_data = {}
        # concat data
        template_data['config_version'] = "\n".join([' * @version {} v{}'.format(*x) for x in zip(data['source'], data['version'])])
        # load template
        template_protocol = self.load('template/protocol.h.txt', 'plaintext')
        template_union = self.load('template/union.txt', 'plaintext')
        template_struct = self.load('template/struct.txt', 'plaintext')
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
        with open('dist/protocol.h', 'w', encoding="utf-8") as file:
            file.write(template_protocol.format(**template_data))


if __name__ == '__main__':
    nb = NodeBridge(['src/judge.yml','src/host.yml','src/user.yml'])
