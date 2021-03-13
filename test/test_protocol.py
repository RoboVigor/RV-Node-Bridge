
    # protocol_info_table = {
    #     'autoaim': {'id': 0x401, 'description':'for test', 'size':24, 'struct':{
    #         'x':   {'format':'floatle:32'},
    #         'y':   {'format':'floatle:32'},
    #         'z':   {'format':'floatle:32', 'arraysize':2},
    #         'test':{'struct': 'Test_Type', 'arraysize':2}
    #     }},
    #     'Test_Type': {'size':4, 'struct':{
    #         'x':   {'format':'floatle:32'}
    #     }},
    # }
    # test_data = {'x':1.11,'y':2.22,'z':[3.33,4.44],'test':[{'x':5.55},{'x':6.66}]}
    # test_packet = pack('autoaim', test_data)
    # print('pack result: ', test_packet)
    # print('unpack result: ', unpack(test_packet))