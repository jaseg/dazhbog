#!/usr/bin/env python3
import re
import os

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('cmsis_device_header', nargs='+', type=argparse.FileType('rb'))
    args = parser.parse_args()
    
    print('#ifndef __GENERATED_CMSIS_HEADER_EXPORTS__')
    print('#define __GENERATED_CMSIS_HEADER_EXPORTS__')
    print()
    for header in args.cmsis_device_header:
        lines = header.readlines()
        name = os.path.basename(header.name)
        print('#include <{}>'.format(name))
        print()

        print('/* {} */'.format(name))
        for l in lines:
            match = re.match(b'^#define (\w+)\s+\W*(\w+_TypeDef|\w+_Type).*$', l)
            if match:
                inst, typedef = match.groups()
                inst, typedef = inst.decode(), typedef.decode()
                print('{} *{} = {};'.format(typedef, inst.lower(), inst))
        print()
    print('#endif//__GENERATED_CMSIS_HEADER_EXPORTS__')

