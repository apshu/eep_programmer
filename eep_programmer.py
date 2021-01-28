import argparse
import math
import os.path
import sys
import time

try:
    import hid
except ImportError as e:
    print(e.msg, file=sys.stderr)
    print('hidapi module missing. Install with "python -m pip install hidapi" command', file=sys.stderr)
    exit(0)
try:
    import intelhex
except ImportError as e:
    print(e.msg, file=sys.stderr)
    print('intelhex module missing. Install with "python -m pip install intelhex" command', file=sys.stderr)
    exit(0)

import MCP2221A as MCP2221A


class IICEEPROM(object):
    def __init__(self, mcp):
        self.mcp: MCP2221A.MCP2221A = mcp
        self.page_size = 256
        self.iic_address = 0x50
        self.read_retries = 5
        self.write_retries = 10

    def data_to_address(self, address, data):
        return False

    def read_from_address(self, address, num_bytes):
        return b''


class EEPROM_64k(IICEEPROM):
    def get_address_bytes(self, address):
        start_address = f'{address:05x}'
        address_L = int(start_address[3:5], 16)
        address_H = int(start_address[1:3], 16)
        address_U = int(start_address[0], 16) & 3
        return address_L, address_H, address_U

    def data_to_address(self, address, data):
        if len(data) <= self.page_size:
            address_L, address_H, address_U = self.get_address_bytes(address)
            for retries in range(self.write_retries):
                status = self.mcp.iic_write(self.mcp.as_7bit_iic_address(self.iic_address | address_U), [address_H, address_L] + list(data))
                if status.isIdle:
                    break
            return status.isIdle
        return False

    def read_from_address(self, address, num_bytes):
        result = b''
        if num_bytes:
            address_L, address_H, address_U = self.get_address_bytes(address)
            for retries in range(self.read_retries):
                status = self.mcp.iic_write(self.mcp.as_7bit_iic_address(self.iic_address | address_U), [address_H, address_L])
                if status.isIdle:
                    result = self.mcp.iic_read(self.mcp.as_7bit_iic_address(self.iic_address | address_U), num_bytes)
                    if result is not self.mcp.MCP2221status:
                        break
                    if result is self.mcp.MCP2221status:
                        return b'', result
        return result, self.mcp.MCP2221status.IDLE


def file_to_eeprom(arguments):
    mcp = MCP2221A.MCP2221A(path=arguments.hid)
    mcp.iic_init(speed=arguments.iic_clock * 1000)
    print(f'Using USB device: {mcp.mcp2221a.get_product_string()} ({mcp.mcp2221a.get_manufacturer_string()})')
    print(f'I²C clock speed {mcp.iic_comm_speed / 1000:.1f}kHz')
    if arguments.eep_detect:
        total_memory_capacity = 0
        print('EEPROM chips (≤64k):')
        for address_U in range(0, 8):
            status = mcp.iic_write(mcp.as_7bit_iic_address(0x50 | address_U), [])
            print(f'{("  ✗ No ", " ✓ Yes")[status.isIdle]}   0x{address_U * 0x10000:06X}-0x{(address_U + 1) * 0x10000 - 1:06X}')
            if status.isIdle:
                total_memory_capacity += 0x10000
        print(f'Total memory capacity: {total_memory_capacity >> 10} kbytes')
        exit()
    ihx = intelhex.IntelHex()
    ihx.padding = 0xff
    file_type = 'hex'
    if arguments.read_file.fileno() == 0 or arguments.read_file.isatty():
        input_file_name = '<stdin>'
    else:
        file_extension = os.path.splitext(arguments.read_file.name)[1]
        file_type = 'hex' if file_extension.casefold() == '.hex'.casefold() else 'bin'
        input_file_name = f'"{os.path.realpath(arguments.read_file.name)}" file'
    print(f'Loading from {input_file_name} in {file_type} format.')

    try:
        ihx.loadfile(arguments.read_file, file_type)
    except Exception as e:
        return False, f'Error reading input file "{str(e)}"'
    if not len(ihx):
        return False, 'Empty input file'
    eep = EEPROM_64k(mcp)
    start_addr = int(min(ihx.addresses()) / eep.page_size) * eep.page_size
    last_address = math.ceil(max(ihx.addresses()) / eep.page_size) * eep.page_size
    data_length = len(ihx)
    start_time = time.monotonic()
    print('0%', end='')
    for addr in range(start_addr, last_address, eep.page_size):
        print('.', end='', flush=True)
        bytes_in_chunk = min(eep.page_size, data_length - addr)
        write_data = ihx.tobinarray(start=addr, size=bytes_in_chunk)
        if not eep.data_to_address(addr, write_data):
            return False, f'Error writing to address {addr}.'
        read_data, read_status = eep.read_from_address(addr, bytes_in_chunk)
        if not read_status.isIdle:
            return False, f'Error reading from address {addr}.'
        if write_data.tolist() != read_data:
            return False, f'Error verifying address {addr}.'
        if time.monotonic() - start_time > 5:
            start_time = time.monotonic()
            procent = int(addr * 100 / data_length)
            if procent < 100:
                print(f'{procent}%', end='', flush=True)
    print('100% ')
    return True, f'Successfully verified {len(ihx)} bytes.'


def main():
    enumMCPS = hid.enumerate(0x04D8, 0x00DD)
    parser = argparse.ArgumentParser(description='Program EEPROM with file using MCP2221 bridge.')
    parser.add_argument('-r', '--read_file', help='Input file', type=argparse.FileType('rb'), default='-')
    parser.add_argument('-i', '--hid', help='HID Path or id in the list (output of --list_hid command)', type=str, default='0')
    parser.add_argument('-l', '--list_hid', help='List I²C converter devices', action='store_true')
    parser.add_argument('-d', '--eep_detect', help='Detect connected EEPROM', action='store_true')
    parser.add_argument('-c', '--iic_clock', help='I²C clock speed 100=100kHz', type=float, default=100)
    parser.add_argument('-v', '--version', action='version', version='EEPROM programmer v1.0')
    args = parser.parse_args()
    parser.print_usage()
    if args.list_hid:
        print('Available programmer devices:')
        for idx, MCP in enumerate(enumMCPS):
            print(f'{idx}) {MCP["product_string"]} → --hid="{MCP["path"].decode()}"')
        exit()
    if args.hid.isnumeric():
        if not enumMCPS:
            print('No MCP2221 I²C converter found', file=sys.stderr)
            exit(0)
        args.hid = enumMCPS[int(args.hid)]['path']
    else:
        args.hid = args.hid.encode('ascii')
    is_success, status = file_to_eeprom(args)
    print(f'{("✗", "✓")[is_success]} {status}')


if __name__ == '__main__':
    main()
