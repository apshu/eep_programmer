import sys
from enum import IntEnum
from time import sleep

try:
    import hid
except ImportError as e:
    print(e.msg, file=sys.stderr)
    print('hidapi module missing. Install with "python -m pip install hidapi" command', file=sys.stderr)
    exit(0)


class MCP2221AException(Exception):
    pass


class _SFR(object):
    def __init__(self, address=None, init_value=0, reader_func=None, changed_func=None):
        self._sfrvalue = init_value
        self._address = address
        self._reader_func = reader_func
        self._changed_func = changed_func

    def _change_bit(self, bit_number, new_value):
        if -1 < bit_number < 8:
            if new_value:
                self.as_byte |= 1 << bit_number
            else:
                self.as_byte &= ~(1 << bit_number)

    def _get_bit(self, bit_number):
        return 1 if self.as_byte & (1 << bit_number) else 0

    @property
    def address(self):
        return self._address

    @property
    def as_byte(self):
        if callable(self._reader_func):
            return self._reader_func() & 0xFF
        return self._sfrvalue & 0xFF

    @as_byte.setter
    def as_byte(self, new_value):
        new_value &= 0xFF
        old_value = self.as_byte
        if new_value != old_value:
            if callable(self._changed_func):
                self._sfrvalue = self._changed_func(new_value)
            else:
                self._sfrvalue = new_value

    def __repr__(self):
        return f'<{self.__class__.__name__}{self.address}=0x{self.as_byte:02X}>'


class MCP2221A(object):
    class _GPIO_PIN(_SFR):
        @staticmethod
        def __tristate(value):
            return value if 0 <= value <= 1 else -1

        @property
        def DIRECTION(self):
            return self.__tristate((self.as_byte >> 2) & 0x03)

        @DIRECTION.setter
        def DIRECTION(self, new_value):
            self._change_bit(3, self.__tristate(new_value) != new_value)
            self._change_bit(2, new_value & 1)

        @property
        def PINSTATE(self):
            return self.__tristate(self.as_byte & 0x03)

        @PINSTATE.setter
        def PINSTATE(self, new_value):
            self._change_bit(1, self.__tristate(new_value) != new_value)
            self._change_bit(0, new_value & 1)

        @property
        def PINFUNCTION(self):
            return (self.as_byte >> 4) & 0x07

        @PINFUNCTION.setter
        def PINFUNCTION(self, new_value):
            self.as_byte = (self.as_byte & 0x0F) | ((new_value & 0x07) << 4)

        def __repr__(self):
            if self.PINFUNCTION:
                pinfunc = f'non GPIO (0x{self.PINFUNCTION:X})'
            else:
                pinfunc = f'GPIO{("cfg ERROR", "out", "in")[self.DIRECTION + 1]} = {("cfg ERROR", "LOW", "HIGH")[self.PINSTATE + 1]}'
            return f'<{self.__class__.__name__}{self.address}: {pinfunc}>'

    class _GPSETTING(_SFR):
        @property
        def as_byte(self):
            return super().as_byte & 0x1F

        @as_byte.setter
        def as_byte(self, new_value):
            super(__class__, self.__class__).as_byte.__set__(self, new_value & 0x1F)

        def __setattr__(self, key, value):
            # write only fields
            bit = -1
            if key == 'GPIOOUTVAL':
                bit = 4
            elif key == 'GPIODIR':
                bit = 3
            else:
                super().__setattr__(key, value)
            self._change_bit(bit, value)

        @property
        def GPDES(self):
            return self.as_byte & 0x07

        @GPDES.setter
        def GPDES(self, new_value):
            self.as_byte = (self.as_byte & 0x07) | (new_value & 0x07)

        def __repr__(self):
            return f'<{self.__class__.__name__}{self.address}=0x{self.as_byte:02X} ( GPDES={self.GPDES} )>'

    class MCP2221status(IntEnum):
        IDLE = 0
        START_TOUT = 0x12
        RSTART_TOUT = 0x17
        WRADDRL_WSEND = 0x21
        WRADDRL_TOUT = 0x23
        WRADDRL_NACK = 0x25
        RESP_I2C_MULTIDATA_SENDING = 0x41
        RESP_I2C_MULTIDATA_IDLE = 0x43
        RESP_I2C_WRDATA_TOUT = 0x44
        STATE_NO_STOP_IDLE = 0x45
        RESP_I2C_RDDATA_TOUT = 0x52
        RESP_I2C_STOP_TOUT = 0x62
        RESP_INCOMPLETE_DATA = 127
        RDADDR_NAK = -1
        STATE_ADDRESS_NAK = -2
        STATE_BUSY = -3
        INVALID_I2C_ADDRESS = -4
        INVALID_I2C_DATA = -5
        I2C_DATA_TOO_LONG = -6
        I2C_DATA_TOO_SHORT = -7
        INVALID_HID_DATA = -8
        RESP_UNKNOWN_STATUS = -10000

        @classmethod
        def _missing_(cls, value):
            return cls.RESP_UNKNOWN_STATUS

        @property
        def isNAK(self):
            return self in (self.WRADDRL_NACK, self.STATE_ADDRESS_NAK, self.RESP_I2C_RDDATA_TOUT, self.RDADDR_NAK)

        @property
        def isIdle(self):
            return self in (self.IDLE, self.RESP_I2C_MULTIDATA_IDLE, self.STATE_NO_STOP_IDLE)

    def __init__(self, VID=0x04D8, PID=0x00DD, devnum=0, path=''):
        self._iic_speed = None
        self.mcp2221a = hid.device()
        if path:
            self.mcp2221a.open_path(path)
        else:
            enumMCPS = hid.enumerate(VID, PID)
            if len(enumMCPS) < 1:
                raise MCP2221AException(f'{VID:04X}:{PID:04X} MCP2221A device not found')
            if len(enumMCPS) <= devnum:
                raise MCP2221AException(f'Connect to {VID:04X}:P{PID:04X}[#{devnum}] MCP2221A device not possible, only {len(enumMCPS)} devices found.')
            self.mcp2221a.open_path(enumMCPS[devnum]["path"])
        self.__gpio = tuple([self._GPIO_PIN(addr, 0xFF) for addr in range(4)])
        self.pins_read()

    @property
    def GPIO(self):
        return self.__gpio

    def _read_RAM_settings(self):
        return self._get_command_response([0, 0x61])

    def pins_read(self) -> bool:
        RAM_settings = self._read_RAM_settings()
        if RAM_settings[1]:
            return False
        rbuf = self._get_command_response([0, 0x51])
        if rbuf[1]:
            return False
        if rbuf[1] == 0:
            for gpio_id in range(4):
                self.GPIO[gpio_id].PINSTATE = rbuf[2 + 2 * gpio_id]
                self.GPIO[gpio_id].DIRECTION = rbuf[3 + 2 * gpio_id]
                self.GPIO[gpio_id].PINFUNCTION = RAM_settings[22 + gpio_id] & 0x07
        return True

    def pins_write(self) -> bool:
        sram_cmd_buffer = [0, 0x60, 0, 0, 0, 0, 0, 0, 0xFF]
        gpio_cmd_buffer = [0, 0x50, 0]
        for gpio_id, gpio in enumerate(self.GPIO):
            gpsetting = self._GPSETTING(gpio_id, 0)
            gpsetting.GPDES = gpio.PINFUNCTION
            gpsetting.GPIOOUTVAL = gpio.PINSTATE
            gpsetting.IODIR = gpio.DIRECTION
            sram_cmd_buffer.append(gpsetting.as_byte)
            gpio_cmd_buffer.append(0 if gpio.DIRECTION else 0xFF)
            gpio_cmd_buffer.append(0xFF if gpio.PINSTATE else 0)
            gpio_cmd_buffer.append(0 if gpio.PINFUNCTION else 0xFF)
            gpio_cmd_buffer.append(0xFF if gpio.DIRECTION else 0)
        srbuf = self._get_command_response(sram_cmd_buffer)
        gpbuf = self._get_command_response(gpio_cmd_buffer)
        return not bool(srbuf[1] or gpbuf[1])

    def reset(self):  # Disconnect, Reset and Reconnect
        self.mcp2221a.write([0x00, 0x70, 0xAB, 0xCD, 0xEF] + [0] * 60)
        self.mcp2221a = None  # Disconnected from the system
        sleep(1)

    @staticmethod
    def _iic_divider_to_speed(divider):
        return 12000000 / (divider + 3)

    @property
    def _iic_speed_divider(self):
        return self.iic_status_read()[14]

    @property
    def iic_comm_speed(self):
        return self._iic_divider_to_speed(self._iic_speed_divider)

    # returns the speed set
    # may throw exception
    def iic_init(self, speed=0) -> int:  # default clock speed = 100kHz
        speed = max(speed, 0) or self.iic_comm_speed or 1
        buf = [0x00, 0x10, 0x00, 0x00]
        # The I2C/SMBus system clock divider that will be used to establish the communication speed
        speed_divider = int((12000000 / speed) - 3)
        speed_divider = max(min(speed_divider, 255), 0)  # limit to 0...255
        if self._iic_speed_divider != speed_divider:
            buf.append(0x20)
            buf.append(speed_divider)
        buf.extend([0] * (65 - len(buf)))
        for iter in range(10):
            rbuf = self._get_command_response(buf)
            if rbuf[1] != 0:
                # Command processing busy
                sleep(0.1)  # wait 100msec
                continue  # retry
            if buf[4] and (rbuf[3] != buf[4]) or rbuf[8]:
                # set speed requested, unable to execute request
                buf[3] = 0x10 if buf[3] == 0 else 0  # Toggle comm cancel request command
                sleep(0.1)  # wait 100msec
                continue  # retry
            if rbuf[22] == 0 or rbuf[23] == 0:
                raise MCP2221AException(f'I²C bus lines invalid state:{(" SCL is low", "")[bool(rbuf[22])]}{(" SDA is low", "")[bool(rbuf[23])]}.')
            if rbuf[3] != buf[4]:
                raise MCP2221AException(f'I²C speed not set, error code 0x{rbuf[3]:02X}')
            if rbuf[3]:
                if rbuf[4] != speed_divider or rbuf[14] != speed_divider:
                    raise MCP2221AException(f'I²C incorrect speed set by the device')
            divider_now = rbuf[14]
            return self._iic_divider_to_speed(divider_now)
        if rbuf[1] != 0:
            raise MCP2221AException('Command 0x10 not accepted')
        raise MCP2221AException(f'Incorrect response to I²C Init command, status={self.i2c_state.name}')

    @property
    def iic_is_scl_high(self):
        return bool(self.iic_status_read()[22])

    @property
    def iic_is_sda_high(self):
        return bool(self.iic_status_read()[23])

    @property
    def i2c_state(self) -> MCP2221status:
        return self.MCP2221status(self.iic_status_read()[8])

    def iic_status_read(self):
        return self._get_command_response([0x00, 0x10])

    def _get_command_response(self, data_out, max_retries=10) -> bytes:
        max_retries = min(max_retries, 1)
        if data_out[0] != 0:
            raise MCP2221AException('Invalid HID buffer format')
        data_out.extend([0] * (65 - len(data_out)))
        for iter in range(max_retries):
            self.mcp2221a.write(data_out)
            rbuf = self.mcp2221a.read(65)
            if rbuf[0] == data_out[1]:
                # got reply for requested command
                return rbuf
        raise MCP2221AException(f'HID Communication out of sync after {max_retries} {("retries", "retry")[max_retries == 1]}.')  # Repair with Device Reset command

    def _i2c_write(self, command, address, data):
        is_address_10bit = address & 0x7C00 == 0x7800
        status = self.MCP2221status.IDLE
        data_bytes_to_send = len(data)
        if data_bytes_to_send > (65534 if is_address_10bit else 65535):  # Reserve one byte for 10bit
            status = self.MCP2221status.I2C_DATA_TOO_LONG
        else:
            if address < 0 or (address > 0x7F and not is_address_10bit) or address > 0x7BFF:
                # invalid 7bit or 10bit address
                status = self.MCP2221status.INVALID_I2C_ADDRESS
            else:
                if data and (max(data) > 255 or min(data) < 0):
                    # data is outside of 0..255 range
                    status = self.MCP2221status.INVALID_I2C_DATA
                else:
                    length_lo = data_bytes_to_send & 0xFF
                    length_hi = (data_bytes_to_send >> 8) & 0xFF
                    send_address = True
                    source_ptr = 0
                    if not self.i2c_state.isIdle:
                        self.iic_init()
                    while data_bytes_to_send or send_address:
                        hid_write_buffer = [0, command, length_lo, length_hi]
                        send_address = False
                        if is_address_10bit:
                            hid_write_buffer.append((address >> 7) & 0xFE)
                            hid_write_buffer.append(address & 0xFF)
                            address = (address >> 7) & 0xFE
                            is_address_10bit = False
                        else:
                            hid_write_buffer.append(address << 1)
                        while (data_bytes_to_send):
                            if len(hid_write_buffer) >= 65:
                                break
                            hid_write_buffer.append(data[source_ptr])
                            source_ptr += 1
                            data_bytes_to_send -= 1
                        hid_write_buffer.extend([0] * (65 - len(hid_write_buffer)))
                        status = self._mcp_wait_send(hid_write_buffer)
                        if status.isIdle:
                            continue
                        break
        return status

    def _mcp_wait_send(self, hid_buffer, max_retries=100):
        max_retries = min(max_retries, 1)
        status = self.MCP2221status.RESP_UNKNOWN_STATUS
        for iter in range(max_retries):
            rbuf = self._get_command_response(hid_buffer)
            if rbuf[1] == 0:
                # No error, get I2C status
                for iter in range(100):
                    rbuf = self.iic_status_read()  # with the check we also wait a milisecond.
                    if rbuf[1]:
                        status = self.MCP2221status.STATE_BUSY
                        continue
                    if rbuf[20] & 0x40:
                        status = self.MCP2221status.STATE_ADDRESS_NAK
                        break
                    status = self.MCP2221status(rbuf[8])
                    if status == self.MCP2221status.RESP_I2C_MULTIDATA_SENDING:
                        # sending multidata chunk
                        sleep(0.01)  # Wait
                        continue  # retry
                    if status == self.MCP2221status.RESP_UNKNOWN_STATUS:
                        # Unknown state - maybe new FW in MCP2221
                        sleep(0.01)  # Wait
                        continue  # retry
                    return status
                return status
            else:
                status = self.MCP2221status(rbuf[2])
                if status != self.MCP2221status.RESP_UNKNOWN_STATUS:
                    # Known error, end
                    return status
                sleep(0.01)  # Wait
                continue  # Retry
        return status

    def _i2c_read(self, command, address, num_bytes_to_read, max_retries=100):
        is_address_10bit = address & 0x7C00 == 0x7800
        status = self.MCP2221status.IDLE
        if num_bytes_to_read > 65535 or num_bytes_to_read < 1:
            status = self.MCP2221status.I2C_DATA_TOO_SHORT if num_bytes_to_read < 1 else self.MCP2221status.I2C_DATA_TOO_LONG
        else:
            if address < 0 or (address > 0x7F and not is_address_10bit) or address > 0x7BFF:
                # invalid 7bit or 10bit address
                status = self.MCP2221status.INVALID_I2C_ADDRESS
            else:
                if not self.i2c_state.isIdle:
                    self.iic_init()
                if is_address_10bit:
                    self.iic_write_no_stop(address >> 8, [address & 0xff])  # write 10bit address
                    address >>= 8  # Keep only the upper byte for 10bit address read command
                    command = 0x93  # Repeated start condition read
                buf = [0, command, num_bytes_to_read & 0xFF, (num_bytes_to_read >> 8) & 0xFF, (address << 1) | 1]
                rbuf = self._get_command_response(buf)
                status = self.MCP2221status(rbuf[2 if rbuf[1] else 8])
                if status.isIdle:
                    # Read command accepted
                    incoming_data = []
                    retries = max_retries
                    while retries > 0:
                        rbuf = self._get_command_response([0, 0x40])
                        if rbuf[2] == 0x25:
                            # Read address NAK
                            status = self.MCP2221status.RDADDR_NAK
                            break
                        if rbuf[1]:
                            status = self.MCP2221status(rbuf[2])
                            if status == self.MCP2221status.RESP_UNKNOWN_STATUS:
                                status = self.MCP2221status.STATE_BUSY
                                retries -= 1  # One try less
                                sleep(0.01)  # wait
                                continue
                            break
                        if rbuf[3] == 127:
                            # Error signalled
                            status = self.MCP2221status.RESP_INCOMPLETE_DATA
                            retries -= 1  # One try less
                            sleep(0.01)  # wait
                            break
                        if rbuf[3] > 60:
                            # Data error
                            status = self.MCP2221status.INVALID_HID_DATA
                            break
                        else:
                            retries = min(retries + 1, max_retries)
                            incoming_data.extend(rbuf[4:rbuf[3] + 4])
                            if len(incoming_data) >= num_bytes_to_read:
                                if len(incoming_data) == num_bytes_to_read:
                                    status = incoming_data
                                if len(incoming_data) > num_bytes_to_read:
                                    status = self.MCP2221status.I2C_DATA_TOO_LONG
                                break
        return status

    # Write data to 7bit or 10bit I²C address
    def iic_write(self, address, data=None):
        data = data or b''
        return self._i2c_write(0x90, address=address, data=data)

    # Write data to 7bit or 10bit I²C address, without issuing stop condition
    def iic_write_no_stop(self, address, data=None):
        data = data or b''
        return self._i2c_write(0x94, address=address, data=data)

    # Write data to 7bit or 10bit I²C address, with start/repeated start condition
    def iic_write_repeated(self, address, data=None):
        data = data or b''
        return self._i2c_write(0x92, address=address, data=data)

    # Read minimum 1byte from 7bit or 10bit address
    def iic_read(self, address, num_bytes_to_read=1):
        return self._i2c_read(0x91, address, num_bytes_to_read)

    # Read minimum 1byte from 7bit or 10bit address, with repeated start condition
    def iic_read_repeated(self, addrs, num_bytes_to_read=1):
        return self._i2c_read(0x93, addrs, num_bytes_to_read)

    @staticmethod
    def as_10bit_iic_address(address):
        return (address & 0x3FF) | 0x7800

    @staticmethod
    def as_7bit_iic_address(address):
        return address & 0x7F

    def adc_read_raw_live(self):
        rbuf = self.iic_status_read()
        if rbuf[1]:
            return -1, -1, -1
        return rbuf[50] + (rbuf[51] << 8), rbuf[52] + (rbuf[53] << 8), rbuf[54] + (rbuf[55] << 8)

    def adc_read_voltage_live(self, VDD=5.0):
        raw_adc_values = self.adc_read_raw_live()
        if min(raw_adc_values) < 0:
            return float('nan'), float('nan'), float('nan')
        rbuf = self._read_RAM_settings()
        if rbuf[1]:
            return float('nan'), float('nan'), float('nan')
        vref = (VDD, 1.024, 2.048, 4.096)[(rbuf[7] >> 3) & 3] if rbuf[7] & 4 else VDD
        multiplier = vref / 1023
        return tuple([adraw * multiplier for adraw in raw_adc_values])


if __name__ == '__main__':
    iic_adapter = MCP2221A()
    print(iic_adapter.adc_read_raw_live())
    print(iic_adapter.adc_read_voltage_live())

    # for iter in range(3):
    #     iic_adapter = MCP2221A()
    #     try:
    #         print('Speed:', iic_adapter.I2C_Init(400000) / 1000, 'kHz')
    #     except MCP2221AException as e:
    #         print(e)
    #         iic_adapter.Reset()
    #         sleep(10)
    #     else:
    #         import secrets
    #
    #         outData = list(secrets.token_bytes(256))
    #         print(iic_adapter.I2C_Write(0x50, [0, 0] + outData))
    #         for iter in range(10):
    #             wr_status = iic_adapter.I2C_Write(0x50, [0, 0])
    #             print(wr_status)
    #             if not wr_status.isNAK:
    #                 retVal = iic_adapter.I2C_Read(0x50, len(outData))
    #                 if retVal == outData:
    #                     print(f'\N{check mark} ', end='')
    #                 else:
    #                     print(f'\N{ballot x} ', end='')
    #                 print(retVal)
    #
    #                 print(iic_adapter.I2C_Read(0x50, 256))
    #                 break
    #         break
