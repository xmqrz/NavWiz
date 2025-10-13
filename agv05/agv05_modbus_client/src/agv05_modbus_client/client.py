from __future__ import absolute_import
from __future__ import unicode_literals

from pymodbus.client.sync import ModbusTcpClient
from pymodbus.constants import Defaults


class ModbusClient(object):

    def __init__(self, ip_address, port, timeout, slave_id):
        # config variable
        self.__ip_address = ip_address
        self.__port = port
        self.__slave_id = slave_id
        self.__timeout = timeout
        self.__error = None
        self.__client = None
        Defaults.Timeout = self.__timeout
        # connect
        self.__connect()

    def __connect(self):
        self.__client = ModbusTcpClient(self.__ip_address, self.__port)
        if self.__client.connect():
            self.__error = None
            return 0
        else:
            self.__error = 'Connection failed'
            return -1

    def read_coil(self, address):
        try:
            result = self.__client.read_coils(address, unit=self.__slave_id)
            assert result
            self.__error = None
            return result.bits[0]
        except Exception:
            self.__client.close()
            self.__error = 'Failed to read coil'
            return -1

    def read_discrete_input(self, address):
        try:
            result = self.__client.read_discrete_inputs(address, unit=self.__slave_id)
            assert result
            self.__error = None
            return result.bits[0]
        except Exception:
            self.__client.close()
            self.__error = 'Failed to read discrete input'
            return -1

    def write_coil(self, address, value):
        try:
            result = self.__client.write_coil(address, value, unit=self.__slave_id)
            assert result.address is not None
            self.__error = None
            return 0
        except Exception:
            self.__client.close()
            self.__error = 'Failed to write coil'
            return -1

    def read_holding_register(self, address):
        try:
            result = self.__client.read_holding_registers(address, unit=self.__slave_id)
            assert result
            self.__error = None
            return result.registers[0]
        except Exception:
            self.__client.close()
            self.__error = 'Failed to read holding register'
            return -1

    def read_input_register(self, address):
        try:
            result = self.__client.read_input_registers(address, unit=self.__slave_id)
            assert result
            self.__error = None
            return result.registers[0]
        except Exception:
            self.__client.close()
            self.__error = 'Failed to read input register'
            return -1

    def write_register(self, address, value):
        try:
            result = self.__client.write_register(address, value, unit=self.__slave_id)
            assert result.address is not None
            self.__error = None
            return 0
        except Exception:
            self.__client.close()
            self.__error = 'Failed to write register'
            return -1

    def mask_write_register(self, address, value, mask):
        try:
            and_mask = ~mask & 0xffff
            or_mask = (value & mask) & 0xffff
            result = self.__client.mask_write_register(address, and_mask, or_mask, unit=self.__slave_id)
            assert result.address is not None
            self.__error = None
            return 0
        except Exception:
            self.__client.close()
            self.__error = 'Failed to mask write register'
            return -1

    def error(self):
        return self.__error
