from __future__ import absolute_import
from __future__ import unicode_literals

from pymodbus.client.sync import ModbusTcpClient
from pymodbus.constants import Defaults


class RemoteIoDevice(object):

    def __init__(self, ip_address, port, timeout, model):
        # config variable
        self.__input_count = 0
        self.__output_count = 0
        self.__input_base = 0
        self.__output_base = 0
        self.__ip_address = ip_address
        self.__port = port
        self.__timeout = timeout
        self.__error = None
        self.__client = None
        Defaults.Timeout = self.__timeout
        # connect
        self.__connect()
        # configure model
        if model is not None:
            self.model_config(model)

    def __connect(self):
        self.__client = ModbusTcpClient(self.__ip_address, self.__port)
        if self.__client.connect():
            self.__error = None
            return 0
        else:
            self.__error = 'Connection failed'
            return -1

    def model_config(self, model):
        if model.strip(' ') == 'ADAM-6256':
            self.__input_count = 0
            self.__output_count = 16
            self.__input_base = 0
            self.__output_base = 16
            self.__error = None
            return 0
        elif model.strip(' ') == 'ADAM-6250':
            self.__input_count = 8
            self.__output_count = 7
            self.__input_base = 0
            self.__output_base = 16
            self.__error = None
            return 0
        elif model.strip(' ') in ['WISE-4050', 'WISE-4060']:
            self.__input_count = 4
            self.__output_count = 4
            self.__input_base = 0
            self.__output_base = 16
            self.__error = None
            return 0
        elif model.strip(' ') == 'WISE-4051':
            self.__input_count = 8
            self.__output_count = 0
            self.__input_base = 0
            self.__output_base = 16
            self.__error = None
            return 0
        elif model.strip(' ') == 'ADAM-6050':
            self.__input_count = 12
            self.__output_count = 6
            self.__input_base = 0
            self.__output_base = 16
            self.__error = None
            return 0
        elif model.strip(' ') == 'ADAM-6052':
            self.__input_count = 8
            self.__output_count = 8
            self.__input_base = 0
            self.__output_base = 16
            self.__error = None
            return 0
        else:
            self.__input_count = 0
            self.__output_count = 0
            self.__input_base = 0
            self.__output_base = 0
            self.__error = 'Invalid Model'
            return -1

    def input_count(self):
        return self.__input_count

    def output_count(self):
        return self.__output_count

    def error(self):
        return self.__error

    def read_inputs(self):
        if not self.__input_count:
            return 0
        try:
            result = self.__client.read_coils(self.__input_base, self.__input_count)
            assert result
            v = 0
            for pin in range(self.__input_count):
                if result.bits[pin]:
                    v |= 1 << pin
            self.__error = None
            return v
        except Exception:
            self.__client.close()
            self.__error = 'Read inputs failed'
            return -1

    def read_outputs(self):
        if not self.__output_count:
            return 0
        try:
            result = self.__client.read_coils(self.__output_base, self.__output_count)
            assert result
            v = 0
            for pin in range(self.__output_count):
                if result.bits[pin]:
                    v |= 1 << pin
            self.__error = None
            return v
        except Exception:
            self.__client.close()
            self.__error = 'Read outputs failed'
            return -1

    def mask_write_outputs(self, data, mask):
        if not self.__output_count:
            self.__error = 'No output from current configured Model.'
            return -1
        try:
            start = -1
            states = []
            v = 1
            for pin in range(self.__output_count):
                if v & mask:
                    if start < 0:
                        start = pin
                    states.append(bool(v & data))
                else:
                    if start >= 0:
                        if len(states) == 1:
                            result = self.__client.write_coil(self.__output_base + start, states[0])
                        else:
                            result = self.__client.write_coils(self.__output_base + start, states)
                        assert result.address is not None
                        start = -1
                        states = []
                v <<= 1

            if start >= 0:
                if len(states) == 1:
                    result = self.__client.write_coil(self.__output_base + start, states[0])
                else:
                    result = self.__client.write_coils(self.__output_base + start, states)
                assert result.address is not None
            self.__error = None
            return 0
        except Exception:
            self.__client.close()
            self.__error = 'Write outputs failed'
            return -1
