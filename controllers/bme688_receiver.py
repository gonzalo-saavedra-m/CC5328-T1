from struct import pack, unpack
from serial import Serial
from time import time
from pprint import pprint

# COMUNICATION CONSTANTS
F = pack('1s', 'L'.encode())
P = pack('1s', 'P'.encode())
S = pack('1s', 'S'.encode())
Y = pack('1s', 'Y'.encode())
N = pack('1s', 'N'.encode())


POWERMODES = {
    'Forzado': F,
    'Paralelo': P,
    'Suspensión': S,
}

class BME688_Receiver():
    def __init__(self, ser: Serial, data_size: int=100) -> None:
        self.ser = ser
        self.data_size = data_size

    def in_waiting(self) -> bool:
        return self.ser.in_waiting > 0

    def write(self, message: bytes) -> None:
        self.ser.write(message)

    def wait_for_message(self, message: bytes, timeout=5) -> None:
        start_time = time()
        current_time = start_time
        while current_time < start_time + timeout:
            if not self.in_waiting(): continue
            raw_data: bytes = self.ser.readline()
            if message in raw_data:
                return
        raise TimeoutError(f'Message {message} not received in {timeout}.')


    def read(self, powermode) -> dict:
        """Reads data from the BME688 sensor and returns it as a dictionary.
        If the powermode is set to 'Suspensión', the function will return an empty dictionary.
        """
        self.write(Y)
        self.wait_for_message(b'SELECT_POWER_MODE')
        selected_powermode = POWERMODES[powermode]
        self.write(selected_powermode)
        if selected_powermode == S:
            return {}
        data = b''
        while True:
            if not self.in_waiting(): continue
            raw_data: bytes = self.ser.readline()
            data += raw_data
            if b'F_TOP_GAS' in raw_data:
                break
        raw_temp = data.split(b'S_DATA_TEMP')[1].split(b'F_DATA_TEMP')[0]
        raw_pres = data.split(b'S_DATA_PRES')[1].split(b'F_DATA_PRES')[0]
        raw_hum = data.split(b'S_DATA_HUM')[1].split(b'F_DATA_HUM')[0]
        raw_gas = data.split(b'S_DATA_GAS')[1].split(b'F_DATA_GAS')[0]
        raw_top5_temp = data.split(b'S_TOP_GAS')[1].split(b'F_TOP_GAS')[0]
        raw_top5_pres = data.split(b'S_TOP_GAS')[1].split(b'F_TOP_GAS')[0]
        raw_top5_hum = data.split(b'S_TOP_GAS')[1].split(b'F_TOP_GAS')[0]
        raw_top5_gas = data.split(b'S_TOP_GAS')[1].split(b'F_TOP_GAS')[0]
        data = {
            'temp': unpack(f'{len(raw_temp)//4}f', raw_temp),
            'pres': unpack(f'{len(raw_pres)//4}f', raw_pres),
            'hum': unpack(f'{len(raw_hum)//4}f', raw_hum),
            'gas': unpack(f'{len(raw_gas)//4}f', raw_gas),
            'top5_temp': unpack(f'{len(raw_top5_temp)//4}f', raw_top5_temp),
            'top5_pres': unpack(f'{len(raw_top5_pres)//4}f', raw_top5_pres),
            'top5_hum': unpack(f'{len(raw_top5_hum)//4}f', raw_top5_hum),
            'top5_gas': unpack(f'{len(raw_top5_gas)//4}f', raw_top5_gas),
        }
        pprint(data)
        return data
