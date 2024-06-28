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
            if b'FINISHED' in raw_data:
                break
        # Data is between b'SENDING\0' and b'FINISHED\0'
        filtered_data = data.split(b'SENDING\0')[1].split(b'FINISHED\0')[0]
        print(len(filtered_data))
        raw_values = unpack(f'{len(filtered_data)//4}i', filtered_data)
        readings_len = len(raw_values) - 20
        top5_gas = raw_values[-5:]
        top5_hum = raw_values[-10:-5]
        top5_pres = raw_values[-15:-10]
        top5_temp = raw_values[-20:-15]
        readings = raw_values[:readings_len]
        temp, pres, hum, gas = readings[::4], readings[1::4], readings[2::4], readings[3::4]
        data = {
            'temp': temp,
            'pres': pres,
            'hum': hum,
            'gas': gas,
            'top5_temp': top5_temp,
            'top5_pres': top5_pres,
            'top5_hum': top5_hum,
            'top5_gas': top5_gas
        }
        pprint(data)
        return data
