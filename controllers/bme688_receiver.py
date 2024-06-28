from struct import pack, unpack
from serial import Serial
from time import time

# COMUNICATION CONSTANTS
F = pack('1s', 'F'.encode())
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
    def __init__(self, ser: Serial) -> None:
        self.ser = ser

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

    def parse_line(self, line: bytes) -> tuple:
        try:
            return unpack('4f', line[:-1])
        except:
            return None

    def read(self, powermode) -> dict:
        # """Reads data from the BME688 sensor and returns it as a dictionary.
        # If the powermode is set to 'Suspensión', the function will return an empty dictionary.
        # """
        self.write(Y)
        self.wait_for_message(b'SELECT_POWER_MODE')
        selected_powermode = POWERMODES[powermode]
        self.write(selected_powermode)
        if selected_powermode == S:
            return None
        temp, pres, hum, gas = [], [], [], []
        self.wait_for_message(b'BEGIN_READINGS')
        while True:
            if not self.in_waiting(): continue
            raw_data: bytes = self.ser.readline()
            if b'FINISH_READINGS' in raw_data:
                break
            data = self.parse_line(raw_data)
            if data is None: continue
            temp_, pres_, hum_, gas_ = self.parse_line(raw_data)
            temp.append(temp_)
            pres.append(pres_)
            hum.append(hum_)
            gas.append(gas_)
        top5_temp, top5_pres, top5_hum, top5_gas = [], [], [], []
        self.wait_for_message(b'BEGIN_TOP')
        lines_read = 0
        while True:
            if not self.in_waiting(): continue
            raw_data: bytes = self.ser.readline()
            if b'FINISH_TOP' in raw_data:
                break
            data = self.parse_line(raw_data)
            if data is None: continue
            lines_read += 1
            top5_temp_, top5_pres_, top5_hum_, top5_gas_ = data
            top5_temp.append(top5_temp_)
            top5_pres.append(top5_pres_)
            top5_hum.append(top5_hum_)
            top5_gas.append(top5_gas_)
        print(f'Lines read: {lines_read}')
        data = {
            'temp': temp,
            'pres': pres,
            'hum': hum,
            'gas': gas,
            'top5_temp': top5_temp,
            'top5_pres': top5_pres,
            'top5_hum': top5_hum,
            'top5_gas': top5_gas,
        }
        return data
