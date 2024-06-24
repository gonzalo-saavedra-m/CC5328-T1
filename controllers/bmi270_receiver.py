from serial import Serial
from struct import pack, unpack

# COMUNICATION CONSTANTS
SELECT_POWER_MODE = b'SELECT_POWER_MODE'
L = pack('1s', 'L'.encode())
N = pack('1s', 'N'.encode())
P = pack('1s', 'P'.encode())
S = pack('1s', 'S'.encode())
POWERMODES = {
    'Ahorro de energía': L,
    'Equilibrado': N,
    'Alto rendimiento': P,
    'Suspensión': S
}
READY = b'READY_TO_START'
BEGIN_READINGS = pack('15s', 'BEGIN_READINGS\0'.encode())
STOP__READINGS = pack('15s', 'STOP__READINGS\0'.encode())
FINISHED_READINGS = b'FINISHED_READINGS'
BEGIN_RMS = pack('10s', 'BEGIN_RMS\0'.encode())
FINISHED_RMS = b'FINISHED_RMS'
BEGIN_FFT = pack('10s', 'BEGIN_FFT\0'.encode())
FINISHED_FFT = b'FINISHED_FFT'
BEGIN_PEAKS = pack('12s', 'BEGIN_PEAKS\0'.encode())
FINISHED_PEAKS = b'FINISHED_PEAKS'

class BMI270_Receiver():
    def __init__(self, ser: Serial, data_size: int=100) -> None:
        self.ser = ser
        self.reset_data() # Initialize the data dictionaries.
        self.data_size = data_size

    def reset_data(self) -> None:
        self.acc = { 'x': [], 'y': [], 'z': [] }
        self.gyr = { 'x': [], 'y': [], 'z': [] }
        self.RMS = { 'acc_x': [], 'acc_y': [], 'acc_z': [] }
        self.FFT = {
            'acc_x': { 'r': [], 'i': [] },
            'acc_y': { 'r': [], 'i': [] },
            'acc_z': { 'r': [], 'i': [] },
        }
        self.peaks = {
            'acc_x': [], 'acc_y': [], 'acc_z': [],
            'gyr_x': [], 'gyr_y': [], 'gyr_z': [],
            'RMS.acc_x': [], 'RMS.acc_y': [], 'RMS.acc_z': []
            }

    def in_waiting(self) -> bool:
        return self.ser.in_waiting > 0

    def write(self, message: bytes) -> None:
        self.ser.write(message)

    def read_data_line(self, entry: bytes) -> None:
        try:
            data = unpack('6f', entry[:-1])
            for i, coord in enumerate(['x', 'y', 'z']):
                self.acc[coord].append(data[i])
                self.gyr[coord].append(data[i+3])
        except:
            pass

    def read(self, powermode, **kwargs) -> dict:
        print('Received powermode:', powermode)
        while True:
            if not self.in_waiting: continue
            raw_data: bytes = self.ser.readline()
            if SELECT_POWER_MODE in raw_data:
                selected_powermode = POWERMODES.get(powermode, N)
                self.write(selected_powermode)
                break
        while True:
            if not self.in_waiting: continue
            raw_data = self.ser.readline()
            if FINISHED_READINGS in raw_data:
                break
            self.read_data_line(raw_data)
        self.write(BEGIN_RMS)
        byte_data = []
        while True:
            if not self.in_waiting: continue
            raw_data = self.ser.readline()
            if FINISHED_RMS in raw_data:
                break
            else:
                byte_data.append(raw_data)
        byte_data = b''.join(byte_data)
        byte_data = byte_data[:-1]
        data = unpack(f'{self.data_size*3}f', byte_data)
        self.RMS['acc_x'] = data[:self.data_size]
        self.RMS['acc_y'] = data[self.data_size:2*self.data_size]
        self.RMS['acc_z'] = data[2*self.data_size:]
        self.write(BEGIN_FFT)
        byte_data = []
        while True:
            if not self.in_waiting: continue
            raw_data = self.ser.readline()
            if FINISHED_FFT in raw_data:
                break
            else:
                byte_data.append(raw_data)
        byte_data = b''.join(byte_data)
        byte_data = byte_data[:-1] # Remove the last '\n' character
        data = unpack(f'{self.data_size*6}f', byte_data)
        self.FFT['acc_x']['r'] = data[:self.data_size]
        self.FFT['acc_x']['i'] = data[self.data_size:2*self.data_size]
        self.FFT['acc_y']['r'] = data[2*self.data_size:3*self.data_size]
        self.FFT['acc_y']['i'] = data[3*self.data_size:4*self.data_size]
        self.FFT['acc_z']['r'] = data[4*self.data_size:5*self.data_size]
        self.FFT['acc_z']['i'] = data[5*self.data_size:]
        self.write(BEGIN_PEAKS)
        byte_data = []
        while True:
            if not self.in_waiting: continue
            raw_data = self.ser.readline()
            if FINISHED_PEAKS in raw_data:
                break
            else:
                byte_data.append(raw_data)
        byte_data = b''.join(byte_data)
        byte_data = byte_data[:-1]
        data = unpack('45f', byte_data)
        self.peaks['acc_x'] = data[:5]
        self.peaks['acc_y'] = data[5:10]
        self.peaks['acc_z'] = data[10:15]
        self.peaks['gyr_x'] = data[15:20]
        self.peaks['gyr_y'] = data[20:25]
        self.peaks['gyr_z'] = data[25:30]
        self.peaks['RMS.acc_x'] = data[30:35]
        self.peaks['RMS.acc_y'] = data[35:40]
        self.peaks['RMS.acc_z'] = data[40:]
        # self.write(STOP__READINGS)
        return {
            'acc': self.acc,
            'gyr': self.gyr,
            'RMS': self.RMS,
            'FFT': self.FFT,
            'peaks': self.peaks
        }
