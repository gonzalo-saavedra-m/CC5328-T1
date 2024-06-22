from struct import pack, unpack
from pprint import pprint

# COMUNICATION CONSTANTS
F = pack('1s', 'L'.encode())
P = pack('1s', 'P'.encode())
S = pack('1s', 'S'.encode())
SELECT_POWER_MODE = b'SELECT_POWER_MODE'
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

POWERMODES = {
    'Forzado': F,
    'Paralelo': P,
    'Suspensión': S,
}

class BME688_Receiver():
    def __init__(self, ser, data_size: int=100) -> None:
        self.ser = ser
        self.data_size = data_size

    def in_waiting(self) -> bool:
        return self.ser.in_waiting > 0

    def write(self, message: bytes) -> None:
        self.ser.write(message)

    def read(self, powermode) -> dict:
        selected_powermode = POWERMODES[powermode]
        print(selected_powermode)
        return {}