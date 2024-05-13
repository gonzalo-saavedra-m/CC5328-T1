from typing import Tuple
import serial
from struct import pack, unpack
import sys
from pprint import pprint

# COMUNICATION CONSTANTS
READY = b'READY_TO_START'
BEGIN_READINGS = pack('15s', 'BEGIN_READINGS\0'.encode())
FINISHED_READINGS = b'FINISHED_READINGS'
BEGIN_RMS = pack('10s', 'BEGIN_RMS\0'.encode())
FINISHED_RMS = b'FINISHED_RMS'
BEGIN_FFT = pack('10s', 'BEGIN_FFT\0'.encode())
FINISHED_FFT = b'FINISHED_FFT'
BEGIN_PEAKS = pack('12s', 'BEGIN_PEAKS\0'.encode())
FINISHED_PEAKS = b'FINISHED_PEAKS'
SHUTDOWN = pack('9s', 'SHUTDOWN\0'.encode())

class ReceiverController():
    def __init__(self, port: str, baud_rate: int) -> None:
        self.ser = serial.Serial(port, baud_rate, timeout=5)
        self.reset_data() # Initialize the data dictionaries.

    def reset_data(self) -> None:
        self.acc = {
            'x': [],
            'y': [],
            'z': []
        }
        self.gyr = {
            'x': [],
            'y': [],
            'z': []
        }
        self.RMS = {
            'acc_x': [],
            'acc_y': [],
            'acc_z': [],
            'gyr_x': [],
            'gyr_y': [],
            'gyr_z': []
        }
        self.FFT = {
            'acc_x': [],
            'acc_y': [],
            'acc_z': [],
        }
        self.peaks = {
            'RMS.acc_x': [],
            'RMS.acc_y': [],
            'RMS.acc_z': [],
            'RMS.gyr_x': [],
            'RMS.gyr_y': [],
            'RMS.gyr_z': [],
            'acc_x': [],
            'acc_y': [],
            'acc_z': [],
        }

    def in_waiting(self) -> bool:
        return self.ser.in_waiting > 0

    def write(self, message: bytes) -> None:
        self.ser.write(message)

    def read_data_line(self, entry: bytes) -> None:
        try:
            data = unpack('ffffff', entry[:-1])
            self.acc['x'].append(data[0])
            self.acc['y'].append(data[1])
            self.acc['z'].append(data[2])
            self.gyr['x'].append(data[3])
            self.gyr['y'].append(data[4])
            self.gyr['z'].append(data[5])
            print(f'Appending data: {data}.')
        except Exception as e:
            print(f'Problem reading data entry: {entry}. Skipping...')
            raise(e)

    def read_RMS(self, entry: bytes) -> None:
        try:
            data = unpack('ff', entry)
            output_parser = ['acc_x', 'acc_y', 'acc_z', 'gyr_x', 'gyr_y', 'gyr_z']
            output = output_parser[data[0]]
            self.RMS[output].append(data[1])
        except:
            print(f'Problem reading RMS entry: {entry}. Skipping...')

    def read_FFT(self, entry: bytes) -> None:
        try:
            data = unpack('ff', entry)
            output_parser = ['acc_x', 'acc_y', 'acc_z']
            output = output_parser[data[0]]
            self.FFT[output].append(data[1])
        except:
            print(f'Problem reading FFT entry: {entry}. Skipping...')

    def read_peaks(self, entry: bytes) -> None:
        try:
            data = unpack('ffffff', entry)
            output_parser = ['RMS.acc_x', 'RMS.acc_y', 'RMS.acc_z', 'RMS.gyr_x', 'RMS.gyr_y', 'RMS.gyr_z', 'acc_x', 'acc_y', 'acc_z']
            output = output_parser[data[0]]
            self.peaks[output] = data[1:]
        except:
            print(f'Problem reading peaks entry: {entry}. Skipping...')

    def main(self) -> None:
        try:
            while True:
                if not self.in_waiting: continue
                raw_data: bytes = self.ser.readline()
                print(raw_data)
                if READY in raw_data:
                    print('Received READY message.')
                    input('Press Enter to start the data acquisition process...')
                    print('Sending BEGIN_READINGS message.')
                    self.write(BEGIN_READINGS)
                    break
            while True:
                if not self.in_waiting: continue
                raw_data = self.ser.readline()
                print(f'Received {raw_data}')
                if FINISHED_READINGS in raw_data:
                    print('Received FINISHED_READINGS message.')
                    break
                self.read_data_line(raw_data)
            print('Sending BEGIN_RMS message...')
            self.write(BEGIN_RMS)
            while True:
                if not self.in_waiting: continue
                raw_data = self.ser.readline()
                if FINISHED_RMS in raw_data:
                    print('Received FINISHED_RMS message.')
                    break
                self.read_RMS(raw_data)
            print('Sending BEGIN_FFT message...')
            self.write(BEGIN_FFT)
            while True:
                if not self.in_waiting: continue
                raw_data = self.ser.readline()
                if FINISHED_FFT in raw_data:
                    print('Received FINISHED_FFT message.')
                    break
                self.read_FFT(raw_data)
            print('Sending BEGIN_PEAKS message...')
            self.write(BEGIN_PEAKS)
            while True:
                if not self.in_waiting: continue
                raw_data = self.ser.readline()
                if FINISHED_PEAKS in raw_data:
                    print('Received FINISHED_PEAKS message.')
                    break
                self.read_peaks(raw_data)
            print('Data acquisition process finished. Printing data...')
            print('Acceleration values:')
            pprint(self.acc)
            print('Gyroscope values:')
            pprint(self.gyr)
            print('RMS values:')
            pprint(self.RMS)
            print('FFT values:')
            pprint(self.FFT)
            print('Peaks values:')
            pprint(self.peaks)

        except Exception as e:
            print(f'An error occurred: {e}')
        finally:
            print('Sending SHUTDOWN message...')
            self.write(SHUTDOWN)
            self.ser.close()
            print('Connection closed.')


if __name__ == '__main__':
    PORT = 'COM3'  # Esto depende del sistema operativo
    BAUD_RATE = 115200  # Debe coincidir con la configuracion de la ESP32
    receiver = ReceiverController(PORT, BAUD_RATE)
    receiver.main()
