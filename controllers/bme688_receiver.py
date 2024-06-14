import serial
from struct import pack, unpack
from pprint import pprint

# COMUNICATION CONSTANTS
SELECT_POWER_MODE = b'SELECT_POWER_MODE'
L = pack('1s', 'L'.encode())
N = pack('1s', 'N'.encode())
P = pack('1s', 'P'.encode())
S = pack('1s', 'S'.encode())
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

class BME688_Receiver():
    def __init__(self, port: str, baud_rate: int, data_size: int=100) -> None:
        self.ser = serial.Serial(port, baud_rate, timeout=5)
        self.reset_data() # Initialize the data dictionaries.
        self.data_size = data_size

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
            'acc_z': []
        }
        self.FFT = {
            'acc_x': {
                'r': [],
                'i': []},
            'acc_y': {
                'r': [],
                'i': []},
            'acc_z': {
                'r': [],
                'i': []},
        }
        self.peaks = {
            'acc_x': [],
            'acc_y': [],
            'acc_z': [],
            'gyr_x': [],
            'gyr_y': [],
            'gyr_z': [],
            'RMS.acc_x': [],
            'RMS.acc_y': [],
            'RMS.acc_z': [],
        }

    def in_waiting(self) -> bool:
        return self.ser.in_waiting > 0

    def write(self, message: bytes) -> None:
        self.ser.write(message)

    def read_data_line(self, entry: bytes) -> None:
        try:
            data = unpack('6f', entry[:-1])
            self.acc['x'].append(data[0])
            self.acc['y'].append(data[1])
            self.acc['z'].append(data[2])
            self.gyr['x'].append(data[3])
            self.gyr['y'].append(data[4])
            self.gyr['z'].append(data[5])
        except Exception as e:
            print(f'Problem reading data entry: {entry}. Skipping...')

    def main(self) -> None:
        try:
            while True:
                if not self.in_waiting: continue
                raw_data: bytes = self.ser.readline()
                print(raw_data)
                if SELECT_POWER_MODE in raw_data:
                    print('Received SELECT_POWER_MODE message.')
                    user_input = input('Select the power mode (L, N, P): ')
                    if user_input == 'L':
                        print('Sending L message.')
                        self.write(L)
                    elif user_input == 'N':
                        print('Sending N message.')
                        self.write(N)
                    elif user_input == 'P':
                        print('Sending P message.')
                        self.write(P)
                    else:
                        print('Invalid power mode. Using N...')
                        self.write(N)
                else:
                    continue
                while True:
                    if not self.in_waiting: continue
                    raw_data: bytes = self.ser.readline()
                    print(raw_data)
                    if READY in raw_data:
                        print('Received READY message.')
                        user_input = input('Press ENTER to start data acquisition or type "exit" to finish the program: ')
                        if user_input == 'exit':
                            raise StopIteration
                        print('Sending BEGIN_READINGS message.')
                        self.write(BEGIN_READINGS)
                        break
                    else:
                        continue
                while True:
                    if not self.in_waiting: continue
                    raw_data = self.ser.readline()
                    if FINISHED_READINGS in raw_data:
                        print('Received FINISHED_READINGS message.')
                        break
                    self.read_data_line(raw_data)
                print('Sending BEGIN_RMS message...')
                self.write(BEGIN_RMS)
                byte_data = []
                while True:
                    if not self.in_waiting: continue
                    raw_data = self.ser.readline()
                    if FINISHED_RMS in raw_data:
                        print('Received FINISHED_RMS message.')
                        break
                    else:
                        byte_data.append(raw_data)
                byte_data = b''.join(byte_data)
                byte_data = byte_data[:-1]
                data = unpack(f'{self.data_size*3}f', byte_data)
                self.RMS['acc_x'] = data[:self.data_size]
                self.RMS['acc_y'] = data[self.data_size:2*self.data_size]
                self.RMS['acc_z'] = data[2*self.data_size:]
                print('Sending BEGIN_FFT message...')
                self.write(BEGIN_FFT)
                byte_data = []
                while True:
                    if not self.in_waiting: continue
                    raw_data = self.ser.readline()
                    if FINISHED_FFT in raw_data:
                        print('Received FINISHED_FFT message.')
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
                print('Sending BEGIN_PEAKS message...')
                self.write(BEGIN_PEAKS)
                byte_data = []
                while True:
                    if not self.in_waiting: continue
                    raw_data = self.ser.readline()
                    if FINISHED_PEAKS in raw_data:
                        print('Received FINISHED_PEAKS message.')
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
                print('Data acquisition process finished. Printing data...')
                print('Acceleration values:')
                print(self.acc)
                print('Gyroscope values:')
                print(self.gyr)
                print('RMS values:')
                print(self.RMS)
                print('FFT values:')
                print(self.FFT)
                print('Peaks values:')
                pprint(self.peaks)
                with open('data.txt', 'a') as f:
                    f.write('Acceleration values:\n')
                    f.write(str(self.acc))
                    f.write('\n\nGyroscope values:\n')
                    f.write(str(self.gyr))
                    f.write('\n\nRMS values:\n')
                    f.write(str(self.RMS))
                    f.write('\n\nFFT values:\n')
                    f.write(str(self.FFT))
                    f.write('\n\nPeaks values:\n')
                    f.write(str(self.peaks))
                    f.write('\n')

        except StopIteration:
            print('Exiting the program...')
        except Exception as e:
            print(f'An error occurred: {e}')
        finally:
            print('Sending SHUTDOWN message...')
            self.write(STOP__READINGS)
            self.ser.close()
            print('Connection closed.')


if __name__ == '__main__':
    import sys
    PORT = 'COM3'  # Esto depende del sistema operativo
    BAUD_RATE = 115200  # Debe coincidir con la configuracion de la ESP32
    data_size = sys.argv[1] if int(len(sys.argv)) > 1 else 100
    receiver = BME688_Receiver(PORT, BAUD_RATE, data_size)
    receiver.main()
