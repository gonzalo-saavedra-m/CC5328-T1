from typing import Tuple
import serial
from struct import pack, unpack
import keyboard

# TODO: Send logs to a text file
# TODO: Add timeout on the main loop
class ReceiverController():
    def __init__(self, port: str, baud_rate: int, timeout: int=5) -> None:
        self.ser = serial.Serial(port, baud_rate, timeout=5)
        self.timeout = timeout

    def in_waiting(self) -> bool:
        return self.ser.in_waiting > 0

    def write(self, message: bytes) -> None:
        self.ser.write(message)

    def read(self, encoding: str | bytes) -> tuple:
        return unpack(encoding, self.ser.readline())

    def main(self) -> None:
        while True:
            if not self.in_waiting: continue
            try:
                raw_data: bytes = self.ser.readline()
                if b'Beginning initialization' in raw_data:
                    print('Sengind BEGIN message...')
                    message = pack('6s', 'BEGIN\0'.encode())
                    self.write(message)
                    continue
                data: Tuple[int] = self.read('fff')
                # TODO: Define the order of the data
            except KeyboardInterrupt:
                print('Keyboard exit. Exiting...')
                break
            except Exception:
                pass
        end_message: bytes = pack('4s', 'END\0'.encode())
        self.write(end_message)
        self.ser.close()
        print('Connection closed.')


if __name__ == '__main__':
    PORT = 'COM3'  # Esto depende del sistema operativo
    BAUD_RATE = 115200  # Debe coincidir con la configuracion de la ESP32
    receiver = ReceiverController(PORT, BAUD_RATE)
    receiver.main()
