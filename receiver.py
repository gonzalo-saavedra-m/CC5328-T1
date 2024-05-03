from time import sleep
import serial
from struct import pack, unpack
import keyboard

# Se configura el puerto y el BAUD_Rate
PORT = 'COM3'  # Esto depende del sistema operativo
BAUD_RATE = 115200  # Debe coincidir con la configuracion de la ESP32

# Se abre la conexion serial
ser = serial.Serial(PORT, BAUD_RATE, timeout=1)

# Funciones


def send_message(message):
    """ Funcion para enviar un mensaje a la ESP32 """
    ser.write(message)


def receive_response():
    """ Funcion para recibir un mensaje de la ESP32 """
    response = ser.readline()
    return response


def receive_data():
    """ Funcion que recibe tres floats (fff) de la ESP32 
    y los imprime en consola """
    data = receive_response()
    print(f"Data = {data}")
    data = unpack("fff", data)
    print(f'Received: {data}')
    return data


def send_end_message():
    """ Funcion para enviar un mensaje de finalizacion a la ESP32 """
    end_message = pack('4s', 'END\0'.encode())
    ser.write(end_message)


recibiendo = False
# se debe clickear enter para cambiar el estado de recieving
try:
    while True:
        if keyboard.is_pressed("enter"):  # Escucha para ver si se presiona Enter
            recibiendo = not recibiendo  # Alterna el estado de recepción
            print(f"Recepción {'activada' if recibiendo else 'desactivada'}")

            # Espera a que se suelte la tecla para evitar repetidos cambios
            while keyboard.is_pressed("enter"):
                sleep(0.1)

        if recibiendo:
            try:
                # Intentar leer si hay datos disponibles
                if ser.in_waiting > 0:
                    response = ser.readline()
                    print(f"Recibido: {response}")
            except Exception as e:
                # Manejar errores al leer datos
                print("Error al leer mensaje:", str(e))
                continue

        # Pausa para evitar una alta carga en el bucle
        sleep(0.1)

except KeyboardInterrupt:
    print("Finalizando comunicación.")
finally:
    ser.close()  # Asegúrate de cerrar la conexión serial al terminar

# # Se lee data por la conexion serial
# counter = 0
# while True:
#     if ser.in_waiting > 0:
#         try:
#             response = ser.readline()
#         except KeyboardInterrupt:
#             print('Finalizando comunicacion')
#             break
#         except:
#             print('Error en leer mensaje')
#             continue
