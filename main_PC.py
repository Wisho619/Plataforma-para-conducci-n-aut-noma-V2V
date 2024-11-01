from nrf24l01 import NRF24L01 
from machine import SPI, Pin, PWM, time_pulse_us
import time
from time import sleep, ticks_us, ticks_diff
import struct
import math

# Configuración de pines y constantes
csn = Pin(15, mode=Pin.OUT, value=1)  # Chip Select Not
ce = Pin(14, mode=Pin.OUT, value=0)   # Chip Enable
led = Pin(25, Pin.OUT)                # Onboard LED
payload_size = 16
v1 = 0  # Velocidad del Vehículo 1
v2 = 0  # Velocidad del Vehículo 2
s = 0
last_time = time.ticks_ms()

# Configuración de canales para identificar los vehículos
send_pipe = b"\xe1\xf0\xf0\xf0\xf0"
vehiculo_1_pipe = b"\xd2\xf0\xf0\xf0\xf0"
vehiculo_2_pipe = b"\xd3\xf0\xf0\xf0\xf0"

def setup():
    print("Inicializando el módulo nRF24L01+")
    nrf = NRF24L01(SPI(0), csn, ce, payload_size=payload_size)
    nrf.open_tx_pipe(send_pipe)
    nrf.open_rx_pipe(1, vehiculo_1_pipe)  # Canal para Vehículo 1
    nrf.open_rx_pipe(2, vehiculo_2_pipe)  # Canal para Vehículo 2
    nrf.start_listening()
    return nrf

def flash_led(times: int = None):
    '''Enciende el LED interno el número de veces definido en el parámetro times'''
    for _ in range(times):
        led.value(1)
        sleep(0.1)
        led.value(0)
        sleep(0.1)

def send(nrf, value):
    try:
        full_msg = f"{value:.2f}"
        nrf.stop_listening()
        
        if len(full_msg) < payload_size:
            full_msg = full_msg + ' ' * (payload_size - len(full_msg))  # Rellenar con espacios
        else:
            full_msg = full_msg[:payload_size]
        
        nrf.send(full_msg.encode('utf-8'))
        flash_led(1)
    except OSError as e:
        print(f"Error al enviar mensaje: {e}")
    finally:
        nrf.start_listening()

# Bucle principal
flash_led(1)
nrf = setup()

while True:
    current_time = time.ticks_ms()
    if time.ticks_diff(current_time, last_time) >= 100:
        s += 0.1
        print("Tiempo:", s)
        print("Velocidad Vehículo 1:", v1)
        print("Velocidad Vehículo 2:", v2)
        last_time = current_time

    try:
        # Recibir la velocidad del vehículo líder
        if nrf.any():
            package = nrf.recv()
            msg = package.decode('utf-8').strip()
            flash_led(1)
            
            try:
                speed = float(msg)  # Convertir mensaje a velocidad
                
                # Identificar el vehículo mediante el canal que envió el mensaje
                if nrf.pipe() == 1:
                    v1 = int(speed)  # Actualizar velocidad del Vehículo 1
                    #print("Mensaje de Vehículo 1:", v1)
                elif nrf.pipe() == 2:
                    v2 = int(speed)  # Actualizar velocidad del Vehículo 2
                    #print("Mensaje de Vehículo 2:", v2)

            except ValueError:
                print("El mensaje recibido no es un valor de velocidad válido.")
    except Exception as e:
        print(f"Error: {e}")


    