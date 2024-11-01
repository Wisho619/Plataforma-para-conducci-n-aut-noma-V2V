import machine
from machine import Pin, PWM, Timer, SPI
import time
from time import sleep, ticks_us, ticks_diff
from nrf24l01 import NRF24L01
import struct

csn = Pin(15, mode=Pin.OUT, value=1) # Chip Select Not
ce = Pin(14, mode=Pin.OUT, value=0)  # Chip Enable
led = Pin(25, Pin.OUT)               # Onboard LED
payload_size = 16

# Definición de los pines de los encoders
pin_encoder_a = Pin(10, Pin.IN)
pin_encoder_c = Pin(12, Pin.IN)

pulsos1 = 0
pulsos2 = 0

v = 0

# Pines para el sensor ultrasónico
trig = Pin(2, Pin.OUT)
echo = Pin(3, Pin.IN)

last_time = time.ticks_ms()
# Define the channel or 'pipes' the radios use.
# switch round the pipes depending if this is a sender or receiver pico

role = "send"
# role = "receive"

if role == "send":
    send_pipe = b"\xe1\xf0\xf0\xf0\xf0"
    receive_pipe = b"\xd2\xf0\xf0\xf0\xf0"
else:
    send_pipe = b"\xd2\xf0\xf0\xf0\xf0"
    receive_pipe = b"\xe1\xf0\xf0\xf0\xf0"
    
def setup():
    print("Initialising the nRF24L0+ Module")
    nrf = NRF24L01(SPI(0), csn, ce, payload_size=payload_size)
    nrf.open_tx_pipe(send_pipe)
    nrf.open_rx_pipe(1, receive_pipe)
    nrf.start_listening()
    return nrf

def flash_led(times: int = None):
    ''' Flashed the built in LED the number of times defined in the times parameter '''
    for _ in range(times):
        led.value(1)
        sleep(0.01)
        led.value(0)
        sleep(0.01)

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
        h=0
        #print(role, f"Sorry, message not sent due to: {e}")
    finally:
        nrf.start_listening()

# main code loop
flash_led(1)
nrf = setup()
msg_string = ""

def contar_pulsos1(pin):
    global pulsos1
    pulsos1 += 1
    
def contar_pulsos2(pin):
    global pulsos2
    pulsos2 += 1 # Función para manejar interrupciones del encoder 2
    

class MotorDriver:
    def __init__(self, pwm_pin1, pwm_pin3):
        self.pwm1 = PWM(Pin(pwm_pin1))
        self.pwm3 = PWM(Pin(pwm_pin3))
        self.pwm1.freq(1000)
        self.pwm3.freq(1000)
        self.target_speed = 80
        self.prev2_error = 0
        self.prev_error = 0
        self.error = 0
        self.Kp = 10/23
        self.Ki =5.39/23
        self.Kd = 5.46/23
        self.output_prev = 0
        self.pid_output1 = 0  # Definir como atributos de clase
        self.pid_output2 = 0  # Definir como atributos de clase

    def calculate_pid(self):
        global pulsos1, pulsos2, v
        # Cálculo de velocidad para el encoder 2
        estado_irq = machine.disable_irq()
        #rpm=pulsos1*10/190
        p1=pulsos1
        p2=pulsos2
        print(f" {p1}")
        #print(f" {rpm}")
        #print(f" pulsos2: {pulsos2}")
        # Calcular el PID para ambos motores
        self.pid_output1 = self._calculate_pid_individual(p1)  # Guardar en los atributos
        self.pid_output2 = self._calculate_pid_individual(p2)  # Guardar en los atributos
        
        v = int(p1) # Actualizar el valor de 'v' global
        #print(f"Velocidad calculada por PID: {self.pid_output1}")
        pulsos1 = 0
        pulsos2 = 0
        machine.enable_irq(estado_irq)
    def _calculate_pid_individual(self, current_speed):
        global pulsos1, pulsos2
        gl=368.8
        DT = 0.1
        alpha1 = (self.Kp + self.Ki * DT + self.Kd / DT)
        alpha2 = - (self.Kp) - 2 * self.Kd / DT
        alpha3 = self.Kd / DT

        self.error = (self.target_speed - current_speed)
        u = alpha1 * self.error + alpha2 * self.prev_error + alpha3 * self.prev2_error + self.output_prev
        output= u*gl
        self.prev_error = self.error
        self.prev2_error = self.prev_error
        self.output_prev = u
        m = self.target_speed*gl*1.2
        mi = self.target_speed*gl*0.95
        if output < mi:
            output = mi
            self.error+=100
        elif output > m: # límite máximo
            output = m
        return output

    def move_forward(self):
        # Usar los atributos de la clase
        self.pwm1.duty_u16(int(self.pid_output1))
        self.pwm3.duty_u16(int(self.pid_output2))
    def spinning(self):
        self.pwm1.duty_u16(9000)
        self.pwm3.duty_u16(9000)
        time.sleep(0.1)
    def stop(self):
        self.pwm1.duty_u16(0)
        self.pwm3.duty_u16(0)
        time.sleep(0.1)
    def full(self):
        self.pwm1.duty_u16(60000)
        self.pwm3.duty_u16(60000)
        time.sleep(0.1)

def measure_distance():
    ''' Mide la distancia usando el sensor ultrasónico '''
    trig.value(0)
    sleep(0.002)
    trig.value(1)
    sleep(0.01)
    trig.value(0)
    
    # Inicializar las variables
    signaloff = 0
    signalon = 0
    
    while echo.value() == 0:
        signaloff = ticks_us()
    
    while echo.value() == 1:
        signalon = ticks_us()
    
    duration = ticks_diff(signalon, signaloff)
    
    distance = (duration * 0.0343) / 200  # Convertir a m
    return distance

class Servo:
    def __init__(self, pin):
        self.servo = PWM(Pin(pin))
        self.servo.freq(50)
    
    def set_angle(self, angle):
        d = (int(angle) + 45) * 100000 // 9
        self.servo.duty_ns(int(d))

    def turn_left(self):
        self.set_angle(30)
        time.sleep(0.2)

    def turn_right(self):
        self.set_angle(150)
        time.sleep(0.2)

    def move_forward(self):
        self.set_angle(90)


motor_pwm_pin1 = 7
motor_pwm_pin3 = 8
motor_driver = MotorDriver(motor_pwm_pin1, motor_pwm_pin3)
servo = Servo(0)

sensor_left_pin = Pin(21, Pin.IN)
sensor_right_pin = Pin(20, Pin.IN)

pin_encoder_a.irq(trigger=Pin.IRQ_RISING, handler=contar_pulsos1)
pin_encoder_c.irq(trigger=Pin.IRQ_RISING, handler=contar_pulsos2)

# Bucle principal
while True:
    current_time = time.ticks_ms()
    distance = (min(max(0.1,measure_distance()),1.5))
    if time.ticks_diff(current_time, last_time) >= 100:# Reinicia el conteo de pulsos y manda llamar la función que calcula las salidas PID cada 100ms
        motor_driver.calculate_pid()
        #distance = (min(max(0.1,measure_distance()),1.5))
        send(nrf, v)# Enviar el mensaje con el valor de v
        print(distance)
        last_time = current_time
        
        # Leer valores de los sensores infrarrojos
    left_sensor_value = sensor_left_pin.value()
    right_sensor_value = sensor_right_pin.value()
    motor_driver.move_forward()
        
        # Lógica de seguimiento de línea
    if left_sensor_value == 1 and right_sensor_value == 0:
        servo.turn_left()
        motor_driver.spinning()
        #v=1000
    elif left_sensor_value == 0 and right_sensor_value == 1:
        servo.turn_right()
        motor_driver.spinning()
        #v=2000
    elif distance <= 0.4:
        motor_driver.stop()
    else:
        servo.move_forward()
        
    