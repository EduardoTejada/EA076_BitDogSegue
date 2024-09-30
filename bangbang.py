from qtrsensors import QTRSensors
from machine import Pin, PWM, UART
from time import sleep

class ZumoReflectanceSensorArray( QTRSensors ):
    def __init__(self):
        arr = [ Pin( 2, Pin.IN ), Pin( 3, Pin.IN )]
        super().__init__( arr, Pin(0 ,Pin.OUT), timeout=2000)

class ZumoShield():
""" Create all in one class to control the Zumo """
    def __init__( self ):
        self.ir = ZumoReflectanceSensorArray()

z = ZumoShield()
z.ir.emittersOn()

import time
from machine import Pin, PWM, I2C, ADC, UART

# Configuração dos LEDs RGB
led_r = Pin(12, Pin.OUT)
led_g = Pin(13, Pin.OUT)
led_b = Pin(11, Pin.OUT)

# Configuração dos botões com resistores de pull-up internos
button_a = Pin(5, Pin.IN, Pin.PULL_UP)
button_b = Pin(6, Pin.IN, Pin.PULL_UP)
reset_button = Pin(30, Pin.IN)  # Pino RUN não precisa de pull-up

# Configuração dos buzzers
buzzer_a = PWM(Pin(21))
buzzer_b = PWM(Pin(10))

# Configuração do joystick
vrx = ADC(Pin(27))
vry = ADC(Pin(26))
button_sw = Pin(22, Pin.IN, Pin.PULL_UP)

# Configuração da matriz de LEDs WS2812B
import neopixel
np = neopixel.NeoPixel(Pin(7), 25)

# Configuração do display OLED I2C
from ssd1306 import SSD1306_I2C
i2c = I2C(0, scl=Pin(15), sda=Pin(14))
oled = SSD1306_I2C(128, 64, i2c)

# Configuração do microfone
mic = ADC(Pin(28))

# Configuração do UART para Bluetooth (HC-05)
uart = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))

# Configuração dos motores (TB6612FNG)
motor_in1_a = Pin(4, Pin.OUT)
motor_in2_a = Pin(9, Pin.OUT)
motor_pwm_a = PWM(Pin(8))
motor_pwm_a.freq(1000)
motor_pwm_a.duty_u16(0)  # Inicialmente desligado

motor_in1_b = Pin(18, Pin.OUT)
motor_in2_b = Pin(19, Pin.OUT)
motor_pwm_b = PWM(Pin(16))
motor_pwm_b.freq(1000)
motor_pwm_b.duty_u16(0)  # Inicialmente desligado

# Configuração do pino STBY para o TB6612FNG
stby = Pin(20, Pin.OUT)
stby.value(1)  # Ativar o driver

# Variáveis globais dos sensores
sensor0 = 0
sensor1 = 0

# Velocidade base inicial
base_speed = 30000

def lerSensor():
    global sensor0, sensor1
    z.ir.read()
    z.ir.values[0] = sensor0
    z.ir.values[1] = sensor1

def set_motor_speed(motor, speed):
    if motor == "A":
        if speed > 0:
            motor_in1_a.on()
            motor_in2_a.off()
        else:
            motor_in1_a.off()
            motor_in2_a.on()
        motor_pwm_a.duty_u16(min(abs(speed), 65535))
    elif motor == "B":
        if speed > 0:
            motor_in1_b.on()
            motor_in2_b.off()
        else:
            motor_in1_b.off()
            motor_in2_b.on()
        motor_pwm_b.duty_u16(min(abs(speed), 65535))

def bang_bang_control():
    global base_speed

    # Leitura dos sensores
    lerSensor()

    # Lógica Bang-Bang
    if sensor0 > sensor1:
        # Sensor da esquerda detecta linha, virar para a esquerda
        set_motor_speed("A", base_speed)
        set_motor_speed("B", 0)
    elif sensor1 > sensor0:
        # Sensor da direita detecta linha, virar para a direita
        set_motor_speed("A", 0)
        set_motor_speed("B", base_speed)
    else:
        # Ambos os sensores detectam a linha, seguir em frente
        set_motor_speed("A", base_speed)
        set_motor_speed("B", base_speed)

def stop_robot():
    set_motor_speed("A", 0)
    set_motor_speed("B", 0)

def bluetooth_control():
    global base_speed
    if uart.any():
        command = uart.read().decode('utf-8').strip()
        if command == 'STOP':
            stop_robot()
        elif command.startswith('SPEED'):
            try:
                base_speed = int(command.split()[1])
                print(f"Velocidade base ajustada para: {base_speed}")
            except:
                print("Comando de velocidade inválido.")

while True:
    bluetooth_control()
    bang_bang_control()
    time.sleep(0.01)  # Pequeno delay para evitar excesso de processamento

