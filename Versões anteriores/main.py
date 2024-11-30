from machine import Pin, PWM, UART
from time import sleep
import neopixel

linha = 1 # define a cor da linha da linha a ser seguida: 0 pra branco, 1 pra preto
modo_de_operacao = 1
"""
0: controle por celular
1: controle por bangbang
2: controle por PID (não implementado)
"""

sensor_esq = Pin(2, Pin.IN)
sensor_dir = Pin(3, Pin.IN)

# Configuração da matriz de LEDs WS2812B
np = neopixel.NeoPixel(Pin(7), 25)

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


# Velocidade base inicial
base_speed = 50
parada_flag = 0

# Configuração do UART para Bluetooth (HC-05)
uart = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))

sensor_esq_estado = sensor_dir_estado = 0

def lerSensor():
    global sensor_esq_estado, sensor_dir_estado
    # Ler o estado do pino GPIO0
    sensor_esq_estado = sensor_esq.value()
    sensor_dir_estado = sensor_dir.value()
    
    print("esquerdo: ", sensor_esq_estado, "       direito: ", sensor_dir_estado)
    
    # Aguardar 0.2 segundo antes da próxima leitura
    sleep(0.2)


def send_data(data):
    uart.write(data + '\n')  # Enviar dados com nova linha
    

def set_motor_speed(motor, speed):
    dutycycle = int(speed * 65535 / 100)
    if motor == "A":
        if speed > 0:
            motor_in1_a.off()
            motor_in2_a.on()
        else:
            motor_in1_a.on()
            motor_in2_a.off()
        motor_pwm_a.duty_u16(dutycycle)
    elif motor == "B":
        if speed > 0:
            motor_in1_b.on()
            motor_in2_b.off()
        else:
            motor_in1_b.off()
            motor_in2_b.on()
        motor_pwm_b.duty_u16(dutycycle)

def controle_bang_bang():
    global base_speed, sensor_esq_estado, sensor_dir_estado, linha, parada_flag

    # Leitura dos sensores
    estado_dir = []
    estado_esq = []
    ''' # sistema para fazer a media de diversas leituras (deixa a resposta mais lenta)
    quantidade_de_leituras = 5
    for i in range(0, quantidade_de_leituras):
        lerSensor()
        estado_dir.append(sensor_dir_estado)
        estado_esq.append(sensor_esq_estado)
        sleep(0.01)
    
    sensor_esq_estado = 1 if estado_dir.count(1) > estado_dir.count(0) else 0
    sensor_dir_estado = 1 if estado_esq.count(1) > estado_esq.count(0) else 0
    '''
    
    lerSensor()
    estado_dir.append(sensor_dir_estado)
    estado_esq.append(sensor_esq_estado)
    
    if parada_flag == 1:
        return
    
    # Lógica Bang-Bang
    if sensor_esq_estado == linha and sensor_dir_estado != linha:
        # Sensor da esquerda detecta linha, virar para a esquerda
        set_motor_speed("A", base_speed)
        set_motor_speed("B", 0)
    elif sensor_esq_estado != linha and sensor_dir_estado == linha:
        # Sensor da direita detecta linha, virar para a direita
        set_motor_speed("A", 0)
        set_motor_speed("B", base_speed)
    else:
        # Ambos os sensores detectam a linha, seguir em frente
        set_motor_speed("A", base_speed)
        set_motor_speed("B", base_speed)
    

def andar_para_frente():
    set_motor_speed("A", base_speed)
    set_motor_speed("B", base_speed)

def andar_para_tras():
    global base_speed
    dutycycle = int(base_speed * 65535 / 100)
    
    motor_in1_a.on()
    motor_in2_a.off()
    
    motor_in1_b.off()
    motor_in2_b.on()
    
    motor_pwm_a.duty_u16(dutycycle)
    motor_pwm_b.duty_u16(dutycycle)
    
def andar_para_esquerda():
    global base_speed
    dutycycle = int(base_speed * 65535 / 100)
    
    motor_in1_a.on()
    motor_in2_a.off()
    
    motor_in1_b.on()
    motor_in2_b.off()
    
    motor_pwm_a.duty_u16(dutycycle)
    motor_pwm_b.duty_u16(dutycycle)
    
def andar_para_direita():
    global base_speed
    dutycycle = int(base_speed * 65535 / 100)
    
    motor_in1_a.off()
    motor_in2_a.on()
    
    motor_in1_b.off()
    motor_in2_b.on()
    
    motor_pwm_a.duty_u16(dutycycle)
    motor_pwm_b.duty_u16(dutycycle)

def controle_pid():
    pass

def stop_robot():
    set_motor_speed("A", 0)
    set_motor_speed("B", 0)

def controle_bluetooth():
    global base_speed, parada_flag, modo_de_operacao
    if uart.any():
        command = uart.readline().decode().strip()
        print(f"Received: {command}")
        if command == 'P':
            stop_robot()
            parada_flag = 1
            #send_data("Para")
        elif command == 'p':
            parada_flag = 0
            print("Continua")
            #send_data("Continua")
        elif command.startswith('V'):
            c = command[0]
            try:
                speed0 = command[1]
                speed1 = command[2]
            except:
                return
            base_speed = int(speed0 + speed1)
            print(f"Velocidade: {speed0}{speed1}%")
            #send_data(f"Velocidade: {speed0}{speed1}%")
        elif command == 'R': # controle remoto
            modo_de_operacao = 0
            stop_robot()
        elif command == 'B': # bang bang
            modo_de_operacao = 1
        elif command == 'I': # pid
            modo_de_operacao = 2
        elif(modo_de_operacao == 0 and parada_flag == 0):
            if command == 'F':
                andar_para_frente()
                print("Frente")
            elif command == 'f':
                stop_robot()
            elif command == 'T':
                andar_para_tras()
                print("Tras")
            elif command == 't':
                stop_robot()
            elif command == 'D':
                andar_para_direita()
                print("Direita")
            elif command == 'd':
                stop_robot()
            elif command == 'E':
                andar_para_esquerda()
                print("Esquerda")
            elif command == 'e':
                stop_robot()


while True:
    controle_bluetooth()
    if(modo_de_operacao == 1):
        controle_bang_bang()
    elif(modo_de_operacao == 2):
        controle_pid()
    sleep(0.01)  # Pequeno delay para evitar excesso de processamento

