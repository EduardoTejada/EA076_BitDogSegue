from machine import Pin, PWM, UART, I2C, SoftI2C
from ssd1306 import SSD1306_I2C
from time import sleep
from ads1x15 import ADS1115
import neopixel

linha = 0 # define a cor da linha da linha a ser seguida: 0 pra branco, 1 pra preto
limiar = 0.7

# Velocidade base inicial
base_speed = 60

# Definição das constantes PID
Kp = 10.0  # Constante proporcional 10
Ki = 0.5  # Constante integral
Kd = 1.5  # Constante derivativa

# Variáveis globais para o controle PID
integral = 0.0
last_error = 0.0

modo_de_operacao = 0

#0: controle por celular
#1: controle por bangbang
#2: controle por PID

# Configuração I2C do ADS1115
i2c1_sda = Pin(2)  # GPIO2 como SDA
i2c1_scl = Pin(3)  # GPIO3 como SCL
i2c = I2C(1, scl=i2c1_scl, sda=i2c1_sda, freq=400000)
adc = ADS1115(i2c, address=0x48, gain=1)

rate = 4

sensores = [0, 0, 0, 0]
#sensor_esq = Pin(2, Pin.IN)
#sensor_dir = Pin(3, Pin.IN)

# Configuração da matriz de LEDs WS2812B
np = neopixel.NeoPixel(machine.Pin(7), 25)

it=50

# definir cores para os LEDs
BLU = (0, 0, 1*it)# BLUE
GRE = (0, 1*it, 0)# GREEN
RED = (1*it, 0, 0)#RED
YEL = (1*it, 1*it, 0)# YELLOW
MAGE = (1*it, 0, 1*it)# MANGENTA
CYA = (0, 1*it, 1*it)# CYAN
WHI = (1*it//3, 1*it//3, 1*it//3)# WHITE
BLA = (0, 0, 0)# BLACK

nothing = [
    [BLA, BLA, BLA, BLA, BLA],
    [BLA, BLA, BLA, BLA, BLA],
    [BLA, BLA, BLA, BLA, BLA],
    [BLA, BLA, BLA, BLA, BLA],
    [BLA, BLA, BLA, BLA, BLA]
]

all = [
    [WHI, WHI, WHI, WHI, WHI],
    [WHI, WHI, WHI, WHI, WHI],
    [WHI, WHI, WHI, WHI, WHI],
    [WHI, WHI, WHI, WHI, WHI],
    [WHI, WHI, WHI, WHI, WHI]
]

up_arrow = [
    [BLA, BLA, BLA, BLA, BLA],
    [BLA, BLA, BLA, BLA, BLA],
    [BLA, BLA, BLA, BLA, BLA],
    [BLA, WHI, BLA, WHI, BLA],
    [BLA, BLA, WHI, BLA, BLA]
]

down_arrow = [
    [BLA, BLA, WHI, BLA, BLA],
    [BLA, WHI, BLA, WHI, BLA],
    [BLA, BLA, BLA, BLA, BLA],
    [BLA, BLA, BLA, BLA, BLA],
    [BLA, BLA, BLA, BLA, BLA]
]

left_arrow = [
    [BLA, BLA, BLA, BLA, BLA],
    [BLA, BLA, BLA, WHI, BLA],
    [BLA, BLA, BLA, BLA, WHI],
    [BLA, BLA, BLA, WHI, BLA],
    [BLA, BLA, BLA, BLA, BLA]
]

right_arrow = [
    [BLA, BLA, BLA, BLA, BLA],
    [BLA, WHI, BLA, BLA, BLA],
    [WHI, BLA, BLA, BLA, BLA],
    [BLA, WHI, BLA, BLA, BLA],
    [BLA, BLA, BLA, BLA, BLA]
]

def mostrarMatriz(desenho):
    # definir a matriz 5x5
    led_matrix = desenho

    # inverter a matriz para que seja mostrada exatamente como é escrita
    inverted_matrix = led_matrix[::-1]
    inverted_matrix[0] = inverted_matrix[0][::-1]
    inverted_matrix[2] = inverted_matrix[2][::-1]
    inverted_matrix[4] = inverted_matrix[4][::-1]

    # exibir a matriz invertida nos LEDs
    for i in range(5):
        for j in range(5):
            np[i*5+j] = inverted_matrix[i][j]

    np.write()

mostrarMatriz(nothing)

# Configuração OLED
i2c = SoftI2C(scl=Pin(15), sda=Pin(14))
oled = SSD1306_I2C(128, 64, i2c)

# Configuração dos botões
button_a = Pin(5, Pin.IN, Pin.PULL_UP)
button_b = Pin(6, Pin.IN, Pin.PULL_UP)
estado_botao_a = 0
estado_botao_b = 0

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


flag_parada = 0

# Configuração do UART para Bluetooth (HC-05)
uart = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))

sensor_esq_estado = sensor_dir_estado = 0

def lerSensor():
    global sensores, rate #sensor_esq_estado, sensor_dir_estado
    
    sensores = [0, 0, 0, 0]
    
    # Ler os valores dos canais 0 a 3
    for channel in range(4):
        value = adc.read(rate=rate, channel1=channel)
        sensores[channel] = value
    
    print(sensores)
    
    # Ler o estado do pino GPIO0
    #sensor_esq_estado = sensor_esq.value()
    #sensor_dir_estado = sensor_dir.value()
    
    #print("esquerdo: ", sensor_esq_estado, "       direito: ", sensor_dir_estado)
    
    # Aguardar 0.2 segundo antes da próxima leitura
    #sleep(0.2)

def lerValoresCrus():
    for i, channel in enumerate([0, 1, 2, 3]):
        value = adc.read(rate=4, channel1=channel)
        print(f"Valor normalizado do canal {channel}: {value}")
    print('')

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

def mostrarLinhaMatriz(valores):

    follow_matrix = [
        [BLA, BLA, BLA, BLA, BLA],
        [BLA, BLA, BLA, BLA, BLA],
        [BLA, BLA, BLA, BLA, BLA],
        [BLA, BLA, BLA, BLA, BLA],
        [BLA, BLA, BLA, BLA, BLA]
    ]
    
    il = 200
    vetor = [3, 2, -1, 1, 0]
    for s in range(0, 5):
        if(s != 2):
            s2 = 0 if(valores[vetor[s]] > 1) else int(il*(1-valores[vetor[s]])//3)
            for s1 in range(0, 5):
                follow_matrix[s1][s] = (s2, s2, s2)# WHITE LINE
    mostrarMatriz(follow_matrix)

def iniciaOled():
    oled.fill(0)  # Limpar display
    oled.text("BitDogSegue", 0, 0)
    oled.text("Conecte o BT", 0, 10)
    oled.text("Aperte (A)", 0, 30)
    oled.text("para calibrar", 0, 40)
    oled.show()

def controle_bang_bang():
    global base_speed, linha, flag_parada, limiar, sensores, min_vals, max_vals #sensor_esq_estado, sensor_dir_estado

    # Leitura dos sensores
    #estado_dir = []
    #estado_esq = []
     # sistema para fazer a media de diversas leituras (deixa a resposta mais lenta)
    #quantidade_de_leituras = 5
    #for i in range(0, quantidade_de_leituras):
    #    lerSensor()
    #    estado_dir.append(sensor_dir_estado)
    #    estado_esq.append(sensor_esq_estado)
    #    sleep(0.01)
    # 
    #sensor_esq_estado = 1 if estado_dir.count(1) > estado_dir.count(0) else 0
    #sensor_dir_estado = 1 if estado_esq.count(1) > estado_esq.count(0) else 0
    
    
    #estado_dir.append(sensor_dir_estado)
    #estado_esq.append(sensor_esq_estado)
    
    # Ler e normalizar os valores dos sensores
    normalized_values = read_normalized_values(adc, min_vals, max_vals)
    
    mostrarLinhaMatriz(normalized_values)
    
    print(normalized_values)
    
    sensor_esq_estado = 0 if(linha == 1) else 1
    sensor_dir_estado = 0 if(linha == 1) else 1
    
    if((normalized_values[0] > limiar and linha == 1) or (normalized_values[0] < limiar and linha == 0)):
        sensor_esq_estado = linha
    if((normalized_values[3] > limiar and linha == 1) or (normalized_values[3] < limiar and linha == 0)):
        sensor_dir_estado = linha
    
    if(flag_parada == 0):
        # Lógica Bang-Bang
        if sensor_esq_estado == linha and sensor_dir_estado != linha:
            # Sensor da esquerda detecta linha, virar para a esquerda
            set_motor_speed("A", 0)
            set_motor_speed("B", base_speed)
        elif sensor_esq_estado != linha and sensor_dir_estado == linha:
            # Sensor da direita detecta linha, virar para a direita
            set_motor_speed("A", base_speed)
            set_motor_speed("B", 0)
        else:
            # Ambos os sensores detectam a linha, seguir em frente
            set_motor_speed("A", base_speed)
            set_motor_speed("B", base_speed)
    else:
        set_motor_speed("A", 0)
        set_motor_speed("B", 0)

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

def stop_robot():
    set_motor_speed("A", 0)
    set_motor_speed("B", 0)


def checarBotoes():
    global estado_botao_a, estado_botao_b, flag_parada, adc, integral, last_error
    
    lerBotoes()
    if(estado_botao_b):
        if(flag_parada):
            flag_parada = 0
        else:
            flag_parada = 1
            integral = 0.0
            last_error = 0.0
            stop_robot()
        sleep(0.2)


def controle_bluetooth():
    global base_speed, flag_parada, modo_de_operacao, integral, last_error, Kp, Kd, Ki
    if uart.any():
        command = uart.readline().decode().strip()
        print(f"Received: {command}")
        if command == 'P':
            stop_robot()
            flag_parada = 1
            #send_data("Para")
        elif command == 'p':
            flag_parada = 0
            print("Continua")
            integral = 0
            last_error = 0
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
            integral = 0
            last_error = 0
            modo_de_operacao = 2
        elif(modo_de_operacao == 0 and flag_parada == 0):
            if command == 'F':
                mostrarMatriz(up_arrow)
                andar_para_frente()
                print("Frente")
            elif command == 'f':
                mostrarMatriz(nothing)
                stop_robot()
            elif command == 'T':
                mostrarMatriz(down_arrow)
                andar_para_tras()
                print("Tras")
            elif command == 't':
                mostrarMatriz(nothing)
                stop_robot()
            elif command == 'D':
                mostrarMatriz(right_arrow)
                andar_para_direita()
                print("Direita")
            elif command == 'd':
                mostrarMatriz(nothing)
                stop_robot()
            elif command == 'E':
                mostrarMatriz(left_arrow)
                andar_para_esquerda()
                print("Esquerda")
            elif command == 'e':
                mostrarMatriz(nothing)
                stop_robot()


def lerBotoes():
    global estado_botao_a, estado_botao_b, flag_parada
    
    if button_a.value() == 0 and estado_botao_a == 0:
        estado_botao_a = 1
    elif button_a.value() == 1 and estado_botao_a == 1:
        estado_botao_a = 0
    
    if button_b.value() == 0 and estado_botao_b == 0:
        estado_botao_b = 1
    elif button_b.value() == 1 and estado_botao_b == 1:
        estado_botao_b = 0

# Função para normalizar um valor com base nos valores mínimo e máximo
def normalize(value, min_val, max_val):
    return (value - min_val) / (max_val - min_val)


# Função para calibrar os sensores e obter os valores mínimos e máximos
def calibrate_sensors(adc, samples=40):
    min_vals = [float('inf')] * 4
    max_vals = [-float('inf')] * 4
    
    ligado = 0
    
    for k in range(samples):
        for i, channel in enumerate([0, 1, 2, 3]):
            value = adc.read(rate=4, channel1=channel)
            if value < min_vals[i]:
                min_vals[i] = value
            if value > max_vals[i]:
                max_vals[i] = value
        print(min_vals, max_vals)
        sleep(0.1)
        print(k)
        if(k % 5 == 0):
            oled.fill(0)
            oled.text("Calibrando...", 0, 0)
            oled.text(str(k*2.5), 0, 10)
            oled.text("%", 30, 10)
            oled.show()
            if(ligado == 0):
                mostrarMatriz(all)
                ligado = 1
            else:
                mostrarMatriz(nothing)
                ligado = 0
    
    oled.fill(0)
    oled.text("Calibrado", 0, 0)
    oled.text("100", 0, 10)
    oled.text("%", 33, 10)
    oled.show()
    mostrarMatriz(all)
    sleep(0.5)
    mostrarMatriz(all)
    sleep(0.5)
    mostrarMatriz(nothing)
    return min_vals, max_vals

# Função para ler e normalizar os valores dos sensores
def read_normalized_values(adc, min_vals, max_vals):
    normalized_values = []
    for i, channel in enumerate([0, 1, 2, 3]):
        value = adc.read(rate=4, channel1=channel)
        norm_value = normalize(value, min_vals[i], max_vals[i])
        normalized_values.append(norm_value)
        #print(f"Valor normalizado do canal {channel}: {norm_value}")
    
    return normalized_values

def controle_pid():
    global integral, last_error, base_speed, sensores, min_vals, max_vals, kp, kd, ki, flag_parada
    
    # Ler e normalizar os valores dos sensores
    normalized_values = read_normalized_values(adc, min_vals, max_vals)
    
    if(not flag_parada):
        # Calcular a posição ponderada do robô com base nos sensores
        # Atribuir pesos aos sensores: mais à esquerda, maior peso negativo; mais à direita, maior peso positivo
        weights = [-3, -1, 1, 3]
        position = sum(weight * value for weight, value in zip(weights, normalized_values))
        
        # O erro é a posição ponderada em relação ao centro (0)
        error = position
        print("erro", error)
        # Calcular a parte proporcional
        P = Kp * error
        
        # Calcular a parte integral
        integral += error
        I = Ki * integral
        
        # Calcular a parte derivativa
        derivative = error - last_error
        D = Kd * derivative
        
        # Calcular a saída PID
        output = P + I + D
        
        # Atualizar o último erro
        last_error = error
        
        # Calcular as velocidades dos motores
        left_motor_speed = base_speed - int(output*0.8)
        if left_motor_speed > 100:
            left_motor_speed = 100 
        elif left_motor_speed < -100:
            right_motor_speed = -100
        
        right_motor_speed = base_speed + int(output*1.2)
        if right_motor_speed > 100:
            right_motor_speed = 100 
        elif right_motor_speed < -100:
            right_motor_speed = -100
        
        print("left: ", left_motor_speed)
        print("right: ", right_motor_speed)
        
        print(f"normalized_values: {normalized_values}")
        print("")
    
        # Definir a velocidade dos motores
        set_motor_speed("A", left_motor_speed)
        set_motor_speed("B", right_motor_speed)
    else:
        set_motor_speed("A", 0)
        set_motor_speed("B", 0)
    
    mostrarLinhaMatriz(normalized_values)

def esperarClickEmA():
    lerBotoes()
    while(estado_botao_a == 0):
        lerBotoes()
        sleep(0.01)


iniciaOled()
esperarClickEmA()

min_vals, max_vals = calibrate_sensors(adc, 40)

oled.fill(0)  # Limpar display
oled.text("BitDogSegue", 0, 0)
oled.text("Conecte o BT", 0, 10)
oled.show()

oled.fill(0)  # Limpar display
oled.show()

while True:
    controle_bluetooth()
    if(modo_de_operacao == 1):
        controle_bang_bang()
    elif(modo_de_operacao == 2):
        controle_pid()
    checarBotoes()
    
    sleep(0.01)  # Pequeno delay para evitar excesso de processamento


