from machine import I2C, Pin
import time



# 定义 I2C 地址和电机特定地址
I2C_ADDR = 0x34
ADC_BAT_ADDR = 0x00
MOTOR_TYPE_ADDR = 0x14
MOTOR_ENCODER_POLARITY_ADDR = 0x15
MOTOR_FIXED_PWM_ADDR = 0x1F
MOTOR_FIXED_SPEED_ADDR = 0x33
MOTOR_ENCODER_TOTAL_ADDR = 0x3C

# 使用 I2C 初始化 I2C
i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)

# 向 I2C 写入一个字节的函数
def wire_write_byte(val):
    i2c.writeto(I2C_ADDR, bytes([val]))

# 向 I2C 写入数据数组的函数
def wire_write_data_array(reg, val):
    data = bytearray([reg]) + bytearray(val)
    i2c.writeto(I2C_ADDR, data)

# 从特定地址读取一个字节的函数
def wire_read_data_byte(reg):
    i2c.writeto(I2C_ADDR, bytes([reg]))
    data = i2c.readfrom(I2C_ADDR, 1)
    return data[0]

# 初始化电机类型和编码器极性
MotorType = 3  # 示例电机类型值
MotorEncoderPolarity = 0
wire_write_data_array(MOTOR_TYPE_ADDR, [MotorType])
wire_write_data_array(MOTOR_ENCODER_POLARITY_ADDR, [MotorEncoderPolarity])

# 定义速度数组
p1 = [100, 100, 100, 100]
p2 = [-100, -100, -100, -100]
s1 = [50, 50, 50, 50]
s2 = [-50, -50, -50, -50]
stop = [0, 0, 0, 0]
EncodeTotal = [0, 0, 0, 0]  # 初始化电机编码器总值

# 定义启动和停止按钮引脚
start_button_pin_1 = Pin(23, Pin.IN, Pin.PULL_UP)
start_button_pin_2 = Pin(25, Pin.IN, Pin.PULL_UP)
stop_button_pin_1 = Pin(26, Pin.IN, Pin.PULL_UP)
stop_button_pin_2 = Pin(32, Pin.IN, Pin.PULL_UP)

#Pin 34 與 usb uart



motor_running = False

def start_motor():
    global motor_running
    if not motor_running:
        wire_write_data_array(MOTOR_FIXED_SPEED_ADDR, p1)
        motor_running = True

def stop_motor():
    global motor_running
    if motor_running:
        wire_write_data_array(MOTOR_FIXED_SPEED_ADDR, stop)
        motor_running = False

# 控制电机的主循环
while True:
    # 读取启动和停止按钮状态
    if start_button_pin_1.value() == 0:
        start_motor()
    if start_button_pin_2.value() == 0:
        start_motor()

    if stop_button_pin_2.value() == 0:
        stop_motor()
    if stop_button_pin_1.value() == 0:
        stop_motor()

    # 读取电压值
    data = bytearray(2)
    i2c.writeto(I2C_ADDR, bytes([ADC_BAT_ADDR]))
    i2c.readfrom(I2C_ADDR, 2, data)
    v = data[0] + (data[1] << 8)
    print("V = {}mV".format(v))

    # 读取电机编码器总值
    data = bytearray(16)
    i2c.readfrom_mem_into(I2C_ADDR, MOTOR_ENCODER_TOTAL_ADDR, data)
    EncodeTotal = [int.from_bytes(data[i:i+4], 'little') for i in range(0, len(data), 4)]

    time.sleep(0.1)