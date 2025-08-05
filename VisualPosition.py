################################################################################################################
#程序使用说明：
#1.该程序用于通过二维码扫描获取物体的x位置，适用于需要精确定位的应用场景。
#2.读头离二维码的高度设定为12cm±4cm，若布置高度和设定高度差异大，可运行PixelProportion.py脚本，通过计算二维码的像素比例系数，来提高定位精度。
#3.读头的初始位置为上一次运行结束时的位置，若读头重新安装在新的位置，有必要运行PixelProportion.py脚本，来更新初始位置。
################################################################################################################
import time, math, os, gc, sys, _thread
from media.sensor import *
from media.display import *
from media.media import *
from machine import UART, FPIOA, Pin, SPI, WDT
################################################################################################################
class FileHandler:
    """PixelProportion文件的格式为.txt
    文件的第一个数是像素比例系数proportion,第二个数是初始位置存档,front_position
    """
    def __init__(self, file_path):
        self.file_path = file_path
    def read_data(self):
        """从文件中读取两个整数数据(proportion, front_position)，
        如果文件不存在或格式错误，返回默认值
        """
        try:
            # 尝试读取文件内容
            with open(self.file_path, 'r', encoding='utf-8') as file:
                data = file.read().strip().split()
                if not data:
                    if len(data) != 2:
                        with open(self.file_path, 'w', encoding='utf-8') as file:
                            file.write("25 500")
                            return 25, 500
                else:
                    return int(data[0]), int(data[1])
        except:
            # 如果文件不存在或读取时出错，创建并写入默认值
            with open(self.file_path, 'w') as file:
                file.write("25 500")
                return 25, 500

    def write_data(self, num1, num2):
        """将两个整数数据(proportion, front_position)写入文件"""
        try:
            with open(self.file_path, 'w', encoding='utf-8') as file:
                file.write(f"{num1} {num2}")
        except Exception as e:
            print(f"写入文件时发生错误：{e}")

    def update_data(self, num1=None, num2=None):
        """更新指定的整数数据，可以单独更新一个或两个数据"""
        # 先读取现有数据
        current_num1, current_num2 = self.read_data()

        # 更新指定的数据
        if num1 is not None:
            current_num1 = num1
        if num2 is not None:
            current_num2 = num2

        # 写入更新后的数据
        self.write_data(current_num1, current_num2)

    def read_proportion(self):
        """读取 像素系数"""
        num1, _ = self.read_data()
        return num1

    def read_front_position(self):
        """读取 起始位置"""
        _, num2 = self.read_data()
        return num2

    def write_prportion(self, num1):
        """写入 像素系数"""
        self.update_data(num1=num1)

    def write_front_position(self, num2):
        """写入 起始位置"""
        self.update_data(num2=num2)
#################################################################################################################
class WS2812B:
    def __init__(self, spi_id = 1, num_leds = 9, baudrate=5000000):
        self.spi = SPI(spi_id, baudrate=baudrate, polarity=0, phase=0, bits=8)
        self.num_leds = num_leds
        self.wirtebuffer = bytearray(num_leds*24)#Each LED requires 24 bits of data (8 bits each for R+G+B)

    def set_pixel(self, led_id, r, g, b):
        offset = led_id * 24  # Each LED bead occupies 24 bits
        for i in range(8):
            # Convert RGB data to SPI timing (0=0xF0, 1=0xC0)
            self.wirtebuffer[offset + i] = 0xF0 if (g & (1 << (7 - i))) else 0xC0  # Green
            self.wirtebuffer[offset + 8 + i] = 0xF0 if (r & (1 << (7 - i))) else 0xC0  # Red
            self.wirtebuffer[offset + 16 + i] = 0xF0 if (b & (1 << (7 - i))) else 0xC0  # Blue

    def show(self):
        self.spi.write(self.wirtebuffer)
        time.sleep_us(50)  # WS2812B requires a low-level reset signal of at least 50us

    def set_single_color(self, option):
        #option:0-red;1-green;2-blue
        n = self.num_leds
        colors = {
            0: (255, 0, 0),     # red
            1: (0, 255, 0),     # green
            2: (0, 0, 255),     # blue
            3: (255, 255, 255), #white
            4: (255, 255, 0),   #yellow
            5: (148, 0, 211),   #purple
            6: (0, 0, 0)        #none
        }
        if option not in colors:
            # print("Invalid option")
            return
        color = colors[option]
        colors_data = [color for _ in range(n)]
        for i in range(n):
            self.set_pixel(i, *colors_data[i])
        self.show()
################################################################################################################
#codestatus: work
################################################################################################################
#global variable
DETECT_WIDTH = ALIGN_UP(640, 16)
DETECT_HEIGHT = ALIGN_UP(480, 16)
ReqTelBuf = bytearray(1)
statusbits = b'\x00\x01\x02\x04\x08' # statusbits: ev,np,err,wrn
count_of_has_position = 0
error_of_none_position = 2
fh = FileHandler("/data/PixelProportion.txt")
proportion,front_position = fh.read_data()
codestatus = 0 #0: work, 1: debug
################################################################################################################
def hardware_init():
    # camera initialization
    sensor = Sensor(id = 0, width = DETECT_WIDTH, height = DETECT_HEIGHT, fps=30)
    sensor.reset()
    sensor.set_framesize(width = DETECT_WIDTH, height = DETECT_HEIGHT)
    sensor.set_pixformat(Sensor.GRAYSCALE)
    # Display.init(Display.VIRT, width = DETECT_WIDTH, height = DETECT_HEIGHT, to_ide = True, fps = 30)
    MediaManager.init()
    sensor.run()
    #rs485 interface: UART1 TX-GPIO3, RX-GPIO4
    fpioa = FPIOA()
    fpioa.set_function(3, fpioa.UART1_TXD)
    fpioa.set_function(4, fpioa.UART1_RXD)
    #8-E-1 Mode
    uart = UART(UART.UART1, baudrate=115200, bits=UART.EIGHTBITS, parity=UART.PARITY_EVEN, stop=UART.STOPBITS_ONE)
    #spi interface:spi1 MOSI-16 MISO-17
    fpioa.set_function(16, fpioa.QSPI0_D0)
    leds = WS2812B()
    leds.set_single_color(0)
    return sensor, uart, leds
#################################################################################################################
sensor, uart, _ = hardware_init()
#################################################################################################################
def uart_read(readbuf):
    global uart
    return uart.read(readbuf)
#################################################################################################################
def uart_send(writebuf):
    global uart
    uart.write(writebuf)
#############################################################################################################
class RingBuffer:
    def __init__(self, size=2):
        self.size = size
        self.buffer = [None] * size
        self.head = 0
        self.tail = 0
        self.full = False

    def put(self, img):
        if self.full:
            # 如果缓冲区已满，移动尾指针以覆盖旧图片
            self.tail = (self.tail + 1) % self.size
            self.full = False
        self.buffer[self.head] = img
        self.head = (self.head + 1) % self.size
        if self.head == self.tail:
            self.full = True

    def get(self):
        if self.empty():
            return None
        img = self.buffer[self.tail]
        self.tail = (self.tail + 1) % self.size
        self.full = False
        return img

    def empty(self):
        return self.head == self.tail and not self.full

    def is_full(self):
        return self.full
############################################################################################################
ring_buffer = RingBuffer()
buffer_lock = _thread.allocate_lock()
###############################################################################################################
def safe_snapshot():
    global sensor, ring_buffer
    try:
        while True:
            os.exitpoint()
            buffer_lock.acquire()
            ring_buffer.put(sensor.snapshot().lens_corr(0.2).\
                copy(roi = (3*DETECT_WIDTH//8, DETECT_HEIGHT//4, DETECT_WIDTH//4, DETECT_HEIGHT//2),
                copy_to_fb= True))
            buffer_lock.release()
            gc.collect()
    except:
        sensor.reset()
###############################################################################################################
#Split QR code string，string format:
# "E000028["
# "D000037t"
def split_payload(payload):
    """将扫描所得值中的位置信息提取出来，并判断数据是否准确
    :param payload:字符串
    """
    global front_position
    if not isinstance(payload, str) or len(payload) != 8:
        return None

    #payload的格式："E000028[" 、"D000037t"
    if (payload[0] != 'E') and (payload[0] != 'D'):
        return None

    try:
        current_payload = payload[1:7]
    except ValueError:
        return None

    for s in current_payload:
        if not s.isdigit():
            return None
# Directly slice and extract the numerical part
#If the current location is not on the same order of magnitude as the previous location,
# it is considered that there is a problem with the data
    if front_position == 0 :
        return int(current_payload)

    if (int(current_payload) / front_position) < 1:
        return int(current_payload)
    else:
        return None
###############################################################################################################
#To determine if the position signal is error or lost, filter the position signal.
def write_position_to_file(current_position):
    """将计算的x位置数据写入参数文件
    """
    global front_position, count_of_has_position, fh

    if current_position == -1:
        return None

    if count_of_has_position > 10:
        fh.write_front_position(current_position)
        count_of_has_position = 0
###############################################################################################################
def get_blobs_roi(qr_blobs, img_height):
    y = 0
    h = img_height
    for blob in qr_blobs:
        _, y, _, h = blob.rect()
        # 调整高度为 8 的整数倍
        h = ((h + 7) // 8) * 8
        # 确保 y 不超出图像边界
        y = max(0, y - (h - blob.h()) // 2)
        # 调整 y，确保截取区域不超出图像边界
        y = min(y, img_height - h)
    return y, h
##############################################################################################################
thresholds = [(180, 215)]
###############################################################################################################
def process_image():
    image = None
    while image == None:
        os.exitpoint()
        buffer_lock.acquire()
        image = ring_buffer.get()
        buffer_lock.release()
    # 获取原始图像的宽度和高度
    img_height = image.height()
    img_width = image.width()
    camera_center_x = img_width // 2
    # Display.show_image(image)
    qr_blobs = image.find_blobs(thresholds, pixels_threshold=1000, area_threshold=140, merge=True)
    if not qr_blobs:
        # print("can't find blob")
        return image.find_datamatrices(effort = 400), camera_center_x
    y, h = get_blobs_roi(qr_blobs, img_height)
    if h < 60 :
        # print("no qr in blob")
        return image.find_datamatrices(effort = 400), camera_center_x
    else:
        # print("get blob success!")
        qrcode_img = image.copy(roi = (0, y, img_width, h))
        # Display.show_image(qrcode_img)
        return  qrcode_img.find_datamatrices(effort = 400), camera_center_x
###############################################################################################################
def caculate_xpostion():
    """扫描二维码，并通过一系列计算处理得到最终x位置
    :param avg_position:int,得到的最终x位置
    :param camera_center_x: 图片像素中心点
    :param payload:字符串，得到的二维码值
    :param number:int,从二维码值中分割得到的x位置,单位cm
    :type valid_positions:[int, ...],图片计算的多个x位置的列表
    """
    global front_position, proportion, count_of_has_position, ring_buffer
    avg_position = -1

    for i in range(error_of_none_position):
        os.exitpoint()

        detected_matrices = []
        detected_matrices, camera_center_x = process_image()
#        print(f"camera_center_x is {camera_center_x}")

        if not detected_matrices:
            count_of_has_position = 0
            # print("detected matrix is empty")
            break

        number_of_min_distance = None
        min_distance = camera_center_x
        for matrix in detected_matrices:
            payload = matrix.payload()
            if payload is None:
                # print("payload is null")
                continue

            number = split_payload(payload) #(mm)
            if number is None:
                # print("payload is wrong")
                continue

            corners = matrix.corners()
            qr_center_x_list = []
            for t in corners:
                qr_center_x_list.append(t[0])
            qr_center_x = sum(qr_center_x_list)/len(qr_center_x_list)
            # print(f"qr_center_x is {qr_center_x}")
            distance_to_camera_center = camera_center_x - qr_center_x
            if abs(distance_to_camera_center) < min_distance:
                min_distance = distance_to_camera_center
                # print(f"min_distance is {min_distance}")
                number_of_min_distance = number
                # print(f"number_of_min_distance is {number_of_min_distance}")

        if number_of_min_distance is not None:
            position = min_distance/proportion + number_of_min_distance*1
            avg_position = round(10*position)
            front_position = avg_position
            count_of_has_position += 1
            write_position_to_file(avg_position)

        if avg_position != -1 :
            break

    return avg_position
#############################################################################################################
#Please refer to page 22 of the instruction manual for detailed rules
def request_telegram():
    """判断是否发送请求，接收两个字节校验
    :param readbufs:接收的两个校验字节序列 eg:A35C
    """
    global ReqTelBuf
    readbufs = uart_read(2)
    if not readbufs or len(readbufs) < 2:
        return False
    byte0 = readbufs[0]
    byte1 = readbufs[1]

    #校验规则：第一个字节为0xA_,第二个字节为第一个字节取反，并将第一个字节低两位赋给ReqTelBuf
    if byte0 != ((~byte1) & 0xFF):
        return False

    if (byte0 | 0x03) == 0xA3:
        ReqTelBuf[0] = byte0
        return True
##############################################################################################################
def response_telegram(caculate_pos, i):
    global ReqTelBuf, statusbits
    if caculate_pos == -1:
        return False
    #byte1:
    writebuf = bytearray(9)
    head_address = ReqTelBuf[0] & 0x03
    writebuf[0] = statusbits[i] | (head_address<<4)

    for j in range(1, 9):
        writebuf[j] = 0x00
    if i == 0:
        #byte2~byte6
        writebuf[1] = (caculate_pos>>21) & 0x00000007
        writebuf[2] = (caculate_pos>>14) & 0x0000007F
        writebuf[3] = (caculate_pos>>7) & 0x0000007F
        writebuf[4] = caculate_pos & 0x0000007F

    uart_send(writebuf)

    return True
##################################################################################################################
def main():
    #    wdt1 = WDT(1, 3)
    while True:
        os.exitpoint()
        x_position = caculate_xpostion()
        if x_position != -1:
            uart_send("\r\n")
            uart_send(f"x position is {x_position}\r\n")
            uart_send("\r\n")
            print(x_position)
        else:
            uart_send("null\r\n")
        # print("Free RAM:", gc.mem_free())
        # os.exitpoint()
        # if request_telegram():
        #     x_position = caculate_xpostion()
        #     if response_telegram(x_position, 0):
        #         pass
        #     else:
        #         response_telegram(0,2)
        # else:
        #         response_telegram(0,1)
        #     pass
        # wdt1.feed()
#################################################################################################################
################################################################################################################
#codestatus: debug
################################################################################################################
def distance_proportion():
    """计算初始位置的x位置,并得到像素比例系数
    :typr first_payload:[int, ...],初始位置得到的x位置列表
    :typr distance_proportion:[int, ...],像素比例系数列表
    """
    global proportion, front_position, ring_buffer

    detected_matrices = []
    timeout = 0
    while True:
        os.exitpoint()
        buffer_lock.acquire()
        distance_img = ring_buffer.get()
        buffer_lock.release()
        detected_matrices = distance_img.find_datamatrices(effort = 400)
        distance_img.clear()
        del distance_img
        timeout+=1
        if detected_matrices:
            break
        if timeout > 2000:
            print("plesae reset and try again")
            return None


    first_payload = []
    distance_proportion = []
    for matrix in detected_matrices:
        if matrix:
            n = (matrix.w() + matrix.h()) // 2
            distance_proportion.append(n)
            try:
                payload = matrix.payload()
                # print(f"payload is {payload}")
                paynumber = split_payload(payload)
                if paynumber is not None:
                    first_payload.append(paynumber)
            except Exception as e:
                print(f"Error processing matrix payload: {e}")
    del detected_matrices

    if first_payload:
        front_position = 10 * sum(first_payload) // len(first_payload)  # Rough position
    else:
        print("Don't get the right position, plesase try again")

    if distance_proportion:
        pixel_diff = sum(distance_proportion) // len(distance_proportion)
        proportion = round(pixel_diff)
    else:
        print("Don't get the right propotion, plesase try again")
    # print(f"{proportion}, {front_position}")
    print("Debug success!")
    gc.collect()
################################################################################################################
def pixel_proportion():
    global proportion, front_position
    file_handler = FileHandler("/data/PixelProportion.txt")
    proportion, front_position = file_handler.read_data()
    # print(f"proportion is {proportion}")
    # print(f"front_position is {front_position}")
    distance_proportion()
    file_handler.write_data(proportion, front_position)
###############################################################################################################
##################################################################################################################
if __name__ == '__main__':
    print("start")
    try:
        _thread.start_new_thread(safe_snapshot, ())
        if codestatus == 0:
            main()
        else:
            pixel_proportion()
    except KeyboardInterrupt as e:
        print(f"user stop")
    except BaseException as e:
        print(f"Exception '{e}'")
    except:
        pass
    finally:
        uart.deinit()
        if isinstance(sensor, Sensor):
            sensor.stop()
        Display.deinit()
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        MediaManager.deinit()
        gc.collect()



