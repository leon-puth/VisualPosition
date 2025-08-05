import time, math, os, gc, sys, _thread
from media.sensor import *
from media.display import *
from media.media import *
from machine import UART, FPIOA, Pin, SPI, WDT
################################################################################################################
#gloabl value
DETECT_WIDTH = ALIGN_UP(640, 16)
DETECT_HEIGHT = ALIGN_UP(480, 16)
codestatus = 0 #0: work, 1: debug
#################################################################################################################
class FileHandler:
    """
    The format of PixelProportion file is txt.
    The first number in the file is the pixel ratio, 
    and the second number is the initial position archive, front_position
    """
    def __init__(self, file_path):
        self.file_path = file_path
    def read_data(self):
        """
        Read two integer data (proportion, front_position) from the file,
        If the file does not exist or the format is incorrect, 
        return the default value
        """
        try:
            # Attempt to read file content
            with open(self.file_path, 'r', encoding='utf-8') as file:
                data = file.read().strip().split()
                if not data:
                    if len(data) != 2:
                        with open(self.file_path, 'w', 
                                  encoding='utf-8') as file:
                            file.write("25 500")
                            return 25, 500
                else:
                    return int(data[0]), int(data[1])
        except:
            # If the file does not exist or there is an error reading, 
            # create and write the default value
            with open(self.file_path, 'w') as file:
                file.write("25 500")
                return 25, 500

    def write_data(self, num1, num2):
        """
        Write two integer data (proportion, front_position)
        to a file
        """
        try:
            with open(self.file_path, 'w', encoding='utf-8') as file:
                file.write(f"{num1} {num2}")
        except Exception as e:
            print(f"Error occurred while writing file:{e}")

    def update_data(self, num1=None, num2=None):
        """
        Update specified integer data, 
        can update one or two data separately
        """
        # Read existing data first
        current_num1, current_num2 = self.read_data()

        # Update specified data
        if num1 is not None:
            current_num1 = num1
        if num2 is not None:
            current_num2 = num2

        # Write updated data
        self.write_data(current_num1, current_num2)

    def read_proportion(self):
        """Read pixel coefficients"""
        num1, _ = self.read_data()
        return num1

    def read_front_position(self):
        """Read the starting position"""
        _, num2 = self.read_data()
        return num2

    def write_prportion(self, num1):
        """Write pixel coefficients"""
        self.update_data(num1=num1)

    def write_front_position(self, num2):
        """Write starting position"""
        self.update_data(num2=num2)
#################################################################################################################
class WS2812B:
    """
    The class is used for WS2812B LED
    """
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
#work global variable
ReqTelBuf = bytearray(1)
statusbits = b'\x00\x01\x02\x04\x08' # statusbits: ev,np,err,wrn
count_of_has_position = 0
error_of_none_position = 2
fh = FileHandler("/data/PixelProportion.txt")
proportion,front_position = fh.read_data()#propotion is defined by the distance between camera and datamatrix
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
    uart = UART(UART.UART1, baudrate=115200, bits=UART.EIGHTBITS, 
                parity=UART.PARITY_EVEN, stop=UART.STOPBITS_ONE)
    #spi interface:spi1 MOSI-16 MISO-17
    fpioa.set_function(16, fpioa.QSPI0_D0)
    leds = WS2812B()
    leds.set_single_color(0)
    return sensor, uart, leds
#############################################################################################################
sensor, uart, _ = hardware_init()
#############################################################################################################
class RingBuffer:
    """
    To store the snapshot, used as a buffer
    """
    def __init__(self, size=2):
        self.size = size
        self.buffer = [None] * size
        self.head = 0
        self.tail = 0
        self.full = False

    def put(self, img):
        if self.full:
            # If the buffer is full, 
            # move the tail pointer to overwrite the old image
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
    """
    To get the snapshot, and store into the buffer
    """
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
    """
    Extract the positional information from the scanned values 
    and determine whether the data is accurate
    :param payload: str
    """
    global front_position
    if not isinstance(payload, str) or len(payload) != 8:
        return None

    #Format of payload："E000028[" 、"D000037t"
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
def write_position_to_file(current_position):
    """
    Write current x-position data into the parameter file
    """
    global front_position, count_of_has_position, fh

    if current_position == -1:
        return None

    if count_of_has_position > 10:
        fh.write_front_position(current_position)
        count_of_has_position = 0
###############################################################################################################
def get_blobs_roi(dm_blobs, img_height):
    """
    if realization use blob desize area, the func
    is to adjust the picture, y and height 
    """
    y = 0
    h = img_height
    for blob in dm_blobs:
        _, y, _, h = blob.rect()
        # Adjust the height to an integer multiple of 8
        h = ((h + 7) // 8) * 8
        # Ensure that y does not exceed the image boundary
        y = max(0, y - (h - blob.h()) // 2)
        # Adjust y to ensure that the cropped area 
        # does not exceed the image boundary
        y = min(y, img_height - h)
    return y, h
##############################################################################################################
thresholds = [(180, 215)]
###############################################################################################################
def process_image():
    """
    prcess The obtained image and find datamatrix
    to get detected matrix(x, y, w, h, [payload])
    """
    image = None
    while image == None:
        os.exitpoint()
        buffer_lock.acquire()
        image = ring_buffer.get()
        buffer_lock.release()

    # Obtain the width and height of the original image
    img_height = image.height()
    img_width = image.width()
    camera_center_x = img_width // 2
    
    # Display.show_image(image)
    dm_blobs = image.find_blobs(thresholds, pixels_threshold=1000, 
                                area_threshold=140, merge=True)
    if not dm_blobs:
        # print("can't find blob")
        return image.find_datamatrices(effort = 400), camera_center_x
    y, h = get_blobs_roi(dm_blobs, img_height)
    if h < 60 :
        # print("no qr in blob")
        return image.find_datamatrices(effort = 400), camera_center_x
    else:
        # print("get blob success!")
        dm_img = image.copy(roi = (0, y, img_width, h))
        # Display.show_image(dm_img)
        return  dm_img.find_datamatrices(effort = 400), camera_center_x
###############################################################################################################
def caculate_xpostion():
    """Scan the data matrix and 
    obtain the final x-position through a series of calculations and processing
    :param current_position:int,The final x-position obtained
    :param camera_center_x: Image pixel center point
    :param payload:String, obtained QR code value
    :param number:int,The x-position segmented from the QR code value, in centimeters
    :type valid_positions:[int, ...],List of multiple x positions for image calculation
    """
    global front_position, proportion, count_of_has_position, ring_buffer
    current_position = -1

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
            current_position = round(10*position)
            front_position = current_position
            count_of_has_position += 1
            write_position_to_file(current_position)

        if current_position != -1 :
            break

    return current_position
#################################################################################################################
def uart_read(readbuf):
    global uart
    return uart.read(readbuf)
#################################################################################################################
def uart_send(writebuf):
    global uart
    uart.write(writebuf)
#############################################################################################################
#Please refer to page 22 of the instruction manual for detailed rules
def request_telegram():
    """Determine whether to send a request and 
    receive two byte verification
    :param readbufs:Received two check byte sequences eg:A35C
    """
    global ReqTelBuf
    readbufs = uart_read(2)
    if not readbufs or len(readbufs) < 2:
        return False
    byte0 = readbufs[0]
    byte1 = readbufs[1]

    #Verification rule: The first byte is 0xA_, 
    # the second byte is the inverse of the first byte, 
    # and the lower two bits of the first byte are assigned to ReqTelBuf
    if byte0 != ((~byte1) & 0xFF):
        return False

    if (byte0 | 0x03) == 0xA3:
        ReqTelBuf[0] = byte0
        return True
##############################################################################################################
def response_telegram(caculate_pos, i):
    """
    send x-position to slave
    """
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
    wdt1 = WDT(1, 3)
    while True:
        os.exitpoint()
        # print("Free RAM:", gc.mem_free())
        if request_telegram():
            x_position = caculate_xpostion()
            if response_telegram(x_position, 0):
                pass
            else:
                response_telegram(0,2)
        else:
            response_telegram(0,1)
            pass
        wdt1.feed()
################################################################################################################
#codestatus: debug
################################################################################################################
def distance_proportion():
    """Calculate the x-position of the initial position and obtain the pixel scale coefficient
    :typr first_payload:[int, ...],List of x positions obtained from the initial position
    :typr distance_proportion:[int, ...],Pixel Scale Coefficient List
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
################################################################################################################
#codestatus: debug
################################################################################################################
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



