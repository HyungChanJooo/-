import cv2
import numpy as np
import matplotlib.pyplot as plt
import pytesseract
import time
pytesseract.pytesseract.tesseract_cmd = r'/usr/bin/tesseract'
from picamera2 import Picamera2
import re
import serial
import smbus2
import RPi.GPIO as GPIO

# 초음파 센서 설정
TRIG1 = 17
ECHO1 = 27
TRIG2 = 22
ECHO2 = 23

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG1, GPIO.OUT)
GPIO.setup(ECHO1, GPIO.IN)
GPIO.setup(TRIG2, GPIO.OUT)
GPIO.setup(ECHO2, GPIO.IN)


#LCD 패널 설정
I2C_ADDR = 0x27
LCD_WIDTH = 16
LCD_CHR = 1
LCD_CMD = 0
LCD_LINE_1 = 0x80
LCD_LINE_2 = 0xC0
LCD_BACKLIGHT = 0x08
ENABLE = 0b00000100
bus = smbus2.SMBus(1)


# 아두이노와 블루투스 통신

ard = serial.Serial('/dev/rfcomm0', 9600)
ard.reset_input_buffer()


# 주차 자리 층별 list 설정
line_list = []
enter_list = [0 for i in range(18)]
prk_area_1=[0 for i in range(6)]
prk_area_2=[0 for i in range(6)]
prk_area_3=[0 for i in range(6)]

# 입주민 차량번호 list
with open('resident.txt', 'r', encoding='utf-8') as file:
    for line in file:
        
        line_list.append(line.strip())

# 초음파센서 거리 측정 함수
def measure_distance(trig, echo):
   
    
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)

    
    start_time = time.time()
    stop_time = time.time()

    while GPIO.input(echo) == 0:
        start_time = time.time()
    
    while GPIO.input(echo) == 1:
        stop_time = time.time()

   
    elapsed_time = stop_time - start_time
    distance = (elapsed_time * 34300) / 2  
    return distance
        


# lcd 패널 화면 초기화 함수
def lcd_init():
    
    lcd_byte(0x33, LCD_CMD)
    lcd_byte(0x32, LCD_CMD) 
    lcd_byte(0x06, LCD_CMD)  
    lcd_byte(0x0C, LCD_CMD)  
    lcd_byte(0x28, LCD_CMD)  
    lcd_byte(0x01, LCD_CMD)  
    time.sleep(0.005)

#LCD 데이터 전송 함수
def lcd_byte(bits, mode):
    
    bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT
    bits_low = mode | ((bits << 4) & 0xF0) | LCD_BACKLIGHT

   
    bus.write_byte(I2C_ADDR, bits_high)
    lcd_toggle_enable(bits_high)

   
    bus.write_byte(I2C_ADDR, bits_low)
    lcd_toggle_enable(bits_low)

#LCD 데이터 처리 함수
def lcd_toggle_enable(bits):
   
    time.sleep(0.0005)
    bus.write_byte(I2C_ADDR, (bits | ENABLE))
    time.sleep(0.0005)
    bus.write_byte(I2C_ADDR, (bits & ~ENABLE))
    time.sleep(0.0005)


#LCD 화면 출력 함수
def lcd_string(message, line):
  
    message = message.ljust(LCD_WIDTH, " ")
    lcd_byte(line, LCD_CMD)
    for char in message:
        lcd_byte(ord(char), LCD_CHR)


# 아두이노 신호 전송 함수
def ard_send(coomand,number):
    ard.write(f"{command},{number}\n".encode('utf-8'))
    time.sleep(0.1)


#카메라 작동 및 촬영 함수
def cam():
    picam2 = Picamera2()
    picam2.configure(picam2.create_still_configuration())
    picam2.start()
    frame = picam2.capture_array()
    cv2.imwrite('car.png', frame)
    picam2.close


#입출차 알고리즘 함수
def ent(ent_chars):
    rr=-1

    aa=bb=cc = 1
    for i in range(18):
        if ent_chars == line_list[i]:       
            if i < 6:
                for j in range(6):
                    aa *= prk_area_1[j]
                if aa == 0:
                    for k in range(6):
                        if prk_area_1[k]==0:
                            prk_area_1[k]=1
                            if k < 2:
                                ard_send("ENTER",k)
                                enter_list[k]=ent_chars 
                                rr=k
                                return rr
                                
                            elif k < 4:
                                ard_send("ENTER",k+4)
                                enter_list[k+4]=ent_chars
                                rr=k+4
                                return rr
                                
                            elif k < 6:
                                ard_send("ENTER",k+8)
                                enter_list[k+8]=ent_chars
                                rr=k+8
                                return rr
                elif aa == 1:
                    for p in range(0,5,2): # [0,2,4]
                        if prk_area_2[p]==0:    
                            prk_area_2[p]=1
                            if p < 1:
                                ard_send("ENTER",p+2)
                                enter_list[p+2]=ent_chars
                                rr=p+2
                                return rr
                                
                            elif p < 3:
                                ard_send("ENTER",p+6)
                                enter_list[p+6]=ent_chars
                                rr=p+6
                                return rr
                                
                            elif p < 5:
                                ard_send("ENTER",p+10)
                                enter_list[p+10]=ent_chars
                                rr=p+10
                                return rr
                                
                        elif prk_area_2[p+1]==0:
                            prk_area_2[p+1]=1
                            if p < 1:
                                ard_send("ENTER",p+3)
                                enter_list[p+3]=ent_chars
                                rr=p+3
                                return rr
                                
                            elif p < 3:
                                ard_send("ENTER",p+7)
                                enter_list[p+7]=ent_chars
                                rr= p+7
                                return rr
                                
                            elif p < 5:
                                ard_send("ENTER",p+11)
                                enter_list[p+11]=ent_chars
                                rr= p+11
                                return rr
                                

                        elif prk_area_3[p]==0:
                            prk_area_3[p]=1
                            if p < 1:
                                ard_send("ENTER",p+4)
                                enter_list[p+4]=ent_chars
                                rr= p+4
                                return rr
                                
                            elif p < 3:
                                ard_send("ENTER",p+8)
                                enter_list[p+8]=ent_chars
                                rr=p+3
                                return rr
                                
                            elif p < 5:
                                ard_send("ENTER",p+12)
                                enter_list[p+12]=ent_chars
                                rr=p+8
                                return rr
                                
                        elif prk_area_3[p+1]==0:
                            prk_area_3[p+1]=1
                            if p < 1:
                                ard_send("ENTER",p+5)
                                enter_list[p+5]=ent_chars
                                rr=p+5
                                return rr
                                
                            elif p < 3:
                                ard_send("ENTER",p+9)
                                enter_list[p+9]=ent_chars
                                rr=p+9
                                return rr
                                
                            elif p < 5:
                                ard_send("ENTER",p+13)
                                enter_list[p+13]=ent_chars
                                rr=p+13
                                return rr
            elif i < 12:
                for j in range(6):
                    bb *=prk_area_2[j]
                    if bb == 0:
                        for k in range(6):
                            if prk_area_2[k]==0:
                                prk_area_2[k]=1
                                if k < 2:
                                    ard_send("ENTER",k+2)
                                    enter_list[k+2]=ent_chars
                                    rr=k+2
                                    return rr
                                    
                                elif k < 4:
                                    ard_send("ENTER",k+6)
                                    enter_list[k+6]=ent_chars
                                    rr=k+6
                                    return rr
                                    

                                elif k < 6:
                                    ard_send("ENTER",k+10)
                                    enter_list[k+10]=ent_chars
                                    rr=k+10
                                    return rr
                    elif bb == 1:
                        for p in range(0,5,2):
                            if prk_area_1[p]==0:
                                prk_area_1[p]=1
                                if p < 1:
                                    ard_send("ENTER",p)
                                    enter_list[p]=ent_chars
                                    rr=p
                                    return rr
                                    
                                elif p < 3:
                                    ard_send("ENTER",p+4)
                                    enter_list[p+4]=ent_chars
                                    rr=p+4
                                    return rr
                                elif p < 5:
                                    ard_send("ENTER",p+8)
                                    enter_list[p+8]=ent_chars
                                    rr=p+8
                                    return rr    
                            elif prk_area_1[p+1]==0:
                                prk_area_1[p+1]=1
                                if p < 1:
                                    ard_send("ENTER",p+1)
                                    enter_list[p+1]=ent_chars
                                    rr=p+1
                                    return rr
                                elif p < 3:
                                    ard_send("ENTER",p+5)
                                    enter_list[p+5]=ent_chars
                                    rr=p+5
                                    return rr
                                elif p < 5:
                                    ard_send("ENTER",p+9)
                                    enter_list[p+9]=ent_chars
                                    rr=p+9
                                    return rr
                            elif prk_area_3[p]==0:
                                prk_area_3[p]=1
                                if p < 1:
                                    ard_send("ENTER",p+9)
                                    enter_list[p+4]=ent_chars
                                    rr=p+9
                                    return rr
                                elif p < 3:
                                    ard_send("ENTER",p+8)
                                    enter_list[p+8]=ent_chars
                                    rr=p+8
                                    return rr
                                elif p < 5:
                                    ard_send("ENTER",p+12)
                                    enter_list[p+12]=ent_chars
                                    rr=p+12
                                    return rr
                            elif prk_area_3[p+1]==0:
                                prk_area_3[p+1]=1
                                if p < 1:
                                    ard_send("ENTER",p+5)
                                    enter_list[p+5]=ent_chars
                                    rr=p+5
                                    return rr
                                elif p < 3:
                                    ard_send("ENTER",p+9)
                                    enter_list[p+9]=ent_chars
                                    rr=p+9
                                    return rr
                                elif p < 5:
                                    ard_send("ENTER",p+13)
                                    enter_list[p+13]=ent_chars
                                    rr=p+13
                                    return rr
            elif i < 18:
                for j in range(6):
                    cc *= prk_area_3[j]
                if cc == 0:
                    for k in range(6):
                        if prk_area_3[k]==0:
                            prk_area_3[k]=1
                            if k < 2:
                                ard_send("ENTER",k+4)
                                enter_list[k+4]=ent_chars
                                rr=k+4
                                return rr
                            elif k < 4:
                                ard_send("ENTER",k+8)
                                enter_list[k+8]=ent_chars
                                rr=k+8
                                return rr
                            elif k < 6:
                                ard_send("ENTER",k+12)
                                enter_list[k+12]=ent_chars
                                rr=k+12
                                return rr
                elif cc == 1 :
                    for p in range(0,5,2): # [0,2,4]
                         
                        if prk_area_2[p]==0:    
                            prk_area_2[p]=1
                            if p < 1:
                                ard_send("ENTER",p+2)
                                enter_list[p+2]=ent_chars
                                rr=p+2
                                return rr
                            elif p < 3:
                                ard_send("ENTER",p+6)
                                enter_list[p+6]=ent_chars
                                rr=p+6
                                return rr
                            elif p < 5:
                                ard_send("ENTER",p+10)
                                enter_list[p+10]=ent_chars
                                rr=p+10
                                return rr
                        elif prk_area_2[p+1]==0:
                            prk_area_2[p+1]=1
                            if p < 1:
                                ard_send("ENTER",p+3)
                                enter_list[p+3]=ent_chars
                                rr=p+3
                                return rr
                            elif p < 3:
                                ard_send("ENTER",p+7)
                                enter_list[p+7]=ent_chars
                                rr=p+7
                                return rr
                            elif p < 5:
                                ard_send("ENTER",p+11)
                                enter_list[p+11]=ent_chars
                                rr=p+11
                                return rr
                        elif prk_area_1[p]==0:
                            prk_area_1[p]=1
                            if p < 1:
                                ard_send("ENTER",p)
                                enter_list[p]=ent_chars
                                rr=p
                                return rr
                            elif p < 3:
                                ard_send("ENTER",p+4)
                                enter_list[p+4]=ent_chars
                                rr=p+4
                                return rr
                            elif p < 5:
                                ard_send("ENTER",p+8)
                                enter_list[p+8]=ent_chars
                                rr=p+8
                                return rr   
                        elif prk_area_1[p+1]==0:
                            prk_area_1[p+1]=1
                            if p < 1:
                                ard_send("ENTER",p+1)
                                enter_list[p+1]=ent_chars
                                rr=p+1
                                return rr
                            elif p < 3:
                                ard_send("ENTER",p+5)
                                enter_list[p+5]=ent_chars
                                rr=p+5
                                return rr
                            elif p < 5:
                                ard_send("ENTER",p+9)
                                enter_list[p+9]=ent_chars
                                rr=p+9
                                return rr
    if rr = -1:
        return rr
def exit(exit_chars):
    for i in range(18):
        if exit_chars == enter_list[i]:
            enter_list[i]=0
            ard_send("EXIT",i)
            if i < 2 :
                prk_area_1[i]=0
                break
            elif i < 4 :
                prk_area_2[i-2]=0
                break
            elif i < 6 :
                prk_area_3[i-4]=0
                break
            elif i < 8 :
                prk_area_1[i-4]=0
                break
            elif i < 10 :
                prk_area_2[i-6]=0
                break
            elif i < 12 :
                prk_area_3[i-8]=0
                break
            elif i < 14 :
                prk_area_1[i-8]=0
                break
            elif i < 16 :
                prk_area_2[i-10]=0
                break
            elif i < 18 :
                prk_area_1[i-12]=0
                break


# 문자열 인식 함수
def find_chars(contour_list): 
    matched_result_idx = []

    for d1 in contour_list:
        matched_contours_idx = []
        for d2 in contour_list:
            if d1['idx'] == d2['idx']:
                continue
            dx = abs(d1['cx'] - d2['cx'])
            dy = abs(d1['cy'] - d2['cy'])
            diagonal_length1 = np.sqrt(d1['w'] ** 2 + d1['h'] ** 2)
            distance = np.linalg.norm(np.array([d1['cx'], d1['cy']]) - np.array([d2['cx'], d2['cy']]))
            angle_diff = np.degrees(np.arctan(dy / dx)) if dx != 0 else 90
            area_diff = abs(d1['w'] * d1['h'] - d2['w'] * d2['h']) / (d1['w'] * d1['h'])
            width_diff = abs(d1['w'] - d2['w']) / d1['w']
            height_diff = abs(d1['h'] - d2['h']) / d1['h']

            if distance < diagonal_length1 * 5 and angle_diff < 12.0 and area_diff < 0.5 and width_diff < 0.8 and height_diff < 0.2:
                matched_contours_idx.append(d2['idx'])

        matched_contours_idx.append(d1['idx'])

        if len(matched_contours_idx) < 3:
            continue

        matched_result_idx.append(matched_contours_idx)
        unmatched_contour_idx = [d['idx'] for d in contour_list if d['idx'] not in matched_contours_idx]
        unmatched_contour = [contour_list[i] for i in unmatched_contour_idx]

        if unmatched_contour:  # Proceed with recursion if there are unmatched contours
            recursive_contour_list = find_chars(unmatched_contour)
            matched_result_idx.extend(recursive_contour_list)

        break

    return matched_result_idx


#문자 추출 함수
def text():
    plt.style.use('dark_background')
    img_ori = cv2.imread('car.png')

    height, width, channel = img_ori.shape
    gray = cv2.cvtColor(img_ori, cv2.COLOR_BGR2GRAY)
    structuringElement = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

    imgTopHat = cv2.morphologyEx(gray, cv2.MORPH_TOPHAT, structuringElement)
    imgBlackHat = cv2.morphologyEx(gray, cv2.MORPH_BLACKHAT, structuringElement)
    imgGrayscalePlusTopHat = cv2.add(gray, imgTopHat)
    gray = cv2.subtract(imgGrayscalePlusTopHat, imgBlackHat)
    img_blurred = cv2.GaussianBlur(gray, ksize=(5, 5), sigmaX=0)

    img_thresh = cv2.adaptiveThreshold(
        img_blurred, 
        maxValue=255.0, 
        adaptiveMethod=cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
        thresholdType=cv2.THRESH_BINARY_INV, 
        blockSize=19, 
        C=9
    )
    contours, _ = cv2.findContours(
        img_thresh, 
        mode=cv2.RETR_LIST, 
        method=cv2.CHAIN_APPROX_SIMPLE
    )

    contours_dict = []
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        contours_dict.append({
            'contour': contour,
            'x': x,
            'y': y,
            'w': w,
            'h': h,
            'cx': x + (w / 2),
            'cy': y + (h / 2)
        })
    MIN_AREA = 80
    MIN_WIDTH, MIN_HEIGHT = 2, 8
    MIN_RATIO, MAX_RATIO = 0.25, 1.0

    possible_contours = []
    for d in contours_dict:
        area = d['w'] * d['h']
        ratio = d['w'] / d['h']
        if area > MIN_AREA and d['w'] > MIN_WIDTH and d['h'] > MIN_HEIGHT and MIN_RATIO < ratio < MAX_RATIO:
            d['idx'] = len(possible_contours)
            possible_contours.append(d)

    result_idx = find_chars(possible_contours)
    matched_result = [[possible_contours[idx] for idx in idx_list] for idx_list in result_idx]

    PLATE_WIDTH_PADDING = 1.3
    PLATE_HEIGHT_PADDING = 1.5
    MIN_PLATE_RATIO = 3
    MAX_PLATE_RATIO = 10
    
    plate_imgs = []
    plate_infos = []
    for matched_chars in matched_result:
        sorted_chars = sorted(matched_chars, key=lambda x: x['cx'])
        plate_cx = (sorted_chars[0]['cx'] + sorted_chars[-1]['cx']) / 2
        plate_cy = (sorted_chars[0]['cy'] + sorted_chars[-1]['cy']) / 2
        plate_width = (sorted_chars[-1]['x'] + sorted_chars[-1]['w'] - sorted_chars[0]['x']) * PLATE_WIDTH_PADDING
        sum_height = sum(d['h'] for d in sorted_chars)
        plate_height = int(sum_height / len(sorted_chars) * PLATE_HEIGHT_PADDING)
        triangle_height = sorted_chars[-1]['cy'] - sorted_chars[0]['cy']
        triangle_hypotenus = np.linalg.norm(
            np.array([sorted_chars[0]['cx'], sorted_chars[0]['cy']]) - 
            np.array([sorted_chars[-1]['cx'], sorted_chars[-1]['cy']])
        )
        angle = np.degrees(np.arcsin(triangle_height / triangle_hypotenus))
        rotation_matrix = cv2.getRotationMatrix2D(center=(plate_cx, plate_cy), angle=angle, scale=1.0)
        img_rotated = cv2.warpAffine(img_thresh, M=rotation_matrix, dsize=(width, height))
        img_cropped = cv2.getRectSubPix(
            img_rotated, 
            patchSize=(int(plate_width), int(plate_height)), 
            center=(int(plate_cx), int(plate_cy))
        )
        if img_cropped.shape[1] / img_cropped.shape[0] < MIN_PLATE_RATIO or img_cropped.shape[1] / img_cropped.shape[0] > MAX_PLATE_RATIO:
            continue
        plate_imgs.append(img_cropped)
        plate_infos.append({
            'x': int(plate_cx - plate_width / 2),
            'y': int(plate_cy - plate_height / 2),
            'w': int(plate_width),
            'h': int(plate_height)
        })
    plate_chars = []
    for i, plate_img in enumerate(plate_imgs):
        plate_img = cv2.resize(plate_img, dsize=(0, 0), fx=1.6, fy=1.6)
        _, plate_img = cv2.threshold(plate_img, thresh=0.0, maxval=255.0, type=cv2.THRESH_BINARY | cv2.THRESH_OTSU)

        contours, _ = cv2.findContours(plate_img, mode=cv2.RETR_LIST, method=cv2.CHAIN_APPROX_SIMPLE)

        plate_min_x, plate_min_y = plate_img.shape[1], plate_img.shape[0]
        plate_max_x, plate_max_y = 0, 0

        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)

            area = w * h
            ratio = w / h
            if area > MIN_AREA and w > MIN_WIDTH and h > MIN_HEIGHT and MIN_RATIO < ratio < MAX_RATIO:
                if x < plate_min_x:
                    plate_min_x = x
                if y < plate_min_y:
                    plate_min_y = y
                if x + w > plate_max_x:
                    plate_max_x = x + w
                if y + h > plate_max_y:
                    plate_max_y = y + h

        img_result = plate_img[plate_min_y:plate_max_y, plate_min_x:plate_max_x]

        img_result = cv2.GaussianBlur(img_result, ksize=(3, 3), sigmaX=0)
        _, img_result = cv2.threshold(img_result, thresh=0.0, maxval=255.0, type=cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        img_result = cv2.copyMakeBorder(img_result, top=10, bottom=10, left=10, right=10, borderType=cv2.BORDER_CONSTANT, value=(0,0,0))

        chars = pytesseract.image_to_string(img_result, lang='kor', config='--psm 8 --oem 3')

        result_chars = ''
        has_digit = False
        for c in chars:
            if 0xAC00 <= ord(c) <= 0xD7A3 or c.isdigit():
                if c.isdigit():
                    has_digit = True
                result_chars += c

    plate_chars.append(result_chars)
    if len(result_chars) > 0:
        match = re.match(r"(\d+)([\uAC00-\uD7A3]+)(\d+)", result_chars)
    if match:
        front_numbers = match.group(1)
        korean_chars = match.group(2)
        rear_numbers = match.group(3)
        if len(rear_numbers) == 5:
            rear_numbers = rear_numbers[:-1]
        if len(front_numbers) == 4:
            front_numbers = front_numbers[1:]
        corrected_plate_text = front_numbers + korean_chars + rear_numbers
        plate_chars[-1] = corrected_plate_text
    longest_idx, longest_text = -1, 0

    for i, text in enumerate(plate_chars):
        if len(text) > longest_text:
            longest_text = len(text)
            longest_idx = i
    return text



while True:
    dist1 = measure_distance(TRIG1, ECHO1)
    dist2 = measure_distance(TRIG2, ECHO2)

    try:
        
        if dist1 <= 10:
            lcd_string("PLEASE",LCD_LINE_1)
            lcd_string("WAIT",LCD_LINE_2)
            cam()
            tt=text()
            rr=ent(tt)
            if rr < 0:
                lcd_string("Not a resident", LCD_LINE_1)
            elif rr < 6 :
                lcd_string("park in B1", LCD_LINE_1)
                lcd_string(str(rr+1),LCD_LINE_2)
            elif rr < 12 :
                lcd_string("park in B2", LCD_LINE_1)
                lcd_string(str(rr-5),LCD_LINE_2)
            elif rr < 18 :
                lcd_string("park in B3", LCD_LINE_1)
                lcd_string(str(rr-11),LCD_LINE_2)
            
            time.sleep(2)
            lcd_init()

        elif dist2 <= 10:
            
            cam()
            ee=text()
            exit(ee)
            time.sleep(2)
            lcd_init()
            

    except:
        continue

