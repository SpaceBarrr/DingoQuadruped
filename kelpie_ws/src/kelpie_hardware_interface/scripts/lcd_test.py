#!/usr/bin/python
# -*- coding: UTF-8 -*-
#import chardet
import os
import sys 
import socket
import time
import logging
from PIL import Image,ImageDraw,ImageFont

sys.path.insert(0,
                os.path.dirname(os.path.realpath(__file__)).replace('scripts', 'src') + \
                "/kelpie_hardware_interface/lcd")
import LCD_1inch47

# Raspberry Pi pin configuration:
# RST = 27
# DC = 25
# BL = 18
# bus = 0
# device = 0
ssid = os.popen("iwgetid -r").read().strip()
hostname = socket.gethostname()
ip_address = socket.gethostbyname(hostname)
battery_percentage = 0.7

logging.basicConfig(level=logging.DEBUG)

try:
    # display with hardware SPI:
    ''' Warning!!!Don't  creation of multiple displayer objects!!! '''
    #disp = LCD_1inch47.LCD_1inch47(spi=SPI.SpiDev(bus, device),spi_freq=10000000,rst=RST,dc=DC,bl=BL)
    disp = LCD_1inch47.LCD_1inch47()
    # Initialize library.
    disp.Init()
    # Clear display.
    disp.clear()
    #Set the backlight to 100
    disp.bl_DutyCycle(50)

    # Create blank image for drawing.
    image1 = Image.new("RGB", (disp.height, disp.width), "black")
    draw = ImageDraw.Draw(image1)

    # rospy.loginfo("Importing fonts...")
    Font1 = ImageFont.truetype("/usr/share/fonts/truetype/Font02.ttf", 25)
    Font1_small = ImageFont.truetype("/usr/share/fonts/truetype/Font02.ttf", 20)
    Font1_large = ImageFont.truetype("/usr/share/fonts/truetype/Font02.ttf", 60)
    Font2 = ImageFont.truetype("/usr/share/fonts/truetype/Font01.ttf", 35)
    Font3 = ImageFont.truetype("/usr/share/fonts/truetype/Font02.ttf", 120)

    draw.text((20, 110), 'SSID: ' + ssid, fill="WHITE", font=Font1)
    draw.text((20, 135), 'IP: ' + ip_address, fill="WHITE", font=Font1)
    current_time = time.strftime("%I:%M:%S%p")
    draw.text((220, 0), current_time, fill="WHITE", font=Font1_small)

    ## Battery indication bar
    black = Image.new("RGB", (320, 172), "black")

    print(os.getcwd() + "\n")
    batt_status = Image.open('../lib/emptybatterystatus_white.png')

    batt_draw = ImageDraw.Draw(batt_status)

    if battery_percentage <= 0.20:
        batt_fill = "RED"
    elif 0.20 < battery_percentage <= 0.60:
        batt_fill = "#d49b00"  # yellow
    else:
        batt_fill = "#09ab00"  # green

    batt_draw.rounded_rectangle([(42, 92), (42 + (153 * battery_percentage), 170)], 8, fill=batt_fill)
    batt_draw.text((68, 95), str(int(battery_percentage * 100)) + "%", fill="WHITE", font=Font1_large)
    batt_scale_factor = 0.8
    resized_batt_status = batt_status.resize(
        (int(batt_status.size[0] * batt_scale_factor), int(batt_status.size[1] * batt_scale_factor)))
    image1.paste(resized_batt_status, (62, -40), resized_batt_status.convert('RGBA'))

    image1 = image1.rotate(0)
    image1 = image1.transpose(Image.ROTATE_270)
    disp.ShowImage(image1)

    time.sleep(3)
    disp.module_exit()

except IOError as e:
    logging.info(e)    

except KeyboardInterrupt:
    disp.module_exit()
    logging.info("quit:")
    exit()

# draw battery voltage
# network it's connected to
# ip address (optional)