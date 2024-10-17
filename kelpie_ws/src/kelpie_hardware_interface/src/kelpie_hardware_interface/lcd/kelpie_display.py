#!/usr/bin/python
# -*- coding: UTF-8 -*-

import os
import socket
import time
from PIL import Image,ImageDraw,ImageFont

from kelpie_hardware_interface.lcd.LCD_1inch47 import LCD_1inch47
class KelpieDisplay:
    def __init__(self):
        self.display = LCD_1inch47()
        # Initialize library.
        self.display.Init()
        # Clear display.
        self.display.clear()
        # Set the backlight to 100
        self.display.bl_DutyCycle(50)
        self.batt_min_v = 7
        self.batt_max_v = 8.4
        self._battery_v = 8.4

    def _draw(self, battery_percentage):
        ssid = os.popen("iwgetid -r").read().strip()
        hostname = socket.gethostname()
        ip_address = socket.gethostbyname(hostname)

        # Create blank image for drawing.
        image1 = Image.new("RGB", (self.display.height, self.display.width), "black")
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
        # black = Image.new("RGB", (320, 172), "black")

        print(os.getcwd() + "\n")
        curr_path = __file__.replace("/src/kelpie_hardware_interface/lcd/kelpie_display.py","/lib/")
        batt_status = Image.open(curr_path+'emptybatterystatus_white.png')

        batt_draw = ImageDraw.Draw(batt_status)

        if battery_percentage <= 0.20:
            batt_fill = "RED"
        elif 0.20 < battery_percentage <= 0.60:
            batt_fill = "#d49b00"  # yellow
        else:
            batt_fill = "#09ab00"  # green
        try:
            batt_draw.rounded_rectangle([(42, 92), (42 + (153 * battery_percentage), 170)], 8, fill=batt_fill)
            batt_draw.text((68, 95), str(int(battery_percentage * 100)) + "%", fill="WHITE", font=Font1_large)
            batt_scale_factor = 0.8
            resized_batt_status = batt_status.resize(
                (int(batt_status.size[0] * batt_scale_factor), int(batt_status.size[1] * batt_scale_factor)))
            image1.paste(resized_batt_status, (62, -40), resized_batt_status.convert('RGBA'))

            image1 = image1.rotate(0)
            image1 = image1.transpose(Image.ROTATE_270)
            self.display.ShowImage(image1)
        except:
            print("Batt display error")

    @property
    def battery_v(self):
        return self._battery_v

    @battery_v.setter
    def battery_v(self, value):
        self._battery_v = value

        battery_level = min(100, max(0, (value - self.batt_min_v)/(self.batt_max_v - self.batt_min_v)))
        self._draw(battery_level)

