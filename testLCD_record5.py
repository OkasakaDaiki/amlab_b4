#!/usr/bin/env python
import errno
import spl_lib as spl
from scipy.signal import lfilter
import numpy
import pyaudio
import wave
import numpy as np
import os, time
import RPi.GPIO as GPIO
import sys
from threading import Timer
import smbus2 as smbus
import subprocess
import concurrent.futures as cf

def get_path(base, tail, head=''):
    return os.path.join(base, tail) if head == '' else get_path(head, get_path(base, tail)[1:])

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
HTML_PATH = get_path(BASE_DIR, 'html/main.html', 'file:///')
SINGLE_DECIBEL_FILE_PATH = get_path(BASE_DIR, 'decibel_data/single_decibel.txt')
MAX_DECIBEL_FILE_PATH = get_path(BASE_DIR, 'decibel_data/max_decibel.txt')

#'''
#Listen to mic
#'''

def is_meaningful(old, new):
    return abs(old - new) > 3

def update_text(path, content):
    try:
        f = open(path, 'w')
    except IOError as e:
        print(e)
    else:
        f.write(content)
        f.close()

def click(id):
    driver.find_element_by_id(id).click()

def open_html(path):
    driver.get(path)

def update_max_if_new_is_larger_than_max(new, max):
    print("update_max_if_new_is_larger_than_max called")
    if new > max:
        print("max observed")
        update_text(MAX_DECIBEL_FILE_PATH, 'MAX: {:.2f} dBA'.format(new))
        click('update_max_decibel')
        return new
    else:
        return max

shutdownPin = 6;
resetPin = 5; 

i2c = smbus.SMBus(1) # 1 is bus number
addr02=0x3e #lcd
_command=0x00
_data=0x40
_clear=0x01
_home=0x02
display_On=0x0f
LCD_2ndline=0x40+0x80

time.sleep(1)

#LCD AQM0802/1602
def command( code ):
    i2c.write_byte_data(addr02, _command, code)
    #time.sleep(0.1)

def writeLCD( message ):
    mojilist=[]
    for moji in message:
        mojilist.append(ord(moji))
    i2c.write_i2c_block_data(addr02, _data, mojilist) # 
    #time.sleep(0.1)
    
def shutdown(channel):
    os.system("sudo shutdown -h now")
      
def reboot(channel):
    os.system("sudo reboot")
      
def init():
    command(0x38) # 2 line disp mode, 8-bit trans mode, font size is normal 
    command(0x39) # to use extension function
    command(0x14) # configure clock signal 
    command(0x73) # configure contrast
    command(0x56) # Power/ICON/Contrast control 
    command(0x6c) # Follower control
    command(0x38) # 2 line disp mode, 8-bit trans mode, font size is normal 
    command(_clear) # clear display
    command(display_On) # cursor ON, display ON, blink ON
    command(0x0c) # display ON, else OFF 
    #usleep(39)

init ()
command(_clear)
writeLCD("RMS: ")

fs = 16000
channel = 1
counter = 0
size = 2**13
NUMERATOR, DENOMINATOR = spl.A_weighting(fs)
audio = pyaudio.PyAudio()

stream = audio.open(format = pyaudio.paInt16,
                    channels = int(channel),
                    rate = int(fs),
                    input = True,
                    frames_per_buffer = size)

def listen(old=0, error_count=0, min_decibel=100, max_decibel=0):
        try:
            ## read() returns string. You need to decode it into an array later.
            block = stream.read(size, exception_on_overflow = False)
        except IOError as e:
            error_count += 1
            print(" (%d) Error type is: %s" % (error_count, e))
        else:
            ## Int16 is a numpy data type which is Integer (-32768 to 32767)
            ## If you put Int8 or Int32, the result numbers will be ridiculous
            decoded_block = numpy.fromstring(block, 'Int16')
            ## This is where you apply A-weighted filter
            y = lfilter(NUMERATOR, DENOMINATOR, decoded_block)
            new_decibel = 20*numpy.log10(spl.rms_flat(y))
            if True:
                print('A-weighted: {:+.2f} dB'.format(new_decibel))
                return round(new_decibel, 1)

if __name__ == '__main__':
    executor = cf.ThreadPoolExecutor(max_workers = 4)

    data = []
    buf = []

    #stream.start_stream()
    print("Recording.")
    time.sleep(1)
    
    GPIO.setmode(GPIO.BCM)
    
    GPIO.setup(resetPin, GPIO.IN, pull_up_down = GPIO.PUD_UP)
    GPIO.setup(shutdownPin, GPIO.IN, pull_up_down = GPIO.PUD_UP)

    GPIO.add_event_detect(resetPin, GPIO.FALLING, callback = reboot, bouncetime = 2000)
    GPIO.add_event_detect(shutdownPin, GPIO.FALLING, callback = shutdown, bouncetime = 2000)

    while 1:
        executor.submit(command(LCD_2ndline)) # display at 2nd line of display
        #res = subprocess.check_output(['vcgencmd','measure_temp'])
        executor.submit(writeLCD(str(listen())+'dBA')) # display rms level on small display
        if (GPIO.event_detected(resetPin)):
            break
        if (GPIO.event_detected(shutdownPin)):
            break
        time.sleep(1);

GPIO.cleanup()
