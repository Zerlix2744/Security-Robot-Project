import Jetson.GPIO as GPIO
import time
import subprocess
import os

button_pin = 17

GPIO.setmode(GPIO.BCM)
GPIO.setup(button_pin, GPIO.IN)

def button_callback(channel):
    print("Button was pushed!")
    os.environ['DISPLAY'] = ':0'  # เซ็ต DISPLAY environment variable
    # Source ไฟล์ setup ของ ROS Melodic ก่อนที่จะรัน roscore
    subprocess.Popen(["gnome-terminal", "--", "bash", "-c", "source /opt/ros/melodic/setup.bash; roscore; exec bash"])

GPIO.add_event_detect(button_pin, GPIO.FALLING, callback=button_callback, bouncetime=500)

try:
    # รอการกดปุ่ม
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("Program exited.")
finally:
    GPIO.cleanup()

