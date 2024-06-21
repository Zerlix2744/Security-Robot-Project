import Jetson.GPIO as GPIO
import time
import subprocess
import os

button_pin = 27

GPIO.setmode(GPIO.BCM)
GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # ตั้งค่าให้ใช้ pull-up resistor

def button_callback(channel):
    print("Button was pushed!")
    os.environ['DISPLAY'] = ':0'  # เซ็ต DISPLAY environment variable

    # ตรวจสอบว่ากระบวนการ rosrun goalline goalline กำลังรันอยู่หรือไม่
    clear_result = subprocess.run(['pgrep', '-f', 'rosrun goalline goalline'], stdout=subprocess.PIPE)
    if clear_result.stdout:
        print("rosrun goalline goalline is already running.")
    else:
        # หน่วงเวลา 5 วินาที
        time.sleep(5)

        # รันคำสั่ง rosrun
        subprocess.Popen(["gnome-terminal", "--", "bash", "-c", "source /home/sakson/catkin_ws/devel/setup.bash; rosrun goalline goalline; exec bash"])

	 # ตรวจสอบว่ากระบวนการ live.py กำลังรันอยู่หรือไม่
    live_py_result = subprocess.run(['pgrep', '-f', 'live.py'], stdout=subprocess.PIPE)
    if live_py_result.stdout:
        print("live.py is already running.")
    else:
        # รันไฟล์ live.py บนเดสก์ท็อป
        subprocess.Popen(["gnome-terminal", "--", "bash", "-c", "cd /home/sakson/Desktop; python3 live.py; exec bash"])

    # ตรวจสอบว่ากระบวนการ ngrok กำลังรันอยู่หรือไม่
    ngrok_result = subprocess.run(['pgrep', '-f', 'ngrok'], stdout=subprocess.PIPE)
    if ngrok_result.stdout:
        print("ngrok is already running.")
    else:
        # รันคำสั่ง ngrok
        subprocess.Popen(["gnome-terminal", "--", "bash", "-c", "ngrok http --domain=happy-happily-mullet.ngrok-free.app 5000; exec bash"])

GPIO.add_event_detect(button_pin, GPIO.FALLING, callback=button_callback, bouncetime=500)

try:
    # รอการกดปุ่ม
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("Program exited.")
finally:
    GPIO.cleanup()

