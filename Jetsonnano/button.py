import Jetson.GPIO as GPIO
import time
import subprocess
import os

button_pin = 17
led_pin = 23  # กำหนดหมายเลขของขาที่ LED ต่อ

GPIO.setmode(GPIO.BCM)
GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(led_pin, GPIO.OUT)  # กำหนดขาของ LED เป็นเอาต์พุต

def button_callback(channel):
    print("Button was pushed!")
    os.environ['DISPLAY'] = ':0'
    
    roscore_result = subprocess.run(['pgrep', '-f', 'roscore'], stdout=subprocess.PIPE)
    if roscore_result.stdout:
        print("roscore is already running.")
    else:
        subprocess.Popen(["gnome-terminal", "--", "bash", "-c", "source /opt/ros/melodic/setup.bash; roscore; exec bash"])
        time.sleep(7)
    
    roslaunch_result = subprocess.run(['pgrep', '-f', 'turtlebot3_navigation.launch'], stdout=subprocess.PIPE)
    if roslaunch_result.stdout:
        print("turtlebot3_navigation.launch is already running.")
    else:
        subprocess.Popen(["gnome-terminal", "--", "bash", "-c", "source /home/sakson/catkin_ws/devel/setup.bash;export TURTLEBOT3_MODEL=burger; roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/home/sakson/Desktop/fix1.yaml; exec bash"])
        time.sleep(3)    

    clear_result = subprocess.run(['pgrep', '-f', 'rosrun clear clear'], stdout=subprocess.PIPE)
    if clear_result.stdout:
        print("rosrun clear clear is already running.")
    else:
        time.sleep(3)
        subprocess.Popen(["gnome-terminal", "--", "bash", "-c", "source /home/sakson/catkin_ws/devel/setup.bash; rosrun clear clear; exec bash"])
        
        # เปิด LED เมื่อ Costmaps ได้รับการล้างเสร็จสมบูรณ์ และไม่ต้องดับ
        GPIO.output(led_pin, GPIO.LOW)

# เพิ่มเงื่อนไขเพื่อปิด LED เมื่อเริ่มต้นโปรแกรม
GPIO.output(led_pin, GPIO.HIGH)

GPIO.add_event_detect(button_pin, GPIO.FALLING, callback=button_callback, bouncetime=500)

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("Program exited.")
finally:
    GPIO.cleanup()

