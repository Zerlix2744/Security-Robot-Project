import cv2
from flask import Flask, Response

app = Flask(__name__)

def gen_frames():
    camera = cv2.VideoCapture(0)  # เปิดกล้อง USB
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280) # ตั้งค่าความกว้างเป็น 1280 pixels
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720) # ตั้งค่าความสูงเป็น 720 pixels
    while True:
        success, frame = camera.read()
        if not success:
            break
        else:
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/')
def video_feed():
    return Response(gen_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)

