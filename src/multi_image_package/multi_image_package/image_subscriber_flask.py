import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
from ultralytics import YOLO
import numpy as np
from flask import Flask, Response, render_template_string, request
import threading

app = Flask(__name__)
points = []
image_subscriber = None
status = "단속 전"

def draw_polygon(event, x, y, flags, param):
    global points, status
    if event == cv2.EVENT_LBUTTONDOWN:
        points.append((x, y))
        print(f"Point {len(points)}: ({x}, {y})")
        if len(points) >= 3 and status == "단속 전":
            status = "단속 중"  # 다각형이 설정되면 자동으로 "단속 중" 상태로 전환

def is_inside_polygon(point, polygon):
    poly_array = np.array(polygon, dtype=np.int32)
    result = cv2.pointPolygonTest(poly_array, point, False)
    return result >= 0

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.bridge = CvBridge()
        self.subscription_1 = self.create_subscription(
            Image, 'camera_image_1', self.image_callback_1, 10
        )
        self.subscription_2 = self.create_subscription(
            Image, 'camera_image_2', self.image_callback_2, 10
        )
        self.model = YOLO('/home/yjh/Doosan/Real_project_ws/Week2/_best.pt')
        self.classNames = ['car']
        self.latest_frame_1 = None
        self.latest_frame_2 = None

    def image_callback_1(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        global status
        if status == "단속 중":
            frame, alarm_triggered = self.process_frame_with_yolo_and_polygon(frame)
            if alarm_triggered:
                status = "도주차량 발생"  # 차량이 영역을 벗어나면 "도주차량 발생" 상태로 전환
        self.latest_frame_1 = frame

    def image_callback_2(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.latest_frame_2 = frame

    def process_frame_with_yolo_and_polygon(self, frame):
        alarm_triggered = False
        results = self.model(frame, stream=True)

        for r in results:
            for box in r.boxes:
                confidence = box.conf[0]
                if confidence >= 0.7:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    cls = int(box.cls[0])
                    if cls < len(self.classNames):
                        cv2.putText(frame, f"{self.classNames[cls]}: {confidence:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2

                    if len(points) >= 3:
                        if not is_inside_polygon((center_x, center_y), points):
                            alarm_triggered = True

        for point in points:
            cv2.circle(frame, point, 5, (0, 255, 0), -1)
        if len(points) >= 3:
            poly_array = np.array(points, np.int32)
            cv2.polylines(frame, [poly_array], isClosed=True, color=(255, 0, 0), thickness=2)
        
        return frame, alarm_triggered

def generate_frames(camera_id):
    while True:
        frame = image_subscriber.latest_frame_1 if camera_id == 1 else image_subscriber.latest_frame_2
        if frame is not None:
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed_1')
def video_feed_1():
    return Response(generate_frames(1), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/video_feed_2')
def video_feed_2():
    return Response(generate_frames(2), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/set_status', methods=['POST'])
def set_status():
    global status
    status = request.form.get('status')
    return "Status updated", 200

@app.route('/')
def index():
    global status
    if status == "단속 전":
        title = "단속 전"
        video_content = f"""
            <div class="video-container single-video">
                <div class="video">
                    <h2>{title}</h2>
                    <img src="{{{{ url_for('video_feed_1') }}}}" width="640" height="480">
                </div>
            </div>
        """
    elif status == "단속 중":
        title = "단속 중"
        video_content = f"""
            <div class="video-container single-video">
                <div class="video">
                    <h2>{title}</h2>
                    <img src="{{{{ url_for('video_feed_1') }}}}" width="640" height="480">
                </div>
            </div>
        """
    elif status == "도주차량 발생":
        title = "도주차량 발생"
        video_content = f"""
            <div class="video-container double-video">
                <div class="video">
                    <h2>Camera 1</h2>
                    <img src="{{{{ url_for('video_feed_1') }}}}" width="640" height="480">
                </div>
                <div class="video">
                    <h2>Camera 2</h2>
                    <img src="{{{{ url_for('video_feed_2') }}}}" width="640" height="480">
                </div>
            </div>
        """

    return render_template_string(f"""
        <html>
            <head>
                <style>
                    .video-container {{ display: flex; align-items: center; justify-content: center; margin-top: 20px; }}
                    .single-video {{ flex-direction: column; }}
                    .double-video {{ flex-direction: row; }}
                    .video {{ margin: 10px; text-align: center; }}
                    .button-container {{ text-align: center; margin-top: 20px; }}
                </style>
                <script>
                    function setStatus(newStatus) {{
                        fetch('/set_status', {{
                            method: 'POST',
                            headers: {{ 'Content-Type': 'application/x-www-form-urlencoded' }},
                            body: 'status=' + newStatus
                        }}).then(() => location.reload());
                    }}
                </script>
            </head>
            <body>
                <h1>{title}</h1>
                {video_content}
                <div class="button-container">
                    <button onclick="setStatus('단속 전')">단속 전</button>
                    <button onclick="setStatus('단속 중')">단속 중</button>
                    <button onclick="setStatus('도주차량 발생')">도주차량 발생</button>
                </div>
            </body>
        </html>
    """)

def main(args=None):
    global image_subscriber
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()

    flask_thread = threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5000))
    flask_thread.daemon = True
    flask_thread.start()

    cv2.namedWindow("Camera Image 1")
    cv2.setMouseCallback("Camera Image 1", draw_polygon)

    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
