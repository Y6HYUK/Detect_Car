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

# Flask 애플리케이션 객체 생성
app = Flask(__name__)
points = []
image_subscriber = None  # ImageSubscriber 인스턴스를 전역 변수로 설정
status = "단속 전"  # 상태 초기값 설정

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
        self.latest_frame_1 = frame

    def image_callback_2(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.latest_frame_2 = frame

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
    # 상태에 따라 화면 구성과 타이틀 변경
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

    # 렌더링할 HTML 템플릿
    return render_template_string(f"""
        <html>
            <head>
                <style>
                    /* 비디오 컨테이너 중앙 정렬 */
                    .video-container {{
                        display: flex;
                        align-items: center;
                        justify-content: center;
                        margin-top: 20px;
                    }}

                    /* 단일 비디오일 때 중앙 정렬 */
                    .single-video {{
                        flex-direction: column;
                    }}

                    /* 두 개의 비디오일 때 좌우로 정렬 */
                    .double-video {{
                        flex-direction: row;
                    }}

                    .video {{
                        margin: 10px;
                        text-align: center;
                    }}

                    .button-container {{
                        text-align: center;
                        margin-top: 20px;
                    }}
                </style>
                <script>
                    function setStatus(newStatus) {{
                        fetch('/set_status', {{
                            method: 'POST',
                            headers: {{
                                'Content-Type': 'application/x-www-form-urlencoded',
                            }},
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

    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
