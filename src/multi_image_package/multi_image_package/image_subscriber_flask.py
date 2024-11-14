import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from flask import Flask, Response, render_template_string, request, jsonify
import threading
from ultralytics import YOLO
import time

# Flask 애플리케이션 객체 생성
app = Flask(__name__)
points = []  # 다각형 영역 좌표
image_subscriber = None
status = "단속 전"

# 다각형 내 포함 여부 확인 함수
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
        # YOLO 모델 로드
        try:
            self.model = YOLO('/home/yjh/Doosan/Real_project_ws/Week2/_best.pt')
            self.get_logger().info('YOLO 모델 로드 성공')
        except Exception as e:
            self.get_logger().error(f'YOLO 모델 로드 실패: {e}')
            self.model = None
        self.classNames = ['car']
        self.latest_frame_1 = None

    def image_callback_1(self, msg):
        global status
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # "단속 중" 상태에서만 YOLO 탐지를 적용
        if status == "단속 중":
            frame, alarm_triggered = self.process_frame_with_yolo_and_polygon(frame)
            if alarm_triggered:
                status = "도주차량 발생"  # 차량이 영역을 벗어났을 때 상태 전환
        else:
            frame = self.draw_polygon_and_points(frame)  # "단속 전" 상태에서는 폴리곤만 그림
        self.latest_frame_1 = frame

    def process_frame_with_yolo_and_polygon(self, frame):
        alarm_triggered = False
        if self.model is None:
            return frame, alarm_triggered
        
        # YOLO 모델을 사용한 객체 감지
        results = self.model(frame, stream=True)
        for r in results:
            for box in r.boxes:
                confidence = box.conf[0]
                if confidence >= 0.7:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    cls = int(box.cls[0])
                    if cls < len(self.classNames):
                        cv2.putText(frame, f"{self.classNames[cls]}: {confidence:.2f}", 
                                    (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    
                    # 바운딩 박스 중심 계산
                    center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2

                    # 객체가 다각형을 벗어났는지 확인
                    if len(points) == 4 and not is_inside_polygon((center_x, center_y), points):
                        alarm_triggered = True

        frame = self.draw_polygon_and_points(frame)  # 프레임에 다각형 및 포인트 표시
        return frame, alarm_triggered

    def draw_polygon_and_points(self, frame):
        for point in points:
            cv2.circle(frame, point, 5, (0, 255, 0), -1)
        if len(points) == 4:
            poly_array = np.array(points, np.int32)
            cv2.polylines(frame, [poly_array], isClosed=True, color=(255, 0, 0), thickness=2)
        return frame

def generate_frames(camera_id):
    while True:
        frame = image_subscriber.latest_frame_1
        if frame is not None:
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed_1')
def video_feed_1():
    return Response(generate_frames(1), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/add_point', methods=['POST'])
def add_point():
    global status, points
    if status == "단속 전":
        x, y = int(request.form['x']), int(request.form['y'])
        points.append((x, y))
        if len(points) == 4:
            status = "단속 중"  # 4개의 점이 설정되면 "단속 중" 상태로 전환
    return jsonify({"points": points, "status": status})

@app.route('/')
def index():
    global status
    title = status
    if status == "단속 전":
        video_content = f"""
            <div class="video">
                <h2>{title}</h2>
                <img src="{{{{ url_for('video_feed_1') }}}}" width="640" height="480" onclick="addPoint(event)">
            </div>
        """
    elif status == "단속 중":
        video_content = f"""
            <div class="video">
                <h2>{title}</h2>
                <img src="{{{{ url_for('video_feed_1') }}}}" width="640" height="480">
            </div>
        """
    elif status == "도주차량 발생":
        video_content = f"""
            <div class="double-video">
                <div class="video">
                    <h2>Camera 1</h2>
                    <img src="{{{{ url_for('video_feed_1') }}}}" width="640" height="480">
                </div>
            </div>
        """

    return render_template_string(f"""
        <html>
            <head>
                <style>
                    .video-container {{ display: flex; justify-content: center; margin-top: 20px; }}
                    .video {{ margin: 10px; text-align: center; }}
                </style>
                <script>
                    function addPoint(event) {{
                        const x = event.offsetX;
                        const y = event.offsetY;
                        fetch('/add_point', {{
                            method: 'POST',
                            headers: {{ 'Content-Type': 'application/x-www-form-urlencoded' }},
                            body: 'x=' + x + '&y=' + y
                        }}).then(response => response.json()).then(data => {{
                            if (data.status === "단속 중") {{
                                location.reload();  // 단속 중 상태로 전환되면 페이지 새로고침
                            }}
                        }});
                    }}
                </script>
            </head>
            <body>
                <h1>{title}</h1>
                <div class="video-container">{video_content}</div>
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
