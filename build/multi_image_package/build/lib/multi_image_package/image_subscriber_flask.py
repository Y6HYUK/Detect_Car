import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
from ultralytics import YOLO
import numpy as np
from flask import Flask, Response
import threading

# 마우스로 설정된 좌표를 저장할 리스트
points = []

# 마우스 이벤트 콜백 함수 (필요시 유지)
def draw_polygon(event, x, y, flags, param):
    global points
    if event == cv2.EVENT_LBUTTONDOWN:
        points.append((x, y))
        print(f"Point {len(points)}: ({x}, {y})")

# 점이 다각형 내부에 있는지 확인하는 함수
def is_inside_polygon(point, polygon):
    poly_array = np.array(polygon, dtype=np.int32)
    result = cv2.pointPolygonTest(poly_array, point, False)
    return result >= 0

# Flask 앱 생성
app = Flask(__name__)

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.bridge = CvBridge()

        # 두 개의 이미지 토픽을 구독
        self.subscription_1 = self.create_subscription(
            Image,
            'camera_image_1',
            self.image_callback_1,
            10
        )
        self.subscription_2 = self.create_subscription(
            Image,
            'camera_image_2',
            self.image_callback_2,
            10
        )

        # YOLO 모델 로드
        try:
            self.model = YOLO('/home/yjh/Doosan/Real_project_ws/Week2/_best.pt')  # 올바른 모델 경로로 수정
            self.get_logger().info('YOLO 모델 로드 성공')
        except Exception as e:
            self.get_logger().error(f'YOLO 모델 로드 실패: {e}')
            self.model = None

        self.classNames = ['car']  # 클래스 이름
        self.latest_frame = None  # Flask로 전달할 최신 프레임

    def process_frame_with_yolo_and_polygon(self, frame):
        # YOLO 감지 및 다각형 영역 체크
        # (코드 내용 동일)
        alarm_triggered = False

        # 최신 프레임 업데이트
        self.latest_frame = frame

    def image_callback_1(self, msg):
        # 첫 번째 이미지 토픽에서 받은 메시지 처리
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process_frame_with_yolo_and_polygon(frame)

    def image_callback_2(self, msg):
        # 두 번째 이미지 토픽에서 받은 메시지 처리 (YOLO 미적용)
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.latest_frame = frame  # 최신 프레임을 업데이트

# 프레임을 Flask 스트림으로 전달하는 함수
def generate_frames():
    while True:
        if image_subscriber.latest_frame is not None:
            # OpenCV 이미지를 JPEG로 인코딩
            ret, buffer = cv2.imencode('.jpg', image_subscriber.latest_frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

# Flask 경로 설정
@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

def main(args=None):
    global image_subscriber
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()

    # Flask 서버를 별도의 스레드에서 실행
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
