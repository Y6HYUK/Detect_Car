import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
# Point: target_coordinates 토픽에서 수신하는 좌표 정보에 사용되는 메시지 타입.
# PoseStamped: 이동 목표 지점(goal)을 설정할 때 사용하는 메시지 타입.
from nav2_msgs.action import NavigateToPose # NavigateToPose: 네비게이션 액션 타입으로, AMR이 특정 위치로 이동할 수 있도록 함
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile

class AMRNavigator(Node):
    def __init__(self):
        super().__init__('amr_navigator')
        
        # QoS 설정 (토픽 구독에 사용)
        qos_profile = QoSProfile(depth=10)
        '''
        depth=10으로 QoS 설정을 지정하여 최근의 10개의 메시지를 보관함
        이는 네트워크 상태나 메시지 수신 지연 문제로 인해 메시지를 놓치는 것을 방지하기 위함
        '''
        
        # target_coordinates 토픽 구독
        self.subscription = self.create_subscription(
            Point, 'target_coordinates', self.target_callback, qos_profile
        )
        
        # 네비게이션 액션 클라이언트 생성
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        '''
        액션 클라이언트 생성:
        navigate_to_pose라는 이름의 네비게이션 액션 클라이언트를 생성
        이 액션 클라이언트를 통해 목표 지점으로 이동하는 명령을 네비게이션 스택에 전달 가능
        '''
        
        # 초기화 메시지
        self.get_logger().info('AMR Navigator Node Initialized. Waiting for target coordinates...')

        # 현재 목표 수락 상태
        self.current_goal_accepted = False  
        # self.current_goal_accepted: 현재 목표가 수락되었는지 여부를 추적하여 중복 요청을 방지합니다.
        

    def target_callback(self, msg):
        """
        target_coordinates 토픽에서 목표 좌표를 수신하여 네비게이션 액션을 실행합니다.
        """
        if not self.current_goal_accepted:  # 이전 목표가 진행 중이면 무시
            self.get_logger().info(f"Received target coordinates: x={msg.x}, y={msg.y}")
            self.navigate_to_target(msg.x, msg.y)

    def navigate_to_target(self, x, y):
        """
        목표 좌표로 이동 명령을 네비게이션 스택에 전송합니다.
        """
        # 목표 메시지 생성
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'  # 맵 기준 좌표
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0  # 방향 (정면) orientation.w = 1.0으로 설정하여 목표 지점의 방향을 기본값으로 설정

        # 액션 서버 연결 대기
        self.nav_client.wait_for_server() # self.nav_client.wait_for_server()를 사용하여 액션 서버에 연결될 때까지 대기

        # 목표 전송
        self.get_logger().info('Sending goal to navigation server...')
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        네비게이션 액션 서버로부터 목표 수락 여부를 확인합니다.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Goal rejected by navigation server.')
            self.current_goal_accepted = False
            return

        self.get_logger().info('Goal accepted. Navigating to target...')
        self.current_goal_accepted = True

        '''
        goal_handle.accepted가 True인 경우 목표가 수락된 것으로 간주하고, 이동을 시작합니다.
        False인 경우 경고 메시지를 출력하고 다음 목표를 대기합니다.
        '''

        # 이동 결과 콜백
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        """
        네비게이션 액션의 결과를 처리합니다.
        """
        result = future.result()
        if result.status == 4:  # result.status == 4는 목표 지점에 성공적으로 도착했음을 의미
            self.get_logger().info('Successfully reached the target.')
        else:
            self.get_logger().warning('Failed to reach the target.')

        self.current_goal_accepted = False  # 다음 목표를 받을 준비 완료

def main(args=None):
    rclpy.init(args=args)
    node = AMRNavigator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down AMR Navigator Node.')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

'''
설명

토픽 구독:
target_coordinates 토픽을 구독하여 좌표를 수신합니다.

네비게이션 액션 클라이언트:
nav2_msgs.action.NavigateToPose 액션 클라이언트를 생성하여 네비게이션 스택에 목표 좌표를 전달하고 이동을 실행합니다.

목표 좌표 설정 및 전송:
navigate_to_target 메서드에서 PoseStamped 메시지를 사용하여 목표 좌표를 설정하고 네비게이션 액션 서버에 목표 지점을 전송합니다.

응답 및 결과 콜백:
goal_response_callback: 목표 좌표가 수락되었는지 확인.
navigation_result_callback: AMR이 목표 지점에 성공적으로 도착했는지 확인.
'''

'''
프로세스 보완 및 주요 포인트

Initial Pose 설정:
이 코드는 initial pose를 직접 설정하지 않습니다. 대신, Rviz에서 2D Pose Estimate 도구를 사용하여 초기 위치를 설정해야 합니다.
초기 위치는 네비게이션 스택이 AMR의 위치를 추적하는 데 필수적입니다.

목표 좌표 수신 및 이동:
target_coordinates 토픽에서 목표 좌표를 수신하면 네비게이션 스택의 NavigateToPose 액션을 호출하여 AMR이 해당 지점으로 이동합니다.

장애물 회피 및 경로 재설정:
네비게이션 스택의 경로 생성 알고리즘은 자동으로 장애물을 감지하고 경로를 재설정합니다.
추가적인 코드는 필요하지 않으며, 네비게이션 스택의 내부 동작으로 처리됩니다.

결과 처리 및 상태 관리:
목표가 수락되었는지 확인 (goal_response_callback).
이동 결과를 처리 (navigation_result_callback).
이동이 완료되거나 실패하면 다음 목표를 받을 수 있도록 상태를 초기화.

Rviz에서 시각화:
Rviz를 사용해 AMR의 경로, 장애물, 현재 위치 등을 실시간으로 시각화할 수 있습니다.

명령:
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true

'''


'''
실행 순서
네비게이션 스택 실행:
네비게이션 스택과 맵을 로드합니다.
예시:
ros2 launch nav2_bringup bringup_launch.py map:=/path/to/your/map.yaml use_sim_time:=false


Rviz에서 Initial Pose 설정:
Rviz에서 2D Pose Estimate를 사용해 AMR의 초기 위치를 설정합니다.

AMR 구동 노드 실행:
python3 amr_navigation_node.py

도주 차량 좌표 발행:
기존 target_coordinates 토픽 발행 노드(bounding_box_publisher.py)를 실행하면, AMR이 해당 좌표로 이동합니다.

'''

'''
전체 로직 실행을 위한 명령어

1. ROS2 네비게이션 스택 실행
첫 번째 터미널:
ROS2 네비게이션 스택과 맵을 로드합니다:

ros2 launch nav2_bringup bringup_launch.py map:=/path/to/your/map.yaml use_sim_time:=false
(맵의 경로(/path/to/your/map.yaml)를 정확히 설정)

2. 이미지 퍼블리셔 노드 실행
두 번째 터미널:
ros2 run multi_image_package image_publisher_1

세 번째 터미널:
ros2 run multi_image_package image_publisher_2

3. 서브스크라이버 노드 실행
네 번째 터미널:
서브스크라이버 노드에서 camera_image_1과 camera_image_2를 구독하고, 바운딩 박스를 감지하여 target_coordinates 토픽으로 좌표를 발행
ros2 run multi_image_package image_subscriber_flask

4. AMR 구동 노드 실행
다섯 번째 터미널:
target_coordinates 토픽을 구독하여 AMR을 목표 좌표로 이동
ros2 run multi_image_package amr_operating'''