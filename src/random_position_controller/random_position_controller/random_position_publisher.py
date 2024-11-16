import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from math import atan2, sin, cos

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # 초기 위치 → 2번째 위치 → 웨이포인트 이동 단계 설정
        self.stage = "to_second_position"  # 현재 이동 단계

        # 초기 위치와 2번째 위치
        self.second_position = {"x": 0.21809444374079054, "y": 0.10399797519326827, "z": 0.0,
                                "orientation": {"x": 0.0, "y": 0.0, "z": 0.13264224779805608, "w": 0.9911639794196917}}

        # 2번째 위치 → 최종 위치 웨이포인트 설정
        self.start_position = {"x": 0.21809444374079054, "y": 0.10399797519326827}
        self.final_position = {"x": -0.6843737166483828, "y": 1.779173227959638}
        self.num_waypoints = 5
        self.waypoints = self.generate_waypoints(self.start_position, self.final_position, self.num_waypoints)
        self.current_waypoint_index = 0

        # 주기적으로 목표 퍼블리시
        self.timer = self.create_timer(4.0, self.publish_goal)  # 주기 조정 가능
        self.second_position_wait_time = 10  # 2번째 위치에서 대기 시간 (초)
        self.elapsed_time = 0  # 경과 시간

    def generate_waypoints(self, start, end, num_points):
        """시작 위치와 끝 위치 사이에 웨이포인트를 생성"""
        waypoints = []
        for i in range(1, num_points + 1):
            fraction = i / num_points
            waypoint_x = start["x"] + fraction * (end["x"] - start["x"])
            waypoint_y = start["y"] + fraction * (end["y"] - start["y"])
            waypoints.append({"x": waypoint_x, "y": waypoint_y})
        return waypoints

    def publish_goal(self):
        """현재 단계에 따라 목표 퍼블리시"""
        if self.stage == "to_second_position":
            # 2번째 위치로 이동
            if self.elapsed_time >= self.second_position_wait_time:
                self.get_logger().info("Finished waiting at second position. Switching to waypoints.")
                self.stage = "to_waypoints"  # 다음 단계로 변경
            else:
                self.publish_position(self.second_position)
                self.elapsed_time += 4  # 타이머 간격(4초)을 누적
        elif self.stage == "to_waypoints":
            # 웨이포인트 퍼블리시
            if self.current_waypoint_index < len(self.waypoints):
                waypoint = self.waypoints[self.current_waypoint_index]
                self.publish_waypoint(waypoint)
                self.current_waypoint_index += 1
            else:
                self.get_logger().info("All waypoints published!")
                self.stage = "done"

    def publish_position(self, position):
        """특정 위치로 이동"""
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = self.get_clock().now().to_msg()

        # 위치 설정
        goal_msg.pose.position.x = position["x"]
        goal_msg.pose.position.y = position["y"]
        goal_msg.pose.position.z = position["z"]

        # 방향 설정
        goal_msg.pose.orientation.x = position["orientation"]["x"]
        goal_msg.pose.orientation.y = position["orientation"]["y"]
        goal_msg.pose.orientation.z = position["orientation"]["z"]
        goal_msg.pose.orientation.w = position["orientation"]["w"]

        self.publisher_.publish(goal_msg)
        self.get_logger().info(f"Published goal to second position: x={position['x']}, y={position['y']}")

    def publish_waypoint(self, waypoint):
        """웨이포인트로 이동"""
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = self.get_clock().now().to_msg()

        # 위치 설정
        goal_msg.pose.position.x = waypoint["x"]
        goal_msg.pose.position.y = waypoint["y"]
        goal_msg.pose.position.z = 0.0

        # 방향 계산
        orientation = self.calculate_orientation(self.start_position, waypoint)
        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = orientation["z"]
        goal_msg.pose.orientation.w = orientation["w"]

        self.publisher_.publish(goal_msg)
        self.get_logger().info(f"Published waypoint: x={waypoint['x']}, y={waypoint['y']}")

    def calculate_orientation(self, current, next_wp):
        """현재 위치와 다음 웨이포인트를 기반으로 방향 계산"""
        dx = next_wp["x"] - current["x"]
        dy = next_wp["y"] - current["y"]
        angle = atan2(dy, dx)
        return {
            "z": sin(angle / 2),
            "w": cos(angle / 2)
        }

def main(args=None):
    rclpy.init(args=args)
    node = WaypointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Waypoint Publisher shutting down.")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
