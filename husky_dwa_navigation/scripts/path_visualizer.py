#!/usr/bin/env python3

import rospy
import json
import math
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix
import tf2_ros
import tf2_geometry_msgs

class PathVisualizer:
    """경로 시각화 전용 클래스"""

    def __init__(self):
        rospy.init_node('path_visualizer', anonymous=True)

        # 궤적 기록 (map 좌표계 기준)
        self.fasterlio_trajectory = []
        self.gps_trajectory = []
        self.corrected_trajectory = []
        self.latest_waypoints = None

        # 현재 위치
        self.current_pose = None

        # TF listener (EKF의 map → odom TF 사용)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # 시각화 Publishers
        self.fasterlio_path_pub = rospy.Publisher("/visualization/fasterlio_path", Marker, queue_size=10)
        self.gps_path_pub = rospy.Publisher("/visualization/gps_path", Marker, queue_size=10)
        self.corrected_path_pub = rospy.Publisher("/visualization/corrected_path", Marker, queue_size=10)
        self.waypoints_pub = rospy.Publisher("/visualization/waypoints", MarkerArray, queue_size=10)

        # Subscribers
        rospy.Subscriber("/Odometry", Odometry, self.fasterlio_callback)
        rospy.Subscriber("/ublox/fix", NavSatFix, self.gps_callback)
        rospy.Subscriber("/waypoints", String, self.waypoints_callback)

        # Timers
        rospy.Timer(rospy.Duration(0.5), self.publish_visualization)

        rospy.loginfo("🎨 경로 시각화기 시작!")
        rospy.loginfo("📊 FasterLIO, GPS, EKF 융합 경로 및 웨이포인트 시각화")
        rospy.loginfo("🔗 EKF의 map → odom TF 사용")

    def distance_check(self, pose1, pose2, threshold):
        """거리 체크"""
        if "x" not in pose1 or "x" not in pose2:
            return True
        dx = pose1["x"] - pose2["x"]
        dy = pose1["y"] - pose2["y"]
        return math.sqrt(dx*dx + dy*dy) > threshold

    def fasterlio_callback(self, msg):
        """FasterLIO 궤적 기록 (원본 좌표)"""
        point = {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "z": msg.pose.pose.position.z,
            "timestamp": msg.header.stamp.to_sec()
        }

        # 0.5m 이상 이동 시에만 기록
        if not self.fasterlio_trajectory or self.distance_check(point, self.fasterlio_trajectory[-1], 0.5):
            self.fasterlio_trajectory.append(point)
            rospy.loginfo_throttle(5, f"📍 FasterLIO 궤적: {len(self.fasterlio_trajectory)}개 포인트")

    def gps_callback(self, msg):
        """GPS 궤적 기록 (단순 기록용)"""
        if msg.status.status >= 0:
            point = {
                "lat": msg.latitude,
                "lon": msg.longitude,
                "timestamp": msg.header.stamp.to_sec()
            }

            if not self.gps_trajectory or len(self.gps_trajectory) == 0:
                self.gps_trajectory.append(point)
                rospy.loginfo_throttle(5, f"📡 GPS 데이터 기록: {len(self.gps_trajectory)}개 포인트")

    def get_current_pose_from_tf(self):
        """TF를 사용해서 현재 위치 가져오기 (EKF의 map → odom TF 사용)"""
        try:
            # map 프레임에서 base_link의 위치 가져오기
            transform = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time())

            self.current_pose = {
                "x": transform.transform.translation.x,
                "y": transform.transform.translation.y,
                "z": transform.transform.translation.z,
                "timestamp": rospy.Time.now().to_sec()
            }

            # 보정된 궤적 기록
            if not self.corrected_trajectory or self.distance_check(self.current_pose, self.corrected_trajectory[-1], 0.5):
                self.corrected_trajectory.append(self.current_pose.copy())

            return True
        except Exception as e:
            rospy.logwarn_throttle(10, f"TF 조회 실패: {e}")
            return False

    def waypoints_callback(self, msg):
        """Waypoints 수신"""
        try:
            data = json.loads(msg.data)
            if "waypoints" in data:
                self.latest_waypoints = data["waypoints"]
                rospy.loginfo(f"📥 {len(self.latest_waypoints)}개 waypoints 수신")
                self.visualize_waypoints()
        except Exception as e:
            rospy.logerr(f"❌ Waypoints 파싱 오류: {e}")

    def publish_visualization(self, event):
        """모든 시각화 발행"""
        # TF에서 현재 위치 가져오기
        self.get_current_pose_from_tf()

        self.visualize_fasterlio_path()
        self.visualize_gps_path()
        self.visualize_corrected_path()
        self.visualize_waypoints()

    def visualize_fasterlio_path(self):
        """FasterLIO 원본 경로 (회색)"""
        if len(self.fasterlio_trajectory) < 2:
            return

        marker = self.create_path_marker(
            self.fasterlio_trajectory, "fasterlio_path", 0,
            (0.5, 0.5, 0.5), 2.0, "base_link"
        )
        self.fasterlio_path_pub.publish(marker)

    def visualize_gps_path(self):
        """GPS 경로 (파란색) - 간단한 표시용"""
        if len(self.gps_trajectory) < 1:
            return

        # GPS는 단순히 포인트로만 표시
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "gps_points"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 2.0
        marker.scale.y = 2.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0

        # GPS 포인트들 추가 (위경도 그대로 표시)
        for gps_point in self.gps_trajectory:
            point = Point()
            point.x = gps_point["lat"] * 100000  # 시각화용 스케일링
            point.y = gps_point["lon"] * 100000
            point.z = 0
            marker.points.append(point)

        self.gps_path_pub.publish(marker)

    def visualize_corrected_path(self):
        """보정된 경로 (빨간색)"""
        if len(self.corrected_trajectory) < 2:
            return

        marker = self.create_path_marker(
            self.corrected_trajectory, "corrected_path", 0,
            (1.0, 0.0, 0.0), 3.0, "map"
        )
        self.corrected_path_pub.publish(marker)


    def visualize_waypoints(self):
        """웨이포인트 시각화"""
        marker_array = MarkerArray()

        # 기존 마커 삭제
        delete_marker = Marker()
        delete_marker.header.frame_id = "map"
        delete_marker.header.stamp = rospy.Time.now()
        delete_marker.ns = "waypoints"
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        if not self.latest_waypoints:
            self.waypoints_pub.publish(marker_array)
            return

        # 웨이포인트 마커들 생성
        for i, wp in enumerate(self.latest_waypoints):
            if "x" in wp and "y" in wp:
                # UTM 좌표 웨이포인트
                cube = Marker()
                cube.header.frame_id = "map"
                cube.header.stamp = rospy.Time.now()
                cube.ns = "waypoints"
                cube.id = i
                cube.type = Marker.CUBE
                cube.action = Marker.ADD
                cube.pose.position.x = float(wp["x"])
                cube.pose.position.y = float(wp["y"])
                cube.pose.position.z = 2.0
                cube.pose.orientation.w = 1.0
                cube.scale.x = 4.0
                cube.scale.y = 4.0
                cube.scale.z = 2.5

                # 색상 구분
                if i == 0:
                    cube.color.r, cube.color.g, cube.color.b = 0.0, 1.0, 0.0  # 시작점 - 녹색
                elif i == len(self.latest_waypoints) - 1:
                    cube.color.r, cube.color.g, cube.color.b = 1.0, 0.0, 0.0  # 끝점 - 빨간색
                else:
                    cube.color.r, cube.color.g, cube.color.b = 1.0, 1.0, 0.0  # 중간점 - 노란색

                cube.color.a = 0.9
                marker_array.markers.append(cube)

        self.waypoints_pub.publish(marker_array)

    def create_path_marker(self, trajectory, namespace, marker_id, color, line_width, frame_id):
        """경로 마커 생성"""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = line_width
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0

        points = []
        for pt in trajectory:
            if "x" in pt and "y" in pt:
                points.append(Point(x=pt["x"], y=pt["y"], z=pt.get("z", 0)))

        marker.points = points
        return marker

if __name__ == '__main__':
    try:
        visualizer = PathVisualizer()
        rospy.loginfo("🎉 경로 시각화기 실행 중...")
        rospy.loginfo("🎨 시각화 토픽들:")
        rospy.loginfo("  - /visualization/fasterlio_path (회색)")
        rospy.loginfo("  - /visualization/corrected_path (빨간색)")
        rospy.loginfo("  - /visualization/waypoints")
        rospy.loginfo("🔗 EKF map → odom TF 사용")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 시스템 종료")
    except Exception as e:
        rospy.logerr(f"❌ 시스템 오류: {e}")