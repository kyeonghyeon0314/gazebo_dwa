#!/usr/bin/env python3
"""
GPS → UTM 변환 노드 (navsat_transform 대체)
- NavSatFix (위도/경도) → Odometry (UTM 상대 좌표)
- EKF의 'odom1' 입력으로 사용 + map→odom TF 발행
"""

import rospy
import utm
import tf2_ros
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

class GPSToUTMSimple:
    def __init__(self):
        rospy.init_node('gps_to_utm_simple')

        # Datum 설정 (첫 GPS 위치를 원점으로)
        self.datum_set = False
        self.datum_utm_x = None
        self.datum_utm_y = None
        self.datum_zone = None
        self.datum_letter = None

        # 구독/발행
        self.gps_sub = rospy.Subscriber('/ublox/fix', NavSatFix, self.gps_callback)
        self.utm_pub = rospy.Publisher('/pose/gps', PoseWithCovarianceStamped, queue_size=10)

        # TF broadcaster (map → odom 발행)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        rospy.loginfo("GPS to UTM converter initialized (simple mode, publishing PoseWithCovarianceStamped + map→odom TF)")

    def gps_callback(self, msg):
        """GPS → UTM 변환"""
        # UTM 변환
        try:
            utm_x, utm_y, zone, letter = utm.from_latlon(msg.latitude, msg.longitude)
        except Exception as e:
            rospy.logwarn(f"UTM conversion failed: {e}")
            return

        # 첫 GPS 수신 시 datum 설정
        if not self.datum_set:
            self.datum_utm_x = utm_x
            self.datum_utm_y = utm_y
            self.datum_zone = zone
            self.datum_letter = letter
            self.datum_set = True
            rospy.loginfo(f"Datum set: UTM ({utm_x:.2f}, {utm_y:.2f}) zone {zone}{letter}")
            # 첫 메시지는 원점 설정에만 사용하고 발행하지 않음
            return

        # Datum 대비 상대 위치 계산
        relative_x = utm_x - self.datum_utm_x
        relative_y = utm_y - self.datum_utm_y

        # PoseWithCovarianceStamped 메시지 생성
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = msg.header.stamp
        pose_msg.header.frame_id = "map"  # EKF의 world_frame과 일치시켜야 함

        # 위치만 설정
        pose_msg.pose.pose.position.x = relative_x
        pose_msg.pose.pose.position.y = relative_y
        pose_msg.pose.pose.position.z = 0.0 # 2D 환경이므로 0으로 설정

        # 방향은 단위 쿼터니언 (중립, 방향 정보 없음을 의미)
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = 0.0
        pose_msg.pose.pose.orientation.w = 1.0

        # 공분산 설정 (가장 중요!)
        # EKF가 NaN에 빠지지 않도록 0이 아닌 적절한 값을 설정해야 합니다.
        # [x, y, z, roll, pitch, yaw] 순서의 분산(variance) 값
        # 사용하지 않는 값(z, roll, pitch, yaw)은 매우 큰 분산 값을 주어 EKF가 무시하도록 함
        pose_msg.pose.covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # x, y 위치에 대한 분산 설정 (대각 행렬)
        pose_msg.pose.covariance[0] = 0.1  # X 위치 분산
        pose_msg.pose.covariance[7] = 0.1  # Y 위치 분산
        
        # 사용하지 않는 측정값들의 분산은 매우 큰 값으로 설정
        pose_msg.pose.covariance[14] = 99999  # Z 위치 분산
        pose_msg.pose.covariance[21] = 99999  # Roll 방향 분산
        pose_msg.pose.covariance[28] = 99999  # Pitch 방향 분산
        pose_msg.pose.covariance[35] = 99999  # Yaw 방향 분산

        # 발행
        self.utm_pub.publish(pose_msg)

        # map → odom TF 발행 (datum 기준으로 고정)
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "map"
        t.child_frame_id = "odom"

        # datum 위치를 map 원점으로 설정 (odom은 datum에서 시작)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # 회전 없음
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

        rospy.logdebug(f"GPS→UTM: ({relative_x:.2f}, {relative_y:.2f})")

if __name__ == '__main__':
    try:
        node = GPSToUTMSimple()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass