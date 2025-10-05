#!/usr/bin/env python3
"""
IESKF Odometry → UTM 좌표 변환
- faster-lio의 /Odometry (odom 좌표계) → UTM 좌표계로 변환
- 첫 GPS 위치를 IESKF 원점의 UTM 좌표로 사용
"""

import rospy
import utm
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry

class IESKFToUTM:
    def __init__(self):
        rospy.init_node('ieskf_to_utm')

        # UTM offset (첫 GPS 위치)
        self.offset_set = False
        self.offset_utm_x = None
        self.offset_utm_y = None

        # 구독/발행
        self.gps_sub = rospy.Subscriber('/ublox/fix', NavSatFix, self.gps_callback, queue_size=1)
        self.ieskf_sub = rospy.Subscriber('/Odometry', Odometry, self.ieskf_callback)
        self.utm_pub = rospy.Publisher('/odometry/ieskf_utm', Odometry, queue_size=10)

        rospy.loginfo("IESKF to UTM converter initialized")

    def gps_callback(self, msg):
        """첫 GPS로 UTM offset 설정"""
        if not self.offset_set:
            try:
                utm_x, utm_y, zone, letter = utm.from_latlon(msg.latitude, msg.longitude)
                self.offset_utm_x = utm_x
                self.offset_utm_y = utm_y
                self.offset_set = True
                rospy.loginfo(f"IESKF UTM offset set: ({utm_x:.2f}, {utm_y:.2f}) zone {zone}{letter}")
                # 한 번만 설정
                self.gps_sub.unregister()
            except Exception as e:
                rospy.logwarn(f"UTM offset setting failed: {e}")

    def ieskf_callback(self, msg):
        """IESKF odom 좌표 → UTM 좌표 변환"""
        if not self.offset_set:
            return  # offset 설정 전까지 무시

        # odom 좌표 + UTM offset = UTM 좌표
        utm_msg = Odometry()
        utm_msg.header.stamp = msg.header.stamp
        utm_msg.header.frame_id = "map"  # UTM 기반
        utm_msg.child_frame_id = "odom"

        # 위치 변환
        utm_msg.pose.pose.position.x = msg.pose.pose.position.x + (self.offset_utm_x % 1000000)  # 큰 값 방지
        utm_msg.pose.pose.position.y = msg.pose.pose.position.y + (self.offset_utm_y % 1000000)
        utm_msg.pose.pose.position.z = 0.0  # 고도는 0으로 설정 (필요시 GPS 고도 사용 가능)

        # 방향은 그대로 (IESKF가 계산한 방향)
        utm_msg.pose.pose.orientation = msg.pose.pose.orientation

        # 속도/각속도도 그대로
        utm_msg.twist = msg.twist

        # 공분산 복사
        utm_msg.pose.covariance = msg.pose.covariance
        utm_msg.twist.covariance = msg.twist.covariance

        # 발행
        self.utm_pub.publish(utm_msg)

if __name__ == '__main__':
    try:
        node = IESKFToUTM()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
