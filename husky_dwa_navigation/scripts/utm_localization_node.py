#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
UTM 기반 Localization 노드 (Python 3.8 호환 버전)
Static transform만 사용한 기본적인 UTM 좌표계 설정
"""

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String

class UTMLocalizationNode:
    def __init__(self):
        rospy.init_node('utm_localization_node', anonymous=True)
        
        # 파라미터 설정
        self.fixed_frame = rospy.get_param('~fixed_frame', 'utm')
        self.robot_frame = rospy.get_param('~robot_frame', 'base_link')
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.utm_zone = rospy.get_param('~utm_zone', 31)
        self.utm_band = rospy.get_param('~utm_band', 'N')
        
        # TF 브로드캐스터
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        
        # 상태 퍼블리셔
        self.status_pub = rospy.Publisher('/localization_status', String, queue_size=1)
        
        # UTM 프레임 설정
        self.setup_utm_transforms()
        
        rospy.loginfo("=== UTM Localization Node 시작 (Python 3.8) ===")
        rospy.loginfo(f"UTM Zone: {self.utm_zone}{self.utm_band}")
        rospy.loginfo(f"Fixed Frame: {self.fixed_frame}")
        
        # 상태 메시지 타이머
        self.status_timer = rospy.Timer(rospy.Duration(2.0), self.publish_status)
    
    def setup_utm_transforms(self):
        """UTM 관련 transform 설정"""
        transforms = []
        
        # UTM -> Map transform (동일 좌표계로 설정)
        utm_to_map = TransformStamped()
        utm_to_map.header.stamp = rospy.Time.now()
        utm_to_map.header.frame_id = self.fixed_frame
        utm_to_map.child_frame_id = self.map_frame
        utm_to_map.transform.translation.x = 0.0
        utm_to_map.transform.translation.y = 0.0
        utm_to_map.transform.translation.z = 0.0
        utm_to_map.transform.rotation.w = 1.0
        transforms.append(utm_to_map)
        
        # 모든 transform 발행
        self.tf_broadcaster.sendTransform(transforms)
        rospy.loginfo("UTM static transforms 설정 완료")
    
    def publish_status(self, event):
        """상태 메시지 발행"""
        status_msg = String()
        status_msg.data = f"UTM Localization Active - Zone: {self.utm_zone}{self.utm_band}"
        self.status_pub.publish(status_msg)

if __name__ == '__main__':
    try:
        utm_localization = UTMLocalizationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("UTM Localization Node 종료")