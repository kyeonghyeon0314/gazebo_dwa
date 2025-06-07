#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
UTM 기반 Localization 노드
GPS, IMU, Odometry 데이터를 융합하여 UTM 좌표계에서 정확한 위치 추정
"""

import rospy
import tf2_ros
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from geographic_msgs.msg import GeoPoseStamped
from std_msgs.msg import String
from geodesy import utm
import numpy as np
import threading
from collections import deque
from scipy.spatial.transform import Rotation as R

class UTMLocalizationNode:
    def __init__(self):
        rospy.init_node('utm_localization_node', anonymous=True)
        
        # 파라미터 설정
        self.fixed_frame = rospy.get_param('~fixed_frame', 'utm')
        self.robot_frame = rospy.get_param('~robot_frame', 'base_link')
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.gps_frame = rospy.get_param('~gps_frame', 'gps_link')
        
        # 필터링 파라미터
        self.gps_weight = rospy.get_param('~gps_weight', 0.7)
        self.odom_weight = rospy.get_param('~odom_weight', 0.2)
        self.imu_weight = rospy.get_param('~imu_weight', 0.1)
        self.max_gps_age = rospy.get_param('~max_gps_age', 2.0)  # seconds
        self.utm_zone = rospy.get_param('~utm_zone', None)
        self.utm_band = rospy.get_param('~utm_band', None)
        
        # 상태 변수
        self.lock = threading.Lock()
        self.utm_origin = None
        self.last_gps_time = None
        self.last_odom_time = None
        self.last_imu_time = None
        self.origin_set = False
        
        # 센서 데이터 저장
        self.latest_gps = None
        self.latest_odom = None
        self.latest_imu = None
        self.gps_history = deque(maxlen=50)
        
        # 융합된 pose 상태
        self.fused_pose = {
            'x': 0.0, 'y': 0.0, 'z': 0.0,
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
            'vx': 0.0, 'vy': 0.0, 'vz': 0.0,
            'timestamp': rospy.Time.now()
        }
        
        # TF 관련
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Publishers
        self.fused_odom_pub = rospy.Publisher('/odometry/filtered', Odometry, queue_size=1)
        self.utm_pose_pub = rospy.Publisher('/utm_pose', GeoPoseStamped, queue_size=1)
        self.localization_status_pub = rospy.Publisher('/localization_status', String, queue_size=1)
        self.pose_cov_pub = rospy.Publisher('/pose_with_covariance', PoseWithCovarianceStamped, queue_size=1)
        
        # Subscribers
        self.gps_sub = rospy.Subscriber('/gps/fix', NavSatFix, self.gps_callback)
        self.odom_sub = rospy.Subscriber('/husky_velocity_controller/odom', Odometry, self.odom_callback)
        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        
        # 융합 타이머
        self.fusion_timer = rospy.Timer(rospy.Duration(0.1), self.fusion_callback)  # 10Hz
        
        rospy.loginfo("=== UTM Localization Node 시작 ===")
        rospy.loginfo(f"Fixed Frame: {self.fixed_frame}")
        rospy.loginfo(f"센서 가중치 - GPS: {self.gps_weight}, Odom: {self.odom_weight}, IMU: {self.imu_weight}")
        
    def gps_callback(self, gps_msg):
        """GPS 데이터 콜백"""
        if gps_msg.status.status < 0:
            return
            
        with self.lock:
            try:
                # GPS 좌표를 UTM으로 변환
                utm_point = utm.fromLatLong(gps_msg.latitude, gps_msg.longitude)
                
                # UTM zone 초기화
                if self.utm_zone is None:
                    self.utm_zone = utm_point.zone
                    self.utm_band = utm_point.band
                    rospy.loginfo(f"UTM Zone 설정: {self.utm_zone}{self.utm_band}")
                
                # 첫 번째 GPS 데이터로 원점 설정
                if not self.origin_set:
                    self.utm_origin = utm_point
                    self.origin_set = True
                    rospy.loginfo(f"UTM 원점 설정: {utm_point.easting:.2f}, {utm_point.northing:.2f}")
                
                # 상대 좌표로 변환
                relative_x = utm_point.easting - self.utm_origin.easting
                relative_y = utm_point.northing - self.utm_origin.northing
                
                self.latest_gps = {
                    'x': relative_x,
                    'y': relative_y,
                    'z': gps_msg.altitude if hasattr(gps_msg, 'altitude') else 0.0,
                    'covariance': gps_msg.position_covariance,
                    'timestamp': gps_msg.header.stamp
                }
                
                self.last_gps_time = rospy.Time.now()
                self.gps_history.append(self.latest_gps)
                
                # UTM pose 발행
                self.publish_utm_pose(utm_point, gps_msg.header.stamp)
                
            except Exception as e:
                rospy.logwarn(f"GPS 데이터 처리 오류: {e}")
    
    def odom_callback(self, odom_msg):
        """Odometry 데이터 콜백"""
        with self.lock:
            self.latest_odom = {
                'x': odom_msg.pose.pose.position.x,
                'y': odom_msg.pose.pose.position.y,
                'z': odom_msg.pose.pose.position.z,
                'orientation': odom_msg.pose.pose.orientation,
                'linear_vel': odom_msg.twist.twist.linear,
                'angular_vel': odom_msg.twist.twist.angular,
                'pose_covariance': odom_msg.pose.covariance,
                'twist_covariance': odom_msg.twist.covariance,
                'timestamp': odom_msg.header.stamp
            }
            self.last_odom_time = rospy.Time.now()
    
    def imu_callback(self, imu_msg):
        """IMU 데이터 콜백"""
        with self.lock:
            self.latest_imu = {
                'orientation': imu_msg.orientation,
                'angular_velocity': imu_msg.angular_velocity,
                'linear_acceleration': imu_msg.linear_acceleration,
                'orientation_covariance': imu_msg.orientation_covariance,
                'angular_velocity_covariance': imu_msg.angular_velocity_covariance,
                'linear_acceleration_covariance': imu_msg.linear_acceleration_covariance,
                'timestamp': imu_msg.header.stamp
            }
            self.last_imu_time = rospy.Time.now()
    
    def fusion_callback(self, event):
        """센서 융합 메인 콜백"""
        if not self.origin_set:
            return
            
        with self.lock:
            current_time = rospy.Time.now()
            
            # 센서 데이터 유효성 검사
            gps_valid = (self.latest_gps is not None and 
                        self.last_gps_time is not None and
                        (current_time - self.last_gps_time).to_sec() < self.max_gps_age)
            
            odom_valid = (self.latest_odom is not None and 
                         self.last_odom_time is not None and
                         (current_time - self.last_odom_time).to_sec() < 1.0)
            
            imu_valid = (self.latest_imu is not None and 
                        self.last_imu_time is not None and
                        (current_time - self.last_imu_time).to_sec() < 1.0)
            
            # 센서 융합 수행
            if gps_valid or odom_valid or imu_valid:
                self.perform_sensor_fusion(gps_valid, odom_valid, imu_valid)
                self.publish_fused_pose()
                self.publish_transforms()
            
            # 상태 메시지 발행
            self.publish_status(gps_valid, odom_valid, imu_valid)
    
    def perform_sensor_fusion(self, gps_valid, odom_valid, imu_valid):
        """센서 데이터 융합"""
        # 가중치 정규화
        total_weight = 0.0
        if gps_valid: total_weight += self.gps_weight
        if odom_valid: total_weight += self.odom_weight
        if imu_valid: total_weight += self.imu_weight
        
        if total_weight == 0:
            return
        
        # 위치 융합 (GPS + Odometry)
        fused_x = fused_y = fused_z = 0.0
        position_weight = 0.0
        
        if gps_valid:
            weight = self.gps_weight / total_weight
            fused_x += self.latest_gps['x'] * weight
            fused_y += self.latest_gps['y'] * weight
            fused_z += self.latest_gps['z'] * weight
            position_weight += weight
        
        if odom_valid:
            weight = self.odom_weight / total_weight
            # Odometry는 상대적이므로 이전 위치에서의 변화량을 적용
            fused_x += self.latest_odom['x'] * weight
            fused_y += self.latest_odom['y'] * weight
            fused_z += self.latest_odom['z'] * weight
            position_weight += weight
        
        # 방향 융합 (IMU + Odometry)
        fused_roll = fused_pitch = fused_yaw = 0.0
        orientation_weight = 0.0
        
        if imu_valid:
            imu_quat = self.latest_imu['orientation']
            imu_roll, imu_pitch, imu_yaw = euler_from_quaternion([
                imu_quat.x, imu_quat.y, imu_quat.z, imu_quat.w])
            
            weight = self.imu_weight / total_weight
            fused_roll += imu_roll * weight
            fused_pitch += imu_pitch * weight
            fused_yaw += imu_yaw * weight
            orientation_weight += weight
        
        if odom_valid:
            odom_quat = self.latest_odom['orientation']
            odom_roll, odom_pitch, odom_yaw = euler_from_quaternion([
                odom_quat.x, odom_quat.y, odom_quat.z, odom_quat.w])
            
            weight = self.odom_weight / total_weight
            fused_roll += odom_roll * weight
            fused_pitch += odom_pitch * weight
            fused_yaw += odom_yaw * weight
            orientation_weight += weight
        
        # 속도 융합 (Odometry 기반)
        fused_vx = fused_vy = fused_vz = 0.0
        if odom_valid:
            fused_vx = self.latest_odom['linear_vel'].x
            fused_vy = self.latest_odom['linear_vel'].y
            fused_vz = self.latest_odom['linear_vel'].z
        
        # 융합 결과 업데이트
        self.fused_pose.update({
            'x': fused_x, 'y': fused_y, 'z': fused_z,
            'roll': fused_roll, 'pitch': fused_pitch, 'yaw': fused_yaw,
            'vx': fused_vx, 'vy': fused_vy, 'vz': fused_vz,
            'timestamp': rospy.Time.now()
        })
    
    def publish_fused_pose(self):
        """융합된 pose 발행"""
        # Odometry 메시지 생성
        fused_odom = Odometry()
        fused_odom.header.stamp = self.fused_pose['timestamp']
        fused_odom.header.frame_id = self.fixed_frame
        fused_odom.child_frame_id = self.robot_frame
        
        # 위치 설정
        fused_odom.pose.pose.position.x = self.fused_pose['x']
        fused_odom.pose.pose.position.y = self.fused_pose['y']
        fused_odom.pose.pose.position.z = self.fused_pose['z']
        
        # 방향 설정 (쿼터니언)
        q = quaternion_from_euler(
            self.fused_pose['roll'], 
            self.fused_pose['pitch'], 
            self.fused_pose['yaw'])
        fused_odom.pose.pose.orientation.x = q[0]
        fused_odom.pose.pose.orientation.y = q[1]
        fused_odom.pose.pose.orientation.z = q[2]
        fused_odom.pose.pose.orientation.w = q[3]
        
        # 속도 설정
        fused_odom.twist.twist.linear.x = self.fused_pose['vx']
        fused_odom.twist.twist.linear.y = self.fused_pose['vy']
        fused_odom.twist.twist.linear.z = self.fused_pose['vz']
        
        # 공분산 설정 (추정값)
        fused_odom.pose.covariance = [0.0] * 36
        fused_odom.pose.covariance[0] = 0.1   # x-x
        fused_odom.pose.covariance[7] = 0.1   # y-y
        fused_odom.pose.covariance[14] = 0.1  # z-z
        fused_odom.pose.covariance[21] = 0.05 # roll-roll
        fused_odom.pose.covariance[28] = 0.05 # pitch-pitch
        fused_odom.pose.covariance[35] = 0.05 # yaw-yaw
        
        fused_odom.twist.covariance = [0.0] * 36
        fused_odom.twist.covariance[0] = 0.1   # vx-vx
        fused_odom.twist.covariance[7] = 0.1   # vy-vy
        fused_odom.twist.covariance[14] = 0.1  # vz-vz
        
        self.fused_odom_pub.publish(fused_odom)
        
        # PoseWithCovariance도 발행
        pose_cov = PoseWithCovarianceStamped()
        pose_cov.header = fused_odom.header
        pose_cov.pose = fused_odom.pose
        self.pose_cov_pub.publish(pose_cov)
    
    def publish_transforms(self):
        """TF 변환 발행"""
        # UTM -> base_link 변환
        transform = TransformStamped()
        transform.header.stamp = self.fused_pose['timestamp']
        transform.header.frame_id = self.fixed_frame
        transform.child_frame_id = self.robot_frame
        
        transform.transform.translation.x = self.fused_pose['x']
        transform.transform.translation.y = self.fused_pose['y']
        transform.transform.translation.z = self.fused_pose['z']
        
        q = quaternion_from_euler(
            self.fused_pose['roll'], 
            self.fused_pose['pitch'], 
            self.fused_pose['yaw'])
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(transform)
        
        # UTM -> map 변환 (동일하게 설정)
        map_transform = TransformStamped()
        map_transform.header.stamp = self.fused_pose['timestamp']
        map_transform.header.frame_id = self.fixed_frame
        map_transform.child_frame_id = self.map_frame
        map_transform.transform.translation.x = 0.0
        map_transform.transform.translation.y = 0.0
        map_transform.transform.translation.z = 0.0
        map_transform.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(map_transform)
    
    def publish_utm_pose(self, utm_point, timestamp):
        """UTM pose 발행"""
        utm_pose = GeoPoseStamped()
        utm_pose.header.stamp = timestamp
        utm_pose.header.frame_id = self.fixed_frame
        
        # UTM 좌표 상대 위치
        utm_pose.pose.position.latitude = utm_point.easting - self.utm_origin.easting
        utm_pose.pose.position.longitude = utm_point.northing - self.utm_origin.northing
        utm_pose.pose.position.altitude = 0.0
        
        self.utm_pose_pub.publish(utm_pose)
    
    def publish_status(self, gps_valid, odom_valid, imu_valid):
        """상태 메시지 발행"""
        status_parts = []
        if gps_valid: status_parts.append("GPS: OK")
        else: status_parts.append("GPS: NO")
        
        if odom_valid: status_parts.append("ODOM: OK")
        else: status_parts.append("ODOM: NO")
        
        if imu_valid: status_parts.append("IMU: OK")
        else: status_parts.append("IMU: NO")
        
        status_msg = String()
        status_msg.data = f"Localization - {', '.join(status_parts)}"
        self.localization_status_pub.publish(status_msg)

if __name__ == '__main__':
    try:
        utm_localization = UTMLocalizationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass