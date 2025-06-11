#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
최종 GPS 기반 Heading 설정 노드
1. GPS 이동 감지 시 자동으로 heading 계산
2. EKF localization에 확실히 적용
3. 완료 후 Ctrl+C와 동일한 방식으로 자동 종료
4. 여러 방법으로 initial pose 설정 시도
"""

import rospy
import tf2_ros
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPoseStamped
from std_msgs.msg import String, Bool
from geodesy import utm
import numpy as np
import threading
from collections import deque
import os
import signal

class FinalGPSHeadingInitializer:
    def __init__(self):
        rospy.init_node('final_gps_heading_initializer', anonymous=True)
        
        # 파라미터 설정
        self.fixed_frame = rospy.get_param('~fixed_frame', 'utm')
        self.robot_frame = rospy.get_param('~robot_frame', 'base_link')
        self.gps_frame = rospy.get_param('~gps_frame', 'gps_link')
        self.heading_samples = rospy.get_param('~heading_samples', 30)
        self.utm_zone = rospy.get_param('~utm_zone', None)
        self.utm_band = rospy.get_param('~utm_band', None)
        
        # 상태 변수
        self.lock = threading.Lock()
        self.gps_data_buffer = deque(maxlen=self.heading_samples)
        self.initial_position_set = False
        self.heading_initialized = False
        self.heading_applied = False
        self.utm_origin = None
        
        # 초기 GPS 데이터 저장 (이동 거리 계산용)
        self.first_gps_data = None
        self.current_gps_data = None
        
        # 로그 출력 제어 변수
        self.last_log_time = rospy.Time.now()
        self.last_progress_log_time = rospy.Time.now()
        self.log_interval = 5.0  # 5초마다 좌표 비교 로그 출력
        self.progress_log_interval = 10.0  # 10초마다 진행 상황 로그 출력
        
        # 자동 캘리브레이션 관련 변수
        self.auto_calibration_started = False
        self.start_time = None
        self.min_movement_distance = 0.5  # 최소 이동거리
        self.stability_check_time = 8.0   # 8초 동안 데이터 수집 후 계산
        self.min_samples_required = 10    # 최소 필요 샘플 수
        
        # Initial pose 적용 관련 변수
        self.calculated_heading = None
        
        # TF 관련
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Publishers - 다양한 토픽에 발행
        self.initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
        self.initial_pose_pub_ekf = rospy.Publisher('/set_pose', PoseWithCovarianceStamped, queue_size=1)
        self.utm_pose_pub = rospy.Publisher('/utm_pose', GeoPoseStamped, queue_size=1)
        self.status_pub = rospy.Publisher('/gps_heading_status', String, queue_size=1)
        self.heading_applied_pub = rospy.Publisher('/heading_applied', Bool, queue_size=1)
        
        # Subscribers
        self.gps_sub = rospy.Subscriber('/gps/fix', NavSatFix, self.gps_callback)
        
        rospy.loginfo("=== 최종 GPS Heading Initializer 시작 ===")
        rospy.loginfo("📍 UTM 좌표 차이 로깅 활성화 (5초당 1번)")
        rospy.loginfo("🤖 이동 감지 시 자동으로 heading 계산을 시작합니다")
        rospy.loginfo("🎯 완료 후 노드가 자동으로 종료됩니다 (Ctrl+C와 동일)")
        
        # 파라미터 정보 출력
        rospy.loginfo(f"최소 이동 거리: {self.min_movement_distance} m")
        rospy.loginfo(f"GPS 샘플 수: {self.heading_samples}")
        rospy.loginfo(f"안정성 체크 시간: {self.stability_check_time} 초")
        
    def gps_callback(self, gps_msg):
        """GPS 데이터 콜백"""
        if gps_msg.status.status < 0:  # GPS 신호가 없는 경우
            return
            
        with self.lock:
            try:
                # GPS 좌표를 UTM으로 변환
                utm_point = utm.fromLatLong(gps_msg.latitude, gps_msg.longitude)
                
                # UTM zone 초기화
                if self.utm_zone is None:
                    self.utm_zone = utm_point.zone
                    self.utm_band = utm_point.band
                    rospy.loginfo(f"🌍 UTM Zone 설정: {self.utm_zone}{self.utm_band}")
                
                # 첫 번째 GPS 데이터로 원점 설정
                if not self.initial_position_set:
                    self.utm_origin = utm_point
                    self.initial_position_set = True
                    self.start_time = rospy.Time.now()
                    self.publish_utm_to_map_transform(utm_point)
                    rospy.loginfo(f"📍 UTM 원점 설정: {utm_point.easting:.2f}, {utm_point.northing:.2f}")
                    rospy.loginfo("🚶 로봇을 움직여주세요. 자동으로 이동을 감지합니다...")
                    
                    # 첫 번째 GPS 데이터 저장
                    self.first_gps_data = {
                        'easting': utm_point.easting,
                        'northing': utm_point.northing,
                        'timestamp': gps_msg.header.stamp
                    }
                
                # 현재 GPS 데이터 업데이트
                self.current_gps_data = {
                    'easting': utm_point.easting,
                    'northing': utm_point.northing,
                    'timestamp': gps_msg.header.stamp
                }
                
                # GPS 데이터를 버퍼에 저장 (상대 좌표)
                relative_x = utm_point.easting - self.utm_origin.easting
                relative_y = utm_point.northing - self.utm_origin.northing
                
                gps_data = {
                    'timestamp': gps_msg.header.stamp,
                    'x': relative_x,
                    'y': relative_y,
                    'covariance': gps_msg.position_covariance
                }
                
                self.gps_data_buffer.append(gps_data)
                
                # 좌표 비교 로그 출력 (5초마다)
                self.log_coordinate_comparison(utm_point)
                
                # 자동 캘리브레이션 실행
                if not self.heading_initialized and self.initial_position_set:
                    self.auto_calculate_heading()
                
                # UTM pose 발행
                self.publish_utm_pose(utm_point, gps_msg.header.stamp)
                
            except Exception as e:
                rospy.logerr(f"GPS 콜백 처리 오류: {e}")
    
    def log_coordinate_comparison(self, utm_point):
        """좌표 비교 로그 출력 (5초마다)"""
        current_time = rospy.Time.now()
        if (current_time - self.last_log_time).to_sec() >= self.log_interval:
            self.last_log_time = current_time
            
            # 시뮬레이션 좌표 가져오기 (기본값 사용)
            sim_x = 166026.468
            sim_y = 0.930
            
            diff_x = utm_point.easting - sim_x
            diff_y = utm_point.northing - sim_y
            distance_diff = np.sqrt(diff_x*diff_x + diff_y*diff_y)
            
            rospy.loginfo("============================================================")
            rospy.loginfo("📊 GPS vs 시뮬레이션 UTM 좌표 비교")
            rospy.loginfo(f"🛰️  GPS UTM:    E={utm_point.easting:.3f}, N={utm_point.northing:.3f}")
            rospy.loginfo(f"🎮 시뮬레이션:  E={sim_x:.3f}, N={sim_y:.3f}")
            rospy.loginfo(f"📏 차이:       ΔE={diff_x:.3f}m, ΔN={diff_y:.3f}m")
            rospy.loginfo(f"📐 거리 차이:   {distance_diff:.3f}m")
            rospy.loginfo("============================================================")
    
    def auto_calculate_heading(self):
        """자동으로 이동 감지 및 heading 계산"""
        # 충분한 데이터가 모일 때까지 대기
        if len(self.gps_data_buffer) < self.min_samples_required:
            return
            
        # 시작 시간으로부터 일정 시간 경과 확인
        elapsed_time = (rospy.Time.now() - self.start_time).to_sec()
        
        # 첫 번째 GPS 데이터와 현재 GPS 데이터 비교하여 총 이동 거리 계산
        if self.first_gps_data is None or self.current_gps_data is None:
            return
            
        dx = self.current_gps_data['easting'] - self.first_gps_data['easting']
        dy = self.current_gps_data['northing'] - self.first_gps_data['northing']
        total_distance = np.sqrt(dx*dx + dy*dy)
        
        # 진행 상황 로그 출력 (10초마다만)
        current_time = rospy.Time.now()
        if not self.auto_calibration_started and (current_time - self.last_progress_log_time).to_sec() >= self.progress_log_interval:
            self.last_progress_log_time = current_time
            rospy.loginfo(f"🔍 현재 총 이동 거리: {total_distance:.3f}m (필요: {self.min_movement_distance}m)")
            rospy.loginfo(f"⏰ 경과 시간: {elapsed_time:.1f}초")
            rospy.loginfo(f"📊 GPS 샘플: {len(self.gps_data_buffer)}/{self.heading_samples}")
        
        # 자동 캘리브레이션 시작 조건 확인
        if not self.auto_calibration_started:
            # 최소 이동 거리와 시간 조건 확인
            if total_distance >= self.min_movement_distance and elapsed_time >= self.stability_check_time:
                self.auto_calibration_started = True
                rospy.loginfo("🚀 충분한 이동 감지! 자동 캘리브레이션 시작")
                rospy.loginfo(f"📏 감지된 총 이동 거리: {total_distance:.3f}m")
                rospy.loginfo(f"📐 이동 방향: ΔE={dx:.3f}m, ΔN={dy:.3f}m")
                self.calculate_and_set_heading(total_distance, dx, dy)
            return
    
    def calculate_and_set_heading(self, distance, dx, dy):
        """이동 데이터를 통해 heading 계산 및 설정"""
        # Heading 계산 (atan2를 사용하여 방향 계산)
        self.calculated_heading = np.arctan2(dy, dx)
        
        rospy.loginfo("🎉 ============================================")
        rospy.loginfo("🧭 GPS 기반 Heading 계산 완료!")
        rospy.loginfo(f"📐 계산된 Heading: {np.degrees(self.calculated_heading):.1f}도")
        rospy.loginfo(f"📏 총 이동 거리: {distance:.2f}m")
        rospy.loginfo(f"📊 사용된 GPS 샘플: {len(self.gps_data_buffer)}개")
        rospy.loginfo(f"🧮 이동 벡터: ΔE={dx:.3f}m, ΔN={dy:.3f}m")
        rospy.loginfo("🎉 ============================================")
        
        self.heading_initialized = True
        
        # Initial pose 설정 시도
        self.apply_initial_pose_multiple_ways()
        
        rospy.loginfo("✅ Heading 설정이 완료되었습니다.")
        rospy.loginfo("🔄 3초 후 자동으로 종료됩니다...")
        
        # 마지막 상태 정보 발행
        rospy.sleep(1.0)
        self.publish_status("Heading 캘리브레이션 완료 - 노드 종료 중")
        rospy.sleep(2.0)
        
        rospy.loginfo("👋 GPS Heading Initializer 종료")
        
        # Ctrl+C와 동일한 방식으로 자동 종료 (SIGINT 신호 전송)
        os.kill(os.getpid(), signal.SIGINT)
        
    def apply_initial_pose_multiple_ways(self):
        """다양한 방법으로 initial pose 적용"""
        if self.calculated_heading is None:
            return
            
        rospy.loginfo("🔧 다양한 방법으로 Initial Pose 적용을 시도합니다...")
        
        # 방법 1: /initialpose 토픽 (RViz, AMCL용)
        self.publish_initial_pose_to_topic('/initialpose', self.initial_pose_pub)
        
        # 방법 2: /set_pose 토픽 (EKF용)
        self.publish_initial_pose_to_topic('/set_pose', self.initial_pose_pub_ekf)
        
        # 방법 3: utm frame과 map frame 둘 다 시도
        self.publish_initial_pose_multiple_frames()
        
        rospy.loginfo("🔧 Initial Pose 적용 완료. Localization이 업데이트될 때까지 기다려주세요.")
        
    def publish_initial_pose_to_topic(self, topic_name, publisher):
        """특정 토픽에 initial pose 발행"""
        initial_pose = self.create_initial_pose_message(self.fixed_frame)
        publisher.publish(initial_pose)
        rospy.loginfo(f"📍 Initial pose published to {topic_name}")
        
    def publish_initial_pose_multiple_frames(self):
        """여러 frame에 대해 initial pose 발행"""
        frames_to_try = ['utm', 'map', 'odom']
        
        for frame in frames_to_try:
            initial_pose = self.create_initial_pose_message(frame)
            self.initial_pose_pub.publish(initial_pose)
            rospy.sleep(0.1)  # 짧은 지연
            rospy.loginfo(f"📍 Initial pose published with frame: {frame}")
            
    def create_initial_pose_message(self, frame_id):
        """Initial pose 메시지 생성"""
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.stamp = rospy.Time.now()
        initial_pose.header.frame_id = frame_id
        
        # 현재 GPS 기반 위치 사용
        if len(self.gps_data_buffer) > 0:
            latest_gps = self.gps_data_buffer[-1]
            initial_pose.pose.pose.position.x = latest_gps['x']
            initial_pose.pose.pose.position.y = latest_gps['y']
        else:
            initial_pose.pose.pose.position.x = 0.0
            initial_pose.pose.pose.position.y = 0.0
            
        initial_pose.pose.pose.position.z = 0.0
        
        # 계산된 heading으로 orientation 설정
        quat = quaternion_from_euler(0, 0, self.calculated_heading)
        initial_pose.pose.pose.orientation.x = quat[0]
        initial_pose.pose.pose.orientation.y = quat[1]
        initial_pose.pose.pose.orientation.z = quat[2]
        initial_pose.pose.pose.orientation.w = quat[3]
        
        # Covariance 설정 (GPS 기반이므로 위치는 높은 신뢰도, heading은 중간 신뢰도)
        covariance = [0.0] * 36
        covariance[0] = 0.25   # x
        covariance[7] = 0.25   # y  
        covariance[35] = 0.1   # yaw (계산된 heading)
        initial_pose.pose.covariance = covariance
        
        return initial_pose
    
    def publish_utm_to_map_transform(self, utm_point):
        """UTM에서 map으로의 transform 발행"""
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = self.fixed_frame
        transform.child_frame_id = "map"
        
        # UTM 원점을 map의 원점으로 설정
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        
        # 회전 없음 (UTM과 map이 동일한 방향)
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(transform)
    
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
    
    def publish_status(self, message):
        """상태 메시지 발행"""
        status_msg = String()
        status_msg.data = message
        self.status_pub.publish(status_msg)
    
    def run(self):
        """메인 루프"""
        rate = rospy.Rate(10)  # 10Hz
        
        while not rospy.is_shutdown():
            # UTM to map transform 지속적으로 발행
            if self.initial_position_set:
                self.publish_utm_to_map_transform(self.utm_origin)
            
            # 상태 메시지 주기적 발행
            if not self.heading_initialized and self.initial_position_set:
                if not self.auto_calibration_started:
                    self.publish_status("이동 감지 대기 중... 로봇을 움직여주세요")
                else:
                    self.publish_status("Heading 계산 중...")
            elif self.heading_initialized:
                self.publish_status(f"Heading 캘리브레이션 완료: {np.degrees(self.calculated_heading):.1f}도")
                # Heading 적용 상태 발행
                applied_msg = Bool()
                applied_msg.data = True
                self.heading_applied_pub.publish(applied_msg)
            elif not self.initial_position_set:
                self.publish_status("GPS 신호 대기 중...")
            
            rate.sleep()

if __name__ == '__main__':
    try:
        rospy.loginfo("최종 GPS Heading Initializer 시작")
        gps_heading_init = FinalGPSHeadingInitializer()
        gps_heading_init.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("GPS Heading Initializer 종료")
        pass