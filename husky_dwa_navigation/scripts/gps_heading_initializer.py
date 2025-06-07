#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
GPS 기반 초기 Heading 설정 노드 (개선된 버전)
1. Space바를 누르기 전까지는 동작하지 않음
2. 로봇 전방 3m에 임시 목적지 생성
3. 이동 경로를 통해 초기 heading 설정
"""

import rospy
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, PoseStamped, Twist
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPoseStamped
from std_msgs.msg import String, Bool
from geodesy import utm
import numpy as np
import threading
from collections import deque
import termios
import sys
import tty
import select

class GPSHeadingInitializer:
    def __init__(self):
        rospy.init_node('gps_heading_initializer', anonymous=True)
        
        # 파라미터 설정
        self.fixed_frame = rospy.get_param('~fixed_frame', 'utm')
        self.robot_frame = rospy.get_param('~robot_frame', 'base_link')
        self.gps_frame = rospy.get_param('~gps_frame', 'gps_link')
        self.min_speed_threshold = rospy.get_param('~min_speed_threshold', 0.2)  # m/s
        self.heading_samples = rospy.get_param('~heading_samples', 15)
        self.forward_distance = rospy.get_param('~forward_distance', 3.0)  # 전방 3m
        self.utm_zone = rospy.get_param('~utm_zone', None)
        self.utm_band = rospy.get_param('~utm_band', None)
        
        # 상태 변수
        self.lock = threading.Lock()
        self.gps_data_buffer = deque(maxlen=self.heading_samples)
        self.initial_position_set = False
        self.heading_initialized = False
        self.space_pressed = False
        self.calibration_active = False
        self.forward_goal_sent = False
        self.utm_origin = None
        self.initial_robot_pose = None
        
        # TF 관련
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Publishers
        self.initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
        self.utm_pose_pub = rospy.Publisher('/utm_pose', GeoPoseStamped, queue_size=1)
        self.calibration_goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.status_pub = rospy.Publisher('/gps_heading_status', String, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # Subscribers
        self.gps_sub = rospy.Subscribe('/gps/fix', NavSatFix, self.gps_callback)
        self.space_sub = rospy.Subscriber('/space_key_pressed', Bool, self.space_callback)
        
        # 키보드 입력 처리를 위한 스레드
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
        
        rospy.loginfo("=== GPS Heading Initializer 시작 ===")
        rospy.loginfo("Space바를 눌러 초기 heading 캘리브레이션을 시작하세요.")
        rospy.loginfo(f"전방 목표 거리: {self.forward_distance}m")
        rospy.loginfo(f"최소 이동 속도: {self.min_speed_threshold} m/s")
        
    def keyboard_listener(self):
        """키보드 입력 감지 (별도 스레드)"""
        try:
            # 터미널 설정 저장
            old_settings = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())
            
            while not rospy.is_shutdown():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    if key == ' ':  # Space바
                        self.handle_space_press()
                    elif key == '\x03':  # Ctrl+C
                        break
                        
        except Exception as e:
            rospy.logwarn(f"키보드 입력 처리 오류: {e}")
        finally:
            # 터미널 설정 복원
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            except:
                pass
    
    def handle_space_press(self):
        """Space바 입력 처리"""
        if not self.space_pressed and not self.heading_initialized:
            self.space_pressed = True
            rospy.loginfo("⚡ Space바 감지! GPS 기반 heading 캘리브레이션을 시작합니다...")
            self.start_calibration()
        elif self.heading_initialized:
            rospy.loginfo("📍 Heading이 이미 초기화되었습니다.")
        else:
            rospy.loginfo("🔄 캘리브레이션이 이미 진행 중입니다...")
    
    def space_callback(self, msg):
        """Space 키 토픽 콜백 (대안적 입력 방법)"""
        if msg.data:
            self.handle_space_press()
    
    def start_calibration(self):
        """캘리브레이션 프로세스 시작"""
        if not self.initial_position_set:
            rospy.logwarn("⚠️  GPS 신호를 기다리는 중...")
            return
            
        self.calibration_active = True
        self.publish_status("캘리브레이션 시작: 전방 목표점 생성 중...")
        
        # 현재 로봇 위치 가져오기
        try:
            transform = self.tf_buffer.lookup_transform(
                self.fixed_frame, self.robot_frame, 
                rospy.Time(), rospy.Duration(1.0))
            
            current_x = transform.transform.translation.x
            current_y = transform.transform.translation.y
            
            # 현재 방향 가져오기 (추정값 또는 0)
            orientation = transform.transform.rotation
            _, _, current_yaw = euler_from_quaternion([
                orientation.x, orientation.y, orientation.z, orientation.w])
            
        except Exception as e:
            rospy.logwarn(f"현재 위치 가져오기 실패, GPS 위치 사용: {e}")
            if len(self.gps_data_buffer) > 0:
                latest_gps = self.gps_data_buffer[-1]
                current_x = latest_gps['x']
                current_y = latest_gps['y']
                current_yaw = 0.0  # 초기 방향 추정값
            else:
                rospy.logerr("❌ GPS 데이터가 없어 캘리브레이션을 시작할 수 없습니다.")
                return
        
        # 전방 3m 목표점 생성
        forward_x = current_x + self.forward_distance * np.cos(current_yaw)
        forward_y = current_y + self.forward_distance * np.sin(current_yaw)
        
        self.send_forward_goal(forward_x, forward_y)
        self.initial_robot_pose = {'x': current_x, 'y': current_y, 'yaw': current_yaw}
        
        rospy.loginfo(f"🎯 전방 목표점 설정: ({forward_x:.2f}, {forward_y:.2f})")
        rospy.loginfo("🚶 로봇이 이동하여 heading을 측정합니다...")
        
    def send_forward_goal(self, x, y):
        """전방 목표점 발송"""
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = self.fixed_frame
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0
        
        self.calibration_goal_pub.publish(goal)
        self.forward_goal_sent = True
        self.publish_status("전방 목표점으로 이동 중...")
        
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
                    self.publish_utm_to_map_transform(utm_point)
                    rospy.loginfo(f"📍 UTM 원점 설정: {utm_point.easting:.2f}, {utm_point.northing:.2f}")
                
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
                
                # 캘리브레이션이 활성화된 경우에만 heading 계산
                if self.calibration_active and not self.heading_initialized:
                    self.calculate_and_set_heading()
                
                # UTM pose 발행
                self.publish_utm_pose(utm_point, gps_msg.header.stamp)
                
            except Exception as e:
                rospy.logwarn(f"GPS 데이터 처리 중 오류: {e}")
    
    def calculate_and_set_heading(self):
        """GPS 데이터를 사용하여 heading 계산 및 설정"""
        if len(self.gps_data_buffer) < 5:  # 충분한 데이터 확보
            return
            
        # 최신 데이터와 시작 데이터 비교
        latest_data = self.gps_data_buffer[-1]
        start_data = self.gps_data_buffer[0]
        
        # 이동 거리 및 방향 계산
        dx = latest_data['x'] - start_data['x']
        dy = latest_data['y'] - start_data['y']
        total_distance = np.sqrt(dx**2 + dy**2)
        
        # 시간 차이 계산
        time_diff = (latest_data['timestamp'] - start_data['timestamp']).to_sec()
        
        if time_diff > 0:
            avg_speed = total_distance / time_diff
            
            # 최소 속도 및 거리 임계값 확인
            if avg_speed >= self.min_speed_threshold and total_distance >= 1.0:
                # 실제 이동 방향을 기반으로 heading 계산
                measured_heading = np.arctan2(dy, dx)
                
                # 로봇을 정지시키기
                self.stop_robot()
                
                # 초기 pose 설정
                self.set_initial_pose(latest_data['x'], latest_data['y'], 
                                    measured_heading, latest_data['covariance'])
                self.heading_initialized = True
                self.calibration_active = False
                
                rospy.loginfo("🎉 GPS 기반 Heading 캘리브레이션 완료!")
                rospy.loginfo(f"  📍 최종 위치: ({latest_data['x']:.2f}, {latest_data['y']:.2f})")
                rospy.loginfo(f"  🧭 측정된 Heading: {np.degrees(measured_heading):.1f}도")
                rospy.loginfo(f"  📏 총 이동거리: {total_distance:.2f}m")
                rospy.loginfo(f"  ⏱️  평균 속도: {avg_speed:.2f} m/s")
                
                self.publish_status("캘리브레이션 완료!")
                
    def stop_robot(self):
        """로봇 정지"""
        stop_cmd = Twist()
        for _ in range(5):  # 확실한 정지를 위해 여러 번 발송
            self.cmd_vel_pub.publish(stop_cmd)
            rospy.sleep(0.1)
    
    def set_initial_pose(self, x, y, heading, covariance):
        """초기 pose 설정"""
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.stamp = rospy.Time.now()
        initial_pose.header.frame_id = self.fixed_frame
        
        # 위치 설정
        initial_pose.pose.pose.position.x = x
        initial_pose.pose.pose.position.y = y
        initial_pose.pose.pose.position.z = 0.0
        
        # 방향 설정 (쿼터니언)
        q = quaternion_from_euler(0, 0, heading)
        initial_pose.pose.pose.orientation.x = q[0]
        initial_pose.pose.pose.orientation.y = q[1]
        initial_pose.pose.pose.orientation.z = q[2]
        initial_pose.pose.pose.orientation.w = q[3]
        
        # 공분산 설정
        initial_pose.pose.covariance = [0.0] * 36
        if len(covariance) >= 9:
            initial_pose.pose.covariance[0] = covariance[0]   # x-x
            initial_pose.pose.covariance[7] = covariance[4]   # y-y
            initial_pose.pose.covariance[35] = 0.05           # yaw-yaw (정밀한 값)
        else:
            # 기본 공분산 값
            initial_pose.pose.covariance[0] = 0.5   # x
            initial_pose.pose.covariance[7] = 0.5   # y
            initial_pose.pose.covariance[35] = 0.05 # yaw
        
        # 초기 pose 발행
        self.initial_pose_pub.publish(initial_pose)
        rospy.loginfo("📤 초기 pose가 /initialpose 토픽으로 발행되었습니다.")
    
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
            if not self.space_pressed and not self.heading_initialized:
                self.publish_status("Space바를 눌러 heading 캘리브레이션을 시작하세요.")
            elif self.calibration_active:
                self.publish_status("캘리브레이션 진행 중...")
            elif self.heading_initialized:
                self.publish_status("Heading 캘리브레이션 완료")
            
            rate.sleep()

if __name__ == '__main__':
    try:
        gps_heading_init = GPSHeadingInitializer()
        gps_heading_init.run()
    except rospy.ROSInterruptException:
        pass