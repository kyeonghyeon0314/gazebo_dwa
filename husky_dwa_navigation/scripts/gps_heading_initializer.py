#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
GPS 기반 초기 Heading 설정 노드 (Python 3.8 호환 버전)
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

        self.print_initialization_info()
        
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
        self.gps_sub = rospy.Subscriber('/gps/fix', NavSatFix, self.gps_callback)
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
            except Exception:
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
        rospy.loginfo("🚀 캘리브레이션 시작!")
        
        # 현재 로봇 위치 가져오기
        try:
            # TF가 사용 가능할 때까지 대기
            if self.tf_buffer.can_transform(
                self.fixed_frame, self.robot_frame, 
                rospy.Time(), rospy.Duration(5.0)):
                
                transform = self.tf_buffer.lookup_transform(
                    self.fixed_frame, self.robot_frame, 
                    rospy.Time(), rospy.Duration(1.0))
                
                current_x = transform.transform.translation.x
                current_y = transform.transform.translation.y
                
                rospy.loginfo(f"🎯 현재 위치: ({current_x:.2f}, {current_y:.2f})")
                
            else:
                raise Exception("TF not available")
                
        except Exception as e:
            rospy.logwarn(f"TF 조회 실패, GPS 위치 사용: {str(e)}")
            if len(self.gps_data_buffer) > 0:
                latest_gps = self.gps_data_buffer[-1]
                current_x = latest_gps['x']
                current_y = latest_gps['y']
                rospy.loginfo(f"🎯 GPS 기반 위치: ({current_x:.2f}, {current_y:.2f})")
            else:
                rospy.logerr("❌ 위치 정보를 가져올 수 없습니다.")
                return
        
        # 전방 목표점 설정
        forward_x = current_x + self.forward_distance * np.cos(0)  # 초기 방향 추정
        forward_y = current_y + self.forward_distance * np.sin(0)
        
        rospy.loginfo(f"🎯 전방 목표점: ({forward_x:.2f}, {forward_y:.2f})")
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

    def print_initialization_info(self):
        """초기화 정보를 깔끔하게 한 번에 출력"""
        info_msg = f"""
{'='*60}
GPS Heading Initializer 시작 (Python 3.8)
{'='*60}
설정 정보:
- 고정 프레임: {self.fixed_frame}
- 로봇 프레임: {self.robot_frame}
- 전방 목표 거리: {self.forward_distance}m
- 최소 이동 속도: {self.min_speed_threshold} m/s
- 샘플 수: {self.heading_samples}개

Space바를 눌러 초기 heading 캘리브레이션을 시작하세요.
{'='*60}
"""
        rospy.loginfo(info_msg)


        
        
    def gps_callback(self, gps_msg):
        """GPS 데이터 콜백"""
        try:
            if gps_msg.status.status >= 0:
                utm_point = utm.fromLatLong(gps_msg.latitude, gps_msg.longitude)
                
                if not self.initial_position_set:
                    self.utm_origin = utm_point
                    self.initial_position_set = True
                    
                    # 개선: 원점 설정 정보를 깔끔하게 출력
                    rospy.loginfo(f"📍 UTM 원점 설정 완료")
                    rospy.loginfo(f"   - UTM Zone: {utm_point.zone}{utm_point.band}")
                    rospy.loginfo(f"   - 좌표: ({utm_point.easting:.2f}, {utm_point.northing:.2f})")
                    rospy.loginfo(f"   - 캘리브레이션 준비 완료")
                
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
            rospy.logerr(f"GPS 콜백 처리 오류: {e}")
    
    def calculate_and_set_heading(self):
        """GPS 이동 데이터를 통해 heading 계산 및 설정"""
        if len(self.gps_data_buffer) < 5:  # 최소 5개 샘플 필요
            return
            
        # 이동 거리 계산
        recent_data = list(self.gps_data_buffer)[-5:]  # 최근 5개 데이터
        start_pos = recent_data[0]
        end_pos = recent_data[-1]
        
        dx = end_pos['x'] - start_pos['x']
        dy = end_pos['y'] - start_pos['y']
        distance = np.sqrt(dx*dx + dy*dy)
        
        # 충분히 이동했는지 확인
        if distance < self.min_speed_threshold * 2:  # 2초간 최소 이동 거리
            return
        
        # Heading 계산
        calculated_heading = np.arctan2(dy, dx)
        
        # Initial pose 설정
        self.set_initial_pose_with_heading(calculated_heading)
        
        rospy.loginfo(f"🧭 GPS 기반 Heading 계산 완료: {np.degrees(calculated_heading):.1f}도")
        rospy.loginfo(f"📏 이동 거리: {distance:.2f}m")
        
        self.heading_initialized = True
        self.calibration_active = False
        self.publish_status(f"Heading 캘리브레이션 완료: {np.degrees(calculated_heading):.1f}도")
    
    def set_initial_pose_with_heading(self, heading):
        """계산된 heading으로 initial pose 설정"""
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.stamp = rospy.Time.now()
        initial_pose.header.frame_id = self.fixed_frame
        
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
        quat = quaternion_from_euler(0, 0, heading)
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
        
        self.initial_pose_pub.publish(initial_pose)
        rospy.loginfo("📍 Initial pose with GPS heading published")
    
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