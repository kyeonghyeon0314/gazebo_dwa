#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import json
import utm
import math
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatusArray

class WaypointNavigator:
    """Localization 기반 Waypoint Navigation 노드
    
    - 다중 위치 소스를 통한 강건한 localization
    - UTM 좌표계에서 waypoint navigation 수행
    - SUCCESS 상태 디바운싱으로 중복 처리 방지
    """
    
    def __init__(self):
        rospy.init_node('waypoint_navigator', anonymous=True)
        
        # Waypoints 사전 정의 (UTM 절대 좌표)
        # simcity_gazebo.world 기준 UTM 좌표
        self.waypoints_utm = [
            {"x": 42, "y": 0},
            {"x": 44, "y": -45},
            {"x": -15, "y": -45},
            {"x": -45, "y": -45},
            {"x": -67, "y": -45},
            {"x": -72, "y": -22},
            {"x": -67, "y": 0},
            {"x": -45, "y": 0},
            {"x": -45, "y": -45},
            {"x": -45, "y": -92},
            {"x": -41, "y": -98},
            {"x": -15, "y": -100},
        ]
        
        # GPS 관련 변수 (검증용)
        self.utm_origin_set = False
        
        # 상태 변수
        self.current_waypoint_index = 0
        self.is_navigating = False
        
        # ✅ SUCCESS 상태 디바운싱을 위한 변수들
        self.current_goal_sent = False  # 현재 waypoint에 대한 goal 발행 여부
        self.waypoints_published = False  # waypoints 시각화 발행 여부
        self.last_success_time = rospy.Time(0)  # 마지막 SUCCESS 처리 시간
        self.success_debounce_duration = 3.0  # SUCCESS 디바운싱 시간 (3초)
        self.waypoint_reached_threshold = 10.0  # waypoint 도달 판정 거리 (10m)
        
        # ✅ 다중 위치 소스 관리
        self.current_pose_utm = None  # 주 위치 정보
        self.pose_source = "none"     # 현재 사용 중인 위치 소스
        self.pose_last_received = rospy.Time(0)  # 마지막 위치 정보 수신 시간
        self.pose_timeout = 5.0  # 위치 정보 타임아웃 (5초)
        
        # GPS 관련
        self.current_gps = None
        self.last_good_gps = None
        
        # Publishers
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.waypoints_pub = rospy.Publisher('/waypoints', String, queue_size=1)
        self.status_pub = rospy.Publisher('/waypoint_navigator/status', String, queue_size=1)
        
        # ✅ 다중 위치 소스 Subscribers
        rospy.Subscriber("/fused_odom", PoseWithCovarianceStamped, self.fused_odom_callback)
        rospy.Subscriber("/Odometry", Odometry, self.odometry_callback)  # 대안 위치 소스
        rospy.Subscriber("/robot_pose", PoseWithCovarianceStamped, self.robot_pose_callback)  # 추가 대안
        
        # 기타 Subscribers
        rospy.Subscriber("/ublox/fix", NavSatFix, self.gps_callback)
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.move_base_status_callback)
        
        # ✅ 상태 모니터링용 타이머들
        rospy.Timer(rospy.Duration(2.0), self.status_monitor)
        rospy.Timer(rospy.Duration(1.0), self.pose_health_check)  # 위치 정보 상태 체크
        
        rospy.loginfo("🚀 Waypoint Navigator 시작!")
        rospy.loginfo(f"📍 총 {len(self.waypoints_utm)}개의 UTM waypoints 설정됨")
        rospy.loginfo("📡 다중 Localization 소스: /fused_odom, /Odometry, /robot_pose")
        rospy.loginfo("✅ 오직 move_base SUCCESS 상태에서만 다음 waypoint로 이동")
        rospy.loginfo(f"⏱️  SUCCESS 디바운싱: {self.success_debounce_duration}초")
        
        # ✅ waypoints 시각화를 한번만 발행
        rospy.sleep(2.0)  # 시스템 초기화 대기
        self.publish_waypoints_visualization()
        
        # 위치 정보 대기 후 네비게이션 시작
        rospy.Timer(rospy.Duration(5.0), self.delayed_start)
    
    def delayed_start(self, event):
        """위치 정보 안정화 후 네비게이션 시작"""
        if self.current_pose_utm is not None:
            rospy.loginfo(f"✅ 위치 정보 안정화 완료. 네비게이션 시작! (소스: {self.pose_source})")
            self.start_navigation()
            event.shutdown()  # 타이머 중지
        else:
            rospy.logwarn("⚠️  위치 정보를 아직 받지 못했습니다. 5초 후 재시도...")
    
    def fused_odom_callback(self, msg):
        """주 위치 소스: /fused_odom"""
        self.update_pose_utm(msg.pose.pose, "fused_odom")
        rospy.logdebug("📍 /fused_odom에서 위치 정보 수신")
    
    def odometry_callback(self, msg):
        """대안 위치 소스: /Odometry"""
        if self.pose_source == "none" or self.is_pose_stale():
            self.update_pose_utm(msg.pose.pose, "Odometry")
            rospy.loginfo_throttle(10, "📍 /Odometry를 위치 소스로 사용 중")
    
    def robot_pose_callback(self, msg):
        """추가 대안 위치 소스: /robot_pose"""
        if self.pose_source == "none" or self.is_pose_stale():
            self.update_pose_utm(msg.pose.pose, "robot_pose")
            rospy.loginfo_throttle(10, "📍 /robot_pose를 위치 소스로 사용 중")
    
    def update_pose_utm(self, pose, source):
        """UTM 위치 정보 업데이트"""
        try:
            self.current_pose_utm = {
                "x": pose.position.x,
                "y": pose.position.y,
                "z": pose.position.z,
                "qx": pose.orientation.x,
                "qy": pose.orientation.y,
                "qz": pose.orientation.z,
                "qw": pose.orientation.w
            }
            self.pose_source = source
            self.pose_last_received = rospy.Time.now()
            
            # 처음 위치 정보를 받았을 때 로그
            if self.pose_source != source:
                rospy.loginfo(f"✅ 위치 소스 전환: {source}")
                rospy.loginfo(f"   현재 위치: ({pose.position.x:.2f}, {pose.position.y:.2f})")
                
        except Exception as e:
            rospy.logwarn(f"❌ 위치 정보 업데이트 실패 ({source}): {e}")
    
    def is_pose_stale(self):
        """위치 정보가 오래되었는지 확인"""
        if self.pose_last_received == rospy.Time(0):
            return True
        
        time_diff = (rospy.Time.now() - self.pose_last_received).to_sec()
        return time_diff > self.pose_timeout
    
    def pose_health_check(self, event):
        """위치 정보 상태 체크"""
        if self.current_pose_utm is None:
            rospy.logwarn_throttle(10, "⚠️  위치 정보를 받지 못했습니다!")
            rospy.logwarn_throttle(10, "   확인 사항:")
            rospy.logwarn_throttle(10, "   1. /fused_odom 토픽 상태: rostopic echo /fused_odom")
            rospy.logwarn_throttle(10, "   2. /Odometry 토픽 상태: rostopic echo /Odometry")
            rospy.logwarn_throttle(10, "   3. path_visualizer.py 실행 상태 확인")
        elif self.is_pose_stale():
            rospy.logwarn_throttle(10, f"⚠️  위치 정보가 {self.pose_timeout}초 이상 업데이트되지 않음 (소스: {self.pose_source})")
        else:
            rospy.loginfo_throttle(30, f"✅ 위치 정보 정상 (소스: {self.pose_source})")
    
    def gps_to_utm(self, lat, lon):
        """GPS 좌표를 UTM으로 변환 (GPS 검증용)"""
        try:
            if abs(lat) < 0.01 and abs(lon) < 0.01:
                utm_x = lon * 111320
                utm_y = lat * 111320
                return utm_x, utm_y, "52S"
            else:
                utm_x, utm_y, zone_number, zone_letter = utm.from_latlon(lat, lon)
                return utm_x, utm_y, f"{zone_number}{zone_letter}"
        except Exception as e:
            rospy.logwarn(f"❌ GPS->UTM 변환 실패: {e}")
            return 0.0, 0.0, "52S"
    
    def calculate_distance(self, pos1, pos2):
        """좌표간 거리 계산"""
        if pos1 is None or pos2 is None:
            return float('inf')
        
        try:
            distance = math.sqrt((pos1["x"] - pos2["x"])**2 + (pos1["y"] - pos2["y"])**2)
            return distance
        except Exception as e:
            rospy.logwarn(f"❌ 거리 계산 실패: {e}")
            return float('inf')
    
    def gps_callback(self, msg):
        """GPS 데이터 업데이트 (검증용)"""
        if msg.status.status >= 0:  # GPS 신호가 유효한 경우
            self.current_gps = {
                "lat": msg.latitude,
                "lon": msg.longitude,
                "alt": msg.altitude
            }
            
            # GPS 원점 설정 (처음 한 번만)
            if not self.utm_origin_set:
                self.utm_origin_set = True
                rospy.loginfo(f"🎯 GPS 원점 설정: ({msg.latitude:.6f}, {msg.longitude:.6f})")
            
            # UTM 변환 (검증용)
            utm_x, utm_y, _ = self.gps_to_utm(msg.latitude, msg.longitude)
            self.last_good_gps = {
                "x": utm_x,
                "y": utm_y,
                "lat": msg.latitude,
                "lon": msg.longitude
            }
    
    def is_waypoint_reached(self, waypoint):
        """현재 위치에서 waypoint 도달 여부 확인"""
        if self.current_pose_utm is None:
            rospy.logdebug("⚠️  current_pose_utm이 None입니다")
            return False
        
        distance = self.calculate_distance(self.current_pose_utm, waypoint)
        is_reached = distance <= self.waypoint_reached_threshold
        
        rospy.logdebug(f"📏 거리 확인: {distance:.2f}m, 도달 여부: {is_reached}")
        return is_reached
    
    def move_base_status_callback(self, msg):
        """move_base 상태 모니터링 - SUCCESS 디바운싱 적용"""
        if not msg.status_list:
            return
            
        latest_status = msg.status_list[-1]
        current_time = rospy.Time.now()
        
        # ✅ SUCCESS 상태 처리 (디바운싱 적용)
        if latest_status.status == 3 and self.current_goal_sent:
            # 마지막 SUCCESS 처리 후 충분한 시간이 지났는지 확인
            time_since_last_success = (current_time - self.last_success_time).to_sec()
            
            if time_since_last_success < self.success_debounce_duration:
                rospy.loginfo_throttle(5, f"⏳ SUCCESS 디바운싱: {self.success_debounce_duration - time_since_last_success:.1f}초 남음")
                return
            
            # ✅ 위치 정보 유효성 확인
            if self.current_pose_utm is None:
                rospy.logwarn("⚠️  SUCCESS 수신했지만 현재 위치 정보가 없음 - SUCCESS 무시")
                return
            
            # ✅ 추가 검증: 실제로 waypoint 근처에 있는지 확인
            if self.current_waypoint_index < len(self.waypoints_utm):
                current_wp = self.waypoints_utm[self.current_waypoint_index]
                
                if not self.is_waypoint_reached(current_wp):
                    distance = self.calculate_distance(self.current_pose_utm, current_wp)
                    rospy.logwarn(f"⚠️  SUCCESS 수신했지만 waypoint에서 {distance:.1f}m 떨어져 있음 (임계값: {self.waypoint_reached_threshold}m)")
                    rospy.logwarn(f"   현재 위치: ({self.current_pose_utm['x']:.2f}, {self.current_pose_utm['y']:.2f})")
                    rospy.logwarn(f"   목표 위치: ({current_wp['x']:.2f}, {current_wp['y']:.2f})")
                    rospy.logwarn("🔄 거리 검증 실패 - SUCCESS 무시")
                    return
            
            # ✅ 모든 검증 통과 - 다음 waypoint로 이동
            rospy.loginfo("🎯 move_base SUCCESS! 거리 검증 완료. 다음 waypoint로 이동...")
            self.last_success_time = current_time  # SUCCESS 처리 시간 기록
            self.move_to_next_waypoint()
        
        # ✅ Goal failed 시에는 아무것도 하지 않음 (재시도 없음, 다음 waypoint 이동 없음)
        elif latest_status.status in [4, 5] and self.current_goal_sent:
            rospy.logwarn("❌ move_base 실패. 대기 중... (다음 waypoint 이동 없음)")
            # 아무것도 하지 않음 - 사용자가 수동으로 처리하거나 다른 방법으로 해결
        
        # ✅ 기타 상태 로깅
        elif latest_status.status == 1 and self.current_goal_sent:
            rospy.loginfo_throttle(10, "🔄 move_base ACTIVE - 목적지로 이동 중...")
        elif latest_status.status == 0 and self.current_goal_sent:
            rospy.loginfo_throttle(10, "⏳ move_base PENDING - 목적지 대기 중...")
    
    def start_navigation(self):
        """Navigation 시작"""
        if len(self.waypoints_utm) == 0:
            rospy.logwarn("❌ UTM Waypoints가 설정되지 않음!")
            return
        
        if self.current_pose_utm is None:
            rospy.logwarn("❌ 위치 정보가 없어서 네비게이션을 시작할 수 없습니다!")
            return
        
        self.is_navigating = True
        self.current_waypoint_index = 0
        rospy.loginfo("🚀 UTM Waypoint Navigation 시작!")
        rospy.loginfo(f"   현재 위치: ({self.current_pose_utm['x']:.2f}, {self.current_pose_utm['y']:.2f})")
        rospy.loginfo(f"   위치 소스: {self.pose_source}")
        self.send_current_waypoint()
    
    def send_current_waypoint(self):
        """현재 waypoint를 move_base goal로 전송 (한번만 발행)"""
        if self.current_waypoint_index >= len(self.waypoints_utm):
            rospy.loginfo("🏁 모든 waypoints 완주!")
            self.is_navigating = False
            return
    
        # ✅ 이미 현재 waypoint에 대한 goal을 발행했다면 skip
        if self.current_goal_sent:
            rospy.loginfo_throttle(10, f"⏳ Waypoint {self.current_waypoint_index + 1} 이미 발행됨. move_base SUCCESS 대기 중...")
            return
    
        current_wp = self.waypoints_utm[self.current_waypoint_index]
    
        # ✅ 순수 UTM 절대좌표로 목표점 생성
        goal = PoseStamped()
        goal.header.frame_id = "utm"  # UTM 절대좌표계
        goal.header.stamp = rospy.Time(0)  # 최신 TF 사용
    
        # ✅ UTM 절대좌표 직접 사용 (변환 없음)
        goal.pose.position.x = float(current_wp["x"])
        goal.pose.position.y = float(current_wp["y"])
        goal.pose.position.z = 0.0
    
        # ✅ 방향은 UTM 좌표계 기준으로 계산
        if self.current_waypoint_index < len(self.waypoints_utm) - 1:
            next_wp = self.waypoints_utm[self.current_waypoint_index + 1]
            dx = next_wp["x"] - current_wp["x"]
            dy = next_wp["y"] - current_wp["y"]
            yaw = math.atan2(dy, dx)
        else:
            yaw = 0.0  # 북향
    
        # ✅ UTM 좌표계 기준 방향 설정
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
    
        # ✅ Goal 발행 (한번만)
        self.goal_pub.publish(goal)
        self.current_goal_sent = True  # 현재 waypoint goal 발행 완료 표시
    
        # ✅ 절대좌표 보장을 위한 로깅
        rospy.loginfo(f"📍 UTM 절대좌표 Goal 전송 (한번만):")
        rospy.loginfo(f"   Waypoint: {self.current_waypoint_index + 1}/{len(self.waypoints_utm)}")
        rospy.loginfo(f"   Frame: {goal.header.frame_id}")
        rospy.loginfo(f"   Position: ({goal.pose.position.x:.1f}, {goal.pose.position.y:.1f})")
        rospy.loginfo(f"   Orientation: yaw={math.degrees(yaw):.1f}°")
        rospy.loginfo(f"   현재 위치: ({self.current_pose_utm['x']:.2f}, {self.current_pose_utm['y']:.2f})")
        rospy.loginfo(f"   ✅ move_base SUCCESS 상태에서만 다음 waypoint로 이동")
        rospy.loginfo(f"   🛡️  SUCCESS 디바운싱: {self.success_debounce_duration}초")
        rospy.loginfo(f"   📏 도달 임계값: {self.waypoint_reached_threshold}m")
    
        # ✅ 상태 발행 (절대좌표 정보 포함)
        status_msg = {
            "current_waypoint": self.current_waypoint_index + 1,
            "total_waypoints": len(self.waypoints_utm),
            "target_utm_absolute": {
                "x": float(current_wp["x"]),
                "y": float(current_wp["y"]),
                "frame": "utm"
            },
            "current_pose": {
                "x": self.current_pose_utm["x"] if self.current_pose_utm else None,
                "y": self.current_pose_utm["y"] if self.current_pose_utm else None,
                "source": self.pose_source
            },
            "status": "navigating",
            "goal_sent_once": True,
            "strict_mode": True,  # SUCCESS에서만 다음 waypoint 이동
            "success_debounce_enabled": True,
            "success_debounce_duration": self.success_debounce_duration
        }
        self.status_pub.publish(String(data=json.dumps(status_msg)))
    
    def move_to_next_waypoint(self):
        """다음 waypoint로 이동 - 오직 move_base SUCCESS+거리 검증에서만 호출됨"""
        rospy.loginfo(f"✅ Waypoint {self.current_waypoint_index + 1} 완료!")
        
        self.current_waypoint_index += 1
        self.current_goal_sent = False  # ✅ 새 waypoint를 위해 goal 발행 플래그 리셋
        
        if self.current_waypoint_index >= len(self.waypoints_utm):
            rospy.loginfo("🏁 모든 waypoints 완주!")
            self.is_navigating = False
            
            # 완주 상태 발행
            status_msg = {
                "current_waypoint": len(self.waypoints_utm),
                "total_waypoints": len(self.waypoints_utm),
                "status": "completed"
            }
            self.status_pub.publish(String(data=json.dumps(status_msg)))
        else:
            rospy.loginfo(f"➡️ 다음 waypoint로 이동: {self.current_waypoint_index + 1}/{len(self.waypoints_utm)}")
            # ✅ 다음 waypoint로 이동하기 전 충분한 대기 시간
            rospy.sleep(2.0)  # 시스템이 안정화될 시간 제공
            self.send_current_waypoint()  # ✅ 새 waypoint goal 발행 (한번만)
    
    def status_monitor(self, event):
        """상태 모니터링"""
        if not self.is_navigating:
            return
        
        if self.current_pose_utm is None:
            rospy.logwarn_throttle(10, "⚠️  위치 정보 없음 - 네비게이션 대기 중...")
            return
        
        if self.current_waypoint_index >= len(self.waypoints_utm):
            return
        
        current_wp = self.waypoints_utm[self.current_waypoint_index]
        
        # ✅ 단순 상태 모니터링만 (다음 waypoint 이동 없음)
        if self.current_goal_sent:
            pose_distance = self.calculate_distance(self.current_pose_utm, current_wp)
            time_since_last_success = (rospy.Time.now() - self.last_success_time).to_sec()
            
            rospy.loginfo_throttle(10, f"📍 현재 상태:")
            rospy.loginfo_throttle(10, f"   목표: Waypoint {self.current_waypoint_index + 1}/{len(self.waypoints_utm)}")
            rospy.loginfo_throttle(10, f"   거리: {pose_distance:.2f}m (임계값: {self.waypoint_reached_threshold}m)")
            rospy.loginfo_throttle(10, f"   현재 위치: ({self.current_pose_utm['x']:.2f}, {self.current_pose_utm['y']:.2f})")
            rospy.loginfo_throttle(10, f"   목표 위치: ({current_wp['x']:.2f}, {current_wp['y']:.2f})")
            rospy.loginfo_throttle(10, f"   위치 소스: {self.pose_source}")
            rospy.loginfo_throttle(10, f"   대기: move_base SUCCESS 상태")
            rospy.loginfo_throttle(10, f"   디바운싱: 마지막 SUCCESS 후 {time_since_last_success:.1f}초 경과")
    
    def publish_waypoints_visualization(self):
        """Waypoints 시각화를 위한 데이터 발행 (한번만)"""
        if self.waypoints_published:
            return  # ✅ 이미 발행했으면 skip
            
        waypoints_data = {
            "frame": "utm",  # 절대좌표계 명시
            "coordinate_type": "absolute_utm",
            "waypoints": []
        }
    
        # ✅ UTM 절대좌표를 x, y 형태로 직접 발행
        for i, wp in enumerate(self.waypoints_utm):
            waypoints_data["waypoints"].append({
                "index": i,
                "x": float(wp["x"]),  # UTM 절대좌표
                "y": float(wp["y"]),  # UTM 절대좌표
                "completed": False,  # 초기에는 모두 미완료
                "is_current": i == 0  # 첫번째가 현재 목표
            })
    
        self.waypoints_pub.publish(String(data=json.dumps(waypoints_data)))
        self.waypoints_published = True  # ✅ 발행 완료 표시
    
        # ✅ 디버깅 로그
        rospy.loginfo(f"📍 UTM 절대좌표 Waypoints 발행 완료 (한번만): {len(waypoints_data['waypoints'])}개")
        rospy.loginfo(f"   좌표계: {waypoints_data['frame']} (절대좌표)")
        rospy.loginfo(f"   엄격 모드: 오직 move_base SUCCESS에서만 다음 waypoint 이동")
        rospy.loginfo(f"   안전 모드: SUCCESS 디바운싱 + 거리 검증 + 다중 위치 소스")

if __name__ == '__main__':
    try:
        navigator = WaypointNavigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Waypoint Navigator 종료")