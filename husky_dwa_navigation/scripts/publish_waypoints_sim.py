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
    
    - fused_odom을 통해 정확한 localization 정보 수신
    - UTM 좌표계에서 waypoint navigation 수행
    - GPS 검증을 통한 도달 확인
    """
    
    def __init__(self):
        rospy.init_node('waypoint_navigator', anonymous=True)
        
        # Waypoints 사전 정의 (UTM 절대 좌표)
        # citysim_gazebo.world 기준 UTM 좌표
        self.waypoints_utm = [
            {"x": 44, "y": 0},
            {"x": 44, "y": 45},
            {"x": -15, "y": 45},
            {"x": -15, "y": 0},
            {"x": -45, "y": 0},
            {"x": -72, "y": 0},
            {"x": -72, "y": -45},
            {"x": -45, "y": -45},
            {"x": -45, "y": 0},
            {"x": 0, "y": 0}
        ]
        
        # GPS 관련 변수 (검증용)
        self.utm_origin_set = False
        
        # 상태 변수
        self.current_waypoint_index = 0
        self.waypoint_reached_threshold = 2.0  # 2미터 이내 도달로 판단
        self.gps_verification_threshold = 3.0   # GPS 검증 기준 거리 (3미터)
        self.is_navigating = False
        self.goal_sent = False
        
        # 현재 위치 정보
        self.current_pose_utm = None
        self.current_gps = None
        self.last_good_gps = None
        
        # Publishers
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.waypoints_pub = rospy.Publisher('/waypoints', String, queue_size=1)
        self.status_pub = rospy.Publisher('/waypoint_navigator/status', String, queue_size=1)
        
        # Subscribers
        rospy.Subscriber("/fused_odom", PoseWithCovarianceStamped, self.pose_callback)
        rospy.Subscriber("/ublox/fix", NavSatFix, self.gps_callback)
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.move_base_status_callback)
        
        # Timer for waypoint management
        rospy.Timer(rospy.Duration(1.0), self.waypoint_manager)
        rospy.Timer(rospy.Duration(5.0), self.publish_waypoints_visualization)
        
        rospy.loginfo("🚀 Waypoint Navigator 시작!")
        rospy.loginfo(f"📍 총 {len(self.waypoints_utm)}개의 UTM waypoints 설정됨")
        rospy.loginfo("📡 Localization: /fused_odom 토픽 구독 (UTM 좌표)")
        
        # 시스템 초기화 대기 후 바로 네비게이션 시작
        rospy.sleep(2.0)
        self.start_navigation()
    
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
        return math.sqrt((pos1["x"] - pos2["x"])**2 + (pos1["y"] - pos2["y"])**2)
    
    def pose_callback(self, msg):
        """로봇 pose 업데이트 (path_visualizer.py에서 발행하는 UTM 좌표)"""
        self.current_pose_utm = {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "z": msg.pose.pose.position.z,
            "qx": msg.pose.pose.orientation.x,
            "qy": msg.pose.pose.orientation.y,
            "qz": msg.pose.pose.orientation.z,
            "qw": msg.pose.pose.orientation.w
        }
    
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
    
    def move_base_status_callback(self, msg):
        """move_base 상태 모니터링"""
        if not msg.status_list:
            return
            
        latest_status = msg.status_list[-1]
        
        # Goal reached (SUCCESS)
        if latest_status.status == 3 and self.goal_sent:
            rospy.loginfo("🎯 목적지 도달! GPS로 위치 검증 시작...")
            self.verify_waypoint_with_gps()
        
        # Goal failed (ABORTED, REJECTED)
        elif latest_status.status in [4, 5] and self.goal_sent:
            rospy.logwarn("❌ 목적지 도달 실패. 다시 시도...")
            rospy.sleep(2.0)
            self.send_current_waypoint()
    
    def start_navigation(self):
        """Navigation 시작"""
        if len(self.waypoints_utm) == 0:
            rospy.logwarn("❌ UTM Waypoints가 설정되지 않음!")
            return
        
        self.is_navigating = True
        self.current_waypoint_index = 0
        rospy.loginfo("🚀 UTM Waypoint Navigation 시작!")
        self.send_current_waypoint()
    
    def send_current_waypoint(self):
        """현재 waypoint를 move_base goal로 전송 (UTM 좌표)"""
        if self.current_waypoint_index >= len(self.waypoints_utm):
            rospy.loginfo("🏁 모든 waypoints 완주!")
            self.is_navigating = False
            return
        
        current_wp = self.waypoints_utm[self.current_waypoint_index]
        
        # PoseStamped 메시지 생성 (UTM frame 사용)
        goal = PoseStamped()
        goal.header.frame_id = "utm"  # UTM frame 사용
        goal.header.stamp = rospy.Time.now()
        
        # UTM 좌표 직접 사용
        goal.pose.position.x = current_wp["x"]
        goal.pose.position.y = current_wp["y"]
        goal.pose.position.z = 0.0
        
        # 방향은 다음 waypoint 방향으로 설정
        if self.current_waypoint_index < len(self.waypoints_utm) - 1:
            next_wp = self.waypoints_utm[self.current_waypoint_index + 1]
            dx = next_wp["x"] - current_wp["x"]
            dy = next_wp["y"] - current_wp["y"]
            yaw = math.atan2(dy, dx)
        else:
            yaw = 0.0  # 마지막 waypoint는 정북 방향
        
        # Quaternion 설정
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        
        # Goal 발행
        self.goal_pub.publish(goal)
        self.goal_sent = True
        
        rospy.loginfo(f"📍 Waypoint {self.current_waypoint_index + 1}/{len(self.waypoints_utm)} 전송: "
                      f"UTM({current_wp['x']:.1f}, {current_wp['y']:.1f})")
        
        # 상태 발행
        status_msg = {
            "current_waypoint": self.current_waypoint_index + 1,
            "total_waypoints": len(self.waypoints_utm),
            "target_utm": current_wp,
            "status": "navigating"
        }
        self.status_pub.publish(String(data=json.dumps(status_msg)))
    
    def verify_waypoint_with_gps(self):
        """GPS를 이용한 waypoint 도달 검증 (선택적)"""
        # GPS 데이터가 있으면 검증, 없으면 바로 다음 waypoint로 진행
        if self.last_good_gps is None or self.current_waypoint_index >= len(self.waypoints_utm):
            rospy.loginfo("⚠️ GPS 검증 생략 - 다음 waypoint로 진행...")
            self.move_to_next_waypoint()
            return
        
        current_wp_utm = self.waypoints_utm[self.current_waypoint_index]
        
        # GPS 위치와 waypoint 간 거리 계산 (UTM 기준)
        gps_distance = self.calculate_distance(self.last_good_gps, current_wp_utm)
        
        rospy.loginfo(f"📡 GPS 검증: 목적지까지 거리 {gps_distance:.2f}m")
        
        if gps_distance <= self.gps_verification_threshold:
            rospy.loginfo("✅ GPS 검증 성공! 다음 waypoint로 이동")
            self.move_to_next_waypoint()
        else:
            rospy.logwarn(f"❌ GPS 검증 실패 (거리: {gps_distance:.2f}m > {self.gps_verification_threshold}m)")
            rospy.loginfo("🔄 목적지 재전송...")
            rospy.sleep(2.0)
            self.send_current_waypoint()
    
    def move_to_next_waypoint(self):
        """다음 waypoint로 이동"""
        self.current_waypoint_index += 1
        self.goal_sent = False
        
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
            rospy.sleep(1.0)  # 잠깐 대기 후 다음 목적지 전송
            self.send_current_waypoint()
    
    def waypoint_manager(self, event):
        """Waypoint 상태 관리 (백업 시스템)"""
        if not self.is_navigating or self.current_pose_utm is None:
            return
        
        if self.current_waypoint_index >= len(self.waypoints_utm):
            return
        
        current_wp = self.waypoints_utm[self.current_waypoint_index]
        
        # UTM 좌표계에서 직접 거리 비교
        pose_distance = self.calculate_distance(self.current_pose_utm, current_wp)
        
        # waypoint 도달 확인 (백업)
        if pose_distance <= self.waypoint_reached_threshold and self.goal_sent:
            rospy.loginfo_throttle(5, f"📍 UTM 기준 waypoint 근접: {pose_distance:.2f}m")
    
    def publish_waypoints_visualization(self, event):
        """Waypoints 시각화를 위한 데이터 발행 (UTM 좌표 직접 발행)"""
        waypoints_data = {
            "waypoints": []
        }
        
        # UTM 좌표를 x, y 형태로 직접 발행
        for i, wp in enumerate(self.waypoints_utm):
            waypoints_data["waypoints"].append({
                "x": wp["x"],
                "y": wp["y"],
                "index": i,
                "completed": i < self.current_waypoint_index
            })
        
        self.waypoints_pub.publish(String(data=json.dumps(waypoints_data)))
        
        # 로그 (시각화 디버깅용)
        if len(waypoints_data["waypoints"]) > 0:
            first_wp = waypoints_data["waypoints"][0]
            rospy.loginfo_throttle(10, f"📍 UTM Waypoints 발행: {len(waypoints_data['waypoints'])}개, "
                                   f"첫 번째: ({first_wp['x']:.1f}, {first_wp['y']:.1f})")

if __name__ == '__main__':
    try:
        navigator = WaypointNavigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Waypoint Navigator 종료")