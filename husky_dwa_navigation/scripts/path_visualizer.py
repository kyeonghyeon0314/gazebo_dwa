#!/usr/bin/env python3

import rospy
import json
import utm
import math
import numpy as np
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix
import tf2_ros
from geometry_msgs.msg import TransformStamped
import tf.transformations as tf_trans

class UTMHeadingCorrection:
    """UTM 기반 점진적 Heading 보정 Localizer - UTM 절대좌표 통일"""
    
    def __init__(self):
        rospy.set_param('/use_sim_time', True)
        rospy.init_node('utm_heading_correction', anonymous=True)

        # 🎯 UTM 좌표계 설정 (GPS_FIRST 전략)
        self.utm_origin = None              # 첫 GPS로 설정될 UTM 원점
        self.utm_zone = None                # UTM 존
        self.first_gps_received = False     # 첫 GPS 수신 여부
        self.last_good_gps = None
        
        # FasterLIO 기준점
        self.fasterlio_origin = None
        
        # 🔥 점진적 Heading 보정 시스템
        self.correction_system = {
            "heading_correction": 0.0,         # 현재 적용 중인 heading 보정
            "initial_alignment_done": False,   # 초기 정렬 완료 여부
            "last_correction_time": 0.0,       # 마지막 보정 시간
        }
        
        # 궤적 기록 (모두 UTM 절대좌표)
        self.fasterlio_trajectory_utm = []     # FasterLIO → UTM 변환
        self.gps_trajectory_utm = []           # GPS → UTM
        self.corrected_trajectory_utm = []     # Heading 보정된 FasterLIO
        self.latest_waypoints = None
        
        # 🎯 현재 위치 (UTM 좌표)
        self.current_pose_utm = None
        self.pose_covariance = np.eye(6) * 0.1
        
        # 거리 추적
        self.total_distance = 0.0
        self.last_position = None

        # ✅ UTM 절대좌표 Publishers (통일)
        self.pose_pub = rospy.Publisher("/robot_pose", PoseWithCovarianceStamped, queue_size=1)
        self.odom_pub = rospy.Publisher("/fused_odom", Odometry, queue_size=1)
        
        # ✅ UTM 절대좌표 시각화 Publishers
        self.fasterlio_path_pub = rospy.Publisher("/fasterlio_path", Marker, queue_size=10)
        self.gps_path_pub = rospy.Publisher("/gps_path", Marker, queue_size=10)
        self.corrected_path_pub = rospy.Publisher("/corrected_path", Marker, queue_size=10)
        self.uncertainty_pub = rospy.Publisher("/pose_uncertainty", Marker, queue_size=10)
        self.waypoints_pub = rospy.Publisher("/global_waypoints", MarkerArray, queue_size=10)

        #data analysis
        self.raw_analysis_pub = rospy.Publisher("/analysis/raw_fasterlio", Odometry, queue_size=1)
        self.initial_analysis_pub = rospy.Publisher("/analysis/initial_corrected", Odometry, queue_size=1)
        self.full_analysis_pub = rospy.Publisher("/analysis/fully_corrected", Odometry, queue_size=1)

        self.initial_alignment_completed = False

        # ✅ 공통 Publishers
        self.gps_data_pub = rospy.Publisher("/gps_data", String, queue_size=10)
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Subscribers
        rospy.Subscriber("/Odometry", Odometry, self.fasterlio_callback)
        rospy.Subscriber("/ublox/fix", NavSatFix, self.gps_callback)
        rospy.Subscriber("/waypoints", String, self.waypoints_callback)
        
        # Timers
        rospy.Timer(rospy.Duration(0.1), self.publish_current_pose)
        rospy.Timer(rospy.Duration(0.5), self.publish_visualization)
        rospy.Timer(rospy.Duration(0.1), self.broadcast_tf)
        rospy.Timer(rospy.Duration(1.0), self.publish_gps_data)
        rospy.Timer(rospy.Duration(10.0), self.check_gradual_heading_correction)
        
        rospy.loginfo("🚀 UTM 기반 Heading 보정 Localizer 시작!")
        rospy.loginfo("📍 GPS_FIRST 전략: 첫 실시간 GPS 수신 시 UTM 원점 설정")
        rospy.loginfo("🌍 모든 좌표계 UTM 절대좌표로 통일!")

    
    def setup_utm_origin_from_gps(self, lat, lon):
        """GPS_FIRST 전략: 첫 GPS를 UTM 원점으로 설정"""
        if not self.first_gps_received:  # 🔥 첫 GPS만 설정
            easting, northing, zone_num, zone_letter = utm.from_latlon(lat, lon)
            
            self.utm_origin = {
                "easting": easting,
                "northing": northing,
                "lat": lat,
                "lon": lon
            }
            self.utm_zone = f"{zone_num}{zone_letter}"
            self.first_gps_received = True
            
            rospy.loginfo(f"🎯 UTM 원점 설정 완료!")
            rospy.loginfo(f"   GPS: ({lat:.6f}, {lon:.6f})")
            rospy.loginfo(f"   UTM: ({easting:.1f}, {northing:.1f})")
            rospy.loginfo(f"   Zone: {self.utm_zone}")
            
            return True
        return False
    
    def gps_to_utm(self, lat, lon):
        """GPS를 UTM 절대좌표로 변환"""
        easting, northing, zone_num, zone_letter = utm.from_latlon(lat, lon)
        return easting, northing, f"{zone_num}{zone_letter}"
    
    def fasterlio_to_utm(self, fasterlio_x, fasterlio_y):
        """FasterLIO 좌표를 UTM 절대좌표로 변환"""
        if not self.utm_origin or not self.fasterlio_origin:
            return fasterlio_x, fasterlio_y
        
        # 1단계: FasterLIO 원점 기준 상대좌표
        rel_x = fasterlio_x - self.fasterlio_origin["x"]
        rel_y = fasterlio_y - self.fasterlio_origin["y"]
        
        # 2단계: Heading 보정 적용 (UTM 원점 기준)
        if self.correction_system["initial_alignment_done"]:
            corrected_x, corrected_y = self.rotate_point_around_origin(
                rel_x, rel_y, self.correction_system["heading_correction"]
            )
        else:
            corrected_x, corrected_y = rel_x, rel_y
        
        # 3단계: UTM 절대좌표로 변환
        utm_x = np.float64(corrected_x) + np.float64(self.utm_origin["easting"])
        utm_y = np.float64(corrected_y) + np.float64(self.utm_origin["northing"])
        
        return utm_x, utm_y
    
    def rotate_point_around_origin(self, x, y, angle, origin_x=0.0, origin_y=0.0):
        """원점 기준으로 점 회전"""
        rel_x = x - origin_x
        rel_y = y - origin_y
        
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        
        rotated_x = rel_x * cos_a - rel_y * sin_a
        rotated_y = rel_x * sin_a + rel_y * cos_a
        
        final_x = rotated_x + origin_x
        final_y = rotated_y + origin_y
        
        return final_x, final_y
    
    def calculate_trajectory_heading(self, trajectory, min_distance=2.0):
        """궤적에서 heading 계산"""
        if len(trajectory) < 2:
            return None
        
        max_distance = 0
        best_heading = None
        
        for i in range(len(trajectory)):
            for j in range(i + 1, len(trajectory)):
                p1 = trajectory[i]
                p2 = trajectory[j]
                
                if "x" not in p1 or "y" not in p1 or "x" not in p2 or "y" not in p2:
                    continue
                
                distance = math.sqrt((p2["x"] - p1["x"])**2 + (p2["y"] - p1["y"])**2)
                
                if distance > max_distance and distance >= min_distance:
                    max_distance = distance
                    best_heading = math.atan2(p2["y"] - p1["y"], p2["x"] - p1["x"])
        
        if best_heading is not None:
            rospy.loginfo(f"✅ Heading 계산: {math.degrees(best_heading):.1f}도 (거리: {max_distance:.1f}m)")
        
        return best_heading
    
    def perform_initial_heading_alignment(self):
        """🎯 초기 Heading 정렬 (1회만)"""
        if len(self.fasterlio_trajectory_utm) < 3 or len(self.gps_trajectory_utm) < 3:
            rospy.logwarn("❌ 초기 Heading 정렬용 궤적 데이터 부족")
            return False
        
        # FasterLIO 방향 계산 (UTM 변환된)
        fasterlio_heading = self.calculate_trajectory_heading(self.fasterlio_trajectory_utm, 1.0)
        if fasterlio_heading is None:
            return False
        
        # GPS 방향 계산 (UTM)
        gps_heading = self.calculate_trajectory_heading(self.gps_trajectory_utm, 1.0)
        if gps_heading is None:
            return False
        
        # 회전각 계산
        angle_diff = gps_heading - fasterlio_heading
        
        # 각도 정규화
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # 초기 Heading 보정 설정
        self.correction_system["heading_correction"] = angle_diff
        self.correction_system["initial_alignment_done"] = True
        
        rospy.loginfo(f"🎯 초기 Heading 정렬 완료!")
        rospy.loginfo(f"   FasterLIO 방향: {math.degrees(fasterlio_heading):.1f}도")
        rospy.loginfo(f"   GPS 방향: {math.degrees(gps_heading):.1f}도")
        rospy.loginfo(f"   초기 회전 보정: {math.degrees(angle_diff):.1f}도")
        
        # 전체 궤적 재계산
        self.recalculate_all_trajectories()
        
        # 🔥 Initial Corrected 상태 플래그 설정
        self.initial_alignment_completed = True
        
        # 🔥 현재 보정된 위치를 Initial Corrected로 발행
        self.publish_initial_corrected_pose()
        return True
    
    def publish_initial_corrected_pose(self):
        """Initial Heading Correction 결과 발행"""
        if not self.current_pose_utm:
            return
            
        initial_odom = Odometry()
        initial_odom.header.stamp = rospy.Time.now()
        initial_odom.header.frame_id = "utm"
        initial_odom.child_frame_id = "base_link"
        
        initial_odom.pose.pose.position.x = self.current_pose_utm["x"]
        initial_odom.pose.pose.position.y = self.current_pose_utm["y"]
        initial_odom.pose.pose.position.z = self.current_pose_utm["z"]
        initial_odom.pose.pose.orientation.x = self.current_pose_utm["qx"]
        initial_odom.pose.pose.orientation.y = self.current_pose_utm["qy"]
        initial_odom.pose.pose.orientation.z = self.current_pose_utm["qz"]
        initial_odom.pose.pose.orientation.w = self.current_pose_utm["qw"]
        
        self.initial_analysis_pub.publish(initial_odom)
    
    def perform_gradual_heading_correction(self):
        """🔥 점진적 Heading 보정"""
        if len(self.corrected_trajectory_utm) < 10 or len(self.gps_trajectory_utm) < 10:
            return False
        
        # 전체 방향 계산
        corrected_start = self.corrected_trajectory_utm[0]
        corrected_end = self.corrected_trajectory_utm[-1]
        
        gps_start = self.gps_trajectory_utm[0]
        gps_end = self.gps_trajectory_utm[-1]
        
        corrected_dx = corrected_end["x"] - corrected_start["x"]
        corrected_dy = corrected_end["y"] - corrected_start["y"]
        corrected_distance = math.sqrt(corrected_dx**2 + corrected_dy**2)
        
        gps_dx = gps_end["x"] - gps_start["x"]
        gps_dy = gps_end["y"] - gps_start["y"]
        gps_distance = math.sqrt(gps_dx**2 + gps_dy**2)
        
        if corrected_distance < 10.0 or gps_distance < 10.0:
            return False
        
        corrected_heading = math.atan2(corrected_dy, corrected_dx)
        gps_heading = math.atan2(gps_dy, gps_dx)
        
        angle_diff = gps_heading - corrected_heading
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        if abs(angle_diff) < math.radians(1.0):
            return False
        
        # 점진적 보정 (10%씩)
        correction_ratio = 0.1
        additional_correction = angle_diff * correction_ratio
        
        old_correction = self.correction_system["heading_correction"]
        self.correction_system["heading_correction"] += additional_correction
        
        rospy.loginfo(f"🔄 점진적 Heading 보정:")
        rospy.loginfo(f"   전체 각도 차이: {math.degrees(angle_diff):.1f}도")
        rospy.loginfo(f"   추가 보정: {math.degrees(additional_correction):.1f}도")
        rospy.loginfo(f"   총 보정: {math.degrees(old_correction):.1f}→{math.degrees(self.correction_system['heading_correction']):.1f}도")
        
        self.recalculate_all_trajectories()
        return True
    
    def check_gradual_heading_correction(self, event):
        """점진적 Heading 보정 체크 (10초마다)"""
        if not self.correction_system["initial_alignment_done"]:
            return
        
        current_time = rospy.Time.now().to_sec()
        time_since_last = current_time - self.correction_system.get("last_correction_time", 0)
        
        if time_since_last > 10.0:
            if self.perform_gradual_heading_correction():
                self.correction_system["last_correction_time"] = current_time
    
    def recalculate_all_trajectories(self):
        """전체 FasterLIO 궤적을 UTM으로 재계산"""
        if not self.fasterlio_trajectory_utm:
            return
        
        # 보정된 궤적 재계산
        self.corrected_trajectory_utm = []
        
        for fasterlio_point in self.fasterlio_trajectory_utm:
            # 원본 FasterLIO 좌표 (UTM 변환 전) 복원
            if not self.fasterlio_origin:
                continue
                
            # UTM에서 FasterLIO 원본으로 역변환
            rel_x = fasterlio_point["x"] - self.utm_origin["easting"]
            rel_y = fasterlio_point["y"] - self.utm_origin["northing"]
            original_x = rel_x + self.fasterlio_origin["x"]
            original_y = rel_y + self.fasterlio_origin["y"]
            
            # 다시 보정 적용하여 UTM으로 변환
            corrected_utm_x, corrected_utm_y = self.fasterlio_to_utm(original_x, original_y)
            
            corrected_point = fasterlio_point.copy()
            corrected_point["x"] = corrected_utm_x
            corrected_point["y"] = corrected_utm_y
            
            self.corrected_trajectory_utm.append(corrected_point)
        
        rospy.loginfo(f"✅ {len(self.corrected_trajectory_utm)}개 포인트 재계산 완료")
    
    def calculate_endpoint_distance_error(self):
        """현재 끝점과 GPS 끝점 사이의 거리 오차 계산"""
        if not self.corrected_trajectory_utm or not self.gps_trajectory_utm:
            return None
        
        corrected_end = self.corrected_trajectory_utm[-1]
        gps_end = self.gps_trajectory_utm[-1]
        
        dx = corrected_end["x"] - gps_end["x"]
        dy = corrected_end["y"] - gps_end["y"]
        distance_error = math.sqrt(dx*dx + dy*dy)
        
        return distance_error
    
    def update_distance(self, new_position):
        """이동 거리 업데이트"""
        if self.last_position is not None:
            dx = new_position["x"] - self.last_position["x"]
            dy = new_position["y"] - self.last_position["y"]
            distance = math.sqrt(dx*dx + dy*dy)
            self.total_distance += distance
        
        self.last_position = new_position.copy()
    
    def fasterlio_callback(self, msg):
        """🎯 FasterLIO 메인 콜백 - UTM 변환"""
        timestamp = msg.header.stamp.to_sec()
        
        # FasterLIO 원시 pose
        fasterlio_pose = {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "z": msg.pose.pose.position.z,
            "qx": msg.pose.pose.orientation.x,
            "qy": msg.pose.pose.orientation.y,
            "qz": msg.pose.pose.orientation.z,
            "qw": msg.pose.pose.orientation.w,
            "timestamp": timestamp
        }
        
        # 첫 번째 포즈면 기준점 설정
        if self.fasterlio_origin is None:
            self.fasterlio_origin = fasterlio_pose.copy()
            rospy.loginfo("🎯 FasterLIO 기준점 설정 완료")
        
        # UTM으로 변환 (보정 없이)
        if self.utm_origin:
            rel_x = np.float64(fasterlio_pose["x"]) - np.float64(self.fasterlio_origin["x"])
            rel_y = np.float64(fasterlio_pose["y"]) - np.float64(self.fasterlio_origin["y"])
            utm_x = np.float64(rel_x) + np.float64(self.utm_origin["easting"])
            utm_y = np.float64(rel_y) + np.float64(self.utm_origin["northing"])
            
            utm_point = {
                "x": utm_x,
                "y": utm_y,
                "z": fasterlio_pose["z"],
                "qx": fasterlio_pose["qx"],
                "qy": fasterlio_pose["qy"],
                "qz": fasterlio_pose["qz"],
                "qw": fasterlio_pose["qw"],
                "timestamp": timestamp
            }
            
            # FasterLIO 궤적 기록 (UTM)
            if not self.fasterlio_trajectory_utm or self.distance_check_utm(utm_point, self.fasterlio_trajectory_utm[-1], 0.5):
                self.fasterlio_trajectory_utm.append(utm_point.copy())
        
        # Heading 보정 적용하여 UTM 변환
        corrected_utm_x, corrected_utm_y = self.fasterlio_to_utm(fasterlio_pose["x"], fasterlio_pose["y"])
        
        # Orientation에도 heading 보정 적용
        corrected_qx, corrected_qy, corrected_qz, corrected_qw = self.apply_heading_correction_to_orientation(
            fasterlio_pose["qx"], fasterlio_pose["qy"], fasterlio_pose["qz"], fasterlio_pose["qw"]
        )
        
        # 현재 위치 업데이트 (UTM 좌표)
        self.current_pose_utm = {
            "x": corrected_utm_x,
            "y": corrected_utm_y,
            "z": fasterlio_pose["z"],
            "qx": corrected_qx,
            "qy": corrected_qy,
            "qz": corrected_qz,
            "qw": corrected_qw,
            "timestamp": timestamp
        }
        
        # 거리 업데이트
        self.update_distance(self.current_pose_utm)
        
        # 보정된 궤적 기록 (UTM)
        if self.utm_origin:
            if not self.corrected_trajectory_utm or self.distance_check_utm(self.current_pose_utm, self.corrected_trajectory_utm[-1], 0.5):
                self.corrected_trajectory_utm.append(self.current_pose_utm.copy())
        
        # 초기 Heading 정렬 체크 (2m 이동 후)
        if not self.correction_system["initial_alignment_done"] and self.total_distance >= 2.0:
            rospy.loginfo(f"📏 총 이동거리 {self.total_distance:.1f}m → 초기 Heading 정렬 수행")
            self.perform_initial_heading_alignment()
        
        # 불확실성 업데이트
        if self.correction_system["initial_alignment_done"]:
            uncertainty = 2.0
        else:
            uncertainty = 10.0
        
        self.pose_covariance[0,0] = uncertainty
        self.pose_covariance[1,1] = uncertainty
        
        # 끝점 거리 오차 표시
        distance_error = self.calculate_endpoint_distance_error()
        if distance_error is not None:
            rospy.loginfo_throttle(2, f"🎯 UTM 위치: ({corrected_utm_x:.1f}, {corrected_utm_y:.1f}), 끝점오차: {distance_error:.1f}m, 총거리: {self.total_distance:.1f}m")
        else:
            rospy.loginfo_throttle(2, f"🎯 UTM 위치: ({corrected_utm_x:.1f}, {corrected_utm_y:.1f}), 총거리: {self.total_distance:.1f}m")

        raw_odom = Odometry()      
        raw_odom.header.stamp = rospy.Time.now()
        raw_odom.header.frame_id = "utm"
        raw_odom.child_frame_id = "base_link"
        
        # UTM 변환된 Raw 좌표 발행
        if self.utm_origin and self.fasterlio_origin:
            rel_x = np.float64(fasterlio_pose["x"]) - np.float64(self.fasterlio_origin["x"])
            rel_y = np.float64(fasterlio_pose["y"]) - np.float64(self.fasterlio_origin["y"])
            utm_x = np.float64(rel_x) + np.float64(self.utm_origin["easting"])
            utm_y = np.float64(rel_y) + np.float64(self.utm_origin["northing"])
            
            raw_odom.pose.pose.position.x = utm_x
            raw_odom.pose.pose.position.y = utm_y
            raw_odom.pose.pose.position.z = fasterlio_pose["z"]
            raw_odom.pose.pose.orientation.x = fasterlio_pose["qx"]
            raw_odom.pose.pose.orientation.y = fasterlio_pose["qy"]
            raw_odom.pose.pose.orientation.z = fasterlio_pose["qz"]
            raw_odom.pose.pose.orientation.w = fasterlio_pose["qw"]
            
            self.raw_analysis_pub.publish(raw_odom)

    def apply_heading_correction_to_orientation(self, qx, qy, qz, qw):
        """Orientation에도 heading 보정 적용"""
        if not self.correction_system["initial_alignment_done"]:
            return qx, qy, qz, qw
        
        # Quaternion → Euler
        euler = tf_trans.euler_from_quaternion([qx, qy, qz, qw])
        roll, pitch, yaw = euler
        
        # Yaw에 heading 보정 적용
        corrected_yaw = yaw + self.correction_system["heading_correction"]
        
        # 각도 정규화
        while corrected_yaw > math.pi:
            corrected_yaw -= 2 * math.pi
        while corrected_yaw < -math.pi:
            corrected_yaw += 2 * math.pi
        
        # Euler → Quaternion
        corrected_quat = tf_trans.quaternion_from_euler(roll, pitch, corrected_yaw)
        
        rospy.loginfo_throttle(5, f"🧭 Orientation 보정: "
                                 f"원본 yaw={math.degrees(yaw):.1f}°, "
                                 f"보정값={math.degrees(self.correction_system['heading_correction']):.1f}°, "
                                 f"보정된 yaw={math.degrees(corrected_yaw):.1f}°")
        
        return corrected_quat[0], corrected_quat[1], corrected_quat[2], corrected_quat[3]
    
    def gps_callback(self, msg):
        """GPS 콜백 - UTM 절대좌표로 저장"""
        if msg.status.status >= 0:
            # 🔥 첫 실시간 GPS로 UTM 원점 설정
            if not self.first_gps_received:
                self.setup_utm_origin_from_gps(msg.latitude, msg.longitude)
                rospy.loginfo(f"🎯 실시간 GPS로 UTM 원점 설정: ({msg.latitude:.6f}, {msg.longitude:.6f})")
            
            timestamp = msg.header.stamp.to_sec()
            gps_utm_x, gps_utm_y, zone = self.gps_to_utm(msg.latitude, msg.longitude)
            
            self.last_good_gps = {
                "x": gps_utm_x,
                "y": gps_utm_y,
                "timestamp": timestamp,
                "lat": msg.latitude,
                "lon": msg.longitude,
                "utm_zone": zone
            }
            
            # GPS 궤적 기록 (UTM)
            if not self.gps_trajectory_utm or self.distance_check_utm(self.last_good_gps, self.gps_trajectory_utm[-1], 1.0):
                self.gps_trajectory_utm.append(self.last_good_gps.copy())
                rospy.loginfo_throttle(5, f"📡 GPS UTM: ({gps_utm_x:.1f}, {gps_utm_y:.1f}) | 총 {len(self.gps_trajectory_utm)}개")
    
    def distance_check_utm(self, pose1, pose2, threshold):
        """UTM 좌표 거리 체크"""
        dx = pose1["x"] - pose2["x"]
        dy = pose1["y"] - pose2["y"]
        return math.sqrt(dx*dx + dy*dy) > threshold
    
    def waypoints_callback(self, msg):
        """웨이포인트 저장 - 카카오 API waypoints 지원"""
        try:
            waypoints_data = json.loads(msg.data)
            
            # 기존 방식 (waypoints 키)
            if "waypoints" in waypoints_data:
                self.latest_waypoints = waypoints_data["waypoints"]
                rospy.loginfo(f"🗺️ 웨이포인트 (기존): {len(self.latest_waypoints)}개")
            
            # 카카오 API 방식 (waypoints_array 키)
            elif "waypoints_array" in waypoints_data:
                self.latest_waypoints = waypoints_data["waypoints_array"]
                rospy.loginfo(f"🗺️ 웨이포인트 (카카오): {len(self.latest_waypoints)}개")
            
            # 직접 배열인 경우
            elif isinstance(waypoints_data, list):
                self.latest_waypoints = waypoints_data
                rospy.loginfo(f"🗺️ 웨이포인트 (배열): {len(self.latest_waypoints)}개")
            
            else:
                rospy.logwarn("⚠️ 알 수 없는 waypoints 형식")
                self.latest_waypoints = None
                
        except Exception as e:
            rospy.logerr(f"❌ Waypoints 오류: {e}")
    
    def publish_current_pose(self, event):
        """현재 위치 발행 - UTM 절대좌표"""
        if self.current_pose_utm is None:
            return
        
        current_time = rospy.Time.now()
        
        # ✅ UTM 절대좌표 Pose 발행
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = current_time
        pose_msg.header.frame_id = "utm"  # UTM 절대좌표계
        
        # UTM 절대좌표 직접 사용
        pose_msg.pose.pose.position.x = self.current_pose_utm["x"]
        pose_msg.pose.pose.position.y = self.current_pose_utm["y"]
        pose_msg.pose.pose.position.z = self.current_pose_utm["z"]
        pose_msg.pose.pose.orientation.x = self.current_pose_utm["qx"]
        pose_msg.pose.pose.orientation.y = self.current_pose_utm["qy"]
        pose_msg.pose.pose.orientation.z = self.current_pose_utm["qz"]
        pose_msg.pose.pose.orientation.w = self.current_pose_utm["qw"]
        
        pose_msg.pose.covariance = self.pose_covariance.flatten().tolist()
        
        self.pose_pub.publish(pose_msg)
        
        # ✅ UTM 절대좌표 Odom 발행
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "utm"  # UTM 절대좌표계
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose = pose_msg.pose
        
        self.odom_pub.publish(odom_msg)
        full_odom = odom_msg  # 기존 odom_msg 재사용
        self.full_analysis_pub.publish(full_odom)

    
    def publish_visualization(self, event):
        """시각화 발행 - UTM 절대좌표"""
        self.visualize_fasterlio_path()
        self.visualize_gps_path()
        self.visualize_corrected_path()
        self.visualize_uncertainty()
        self.visualize_waypoints()
    
    def visualize_fasterlio_path(self):
        """FasterLIO 원본 경로 (회색) - UTM 절대좌표"""
        if len(self.fasterlio_trajectory_utm) < 2:
            return
        
        marker = self.create_utm_path_marker(
            self.fasterlio_trajectory_utm, "fasterlio_original", 0,
            (0.5, 0.5, 0.5), 2.0
        )
        self.fasterlio_path_pub.publish(marker)
    
    def visualize_gps_path(self):
        """GPS 경로 (파란색) - UTM 절대좌표"""
        if len(self.gps_trajectory_utm) < 2:
            return
            
        marker = self.create_utm_path_marker(
            self.gps_trajectory_utm, "gps_path", 0,
            (0.0, 0.0, 1.0), 3.0
        )
        self.gps_path_pub.publish(marker)
    
    def visualize_corrected_path(self):
        """보정된 FasterLIO 경로 (빨간색) - UTM 절대좌표"""
        if len(self.corrected_trajectory_utm) < 2:
            return
        marker = self.create_utm_path_marker(
            self.corrected_trajectory_utm, "corrected_path", 0,
            (1.0, 0.0, 0.0), 3.0
        )
        self.corrected_path_pub.publish(marker)
    
    def visualize_uncertainty(self):
        """현재 위치 불확실성 - UTM 절대좌표"""
        if self.current_pose_utm is None:
            return
        
        uncertainty = math.sqrt(self.pose_covariance[0,0])
        
        marker = Marker()
        marker.header.frame_id = "utm"  # UTM 절대좌표계
        marker.header.stamp = rospy.Time.now()
        marker.ns = "pose_uncertainty"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        # UTM 절대좌표 직접 사용
        marker.pose.position.x = self.current_pose_utm["x"]
        marker.pose.position.y = self.current_pose_utm["y"]
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = uncertainty * 2.0
        marker.scale.y = uncertainty * 2.0
        marker.scale.z = 0.1
        
        # 정렬 상태에 따른 색상
        if self.correction_system["initial_alignment_done"]:
            marker.color.r, marker.color.g, marker.color.b = 0.0, 1.0, 0.0  # 녹색
        else:
            marker.color.r, marker.color.g, marker.color.b = 1.0, 1.0, 0.0  # 노란색
        
        marker.color.a = 0.3
        self.uncertainty_pub.publish(marker)
    
    def visualize_waypoints(self):
        """웨이포인트 시각화 - UTM 절대좌표 (카카오 API 지원)"""
        marker_array = MarkerArray()
        
        delete_marker = Marker()
        delete_marker.header.frame_id = "utm"  # UTM 절대좌표계
        delete_marker.header.stamp = rospy.Time.now()
        delete_marker.ns = "global_waypoints"
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        if not self.latest_waypoints:
            self.waypoints_pub.publish(marker_array)
            return
        
        # 연결선
        line_marker = Marker()
        line_marker.header.frame_id = "utm"  # UTM 절대좌표계
        line_marker.header.stamp = rospy.Time.now()
        line_marker.ns = "global_waypoints"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 3.0
        line_marker.color.r = 1.0
        line_marker.color.g = 0.5
        line_marker.color.b = 0.0
        line_marker.color.a = 1.0
        line_marker.pose.orientation.w = 1.0
        
        points = []
        for wp in self.latest_waypoints:
            utm_x, utm_y, _ = self.gps_to_utm(wp["lat"], wp["lon"])
            # UTM 절대좌표 직접 사용
            points.append(Point(x=utm_x, y=utm_y, z=0))
        
        line_marker.points = points
        marker_array.markers.append(line_marker)
        
        # 웨이포인트 큐브들
        for i, wp in enumerate(self.latest_waypoints):
            utm_x, utm_y, _ = self.gps_to_utm(wp["lat"], wp["lon"])
            
            cube = Marker()
            cube.header.frame_id = "utm"  # UTM 절대좌표계
            cube.header.stamp = rospy.Time.now()
            cube.ns = "global_waypoints"
            cube.id = i + 1
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose.position.x = utm_x
            cube.pose.position.y = utm_y
            cube.pose.position.z = 0
            cube.pose.orientation.w = 1.0
            cube.scale.x = 4.0
            cube.scale.y = 4.0
            cube.scale.z = 1.0
            cube.color.r = 1.0
            cube.color.g = 1.0
            cube.color.b = 0.0
            cube.color.a = 1.0
            
            marker_array.markers.append(cube)
        
        self.waypoints_pub.publish(marker_array)
        rospy.loginfo_throttle(10, f"🗺️ 웨이포인트 시각화: {len(self.latest_waypoints)}개 (UTM 절대좌표)")
    
    def create_utm_path_marker(self, trajectory, namespace, marker_id, color, line_width):
        """UTM 절대좌표 경로 마커 생성"""
        marker = Marker()
        marker.header.frame_id = "utm"  # UTM 절대좌표계
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
            # UTM 절대좌표 직접 사용
            points.append(Point(x=pt["x"], y=pt["y"], z=pt.get("z", 0)))
        
        marker.points = points
        return marker
    
    def broadcast_tf(self, event):
        """TF 브로드캐스트 - UTM 절대좌표계 기준"""
        if self.current_pose_utm is None:
            return
        
        current_time = rospy.Time.now()
        transforms = []
        
        # ✅ utm → camera_init (직접 연결)
        utm_to_camera = TransformStamped()
        utm_to_camera.header.stamp = current_time
        utm_to_camera.header.frame_id = "utm"          # UTM 절대좌표계
        utm_to_camera.child_frame_id = "camera_init"   # FasterLIO
        
        # FasterLIO 원점을 UTM 절대좌표에 매핑
        if self.utm_origin and self.fasterlio_origin:
            utm_to_camera.transform.translation.x = (
                self.utm_origin["easting"] - self.fasterlio_origin["x"]
            )
            utm_to_camera.transform.translation.y = (
                self.utm_origin["northing"] - self.fasterlio_origin["y"]
            )
            utm_to_camera.transform.translation.z = 0.0
        
        # Heading 보정 적용
        if self.correction_system["initial_alignment_done"]:
            correction_yaw = self.correction_system["heading_correction"]
            utm_to_camera.transform.rotation.z = math.sin(correction_yaw / 2.0)
            utm_to_camera.transform.rotation.w = math.cos(correction_yaw / 2.0)
        else:
            utm_to_camera.transform.rotation.w = 1.0
        
        transforms.append(utm_to_camera)
        
        # 🔗 body → base_link (ROS 표준 호환성)
        body_to_base = TransformStamped()
        body_to_base.header.stamp = current_time
        body_to_base.header.frame_id = "body"
        body_to_base.child_frame_id = "base_link"
        body_to_base.transform.rotation.w = 1.0
        
        transforms.append(body_to_base)
        
        # 모든 TF 발행
        self.tf_broadcaster.sendTransform(transforms)
    
    def publish_gps_data(self, event):
        """GPS 기준점 발행"""
        if self.utm_origin:
            gps_data = {
                "latitude": self.utm_origin["lat"],
                "longitude": self.utm_origin["lon"]
            }
            self.gps_data_pub.publish(json.dumps(gps_data))

if __name__ == '__main__':
    try:
        localizer = UTMHeadingCorrection()
        rospy.loginfo("🎉 UTM 기반 Heading 보정 Localizer 실행 중...")
        rospy.loginfo("🌍 RViz Fixed Frame을 'utm'으로 설정하세요!")
        rospy.loginfo("✅ 모든 좌표계가 UTM 절대좌표로 통일되었습니다!")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 시스템 종료")
    except Exception as e:
        rospy.logerr(f"❌ 시스템 오류: {e}")