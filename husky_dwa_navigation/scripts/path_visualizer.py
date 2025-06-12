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
    """UTM 기반 점진적 Heading 보정 Localizer - 동적 TF 반영"""
    
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
        self.current_body_pose = None       # 현재 FasterLIO body pose
        
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

        # ✅ UTM 절대좌표 Publishers
        self.pose_pub = rospy.Publisher("/robot_pose", PoseWithCovarianceStamped, queue_size=1)
        self.odom_pub = rospy.Publisher("/fused_odom", Odometry, queue_size=1)
        
        # ✅ UTM 절대좌표 시각화 Publishers
        self.fasterlio_path_pub = rospy.Publisher("/fasterlio_path", Marker, queue_size=10)
        self.gps_path_pub = rospy.Publisher("/gps_path", Marker, queue_size=10)
        self.corrected_path_pub = rospy.Publisher("/corrected_path", Marker, queue_size=10)
        self.uncertainty_pub = rospy.Publisher("/pose_uncertainty", Marker, queue_size=10)
        self.waypoints_pub = rospy.Publisher("/global_waypoints", MarkerArray, queue_size=10)

        # 분석용 Publishers
        self.raw_analysis_pub = rospy.Publisher("/analysis/raw_fasterlio", Odometry, queue_size=1)
        self.initial_analysis_pub = rospy.Publisher("/analysis/initial_corrected", Odometry, queue_size=1)
        self.full_analysis_pub = rospy.Publisher("/analysis/fully_corrected", Odometry, queue_size=1)
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
        rospy.Timer(rospy.Duration(0.1), self.broadcast_dynamic_tf)  # 🔥 동적 TF
        rospy.Timer(rospy.Duration(1.0), self.publish_gps_data)
        rospy.Timer(rospy.Duration(10.0), self.check_gradual_heading_correction)
        
        rospy.loginfo("🚀 UTM 기반 Heading 보정 Localizer 시작!")
        rospy.loginfo("📍 GPS_FIRST 전략: 첫 실시간 GPS 수신 시 UTM 원점 설정")
        rospy.loginfo("🌍 모든 좌표계 UTM 절대좌표로 통일!")
        rospy.loginfo("🔄 동적 TF 발행으로 실시간 움직임 반영!")

    def setup_utm_origin_from_gps(self, lat, lon):
        """🎯 FasterLIO와 동기화된 GPS 원점 설정"""
        if not self.first_gps_received and self.fasterlio_origin is not None:
            # 시뮬레이션 GPS 설정 고려 (0.0, 0.0 기준)
            if abs(lat) < 0.01 and abs(lon) < 0.01:
                rospy.loginfo("🎮 시뮬레이션 GPS 감지: 0.0, 0.0 기준점 사용")
                # 시뮬레이션에서는 직접 UTM 변환
                easting, northing = lat * 111320, lon * 111320  # 근사 변환
                zone_num, zone_letter = 52, 'S'  # 시뮬레이션 기본 존
            else:
                easting, northing, zone_num, zone_letter = utm.from_latlon(lat, lon)
            
            # 🔥 FasterLIO 현재 위치 고려한 동기화
            if self.current_body_pose:
                fasterlio_rel_x = self.current_body_pose["x"] - self.fasterlio_origin["x"]
                fasterlio_rel_y = self.current_body_pose["y"] - self.fasterlio_origin["y"]
                
                # UTM 원점을 FasterLIO 현재 위치에 맞춰 조정
                adjusted_easting = easting - fasterlio_rel_x
                adjusted_northing = northing - fasterlio_rel_y
                
                rospy.loginfo(f"🔄 동기화: FasterLIO 상대위치({fasterlio_rel_x:.2f}, {fasterlio_rel_y:.2f})")
                rospy.loginfo(f"   조정 전 UTM: ({easting:.1f}, {northing:.1f})")
                rospy.loginfo(f"   조정 후 UTM: ({adjusted_easting:.1f}, {adjusted_northing:.1f})")
                
                easting = adjusted_easting
                northing = adjusted_northing
            
            self.utm_origin = {
                "easting": easting,
                "northing": northing,
                "lat": lat,
                "lon": lon
            }
            self.utm_zone = f"{zone_num}{zone_letter}"
            self.first_gps_received = True
            
            rospy.loginfo(f"🎯 동기화된 UTM 원점 설정 완료!")
            rospy.loginfo(f"   GPS: ({lat:.6f}, {lon:.6f})")
            rospy.loginfo(f"   UTM: ({easting:.1f}, {northing:.1f})")
            rospy.loginfo(f"   Zone: {self.utm_zone}")
            
            return True
        return False
    
    def gps_to_utm(self, lat, lon):
        """GPS를 UTM 절대좌표로 변환 - 시뮬레이션 고려"""
        if abs(lat) < 0.01 and abs(lon) < 0.01:
            # 시뮬레이션 GPS 처리
            easting = lon * 111320
            northing = lat * 111320
            return easting, northing, "52S"
        else:
            easting, northing, zone_num, zone_letter = utm.from_latlon(lat, lon)
            return easting, northing, f"{zone_num}{zone_letter}"
    
    def fasterlio_to_utm(self, fasterlio_x, fasterlio_y):
        """FasterLIO 좌표를 UTM 절대좌표로 변환"""
        if not self.utm_origin or not self.fasterlio_origin:
            return fasterlio_x, fasterlio_y
        
        # 1단계: FasterLIO 원점 기준 상대좌표
        rel_x = fasterlio_x - self.fasterlio_origin["x"]
        rel_y = fasterlio_y - self.fasterlio_origin["y"]
        
        # 2단계: Heading 보정 적용
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
        """🎯 초기 Heading 정렬"""
        if len(self.fasterlio_trajectory_utm) < 3 or len(self.gps_trajectory_utm) < 3:
            rospy.logwarn("❌ 초기 Heading 정렬용 궤적 데이터 부족")
            return False
        
        # 방향 계산
        fasterlio_heading = self.calculate_trajectory_heading(self.fasterlio_trajectory_utm, 1.0)
        gps_heading = self.calculate_trajectory_heading(self.gps_trajectory_utm, 1.0)
        
        if fasterlio_heading is None or gps_heading is None:
            return False
        
        # 회전각 계산 및 정규화
        angle_diff = gps_heading - fasterlio_heading
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # 보정 설정
        self.correction_system["heading_correction"] = angle_diff
        self.correction_system["initial_alignment_done"] = True
        
        rospy.loginfo(f"🎯 초기 Heading 정렬 완료!")
        rospy.loginfo(f"   FasterLIO: {math.degrees(fasterlio_heading):.1f}도")
        rospy.loginfo(f"   GPS: {math.degrees(gps_heading):.1f}도")
        rospy.loginfo(f"   보정: {math.degrees(angle_diff):.1f}도")
        
        self.recalculate_all_trajectories()
        return True
    
    def recalculate_all_trajectories(self):
        """전체 FasterLIO 궤적 재계산"""
        if not self.fasterlio_trajectory_utm:
            return
        
        self.corrected_trajectory_utm = []
        
        for fasterlio_point in self.fasterlio_trajectory_utm:
            if not self.fasterlio_origin:
                continue
                
            # 원본 복원 및 재변환
            rel_x = fasterlio_point["x"] - self.utm_origin["easting"]
            rel_y = fasterlio_point["y"] - self.utm_origin["northing"]
            original_x = rel_x + self.fasterlio_origin["x"]
            original_y = -(rel_y + self.fasterlio_origin["y"])
            
            corrected_utm_x, corrected_utm_y = self.fasterlio_to_utm(original_x, original_y)
            
            corrected_point = fasterlio_point.copy()
            corrected_point["x"] = corrected_utm_x
            corrected_point["y"] = corrected_utm_y
            
            self.corrected_trajectory_utm.append(corrected_point)
        
        rospy.loginfo(f"✅ {len(self.corrected_trajectory_utm)}개 포인트 재계산 완료")
    
    def update_distance(self, new_position):
        """이동 거리 업데이트"""
        if self.last_position is not None:
            dx = new_position["x"] - self.last_position["x"]
            dy = new_position["y"] - self.last_position["y"]
            distance = math.sqrt(dx*dx + dy*dy)
            self.total_distance += distance
        
        self.last_position = new_position.copy()
    
    def distance_check_utm(self, pose1, pose2, threshold):
        """UTM 좌표 거리 체크"""
        dx = pose1["x"] - pose2["x"]
        dy = pose1["y"] - pose2["y"]
        return math.sqrt(dx*dx + dy*dy) > threshold
    
    def fasterlio_callback(self, msg):
        """🎯 FasterLIO 메인 콜백"""
        timestamp = msg.header.stamp.to_sec()
        
        # FasterLIO 원시 pose 저장
        self.current_body_pose = {
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
            self.fasterlio_origin = self.current_body_pose.copy()
            rospy.loginfo("🎯 FasterLIO 기준점 설정 완료")
        
        # UTM 변환 (보정 없이)
        if self.utm_origin:
            rel_x = np.float64(self.current_body_pose["x"]) - np.float64(self.fasterlio_origin["x"])
            rel_y = np.float64(self.current_body_pose["y"]) - np.float64(self.fasterlio_origin["y"])
            utm_x = np.float64(rel_x) + np.float64(self.utm_origin["easting"])
            utm_y = np.float64(rel_y) + np.float64(self.utm_origin["northing"])
            
            utm_point = {
                "x": utm_x,
                "y": utm_y,
                "z": self.current_body_pose["z"],
                "qx": self.current_body_pose["qx"],
                "qy": self.current_body_pose["qy"],
                "qz": self.current_body_pose["qz"],
                "qw": self.current_body_pose["qw"],
                "timestamp": timestamp
            }
            
            # FasterLIO 궤적 기록
            if not self.fasterlio_trajectory_utm or self.distance_check_utm(utm_point, self.fasterlio_trajectory_utm[-1], 0.5):
                self.fasterlio_trajectory_utm.append(utm_point.copy())
        
        # Heading 보정 적용하여 UTM 변환
        corrected_utm_x, corrected_utm_y = self.fasterlio_to_utm(
            self.current_body_pose["x"], self.current_body_pose["y"]
        )
        
        # Orientation 보정
        corrected_qx, corrected_qy, corrected_qz, corrected_qw = self.apply_heading_correction_to_orientation(
            self.current_body_pose["qx"], self.current_body_pose["qy"],
            self.current_body_pose["qz"], self.current_body_pose["qw"]
        )
        
        # 현재 위치 업데이트 (UTM 좌표)
        self.current_pose_utm = {
            "x": corrected_utm_x,
            "y": corrected_utm_y,
            "z": self.current_body_pose["z"],
            "qx": corrected_qx,
            "qy": corrected_qy,
            "qz": corrected_qz,
            "qw": corrected_qw,
            "timestamp": timestamp
        }
        
        # 거리 및 궤적 업데이트
        self.update_distance(self.current_pose_utm)
        
        if self.utm_origin:
            if not self.corrected_trajectory_utm or self.distance_check_utm(self.current_pose_utm, self.corrected_trajectory_utm[-1], 0.5):
                self.corrected_trajectory_utm.append(self.current_pose_utm.copy())
        
        # 초기 정렬 체크
        if not self.correction_system["initial_alignment_done"] and self.total_distance >= 2.0:
            rospy.loginfo(f"📏 총 이동거리 {self.total_distance:.1f}m → 초기 Heading 정렬 수행")
            self.perform_initial_heading_alignment()
        
        # 불확실성 업데이트
        uncertainty = 2.0 if self.correction_system["initial_alignment_done"] else 10.0
        self.pose_covariance[0,0] = uncertainty
        self.pose_covariance[1,1] = uncertainty
        
        # 상태 로그
        rospy.loginfo_throttle(2, f"🎯 UTM 위치: ({corrected_utm_x:.1f}, {corrected_utm_y:.1f}), 누적 거리: {self.total_distance:.1f}m")

    def apply_heading_correction_to_orientation(self, qx, qy, qz, qw):
        """Orientation에 heading 보정 적용"""
        if not self.correction_system["initial_alignment_done"]:
            return qx, qy, qz, qw
        
        # Quaternion → Euler
        euler = tf_trans.euler_from_quaternion([qx, qy, qz, qw])
        roll, pitch, yaw = euler
        
        # Yaw 보정
        corrected_yaw = yaw + self.correction_system["heading_correction"]
        
        # 정규화
        while corrected_yaw > math.pi:
            corrected_yaw -= 2 * math.pi
        while corrected_yaw < -math.pi:
            corrected_yaw += 2 * math.pi
        
        # Euler → Quaternion
        corrected_quat = tf_trans.quaternion_from_euler(roll, pitch, corrected_yaw)
        
        rospy.loginfo_throttle(5, f"🧭 Orientation 보정: {math.degrees(yaw):.1f}° → {math.degrees(corrected_yaw):.1f}°")
        
        return corrected_quat[0], corrected_quat[1], corrected_quat[2], corrected_quat[3]
    
    def gps_callback(self, msg):
        """GPS 콜백 - FasterLIO와 동기화"""
        if msg.status.status >= 0:
            # FasterLIO 준비 대기
            if not self.first_gps_received:
                if self.fasterlio_origin is None:
                    rospy.loginfo_throttle(2, "⏳ FasterLIO 대기 중... GPS 원점 설정 보류")
                    return
                else:
                    self.setup_utm_origin_from_gps(msg.latitude, msg.longitude)
                    rospy.loginfo(f"🎯 동기화된 GPS 원점: ({msg.latitude:.6f}, {msg.longitude:.6f})")
            
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
            
            # GPS 궤적 기록
            if not self.gps_trajectory_utm or self.distance_check_utm(self.last_good_gps, self.gps_trajectory_utm[-1], 1.0):
                self.gps_trajectory_utm.append(self.last_good_gps.copy())
                rospy.loginfo_throttle(5, f"📡 GPS UTM (동기화됨): ({gps_utm_x:.1f}, {gps_utm_y:.1f}) | 총 {len(self.gps_trajectory_utm)}개")
    
    # def waypoints_callback(self, msg):
    #     """웨이포인트 저장"""
    #     try:
    #         waypoints_data = json.loads(msg.data)
            
    #         if "waypoints" in waypoints_data:
    #             self.latest_waypoints = waypoints_data["waypoints"]
    #         elif "waypoints_array" in waypoints_data:
    #             self.latest_waypoints = waypoints_data["waypoints_array"]
    #         elif isinstance(waypoints_data, list):
    #             self.latest_waypoints = waypoints_data
    #         else:
    #             self.latest_waypoints = None
                
    #     except Exception as e:
    #         rospy.logerr(f"❌ Waypoints 오류: {e}")
    def waypoints_callback(self, msg):
        """웨이포인트 저장 - UTM 좌표 직접 지원"""
        try:
            waypoints_data = json.loads(msg.data)
        
            if "waypoints" in waypoints_data:
                self.latest_waypoints = waypoints_data["waypoints"]
            elif "waypoints_array" in waypoints_data:
                self.latest_waypoints = waypoints_data["waypoints_array"]
            elif isinstance(waypoints_data, list):
                self.latest_waypoints = waypoints_data
            else:
                self.latest_waypoints = None
            
            rospy.loginfo_throttle(5, f"📍 Waypoints 수신: {len(self.latest_waypoints) if self.latest_waypoints else 0}개")
            
        except Exception as e:
            rospy.logerr(f"❌ Waypoints 오류: {e}")
    
    def broadcast_dynamic_tf(self, event):
        """🔥 동적 TF 브로드캐스트 - 실시간 움직임 반영"""
        if self.current_pose_utm is None or not self.utm_origin:
            return
        
        current_time = rospy.Time.now()
        
        # utm -> base_link 직접 변환 (동적)
        utm_to_base = TransformStamped()
        utm_to_base.header.stamp = current_time
        utm_to_base.header.frame_id = "utm"
        utm_to_base.child_frame_id = "base_link"
        
        # 🔥 핵심: 현재 UTM 위치를 TF에 직접 반영
        utm_to_base.transform.translation.x = self.current_pose_utm["x"]
        utm_to_base.transform.translation.y = self.current_pose_utm["y"]
        utm_to_base.transform.translation.z = self.current_pose_utm["z"]
        utm_to_base.transform.rotation.x = self.current_pose_utm["qx"]
        utm_to_base.transform.rotation.y = self.current_pose_utm["qy"]
        utm_to_base.transform.rotation.z = self.current_pose_utm["qz"]
        utm_to_base.transform.rotation.w = self.current_pose_utm["qw"]
        
        self.tf_broadcaster.sendTransform(utm_to_base)
        
        rospy.loginfo_throttle(10, f"📡 동적 TF 발행: utm->base_link ({self.current_pose_utm['x']:.1f}, {self.current_pose_utm['y']:.1f})")
    
    def publish_current_pose(self, event):
        """현재 위치 발행"""
        if self.current_pose_utm is None:
            return
        
        current_time = rospy.Time.now()
        
        # Pose 발행
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = current_time
        pose_msg.header.frame_id = "utm"
        
        pose_msg.pose.pose.position.x = self.current_pose_utm["x"]
        pose_msg.pose.pose.position.y = self.current_pose_utm["y"]
        pose_msg.pose.pose.position.z = self.current_pose_utm["z"]
        pose_msg.pose.pose.orientation.x = self.current_pose_utm["qx"]
        pose_msg.pose.pose.orientation.y = self.current_pose_utm["qy"]
        pose_msg.pose.pose.orientation.z = self.current_pose_utm["qz"]
        pose_msg.pose.pose.orientation.w = self.current_pose_utm["qw"]
        pose_msg.pose.covariance = self.pose_covariance.flatten().tolist()
        
        self.pose_pub.publish(pose_msg)
        
        # Odom 발행
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "utm"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose = pose_msg.pose
        
        self.odom_pub.publish(odom_msg)
        self.full_analysis_pub.publish(odom_msg)

    def publish_visualization(self, event):
        """시각화 발행"""
        self.visualize_fasterlio_path()
        self.visualize_gps_path()
        self.visualize_corrected_path()
        self.visualize_uncertainty()
        self.visualize_waypoints()
    
    def visualize_fasterlio_path(self):
        """FasterLIO 원본 경로 (회색)"""
        if len(self.fasterlio_trajectory_utm) < 2:
            return
        
        marker = self.create_utm_path_marker(
            self.fasterlio_trajectory_utm, "fasterlio_original", 0,
            (0.5, 0.5, 0.5), 2.0
        )
        self.fasterlio_path_pub.publish(marker)
    
    def visualize_gps_path(self):
        """GPS 경로 (파란색)"""
        if len(self.gps_trajectory_utm) < 2:
            return
            
        marker = self.create_utm_path_marker(
            self.gps_trajectory_utm, "gps_path", 0,
            (0.0, 0.0, 1.0), 3.0
        )
        self.gps_path_pub.publish(marker)
    
    def visualize_corrected_path(self):
        """보정된 FasterLIO 경로 (빨간색)"""
        if len(self.corrected_trajectory_utm) < 2:
            return
        marker = self.create_utm_path_marker(
            self.corrected_trajectory_utm, "corrected_path", 0,
            (1.0, 0.0, 0.0), 3.0
        )
        self.corrected_path_pub.publish(marker)
    
    def visualize_uncertainty(self):
        """현재 위치 불확실성"""
        if self.current_pose_utm is None:
            return
        
        uncertainty = math.sqrt(self.pose_covariance[0,0])
        
        marker = Marker()
        marker.header.frame_id = "utm"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "pose_uncertainty"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
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
    
    # def visualize_waypoints(self):
    #     """웨이포인트 시각화"""
    #     marker_array = MarkerArray()
        
    #     delete_marker = Marker()
    #     delete_marker.header.frame_id = "utm"
    #     delete_marker.header.stamp = rospy.Time.now()
    #     delete_marker.ns = "global_waypoints"
    #     delete_marker.action = Marker.DELETEALL
    #     marker_array.markers.append(delete_marker)
        
    #     if not self.latest_waypoints:
    #         self.waypoints_pub.publish(marker_array)
    #         return
        
    #     # 연결선
    #     line_marker = Marker()
    #     line_marker.header.frame_id = "utm"
    #     line_marker.header.stamp = rospy.Time.now()
    #     line_marker.ns = "global_waypoints"
    #     line_marker.id = 0
    #     line_marker.type = Marker.LINE_STRIP
    #     line_marker.action = Marker.ADD
    #     line_marker.scale.x = 3.0
    #     line_marker.color.r = 1.0
    #     line_marker.color.g = 0.5
    #     line_marker.color.b = 0.0
    #     line_marker.color.a = 1.0
    #     line_marker.pose.orientation.w = 1.0
        
    #     points = []
    #     for wp in self.latest_waypoints:
    #         utm_x, utm_y, _ = self.gps_to_utm(wp["lat"], wp["lon"])
    #         points.append(Point(x=utm_x, y=utm_y, z=0))
        
    #     line_marker.points = points
    #     marker_array.markers.append(line_marker)
        
    #     # 웨이포인트 큐브들
    #     for i, wp in enumerate(self.latest_waypoints):
    #         utm_x, utm_y, _ = self.gps_to_utm(wp["lat"], wp["lon"])
            
    #         cube = Marker()
    #         cube.header.frame_id = "utm"
    #         cube.header.stamp = rospy.Time.now()
    #         cube.ns = "global_waypoints"
    #         cube.id = i + 1
    #         cube.type = Marker.CUBE
    #         cube.action = Marker.ADD
    #         cube.pose.position.x = utm_x
    #         cube.pose.position.y = utm_y
    #         cube.pose.position.z = 0
    #         cube.pose.orientation.w = 1.0
    #         cube.scale.x = 4.0
    #         cube.scale.y = 4.0
    #         cube.scale.z = 1.0
    #         cube.color.r = 1.0
    #         cube.color.g = 1.0
    #         cube.color.b = 0.0
    #         cube.color.a = 1.0
            
    #         marker_array.markers.append(cube)
        
    #     self.waypoints_pub.publish(marker_array)
    def visualize_waypoints(self):
        """웨이포인트 시각화 - UTM 좌표 직접 지원"""
        marker_array = MarkerArray()
    
        delete_marker = Marker()
        delete_marker.header.frame_id = "utm"
        delete_marker.header.stamp = rospy.Time.now()
        delete_marker.ns = "global_waypoints"
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
    
        if not self.latest_waypoints:
            self.waypoints_pub.publish(marker_array)
            return
    
        # 연결선
        line_marker = Marker()
        line_marker.header.frame_id = "utm"
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
            # UTM 좌표를 직접 사용하거나 GPS에서 변환
            if "x" in wp and "y" in wp:
                # UTM 좌표 직접 사용 (새로운 형식)
                utm_x, utm_y = wp["x"], wp["y"]
            elif "lat" in wp and "lon" in wp:
                # GPS 좌표에서 UTM 변환 (기존 형식)
                utm_x, utm_y, _ = self.gps_to_utm(wp["lat"], wp["lon"])
            else:
                continue
            
            points.append(Point(x=utm_x, y=utm_y, z=0))
    
        line_marker.points = points
        marker_array.markers.append(line_marker)
    
        # 웨이포인트 큐브들
        for i, wp in enumerate(self.latest_waypoints):
            # UTM 좌표를 직접 사용하거나 GPS에서 변환
            if "x" in wp and "y" in wp:
                utm_x, utm_y = wp["x"], wp["y"]
            elif "lat" in wp and "lon" in wp:
                utm_x, utm_y, _ = self.gps_to_utm(wp["lat"], wp["lon"])
            else:
                continue
        
            cube = Marker()
            cube.header.frame_id = "utm"
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
        
            # 완료된 waypoint는 초록색, 아직 안된 것은 노란색
            if wp.get("completed", False):
                cube.color.r, cube.color.g, cube.color.b = 0.0, 1.0, 0.0  # 초록색
            else:
                cube.color.r, cube.color.g, cube.color.b = 1.0, 1.0, 0.0  # 노란색
            cube.color.a = 1.0

            marker_array.markers.append(cube)
    
            self.waypoints_pub.publish(marker_array)
            rospy.loginfo_throttle(10, f"🗺️ 웨이포인트 시각화: {len(self.latest_waypoints)}개 (UTM 좌표)")
    
    def create_utm_path_marker(self, trajectory, namespace, marker_id, color, line_width):
        """UTM 절대좌표 경로 마커 생성"""
        marker = Marker()
        marker.header.frame_id = "utm"
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
            points.append(Point(x=pt["x"], y=pt["y"], z=pt.get("z", 0)))
        
        marker.points = points
        return marker
    
    def publish_gps_data(self, event):
        """GPS 기준점 발행"""
        if self.utm_origin:
            gps_data = {
                "latitude": self.utm_origin["lat"],
                "longitude": self.utm_origin["lon"]
            }
            self.gps_data_pub.publish(json.dumps(gps_data))

    def check_gradual_heading_correction(self, event):
        """점진적 Heading 보정 체크"""
        if not self.correction_system["initial_alignment_done"]:
            return
        
        current_time = rospy.Time.now().to_sec()
        time_since_last = current_time - self.correction_system.get("last_correction_time", 0)
        
        if time_since_last > 10.0:
            # 점진적 보정 로직 구현 가능
            self.correction_system["last_correction_time"] = current_time


if __name__ == '__main__':
    try:
        localizer = UTMHeadingCorrection()
        rospy.loginfo("🎉 UTM 기반 Heading 보정 Localizer 실행 중...")
        rospy.loginfo("🌍 RViz Fixed Frame을 'utm'으로 설정하세요!")
        rospy.loginfo("✅ 모든 좌표계가 UTM 절대좌표로 통일되었습니다!")
        rospy.loginfo("🔄 FasterLIO와 GPS 시작점 자동 동기화!")
        rospy.loginfo("📡 동적 TF로 실시간 움직임 반영!")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 시스템 종료")
    except Exception as e:
        rospy.logerr(f"❌ 시스템 오류: {e}")