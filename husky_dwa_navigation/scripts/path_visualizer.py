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

class DualCoordinateUTMLocalizer:
    """이중 좌표계 UTM Localizer - 정확한 방향 보정 + RViz 호환성"""
    
    def __init__(self):
        rospy.set_param('/use_sim_time', True)
        rospy.init_node('dual_utm_localizer', anonymous=True)

        # 🎯 이중 좌표계 시스템
        self.utm_absolute_origin = None     # 실제 UTM 절대좌표 원점
        self.utm_local_origin = None        # RViz용 로컬 UTM 원점 (0,0)
        self.utm_zone = None
        self.first_gps_received = False
        
        # FasterLIO 기준점
        self.fasterlio_origin = None
        
        # 🔥 개선된 방향 보정 시스템
        self.correction_system = {
            "heading_correction": 0.0,
            "utm_convergence_angle": 0.0,      # UTM 수렴각 보정
            "initial_alignment_done": False,
            "last_correction_time": 0.0,
            "gps_quality_threshold": 3.0,      # GPS 품질 필터링
        }
        
        # 궤적 기록 (절대좌표와 로컬좌표 분리)
        self.absolute_trajectory = {
            "fasterlio": [],     # 절대 UTM 좌표
            "gps": [],           # 절대 UTM 좌표  
            "corrected": []      # 보정된 절대 UTM 좌표
        }
        
        self.local_trajectory = {
            "fasterlio": [],     # 로컬 좌표 (RViz용)
            "gps": [],           # 로컬 좌표 (RViz용)
            "corrected": []      # 보정된 로컬 좌표 (RViz용)
        }
        
        # 현재 위치
        self.current_pose_absolute = None
        self.current_pose_local = None
        self.pose_covariance = np.eye(6) * 0.1
        
        # 거리 및 품질 추적
        self.total_distance = 0.0
        self.last_position = None
        self.latest_waypoints = None

        # Publishers - 로컬 좌표계 사용 (RViz 호환)
        self.pose_pub = rospy.Publisher("/robot_pose", PoseWithCovarianceStamped, queue_size=1)
        self.odom_pub = rospy.Publisher("/fused_odom", Odometry, queue_size=1)
        
        # 시각화 Publishers - 로컬 좌표계
        self.fasterlio_path_pub = rospy.Publisher("/fasterlio_path", Marker, queue_size=10)
        self.gps_path_pub = rospy.Publisher("/gps_path", Marker, queue_size=10)
        self.corrected_path_pub = rospy.Publisher("/corrected_path", Marker, queue_size=10)
        self.uncertainty_pub = rospy.Publisher("/pose_uncertainty", Marker, queue_size=10)
        self.waypoints_pub = rospy.Publisher("/global_waypoints", MarkerArray, queue_size=10)

        # 분석용 Publishers - 절대좌표 정보 포함
        self.analysis_pub = rospy.Publisher("/analysis/coordinate_info", String, queue_size=1)
        self.absolute_pose_pub = rospy.Publisher("/absolute_pose", PoseStamped, queue_size=1)
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Subscribers
        rospy.Subscriber("/Odometry", Odometry, self.fasterlio_callback)
        rospy.Subscriber("/ublox/fix", NavSatFix, self.gps_callback)
        rospy.Subscriber("/waypoints", String, self.waypoints_callback)
        
        # Timers
        rospy.Timer(rospy.Duration(0.1), self.publish_poses)
        rospy.Timer(rospy.Duration(0.5), self.publish_visualization)
        rospy.Timer(rospy.Duration(0.1), self.broadcast_tf)
        rospy.Timer(rospy.Duration(2.0), self.publish_analysis)
        rospy.Timer(rospy.Duration(10.0), self.check_heading_correction)
        
        rospy.loginfo("🚀 이중 좌표계 UTM Localizer 시작!")
        rospy.loginfo("📍 절대좌표로 정확한 방향 보정 + 로컬좌표로 RViz 시각화")
        rospy.loginfo("🌍 RViz Fixed Frame을 'map'으로 설정하세요!")

    def setup_dual_utm_system(self, lat, lon):
        """이중 UTM 좌표계 설정"""
        if not self.first_gps_received:
            easting, northing, zone_num, zone_letter = utm.from_latlon(lat, lon)
            
            # 절대 UTM 좌표 원점 (실제 UTM 값)
            self.utm_absolute_origin = {
                "easting": easting,
                "northing": northing,
                "lat": lat,
                "lon": lon
            }
            
            # 로컬 UTM 좌표 원점 (RViz용, 0,0)
            self.utm_local_origin = {
                "easting": 0.0,
                "northing": 0.0,
                "lat": lat,
                "lon": lon
            }
            
            self.utm_zone = f"{zone_num}{zone_letter}"
            
            # UTM 수렴각 계산 (정확한 방향 보정용)
            self.correction_system["utm_convergence_angle"] = self.calculate_utm_convergence_angle(lat, lon)
            
            self.first_gps_received = True
            
            rospy.loginfo(f"🎯 이중 UTM 좌표계 설정 완료!")
            rospy.loginfo(f"   GPS: ({lat:.6f}, {lon:.6f})")
            rospy.loginfo(f"   절대 UTM: ({easting:.1f}, {northing:.1f})")
            rospy.loginfo(f"   로컬 UTM: (0.0, 0.0)")
            rospy.loginfo(f"   Zone: {self.utm_zone}")
            rospy.loginfo(f"   수렴각: {math.degrees(self.correction_system['utm_convergence_angle']):.3f}도")
            
            return True
        return False

    def calculate_utm_convergence_angle(self, lat, lon):
        """UTM 수렴각 계산 (Grid North vs True North)"""
        # UTM 중앙 경선 계산
        zone_num = int((lon + 180) / 6) + 1
        central_meridian = (zone_num - 1) * 6 - 180 + 3
        
        # 수렴각 계산 (근사식)
        lat_rad = math.radians(lat)
        lon_diff = math.radians(lon - central_meridian)
        
        convergence = lon_diff * math.sin(lat_rad)
        
        rospy.loginfo(f"🧭 UTM 수렴각 계산: 경도차={math.degrees(lon_diff):.3f}도, 수렴각={math.degrees(convergence):.3f}도")
        
        return convergence

    def gps_to_dual_coordinates(self, lat, lon):
        """GPS를 절대좌표와 로컬좌표로 동시 변환"""
        easting, northing, zone_num, zone_letter = utm.from_latlon(lat, lon)
        zone = f"{zone_num}{zone_letter}"
        
        # 절대 좌표
        absolute_x = easting
        absolute_y = northing
        
        # 로컬 좌표 (RViz용)
        if self.utm_absolute_origin:
            local_x = easting - self.utm_absolute_origin["easting"]
            local_y = northing - self.utm_absolute_origin["northing"]
        else:
            local_x = 0.0
            local_y = 0.0
        
        return {
            "absolute": (absolute_x, absolute_y, zone),
            "local": (local_x, local_y)
        }

    def perform_enhanced_heading_alignment(self):
        """🎯 개선된 방향 정렬 (절대좌표 기반)"""
        if len(self.absolute_trajectory["fasterlio"]) < 5 or len(self.absolute_trajectory["gps"]) < 5:
            rospy.logwarn("❌ 방향 정렬용 절대좌표 궤적 데이터 부족")
            return False
        
        # 고품질 GPS 데이터만 사용
        quality_gps = [gps for gps in self.absolute_trajectory["gps"] 
                      if gps.get("hdop", 999) < self.correction_system["gps_quality_threshold"]]
        
        if len(quality_gps) < 3:
            rospy.logwarn("❌ 고품질 GPS 데이터 부족")
            return False
        
        # 절대좌표 기반 방향 계산
        fasterlio_heading = self.calculate_robust_heading(self.absolute_trajectory["fasterlio"])
        gps_heading = self.calculate_robust_heading(quality_gps)
        
        if fasterlio_heading is None or gps_heading is None:
            return False
        
        # UTM 수렴각 보정 적용
        true_gps_heading = gps_heading + self.correction_system["utm_convergence_angle"]
        
        # 회전각 계산
        angle_diff = true_gps_heading - fasterlio_heading
        
        # 각도 정규화
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # 방향 보정 설정
        self.correction_system["heading_correction"] = angle_diff
        self.correction_system["initial_alignment_done"] = True
        
        rospy.loginfo(f"🎯 개선된 방향 정렬 완료!")
        rospy.loginfo(f"   FasterLIO 방향: {math.degrees(fasterlio_heading):.2f}도")
        rospy.loginfo(f"   GPS 원시 방향: {math.degrees(gps_heading):.2f}도")
        rospy.loginfo(f"   UTM 수렴각 보정: {math.degrees(self.correction_system['utm_convergence_angle']):.2f}도")
        rospy.loginfo(f"   보정된 GPS 방향: {math.degrees(true_gps_heading):.2f}도")
        rospy.loginfo(f"   최종 회전 보정: {math.degrees(angle_diff):.2f}도")
        
        self.recalculate_all_trajectories()
        return True

    def calculate_robust_heading(self, trajectory, min_distance=3.0):
        """강화된 방향 계산 (이상치 제거)"""
        if len(trajectory) < 3:
            return None
        
        # 여러 구간의 방향 계산
        headings = []
        
        for i in range(len(trajectory) - 2):
            for j in range(i + 2, len(trajectory)):
                p1 = trajectory[i]
                p2 = trajectory[j]
                
                dx = p2["x"] - p1["x"]
                dy = p2["y"] - p1["y"]
                distance = math.sqrt(dx*dx + dy*dy)
                
                if distance >= min_distance:
                    heading = math.atan2(dy, dx)
                    headings.append(heading)
        
        if len(headings) < 2:
            return None
        
        # 중앙값 사용 (이상치 제거)
        headings.sort()
        median_heading = headings[len(headings) // 2]
        
        rospy.loginfo(f"✅ 강화된 방향 계산: {len(headings)}개 구간, 중앙값={math.degrees(median_heading):.2f}도")
        
        return median_heading

    def gps_callback(self, msg):
        """GPS 콜백 - 이중 좌표계 저장"""
        if msg.status.status >= 0:
            # 첫 GPS로 이중 좌표계 설정
            if not self.first_gps_received:
                self.setup_dual_utm_system(msg.latitude, msg.longitude)
            
            timestamp = msg.header.stamp.to_sec()
            coordinates = self.gps_to_dual_coordinates(msg.latitude, msg.longitude)
            
            # GPS 품질 정보
            hdop = getattr(msg.position_covariance, 'position_covariance', [999])[0]
            
            # 절대좌표 기록
            abs_gps = {
                "x": coordinates["absolute"][0],
                "y": coordinates["absolute"][1],
                "timestamp": timestamp,
                "lat": msg.latitude,
                "lon": msg.longitude,
                "hdop": hdop,
                "utm_zone": coordinates["absolute"][2]
            }
            
            # 로컬좌표 기록 (RViz용)
            local_gps = {
                "x": coordinates["local"][0],
                "y": coordinates["local"][1],
                "timestamp": timestamp,
                "lat": msg.latitude,
                "lon": msg.longitude,
                "hdop": hdop
            }
            
            # 궤적 기록 (거리 체크)
            if (not self.absolute_trajectory["gps"] or 
                self.distance_check(abs_gps, self.absolute_trajectory["gps"][-1], 1.0)):
                
                self.absolute_trajectory["gps"].append(abs_gps)
                self.local_trajectory["gps"].append(local_gps)
                
                rospy.loginfo_throttle(5, f"📡 GPS: 절대({coordinates['absolute'][0]:.1f}, {coordinates['absolute'][1]:.1f}) "
                                          f"로컬({coordinates['local'][0]:.1f}, {coordinates['local'][1]:.1f}) "
                                          f"HDOP:{hdop:.1f}")

    def fasterlio_callback(self, msg):
        """FasterLIO 콜백 - 이중 좌표계 처리"""
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
        
        # 이중 좌표계로 변환
        if self.utm_absolute_origin and self.utm_local_origin:
            # 절대좌표 변환
            abs_pose = self.fasterlio_to_absolute_utm(fasterlio_pose)
            local_pose = self.fasterlio_to_local_utm(fasterlio_pose)
            
            # 현재 위치 업데이트
            self.current_pose_absolute = abs_pose
            self.current_pose_local = local_pose
            
            # 궤적 기록
            if (not self.absolute_trajectory["fasterlio"] or 
                self.distance_check(abs_pose, self.absolute_trajectory["fasterlio"][-1], 0.5)):
                
                self.absolute_trajectory["fasterlio"].append(abs_pose.copy())
                self.local_trajectory["fasterlio"].append(local_pose.copy())
            
            # 거리 업데이트
            self.update_distance(local_pose)
            
            # 방향 정렬 체크
            if not self.correction_system["initial_alignment_done"] and self.total_distance >= 3.0:
                rospy.loginfo(f"📏 총 이동거리 {self.total_distance:.1f}m → 개선된 방향 정렬 수행")
                self.perform_enhanced_heading_alignment()

    def fasterlio_to_absolute_utm(self, fasterlio_pose):
        """FasterLIO를 절대 UTM 좌표로 변환"""
        if not self.utm_absolute_origin or not self.fasterlio_origin:
            return fasterlio_pose
        
        # 상대좌표 계산
        rel_x = fasterlio_pose["x"] - self.fasterlio_origin["x"]
        rel_y = fasterlio_pose["y"] - self.fasterlio_origin["y"]
        
        # 방향 보정 적용
        if self.correction_system["initial_alignment_done"]:
            corrected_x, corrected_y = self.rotate_point(rel_x, rel_y, self.correction_system["heading_correction"])
        else:
            corrected_x, corrected_y = rel_x, rel_y
        
        # 절대 UTM 좌표로 변환
        abs_x = corrected_x + self.utm_absolute_origin["easting"]
        abs_y = corrected_y + self.utm_absolute_origin["northing"]
        
        abs_pose = fasterlio_pose.copy()
        abs_pose["x"] = abs_x
        abs_pose["y"] = abs_y
        
        return abs_pose

    def fasterlio_to_local_utm(self, fasterlio_pose):
        """FasterLIO를 로컬 UTM 좌표로 변환 (RViz용)"""
        if not self.utm_local_origin or not self.fasterlio_origin:
            return fasterlio_pose
        
        # 상대좌표 계산
        rel_x = fasterlio_pose["x"] - self.fasterlio_origin["x"]
        rel_y = fasterlio_pose["y"] - self.fasterlio_origin["y"]
        
        # 방향 보정 적용
        if self.correction_system["initial_alignment_done"]:
            corrected_x, corrected_y = self.rotate_point(rel_x, rel_y, self.correction_system["heading_correction"])
        else:
            corrected_x, corrected_y = rel_x, rel_y
        
        # 로컬 좌표 (원점이 0,0)
        local_x = corrected_x + self.utm_local_origin["easting"]  # +0.0
        local_y = corrected_y + self.utm_local_origin["northing"] # +0.0
        
        local_pose = fasterlio_pose.copy()
        local_pose["x"] = local_x
        local_pose["y"] = local_y
        
        return local_pose

    def rotate_point(self, x, y, angle):
        """점 회전"""
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        
        rotated_x = x * cos_a - y * sin_a
        rotated_y = x * sin_a + y * cos_a
        
        return rotated_x, rotated_y

    def distance_check(self, pose1, pose2, threshold):
        """거리 체크"""
        dx = pose1["x"] - pose2["x"]
        dy = pose1["y"] - pose2["y"]
        return math.sqrt(dx*dx + dy*dy) > threshold

    def update_distance(self, new_position):
        """이동 거리 업데이트"""
        if self.last_position is not None:
            dx = new_position["x"] - self.last_position["x"]
            dy = new_position["y"] - self.last_position["y"]
            distance = math.sqrt(dx*dx + dy*dy)
            self.total_distance += distance
        
        self.last_position = new_position.copy()

    def recalculate_all_trajectories(self):
        """전체 궤적 재계산"""
        rospy.loginfo("🔄 전체 궤적 재계산 중...")
        # 실제 구현은 기존 방식과 유사하지만 이중 좌표계 적용
        pass

    def waypoints_callback(self, msg):
        """웨이포인트 콜백"""
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
                
            rospy.loginfo(f"🗺️ 웨이포인트: {len(self.latest_waypoints) if self.latest_waypoints else 0}개")
                
        except Exception as e:
            rospy.logerr(f"❌ Waypoints 오류: {e}")

    def publish_poses(self, event):
        """위치 발행 - 로컬 좌표계 사용 (RViz 호환)"""
        if self.current_pose_local is None:
            return
        
        current_time = rospy.Time.now()
        
        # 로컬 좌표계 Pose 발행 (RViz용)
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = current_time
        pose_msg.header.frame_id = "map"  # RViz 호환 프레임
        
        pose_msg.pose.pose.position.x = self.current_pose_local["x"]
        pose_msg.pose.pose.position.y = self.current_pose_local["y"]
        pose_msg.pose.pose.position.z = self.current_pose_local["z"]
        pose_msg.pose.pose.orientation.x = self.current_pose_local["qx"]
        pose_msg.pose.pose.orientation.y = self.current_pose_local["qy"]
        pose_msg.pose.pose.orientation.z = self.current_pose_local["qz"]
        pose_msg.pose.pose.orientation.w = self.current_pose_local["qw"]
        
        pose_msg.pose.covariance = self.pose_covariance.flatten().tolist()
        self.pose_pub.publish(pose_msg)
        
        # 로컬 좌표계 Odom 발행
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose = pose_msg.pose
        
        self.odom_pub.publish(odom_msg)
        
        # 절대좌표 정보 발행 (분석용)
        if self.current_pose_absolute:
            abs_pose_msg = PoseStamped()
            abs_pose_msg.header.stamp = current_time
            abs_pose_msg.header.frame_id = "utm_absolute"
            
            abs_pose_msg.pose.position.x = self.current_pose_absolute["x"]
            abs_pose_msg.pose.position.y = self.current_pose_absolute["y"]
            abs_pose_msg.pose.position.z = self.current_pose_absolute["z"]
            abs_pose_msg.pose.orientation.x = self.current_pose_absolute["qx"]
            abs_pose_msg.pose.orientation.y = self.current_pose_absolute["qy"]
            abs_pose_msg.pose.orientation.z = self.current_pose_absolute["qz"]
            abs_pose_msg.pose.orientation.w = self.current_pose_absolute["qw"]
            
            self.absolute_pose_pub.publish(abs_pose_msg)

    def publish_visualization(self, event):
        """시각화 발행 - 로컬 좌표계 사용"""
        self.visualize_paths()
        self.visualize_uncertainty()
        self.visualize_waypoints()

    def visualize_paths(self):
        """경로 시각화"""
        # FasterLIO 경로 (회색)
        if len(self.local_trajectory["fasterlio"]) >= 2:
            marker = self.create_path_marker(
                self.local_trajectory["fasterlio"], "fasterlio_path", 
                (0.5, 0.5, 0.5), 2.0
            )
            self.fasterlio_path_pub.publish(marker)
        
        # GPS 경로 (파란색)
        if len(self.local_trajectory["gps"]) >= 2:
            marker = self.create_path_marker(
                self.local_trajectory["gps"], "gps_path", 
                (0.0, 0.0, 1.0), 3.0
            )
            self.gps_path_pub.publish(marker)
        
        # 보정된 경로 (빨간색)
        if len(self.local_trajectory["corrected"]) >= 2:
            marker = self.create_path_marker(
                self.local_trajectory["corrected"], "corrected_path", 
                (1.0, 0.0, 0.0), 3.0
            )
            self.corrected_path_pub.publish(marker)

    def visualize_uncertainty(self):
        """불확실성 시각화"""
        if self.current_pose_local is None:
            return
        
        uncertainty = math.sqrt(self.pose_covariance[0,0])
        
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "pose_uncertainty"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        marker.pose.position.x = self.current_pose_local["x"]
        marker.pose.position.y = self.current_pose_local["y"]
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
        """웨이포인트 시각화"""
        marker_array = MarkerArray()
        
        # 기존 마커 삭제
        delete_marker = Marker()
        delete_marker.header.frame_id = "map"
        delete_marker.header.stamp = rospy.Time.now()
        delete_marker.ns = "global_waypoints"
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        if not self.latest_waypoints:
            self.waypoints_pub.publish(marker_array)
            return
        
        # 웨이포인트들을 로컬 좌표로 변환하여 시각화
        waypoint_points = []
        for wp in self.latest_waypoints:
            coords = self.gps_to_dual_coordinates(wp["lat"], wp["lon"])
            waypoint_points.append(coords["local"])
        
        # 연결선
        if len(waypoint_points) > 1:
            line_marker = Marker()
            line_marker.header.frame_id = "map"
            line_marker.header.stamp = rospy.Time.now()
            line_marker.ns = "global_waypoints"
            line_marker.id = 0
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.scale.x = 2.0
            line_marker.color.r = 1.0
            line_marker.color.g = 0.5
            line_marker.color.b = 0.0
            line_marker.color.a = 1.0
            line_marker.pose.orientation.w = 1.0
            
            for wp_local in waypoint_points:
                line_marker.points.append(Point(x=wp_local[0], y=wp_local[1], z=0))
            
            marker_array.markers.append(line_marker)
        
        # 웨이포인트 큐브들
        for i, wp_local in enumerate(waypoint_points):
            cube = Marker()
            cube.header.frame_id = "map"
            cube.header.stamp = rospy.Time.now()
            cube.ns = "global_waypoints"
            cube.id = i + 1
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose.position.x = wp_local[0]
            cube.pose.position.y = wp_local[1]
            cube.pose.position.z = 0
            cube.pose.orientation.w = 1.0
            cube.scale.x = 3.0
            cube.scale.y = 3.0
            cube.scale.z = 1.0
            cube.color.r = 1.0
            cube.color.g = 1.0
            cube.color.b = 0.0
            cube.color.a = 1.0
            
            marker_array.markers.append(cube)
        
        self.waypoints_pub.publish(marker_array)

    def create_path_marker(self, trajectory, namespace, color, line_width):
        """경로 마커 생성"""
        marker = Marker()
        marker.header.frame_id = "map"  # RViz 호환 프레임
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = line_width
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0
        
        for pt in trajectory:
            marker.points.append(Point(x=pt["x"], y=pt["y"], z=pt.get("z", 0)))
        
        return marker

    def broadcast_tf(self, event):
        """TF 브로드캐스트 - 로컬 좌표계"""
        if self.current_pose_local is None:
            return
        
        current_time = rospy.Time.now()
        
        transform = TransformStamped()
        transform.header.stamp = current_time
        transform.header.frame_id = "map"
        transform.child_frame_id = "base_link"
        
        transform.transform.translation.x = self.current_pose_local["x"]
        transform.transform.translation.y = self.current_pose_local["y"]
        transform.transform.translation.z = self.current_pose_local["z"]
        transform.transform.rotation.x = self.current_pose_local["qx"]
        transform.transform.rotation.y = self.current_pose_local["qy"]
        transform.transform.rotation.z = self.current_pose_local["qz"]
        transform.transform.rotation.w = self.current_pose_local["qw"]
        
        self.tf_broadcaster.sendTransform(transform)

    def publish_analysis(self, event):
        """분석 정보 발행"""
        if not self.current_pose_absolute or not self.current_pose_local:
            return
        
        analysis_data = {
            "timestamp": rospy.Time.now().to_sec(),
            "absolute_utm": {
                "x": self.current_pose_absolute["x"],
                "y": self.current_pose_absolute["y"],
                "zone": self.utm_zone
            },
            "local_utm": {
                "x": self.current_pose_local["x"],
                "y": self.current_pose_local["y"]
            },
            "heading_correction": {
                "angle_deg": math.degrees(self.correction_system["heading_correction"]),
                "utm_convergence_deg": math.degrees(self.correction_system["utm_convergence_angle"]),
                "alignment_done": self.correction_system["initial_alignment_done"]
            },
            "trajectory_counts": {
                "fasterlio": len(self.absolute_trajectory["fasterlio"]),
                "gps": len(self.absolute_trajectory["gps"]),
                "corrected": len(self.absolute_trajectory["corrected"])
            },
            "total_distance_m": self.total_distance
        }
        
        self.analysis_pub.publish(json.dumps(analysis_data, indent=2))

    def check_heading_correction(self, event):
        """주기적 방향 보정 체크"""
        if not self.correction_system["initial_alignment_done"]:
            return
        
        # 점진적 보정 로직 (기존과 유사하지만 절대좌표 기반)
        current_time = rospy.Time.now().to_sec()
        time_since_last = current_time - self.correction_system.get("last_correction_time", 0)
        
        if time_since_last > 30.0:  # 30초마다 체크
            rospy.loginfo("🔍 주기적 방향 보정 체크 중...")
            # 실제 구현 필요
            self.correction_system["last_correction_time"] = current_time

if __name__ == '__main__':
    try:
        localizer = DualCoordinateUTMLocalizer()
        rospy.loginfo("🎉 이중 좌표계 UTM Localizer 실행 중...")
        rospy.loginfo("📍 절대좌표: 정확한 방향 보정")
        rospy.loginfo("🎯 로컬좌표: RViz 시각화 호환")
        rospy.loginfo("🌍 RViz Fixed Frame: 'map'")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 시스템 종료")
    except Exception as e:
        rospy.logerr(f"❌ 시스템 오류: {e}")