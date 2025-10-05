#!/usr/bin/env python3
"""
센서 융합을 위한 UTM 변환 통합 노드
- faster-lio Odometry → UTM 좌표 변환
- GPS NavSatFix → UTM 좌표 변환
- map → odom TF 발행
"""

import rospy
import utm
import tf2_ros
import math
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class SensorFusionUTM:
    def __init__(self):
        rospy.init_node('sensor_fusion_utm')

        # UTM datum 설정 (첫 GPS 위치를 원점으로)
        self.datum_set = False
        self.datum_utm_x = None
        self.datum_utm_y = None
        self.datum_zone = None
        self.datum_letter = None

        # GPS 필터링 (급격한 점프 감지)
        self.prev_gps_x = None
        self.prev_gps_y = None
        self.max_gps_jump = 100.0  # Gazebo 시뮬레이션: 100m 점프까지 허용 (실제: 10m)

        # 원본 GPS 데이터 필터링 (UTM 변환 전)
        self.prev_gps_lat = None
        self.prev_gps_lon = None
        self.prev_utm_x = None
        self.prev_utm_y = None

        # GPS 기반 방향 추정
        self.gps_yaw = None  # GPS 이동 방향
        self.gps_yaw_history = []  # 방향 이력 (평활화용)
        self.min_gps_distance_for_yaw = 1.0  # 방향 계산 최소 이동 거리 (1m)

        # 구독자
        self.gps_sub = rospy.Subscriber('/ublox/fix', NavSatFix, self.gps_callback)
        self.ieskf_sub = rospy.Subscriber('/Odometry', Odometry, self.ieskf_callback)

        # 발행자
        self.ieskf_utm_pub = rospy.Publisher('/odometry/ieskf_utm', Odometry, queue_size=10)
        self.gps_utm_pub = rospy.Publisher('/pose/gps', PoseWithCovarianceStamped, queue_size=10)

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # map → odom TF 발행 (20Hz, 항상 발행)
        rospy.Timer(rospy.Duration(0.05), self.publish_map_odom_tf)

        rospy.loginfo("Sensor Fusion UTM converter initialized")

    def publish_map_odom_tf(self, event=None):
        """map → odom TF 발행 (항상 identity transform)"""
        try:
            t = TransformStamped()
            # 시뮬레이션 시간 사용 (use_sim_time=true)
            current_time = rospy.Time.now()

            # 시간이 0이면 대기 (시뮬레이션 시간 초기화 전)
            if current_time.to_sec() == 0:
                rospy.logwarn_once("⏳ Waiting for simulation time...")
                return

            t.header.stamp = current_time
            t.header.frame_id = "map"
            t.child_frame_id = "odom"

            # Identity transform (map과 odom이 같은 원점)
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.w = 1.0

            self.tf_broadcaster.sendTransform(t)
            rospy.loginfo_once("✅ map → odom TF 발행 시작")
        except Exception as e:
            rospy.logerr(f"❌ TF 발행 실패: {e}")

    def gps_callback(self, msg):
        """GPS → UTM 변환 및 datum 설정"""
        # GPS 유효성 검사
        if msg.latitude == 0.0 and msg.longitude == 0.0:
            rospy.logwarn_throttle(5.0, "Invalid GPS data (0, 0)")
            return

        # 원본 GPS 데이터 검증 (UTM 변환 전)
        if self.prev_gps_lat is not None and self.prev_gps_lon is not None:
            # 위도/경도 기반 거리 계산 (대략적)
            # 1도 위도 = 약 111km, 1도 경도 = 약 111km * cos(위도)
            dlat = abs(msg.latitude - self.prev_gps_lat) * 111000  # m
            dlon = abs(msg.longitude - self.prev_gps_lon) * 111000 * math.cos(math.radians(msg.latitude))  # m
            distance = math.sqrt(dlat**2 + dlon**2)

            if distance > self.max_gps_jump:
                rospy.logwarn(f"GPS lat/lon jump detected: {distance:.2f}m! "
                              f"Prev: ({self.prev_gps_lat:.6f}, {self.prev_gps_lon:.6f}), "
                              f"Curr: ({msg.latitude:.6f}, {msg.longitude:.6f}). Ignoring.")
                return

        # UTM 변환 - datum zone이 설정되어 있으면 강제로 같은 zone 사용
        try:
            if self.datum_set:
                # datum zone으로 강제 변환하여 zone 경계 문제 방지
                utm_x, utm_y, zone, letter = utm.from_latlon(msg.latitude, msg.longitude,
                                                               force_zone_number=self.datum_zone,
                                                               force_zone_letter=self.datum_letter)
            else:
                # 첫 GPS는 자동 zone 결정
                utm_x, utm_y, zone, letter = utm.from_latlon(msg.latitude, msg.longitude)
        except Exception as e:
            rospy.logwarn(f"UTM conversion failed: {e}")
            return

        # UTM 변환 후 추가 검증 (datum 설정 후)
        if self.datum_set:
            # 이전 UTM 좌표와 비교
            if self.prev_utm_x is not None and self.prev_utm_y is not None:
                utm_distance = math.sqrt((utm_x - self.prev_utm_x)**2 + (utm_y - self.prev_utm_y)**2)
                if utm_distance > self.max_gps_jump:
                    # 상세 디버깅 정보 출력
                    rospy.logerr("=" * 80)
                    rospy.logerr("🚨 GPS JUMP DETECTED - DETAILED DEBUG:")
                    rospy.logerr("  [RAW GPS DATA]")
                    rospy.logerr(f"    Previous: lat={self.prev_gps_lat:.9f}°, lon={self.prev_gps_lon:.9f}°")
                    rospy.logerr(f"    Current:  lat={msg.latitude:.9f}°, lon={msg.longitude:.9f}°")

                    # Raw GPS 차이 계산
                    dlat = abs(msg.latitude - self.prev_gps_lat)
                    dlon = abs(msg.longitude - self.prev_gps_lon)
                    # 대략적인 거리 (위도 1° ≈ 111km, 경도는 위도에 따라 다름)
                    raw_distance_approx = math.sqrt((dlat * 111000)**2 + (dlon * 111000 * math.cos(math.radians(msg.latitude)))**2)
                    rospy.logerr(f"    Delta:    dlat={dlat:.9f}° ({dlat*111000:.2f}m), dlon={dlon:.9f}° ({dlon*111000*math.cos(math.radians(msg.latitude)):.2f}m)")
                    rospy.logerr(f"    Approx Distance: {raw_distance_approx:.2f}m")

                    rospy.logerr("  [UTM CONVERSION]")
                    rospy.logerr(f"    Previous: x={self.prev_utm_x:.2f}m, y={self.prev_utm_y:.2f}m (zone {self.datum_zone}{self.datum_letter})")
                    rospy.logerr(f"    Current:  x={utm_x:.2f}m, y={utm_y:.2f}m (zone {zone}{letter})")
                    rospy.logerr(f"    Delta:    dx={abs(utm_x - self.prev_utm_x):.2f}m, dy={abs(utm_y - self.prev_utm_y):.2f}m")
                    rospy.logerr(f"    UTM Distance: {utm_distance:.2f}m")

                    if zone != self.datum_zone or letter != self.datum_letter:
                        rospy.logerr(f"  ⚠️  UTM ZONE CHANGED: {self.datum_zone}{self.datum_letter} → {zone}{letter}")

                    rospy.logerr(f"  [ANALYSIS]")
                    ratio = utm_distance / raw_distance_approx if raw_distance_approx > 0.001 else 0
                    rospy.logerr(f"    UTM/Raw ratio: {ratio:.2f}x")
                    if ratio > 1.5:
                        rospy.logerr(f"    ⚠️  UTM conversion amplified the jump significantly!")
                    elif raw_distance_approx > self.max_gps_jump:
                        rospy.logerr(f"    ⚠️  Raw GPS data itself has large jump (sensor issue)")
                    else:
                        rospy.logerr(f"    ⚠️  Jump is primarily from UTM conversion (coordinate issue)")

                    rospy.logerr("=" * 80)
                    return

        # 현재 GPS 데이터 저장
        self.prev_gps_lat = msg.latitude
        self.prev_gps_lon = msg.longitude
        self.prev_utm_x = utm_x
        self.prev_utm_y = utm_y

        # 첫 GPS 수신 시 datum 설정
        if not self.datum_set:
            self.datum_utm_x = utm_x
            self.datum_utm_y = utm_y
            self.datum_zone = zone
            self.datum_letter = letter
            self.datum_set = True
            rospy.loginfo(f"Datum set: UTM ({utm_x:.2f}, {utm_y:.2f}) zone {zone}{letter}")
            return

        # UTM zone 변경 감지 (주행 중 zone이 바뀌면 좌표계 전환)
        if zone != self.datum_zone or letter != self.datum_letter:
            rospy.logwarn(f"UTM zone changed from {self.datum_zone}{self.datum_letter} to {zone}{letter}!")

            # 이전 zone의 좌표를 새로운 zone으로 변환
            # (간단한 방법: 현재 GPS를 새 datum으로 설정하고 offset 조정)
            # 더 정확한 방법은 이전 datum의 lat/lon을 새 zone으로 재변환

            # 경고: 이 경우 큰 점프가 발생할 수 있으므로 무시
            rospy.logerr(f"UTM zone transition not supported! Ignoring GPS at zone {zone}{letter}")
            return

        # UTM 절대 좌표 사용 (datum 기준 상대 좌표가 아님)
        absolute_x = utm_x
        absolute_y = utm_y

        # GPS 기반 방향 추정 (충분히 이동했을 때만)
        if self.prev_gps_x is not None and self.prev_gps_y is not None:
            dx = absolute_x - self.prev_gps_x
            dy = absolute_y - self.prev_gps_y
            distance = math.sqrt(dx**2 + dy**2)

            if distance >= self.min_gps_distance_for_yaw:
                # GPS 이동 방향 계산 (UTM 좌표계)
                gps_yaw_raw = math.atan2(dy, dx)

                # 방향 이력에 추가 (최근 5개 평균)
                self.gps_yaw_history.append(gps_yaw_raw)
                if len(self.gps_yaw_history) > 5:
                    self.gps_yaw_history.pop(0)

                # 평활화된 GPS 방향
                self.gps_yaw = sum(self.gps_yaw_history) / len(self.gps_yaw_history)

                rospy.loginfo_throttle(5.0, f"📐 GPS 방향 업데이트: {math.degrees(self.gps_yaw):.1f}° (이동: {distance:.2f}m)")

        # GPS 점프 감지 (outlier 필터링)
        if self.prev_gps_x is not None and self.prev_gps_y is not None:
            distance = ((absolute_x - self.prev_gps_x)**2 + (absolute_y - self.prev_gps_y)**2)**0.5
            if distance > self.max_gps_jump:
                rospy.logwarn(f"GPS jump detected: {distance:.2f}m (from [{self.prev_gps_x:.2f}, {self.prev_gps_y:.2f}] to [{absolute_x:.2f}, {absolute_y:.2f}]). Ignoring this GPS measurement.")
                return  # 이상한 GPS 데이터 무시

        # 이전 GPS 위치 저장
        self.prev_gps_x = absolute_x
        self.prev_gps_y = absolute_y

        # PoseWithCovarianceStamped 발행 (EKF의 pose0 입력) - UTM 절대 좌표
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = msg.header.stamp
        pose_msg.header.frame_id = "map"

        pose_msg.pose.pose.position.x = absolute_x
        pose_msg.pose.pose.position.y = absolute_y
        pose_msg.pose.pose.position.z = 0.0

        # 방향은 단위 쿼터니언 (방향 정보 없음)
        pose_msg.pose.pose.orientation.w = 1.0

        # 공분산 설정
        pose_msg.pose.covariance = [0.0] * 36
        pose_msg.pose.covariance[0] = 0.1   # x
        pose_msg.pose.covariance[7] = 0.1   # y
        pose_msg.pose.covariance[14] = 99999  # z (사용 안 함)
        pose_msg.pose.covariance[21] = 99999  # roll
        pose_msg.pose.covariance[28] = 99999  # pitch
        pose_msg.pose.covariance[35] = 99999  # yaw

        self.gps_utm_pub.publish(pose_msg)

        rospy.logdebug(f"GPS→UTM: ({absolute_x:.2f}, {absolute_y:.2f})")

    def ieskf_callback(self, msg):
        """faster-lio Odometry → UTM 좌표 변환"""
        if not self.datum_set:
            return  # datum 설정 전까지 무시

        # faster-lio는 (0,0)에서 시작하므로, datum offset을 더해서 UTM 절대 좌표로 변환
        # 좌표계 변환: faster-lio (x=전진) → GPS UTM (y=북쪽)
        utm_msg = Odometry()
        utm_msg.header.stamp = msg.header.stamp
        utm_msg.header.frame_id = "map"
        utm_msg.child_frame_id = "base_link"

        # 위치 변환: faster-lio → UTM 절대 좌표
        # 1. 좌표축 변환: faster-lio (x=전진, y=좌) → UTM (x=동, y=북)
        # 2. datum offset 추가: UTM 절대 좌표
        utm_msg.pose.pose.position.x = msg.pose.pose.position.y + self.datum_utm_x  # faster-lio y + offset → UTM x (동쪽)
        utm_msg.pose.pose.position.y = msg.pose.pose.position.x + self.datum_utm_y  # faster-lio x + offset → UTM y (북쪽)
        utm_msg.pose.pose.position.z = 0.0  # 2D

        # 방향 변환: GPS 방향 우선, 없으면 faster-lio 방향
        q = msg.pose.pose.orientation
        roll, pitch, yaw_lio = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # GPS 방향이 있으면 GPS 사용, 없으면 faster-lio +90도
        if self.gps_yaw is not None:
            # GPS 방향으로 faster-lio yaw 보정
            # GPS yaw는 이미 UTM 좌표계 (x=동쪽, y=북쪽)
            yaw_utm = self.gps_yaw

            # faster-lio와 GPS 방향 차이 로그
            yaw_lio_utm = yaw_lio + math.pi / 2.0
            yaw_diff = math.degrees(yaw_utm - yaw_lio_utm)
            rospy.loginfo_throttle(10.0, f"🧭 방향 보정: GPS={math.degrees(yaw_utm):.1f}°, "
                                         f"LIO={math.degrees(yaw_lio_utm):.1f}°, "
                                         f"차이={yaw_diff:.1f}°")
        else:
            # GPS 방향 없음: faster-lio 방향 + 좌표축 회전
            yaw_utm = yaw_lio + math.pi / 2.0
            rospy.loginfo_once("⚠️  GPS 방향 없음. faster-lio 방향 사용 (초기 부정확 가능)")

        # quaternion으로 변환
        q_utm = quaternion_from_euler(roll, pitch, yaw_utm)
        utm_msg.pose.pose.orientation.x = q_utm[0]
        utm_msg.pose.pose.orientation.y = q_utm[1]
        utm_msg.pose.pose.orientation.z = q_utm[2]
        utm_msg.pose.pose.orientation.w = q_utm[3]

        # 속도/각속도: faster-lio 값
        utm_msg.twist = msg.twist

        # 공분산
        utm_msg.pose.covariance = msg.pose.covariance
        utm_msg.twist.covariance = msg.twist.covariance

        self.ieskf_utm_pub.publish(utm_msg)

        # map → odom TF는 GPS callback에서만 발행 (중복 방지)


if __name__ == '__main__':
    try:
        node = SensorFusionUTM()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
