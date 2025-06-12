#!/usr/bin/env python3

import rospy
import json
import math
import numpy as np
from collections import deque, defaultdict
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3Stamped
import time

class GPSDiagnosticTool:
    """GPS 센서 종합 진단 도구 - Citysim Gazebo 월드 대응"""
    
    def __init__(self):
        rospy.init_node('gps_diagnostic_tool', anonymous=True)
        
        # 진단 데이터 저장
        self.gps_data_buffer = deque(maxlen=1000)  # 최근 1000개 GPS 데이터
        self.fix_quality_history = deque(maxlen=100)
        self.position_history = deque(maxlen=200)
        
        # 통계 정보
        self.stats = {
            "total_messages": 0,
            "valid_messages": 0,
            "invalid_messages": 0,
            "no_fix_count": 0,
            "fix_count": 0,
            "rtk_count": 0,
            "start_time": rospy.Time.now().to_sec()
        }
        
        # 품질 분석
        self.quality_metrics = {
            "position_drift": 0.0,
            "avg_accuracy": 0.0,
            "max_accuracy_error": 0.0,
            "position_noise": 0.0,
            "update_rate": 0.0,
            "satellite_count": 0,
            "hdop": 999.0,
            "vdop": 999.0,
            "pdop": 999.0
        }
        
        # 문제점 탐지
        self.issues = {
            "low_satellite_count": False,
            "high_hdop": False,
            "position_jumping": False,
            "low_update_rate": False,
            "frequent_fix_loss": False,
            "poor_accuracy": False,
            "gazebo_simulation_issues": False
        }
        
        # GPS 메시지 타임스탬프 추적
        self.last_gps_time = None
        self.update_intervals = deque(maxlen=50)
        
        # Publishers
        self.diagnostic_pub = rospy.Publisher("/gps_diagnostics", String, queue_size=10)
        self.quality_pub = rospy.Publisher("/gps_quality_metrics", String, queue_size=10)
        self.status_pub = rospy.Publisher("/gps_status_summary", String, queue_size=10)
        
        # Subscribers
        rospy.Subscriber("/ublox/fix", NavSatFix, self.gps_callback)
        
        # 진단 타이머
        rospy.Timer(rospy.Duration(2.0), self.analyze_gps_quality)
        rospy.Timer(rospy.Duration(5.0), self.detect_issues)
        rospy.Timer(rospy.Duration(1.0), self.publish_status)
        rospy.Timer(rospy.Duration(10.0), self.comprehensive_report)
        
        rospy.loginfo("🔍 GPS 진단 도구 시작! (Citysim Gazebo 월드 대응)")
        rospy.loginfo("📡 GPS 토픽: /ublox/fix")
        rospy.loginfo("📊 진단 결과: /gps_diagnostics, /gps_quality_metrics, /gps_status_summary")

    def gps_callback(self, msg):
        """GPS 메시지 분석"""
        current_time = rospy.Time.now().to_sec()
        
        # 메시지 수신 간격 계산
        if self.last_gps_time is not None:
            interval = current_time - self.last_gps_time
            self.update_intervals.append(interval)
        self.last_gps_time = current_time
        
        # GPS 데이터 저장
        gps_data = {
            "timestamp": current_time,
            "header_stamp": msg.header.stamp.to_sec(),
            "latitude": msg.latitude,
            "longitude": msg.longitude,
            "altitude": msg.altitude,
            "status": msg.status.status,
            "service": msg.status.service,
            "position_covariance": list(msg.position_covariance),
            "covariance_type": msg.position_covariance_type
        }
        
        self.gps_data_buffer.append(gps_data)
        self.stats["total_messages"] += 1
        
        # GPS Fix 상태 분석
        self.analyze_fix_status(msg)
        
        # 위치 정확도 분석
        self.analyze_position_accuracy(gps_data)
        
        # 실시간 진단
        self.real_time_diagnostics(gps_data)
        
        # Gazebo 시뮬레이션 특별 진단
        self.diagnose_gazebo_simulation(gps_data)

    def diagnose_gazebo_simulation(self, gps_data):
        """Gazebo 시뮬레이션 특별 진단 - 월드 (0,0) 기준"""
        # 월드가 (0,0) 기준이므로 이 근처 좌표는 정상
        lat_threshold = 0.01  # ±0.01도 내외는 정상
        lon_threshold = 0.01
        
        if abs(gps_data["latitude"]) > lat_threshold or abs(gps_data["longitude"]) > lon_threshold:
            rospy.loginfo_throttle(30, f"📍 GPS 좌표 정상: ({gps_data['latitude']:.6f}, {gps_data['longitude']:.6f})")
        else:
            rospy.loginfo_throttle(30, f"📍 월드 기준점 근처 GPS: ({gps_data['latitude']:.6f}, {gps_data['longitude']:.6f})")
        
        # 타임스탬프 일관성 체크 (시뮬레이션에서 중요)
        time_diff = abs(gps_data["timestamp"] - gps_data["header_stamp"])
        if time_diff > 1.0:  # 1초 이상 차이
            rospy.logwarn_throttle(15, f"⚠️ GPS 타임스탬프 불일치: {time_diff:.2f}초 차이")
        
        # 월드 (0,0) 기준 좌표계 확인
        distance_from_origin = math.sqrt(gps_data["latitude"]**2 + gps_data["longitude"]**2)
        if distance_from_origin < 0.001:  # 원점에서 매우 가까운 경우
            self.issues["gazebo_simulation_issues"] = False  # 이는 정상
        elif distance_from_origin > 1.0:  # 1도 이상 떨어진 경우
            rospy.logwarn_throttle(20, f"⚠️ GPS가 월드 기준점에서 너무 멀리 떨어져 있습니다: {distance_from_origin:.3f}도")
            self.issues["gazebo_simulation_issues"] = True

    def analyze_fix_status(self, msg):
        """GPS Fix 상태 분석"""
        status = msg.status.status
        
        if status >= 0:  # Valid fix
            self.stats["valid_messages"] += 1
            if status == 0:
                self.stats["fix_count"] += 1
            elif status >= 1:
                self.stats["rtk_count"] += 1
            
            self.fix_quality_history.append({
                "timestamp": rospy.Time.now().to_sec(),
                "status": status,
                "service": msg.status.service
            })
        else:  # No fix
            self.stats["invalid_messages"] += 1
            self.stats["no_fix_count"] += 1

    def analyze_position_accuracy(self, gps_data):
        """위치 정확도 분석"""
        # 위치 이력 저장
        position = {
            "lat": gps_data["latitude"],
            "lon": gps_data["longitude"],
            "alt": gps_data["altitude"],
            "timestamp": gps_data["timestamp"],
            "covariance": gps_data["position_covariance"]
        }
        
        self.position_history.append(position)
        
        # 위치 노이즈 계산
        if len(self.position_history) >= 10:
            self.calculate_position_noise()
        
        # 공분산 매트릭스에서 정확도 정보 추출
        self.extract_accuracy_metrics(gps_data)

    def calculate_position_noise(self):
        """위치 노이즈 계산"""
        if len(self.position_history) < 10:
            return
        
        # 최근 위치들의 표준편차 계산
        recent_positions = list(self.position_history)[-20:]
        
        lats = [p["lat"] for p in recent_positions]
        lons = [p["lon"] for p in recent_positions]
        
        lat_std = np.std(lats) if len(lats) > 1 else 0.0
        lon_std = np.std(lons) if len(lons) > 1 else 0.0
        
        # 미터 단위로 변환 (대략적)
        lat_noise_m = lat_std * 111320  # 1도 = 약 111.32km
        lon_noise_m = lon_std * 111320 * math.cos(math.radians(np.mean(lats))) if lats else 0.0
        
        self.quality_metrics["position_noise"] = math.sqrt(lat_noise_m**2 + lon_noise_m**2)

    def extract_accuracy_metrics(self, gps_data):
        """공분산 매트릭스에서 정확도 메트릭 추출"""
        covariance = gps_data["position_covariance"]
        
        if len(covariance) >= 9 and covariance[0] > 0:
            # 대각선 요소들이 분산값
            x_var = covariance[0]  # East 방향 분산
            y_var = covariance[4]  # North 방향 분산
            z_var = covariance[8]  # Up 방향 분산
            
            # 수평 정확도 (CEP: Circular Error Probable)
            horizontal_accuracy = math.sqrt(x_var + y_var)
            self.quality_metrics["avg_accuracy"] = horizontal_accuracy
            
            # HDOP, VDOP, PDOP 추정 (정확하지 않지만 대략적)
            if horizontal_accuracy > 0:
                estimated_hdop = horizontal_accuracy / 3.0  # 대략적 추정
                self.quality_metrics["hdop"] = min(estimated_hdop, 50.0)

    def real_time_diagnostics(self, gps_data):
        """실시간 진단"""
        # 위치 점프 탐지
        if len(self.position_history) >= 2:
            prev_pos = list(self.position_history)[-2]
            curr_pos = list(self.position_history)[-1]
            
            # 거리 계산 (대략적)
            lat_diff = curr_pos["lat"] - prev_pos["lat"]
            lon_diff = curr_pos["lon"] - prev_pos["lon"]
            
            distance_m = math.sqrt((lat_diff * 111320)**2 + (lon_diff * 111320)**2)
            time_diff = curr_pos["timestamp"] - prev_pos["timestamp"]
            
            if time_diff > 0:
                speed_mps = distance_m / time_diff
                
                # 비현실적인 속도 탐지 (50m/s = 180km/h, Gazebo 환경 고려)
                if speed_mps > 50:
                    rospy.logwarn(f"⚠️ GPS 위치 점프 탐지: {distance_m:.1f}m in {time_diff:.1f}s ({speed_mps:.1f}m/s)")
                    self.issues["position_jumping"] = True

    def analyze_gps_quality(self, event):
        """GPS 품질 분석"""
        if len(self.gps_data_buffer) < 5:
            return
        
        # 업데이트 주파수 계산
        if len(self.update_intervals) > 5:
            avg_interval = np.mean(list(self.update_intervals))
            self.quality_metrics["update_rate"] = 1.0 / avg_interval if avg_interval > 0 else 0.0
        
        # Fix 품질 계산
        if len(self.fix_quality_history) > 5:
            valid_fixes = [f for f in self.fix_quality_history if f["status"] >= 0]
            fix_ratio = len(valid_fixes) / len(self.fix_quality_history)
            
            if fix_ratio < 0.8:
                self.issues["frequent_fix_loss"] = True

    def detect_issues(self, event):
        """문제점 탐지"""
        # 업데이트 주파수 체크
        if self.quality_metrics["update_rate"] < 5.0:  # 5Hz 이하
            self.issues["low_update_rate"] = True
        
        # HDOP 체크
        if self.quality_metrics["hdop"] > 5.0:
            self.issues["high_hdop"] = True
        
        # 위치 정확도 체크
        if self.quality_metrics["avg_accuracy"] > 10.0:  # 10m 이상 오차
            self.issues["poor_accuracy"] = True
        
        # 노이즈 레벨 체크
        if self.quality_metrics["position_noise"] > 5.0:  # 5m 이상 노이즈
            rospy.logwarn_throttle(10, f"⚠️ 높은 GPS 노이즈: {self.quality_metrics['position_noise']:.1f}m")

    def publish_status(self, event):
        """GPS 상태 요약 발행"""
        current_time = rospy.Time.now().to_sec()
        runtime = current_time - self.stats["start_time"]
        
        status_summary = {
            "timestamp": current_time,
            "runtime_seconds": runtime,
            "gps_status": "ACTIVE" if len(self.gps_data_buffer) > 0 and (current_time - self.gps_data_buffer[-1]["timestamp"]) < 2.0 else "INACTIVE",
            "update_rate_hz": round(self.quality_metrics["update_rate"], 1),
            "position_accuracy_m": round(self.quality_metrics["avg_accuracy"], 2),
            "position_noise_m": round(self.quality_metrics["position_noise"], 2),
            "hdop": round(self.quality_metrics["hdop"], 2),
            "fix_ratio": round(self.stats["valid_messages"] / max(self.stats["total_messages"], 1), 3),
            "total_messages": self.stats["total_messages"],
            "active_issues": [issue for issue, active in self.issues.items() if active],
            "latest_gps": self.gps_data_buffer[-1] if self.gps_data_buffer else None
        }
        
        self.status_pub.publish(json.dumps(status_summary, indent=2))

    def comprehensive_report(self, event):
        """종합 진단 보고서"""
        if len(self.gps_data_buffer) < 10:
            return
        
        report = {
            "timestamp": rospy.Time.now().to_sec(),
            "diagnostic_summary": {
                "overall_status": self.get_overall_status(),
                "confidence_level": self.calculate_confidence_level(),
                "recommendations": self.get_recommendations()
            },
            "statistics": self.stats.copy(),
            "quality_metrics": self.quality_metrics.copy(),
            "detected_issues": {k: v for k, v in self.issues.items() if v},
            "recent_gps_samples": list(self.gps_data_buffer)[-5:],  # 최근 5개 샘플
            "gazebo_diagnostics": self.diagnose_gazebo_environment(),
            "citysim_world_info": self.analyze_citysim_world()
        }
        
        self.diagnostic_pub.publish(json.dumps(report, indent=2))
        
        # 콘솔 출력
        self.print_diagnostic_summary(report)

    def diagnose_gazebo_environment(self):
        """Gazebo 환경 진단 - 월드 (0,0) 기준"""
        gazebo_status = {
            "simulation_detected": True,
            "world_name": "citysim_gazebo.world",
            "world_spherical_coordinates": {
                "latitude_deg": 0,
                "longitude_deg": 0,
                "elevation": 0,
                "heading_deg": 0
            },
            "gps_plugin_status": "ACTIVE" if self.stats["total_messages"] > 0 else "INACTIVE",
            "recommended_settings": {
                "updateRate": "10.0 또는 20.0",
                "referenceLatitude": "0.0 (월드와 일치)",
                "referenceLongitude": "0.0 (월드와 일치)",
                "referenceAltitude": "0.0",
                "gaussianNoise": "0.001 0.001 0.002 (높은 정밀도)",
                "drift": "0.0 0.0 0.0 (드리프트 없음)"
            },
            "coordinate_consistency": self.check_coordinate_consistency(),
            "plugin_type": "hector_gazebo_ros_gps"
        }
        
        return gazebo_status
    
    def check_coordinate_consistency(self):
        """좌표계 일관성 체크"""
        if not self.gps_data_buffer:
            return "NO_DATA"
        
        latest_gps = self.gps_data_buffer[-1]
        distance_from_expected = math.sqrt(
            (latest_gps["latitude"] - 0.0)**2 + 
            (latest_gps["longitude"] - 0.0)**2
        )
        
        if distance_from_expected < 0.01:  # 1% 도 이내
            return "EXCELLENT"
        elif distance_from_expected < 0.1:  # 10% 도 이내
            return "GOOD"
        else:
            return "POOR"

    def analyze_citysim_world(self):
        """Citysim 월드 분석 - (0,0) 기준"""
        if not self.gps_data_buffer:
            return {"status": "NO_GPS_DATA"}
        
        latest_gps = self.gps_data_buffer[-1]
        
        # 월드 (0,0) 기준점으로부터의 거리
        distance_from_world_origin = math.sqrt(
            latest_gps["latitude"]**2 + latest_gps["longitude"]**2
        )
        
        world_info = {
            "coordinate_system": "WGS84 (월드 기준: 0,0)",
            "world_reference": {
                "latitude": 0.0,
                "longitude": 0.0,
                "elevation": 0.0,
                "heading": 0.0
            },
            "current_position": {
                "latitude": latest_gps["latitude"],
                "longitude": latest_gps["longitude"],
                "altitude": latest_gps["altitude"]
            },
            "distance_from_world_origin_deg": distance_from_world_origin,
            "distance_from_world_origin_m": distance_from_world_origin * 111320,  # 대략적 미터 변환
            "movement_detected": len(self.position_history) > 5,
            "position_variance": self.quality_metrics["position_noise"],
            "coordinate_status": "NORMAL" if distance_from_world_origin < 0.1 else "FAR_FROM_ORIGIN"
        }
        
        return world_info

    def get_overall_status(self):
        """전체 GPS 상태 평가"""
        issue_count = sum(1 for issue in self.issues.values() if issue)
        
        if issue_count == 0:
            return "EXCELLENT"
        elif issue_count <= 2:
            return "GOOD"
        elif issue_count <= 4:
            return "FAIR"
        else:
            return "POOR"

    def calculate_confidence_level(self):
        """GPS 신뢰도 레벨 계산 (0-100)"""
        confidence = 100
        
        # Fix 비율에 따른 감점
        fix_ratio = self.stats["valid_messages"] / max(self.stats["total_messages"], 1)
        confidence -= (1.0 - fix_ratio) * 30
        
        # 정확도에 따른 감점
        if self.quality_metrics["avg_accuracy"] > 5.0:
            confidence -= 20
        elif self.quality_metrics["avg_accuracy"] > 2.0:
            confidence -= 10
        
        # 업데이트 주파수에 따른 감점
        if self.quality_metrics["update_rate"] < 5.0:
            confidence -= 15
        
        # HDOP에 따른 감점
        if self.quality_metrics["hdop"] > 5.0:
            confidence -= 20
        
        # 노이즈에 따른 감점
        if self.quality_metrics["position_noise"] > 3.0:
            confidence -= 15
        
        return max(0, min(100, confidence))

    def get_recommendations(self):
        """개선 권장사항 - Gazebo/Citysim (0,0) 월드 특화"""
        recommendations = []
        
        if self.issues["gazebo_simulation_issues"]:
            recommendations.append("GPS 좌표가 월드 기준점(0,0)에서 너무 멀리 떨어져 있습니다. 월드 spherical_coordinates 확인")
        
        if self.issues["low_update_rate"]:
            recommendations.append("GPS 플러그인 updateRate를 10.0으로 설정 (custom_description.gazebo.xacro)")
        
        if self.issues["high_hdop"]:
            recommendations.append("GPS gaussianNoise를 0.001 0.001 0.002로 낮게 설정하여 정밀도 향상")
        
        if self.issues["position_jumping"]:
            recommendations.append("GPS drift를 0.0 0.0 0.0으로 설정하여 위치 점프 방지")
        
        if self.issues["frequent_fix_loss"]:
            recommendations.append("GPS alwaysOn=true 설정하고 Gazebo 시뮬레이션 real_time_factor 확인")
        
        if self.issues["poor_accuracy"]:
            recommendations.append("월드 spherical_coordinates와 GPS referenceLatitude/Longitude 일치 확인 (둘 다 0.0)")
        
        # 월드 설정 확인 권장사항
        recommendations.append("월드 설정 확인: spherical_coordinates latitude/longitude = 0.0")
        recommendations.append("GPS 설정 확인: referenceLatitude/referenceLongitude = 0.0")
        
        if not any(self.issues.values()):
            recommendations = ["GPS가 월드 (0,0) 기준으로 정상 작동 중입니다!"]
        
        return recommendations

    def print_diagnostic_summary(self, report):
        """진단 요약 콘솔 출력 - Citysim (0,0) 월드 특화"""
        print("\n" + "="*70)
        print("🔍 GPS 진단 보고서 (Citysim Gazebo World - 0,0 기준)")
        print("="*70)
        
        status = report["diagnostic_summary"]["overall_status"]
        confidence = report["diagnostic_summary"]["confidence_level"]
        
        status_emoji = {"EXCELLENT": "🟢", "GOOD": "🟡", "FAIR": "🟠", "POOR": "🔴"}.get(status, "❓")
        
        print(f"📊 전체 상태: {status_emoji} {status} (신뢰도: {confidence:.0f}%)")
        print(f"📡 업데이트 주파수: {self.quality_metrics['update_rate']:.1f} Hz")
        print(f"🎯 위치 정확도: {self.quality_metrics['avg_accuracy']:.2f} m")
        print(f"📈 위치 노이즈: {self.quality_metrics['position_noise']:.2f} m")
        print(f"🛰️ HDOP: {self.quality_metrics['hdop']:.2f}")
        print(f"✅ Fix 성공률: {(self.stats['valid_messages']/max(self.stats['total_messages'],1)*100):.1f}%")
        
        # 현재 GPS 위치 표시 (월드 기준점과의 관계)
        if self.gps_data_buffer:
            latest = self.gps_data_buffer[-1]
            distance_from_origin = math.sqrt(latest['latitude']**2 + latest['longitude']**2)
            print(f"📍 현재 위치: ({latest['latitude']:.6f}, {latest['longitude']:.6f})")
            print(f"🌍 월드 기준점(0,0)으로부터: {distance_from_origin:.6f}도 ({distance_from_origin*111320:.1f}m)")
        
        if report["detected_issues"]:
            print(f"\n⚠️ 탐지된 문제점:")
            for issue in report["detected_issues"]:
                print(f"   • {issue}")
        
        if report["diagnostic_summary"]["recommendations"]:
            print(f"\n💡 권장사항 (Gazebo/Citysim 0,0 기준):")
            for rec in report["diagnostic_summary"]["recommendations"]:
                print(f"   • {rec}")
        
        print("="*70)

    def generate_quick_commands(self):
        """빠른 진단 명령어들"""
        print("\n🔧 Citysim Gazebo GPS 진단 명령어:")
        print("─────────────────────────────────")
        print("rostopic echo /ublox/fix")
        print("rostopic hz /ublox/fix")
        print("rostopic echo /gps_status_summary")
        print("rostopic echo /gps_diagnostics | jq .")
        print("rosparam get /gazebo/gps_controller")
        print("─────────────────────────────────")

if __name__ == '__main__':
    try:
        diagnostic_tool = GPSDiagnosticTool()
        diagnostic_tool.generate_quick_commands()
        
        rospy.loginfo("🎯 GPS 진단 도구 실행 중... (Citysim Gazebo)")
        rospy.loginfo("📊 실시간 상태: rostopic echo /gps_status_summary")
        rospy.loginfo("📋 상세 진단: rostopic echo /gps_diagnostics")
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 GPS 진단 도구 종료")
    except Exception as e:
        rospy.logerr(f"❌ 진단 도구 오류: {e}")