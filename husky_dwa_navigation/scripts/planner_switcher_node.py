#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from dynamic_reconfigure.msg import Config, StrParameter
from dynamic_reconfigure.srv import Reconfigure

class PlannerSwitcher:
    def __init__(self):
        # TF 버퍼 초기화
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 파라미터 로드
        self.costmap_topic = rospy.get_param('~global_costmap_topic', '/move_base/global_costmap/costmap')
        self.switch_service = rospy.get_param('~switch_service', '/move_base/set_parameters')
        self.planner_param = rospy.get_param('~planner_type_parameter', '/move_base/base_global_planner')
        self.default_planner = rospy.get_param('~default_planner', 'global_planner/GlobalPlanner')
        self.alternate_planner = rospy.get_param('~alternate_planner', 'carrot_planner/CarrotPlanner')
        
        # 상태 변수
        self.costmap = None
        self.current_goal = None
        self.using_default_planner = True
        self.have_costmap = False
        
        # 구독자 설정
        self.costmap_sub = rospy.Subscriber(self.costmap_topic, OccupancyGrid, self.costmap_callback)
        
        # 목표 지점 구독
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        self.current_goal_sub = rospy.Subscriber('/move_base/current_goal', PoseStamped, self.current_goal_callback)
        
        rospy.loginfo("플래너 전환기 초기화 완료. 목표 지점 위치에 따라 %s와 %s 사이를 전환합니다.", 
                     self.default_planner, self.alternate_planner)
    
    def costmap_callback(self, costmap_msg):
        """
        Costmap 메시지를 받아서 저장하는 콜백
        """
        self.costmap = costmap_msg
        self.have_costmap = True
        
        # costmap이 업데이트되면 현재 목표 위치에 따라 플래너 재확인
        if self.current_goal is not None:
            self.check_goal_position(self.current_goal)
    
    def goal_callback(self, goal_msg):
        """
        새 목표 지점이 설정되었을 때의 콜백
        """
        # 새 목표 지점 설정 시 해당 위치에 따라 플래너 전환
        self.check_goal_position(goal_msg)
    
    def current_goal_callback(self, goal_msg):
        """
        현재 목표 지점 업데이트 콜백
        """
        self.current_goal = goal_msg
        self.check_goal_position(goal_msg)
    
    def check_goal_position(self, goal_msg):
        """
        목표 위치가 costmap 내부인지 외부인지 확인하고 플래너 전환
        """
        if not self.have_costmap:
            rospy.logdebug("아직 costmap을 받지 못했습니다. 목표 지점 위치를 확인할 수 없습니다.")
            return
        
        # 목표 지점의 좌표를 costmap 프레임으로 변환
        try:
            # 만약 goal_msg의 프레임이 costmap과 다르다면 변환
            if goal_msg.header.frame_id != self.costmap.header.frame_id:
                goal_in_costmap_frame = self.tf_buffer.transform(goal_msg, self.costmap.header.frame_id, rospy.Duration(1.0))
            else:
                goal_in_costmap_frame = goal_msg
            
            # 목표 지점 좌표
            goal_x = goal_in_costmap_frame.pose.position.x
            goal_y = goal_in_costmap_frame.pose.position.y
            
            # costmap 경계 확인
            map_origin_x = self.costmap.info.origin.position.x
            map_origin_y = self.costmap.info.origin.position.y
            map_width = self.costmap.info.width * self.costmap.info.resolution
            map_height = self.costmap.info.height * self.costmap.info.resolution
            
            # 목표 지점이 costmap 경계 밖에 있는지 확인
            is_outside = (goal_x < map_origin_x or 
                          goal_y < map_origin_y or
                          goal_x > map_origin_x + map_width or
                          goal_y > map_origin_y + map_height)
            
            # 필요에 따라 플래너 전환
            if is_outside and self.using_default_planner:
                rospy.loginfo("목표 지점이 글로벌 costmap 경계 밖에 있습니다. CarrotPlanner로 전환합니다.")
                rospy.loginfo("목표 위치: (%.2f, %.2f), 맵 경계: (%.2f, %.2f) ~ (%.2f, %.2f)", 
                              goal_x, goal_y, map_origin_x, map_origin_y, 
                              map_origin_x + map_width, map_origin_y + map_height)
                self.switch_planner(self.alternate_planner)
                self.using_default_planner = False
            elif not is_outside and not self.using_default_planner:
                rospy.loginfo("목표 지점이 글로벌 costmap 경계 내부에 있습니다. GlobalPlanner로 전환합니다.")
                self.switch_planner(self.default_planner)
                self.using_default_planner = True
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("목표 지점 변환 중 오류: %s", str(e))
    
    def switch_planner(self, planner_type):
        """
        dynamic_reconfigure를 사용하여 플래너 유형 변경
        """
        try:
            # dynamic_reconfigure 서비스 클라이언트 설정
            client = rospy.ServiceProxy(self.switch_service, Reconfigure)
            
            # 설정 메시지 생성
            config = Config()
            string_param = StrParameter()
            string_param.name = "base_global_planner"
            string_param.value = planner_type
            config.strs = [string_param]
            
            # 서비스 호출
            response = client(config)
            rospy.loginfo("플래너 전환 완료: %s", planner_type)
            
        except rospy.ServiceException as e:
            rospy.logerr("서비스 호출 실패: %s", str(e))

def main():
    rospy.init_node('planner_switcher_node')
    
    switcher = PlannerSwitcher()
    
    rospy.loginfo("플래너 전환 노드가 실행 중입니다...")
    rospy.spin()

if __name__ == '__main__':
    main()