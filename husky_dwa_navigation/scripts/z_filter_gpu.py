#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
import threading
from collections import deque

class FastZFilter:
    def __init__(self):
        # 파라미터 설정
        self.z_min = rospy.get_param('~z_min', -0.65)
        self.z_max = rospy.get_param('~z_max', 2.0)
        self.use_threading = rospy.get_param('~use_threading', True)
        self.buffer_size = rospy.get_param('~buffer_size', 2)
        
        # 토픽 설정
        input_topic = rospy.get_param('~input_topic', '/velodyne_points')
        output_topic = rospy.get_param('~output_topic', '/velodyne_points_filtered')
        
        # 발행자/구독자 설정
        self.sub = rospy.Subscriber(input_topic, PointCloud2, self.callback, queue_size=1)
        self.pub = rospy.Publisher(output_topic, PointCloud2, queue_size=1)
        
        # 성능 최적화를 위한 멀티스레딩
        if self.use_threading:
            self.processing_queue = deque(maxlen=self.buffer_size)
            self.processing_lock = threading.Lock()
            self.worker_thread = threading.Thread(target=self.processing_worker)
            self.worker_thread.daemon = True
            self.worker_thread.start()
        
        # 성능 모니터링
        self.process_count = 0
        self.total_time = 0.0
        
        rospy.loginfo(f"Fast Z-Filter initialized: z_range=[{self.z_min}, {self.z_max}]")
        rospy.loginfo(f"Threading: {'Enabled' if self.use_threading else 'Disabled'}")
    
    def callback(self, msg):
        if self.use_threading:
            # 멀티스레드 모드: 큐에 넣고 worker thread에서 처리
            with self.processing_lock:
                self.processing_queue.append(msg)
        else:
            # 싱글스레드 모드: 직접 처리
            self.process_message(msg)
    
    def processing_worker(self):
        """별도 스레드에서 실행되는 처리 워커"""
        while not rospy.is_shutdown():
            msg = None
            with self.processing_lock:
                if self.processing_queue:
                    msg = self.processing_queue.popleft()
            
            if msg is not None:
                self.process_message(msg)
            else:
                rospy.sleep(0.001)  # 1ms 대기
    
    def process_message(self, msg):
        try:
            start_time = rospy.get_time()
            
            # 최적화된 포인트 클라우드 읽기
            points_list = list(pc2.read_points(msg, skip_nans=True, 
                                             field_names=("x", "y", "z", "intensity")))
            
            if not points_list:
                return
            
            # NumPy 배열로 변환 (벡터화된 연산 활용)
            points_array = np.array(points_list, dtype=np.float32)
            
            # Z축 필터링 (매우 빠른 벡터화 연산)
            z_mask = (points_array[:, 2] >= self.z_min) & (points_array[:, 2] <= self.z_max)
            filtered_points = points_array[z_mask]
            
            # 필터링된 포인트를 PointCloud2로 변환 후 발행
            if len(filtered_points) > 0:
                self.publish_pointcloud_fast(filtered_points, msg.header)
            
            # 성능 모니터링
            end_time = rospy.get_time()
            processing_time = end_time - start_time
            self.process_count += 1
            self.total_time += processing_time
            
            if self.process_count % 100 == 0:  # 100개마다 평균 시간 출력
                avg_time = self.total_time / self.process_count
                rospy.loginfo(f"Average processing time: {avg_time*1000:.2f}ms "
                            f"(Optimized CPU, Threading: {'On' if self.use_threading else 'Off'})")
                
        except Exception as e:
            rospy.logerr(f"Error in Z-filter processing: {e}")
    
    def publish_pointcloud_fast(self, points_array, header):
        """최적화된 PointCloud2 발행"""
        # 필드 정의
        fields = [
            pc2.PointField('x', 0, pc2.PointField.FLOAT32, 1),
            pc2.PointField('y', 4, pc2.PointField.FLOAT32, 1),
            pc2.PointField('z', 8, pc2.PointField.FLOAT32, 1),
            pc2.PointField('intensity', 12, pc2.PointField.FLOAT32, 1),
        ]
        
        # 헤더 업데이트
        new_header = Header()
        new_header.stamp = rospy.Time.now()
        new_header.frame_id = header.frame_id
        
        # PointCloud2 생성
        pc_msg = pc2.create_cloud(new_header, fields, points_array.tolist())
        self.pub.publish(pc_msg)

def main():
    rospy.init_node('z_filter_gpu')
    filter_node = FastZFilter()
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()