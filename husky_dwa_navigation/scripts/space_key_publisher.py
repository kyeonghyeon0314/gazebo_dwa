#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Space 키 입력을 토픽으로 발행하는 유틸리티 노드
GUI 버튼이나 다른 방법으로 Space 키 이벤트를 발행할 수 있습니다.
"""

import rospy
from std_msgs.msg import Bool
import tkinter as tk
from tkinter import ttk
import threading

class SpaceKeyPublisher:
    def __init__(self):
        rospy.init_node('space_key_publisher', anonymous=True)
        
        # Publisher
        self.space_pub = rospy.Publisher('/space_key_pressed', Bool, queue_size=1)
        
        # GUI 생성
        self.root = tk.Tk()
        self.root.title("GPS Heading Calibration")
        self.root.geometry("400x200")
        self.root.configure(bg='#f0f0f0')
        
        # 메인 프레임
        main_frame = ttk.Frame(self.root, padding="20")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 제목
        title_label = ttk.Label(main_frame, text="GPS Heading Calibration", 
                               font=('Arial', 16, 'bold'))
        title_label.grid(row=0, column=0, pady=(0, 20))
        
        # 설명
        desc_label = ttk.Label(main_frame, 
                              text="아래 버튼을 클릭하여 GPS 기반\nheading 캘리브레이션을 시작하세요.",
                              font=('Arial', 10),
                              justify=tk.CENTER)
        desc_label.grid(row=1, column=0, pady=(0, 20))
        
        # 시작 버튼
        self.start_button = ttk.Button(main_frame, 
                                      text="🚀 Heading 캘리브레이션 시작", 
                                      command=self.send_space_signal,
                                      style='Accent.TButton')
        self.start_button.grid(row=2, column=0, pady=(0, 10), ipadx=20, ipady=10)
        
        # 상태 라벨
        self.status_label = ttk.Label(main_frame, 
                                     text="준비됨", 
                                     font=('Arial', 9),
                                     foreground='green')
        self.status_label.grid(row=3, column=0)
        
        # 스타일 설정
        style = ttk.Style()
        style.configure('Accent.TButton', font=('Arial', 12, 'bold'))
        
        # 중앙 정렬
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)
        
        rospy.loginfo("Space Key Publisher GUI가 시작되었습니다.")
        
    def send_space_signal(self):
        """Space 키 신호 발송"""
        try:
            msg = Bool()
            msg.data = True
            self.space_pub.publish(msg)
            
            rospy.loginfo("Space 키 신호를 발송했습니다.")
            
            # 버튼 상태 업데이트
            self.start_button.configure(text="✅ 신호 발송됨", state='disabled')
            self.status_label.configure(text="캘리브레이션 신호 발송됨", foreground='blue')
            
            # 3초 후 버튼 재활성화
            self.root.after(3000, self.reset_button)
            
        except Exception as e:
            rospy.logerr(f"Space 키 신호 발송 실패: {e}")
            self.status_label.configure(text="신호 발송 실패", foreground='red')
    
    def reset_button(self):
        """버튼 상태 초기화"""
        self.start_button.configure(text="🚀 Heading 캘리브레이션 시작", state='normal')
        self.status_label.configure(text="준비됨", foreground='green')
    
    def run(self):
        """GUI 실행"""
        try:
            # ROS 스핀을 별도 스레드에서 실행
            ros_thread = threading.Thread(target=self.ros_spin)
            ros_thread.daemon = True
            ros_thread.start()
            
            # GUI 메인 루프
            self.root.mainloop()
            
        except Exception as e:
            rospy.logerr(f"GUI 실행 오류: {e}")
        finally:
            self.root.quit()
    
    def ros_spin(self):
        """ROS 스핀 (별도 스레드)"""
        while not rospy.is_shutdown():
            rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        space_publisher = SpaceKeyPublisher()
        space_publisher.run()
    except rospy.ROSInterruptException:
        pass