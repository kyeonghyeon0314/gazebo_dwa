# 개요
gazebo환경에 husky와 velodyne 라이다 추가하고 dwa를 통한 장애물 회피 기동

<div align="center">
  <div style="margin-bottom: 10px;">
    <img src="docs/video/dwa_final.gif" alt="drawing" width="100%"/>
    <p style="text-align: center;">시연영상 4배속</p>
  </div>
</div>

<div align="center">
  <div style="margin-bottom: 10px;">
    <img src="docs/images/rqt_final.png" width="100%">
    <p style="text-align: center;">rqt_graph</p>
  </div>
</div>

# 주요 적용 사항
## husky_customization [[Link](https://github.com/husky/husky_customization)]
- husky에 HDL-32E 추가
### husky_velodyne.launch 수행내용
clearpath_playpen.world에 HDL-32E센서가 부착된 Husky A200 스폰 및 /velodyne_points 토픽 발행

## husky_dwa_navigation
- dwa_planner 패키지 사용
- pcl_manager를 이용하여 PointCluod 필터링
- config 파일 작성을 통한 각종 파라미터 최적화


# 프로젝트의 목표에 맞는 개선 사항[[NavigiationManager](husky_dwa_navigation/scripts/navigation_manager_node.py)]

본 [프로젝트](https://github.com/kyeonghyeon0314/25_Jairlab)를 위한 **NavigationManager**는 GPS waypoints 기반의 장거리 자율 주행을 위해 설계되었습니다. 기존의 ROS navigation stack이 갖는 로컬 costmap 범위 제한과 원거리 목표 접근의 한계를 극복하기 위한 지능형 내비게이션 시스템을 구현했습니다.

### 주요기능

- **실시간 중간 목표 생성 :** 로봇의 로컬 costmap 범위를 벗어난 목표를 처리하기 위해 중간 목표를 동적으로 생성
- **장애물 회피 전략:** 경로 계획 실패 시 중간 목표를 좌우로 이동시켜 장애물 우회
- **원본 목표 로 전환 :** Costmap 내에 원본 목표 존재시 중간 목표에서 원본 목표로 전환


# TODO
```
mkdir -p ~/husky_ws/src
cd husky_ws
catkin_make

cd src
git clone https://github.com/kyeonghyeon0314/gazebo_velodyne_dwa.git

sudo apt-get install ros-noetic-gazebo-ros \
                     ros-noetic-roscpp \
                     ros-noetic-sensor-msgs \
                     ros-noetic-tf \
                     ros-noetic-velodyne-gazebo-plugins \
                     ros-noetic-husky-description \
                     ros-noetic-husky-gazebo \
                     ros-noetic-dwa-local-planner \
                     ros-noetic-move-base

cd ..
catkin_make

roslaunch husky_dwa_navigation husky_velodyne_dwa.launch

```
