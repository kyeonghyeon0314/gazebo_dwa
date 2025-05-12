# gazebo_velodyne_dwa
gazebo환경에 husky와 velodyne 라이다 추가하고 dwa를 통한 장애물 회피 기동


<p align='center'>
    <img src="docs/images/demo.gif" alt="drawing" width="800"/>
</p>


# TODO
```
mkdir -p ~/husky_ws/src
cd husky_ws
catkin_make

cd src
git clone https://github.com/kyeonghyeon0314/gazebo_velodyne_dwa.git

sudo apt-get install ros-<ros_distro>-gazebo-ros \
                     ros-<ros_distro>-roscpp \
                     ros-<ros_distro>-sensor-msgs \
                     ros-<ros_distro>-tf \
                     ros-<ros_distro>-velodyne-gazebo-plugins \
                     ros-<ros_distro>-husky-description \
                     ros-<ros_distro>-husky-gazebo \
                     ros-<ros_distro>-dwa-local-planner \
                     ros-<ros_distro>-move-base

cd ..
catkin_make

roslaunch husky_dwa_navigation husky_velodyne_dwa.launch


```
