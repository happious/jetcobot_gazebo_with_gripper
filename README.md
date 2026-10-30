# JetCobot Gazebo + MoveIt 2

Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic에서 JetCobot 로봇팔을 움직이기 위한 소스 패키지 모음입니다. 로봇 메시, 6축 관절 컨트롤러, MoveIt 설정, 고정 베이스, 테이블·큐브 모델과 손목 RGB 카메라를 포함합니다.

## 폴더 구성

| 경로 | 역할 |
|---|---|
| src/jetcobot_description | 로봇 URDF와 STL/DAE 메시 |
| src/jetcobot_moveit_config | MoveIt 계획·실행, SRDF, RViz 설정 |
| src/jetcobot_gazebo | Gazebo 런치, 컨트롤러, 월드, 테이블·큐브·카메라 모델 |

실물용 jetcobot_bringup, Pinky 패키지, build/install/log, 과거 백업은 필요하지 않아 제외했습니다. 별도의 원본 저장소 clone 없이 이 저장소만 빌드하면 됩니다. 사용자별 메시·컨트롤러 경로는 런치가 설치 위치에서 결정합니다.

## 1. 설치

Ubuntu 24.04에 ROS 2 Jazzy가 설치된 환경에서 실행합니다.

~~~bash
source /opt/ros/jazzy/setup.bash
sudo apt update
sudo apt install -y git python3-colcon-common-extensions python3-rosdep \
  ros-jazzy-moveit ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-xacro \
  ros-jazzy-rqt-image-view liburdfdom-tools

# rosdep가 초기화되지 않은 경우에만 실행
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update

git clone https://github.com/happious/jetcobot_gazebo.git ~/jetcobot_ws
cd ~/jetcobot_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install
source install/setup.bash
~~~

이후 새 터미널마다 아래 환경을 불러옵니다.

~~~bash
source /opt/ros/jazzy/setup.bash
source ~/jetcobot_ws/install/setup.bash
~~~

## 2. 실행

권장: 테이블 위 고정 로봇팔 + 큐브 + 손목 카메라 + MoveIt + RViz.

~~~bash
ros2 launch jetcobot_gazebo table_scene.launch.py
~~~

RViz의 MotionPlanning에서 Planning Group을 arm_group으로 선택하고 목표 자세를 조금 이동한 뒤 Plan → Execute를 누릅니다. Gazebo의 로봇팔도 따라 움직입니다. 종료는 실행 터미널에서 Ctrl+C입니다.

다른 장면은 앞선 실행을 종료한 뒤 시작합니다.

~~~bash
# 빈 월드에 고정 베이스 로봇 + MoveIt/RViz
ros2 launch jetcobot_gazebo fixed_base.launch.py moveit:=true

# 테이블·큐브만 표시
ros2 launch jetcobot_gazebo table_scene.launch.py robot:=false

# GUI 없이 테이블 장면 실행 (MoveIt은 유지)
ros2 launch jetcobot_gazebo table_scene.launch.py gui:=false rviz:=false

# MoveIt/RViz 없이 테이블 장면 실행
ros2 launch jetcobot_gazebo table_scene.launch.py moveit:=false rviz:=false
~~~

일반 gazebo.launch.py도 포함하지만 베이스가 고정되지 않은 실험용 장면입니다. 안정적인 관절 동작에는 fixed_base.launch.py 또는 table_scene.launch.py를 권장합니다. 테이블 런치의 robot/gui/moveit/rviz 기본값은 모두 true입니다. robot:=false이면 로봇 관련 노드를 시작하지 않습니다.

Gazebo 런치는 robot_state_publisher, /clock 브리지, joint_state_broadcaster, arm_group_controller를 자동으로 시작합니다. 같은 ROS 도메인에서 demo.launch.py나 다른 Gazebo 로봇 런치를 동시에 실행하면 컨트롤러가 충돌할 수 있습니다.

## 3. 관절 직접 움직이기

장면 실행을 유지하고 새 터미널에서 환경을 불러온 뒤 실행합니다. 아래 목표는 라디안이며 1번 관절을 0.2 rad로, 나머지를 0으로 이동합니다.

~~~bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 action send_goal /arm_group_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: ['1_Joint', '2_Joint', '3_Joint', '4_Joint', '5_Joint', '6_Joint'], points: [{positions: [0.2, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 3, nanosec: 0}}]}}" --feedback
~~~

두 컨트롤러가 active이고 결과 error_code가 0이면 정상입니다.

## 4. 손목 카메라

테이블 로봇 장면에서 활성화됩니다. RGB 640×480, 시뮬레이션 시간 기준 15Hz이며 깊이 센서는 아닙니다.

~~~bash
ros2 run rqt_image_view rqt_image_view /jetcobot/camera/image_raw
ros2 topic echo /jetcobot/camera/camera_info --once
~~~

RViz에서도 Image 표시를 추가해 /jetcobot/camera/image_raw를 선택할 수 있습니다. 카메라는 손목을 따라 움직이며 초기 자세에서 큐브를 향하지 않을 수 있습니다. 광학 프레임은 wrist_camera_optical_frame입니다. gui:=false에서도 카메라 렌더링에는 EGL/OpenGL 환경이 필요합니다.

## 문제 해결

| 증상 | 확인할 내용 |
|---|---|
| 패키지를 찾지 못함 | 빌드 성공 여부, source ~/jetcobot_ws/install/setup.bash |
| 컨트롤러 대기가 계속됨 | Gazebo 로그의 gz_ros2_control 플러그인 오류, 설치된 ros-jazzy-gz-ros2-control |
| Plan 성공, Execute 실패 | 두 컨트롤러 active, /joint_states와 /clock 수신, 중복 실행 여부 |
| 현재 관절 상태를 받지 못함 | ros2 topic info /joint_states, joint_state_broadcaster의 use_local_topics=false |
| 화면이나 카메라가 검게 나옴 | GPU/EGL/OpenGL 및 디스플레이 환경, headless 실행 옵션 |
| 모델 파일을 찾지 못함 | 저장소의 meshes/models 포함 여부, 다시 빌드 후 환경 source |

~~~bash
ros2 control list_controllers
ros2 topic echo /clock --once
ros2 topic info /joint_states
ros2 param get /move_group use_sim_time
~~~

큐브는 Gazebo 물리 물체이며 MoveIt planning scene에 자동 등록되지 않습니다. 자동 파지, 큐브 인식, 실물 제어는 포함하지 않습니다. 카메라 위치·내부 파라미터와 제어 effort/velocity는 시뮬레이션용 설정입니다.

## 출처 및 상세 문서

- 로봇 및 MoveIt 원본: https://github.com/kemjensak/jetcobot_ros2
- [Gazebo 런치 상세](src/jetcobot_gazebo/README.md)
- [테이블·큐브·카메라 상세](src/jetcobot_gazebo/README.table_scene.md)
- 원본 저장소의 LICENSE 및 각 package.xml의 라이선스 선언을 함께 보존했습니다. 모델·텍스처의 추가 출처/이용 조건은 원본 자산을 확인하세요.

원 작업공간의 기존 기록에는 헤드리스 관절 이동, MoveIt 실행, 고정 베이스 및 카메라 수신 검증이 포함되어 있습니다. 이 저장소 구성에 대한 새 검증 결과는 검증 기록 문서를 참고하세요.
