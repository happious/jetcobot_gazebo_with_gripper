# JetCobot Gazebo 실행

빈 월드, Gazebo, robot_state_publisher, clock 브리지, JetCobot 소환, 관절 컨트롤러를 한 launch로 실행합니다. 로봇 생성 성공 후 joint_state_broadcaster, arm_group_controller를 순서대로 활성화합니다. 선택적으로 MoveIt/RViz도 시작합니다. 원본 URDF를 실행 시 변환하므로 사용자 홈 경로를 하드코딩하지 않습니다.

~~~bash
source /opt/ros/jazzy/setup.bash
source ~/jetcobot_ws/install/setup.bash
ros2 launch jetcobot_gazebo gazebo.launch.py
~~~

MoveIt과 RViz도 함께 실행:

~~~bash
ros2 launch jetcobot_gazebo gazebo.launch.py moveit:=true
~~~

화면 없이 서버만 실행:

~~~bash
ros2 launch jetcobot_gazebo gazebo.launch.py gui:=false
~~~

옵션: gui=true, moveit=false, rviz=true, x=0, y=0, z=0.1. rviz는 moveit=true일 때 적용합니다. 예: x:=0.5 y:=0.0 z:=0.1.

Gazebo나 이전 MoveIt 데모가 같은 ROS 도메인에서 실행 중이면 먼저 종료하세요. 종료는 실행 터미널의 Ctrl+C입니다. 고정 베이스와 물리 튜닝은 기존 가이드 범위에 맞춰 추가하지 않았습니다.

검증: gui=false moveit=true rviz=false로 실행하여 로봇 생성, 컨트롤러 2개 active, 관절 6개 발행, 직접 trajectory 성공, MoveIt Plan+Execute 성공을 확인했습니다. 자동 실행 환경에 DISPLAY가 없어 GUI 자체는 검증하지 않았습니다.

## 월드 및 스폰 위치

기본 월드는 패키지의 worlds/empty.world, 초기 위치는 x=0, y=0, z=0.1m, yaw=0rad입니다. world 인자로 외부 SDF/월드 파일 경로를 지정하면 파일 내부 world 이름을 자동으로 읽어 소환합니다.

~~~bash
ros2 launch jetcobot_gazebo gazebo.launch.py world:=/절대경로/custom.world x:=0.5 y:=0.2 z:=0.1 yaw:=1.57
~~~

이 인자는 Gazebo 로봇 전체의 스폰 위치·방향이며 각 관절의 초기 각도와는 별개입니다. MoveIt의 world 기준 가상 조인트 TF는 기존 설정이므로 스폰 위치를 바꾸면 RViz의 world 배치와 Gazebo의 world 배치는 다를 수 있습니다.

## 베이스 고정 버전 — empty.world

기존 URDF와 gazebo.launch.py는 유지하고 urdf/jetcobot_gz_fixed_base.urdf 및 fixed_base.launch.py를 별도로 추가했습니다. world → dummy를 fixed joint로 연결하며 dummy → base_link는 기존 고정 연결을 유지합니다. 로봇 전체를 static으로 만들지 않아 팔 관절은 움직입니다.

~~~bash
source /opt/ros/jazzy/setup.bash
source ~/jetcobot_ws/install/setup.bash
ros2 launch jetcobot_gazebo fixed_base.launch.py moveit:=true
~~~

empty.world 기본 고정 위치는 (0, 0, 0.1m), 방향은 (0, 0, 0)입니다. 높이 0.1m는 기존 스폰 위치를 유지한 값이며 받침대나 테이블 형상은 추가하지 않았습니다. URDF 조인트에 높이가 포함되므로 고정 launch의 추가 스폰 z 기본값은 0입니다.

고정 URDF가 world → dummy TF를 발행하므로 고정 launch에서는 MoveIt의 기존 static_virtual_joint_tfs를 중복 실행하지 않습니다.

화면 없는 실제 검증에서 양쪽 컨트롤러 active, 직접 관절 이동 성공, MoveIt Plan+Execute 성공을 확인했습니다. 이동 전후 dummy 링크 위치는 (0,0,0.1), roll/pitch/yaw는 0으로 유지되었습니다. 검증 프로세스는 종료했습니다.
