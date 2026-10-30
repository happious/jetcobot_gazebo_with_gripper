# 테이블 위 JetCobot 장면

기존 컨트롤러와 MoveIt 설정을 재사용하는 전용 런치입니다. 기존 런치는 수정하지 않았습니다.

## 실행

```bash
cd ~/jetcobot_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select jetcobot_gazebo --symlink-install
source install/setup.bash

# 테이블 + 큐브 (Gazebo만)
ros2 launch jetcobot_gazebo table_scene.launch.py robot:=false

# 테이블 + 고정 로봇팔 + 큐브 + MoveIt + RViz
ros2 launch jetcobot_gazebo table_scene.launch.py

# GUI 없는 실행
ros2 launch jetcobot_gazebo table_scene.launch.py gui:=false rviz:=false
```

각 실행은 앞 실행을 종료한 뒤 시작합니다. robot, gui, moveit, rviz 옵션은 모두 기본 true입니다. robot:=false이면 MoveIt/RViz를 시작하지 않습니다. moveit:=false는 move_group을 생략합니다.

## 구성

- worlds/table_cube.world: 테이블과 큐브
- worlds/table_robot_cube.world: 테이블이 결합된 로봇팔과 큐브
- urdf/jetcobot_table.urdf: world → table_link → dummy → base_link 고정 연결. 관절 6개는 움직일 수 있습니다.
- models/jetcobot_table/model.sdf: 위 URDF를 변환한 Gazebo 모델
- models/table_marble, models/rubiks_cube: 원본 모델의 실행용 복사본. 루트 models는 수정하지 않았습니다.

상판 Z = 0.964761707m. 베이스는 (0, 0, 0.964761707), 큐브는 (0.22, 0, 0.994699437)에서 시작하여 약 1mm 아래 상판에 안착합니다. 큐브 크기는 원본 약 5.8cm이며 질량과 관성도 원본을 유지했습니다. 테이블 Collada 노드 변환과 0.25 스케일을 반영했습니다.

런치는 설치 위치에 맞게 메시와 컨트롤러 경로를 보정한 임시 월드를 만들고 종료 시 삭제합니다. Gazebo와 MoveIt은 같은 URDF를 사용합니다. 기존 world→dummy 가상 조인트는 중복 적용하지 않습니다. 테이블은 URDF 충돌 형상에 포함됩니다. 큐브의 MoveIt planning scene 동기화와 파지 코드는 포함하지 않았습니다. 현재 MoveIt에서 큐브는 충돌 물체로 등록되어 있지 않습니다.

월드만 직접 열 때는 models 경로를 GZ_SIM_RESOURCE_PATH에 추가해야 합니다. 로봇 SDF 경로 보정과 컨트롤러 시작을 위해 위 런치를 권장합니다.

## 검증

colcon 빌드, Python/XML 구문 검사, 두 월드의 헤드리스 물리 실행을 확인했습니다. 큐브 중심은 Z≈0.993699328m에서 안정적으로 유지되었습니다. joint_state_broadcaster와 arm_group_controller가 active이며 MoveIt은 planning 준비 상태에 도달했습니다. 메시 및 텍스처 로딩 오류를 수정한 뒤 재검증했습니다. 현재 실행 환경에는 X 디스플레이가 없어 RViz/Gazebo GUI 화면은 검증하지 못했습니다. 3D 센서를 추가하지 않아 MoveIt의 octomap sensor 경고는 표시됩니다.

## 손목 RGB 카메라

로봇 장면에는 손목 카메라가 기본 활성화됩니다. 기존 실행 명령을 그대로 사용합니다. robot:=false 장면에는 카메라가 없습니다.

- 이미지: /jetcobot/camera/image_raw (sensor_msgs/msg/Image)
- 카메라 정보: /jetcobot/camera/camera_info (sensor_msgs/msg/CameraInfo)
- 광학 프레임: wrist_camera_optical_frame (+Z 전방, +X 오른쪽, +Y 아래)
- 640×480, RGB8, 15Hz (시뮬레이션 시간 기준), 수평 시야각 60도
- 근거리 0.01m, 원거리 5m. RGB 센서이며 깊이 영상은 제공하지 않습니다.
- Gazebo 카메라 +X축과 ROS 광학 좌표를 고정 조인트로 변환했습니다. 렌더링 위치는 기존 jetcocam 프레임에서 광축 방향 21mm 앞이며, 카메라 외장 앞면을 넘도록 배치했습니다. 실물 보정값을 적용한 위치/내부 파라미터는 아닙니다.

영상 보기:

```bash
source /opt/ros/jazzy/setup.bash
source ~/jetcobot_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view /jetcobot/camera/image_raw
```

또는 RViz에서 Add → Image를 추가한 뒤 위 이미지 토픽을 선택합니다. 초기 자세에서는 카메라가 큐브를 향하지 않을 수 있습니다. 카메라는 손목을 따라 움직이므로 큐브를 보려면 손목 자세를 조정해야 합니다.

gui:=false 실행에는 --headless-rendering을 적용합니다. GUI 없이 EGL 렌더링으로 실제 이미지/CameraInfo 수신, 해상도, 광학 frame_id, RGB 데이터 크기와 시뮬레이션 시간 기준 약 15fps를 검증했습니다. 스냅샷을 확인해 카메라 외장이 시야를 가리지 않는 것도 확인했습니다.
