# JetCobot Gazebo + MoveIt 2

고정된 6축 로봇팔과 **실제로 열고 닫히는 물리 집게**를 Gazebo에서 조작하는 ROS 2 작업공간입니다. 기본 실행은 홈 자세에서 대기합니다. 자동 픽업은 별도 명령으로만 시작합니다.

## 환경 및 설치

Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic 기준입니다. GUI 사용에는 OpenGL이 가능한 Linux 데스크톱이 필요합니다. ROS가 없다면 [Jazzy 공식 설치 안내](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)에 따라 desktop 버전을 먼저 설치하세요. [Gazebo 공식 호환 안내](https://gazebosim.org/docs/harmonic/)도 참고하세요.

다른 ROS 작업공간이 자동 source되지 않는 새 터미널에서 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
sudo apt update
sudo apt install -y git python3-colcon-common-extensions python3-rosdep \
  ros-jazzy-moveit ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-controllers ros-jazzy-gz-physics-vendor \
  ros-jazzy-gz-transport-vendor ros-jazzy-gz-msgs-vendor

# rosdep가 이미 초기화됐다면 건너뜁니다.
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update

git clone https://github.com/happious/jetcobot_gazebo_with_gripper.git ~/jetcobot_ws
cd ~/jetcobot_ws
rosdep install --from-paths src --ignore-src -y --rosdistro jazzy
colcon build --base-paths src --symlink-install
source install/setup.bash
```

별도의 원본 저장소 clone이나 사용자별 모델 경로 수정은 필요하지 않습니다. 새 터미널마다 다음을 실행하세요. 경로는 실제 clone 위치에 맞추면 됩니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/jetcobot_ws/install/setup.bash
```

## 이미 clone한 저장소 업데이트

기존 작업공간을 새로 clone할 필요 없이 해당 저장소에서 최신 변경을 받고 다시 빌드합니다. `~/jetcobot_ws`는 실제 clone 경로로 바꾸세요. 실행 중인 Gazebo/RViz가 있으면 먼저 종료합니다.

```bash
cd ~/jetcobot_ws
git status --short
git pull --ff-only origin main
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y --rosdistro jazzy
colcon build --base-paths src --symlink-install
source install/setup.bash
ros2 pkg prefix jetcobot_gazebo
```

`ros2 pkg prefix jetcobot_gazebo`가 방금 업데이트한 clone의 `install/jetcobot_gazebo` 아래를 가리키는지 확인하세요. 다른 ROS 작업공간의 setup 파일이 함께 source되어 있으면 이전 모델이나 MoveIt 설정을 불러올 수 있으니 새 터미널에서 ROS와 이 작업공간만 source합니다.

`git status --short`에 로컬 수정이 있으면 pull 전에 commit하거나 stash해 보관하세요. 업데이트 후 수정 사항을 다시 적용할 때 충돌이 생기면 내용을 검토하고 해결한 다음 빌드합니다.

## 1. 로봇팔만 켜기 — 기본 사용

```bash
ros2 launch jetcobot_gazebo robot_only.launch.py
```

Gazebo, MoveIt, RViz가 함께 실행됩니다. 지면·고정 받침대와 로봇팔이 있고 상자는 없습니다. 베이스는 world에 fixed joint로 고정되고 팔과 집게 관절은 움직입니다. 팔은 home, 집게는 열린 상태로 시작합니다. 첫 실행은 집게 충돌 형상 분해 때문에 시간이 걸릴 수 있습니다.

RViz의 MotionPlanning 패널에서:

1. Planning Group을 `arm_group`으로 선택합니다.
2. Goal State에 `home` 또는 다른 저장 자세를 선택하거나 목표 위치 표시를 드래그합니다.
3. Plan으로 경로를 확인한 뒤 Execute를 누릅니다.
4. 집게는 Planning Group을 `gripper_group`으로 바꾸고 Goal State의 `open` / `closed`를 선택한 뒤 Plan → Execute로 조작합니다.

home은 `[0, 0, 0, 0, 0, -0.7854]` rad입니다. 하드웨어 원점 탐색 기능은 아닙니다. 사양상 최대 벌림폭 약50.5mm를 참고해 사용자 제작 손가락 mesh의 위치와 가동범위를 근사 조정했습니다. 집게 목표는 open=+0.535rad, closed=-0.83rad이며 CAD 치수에 맞춘 검증 모델은 아닙니다.

## 두 로봇 나란히 실행 — dual arm

```bash
ros2 launch jetcobot_gazebo dual_arm.launch.py
```

두 팔이 Gazebo에서 같은 방향을 보고 Y축으로 40cm 떨어져 시작합니다. Gazebo DART 물리 엔진을 사용합니다. Bullet Featherstone은 한 모델 안의 두 관절 트리 구성을 지원하지 않아 이 launch에는 사용하지 않습니다. MoveIt/RViz에는 `arm_left`, `arm_right`, `both_arms` planning group이 보이고, 네 개 팔/그리퍼 controller가 연결됩니다. `both_arms`를 선택하면 두 팔을 한 MoveIt 계획으로 함께 계획할 수 있습니다. 팔 사이 충돌 쌍은 SRDF에서 끄지 않아 계획 시 충돌 검사에 포함됩니다.

처음에는 GUI 없이 기동 상태만 확인할 수도 있습니다.

```bash
ros2 launch jetcobot_gazebo dual_arm.launch.py gui:=false rviz:=false
```

시작 자세는 양쪽 팔 관절이 0 rad이고 집게가 열린 자세입니다. 이 월드에는 두 로봇 받침대만 있으며, 공유 테이블·바구니 충돌체는 아직 포함하지 않았습니다. 따라서 MoveIt의 충돌 회피는 두 팔 모델끼리만 확인합니다. 실물 두 대의 상태 연결이나 하드웨어 제어 launch가 아닙니다.

## 2. 상자 포함 수동 조작

기존 launch를 Ctrl+C로 종료한 뒤 실행합니다.

```bash
ros2 launch jetcobot_gazebo grasp_test.launch.py
```

같은 로봇팔과 70×30×40mm, 40g 상자 하나가 나타납니다. 홈 자세에서 대기하고 자동으로 잡거나 들어 올리지 않습니다. Gazebo 물체는 MoveIt 충돌 장면에 자동 등록되지 않습니다. 아래 자동 시험은 별도로 지면·받침대·상자를 MoveIt에 등록합니다.

## 3. 자동 픽업 시험

`grasp_test.launch.py`를 새로 켠 초기 상태에서, 다른 터미널에 환경을 불러온 다음 실행합니다. 시험 도중 RViz에서 별도 명령을 보내지 마세요.

```bash
ros2 run jetcobot_gazebo physical_grasp_test.py
```

열기 → 접근 → 닫기 → 80mm 상승 → 약 2초 유지 → 하강 → 해제 확인 → 후퇴를 수행합니다. 실제 접촉과 마찰을 사용하며 상자를 강제로 부착하거나 이동시키지 않습니다.

결과는 `/tmp/physical_grasp_result.json`의 `success`, `error`, 단계별 측정값으로 확인합니다. 재시험은 launch를 완전히 종료하고 새 월드에서 진행합니다.

```bash
ros2 run jetcobot_gazebo physical_grasp_test.py --ros-args \
  -p report:=/tmp/my_grasp_result.json
```

이번 정리본은 headless 환경에서 MoveIt 팔·집게 실행과 자동 파지 전체 동작을 1회 통과했습니다. 자세한 범위와 측정값은 [검증 기록](docs/VALIDATION.md)에 있습니다.

과거 원 작업공간에서는 30mm 폭 상자가 새 월드 2회 통과했습니다. 40mm 폭은 반복 안정 파지에 실패했습니다. 시뮬레이션 기록이며 다른 PC에서의 반복 성공이나 실물 성능을 보장하는 수치는 아닙니다.

## 실행 옵션 및 문제 확인

```bash
# GUI 없이 서버와 MoveIt 실행
ros2 launch jetcobot_gazebo robot_only.launch.py gui:=false rviz:=false

# Gazebo GUI만 표시하고 RViz 생략
ros2 launch jetcobot_gazebo grasp_test.launch.py rviz:=false

ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 topic echo /clock --once
```

`joint_state_broadcaster`, `arm_group_controller`, `gripper_group_controller`가 active여야 합니다.

| 증상 | 확인 |
|---|---|
| 패키지를 찾지 못함 | 빌드 성공과 install/setup.bash source |
| 다른 모델이 실행됨 | overlay를 정리하고 `ros2 pkg prefix jetcobot_gazebo` 확인 |
| 컨트롤러 생성 대기 | Gazebo 물리 엔진·gz_ros2_control 오류와 중복 launch 확인 |
| 집게가 덜 닫힘 | 목표 -0.83rad와 실제 joint_states 비교. 접촉·구동 힘 확인 |
| MoveIt이 다른 모델을 읽음 | `ros2 pkg prefix jetcobot_gazebo`와 `ros2 pkg prefix jetcobot_moveit_config`가 현재 clone의 `install/`을 가리키는지 확인하고 다른 overlay를 source하지 않았는지 확인 |
| Plan 성공, Execute 실패 | controller active, /clock, /joint_states, 중복 명령 확인 |
| gz.transport13 / gz.msgs10 import 실패 | ROS source와 vendor 패키지 설치 확인. 시스템 python3 사용 |
| Bullet Featherstone 로딩 실패 | gz-physics-vendor 설치와 ROS/Gazebo 버전 확인 |
| GUI 표시 실패 | DISPLAY와 GPU/OpenGL 환경 확인 |

한 번에 하나의 로봇 launch만 실행하세요. 자동 시험은 해당 터미널에서 Ctrl+C, 전체 시뮬레이션은 launch 터미널에서 Ctrl+C로 종료합니다.

## 코드 및 출처

- [심화 README: GPT용 구조·인터페이스·파지 구현](README.advanced.md)
- [이번 배포 구성의 검증 기록](docs/VALIDATION.md)
- 원본 로봇 자산·MoveIt 설정: [kemjensak/jetcobot_ros2](https://github.com/kemjensak/jetcobot_ros2). 기존 LICENSE와 패키지별 라이선스 선언을 보존했습니다. 원본 description 패키지의 라이선스 표기는 미정(TODO) 상태입니다.

실물 bringup, 강화학습, 카메라 데모, 팔레타이징, 강제 부착 플러그인, 과거 시험 로그는 배포 대상에서 제외했습니다. 기본 실행 모델은 `jetcobot_articulated.urdf`입니다.
