# 배포 구성 검증 — 2026-11-05

이 검증은 GitHub 배포 디렉터리의 세 패키지를 `/tmp/jetcobot-gh-install`에 새로 빌드해 수행했습니다. 다른 overlay를 source하지 않고 ROS 2 Jazzy만 먼저 source했으며, 시뮬레이션에는 별도의 `ROS_DOMAIN_ID`와 `GZ_PARTITION`을 사용했습니다.

## 결과

| 항목 | 결과 |
|---|---|
| `colcon build --base-paths src --symlink-install` | 3개 패키지 성공 |
| `robot_only.launch.py gui:=false rviz:=false` | Gazebo 로봇 생성, home 초기 상태, MoveIt 시작 |
| MoveIt SRDF 탐색 | `.setup_assistant`가 `config/jetcobot.srdf`를 지정해 fallback 경고 없이 로드 |
| ros2_control | `joint_state_broadcaster`, `arm_group_controller`, `gripper_group_controller` active |
| MoveIt `gripper_group` | closed `-0.83rad`, open `+0.535rad` 모두 실행 성공 (`status=4`, `error_code=1`) |
| `physical_grasp_test.py` | 새로 빌드한 배포본에서 접근·접촉·80mm 상승·유지·하강·해제·후퇴 PASS |
| 물리 고정 조인트 / 물체 teleport | 사용 안 함 (`physics_attachment=false`) |
| GUI / RViz 상호작용 | headless 검증이라 미포함 |
| 다른 PC에서 최초 apt/rosdep 설치 | 미포함 |

MoveIt 그룹 명령과 픽업 측정은 [moveit_smoke.json](verification/moveit_smoke.json), [physical_grasp_result.json](verification/physical_grasp_result.json)에 보존했습니다.

픽업 시험에서 상자는 실제 손가락 접촉으로 최소 79.84mm 올라갔습니다. 유지 중 높이 변화는 0.031mm, 놓은 뒤 수평 이동은 0.144mm였습니다. 이 결과는 시뮬레이션 1회이며 반복 안정성이나 실물 성능을 보장하지 않습니다.

## MoveIt 로봇 설명 연결 방식

- `jetcobot_gazebo/urdf/jetcobot_articulated.urdf`가 Gazebo와 MoveIt에서 함께 쓰는 URDF입니다.
- `jetcobot_moveit_config/.setup_assistant`가 URDF 패키지/경로와 `config/jetcobot.srdf`를 지정합니다.
- launch는 설치 경로를 ament index로 해석해 메시지를 찾고, 런타임에서 수정한 URDF와 SRDF를 MoveIt 및 RViz에 직접 넘깁니다. `/home/<user>` 같은 컴퓨터별 절대 경로는 저장소 코드에 없습니다.
- MoveIt config 패키지가 필요로 하는 RViz, planner, kinematics, controller 파일은 저장소의 `jetcobot_moveit_config` 패키지에 포함되어 있습니다.

## 알려진 제한

- GUI/RViz 디스플레이와 다른 컴퓨터에서의 최초 설치는 아직 확인하지 않았습니다.
- 카메라/깊이 센서를 구성하지 않았기 때문에 MoveIt 로그에 octomap 센서 미정의 경고가 나올 수 있습니다. 이번 계획·실행에는 센서 기반 회피를 사용하지 않았습니다.
- 종료 시 Ctrl+C 처리 중 `move_group`가 segmentation fault를 출력한 사례가 있습니다. MoveIt 실행 및 파지 성공과는 분리된 종료 단계 증상이며 이번 정리에서는 수정하지 않았습니다.
- 40mm 폭 상자는 여유가 작아 반복 파지 대상으로 검증되지 않았습니다. 기본 픽업 시험 대상은 70×30×40mm, 40g입니다.
