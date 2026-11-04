# 배포 구성 검증 — 2026-11-04

검증 대상은 이 저장소의 3개 소스 패키지입니다. 별도 /tmp 빌드·설치 경로에서 실행했으며, 기존 시뮬레이션과 충돌하지 않도록 별도 ROS_DOMAIN_ID와 GZ_PARTITION을 사용했습니다. 다른 PC를 새로 설치한 시험은 아닙니다.

## 결과

| 항목 | 결과 |
|---|---|
| colcon build --symlink-install | 3개 패키지 성공. 의존성 정리 후 재빌드도 성공 |
| Python 구문·URDF/SRDF/월드 XML | 통과 |
| package:// 메시 존재·로컬 절대 경로 잔존 | 모든 참조 존재, 소스 설정에 /home/ane 경로 없음 |
| URDF 수·베이스 고정 | articulated URDF 1개, world→dummy fixed |
| 로봇 단독 월드 | headless 실행 및 MoveIt 기동 성공 |
| 초기 home | 여섯 관절 목표와 실제 오차 0.01rad 이내 |
| 컨트롤러 | joint_state_broadcaster / arm_group_controller / gripper_group_controller 모두 active |
| MoveIt 팔 계획·실행 | 1번 관절 0.1rad 이동 후 home 복귀 성공 |
| MoveIt 집게 계획·실행 | closed(-0.45rad) 후 open(0.12rad) 성공 |
| 상자 월드 자동 파지 | 접근·파지·상승·유지·하강·해제·후퇴 전체 PASS, 새 월드 1회 |
| Git diff 공백 검사·README 내부 링크 | 통과 |
| GUI / RViz 마우스 조작 | 이번 검증에 포함하지 않음 |
| 다른 PC의 apt/rosdep 최초 설치 | 이번 검증에 포함하지 않음 |

집게 실제 측정값은 closed=-0.446417rad, open=0.115759rad였습니다. MoveIt action 결과는 네 번 모두 status=4, error_code=1입니다. [구조화된 MoveIt 검사 결과](verification/moveit_smoke.json)를 참고하세요.

자동 파지 시험의 최소 상승은 79.834mm, 유지 높이 변화는 0.100mm, 최대 기울기는 0.565도, 최대 측정 침투는 0.158mm, 최종 수평 이동은 0.181mm였습니다. [단계별 파지 측정 결과](verification/physical_grasp_result.json)를 포함했습니다. 과거 원 작업공간의 2회 통과 기록과 별개로, 이번 정리본의 새 실행 결과입니다. 반복 안정성을 새로 충분히 평가한 결과는 아닙니다.

## 검증 환경

- ROS 2 Jazzy / Gazebo Harmonic
- ros-jazzy-moveit 2.12.4
- ros-jazzy-gz-ros2-control 1.2.20
- gz-physics-vendor 0.0.9 / gz-transport-vendor 0.0.9 / gz-msgs-vendor 0.0.8
- GUI 비활성(gui:=false rviz:=false), Bullet Featherstone, native_mimic=true

버전은 실제 검증 당시 설치값이며 README 설치 명령은 해당 배포판 저장소의 패키지를 설치합니다.

## 관찰된 메시지 및 한계

깊이 센서를 구성하지 않아 MoveIt의 octomap 센서 미정의 메시지가 출력됩니다. 이번 관절 계획·실행 및 파지에는 영향을 주지 않았습니다. 컨트롤러 초기 활성화 시 한 차례 관절 명령 제한 메시지가 있었지만 이후 초기 자세 유지와 실행 검사는 통과했습니다.

로봇 단독 검증 종료 시 SIGINT 처리 중 move_group에서 segmentation fault가 한 차례 관찰됐습니다. 동작 중 검사와 분리된 종료 단계 문제이며 이번 정리에서 수정하지 않았습니다. 검증용 시뮬레이션 프로세스는 종료했습니다.
