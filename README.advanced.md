# JetCobot: GPT용 구현 및 유지보수 컨텍스트

이 문서는 코드 에이전트가 저장소를 처음 읽을 때 참조하는 기술 명세다. 설치·기본 조작은 README.md에 있다. 기능 설명은 이 배포 소스를 기준으로 한다.

## 범위와 진입점

- 플랫폼: Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic(gz-sim8, transport13, msgs10).
- 기본 기능: world에 고정된 6축 팔, 물리 연동 집게, MoveIt 계획·실행, RViz 수동 조작, 한 상자 물리 픽업·검증.
- `robot_only.launch.py`: 지면·받침대·로봇만. home/open 초기 상태. MoveIt/RViz 기본 활성. 자동 이동 없음.
- `grasp_test.launch.py`: 동일 로봇과 box_A. home/open 초기 상태. 자동 이동 없음.
- `articulated.launch.py`: 두 wrapper가 공유하는 실제 로딩·스폰·제어 구현. 기본 월드는 robot_only.
- `physical_grasp_test.py`: 독립적으로 실행해야 움직이는 자동 픽업 코드. robot_only 월드에는 대상이 없어 실행할 수 없다.
- 미포함: 실물 제어, 카메라 인식/브리지, 강화학습, 팔레타이징, 강제 부착, 상자 teleport. jetcocam은 로봇 메시 링크이며 영상 센서를 의미하지 않는다.

## 파일 지도

| 경로 | 역할 |
|---|---|
| src/jetcobot_description/meshes | 원본 팔·카메라 메시 자산. URDF/기존 display launch는 제거 |
| src/jetcobot_moveit_config/config | kinematics, joint_limits, moveit_controllers, RViz, Pilz 설정 |
| src/jetcobot_gazebo/urdf/jetcobot_articulated.urdf | 배포본의 유일한 URDF. fixed base + 움직이는 집게 + ros2_control |
| src/jetcobot_moveit_config/config/jetcobot.srdf | 팔·집게 그룹, 저장 자세, end effector, self collision 예외 |
| config/articulated_controllers_native.yaml | 기본 native coupling용 effort PD 제어 |
| config/articulated_controllers.yaml | native_mimic=false 대체 경로. 반복 안정 파지 검증 대상 아님 |
| models/articulated_gripper_meshes/*.stl | m 단위로 정규화된 집게 visual/collision 메시 |
| worlds/robot_only.world | ground, robot_pedestal. robot은 launch에서 동적 스폰 |
| worlds/grasp_test.world | 위 구성 + box_A(0.07×0.03×0.04m, 0.04kg) |
| scripts/physical_grasp_test.py | ROS/Gazebo 계측, MoveIt 접근, 파지·해제, 판정, JSON 출력 |

위 표에서 짧은 상대 경로는 src/jetcobot_gazebo 기준이다. 메시 생성 C++ 도구는 배포에 필요하지 않아 제외했고 정규화한 STL을 포함했다. 원본 실험 자료는 이 저장소에 복사하지 않았다.

## 로딩·초기 자세

`articulated.launch.py:setup()` 순서:

1. 설치 share 위치에서 URDF를 읽고 initial_home=true이면 SRDF home의 각 관절값을 ros2_control initial_value에 대입한다. 초기 pose 설정이며 home으로 이동하는 trajectory를 실행하지 않는다.
2. package:// 메시를 설치 경로의 file URI로 해석한다. 로컬 홈 경로를 하드코딩하지 않는다.
3. native_mimic=true이면 gz_ros2_control mimic 구동을 끄고 물리 gear constraint를 사용한다. 엔진 이름에 bullet-featherstone이 없으면 오류를 낸다.
4. 컨트롤러 YAML 절대 경로를 런타임 URDF에 넣는다. `gz sdf -p`로 SDF를 생성한다.
5. gripper 링크 충돌 메시에는 convex_decomposition(max 16 hull, 200000 voxel)을 지정한다. 좌우 손가락에 물리 contact 센서를 추가한다.
6. Gazebo와 robot_state_publisher, /clock bridge, create를 시작한다. create → joint_state_broadcaster → arm controller → gripper controller 성공 순서로 활성화하고 그 후 move_group/RViz를 시작한다.
7. MoveItConfigsBuilder가 `.setup_assistant`에서 URDF 패키지/경로와 `config/jetcobot.srdf`를 읽는다. launch는 ament index로 설치 경로를 찾고, Gazebo와 같은 런타임 robot_description을 MoveIt/RViz에 전달한다. 추가 fake hardware나 다른 demo.launch를 같이 띄우지 않는다.

기본 인자: physics_engine=gz-physics-bullet-featherstone-plugin, native_mimic=true, gui=true, moveit=true, rviz=true, initial_home=true. 두 wrapper는 gui/rviz/world만 공개하며 moveit=true, initial_home=true를 전달한다. 다른 저수준 옵션은 articulated.launch.py를 직접 사용한다.

world→dummy는 fixed이며 높이는 0.1m, dummy→base_link도 fixed다. 모델 전체를 static으로 만들면 관절이 움직이지 않으므로 그렇게 바꾸지 않는다. home=[0,0,0,0,0,-0.7854]rad. 2026-11-05 사용자 제공 사양(벌림폭 약50.5mm)을 바탕으로 손가락 mesh 위치와 gripper travel을 CAD 없이 근사 조정했다. gripper 시작/open=+0.535rad, closed=-0.83rad; mimic 부호는 좌우 링크별로 지정한다.

## MoveIt·제어 인터페이스

- group `arm_group`: base_link→TCP chain, 관절 `1_Joint` … `6_Joint`.
- group `gripper_group`: `gripper_controller`와 다섯 mimic 관절. 독립 구동 축은 하나.
- end effector `gripper`, parent_link `6_Link`.
- 저장 팔 자세: home, ready_to_see, ready_to_grab, scan_front, scan_left, scan_right.
- 저장 집게 자세: open=+0.535rad, closed=-0.83rad. URDF active-joint 허용 범위 -0.83~+0.535rad. CAD 기반 검증은 아니며 실제 접촉 간격은 대조가 필요하다.
- 팔 action: `/arm_group_controller/follow_joint_trajectory` (control_msgs/action/FollowJointTrajectory), position 제어.
- 집게 action: `/gripper_group_controller/follow_joint_trajectory`, effort 기반 trajectory 제어. MoveIt controller 목록에 launch에서 추가한다.
- 상태: `/joint_states`, `/clock`, TF world→TCP. 시뮬레이션 노드에 use_sim_time=true 적용.
- MoveIt: `/move_action`, `/execute_trajectory`, `/compute_ik`, `/compute_cartesian_path`, `/apply_planning_scene`, `/get_planning_scene`.
- Gazebo: `/world/<world>/pose/info`, `/world/<world>/dynamic_pose/info`, `/jetcobot/contacts/left`, `/jetcobot/contacts/right`. 스크립트는 센서 전체 경로도 구독한다.

팔레타이징용 planner.py/pick_place.py는 이 배포본에 없다. 현재 유지되는 픽업 구현은 physical_grasp_test.py 하나다.

## 물리 모델과 알려진 제한

- Bullet Featherstone native gear constraint로 다섯 mimic 관절을 연동한다. native 모드에 추가 mimic servo를 겹쳐 켜지 않는다.
- 0.5ms 물리 step, 컨트롤러 update_rate=1000Hz, 집게 native damping=0.0001, PD P=5.0/D=0.01, effort clamp ±0.02Nm. 관성은 메시 경계 상자로 추정한 값이며 실측 사양이 아니다.
- 기존 DAE는 Gazebo/MoveIt 로더에서 단위 처리가 달랐다. 집게 STL은 m로 정규화했다. TCP는 손가락 두께·간격 중심 기준 (-0.0006074,0.055,-0.0082)m.
- triangle mesh 기본 margin 10mm가 작은 집게를 침범한 이력이 있어 convex hull의 1mm margin 경로를 사용한다. [해당 gz-physics 소스](https://github.com/gazebosim/gz-physics/blob/gz-physics7_7.8.0/bullet-featherstone/src/SDFFeatures.cc)를 참조한다.
- 최대 메시 벌림 약 43mm이며 40mm 상자는 물리 margin까지 고려하면 여유가 작다. 30mm 상자의 성공을 40mm로 일반화하지 않는다.
- 집게 STL에는 DAE 재질 정보가 없어 URDF visual을 팔과 같은 흰색으로 지정한다. RViz의 goal state color도 흰색이다. MoveIt 그리퍼 제한은 2rad/s, 4rad/s²이며 RViz 속도·가속도 scale은 기본 0.5다.
- 수동 월드의 Gazebo 물체는 MoveIt collision scene에 자동 복사되지 않는다. 물체 추가·크기 변경 시 planning scene도 함께 관리해야 한다.

닫기 목표가 상자에 막혀도 접촉 파지 판정을 수행할 수 있도록 닫기 action tolerance를 0.75rad로 둔다. action 종료만으로 파지 성공을 판단하지 않으며 실제 양측 접촉·상승·유지·해제 기준을 별도로 검사한다.

## 픽업 코드 상세

`PhysicalGraspTest`의 주요 함수:

| 함수 | 책임 |
|---|---|
| on_joints/on_pose/on_contacts | joint, 상자 pose, 양손가락 접촉 저장 |
| setup_scene | floor/pedestal/box를 MoveIt에 등록하고 양쪽 fingertip↔box 접촉 허용 |
| gripper_to | FollowJointTrajectory 전송. 상자 접촉 시 닫기 허용 오차 확대 |
| cart_request / approach | Cartesian 요청 구성, IK 후보와 접근 경로 계획 |
| settle / vertical | 팔 목표 도달 확인 및 수직 경로 계획·실행 |
| sample | 상자 pose·finger angles·TCP·접촉·시간을 결과에 기록 |
| bilateral / any_contact | 최근 양측 접촉 및 양수 침투 접촉 확인 |
| run | 전체 시퀀스와 검증 |
| main | 오류 포함 JSON 저장, 실패 시 exit code 1 |

파라미터 기본값: world=grasp_test, box=box_A, grip_z=0.040m, lift_m=0.08m, box_size=[0.07,0.03,0.04]m, speed=0.1, report=/tmp/physical_grasp_result.json. 다른 world를 쓰면 world 이름·상자 이름·box_size를 일치시켜야 한다. lift_m을 낮춰도 성공 판정 최소 상승 60mm는 자동 변경되지 않는다.

run 순서:

1. joint/실제 상자 pose/TF 대기 → initial sample → planning scene 구성.
2. 집게 +0.535rad 개방. yaw=±π/2 중 도달 가능한 방향을 찾아 상자 위 120mm로 접근한 뒤 grip_z까지 하강.
3. -0.83rad 닫기. 상자에 막혀 목표각에 못 가는 경우를 허용하되 실제 양측 접촉이 없으면 상승하지 않는다.
4. 80mm 상승, 0.5초 간격 5개 유지 sample. 실제 높이·접촉·관절 연동·기울기·침투 깊이 검사.
5. 원래 높이로 하강하고 개방. 상자 실제 중심/yaw에 손을 정렬한다. 양측 접촉 깊이 차이로 1회 최대 0.5mm, 축별 누적 ±2mm의 중심 보정을 최대 8회 수행한다.
6. 상자가 초기 바닥 높이 ±0.5mm, tilt<1도이며 양수 침투 접촉이 없어야 후퇴한다. 음수 깊이는 분리된 근접점이므로 겹침으로 판정하지 않는다.
7. 후퇴 후 위치를 검사하고 JSON을 쓴다. 자세/연동의 일부 위반은 내려놓기 이후 실패 처리하지만 모든 오류에 대해 복구 동작을 보장하는 코드는 아니다. 실패 후에는 새 월드에서 재시작한다.

성공 기준: 유지 상승 ≥60mm, 유지 높이 변화 ≤10mm, 다섯 mimic 오차 각각 ≤0.03rad, tilt ≤10도, 측정 침투 ≤3mm, 최근 양측 접촉 유지, 최종 높이 오차 ≤4mm, 최종 수평 이동 ≤15mm. action 성공만으로 파지 성공을 선언하지 않는다. MoveIt의 접촉 허용은 계획 설정이며 Gazebo 충돌은 유지된다.

결과 JSON: success, physics_attachment=false, box_size_m, measurements; 실패 시 error 포함. measurements에는 stage, box(x/y/z/q), contacts, finger_q, finger_angles, sim_time_s, tcp가 들어간다.

## 검증 이력과 수정 시 확인

원 작업공간 기록(2026-11-03): 30mm 상자 새 월드 2회 전체 통과. 2026-11-05 업데이트 모델은 배포 저장소에서 새 isolated build 후 MoveIt 집게 open/closed 실행과 물리 픽업 1회 통과했다. 이번 결과는 docs/VALIDATION.md 및 docs/verification/에 보존했으며 반복 안정성 근거는 아니다. DART, 큰 margin, 40mm 상자의 과거 성공 표시는 현재 반복 안정성의 근거가 아니다.

수정 후에는 다른 overlay가 없는 환경에서 세 패키지를 빌드하고 robot_only/grasp_test의 스폰·컨트롤러·home을 확인한다. 팔/집게 MoveIt 실행과 새 월드 자동 픽업은 별도 확인한다. GUI를 확인하지 못했다면 headless 검증만으로 GUI 통과를 주장하지 않는다. 최신 배포 검증 범위는 docs/VALIDATION.md에 기록한다.
