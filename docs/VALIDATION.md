# 업로드 구성 검증 (2026-10-30)

- 별도 작업폴더에서 colcon build --symlink-install: 3개 패키지 모두 성공.
- table_scene.launch.py 및 fixed_base.launch.py의 --show-args: 정상.
- Python 구문 및 XML/URDF/Xacro/SDF/DAE 파싱: 통과.
- 고정 베이스 헤드리스 실행: 로봇 생성 성공. 현재 호스트에서 다른 로봇(OpenManipulator)의 robot_description/controller_manager가 발견되어 컨트롤러 활성화 검증은 완료하지 못함. 테스트로 시작한 런치는 종료함.
- 이번 업로드 구성의 실제 관절 이동·MoveIt 실행·GUI·카메라 영상은 새로 검증 완료한 것으로 간주하지 않음. 패키지 상세 README의 성공 기록은 원 작업공간의 이전 검증 기록임.

같은 ROS 네트워크의 다른 로봇 시뮬레이터를 종료하거나 ROS 미들웨어의 도메인/라우터 구성을 분리한 후 README의 관절 이동 명령으로 확인하세요.
