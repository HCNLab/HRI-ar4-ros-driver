# HRI AR4 로봇 시스템 설정 가이드

이 문서는 HRI(Human-Robot Interaction) 실험을 위한 AR4 로봇 시스템의 전체 설정 과정을 안내합니다.

---

## 목차

1. [시스템 개요](#1-시스템-개요)
2. [요구 사항](#2-요구-사항)
3. [ROS2 설치](#3-ros2-설치)
4. [프로젝트 설치](#4-프로젝트-설치)
5. [Unity 설정](#5-unity-설정)
6. [실행 가이드](#6-실행-가이드)
7. [문제 해결](#7-문제-해결)

---

## 1. 시스템 개요

이 프로젝트는 다음 구성 요소로 이루어져 있습니다:

```
┌─────────────────────────────────────────────────────────────┐
│                     HRI 실험 시스템                          │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│   ┌─────────────┐     ROS-TCP      ┌─────────────────┐     │
│   │   Unity     │◄───────────────►│  ROS2 Jazzy     │     │
│   │ (Windows)   │    Port:10000   │  (Ubuntu/WSL2)  │     │
│   └─────────────┘                  └────────┬────────┘     │
│                                             │               │
│                                    ┌────────▼────────┐     │
│                                    │  AR4 로봇 암    │     │
│                                    │  (Teensy 4.1)   │     │
│                                    └─────────────────┘     │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 1.1 패키지 구조

| 패키지 | 설명 |
|--------|------|
| `annin_ar4_description` | 로봇 URDF 모델 파일 |
| `annin_ar4_driver` | 하드웨어 인터페이스 및 HRI 스크립트 |
| `annin_ar4_moveit_config` | MoveIt 모션 플래닝 설정 |
| `annin_ar4_gazebo` | Gazebo 시뮬레이션 |
| `ROS-TCP-Endpoint` | Unity-ROS2 TCP 통신 |

### 1.2 주요 기능

- **Unity Digital Twin**: ROS2와 실시간 연동되는 가상 로봇
- **MoveIt 제어**: 충돌 회피 및 경로 계획
- **Gazebo 시뮬레이션**: 실제 로봇 없이 테스트

---

## 2. 요구 사항

### 2.1 하드웨어

| 항목 | 사양 |
|------|------|
| PC | Windows 11 + WSL2 (Ubuntu 24.04) |
| 로봇 | AR4 MK3 로봇 암 + 서보 그리퍼 |
| 컨트롤러 | Teensy 4.1 + Arduino Nano |

### 2.2 소프트웨어

| 항목 | 버전 |
|------|------|
| Ubuntu | 24.04 LTS (WSL2 또는 Native) |
| ROS2 | Jazzy Jalisco |
| Python | 3.10 이상 |
| Unity | 6.0 LTS |

---

## 3. ROS2 설치

### 3.1 Ubuntu 24.04에서 ROS2 Jazzy 설치

터미널을 열고 다음 명령어를 순서대로 실행합니다:

```bash
# 1. 로케일 설정
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 2. Universe 저장소 활성화
sudo apt install software-properties-common
sudo add-apt-repository universe

# 3. ROS2 GPG 키 추가
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

# 4. ROS2 저장소 추가
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 5. 패키지 인덱스 업데이트
sudo apt update
sudo apt upgrade -y  # 기존 패키지 업데이트 (권장)

# 6. ROS2 Jazzy 설치
sudo apt install ros-jazzy-desktop -y

# 7. 개발 도구 설치
sudo apt install ros-dev-tools python3-colcon-common-extensions python3-rosdep -y
```

### 3.2 설치 옵션

| 패키지 | 설명 |
|--------|------|
| `ros-jazzy-desktop` | 전체 설치 (ROS, RViz, 데모, 튜토리얼 포함) - **권장** |
| `ros-jazzy-ros-base` | 최소 설치 (GUI 없음, 서버/로봇용) |
| `ros-jazzy-perception` | ros-base + 이미지/레이저 처리 라이브러리 |

### 3.3 환경 설정

`~/.bashrc` 파일에 다음 내용을 추가합니다:

```bash
# ROS2 Jazzy 환경
source /opt/ros/jazzy/setup.bash

# 워크스페이스 환경 (프로젝트 설치 후)
source ~/ar4_ros_driver/install/setup.bash
```

변경사항 적용:

```bash
source ~/.bashrc
```

### 3.4 설치 확인

ROS2가 정상적으로 설치되었는지 확인합니다:

```bash
# ROS2 버전 확인
printenv ROS_DISTRO
# 출력: jazzy

# 명령어 확인
ros2 --help

# 데모 노드로 테스트 (선택사항)
# 터미널 1:
ros2 run demo_nodes_cpp talker

# 터미널 2:
ros2 run demo_nodes_py listener
```

talker가 메시지를 발행하고 listener가 수신하면 정상입니다.

---

## 4. 프로젝트 설치

### 4.1 저장소 클론

```bash
cd ~
git clone https://github.com/HCNLab/HRI-ar4-ros-driver.git ar4_ros_driver
cd ar4_ros_driver
```

### 4.2 의존성 설치

```bash
# rosdep 초기화 (최초 1회)
sudo rosdep init
rosdep update

# 의존성 설치
rosdep install --from-paths . --ignore-src -r -y
```

### 4.3 빌드

```bash
cd ~/ar4_ros_driver
colcon build
source install/setup.bash
```

### 4.4 시리얼 포트 권한 설정 (최초 1회)

실제 로봇을 사용하려면 시리얼 포트 접근 권한이 필요합니다:

```bash
sudo usermod -aG dialout $USER
```

**주의**: 권한 변경 후 **로그아웃 후 다시 로그인**해야 적용됩니다.

### 4.5 펌웨어 플래싱

로봇을 처음 사용하는 경우, Teensy와 Arduino에 펌웨어를 업로드해야 합니다:

1. Arduino IDE 설치
2. [Bounce2 라이브러리](https://github.com/thomasfredericks/Bounce2) 설치
3. `annin_ar4_firmware/AR4_teensy/AR4_teensy.ino` → Teensy 4.1에 업로드
4. `annin_ar4_firmware/AR4_nano/AR4_nano.ino` → Arduino Nano에 업로드

---

## 5. Unity 설정

### 5.1 Unity 프로젝트 클론

Unity 프로젝트는 별도의 저장소에 있습니다. Git LFS를 사용하므로 먼저 설치해야 합니다.

#### Windows (PowerShell 또는 Git Bash)

```bash
# Git LFS 설치 (최초 1회)
git lfs install

# Unity 프로젝트 클론
cd C:\Users\<사용자명>\Documents
git clone https://github.com/HCNLab/HRI-ditial-twin-factory-setting.git

# LFS 파일 다운로드
cd HRI-ditial-twin-factory-setting
git lfs pull
```

#### WSL2에서 심볼릭 링크 생성 (선택사항)

WSL2에서 Unity 프로젝트에 쉽게 접근하려면:

```bash
cd ~/HRI_robot_unity_project
ln -s "/mnt/c/Users/<사용자명>/Documents/HRI-ditial-twin-factory-setting" unity
```

### 5.2 Unity 프로젝트 열기

1. **Unity Hub** 실행
2. **Open** 클릭 → 클론한 프로젝트 폴더 선택
3. Unity **6.0 LTS** 버전으로 열기
4. 처음 열 때 패키지 import에 시간이 걸릴 수 있습니다

### 5.3 ROS-TCP-Connector 확인

Unity 프로젝트에 ROS-TCP-Connector가 이미 설치되어 있습니다. 만약 없다면:

1. **Window > Package Manager** 열기
2. **+ 버튼 > Add package from git URL** 클릭
3. 다음 URL 입력:
   ```
   https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
   ```

### 5.4 ROS 연결 설정

1. **Robotics > ROS Settings** 메뉴 열기
2. 다음 값 설정:
   - **ROS IP Address**: `127.0.0.1` (WSL2의 경우 WSL IP 주소)
   - **ROS Port**: `10000`
   - **Protocol**: ROS2

### 5.5 WSL2 IP 주소 확인

WSL2를 사용하는 경우, WSL의 IP 주소를 확인하여 Unity에 설정합니다:

```bash
# WSL2 터미널에서 실행
ip addr show eth0 | grep -oP '(?<=inet\s)\d+(\.\d+){3}'
```

---

## 6. 실행 가이드

### 6.1 데모 모드 (MoveIt만)

가장 간단한 테스트 방법입니다. Gazebo나 실제 로봇 없이 MoveIt과 RViz만으로 로봇을 테스트합니다:

```bash
ros2 launch annin_ar4_moveit_config demo.launch.py
```

- FakeSystem 하드웨어 인터페이스 사용
- RViz에서 로봇을 드래그하여 목표 위치 설정
- Plan & Execute로 모션 플래닝 테스트

MoveIt 사용법을 먼저 익히고 싶을 때 권장합니다.

### 6.2 시뮬레이션 모드 (Gazebo)

물리 시뮬레이션이 필요할 때 Gazebo를 사용합니다:

```bash
# 터미널 1: Gazebo 시뮬레이션 실행
ros2 launch annin_ar4_gazebo gazebo.launch.py

# 터미널 2: MoveIt 실행
ros2 launch annin_ar4_moveit_config moveit.launch.py use_sim_time:=true
```

### 6.3 실제 로봇 제어

**주의**: 로봇 주변에 장애물이 없는지 확인하세요!

**중요**: 드라이버 실행 전에 반드시 [수동 캘리브레이션](#수동-캘리브레이션)을 먼저 수행해야 합니다.

```bash
# 터미널 1: 로봇 드라이버 실행 (수동 캘리브레이션 완료 후)
ros2 launch annin_ar4_driver driver.launch.py calibrate:=False

# 터미널 2: MoveIt 실행
ros2 launch annin_ar4_moveit_config moveit.launch.py
```

#### 드라이버 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `ar_model` | `mk3` | AR4 모델 (mk1/mk2/mk3) |
| `include_gripper` | `True` | 그리퍼 사용 여부 |
| `serial_port` | `/dev/ttyACM0` | Teensy 시리얼 포트 |
| `arduino_serial_port` | `/dev/ttyUSB0` | Arduino 시리얼 포트 |

### 6.4 Unity 연동

#### 6.4.1 Unity + MoveIt 데모 모드 (실제 로봇 불필요)

Unity와 MoveIt을 함께 사용하되, 실제 로봇 없이 테스트합니다:

```bash
# 터미널 1: Unity 브릿지 실행
ros2 launch annin_ar4_driver unity_bridge.launch.py

# 터미널 2: MoveIt 데모 모드 실행
ros2 launch annin_ar4_moveit_config demo.launch.py
```

#### 6.4.2 Unity + 실제 로봇 (MoveIt 사용)

실제 로봇과 Unity Digital Twin을 동기화합니다.

**중요**: 드라이버 실행 전에 반드시 [수동 캘리브레이션](#수동-캘리브레이션)을 먼저 수행해야 합니다.

```bash
# 터미널 1: Unity 브릿지 실행
ros2 launch annin_ar4_driver unity_bridge.launch.py

# 터미널 2: 로봇 드라이버 실행 (수동 캘리브레이션 완료 후)
ros2 launch annin_ar4_driver driver.launch.py calibrate:=False

# 터미널 3: MoveIt 실행
ros2 launch annin_ar4_moveit_config moveit.launch.py
```

그 후 Unity 에디터에서 **Play** 버튼을 눌러 실행합니다.

#### 6.4.3 Unity에서 직접 로봇 제어

Unity에서 ROS를 통해 직접 로봇을 제어하려면 `RosJointTrajectoryPublisher.cs` 스크립트를 사용합니다.

**Unity 설정:**
1. Unity 씬에서 `ROSConnection` 오브젝트 선택
2. `RosJointTrajectoryPublisher.cs` 컴포넌트 추가

**ROS 실행 (MoveIt 불필요):**

```bash
# 터미널 1: 로봇 드라이버 실행 (수동 캘리브레이션 완료 후)
ros2 launch annin_ar4_driver driver.launch.py calibrate:=False

# 터미널 2: Unity 브릿지 실행
ros2 launch annin_ar4_driver unity_bridge.launch.py

# 터미널 3: RViz2 실행 (시각화용, 선택사항)
rviz2
```

그 후 Unity 에디터에서 **Play** 버튼을 눌러 실행합니다.

**참고**: 이 방식은 MoveIt의 충돌 회피 및 경로 계획 없이 Unity에서 직접 관절 명령을 전송합니다.

### 6.5 Pick and Place 데모

`pick_and_place.py` 스크립트는 로봇팔이 물체를 집어서 다른 위치에 놓는 전체 시나리오를 자동으로 수행합니다.

#### 6.5.1 기능 개요

- 홈 위치에서 시작
- 지정된 위치에서 물체 집기 (Pick)
- 다른 위치로 이동하여 물체 놓기 (Place)
- 역방향 사이클 수행 (원래 위치로 복귀)
- Unity 피드백 연동 지원 (선택사항)

#### 6.5.2 사전 요구사항

[수동 캘리브레이션](#수동-캘리브레이션)을 완료한 후, 다음이 실행 중이어야 합니다:

```bash
# 터미널 1: 로봇 드라이버 (수동 캘리브레이션 완료 후)
ros2 launch annin_ar4_driver driver.launch.py calibrate:=False

# 터미널 2: MoveIt
ros2 launch annin_ar4_moveit_config moveit.launch.py

# 터미널 3 (Unity 연동 시): Unity 브릿지
ros2 launch annin_ar4_driver unity_bridge.launch.py
```

#### 6.5.3 실행 방법

```bash
# 터미널 4: Pick and Place 데모 실행
ros2 run annin_ar4_driver pick_and_place.py
```

#### 6.5.4 동작 순서

| 단계 | 동작 | 설명 |
|------|------|------|
| 1 | 홈 위치 | 모든 관절 0도 |
| 2 | 그리퍼 열기 | 14mm 열림 |
| 3-4 | Pick 위치 접근 | 위에서 접근 후 하강 |
| 5 | 물체 잡기 | 그리퍼 닫기 |
| 6 | 들어올리기 | 물체를 위로 이동 |
| 7-8 | Place 위치 접근 | 위에서 접근 후 하강 |
| 9 | 물체 놓기 | 그리퍼 열기 |
| R1-R9 | 역방향 사이클 | Place → Pick으로 복귀 |

#### 6.5.5 Unity 피드백 활성화

코드 내 `use_unity_feedback` 변수를 `True`로 설정하면 Unity에서 grasp 성공/실패 피드백을 받을 수 있습니다:

- `/unity/grasp_success` (Bool): 잡기 성공 여부
- `/unity/grasp_feedback` (String): 상세 피드백 메시지

---

## 7. 문제 해결

### 7.1 ROS2 관련

#### 패키지를 찾을 수 없음

```bash
# 환경 변수 다시 소싱
source ~/ar4_ros_driver/install/setup.bash
```

#### 빌드 오류

```bash
# 클린 빌드
cd ~/ar4_ros_driver
rm -rf build install log
colcon build
```

### 7.2 로봇 관련

#### 시리얼 포트 접근 거부

```bash
# 권한 확인
ls -la /dev/ttyACM0

# 권한 부여
sudo chmod 666 /dev/ttyACM0

# 또는 dialout 그룹에 추가 (영구적)
sudo usermod -aG dialout $USER
# 로그아웃 후 다시 로그인
```

#### 시리얼 포트 찾기

```bash
# 연결된 시리얼 장치 확인
ls /dev/tty*

# dmesg로 최근 연결된 장치 확인
dmesg | grep tty | tail -5
```

#### E-Stop 리셋

E-Stop(비상 정지) 버튼을 눌렀다가 해제한 후에는 다음 명령어를 실행하여 리셋해야 합니다:

```bash
ros2 run annin_ar4_driver reset_estop.sh mk3
```

#### 수동 캘리브레이션

로봇을 사용하기 전에 반드시 수동 캘리브레이션을 수행해야 합니다:

1. **전원 및 연결 해제**
   - 그리퍼 전원 OFF
   - 로봇팔 전원 OFF
   - 모든 USB 케이블을 컴퓨터에서 분리

2. **홈 위치로 이동**
   - 토크가 해제된 상태에서 로봇팔을 손으로 잡고 홈 위치로 이동

3. **로봇팔 전원 ON**
   - 전원을 켜면 토크가 설정되어 로봇팔이 현재 위치에 고정됨
   - 손을 놓아도 로봇팔이 움직이지 않는지 확인

4. **연결 복구**
   - 그리퍼 전원 ON
   - 모든 USB 케이블을 컴퓨터에 연결

5. **드라이버 실행**
   ```bash
   ros2 launch annin_ar4_driver driver.launch.py calibrate:=False
   ```

6. **MoveIt 실행 및 테스트**
   ```bash
   ros2 launch annin_ar4_moveit_config moveit.launch.py
   ```
   RViz에서 Plan & Execute를 사용하여 로봇팔이 정상적으로 움직이는지 확인

7. **그리퍼 테스트 (선택사항)**
   ```bash
   # 그리퍼 열기 (position: 0.014)
   ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.014, max_effort: 1.0}}"

   # 그리퍼 닫기 (position: 0.001)
   ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.001, max_effort: 1.0}}"
   ```

### 7.3 Unity 관련

#### ROS 연결 실패

1. Unity ROS Settings에서 IP 주소 확인
2. `ros2 launch annin_ar4_driver unity_bridge.launch.py` 실행 중인지 확인
3. 방화벽이 포트 10000을 차단하지 않는지 확인

#### WSL2에서 Unity 연결

WSL2는 별도의 IP 주소를 사용합니다:

```bash
# WSL2 IP 확인
ip addr show eth0 | grep inet

# Windows에서 WSL2로 포트 포워딩 (PowerShell 관리자 권한)
netsh interface portproxy add v4tov4 listenport=10000 listenaddress=0.0.0.0 connectport=10000 connectaddress=<WSL2_IP>
```

---

## 부록

### A. 유용한 ROS2 명령어

```bash
# 토픽 목록 확인
ros2 topic list

# 토픽 모니터링
ros2 topic echo /unity/robot_command

# 노드 목록
ros2 node list

# 서비스 호출
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers
```

### B. 파일 위치

| 항목 | 경로 |
|------|------|
| ROS2 워크스페이스 | `~/ar4_ros_driver` |
| HRI 스크립트 | `~/ar4_ros_driver/annin_ar4_driver/scripts/` |
| Launch 파일 | `~/ar4_ros_driver/annin_ar4_driver/launch/` |

### C. 참고 자료

- [ROS2 Jazzy 문서](https://docs.ros.org/en/jazzy/)
- [MoveIt2 튜토리얼](https://moveit.picknik.ai/main/index.html)
- [Unity ROS TCP Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector)
- [AR4 로봇 공식 사이트](https://www.anninrobotics.com/)

---

*문서 작성일: 2026-01-13*
*버전: 1.0*
