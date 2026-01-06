# HRI AR4 ROS Driver

Human-Robot Interaction 실험을 위한 AR4 로봇 암 ROS 2 드라이버입니다.
[ycheng517/ar4_ros_driver](https://github.com/ycheng517/ar4_ros_driver)를 기반으로 Unity 통합, EEG 녹화/분석, VR 텔레옵 기능을 추가했습니다.

## 주요 기능

- **Unity 통합**: ROS-TCP-Endpoint를 통한 Unity-ROS2 실시간 통신
- **EEG 녹화/분석**: Unicorn EEG 장치 지원, LSL 스트림, OpenViBE 연동
- **VR 텔레옵**: VR 컨트롤러를 통한 로봇 원격 조작
- **Pick and Place**: 물체 집기 및 놓기 작업 자동화
- **MoveIt 제어**: 모션 플래닝 및 RViz 시각화
- **Gazebo 시뮬레이션**: 가상 환경에서 테스트

## 패키지 구조

| 패키지 | 설명 |
|--------|------|
| `annin_ar4_description` | 로봇 암 및 그리퍼 URDF |
| `annin_ar4_driver` | ros2_control 기반 하드웨어 인터페이스 |
| `annin_ar4_firmware` | Teensy/Arduino 펌웨어 |
| `annin_ar4_moveit_config` | MoveIt 설정 및 RViz |
| `annin_ar4_gazebo` | Gazebo 시뮬레이션 |
| `ROS-TCP-Endpoint` | Unity-ROS2 TCP 통신 |

## HRI 실험 스크립트

| 스크립트 | 설명 |
|----------|------|
| `hri_experiment_controller.py` | HRI 실험 메인 컨트롤러 |
| `eeg_recorder.py` | EEG 데이터 녹화 |
| `eeg_visualizer.py` | EEG 실시간 시각화 |
| `analyze_edf.py` | EDF 포맷 EEG 분석 |
| `analyze_xdf.py` | XDF 포맷 EEG 분석 |
| `lsl_openvibe_bridge.py` | LSL-OpenViBE 브릿지 |
| `unicorn_lsl_stream.py` | Unicorn EEG LSL 스트림 |
| `vr_teleop.py` | VR 텔레오퍼레이션 |
| `pick_and_place.py` | 픽 앤 플레이스 |
| `unity_pick_place.py` | Unity 연동 픽 앤 플레이스 |
| `gripper_test.py` | 그리퍼 테스트 |
| `robot_dance.py` | 로봇 댄스 데모 |

## 설치

### 요구사항

- Ubuntu 24.04
- ROS 2 Jazzy
- Python 3.10+

### 설치 방법

```bash
# 저장소 클론
git clone https://github.com/HCNLab/HRI-ar4-ros-driver.git
cd HRI-ar4-ros-driver

# 의존성 설치
rosdep install --from-paths . --ignore-src -r -y

# 빌드
colcon build

# 환경 설정
source install/setup.bash

# 시리얼 포트 권한 (최초 1회)
sudo addgroup $USER dialout
```

### 펌웨어 플래싱

[annin_ar4_firmware](./annin_ar4_firmware/)의 Teensy/Arduino 스케치를 업로드합니다.
[Bounce2](https://github.com/thomasfredericks/Bounce2) 라이브러리가 필요합니다.

## 사용법

### 1. 실제 로봇 제어

```bash
# 드라이버 실행 (캘리브레이션 포함)
ros2 launch annin_ar4_driver driver.launch.py calibrate:=True

# MoveIt 실행
ros2 launch annin_ar4_moveit_config moveit.launch.py
```

### 2. Unity 연동

```bash
# Unity 브릿지 실행
ros2 launch annin_ar4_driver unity_bridge.launch.py

# Unity 제어 모드
ros2 launch annin_ar4_moveit_config unity_control.launch.py
```

### 3. Gazebo 시뮬레이션

```bash
# Gazebo 실행
ros2 launch annin_ar4_gazebo gazebo.launch.py

# MoveIt 실행
ros2 launch annin_ar4_moveit_config moveit.launch.py use_sim_time:=true
```

### 4. HRI 실험

```bash
# HRI 실험 컨트롤러
ros2 run annin_ar4_driver hri_experiment_controller.py

# EEG 녹화
ros2 run annin_ar4_driver eeg_recorder.py

# VR 텔레옵
ros2 run annin_ar4_driver vr_teleop.py
```

## Launch 파라미터

### driver.launch.py

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `ar_model` | `mk3` | AR4 모델 (mk1/mk2/mk3) |
| `calibrate` | `False` | 캘리브레이션 실행 여부 |
| `include_gripper` | `True` | 그리퍼 포함 여부 |
| `serial_port` | `/dev/ttyACM0` | Teensy 시리얼 포트 |
| `arduino_serial_port` | `/dev/ttyUSB0` | Arduino 시리얼 포트 |

### unity_bridge.launch.py

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `ROS_IP` | `0.0.0.0` | ROS TCP 서버 IP |
| `ROS_TCP_PORT` | `10000` | ROS TCP 포트 |

## 참고

- 원본 저장소: [ycheng517/ar4_ros_driver](https://github.com/ycheng517/ar4_ros_driver)
- Annin Robotics: [https://www.anninrobotics.com](https://www.anninrobotics.com)
- ROS-TCP-Endpoint: [Unity-Technologies/ROS-TCP-Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint)

## License

Apache License 2.0
