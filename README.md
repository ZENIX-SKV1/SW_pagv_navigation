# 소개


# 주요기능
 - VDA5050 Protocol: 표준 FMS 통신 프로토콜 지원
 - BehaviorTree 기반 상태 관리: 5-Layer 계층 구조 (Mode/Safety/System/Mission/Navigation)
 - 8륜 4축 All-Wheel Steering: ICR 제어 알고리즘
 - 경로 추종: 직선 및 곡선(Arc) 경로 지원
 - Dead Reckoning Localization: 엔코더 기반 위치 추정 (향후 SLAM 확장 예정)
 - Isaac Sim 연동: ROS2 Bridge를 통한 시뮬레이션

# SW 구조
![Diagram](image/pagv_navigation.png)

# Usage
- Dependency
```
# ROS2 Humble
sudo apt install ros-humble-desktop

# 의존성 패키지
sudo apt install ros-humble-behaviortree-cpp
sudo apt install ros-humble-tf2-ros
sudo apt install ros-humble-geometry-msgs
sudo apt install ros-humble-nav-msgs

# MQTT (Paho MQTT C++)
sudo apt install libpaho-mqttpp-dev

# JSON 라이브러리
sudo apt install nlohmann-json3-dev
```

- Build
```
cd ~/ros2_ws
colcon build --packages-select pagv_amr_core pagv_motion_controller pagv_localizer pagv_bringup
source install/setup.bash
```

- Run
```
ros2 launch pagv_bringup pagv_nav_system.launch.py
```

- MQTT Broker
```
# Mosquitto 설치 
sudo apt install mosquitto mosquitto-clients 
# Broker 실행 
mosquitto -v 
# Order 전송 테스트 
mosquitto_pub -h localhost -t "agv/v2/ZENIXROBOTICS/0000/order" -f curve.json
```

# Data Flow(Order -> 주행)

# Todo
1. 프로젝트 폴더 및 파일 구조 설계[done]
2. VDA5050_Protocol[done]
3. pagv_amr_core[done]
4. navigation_manager[done]
5. motion_controller[done]
6. localizer[done]
7. system_layer[done]
8. mode_layer
9. safety_layer
10. sequence_check
  - S01_PAGV_Boot_with_Auto
  - S02_PAGV_Boot_with_manual 
  - S03_PAGV_Auto_to_Manual
  - S04_PAGV_Manual_to_Auto
  - S05_PAGV_Load
  - S06_PAGV_Unload
  - S07_PAGV_EMS/FAULT
  - S08_PAGV_CHARGING
  - S09_PAGV_KEYOFF

# Toubleshoot

