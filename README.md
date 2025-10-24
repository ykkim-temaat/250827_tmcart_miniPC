# 250827_tmcart_minipc
miniPC에는 ROS2 Humble python 프로그램과 Main Control Board의 펌웨어 개발환경 및 소스코드가 있음

## ESP32-S3 펌웨어 기본사양
- esp32-s3
- esp-idf v5.1.2
- micro-ROS Agent
- ros2 humble
- micro_ros_espidf_component (https://github.com/micro-ROS/micro_ros_espidf_component)

## python 프로그램 자동시작 daemon
- tmcart.service 파일 내용
$ sudo vi /etc/systemd/system/tmcart.service
```
[Unit]
Description=TMCart MiniPC Node Service
# 네트워크가 준비된 후에 실행되도록 설정
After=network.target

[Service]
# 스크립트를 실행할 사용자 계정
Type=simple
User=temaat
# 스크립트가 위치한 디렉토리
WorkingDirectory=/home/temaat/tmcart/python/
# 실행할 명령 (python3의 전체 경로를 사용하는 것이 좋음)
ExecStart=/home/temaat/tmcart/python/start_tmcart.sh
# 실패 시 항상 재시작
Restart=always

[Install]
WantedBy=multi-user.target  
```
- daemon이 잘 실행되고 있는지 확인
$ systemctl status tmcart.service

- start_tmcart.sh 파일내용
  ```
  #! /bin/bash

  export ROS_DOMAIN_ID=11
  source /opt/ros/humble/setup.bash
  echo "MY_DOMAIN_ID: \033[32m$ROS_DOMAIN_ID\033[0m, ROS2 Humble is activated!"


  # 파이썬 노드를 포그라운드에서 직접 실행 (nohup과 & 제거!)
  python3 /home/temaat/tmcart/python/test_python/99_tmcart_minipc_node.py

  # # systemd가 데몬관리를 모두 해주므로 스크립트 안의 pgrep 중복 실행 방지 로직과 nohup, & 백그라운드 실행이 더 이상 필요 없음 (sudo vi /systemd/system/tmcart_service.sh 참조)
  # if ! pgrep -f "python3 /home/temaat/tmcart/python/test_python/99_tmcart_minipc_node.py" > /dev/null; then
  #     nohup python3 /home/temaat/tmcart/python/test_python/99_tmcart_minipc_node.py > /dev/null 2>&1 &
  #     echo "run python3 99_tmcart_minipc_node"
  # fi
  ```