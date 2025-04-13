# robotis-bioloid-ml

로보티즈 바이올로이드 프리미엄 키트의 16DOF 휴머노이드 A형 로봇을 딥러닝 모델 기반으로 제어해보는 프로젝트

## 프로젝트 개요
![image](https://github.com/user-attachments/assets/ca641654-52f4-4fdd-877d-6b99e85c9e03)
- **대상 로봇**: 로보티즈 바이올로이드 프리미엄 키트 (16DOF 휴머노이드 A형)
- **액추에이터**: AX-12A x 16
- **컨트롤러**: CM-530
- **제어 방식**: 딥러닝 모델 기반 모션 생성 및 제어
- **개발 환경**: 윈도우 11 + Docker + ROS1

## 하드웨어 사양

### 메인 PC (전체 시스템 운영)
- CPU: 라이젠 5 9600x
- GPU: RTX3080 10G
- RAM: 64GB
- OS: 윈도우 11 23H2

## 시스템 아키텍처

1. **윈도우 네이티브 환경**:
   - 딥러닝 모델 훈련 및 추론 (PyTorch/TensorFlow)
   - RTX 3080 GPU 활용
   - 모션 데이터 생성 및 컨테이너로 전송

2. **Docker 컨테이너 (Ubuntu + ROS1)**:
   - ROS1 마스터 및 노드 실행
   - URDF 모델 기반 시뮬레이션
   - 실제 로봇 제어 (CM-530 컨트롤러 인터페이싱)

3. **통신 흐름**:
   - 딥러닝 모델 → 모션 데이터 → Docker 컨테이너 → ROS → CM-530 → AX-12A 서보

## 사용 예정 ROS 패키지
https://github.com/billynugrahas/ROBOTIS-BIOLOID

## 개발 환경 설정

### 0. WSL 설치
```powershell
# PowerShell (관리자 권한)
wsl --install
```

### 1. Docker Desktop 설치

1. [Docker Desktop](https://www.docker.com/products/docker-desktop)에서 윈도우용 설치 파일 다운로드 및 설치
2. WSL2 통합 옵션 활성화

### 2. X서버 설정 (GUI 지원)

1. [VcXsrv](https://sourceforge.net/projects/vcxsrv/) 다운로드 및 설치
2. XLaunch 실행 시 다음 설정 적용:
   - Display number: 0
   - Start no client 선택
   - Native opengl 체크 해제
   - Disable access control 체크

### 3. ROS 컨테이너 생성 및 실행

1. ROS Noetic 이미지 가져오기
```powershell
# PowerShell (관리자 권한)
docker pull osrf/ros:noetic-desktop-full
```

2. ROS 컨테이너 실행
```powershell
# PowerShell (관리자 권한)
docker run -it --name ros_robot_control --env="DISPLAY=host.docker.internal:0.0" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --net=host osrf/ros:noetic-desktop-full
```

### 4. ROS 작업 공간 설정

```bash
# 컨테이너 내부
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 5. USB 장치 인식
-  usbipd 설치 : https://github.com/dorssel/usbipd-win/releases
-  명령 프롬프트를 관리자 권한으로 열고 다음과 같이 입력
```bash
# USB 장치 목록 확인
usbipd list

# 예시 출력:
# BUSID  VID:PID    DEVICE                                                        STATE
# 1-10   0403:6001  USB Serial Converter                                          Not shared

# 해당 장치 공유 설정
usbipd bind --busid=4-14

# WSL2에 장치 연결
usbipd attach --wsl --busid=4-14

# WSL2 실행하고 연결된 USB 확인
wsl
ls -al /dev/ttyUSB*

# 기존 컨테이너 종료하고 제거
docker stop ros_robot_control
docker rm ros_robot_control

# 장치 매핑하여 새 컨테이너 생성
docker run -it --name ros_robot_control --env="DISPLAY=host.docker.internal:0.0" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --device=/dev/ttyUSB0:/dev/ttyUSB0 --net=host osrf/ros:noetic-desktop-full
```

### 6. 컨테이너 내에서 장치 접근 권한 설정
```bash
sudo apt update
sudo apt install -y dialout
sudo usermod -aG dialout root
sudo chmod 666 /dev/ttyUSB0
```

### 7. 로보티즈 패키지 설치
```bash
cd ~/catkin_ws/src
git clone https://github.com/ROBOTIS-GIT/ROBOTIS-Framework.git
git clone https://github.com/ROBOTIS-GIT/ROBOTIS-Framework-msgs.git
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git -b noetic-devel
git clone https://github.com/billynugrahas/ROBOTIS-BIOLOID
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## ROS 패키지 준비
```bash
# xacro 호환성 문제 해결
# 모든 launch 파일에서 xacro.py를 xacro로 변경
find ~/catkin_ws/src -name "*.launch" -exec sed -i 's/xacro.py/xacro/g' {} \;

# launch 파일 실행
roslaunch bioloid_description visualize.launch
```

아래 이미지처럼 나오면 성공
![image](https://github.com/user-attachments/assets/986d5275-7468-4298-9515-9b59bad3da4d)

