# robotis-bioloid-ml

로보티즈 바이올로이드 프리미엄 키트의 16DOF 휴머노이드 A형 로봇을 딥러닝 모델 기반으로 제어해보는 프로젝트

## 프로젝트 개요
![image](https://github.com/user-attachments/assets/ca641654-52f4-4fdd-877d-6b99e85c9e03)
- **대상 로봇**: 로보티즈 바이올로이드 프리미엄 키트 (18DOF 휴머노이드 A형)
- **액추에이터**: AX-12A x 18
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

<details>
<summary>윈도우에서 ROS2만 사용하는 방법</summary>

https://velog.io/@xeno/ROS2-Windows-%EC%84%A4%EC%B9%98
위 링크 대로 따라하는 것이 가장 성공적이였다.

</details>

<details>
<summary>Docker를 사용하는 방법</summary>
   
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
### 4. Docker에서 GPU 사용하도록 설정 (Nvidia)
- wsl을 실행하여 NVIDIA 드라이버가 설치되어 있는지 확인
```bash
nvidia-smi
```
- NVIDIA Container Toolkit 설치
```bash
# 저장소 및 GPG 키 추가
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

# 패키지 업데이트 및 설치
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker
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
docker run --gpus all -it --name ros_robot_control -e DISPLAY=host.docker.internal:0.0 -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute -e LIBGL_ALWAYS_INDIRECT=0 --device=/dev/ttyUSB0:/dev/ttyUSB0 --network=host osrf/ros:noetic-desktop-full
```

### 6. ROS 작업 공간 설정

```bash
# 컨테이너 내부
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 7. 컨테이너 내에서 장치 접근 권한 설정
```bash
sudo apt update
sudo usermod -aG dialout root
sudo chmod 666 /dev/ttyUSB0
```

### 8. 로보티즈 패키지 설치
```bash
# 기본 패키지 설치
sudo apt install -y nano gedit git curl
sudo apt install -y libgl1-mesa-glx libgl1-mesa-dri libglew-dev libglvnd0 libglvnd-dev libegl1 libgles2 mesa-utils

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

## 로봇 애니메이션 실행
새 터미널에서 도커 컨테이너 접속
```bash
docker exec -it ros_robot_control bash

# 스크립트 파일 생성
cd ~/catkin_ws/src/ROBOTIS-BIOLOID/bioloid_description
mkdir -p scripts
cd scripts
nano walking_animation.py
```

다음 파이썬 코드 입력
```python
#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time

class WalkingAnimation:
    def __init__(self):
        rospy.init_node('walking_animation')
        self.joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.rate = rospy.Rate(10)  # 10Hz
        
        # 모든 관절 이름 (실제 로봇 모델에 맞게 수정)
        self.joint_names = [
            'left_shoulder_joint', 'left_arm_joint', 'left_forearm_joint',
            'right_shoulder_joint', 'right_arm_joint', 'right_forearm_joint',
            'left_hip_joint', 'left_hip_2_joint', 'left_thigh_joint', 'left_knee_joint', 'left_ankle_joint', 'left_foot_joint',
            'right_hip_joint', 'right_hip_2_joint', 'right_thigh_joint', 'right_knee_joint', 'right_ankle_joint', 'right_foot_joint'
        ]
        
        # 걷기 애니메이션을 위한 키프레임들
        self.walking_frames = self.generate_walking_frames()
        self.current_frame = 0
        
    def generate_walking_frames(self):
        frames = []
        steps = 20  # 키프레임 수
        
        for i in range(steps):
            t = i / float(steps)
            phase = 2 * math.pi * t
            
            # 기본 자세 (모든 관절은 0)
            positions = [0.0] * len(self.joint_names)
            
            # 다리 관절 설정 (위상차를 두고 사인파로 움직임)
            # 왼쪽 다리
            positions[6] = 0.1 * math.sin(phase)  # left_hip_joint
            positions[7] = 0.1 * math.sin(phase)  # left_hip_2_joint
            positions[8] = 0.4 * math.sin(phase)  # left_thigh_joint
            positions[9] = -0.8 * abs(math.sin(phase))  # left_knee_joint (항상 음수 값)
            positions[10] = 0.4 * math.sin(phase + math.pi/4)  # left_ankle_joint
            positions[11] = 0.1 * math.sin(phase)  # left_foot_joint
            
            # 오른쪽 다리 (반대 위상)
            positions[12] = 0.1 * math.sin(phase + math.pi)  # right_hip_joint
            positions[13] = 0.1 * math.sin(phase + math.pi)  # right_hip_2_joint
            positions[14] = 0.4 * math.sin(phase + math.pi)  # right_thigh_joint
            positions[15] = -0.8 * abs(math.sin(phase + math.pi))  # right_knee_joint
            positions[16] = 0.4 * math.sin(phase + math.pi + math.pi/4)  # right_ankle_joint
            positions[17] = 0.1 * math.sin(phase + math.pi)  # right_foot_joint
            
            # 팔 관절 (다리와 반대로 움직임)
            positions[0] = 0.2 * math.sin(phase + math.pi)  # left_shoulder_joint
            positions[1] = 0.4 * math.sin(phase + math.pi)  # left_arm_joint
            positions[2] = 0.1 * math.sin(phase + math.pi)  # left_forearm_joint
            
            positions[3] = 0.2 * math.sin(phase)  # right_shoulder_joint
            positions[4] = 0.4 * math.sin(phase)  # right_arm_joint
            positions[5] = 0.1 * math.sin(phase)  # right_forearm_joint
            
            frames.append(positions)
        
        return frames
    
    def publish_frame(self, positions):
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = self.joint_names
        joint_state.position = positions
        
        self.joint_pub.publish(joint_state)
    
    def run(self):
        print("Walking animation started. Press Ctrl+C to stop.")
        while not rospy.is_shutdown():
            # 현재 프레임의 관절 위치 발행
            self.publish_frame(self.walking_frames[self.current_frame])
            
            # 다음 프레임으로 이동
            self.current_frame = (self.current_frame + 1) % len(self.walking_frames)
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        animation = WalkingAnimation()
        animation.run()
    except rospy.ROSInterruptException:
        pass
```

catkin 작업 공간 빌드 후 스크립트 실행
```bash
# 파이썬 파일 실행 권한 제공
chmod +x walking_animation.py

# catkin 작업 공간 빌드
cd ~/catkin_ws
catkin_make
source devel/setup.bash

# 스크립트 실행
rosrun bioloid_description walking_animation.py
```

현재 로봇의 관절 이름 확인
```bash
rostopic echo /joint_states -n1
```
</details>

