# robotis-bioloid-ml

로보티즈 바이올로이드 프리미엄 키트의 16DOF 휴머노이드 A형 로봇을 딥러닝 모델 기반으로 제어해보는 프로젝트

## 프로젝트 개요

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
