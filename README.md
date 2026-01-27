# RB_SpotATS_System

Isaac Lab 기반 Spot 로봇에 **ATS(Auto Targeting System)** 모듈을 통합하고,  
환경 변화에 대응하기 위해 **강화학습 정책을 재학습하여 적용한 로봇 시스템 프로젝트**입니다.

본 프로젝트는 강의 및 공개 예제를 기반으로 시작했으며,  
ATS 추가로 인해 변경된 로봇 질량 분포 및 센서 구성에 맞게  
**시스템 설정과 정책을 직접 수정·검증**했습니다.

---

## 프로젝트 개요

- **로봇 플랫폼**: Spot (Isaac Lab Simulation)
- **시뮬레이터**: NVIDIA Isaac Sim / Isaac Lab
- **미들웨어**: ROS 2
- **강화학습**: PPO 기반 locomotion policy
- **주요 확장 요소**: ATS (Auto Targeting System) 통합

---

## 시스템 구성

- ATS 모듈 추가로 로봇 상부 질량 및 관성(Inertia) 변화 발생
- 기존 locomotion 정책으로는 주행 안정성 저하 가능성 존재
- 이에 따라 다음과 같은 대응을 수행했습니다.
  - 강화학습 정책 **재학습**
  - runtime 설정(cfg) 분리 및 관리
  - SLAM / TF / odometry 파라미터 재정렬

위 변경 사항들은 시뮬레이터 및 ROS2 연동 환경에서  
실제 주행 안정성에 영향을 주는 요소들을 중심으로 적용했습니다.

---

## 정책(Policy) 구성

본 프로젝트에는 두 개의 정책이 포함되어 있습니다.

- `spot_policy_baseline.pt`
  - 기존 강의 예제에서 제공되는 기본 locomotion 정책

- `spot_policy_ats_retrained.pt`
  - ATS 모듈 통합으로 변경된 질량 분포 및 센서 구성을 반영하여  
    **새로 재학습한 locomotion 정책**

정책 로딩 경로는 코드에 하드코딩하지 않고,  
`configs/default.yaml`을 통해 관리하도록 구성했습니다.

---

## 강화학습 학습 구조

강화학습 정책의 **학습 파이프라인은 별도의 외부 프로젝트
[`RL_SpotATS`](https://github.com/myoungjelee/RL_SpotATS)에서 관리**했습니다.  
해당 프로젝트는 **Isaac Lab 원본 코드를 직접 수정하지 않고**,  
Isaac Lab 외부 프로젝트 구조를 사용해 구성했습니다.

본 레포지토리(`RB_SpotATS_System`)는  
외부 프로젝트에서 학습된 정책을 가져와 **Spot ATS 시스템에 적용하고,  
시스템 관점에서 검증하는 역할**에 집중합니다.

이를 통해 다음과 같은 구조를 유지했습니다.

- 학습(RL)과 시스템(System) 역할 분리
- Isaac Lab 업데이트에 대한 유지보수 용이성 확보
- 정책 교체 및 비교 실험이 가능한 구조

---

## 주요 변경 사항

- ATS 통합에 따른 **강화학습 정책 재학습**
- 정책 경로를 cfg 기반으로 분리하여 실험/비교 가능 구조로 개선
- SLAM Toolbox 파라미터 조정
  - odometry / TF(frame) 정합성 문제 해결
  - `map → odom → base_link` 프레임 체계 안정화

---

## 브랜치 전략

- `dev`: 실험 및 설정 검증
- `main`: 검증 완료된 결과만 병합

작은 변경 사항도 dev 브랜치에서 검증 후 main에 병합하는 방식으로 관리했습니다.

---

## 참고 및 출처

- 본 프로젝트는 강의 및 공개 예제를 기반으로 시작했습니다.
- 단, ATS 통합, 강화학습 정책 재학습, 시스템 설정 수정은 직접 수행했습니다.

---

## 실행 환경

- Ubuntu 22.04
- ROS 2 Humble
- Isaac Sim 5.0 / Isaac Lab
- Python 3.11
