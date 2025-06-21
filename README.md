# Example of formal verification of path planning software in a ROS environment 
이 저장소는 Jetson TX2 임베디드 플랫폼 상에서 동작하는 ROS 기반 자율주행 시스템의 정형 모델(Uppaal)과 Go 언어 기반 실행 프레임워크를 제공합니다. 본 구현은 *F1TENTH 자율주행 플랫폼을 참조하여 구성되었으며, ROS 노드의 실시간 동작 특성 분석을 목적으로 설계되었습니다.
*https://github.com/f1tenth/f1tenth_doc

# 프로젝트 개요
Jetson TX2 기반 ROS 경로 계획 시스템의 시간 동작을 정형 검증하고 분석

# 시스템 모델링 특징
각 ROS 노드는 비선점형 단일 콜백 처리 구조로 모델링되며, 메시지는 FIFO 정책에 따라 처리됩니다.

CallbackQueue와 rosSpin 템플릿을 통해 ROS의 이벤트 루프 및 콜백 실행을 모사합니다.

센서 입력부터 제어 명령 생성까지의 전체 데이터 흐름을 시간 제약과 함께 모델링하였습니다.
![image](https://github.com/user-attachments/assets/2a4b22c8-a8b8-4d5d-bef5-f93cb3cd5867)

# Jetson TX2 선택 이유
실제 자율주행 로봇 시스템(F1TENTH 등)에서 널리 사용되는 대표적 플랫폼

ARM 기반 CPU 및 제한된 리소스를 활용한 자원 제약 환경 하의 시간 분석 가능




# 관련 문서
-https://github.com/f1tenth/f1tenth_doc
-https://uppaal.org
