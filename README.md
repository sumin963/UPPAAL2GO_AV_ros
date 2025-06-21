# Example of formal verification of path planning software in a ROS environment 
이 저장소는 Jetson TX2 임베디드 플랫폼 상에서 동작하는 ROS 기반 자율주행 시스템의 정형 모델(Uppaal)과 Go 언어 기반 실행 프레임워크를 제공합니다. 
본 구현은 *F1TENTH 자율주행 플랫폼의 하드웨어 사양을 기준으로 구성되었으며, ROS 미들웨어를 직접 호출함으로써 실제 ROS 시스템과의 연동을 기반으로 동작합니다.

# 프로젝트 개요
Jetson TX2 기반 ROS 경로 계획 시스템의 시간 동작을 정형 검증하고 분석

# 시스템 모델링 특징
센서 입력부터 제어 명령 생성까지의 전체 데이터 흐름을 시간 제약과 함께 모델링하였습니다.

ros_algo_TA.xml
![image](https://github.com/user-attachments/assets/2a4b22c8-a8b8-4d5d-bef5-f93cb3cd5867)

본 연구에서 각 ROS 노드는 Uppaal 모델 내에서 Node 템플릿으로 모델링되며, 주요 기능 모듈(Global Planner, Local Planner, Obstacle Detection, Controller)을 각각 하나의 인스턴스로 나타낸다. 각 노드의 콜백 처리는 CallbackQueue와 rosSpin으로 분리하여 표현되며, 이는 ROS의 비동기 이벤트 처리 구조를 충실히 반영한다. 실행 지연(execution delay)은 Uppaal 내의 delay 구문과 clock을 통해 명시적으로 표현된다. 예를 들어, 메시지를 수신한 이후 콜백이 실행되기까지의 대기 시간은 CallbackQueue에 명시된 bounded delay로 모델링되며, 실행 자체는 rosSpin에서 guard와 invariant를 통해 시간 제약과 함께 표현된다. 그림 8은 이러한 개념을 포괄하는 구조도를 나타내며, 실제 모델 구조와 템플릿에 기반한 구체적인 사례를 반영한다.

노드 내(intra-node) 스케줄링은 ROS의 콜백 큐 기반 비선점(non-preemptive) FIFO 정책을 그대로 따르며, 이는 CallbackQueue에서 메시지가 도착 순서대로 처리되는 형태로 모델링된다. 노드 간(inter-node) 메시지 전달 역시 subQueue로 표현되며, 각 큐는 Polisy_FIFO 자동자에 의해 중앙 집중적으로 제어된다. 

ROS1은 단일 스레드 기반의 ros::spin() 처리 구조와 비결정적인 메시지 큐 처리로 인해 하드 실시간(hard real-time) 요구사항을 만족하기 어렵다. 특히, 콜백 큐의 처리 순서가 메시지 도착 순서와 정확히 일치하지 않거나, 실행 시간 예측이 어려운 경우가 발생한다. 본 연구는 이러한 한계를 정형 모델을 통해 형식적으로 분석하고자 하며, Uppaal을 사용해 시간 제약과 상태 동기화를 명시적으로 정의함으로써 ROS의 비결정적 동작을 이론적으로 보완하고 있다. 

ROS2는 DDS 기반 구조로 더 유연한 QoS 설정이 가능하며, 본 연구의 방식은 ROS2 환경에도 확장 가능하다.

# 관련 문서
-*https://github.com/f1tenth
-https://uppaal.org
