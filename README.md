# HaniumProject2017
Utilisation of SLAM technology : Robot to Investigate Dangerous Areas

### 개발된 로봇의 하드웨어 구성도는 다음과 같다.

![Alt text](http://postfiles2.naver.net/MjAxNzEwMTZfMTU5/MDAxNTA4MTMxNDY4MTI3.q6_fw5u_W19HwbOOlx3kHs8FP-vjrCz6GHmGnBfRsDcg.KmxbfkIWaOMOb-PPAZjQtfVYlFibVRWIhuRhrBE6viMg.PNG.katie0809/그림1.png?type=w2)

### 개발된 로봇의 소프트웨어 구성도는 다음과 같다.

![Alt text](http://postfiles13.naver.net/MjAxNzEwMTZfMjMw/MDAxNTA4MTMxNDY4NTA3.b6N1WNcowwyf31Cwl2gsuucgQPst_fwHGgl6ET-_VtIg.5YZSkKDFAXPUxPZ-wB46bgCGOwguuEg9A1_826MsfOgg.PNG.katie0809/그림2.png?type=w2)

Kinect : 키넥트 깊이 센서로부터 깊이 영상 데이터를 받아온다.</br>
depthimage_to_laserscan
* 깊이 영상 데이터를 SLAM에 사용할 수 있는 laserscan 센서 데이터로 변환한다.</br>
* 변환된 데이터는 LaserScan 타입의 /scan 토픽을 발행한다.
* LaserScan 데이터 활용해 hector slam이 구동된다.

move_base
* move_base 노드는 로봇의 모션 계획을 총괄.</br>
* 변환된 laser scan 센서데이터는 이러한 모션 계획에 필요한 costmap 생성에 사용됨.</br>

hector_mapping
* /tf 정보와 /scan 정보를 활용해 hector SLAM이 시작된다.</br>
* SLAM에 필요한 odometry 정보는 hector SLAM내에서 자체적으로 추정하여 사용.</br>

Tf
* 로봇 base frame과 키넥트 camera depth frame간의 상대위치 변환 정보를 발행한다.</br>

explore_server
* ROS 액션서버</br>
* Frontier exploration을 시행한다.</br>
* move_base 기반의 movement 목표를 생성. 연결된 클라이언트에게 목표를 전달한다.</br>

frontier_explore.py
* 점유 격자지도로 발행된 costmap을 받아 다음 frontier를 탐지하고, frontier의 위치를 확인한다.</br>
* 로봇을 frontier가 존재하는 지점으로 보낸다. </br>
* 일정 횟수 이상 탐사가 완료되면 생성된 costmap을 참고해 탐사 시작 위치로 돌아온다.</br>

local_costmap / global_costmap
local_planner, control.py
* 생성된 costmap을 기반으로 local planner는 최종 속도 명령을 /cmd_vel 토픽으로 발행한다. 
* Control.py코드 상에서 이를 적절한 바퀴 속도값으로 변환, 아두이노에 전달한다.

Arduino
* 시리얼 통신을 활용해 변환된 바퀴 별 모터 속도값이 아두이노에 전달된다.
* 아두이노는 이를 받아 실제 로봇을 구동시킨다.


### 소스 설명

Mappingbot 패키지 구성
  
	개발된 로봇은 라즈베리 파이 내에 ROS를 설치하여 제어한다. 이때 ROS에서 기본적으로 제공되는 소프트웨어 패키지에 더해, 
  사용자가 직접 필요 소프트웨어를 포함한 패키지를 정의하여 사용할 수 있다. 본 로봇의 구동을 위해 생성한 패키지명은 Mappingbot으로, 
  라즈베리 파이의 ~/catkin_ws/src 폴더 내에 생성되어있다. Mappingbot 패키지는 다음과 같이 구성되어있다.
	 
 CMakeLists.txt : 패키지 빌드 설정파일. Mappingbot 패키지 빌드를 위한 필요 설정들이 정의되어있다. 로스의 빌드 시스템인 catkin은 기본적으로 CMake를 이용한다. 이때 패키지 내 기본으로 구성되는 CMakeLists.txt 파일에 빌드 환경을 기술하여둔다. Mappingbot 패키지의 구동을 위해서는 rospy, geometry_msgs, sensor_msgs 등이 필요한다. 
 package.xml : 패키지 설정파일. 패키지 실행 및 빌드 종속성과 패키지 내 소스코드 사용에 필요한 기타 소프트웨어를 정의한다. 패키지의 이름, 버전, 설명, 관리자정보 등을 포함할 수 있다.

 /launch : roslaunch에 사용되는 스크립트 폴더
	- mapping_default.launch : Hector SLAM을 odometer 데이터 없이 mappingbot에서 사용하도록 정의 후 구동시키는 런치파일. 
	- move_base_gmappingVer.launch : 터틀봇이 실행된 가제보 가상 환경에서 gmapping으로 SLAM을 구동한다. 		로스 기본 주행스택을 실행하여 임의의 목표지점까지의 경로를 계획, 주행한다.
	- move_base_frontier.launch : 실제 환경에서의 mappingbot 실행을 위한 런치파일.
	- move_base.launch.xml : 로스 기본 주행스택 실행 정의파일. 모션 계획을 총괄한다.
	- global_map.launch : frontier exploration의 explore server를 실행하고, explore_global_costmap 의 세부 설정을 기술한다.

 /params : 관련 파라미터 폴더
	- cost map : 내비게이션에서는  점유 격자 지도(occupancy grid map) 를 사용하게 된다. 이 점유 격자 지도를 기반으로, 로봇의 위치와 센서로 부터 얻은 주변 정보를 이용하여 각 픽셀을 장애물/이동불가영역/이동가능영역 으로 계산하게 되는데 이때에 사용되는 개념이 costmap 이다.
		- costmap_common_params.yaml : 모션 계획에 필요한 공통 파라미터 정의.
		- local_costmap_params.yaml : 국부 모션 계획에 필요한 파라미터 정의.
		- global_costmap_params.yaml: 전역 모션 계획에 필요한 파라미터 정의.
	- base_local_planner_params.yaml : costmap을 통해 계획된 모션 계획을 바탕으로 로봇에게 이동 속도 명령 전달.

 /src : 소스 코드 폴더. 
	- frontier_explore.py : 미지의 영역에 대한 전방위 탐사를 진행한다. 탐사 종료 시 생성된 지도를 참고해 처음 탐사위치로 돌아온다.
	- control.py : 발행되는 주행 제어 명령을 받아 변환 후 아두이노로 전달한다. 아두이노에서 실제 수신한 속도 데이터를 받아와 /arduino 토픽으로 발행한다.

 /srv : rosservice 폴더.
	- GetNextFrontier.srv : 탐사할 다음 frontier의 위치를 geometry_msgs/PoseStamped 타입으로 발행한다.
	- UpdateBoundaryPolygon.srv : 탐사를 진행할 범위 반경을 지정한다.

 /urdf : Mappingbot 하드웨어 설정파일.
	- mappingbot.urdf : mappingbot 3d 모델링 파일
