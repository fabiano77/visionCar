# visionCar(가칭)프로젝트

제18회 임베디드SW경진대회 예선 준비작\
팀 번호 - 2020\
팀 이름 - 알아서가SSU\
소속 - 숭실대학교\
팀원 - 김대희, 강민수, 이상민, 이석준\


raspberry pi 기반 자율주행 모형차 프로젝트\
C++ OpenCV 사용.\


-CustomPicar.h \
-CustomPicar.cpp : 라즈베리파이 개발환경 구축 클래스 \
-Driving_DH.h \
-Driving_DH.cpp : 차선인식 및 주행 클래스 \
-DetectColorSign.h \
-DetectColorSign.cpp : 신호등 인식 및 우선정지 신호 감지 클래스 \
-Calibration.h : 카메라 캘리브레이션 코드 \
-SM_drivingAngle.h\
-SM_drivingAngle.cpp : 터널 인식 및 회전교차로 주행 클래스.\
-main.cpp : 위 클래스 포함 및 수직,수평주차 그리고 장애물 추월 코드 포함 통합코드\
