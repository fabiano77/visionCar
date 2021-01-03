# visionCar 프로젝트
### raspberry pi 자율주행 모형차 프로젝트
#### 2020년 제 18회 임베디드 소프트웨어 경진대회 - 자율주행모형자동차 부문 예선 출품작
시연 영상 URL : <b>[https://youtu.be/Te_8E83aVds](https://youtu.be/Te_8E83aVds)</br></b>
팀 이름 : 알아서가SSU</br>
팀 구성 : 숭실대학교 김대희, 강민수, 이상민, 이석준


## 개발환경
- PiCar-V (SunFounder's Car Kit for Raspberry Pi)
- Raspberry Pi 4 Model B
- C++
- OpenCV 4.2.0

## 동작 사진
![동작 사진](https://github.com/fabiano77/visionCar/blob/master/%EB%8F%99%EC%9E%91%EC%82%AC%EC%A7%84.png?raw=true)

## 주요 코드파일
- CustomPicar.h, cpp : 라즈베리파이 개발환경 구축 클래스 및 함수
- Driving_DH.h, cpp : 차선인식 및 자율주행 클래스 및 함수
- DetectColorSign.h, cpp : 신호등 인식 및 우선정지 신호 감지 클래스 및 함수 
- Calibration.h : 카메라 캘리브레이션 코드
- SM_drivingAngle.h, cpp : 터널 인식 및 회전교차로 주행 클래스 및 함수.
- main.cpp : 위 클래스 포함 및 수직,수평주차 그리고 장애물 추월 코드 포함 통합코드


### Contributors
- Daehee Kim (fabiano77)<br/>
- Minsoo Kang (3neutronstar)<br/>
- Sangmin Lee (smlee212)<br/>
- Seokjun Lee (ykykyk112)<br/>
<br/>
Soongil Univ.
