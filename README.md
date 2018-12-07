# CRHands

## 프로젝트 디렉토리 구성
```
─ project
  ├ (crsf)                  # CRSF SDK
  ├ (config)                # Configuraion files
  ├ config-templates        
  ├ resources
  ├ src    
  └ CMakeLists.txt          # CMake file
```

## 프로젝트 설정
1. CRSF SDK를 다운로드

2. CRSF SDK를 프로젝트 디렉토리로 링크

	a. 프로젝트 디렉토리에서 cmd 창 실행
    
    b. ``mklink /J crsf CRSF_SDK_폴더_디렉토리`` 입력
    
    c. 프로젝트 디렉토리에 생성된 'crsf' 폴더와 CRSF SDK 폴더 간의 교차점이 생성되었는지 확인
    
3. CMake 실행

## 설정 파일
    
1. 설정 파일들을 프로젝트 디렉토리로 링크

	a. 프로젝트 디렉토리에서 cmd 창 실행
    
    b. ``mklink /J config config-templates`` 입력
    
    c. 프로젝트 디렉토리에 생성된 'config' 폴더와 'config-templates' 폴더 간의 교차점이 생성되었는지 확인

2. 'config' 폴더에는 각 장치에 맞는 설정 파일별 폴더들이 존재 (``CHIC_MOCAP`` / ``LEAP`` / ``UNIST_MOCAP``)

	2-1. 기본 설정은 ``CHIC_MOCAP``
    
	2-2. 각 장치에 맞게 ``.xml`` 파일들을 복사하여 사용

3. 'config/panda3d' 폴더에는 디스플레이 방식별 폴더들이 존재 (``render-pipeline-mono`` / ``render-pipeline-vr``)

	3-1. 기본 설정은 VR 모드 (``render-pipeline-vr``)
    
	3-2. 원하는 디스플레이 방식에 맞게 폴더의 파일들을 'config/panda3d/render-pipeline' 폴더에 전체 복사
    
## CHIC_HAND_MOCAP 설정

* ``DynamicModuleConfiguration.xml`` 파일을 실험하는 기기에 맞게 수정
* ``handmocap.interface`` 태그의 ``mode`` / ``port`` / ``mech`` 수정
* ``CRHands.tracker_serial`` 태그의 ``VIVE 트래커 시리얼 넘버`` 수정

## 프로젝트 실행
`CRHands` 프로젝트를 시작 프로젝트로 설정하고 실행

## 프로젝트 실행 이슈

* SteamVR 업데이트로 인해 트래커 설정 별도 조절 필요
	
	1. Steam VR app에서 연결된 트래커 아이콘을 우클릭

	2. Manage Vive Trackers

	3. Select Role 에서 Disabled로 설정

## CHIC_HAND_MOCAP 기능

* 1번 키를 이용하여 Calibration을 수행 (모든 손가락을 일자로 쫙 핀 상태에서 수행)
* Vibration actuator 작동법

	* 함수 원형 : ``interface_hand_mocap_->MakeVibration(int side, const unsigned char data[])``
	* ---
	* side : left = 0 / right = 1
	* ---
	* data : char data[8] = { 0x40, 0x01, 0x05, ``0x07``, ``0x05``, ``0x05``, ``0x4B``, ``0xA5`` };
	* ---
	* data[0] / data[1] / data[2] 는 고정
	* data[3] 이 엄지/검지/중지 각 Vibration actuator를 On/Off 시키는 명령어이며, 3자리 이진수 조합
	* 예를들어, 엄지만 진동을 주려면 = 100(이진수) = 0x04(16진수) / 엄지/검지만 진동을 주려면 = 110(이진수) = 0x06(16진수)
	* ---
	* data[4] / data[5] / data[6] 은 엄지/검지/중지 각 Vibration actuator의 duty cycle
	* duty cycle를 조절하면 실제로 진동의 세기 변화는 미미하고 진동의 떨림(소리)가 커짐 (인지 가용 범위 : 5 ~ 75(십진수))
	* ---
	* data[7] 은 Vibration actuator의 frequency
	* frequency를 조절해야 실제 진동 세기가 변함 (인지 가용 범위: 110 ~200(십진수))
	* ---
	* Vibration stop 함수 : ``interface_hand_mocap_->StopVibration(int side)``

## 기타 기능

* 9번 키를 이용하여 큐브의 위치를 초기 위치로 재설정