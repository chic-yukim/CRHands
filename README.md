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
2. CRSF SDK 폴더 설정: **두 방법 중 하나 사용**
   1. CRSF SDK 를 "crsf" 이름으로 압축 해제
   2. CRSF SDK 를 프로젝트 디렉토리로 링크
      - 프로젝트 디렉토리에서 cmd 창 실행
      - `mklink /J crsf CRSF_SDK_폴더` 입력
      - 프로젝트 디렉토리에 생성된 'crsf' 폴더와 CRSF SDK 폴더 간의 교차점이 생성되었는지 확인
3. CMake 실행

## 설정 파일

1. 설정 파일들을 프로젝트 디렉토리로 링크

	a. 프로젝트 디렉토리에서 cmd 창 실행

    b. `mklink /J config config-templates` 입력

    c. 프로젝트 디렉토리에 생성된 'config' 폴더와 'config-templates' 폴더 간의 교차점이 생성되었는지 확인

2. 'config' 폴더에는 각 장치에 맞는 설정 파일별 폴더들이 존재 (`CHIC_MOCAP` / `LEAP` / `UNIST_MOCAP`)

	2-1. 기본 설정은 `CHIC_MOCAP`

	2-2. 각 장치에 맞게 `.xml` 파일들을 복사하여 사용

3. 'config/panda3d' 폴더에는 디스플레이 방식별 폴더들이 존재 (`render-pipeline-mono` / `render-pipeline-vr`)

	3-1. 기본 설정은 VR 모드 (`render-pipeline-vr`)

	3-2. 원하는 디스플레이 방식에 맞게 폴더의 파일들을 'config/panda3d/render-pipeline' 폴더에 전체 복사

## CHIC_HAND_MOCAP 설정

* `DynamicModuleConfiguration.xml` 파일을 실험하는 기기에 맞게 수정
* `handmocap.interface` 태그의 `commode` /  `mode` / `port` / `mech` 수정
  * `commode`: `ble` 또는 `serial`
  * `mode`: `left`, `right` 또는 `both`
  * `port`: `commode` 가 `serial` 일 경우에만 작동하며, `COM1` 과 같이 COM 포트 이름을 추가
  * `mech`: Hand MoCAP 장비 번호
* `CRHands.tracker_serial` 태그의 `VIVE 트래커 시리얼 넘버` 수정

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
  ```cpp
  interface_hand_mocap_->SetVibration(Hand_MoCAPInterface::HAND_LEFT, Hand_MoCAPInterface::FingerMask::FINGER_MIDDLE);
  ```

## 기타 기능

* 9번 키를 이용하여 큐브의 위치를 초기 위치로 재설정
