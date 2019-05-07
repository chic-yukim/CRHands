# CRHands

## 프로젝트 디렉토리 구성
```
─ project
  ├ (crsf)                  # (1) CRSF SDK
  ├ CRHands
  |  ├ config               # Configuraion files
  |  ├ resources
  |  ├ src
  |  └ CMakeLists.txt       # CMake file
  └ CMakeLists.txt          # CMake file
```

### 프로젝트 설정
1. CRSF SDK를 다운로드
2. CRSF SDK 폴더 설정 (**두 방법 중 하나 사용**)
   1. CRSF SDK 를 **현재** 폴더에 "crsf" 이름으로 압축 해제 (단일 프로젝트)
   2. CRSF SDK 를 **상위** 폴더에 "crsf" 이름으로 압축 해제 (멀티 프로젝트)
3. CMake 실행



## 설정 파일
1. `config` 폴더에는 각 장치에 맞는 설정 파일별 폴더들이 존재 (`CHIC_MOCAP` / `LEAP` / `UNIST_MOCAP`)
   - 기본 설정은 `CHIC_MOCAP`
   - 각 장치에 맞게 `.xml` 파일들을 복사하여 사용
2. `config/panda3d` 폴더에는 디스플레이 방식별 폴더들이 존재 (`render_pipeline_mono` / `render_pipeline_vr`)
   - 기본 설정은 VR 모드 (`render_pipeline_vr`)
   - 원하는 디스플레이 방식에 맞게 폴더의 파일들을 `config/panda3d/render_pipeline` 폴더에 전체 복사

자세한 설정 방법은 다음 항목 참조


### 공통
- `config/panda3d/render_pipeline/plugins.yaml` 설정
  - 맨 아래에 있는 `openvr` 항목에서 `update_camera_pose` 항목을 `false` 로 하면, HMD 움직임과 분리해서 자유롭게 카메라를 이동시킬 수 있다.



### 립모션
- https://github.com/chic-yukim/crsf/wiki/CRSF-Modules:-leapmotion 페이지 참조
- `DynamicModuleConfiguration.xml`
  - `subsystem.leap` 태그 값을 true 로 변경 (`handmocap`, `unistmocap` 은 false 로 변경)

### CHIC 핸드모캡 설정
- https://github.com/chic-yukim/crsf/wiki/CRSF-Modules:-hand_mocap 페이지 참조
- `DynamicModuleConfiguration.xml`
  - `subsystem.handmocap` 태그 값을 true 로 변경 (`leap`, `unistmocap` 은 false 로 변경)

### UNIST 모캡
- `DynamicModuleConfiguration.xml` 파일을 실험하는 기기에 맞게 수정
  - `hand_mocap.interface` 태그
    - port: `COM1` 과 같이 COM 포트 이름을 추가

      ![com-port-image](https://user-images.githubusercontent.com/937305/56501079-71a1f180-6548-11e9-9823-f7dba82b714d.png)

    - version: `new` (기본값), `old`
  - `subsystem.unistmocap` 태그 값을 true 로 변경 (`leap`, `handmocap` 은 false 로 변경)
- `DeviceConfiguration.xml` 파일 설정
  ```xml
  <motion>
      <inputinterface>KinestheticHandMoCAP</inputinterface>
      <outputinterface>KinestheticMoCAPHands</outputinterface>
      <fps>60</fps>
      <joint>26</joint>
  </motion>
  ```



## 프로젝트 실행
`CRHands` 프로젝트를 시작 프로젝트로 설정하고 실행

### 프로젝트 실행 이슈
- SteamVR 업데이트로 인해 트래커 설정 별도 조절 필요
  1. SteamVR 에서 연결된 트래커 아이콘을 우클릭 후 Manage Vive Trackers 메뉴 클릭

     ![steamvr-tracker-right-click](https://user-images.githubusercontent.com/937305/56501922-9481d500-654b-11e9-8adf-8b458d803362.png)

  2. 연결된 트래커 확인

     ![manage-trackers1](https://user-images.githubusercontent.com/937305/56501980-c85cfa80-654b-11e9-85e0-115d80e3de24.png)

  3. Select Role 에서 Disabled 로 설정

     ![manage-trackers2](https://user-images.githubusercontent.com/937305/56502041-fe01e380-654b-11e9-80b2-f03f06bbf243.png)

  4. 변경 완료

     ![manage-trackers3](https://user-images.githubusercontent.com/937305/56502090-1ffb6600-654c-11e9-9d1b-ff3fa7ae0dba.png)

  참고로, "Manage Vive Trackers" 를 클릭해도 창이 나타나지 않는 경우, Vive 를 쓴 후 VR 안에서 설정 메뉴를 통해 동일하게 변경이 가능함.



## App 기능

### CHIC 핸드모캡
- 1번 키를 이용하여 Calibration 을 수행 (모든 손가락을 일자로 쫙 핀 상태에서 수행)

### 기타
- 9번 키를 이용하여 큐브의 위치를 초기 위치로 재설정
