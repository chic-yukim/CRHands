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

## 프로젝트 실행
`CRHands` 프로젝트를 시작 프로젝트로 설정하고 실행