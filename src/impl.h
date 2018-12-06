#pragma once

#include "main.h"

#include <openvr_module.h>
#include <openvr_plugin.hpp>
#include <hand_mocap_module.h>
#include <hand_mocap_interface.h>

#include <render_pipeline/rppanda/showbase/showbase.hpp>
#include <render_pipeline/rpcore/globals.hpp>

#include <crsf/CREngine/TDynamicModuleManager.h>
#include <crsf/RealWorldInterface/TInterfaceManager.h>

#include <crsf/CoexistenceInterface/TDynamicStageMemory.h>

#include <crsf/CRModel/TWorldObject.h>
#include <crsf/CRModel/TCharacter.h>

#include <crsf/CRModel/TCRModel.h>
#include <crsf/CRModel/TCube.h>
#include <crsf/CRModel/TSphere.h>
#include <crsf/CRModel/TCylinder.h>
#include <crsf/CRModel/TBackground.h>

#include <crsf/CRModel/TCRHand.h>
#include <crsf/CRModel/THandPhysicsInteractor.h>
#include <crsf/CREngine/THandInteractionEngineConnector.h>

struct Physics::Impl
{
	const double M_PI = std::acos(-1);

	Impl(Physics& self);

	//////////////////////////////////////////////////////////////////////////
	// Graphics update
	AsyncTask::DoneStatus UpdateGraphics(rppanda::FunctionalTask*);

	//////////////////////////////////////////////////////////////////////////
	// Event
	void SetupEvent(void);

	//////////////////////////////////////////////////////////////////////////
	// Physics
	void StartPhysicsSimulation(void);

	bool BaseCollisionEvent(const std::shared_ptr<crsf::TCRModel>& MyModel, const std::shared_ptr<crsf::TCRModel>& EventedModel);
	bool BaseSeparationEvent(const std::shared_ptr<crsf::TCRModel>& MyModel, const std::shared_ptr<crsf::TCRModel>& EventedModel);

	//////////////////////////////////////////////////////////////////////////
	// Scene
	void SetupScene(void);

	void SetupBackgroundSky(void);
	void SetupGround(void);
	void SetupTable(void);
	void SetupSimpleObject(void);
	void DoSimpleObject(void);
	void SetupCubes(void);
	void SetupSHAPTest(void);
	void SetupConstraintObject(void);

	void BasicGraphicsSetup(const std::shared_ptr<crsf::TCRModel>& obj, const LColorf& color);
	void BasicGraphicsPhysicsSetup(const std::shared_ptr<crsf::TCRModel>& obj, const LColorf& color);

	//////////////////////////////////////////////////////////////////////////
	// Hand
	void SetupHand(void);
	void SetupHand_trans(void);

	void RenderHand_LEAPMOTION(crsf::TAvatarMemoryObject *pAvatarMemoryObject);
	void RenderHand_LEAPMOTION_trans(crsf::TAvatarMemoryObject *pAvatarMemoryObject);
	void RenderHand_MOCAP(crsf::TAvatarMemoryObject *pAvatarMemoryObject);
	void RenderHand_MOCAP_trans(crsf::TAvatarMemoryObject *pAvatarMemoryObject);

	bool Interactor_Collision_Event(const std::shared_ptr<crsf::TCRModel>& MyModel, const std::shared_ptr<crsf::TCRModel>& EventedModel);
	bool Interactor_Inside_Event(const std::shared_ptr<crsf::TCRModel>& MyModel, const std::shared_ptr<crsf::TCRModel>& EventedModel);
	bool Interactor_Separation_Event(const std::shared_ptr<crsf::TCRModel>& MyModel, const std::shared_ptr<crsf::TCRModel>& EventedModel);

	bool Object_Collision_Event(const std::shared_ptr<crsf::TCRModel>& MyModel, const std::shared_ptr<crsf::TCRModel>& EventedModel);
	bool Object_Separation_Event(const std::shared_ptr<crsf::TCRModel>& MyModel, const std::shared_ptr<crsf::TCRModel>& EventedModel);
	bool Object_Update_Event(const std::shared_ptr<crsf::TCRModel>& MyModel);
	bool Object_UpdateAfterCD_Event(const std::shared_ptr<crsf::TCRModel>& MyModel);

	bool Object_Update_Event_Simple(const std::shared_ptr<crsf::TCRModel>& MyModel);
	bool Object_UpdateAfterCD_Event_Simple(const std::shared_ptr<crsf::TCRModel>& MyModel);

	//////////////////////////////////////////////////////////////////////////
	// MoCAP
	LVecBase3 RotateHandMocapPos(LVecBase3 vec3, LQuaternionf quat);
	void SavePoseData();

	//////////////////////////////////////////////////////////////////////////
	// VIVE
	void GetOpenVRModuleData();

	//////////////////////////////////////////////////////////////////////////
	// Math
	float CalcDistance(const LVecBase3& p1, const LVecBase3& p2);





	//////////////////////////////////////////////////////////////////////////
	// Self instance
	Physics& self_;

	//////////////////////////////////////////////////////////////////////////
	// Scene parameter
	std::shared_ptr<crsf::TSphericalSky> m_backgroundSky = nullptr;

	std::shared_ptr<crsf::TCube> m_cubeGround = nullptr;

	std::shared_ptr<crsf::TWorldObject> m_compoundTable = nullptr;

	std::vector<std::shared_ptr<crsf::TCube>> m_vecCube;

	std::shared_ptr<crsf::TSphere> m_obj_SHAP_Tip;
	std::shared_ptr<crsf::TCube> m_obj_SHAP_Lateral;
	std::shared_ptr<crsf::TCylinder> m_obj_SHAP_Tripod;
	std::shared_ptr<crsf::TSphere> m_obj_SHAP_Spherical;
	std::shared_ptr<crsf::TCylinder> m_obj_SHAP_Power;
	std::shared_ptr<crsf::TCube> m_obj_SHAP_Extension;

	std::vector<std::shared_ptr<crsf::TCRModel>> m_vecobjs_SHAP;

	std::shared_ptr<crsf::TCube> m_SimpleObject_1;
	std::shared_ptr<crsf::TCube> m_SimpleObject_2;

	std::vector<std::shared_ptr<crsf::TCRModel>> m_vecConstraintObjects;
	std::shared_ptr<crsf::TCRModel> m_CurrentConstraintObject;

	//////////////////////////////////////////////////////////////////////////
	// Hand parameter
	// Model
	crsf::TCRHand* m_pHand = nullptr;
	crsf::TWorldObject* m_pTwoHand_Object = nullptr;
	crsf::TCharacter* m_pTwoHand_Character = nullptr;
	crsf::THandInteractionEngineConnector* m_pHandInteractionEngineConnector = nullptr;

	crsf::TCRHand* m_pHand_trans = nullptr;
	crsf::TWorldObject* m_pTwoHand_Object_trans = nullptr;
	crsf::TCharacter* m_pTwoHand_Character_trans = nullptr;
	crsf::THandInteractionEngineConnector* m_pHandInteractionEngineConnector_trans = nullptr;

	int  m_nCountUpdatedJoints;
	bool m_bFullCountingJoint;
	int  m_nFullCountingJoint;

	// Physics interactor
	float m_fInteractor_radius = 0.0025f;
	std::map<std::string, std::vector<std::shared_ptr<crsf::TCRModel>>> current_contacted_particles;

	//////////////////////////////////////////////////////////////////////////
	// MoCAP parameter
	Hand_MoCAPInterface* m_interfaceHand_MoCAP = nullptr;

	float m_f3DModelOffset[2][5][4];

	int m_nCalibrationIter;
	bool m_bCalibration;
	bool m_bScaling;

	enum EHandMoCAPMode
	{
		BOTH, LEFT, RIGHT, VIEW, TEST
	};
	EHandMoCAPMode eHandMoCAPMode;

	// user study - accuracy of pose
	std::vector<LVecBase3> m_vecPoseData[3]; // 1/2/3 finger
	int m_nPoseSaveMode = 0;
	int m_nPoseSaveFinger[3];

	//////////////////////////////////////////////////////////////////////////
	// VIVE parameter
	std::shared_ptr<OpenVRModule> m_moduleOpenVR = nullptr;

	std::shared_ptr<crsf::TWorldObject> m_pHmd_node;
	std::shared_ptr<crsf::TWorldObject> m_pHmd_node_trans;

	std::string m_strL_wrist_tracker_serial;
	std::string m_strR_wrist_tracker_serial;

	int tracker_index_left = -1;
	int tracker_index_right = -1;

	LVecBase3 m_vec3Tracker;
	LQuaternionf m_quatTracker;
	LVecBase3 m_vec3Trackers[2];
	LQuaternionf m_quatTrackers[2];

	//////////////////////////////////////////////////////////////////////////
	// Device state
	enum EHandDeviceState
	{
		LEAP_MOTION, HAND_MOCAP, KINESTHETIC_HAND_MOCAP
	};
	EHandDeviceState eHandDevice;

	//////////////////////////////////////////////////////////////////////////
	// Test parameter
	crsf::TWorldObject* test_obj = nullptr;
};
