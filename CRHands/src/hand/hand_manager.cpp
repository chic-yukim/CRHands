/**
* Coexistence Reality Software Framework (CRSF)
* Copyright (c) Center of Human-centered Interaction for Coexistence. All rights reserved.
* See the LICENSE.md file for more details.
*/

#include "hand_manager.hpp"

#include <spdlog/spdlog.h>

#include <render_pipeline/rppanda/showbase/showbase.hpp>
#include <render_pipeline/rpcore/globals.hpp>

#include <crsf/CoexistenceInterface/TDynamicStageMemory.h>
#include <crsf/CREngine/TDynamicModuleManager.h>
#include <crsf/RealWorldInterface/TInterfaceManager.h>

#include <crsf/RenderingEngine/TGraphicRenderEngine.h>
#include <crsf/CRModel/TWorld.h>
#include <crsf/CRModel/TWorldObject.h>
#include <crsf/CRModel/TCharacter.h>

#include <crsf/CRModel/TCRHand.h>

#include <crsf/CREngine/TPhysicsManager.h>

#include <crsf/CoexistenceInterface/TAvatarMemoryObject.h>
#include <crsf/System/TPose.h>

#include <openvr_module.h>

#if _MSC_VER > 1900
#include <rpplugins/openvr/plugin.hpp>
#else
#include <openvr_plugin.hpp>
#endif

#include <hand_mocap_module.h>
#include <hand_mocap_interface.h>

#include <kinesthethic_hand_mocap_module.h>
#include <kinesthethic_hand_mocap_interface.h>

#include "main.hpp"

extern spdlog::logger* global_logger;

HandManager::HandManager(CRHands& app, const boost::property_tree::ptree& props) : app_(app), props_(props)
{
    last_hand_mocap_vibrations_[Hand_MoCAPInterface::HAND_LEFT] = Hand_MoCAPInterface::FINGER_NONE;
    last_hand_mocap_vibrations_[Hand_MoCAPInterface::HAND_RIGHT] = Hand_MoCAPInterface::FINGER_NONE;

	setup_hand();
	get_open_vr_module_data();
	setup_hand_event();
}

HandManager::~HandManager() = default;

void HandManager::get_open_vr_module_data()
{
	// set openVR module instance
	auto m = crsf::TDynamicModuleManager::GetInstance()->GetModuleInstance("openvr");
	if (!m)
		return;
	module_open_vr_ = std::dynamic_pointer_cast<OpenVRModule>(m);

	if (props_.get("subsystem.handmocap", false) || props_.get("subsystem.unistmocap", false))
	{
		// Set tracker serial number
		left_wrist_tracker_serial_ = props_.get("tracker_serial.l_wrist", std::string(""));
		right_wrist_tracker_serial_ = props_.get("tracker_serial.r_wrist", std::string(""));

		// Find tracker
		int tracker_count = 0;

		for (int k = 0, k_end = module_open_vr_->GetMaximumDeviceCount(); k < k_end; ++k)
		{
			if (module_open_vr_->GetOpenVRPlugin()->is_tracked_device_connected(k))
			{
				std::string serial_number;
				module_open_vr_->GetOpenVRPlugin()->get_tracked_device_property(serial_number, k, vr::Prop_SerialNumber_String);

				if (serial_number == left_wrist_tracker_serial_
					&& (hand_mocap_mode_ == BOTH || hand_mocap_mode_ == LEFT))
				{
					tracker_index_left_ = k;
					tracker_count++;
				}
				if (serial_number == right_wrist_tracker_serial_
					&& (hand_mocap_mode_ == BOTH || hand_mocap_mode_ == RIGHT))
				{
					tracker_index_right_ = k;
					tracker_count++;
				}

				if (tracker_count >= 2)
					break;
			}
		}
	}
}

void HandManager::setup_hand_event(void)
{
	// calibration
	accept("1", [this](const Event*) {
		if (interface_hand_mocap_)
		{
			hand_->InitScalingParameter();

			interface_hand_mocap_->FingerInit(Hand_MoCAPInterface::HAND_LEFT);
			interface_hand_mocap_->FingerInit(Hand_MoCAPInterface::HAND_RIGHT);

			is_hand_mocap_calibration_ = true;
		}

		if (interface_unist_mocap_)
		{
			interface_unist_mocap_->Calibration();
		}
	});

	// reset cubes position
	accept("9", [this](const Event*) {
		app_.reset_cubes_position();
	});
}

void HandManager::setup_hand(void)
{
	crsf::TWorld* virtual_world = crsf::TGraphicRenderEngine::GetInstance()->GetWorld();



	// set hand property
	crsf::TCRProperty hand_property;
	hand_property.m_propAvatar.SetJointNumber(44);
	hand_property.m_propHand.m_strName = "Hand";
	float m_vec3fZeroToSensor_x = props_.get("handprop.m_vec3fZeroToSensor_x", 0.0f);
	float m_vec3fZeroToSensor_y = props_.get("handprop.m_vec3fZeroToSensor_y", 0.0f);
	float m_vec3fZeroToSensor_z = props_.get("handprop.m_vec3fZeroToSensor_z", 0.0f);
	hand_property.m_propHand.m_vec3ZeroToSensor = LVecBase3(m_vec3fZeroToSensor_x, m_vec3fZeroToSensor_y, m_vec3fZeroToSensor_z);
	hand_property.m_propHand.SetRenderMode(false, false, true);



	// load hand model
	hand_object_ = virtual_world->LoadModel("resources/models/hands/hand.egg");

	node_hmd_ = crsf::CreateObject<crsf::TWorldObject>("HMD");
	virtual_world->AddWorldObject(node_hmd_);
	node_hmd_->AddWorldObject(hand_object_);

	hand_character_ = hand_object_->GetChild("leap_hand")->GetPtrOf<crsf::TCharacter>();
	hand_character_->MakeAllControlJoint();
	hand_property.m_propHand.m_p3DModel = hand_character_;
	hand_property.m_propHand.m_p3DModel_LeftWrist = hand_character_->FindByName("Bip01FBXASC032RFBXASC032Hand001");
	hand_property.m_propHand.m_p3DModel_RightWrist = hand_character_->FindByName("Bip01FBXASC032RFBXASC032Hand");
	LVecBase3 character_scale = hand_character_->GetScale(virtual_world);
	hand_character_->SetScale(character_scale * 0.01f, virtual_world);

	hand_object_->PrintAll();



	// create hand instance
	hand_ = new crsf::TCRHand(hand_property);



	// set display mode
	hand_object_->DisableTestBounding();
	//hand_object_->GetNodePath().node()->set_attrib(DepthTestAttrib::make(RenderAttrib::PandaCompareFunc::M_never));
	//hand_->Set3DModel_Particle(LColorf(0.1, 0.9, 0.9, 1.0f), 0.5);
	//hand_object_->GetNodePath().set_render_mode_wireframe();



    // create physics interactor
    hand_->Set3DModel_StandardPose();
    hand_->ConstructPhysicsInteractor_FullVertex_Sphere(particle_radius_, 0.02, false, false, "left");
	//hand_->ConstructPhysicsInteractor_FixedVertex_Sphere("resources/models/hands/PhysicsInteractorIndex_full_new_left.txt", particle_radius_, false, false, 0, "left");



	// init CHIC mocap setting
	if (props_.get("subsystem.handmocap", false))
	{
		// get hand mocap module interface instance
		interface_hand_mocap_ = dynamic_cast<Hand_MoCAPInterface*>(crsf::TInterfaceManager::GetInstance()->GetInputInterface("Hand_MoCAP"));

		// set hand mocap mode
		std::string mode = interface_hand_mocap_->GetMoCAPMode();
		if (mode == "both")
			hand_mocap_mode_ = BOTH;
		else if (mode == "left")
			hand_mocap_mode_ = LEFT;
		else if (mode == "right")
			hand_mocap_mode_ = RIGHT;
	}



	// init UNIST mocap setting
	if (props_.get("subsystem.unistmocap", false))
	{
		interface_unist_mocap_ = dynamic_cast<Kinesthetic_HandMoCAPInterface*>(crsf::TInterfaceManager::GetInstance()->GetInputInterface("KinestheticHandMoCAP"));
	}



	// set DSM for hand
	if (props_.get("subsystem.leap", false))
	{
		auto amo = crsf::TDynamicStageMemory::GetInstance()->GetAvatarMemoryObjectByName("Hands");
		crsf::TPhysicsManager::GetInstance()->AddTask([this, amo](void) {
			render_hand_leap(amo);
			return false;
		}, "render_hand_leap");
	}
	else if (props_.get("subsystem.handmocap", false))
	{
		auto amo = crsf::TDynamicStageMemory::GetInstance()->GetAvatarMemoryObjectByName("MoCAPHands");
		crsf::TPhysicsManager::GetInstance()->AddTask([this, amo](void) {
			render_hand_mocap(amo);
			return false;
		}, "render_hand_mocap");
	}
	else if (props_.get("subsystem.unistmocap", false))
	{
		auto amo = crsf::TDynamicStageMemory::GetInstance()->GetAvatarMemoryObjectByName("KinestheticMoCAPHands");
		crsf::TPhysicsManager::GetInstance()->AddTask([this, amo](void) {
			render_unist_mocap(amo);
			return false;
		}, "render_unist_mocap");
	}
}