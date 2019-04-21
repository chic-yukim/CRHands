/**
* Coexistence Reality Software Framework (CRSF)
* Copyright (c) Center of Human-centered Interaction for Coexistence. All rights reserved.
* See the LICENSE.md file for more details.
*/

#include "hand_manager.hpp"

#include <spdlog/logger.h>

#include <render_pipeline/rppanda/showbase/showbase.hpp>
#include <render_pipeline/rpcore/globals.hpp>
#include <render_pipeline/rpcore/render_pipeline.hpp>
#include <render_pipeline/rpcore/pluginbase/manager.hpp>

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

#include <leapmotion_module.h>
#include <leapmotion_interface.h>

#include <hand_mocap_module.h>
#include <hand_mocap_interface.h>

#include <kinesthethic_hand_mocap_module.h>
#include <kinesthethic_hand_mocap_interface.h>

#include "hand/hand_leap.hpp"
#include "hand/hand.hpp"
#include "main.hpp"
#include "user.hpp"

extern spdlog::logger* global_logger;

HandManager::HandManager(MainApp& app, const boost::property_tree::ptree& props) : app_(app), props_(props)
{
    for (auto&& index: tracker_indices_)
        index = -1;

    last_hand_mocap_vibrations_[Hand_MoCAPInterface::HAND_LEFT] = Hand_MoCAPInterface::FINGER_NONE;
    last_hand_mocap_vibrations_[Hand_MoCAPInterface::HAND_RIGHT] = Hand_MoCAPInterface::FINGER_NONE;

    find_trackers();
	setup_hand();
	setup_hand_event();
}

HandManager::~HandManager() = default;

void HandManager::find_trackers()
{
    auto plugin_manager = app_.rendering_engine_->GetRenderPipeline()->get_plugin_mgr();
    if (!plugin_manager->is_plugin_enabled("openvr"))
        return;

    // reset
    for (auto&& index: tracker_indices_)
        index = -1;

    auto openvr_plugin = static_cast<rpplugins::OpenVRPlugin*>(plugin_manager->get_instance("openvr")->downcast());
    for (int k = vr::k_unTrackedDeviceIndex_Hmd + 1; k < vr::k_unMaxTrackedDeviceCount; ++k)
    {
        if (!openvr_plugin->get_vr_system()->IsTrackedDeviceConnected(k))
            continue;

        switch (openvr_plugin->get_tracked_device_class(k))
        {
        case vr::TrackedDeviceClass_GenericTracker:
            if (tracker_indices_[HAND_INDEX_LEFT] == -1)
                tracker_indices_[HAND_INDEX_LEFT] = k;
            else if (tracker_indices_[HAND_INDEX_RIGHT] == -1)
                tracker_indices_[HAND_INDEX_RIGHT] = k;
            break;
        default:
            break;
        }
    }
}

void HandManager::swap_trackers()
{
    std::swap(tracker_indices_[0], tracker_indices_[1]);
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
	// create hand instance
    auto hand = app_.user_->get_hand();
	hand_ = app_.user_->get_hand()->get_hand();

	// set display mode
	//hand_object_->GetNodePath().node()->set_attrib(DepthTestAttrib::make(RenderAttrib::PandaCompareFunc::M_never));
	//hand_->Set3DModel_Particle(LColorf(0.1, 0.9, 0.9, 1.0f), 0.5);
	//hand_object_->GetNodePath().set_render_mode_wireframe();



    // create physics interactor
    hand->setup_physics_interactor(particle_radius_);

	// attach listener function to physics interactor
	hand_->SetPhysicsInteractor_RigidObject();
	hand_->SetPhysicsInteractor_Mass(0.0f);

	// grasp algorithm
	hand_pointer_.push_back(hand_->Get3DModel_RightWrist());
	hand_pointer_.push_back(hand_->Get3DModel_LeftWrist());

	// init CHIC mocap setting
    if (props_.get("subsystem.handmocap", false))
    {
        // get hand mocap module interface instance
        interface_hand_mocap_ = dynamic_cast<Hand_MoCAPInterface*>(crsf::TInterfaceManager::GetInstance()->GetInputInterface("Hand_MoCAP"));

        // set hand mocap mode
        std::string mode = interface_hand_mocap_->GetMoCAPMode();
        if (mode == "both")
        {
            hand_mocap_mode_ = HAND_MOCAP_MODE_BOTH;
        }
        else if (mode == "left")
        {
            hand_mocap_mode_ = HAND_MOCAP_MODE_LEFT;
        }
        else if (mode == "right")
        {
            hand_mocap_mode_ = HAND_MOCAP_MODE_RIGHT;
            swap_trackers();
        }
    }



	// init UNIST mocap setting
	if (props_.get("subsystem.unistmocap", false))
	{
		interface_unist_mocap_ = dynamic_cast<Kinesthetic_HandMoCAPInterface*>(crsf::TInterfaceManager::GetInstance()->GetInputInterface("KinestheticHandMoCAP"));

		if (interface_unist_mocap_)
		{
			// version
			if (interface_unist_mocap_->GetVersion() == "old")
				unist_mocap_joint_number_ = 26;
			else if (interface_unist_mocap_->GetVersion() == "new")
				unist_mocap_joint_number_ = 28;

			// mode
			if (props_.get("subsystem.unistmocap_mode", "") == "left")
				unist_mocap_mode_ = "left";
			else if (props_.get("subsystem.unistmocap_mode", "") == "right")
				unist_mocap_mode_ = "right";
		}
	}



	// set DSM for hand
	if (props_.get("subsystem.leap", false))
	{
        // render my hand motion from leap module's amo
        if (app_.dsm_->HasMemoryObject<crsf::TAvatarMemoryObject>("Hands"))
        {
            hand->set_render_method(app_.dsm_->GetAvatarMemoryObjectByName("Hands"), render_hand_leap_local);
        }
        else
        {
            app_.m_logger->error("Failed to get AvatarMemoryObject of leap motion.");
        }
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