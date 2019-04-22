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
}

void HandManager::setup_hand(void)
{
    auto interface_manager = crsf::TInterfaceManager::GetInstance();

	// init CHIC mocap setting
    // get hand mocap module interface instance
    interface_hand_mocap_ = dynamic_cast<Hand_MoCAPInterface*>(interface_manager->GetInputInterface("Hand_MoCAP"));
    if (interface_hand_mocap_)
    {
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
        interface_unist_mocap_ = dynamic_cast<Kinesthetic_HandMoCAPInterface*>(interface_manager->GetInputInterface("KinestheticHandMoCAP"));

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
}

void HandManager::setup_hand(User* user)
{
    if (!user)
        return;

    if (user->get_hand())
        return;

    // create hand instance
    auto hand = user->make_hand();
    auto crhand = hand->get_hand();
    hand_ = crhand;

    // update hand property
    auto hand_prop = crhand->GetHandProperty();
    // VR mode - leap local translation is 'HMD-to-LEAP'
    if (crsf::TDynamicModuleManager::GetInstance()->IsModuleEnabled("openvr"))
    {
        hand_prop.m_vec3ZeroToSensor[0] = app_.m_property.get("hand.HMD_to_LEAP_x", 0.0f);
        hand_prop.m_vec3ZeroToSensor[1] = app_.m_property.get("hand.HMD_to_LEAP_y", 0.0f);
        hand_prop.m_vec3ZeroToSensor[2] = app_.m_property.get("hand.HMD_to_LEAP_z", 0.0f);
    }
    else // mono mode - leap local translation is 'zero-to-LEAP'
    {
        hand_prop.m_vec3ZeroToSensor[0] = app_.m_property.get("hand.zero_to_LEAP_x", 0.0f);
        hand_prop.m_vec3ZeroToSensor[1] = app_.m_property.get("hand.zero_to_LEAP_y", 0.0f);
        hand_prop.m_vec3ZeroToSensor[2] = app_.m_property.get("hand.zero_to_LEAP_z", 0.0f);
    }
    crhand->SetHandProperty(hand_prop);

    // create physics interactor
    if (app_.physics_manager_)
        hand->setup_physics_interactor(particle_radius_);

    // grasp algorithm
    hand_pointer_.push_back(hand_->Get3DModel_RightWrist());
    hand_pointer_.push_back(hand_->Get3DModel_LeftWrist());

    if (user->get_system_index() == app_.dsm_->GetSystemIndex())
    {
        if (app_.m_property.get("subsystem.leap", false))
        {
            if (app_.dsm_->HasMemoryObject<crsf::TAvatarMemoryObject>("Hands"))
            {
                hand->set_render_method(app_.dsm_->GetAvatarMemoryObjectByName("Hands"), render_hand_leap_local);
            }
            else
            {
                app_.m_logger->error("Failed to get AvatarMemoryObject of leap motion.");
            }
        }
        else if (app_.m_property.get("subsystem.handmocap", false))
        {
            if (app_.dsm_->HasMemoryObject<crsf::TAvatarMemoryObject>("MoCAPHands"))
            {
                hand->set_render_method(app_.dsm_->GetAvatarMemoryObjectByName("MoCAPHands"), [this](Hand* hand, crsf::TAvatarMemoryObject* amo) { render_hand_mocap(hand, amo); });
            }
            else
            {
                app_.m_logger->error("Failed to get AvatarMemoryObject of Hand MoCAP.");
            }
        }
        else if (app_.m_property.get("subsystem.unistmocap", false))
        {
            auto amo = crsf::TDynamicStageMemory::GetInstance()->GetAvatarMemoryObjectByName("KinestheticMoCAPHands");
            crsf::TPhysicsManager::GetInstance()->AddTask([this, amo](void) {
                render_unist_mocap(amo);
                return false;
            }, "render_unist_mocap");
        }
    }

    configure_hand(hand);
}

void HandManager::configure_hand(Hand* hand)
{
    auto crhand = hand->get_hand();
    crhand->SetPhysicsInteractor_KinematicObject();
    crhand->SetPhysicsInteractor_Mass(0.0f);
}
