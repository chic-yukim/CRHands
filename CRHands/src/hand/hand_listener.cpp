/**
* Coexistence Reality Software Framework (CRSF)
* Copyright (c) Center of Human-centered Interaction for Coexistence. All rights reserved.
* See the LICENSE.md file for more details.
*/

#include "hand_manager.hpp"

#include <render_pipeline/rpcore/util/rpmaterial.hpp>

#include <crsf/RenderingEngine/TGraphicRenderEngine.h>
#include <crsf/CRModel/TWorld.h>

#include <crsf/CREngine/TPhysicsManager.h>

#include <crsf/CRModel/TCRModel.h>
#include <crsf/CRModel/TPhysicsModel.h>
#include <crsf/CRModel/TSphere.h>

#include <crsf/CRModel/THandPhysicsInteractor.h>

#include <spdlog/spdlog.h>

#include <hand_mocap_module.h>
#include <hand_mocap_interface.h>

#include "main.hpp"

extern spdlog::logger* global_logger;

bool HandManager::object_collision_event(const std::shared_ptr<crsf::TCRModel>& my_model, const std::shared_ptr<crsf::TCRModel>& evented_model)
{
	if (evented_model->GetModelGroup() == crsf::EMODEL_GROUP_HANDPHYSICSINTERACTOR)
	{
		auto world = crsf::TGraphicRenderEngine::GetInstance()->GetWorld();

		auto physics_manager = crsf::TPhysicsManager::GetInstance();

		auto lin_vel = physics_manager->GetLinearVelocity(my_model);
		auto cur_lin_vel = lin_vel / 100.0;
		physics_manager->SetLinearVelocity(my_model, cur_lin_vel);

		auto ang_vel = physics_manager->GetAngularVelocity(my_model);
		auto cur_ang_vel = ang_vel / 100.0;
		physics_manager->SetAngularVelocity(my_model, cur_ang_vel);

		physics_manager->SetDamping(my_model, 100.0, 100.0);
		physics_manager->ApplyImpulse(my_model, LVecBase3(0), LVecBase3(0));
	}

	return false;
}

bool HandManager::object_separation_event(const std::shared_ptr<crsf::TCRModel>& my_model, const std::shared_ptr<crsf::TCRModel>& evented_model)
{
	crsf::TPhysicsManager::GetInstance()->SetDamping(my_model, 0, 0);
	return false;
}

bool HandManager::object_update_event(const std::shared_ptr<crsf::TCRModel>& my_model)
{
    auto my_model_contact_info = my_model->GetPhysicsModel()->GetContactInfo();
    const auto& contacted_models = my_model_contact_info->GetContactedModel();

    Hand_MoCAPInterface::FingerMask vibration_mask[2] = { Hand_MoCAPInterface::FingerMask::FINGER_NONE, Hand_MoCAPInterface::FingerMask::FINGER_NONE };

    for (const auto& contacted_model : contacted_models)
    {
        if (contacted_model->GetModelGroup() == crsf::EMODEL_GROUP_HANDPHYSICSINTERACTOR)
        {
            auto intr = hand_->FindPhysicsInteractor(contacted_model);
            if (!intr)
                return false;
            auto tag = intr->GetConnectedJointTag();

            // vibration bit masking
            switch (tag)
            {
            case crsf::LEFT__MIDDLE_4:
                vibration_mask[Hand_MoCAPInterface::HAND_LEFT] |= Hand_MoCAPInterface::FingerMask::FINGER_MIDDLE;
                break;
            case crsf::LEFT__INDEX_4:
                vibration_mask[Hand_MoCAPInterface::HAND_LEFT] |= Hand_MoCAPInterface::FingerMask::FINGER_INDEX;
                break;
            case crsf::LEFT__THUMB_4:
                vibration_mask[Hand_MoCAPInterface::HAND_LEFT] |= Hand_MoCAPInterface::FingerMask::FINGER_THUMB;
                break;
            case crsf::RIGHT__MIDDLE_4:
                vibration_mask[Hand_MoCAPInterface::HAND_RIGHT] |= Hand_MoCAPInterface::FingerMask::FINGER_MIDDLE;
                break;
            case crsf::RIGHT__INDEX_4:
                vibration_mask[Hand_MoCAPInterface::HAND_RIGHT] |= Hand_MoCAPInterface::FingerMask::FINGER_INDEX;
                break;
            case crsf::RIGHT__THUMB_4:
                vibration_mask[Hand_MoCAPInterface::HAND_RIGHT] |= Hand_MoCAPInterface::FingerMask::FINGER_THUMB;
                break;
            }
        }
    }

    if (interface_hand_mocap_)
    {
        for (auto hand_index : { Hand_MoCAPInterface::HAND_LEFT, Hand_MoCAPInterface::HAND_RIGHT })
        {
            if (vibration_mask[hand_index] != last_hand_mocap_vibrations_[hand_index])
            {
                interface_hand_mocap_->SetVibration(hand_index, vibration_mask[hand_index]);
                last_hand_mocap_vibrations_[hand_index] = vibration_mask[hand_index];
            }
        }
    }

    return false;
}
