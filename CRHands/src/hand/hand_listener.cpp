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

bool HandManager::interactor_collision_event(const std::shared_ptr<crsf::TCRModel>& my_model, const std::shared_ptr<crsf::TCRModel>& evented_model)
{
	// if contacted model is not interactable, return
	if (evented_model->GetPhysicsModel()->GetIsHandInteractable() == false || !hand_)
	{
		return false;
	}

	// get contact information
	crsf::TContactInfo* my_contact_info = my_model->GetPhysicsModel()->GetContactInfo();
	LVecBase3 contact_position = my_contact_info->GetPositionWorldOnA();
	LVecBase3 direction = my_contact_info->GetNormalWorldOnB();

	// set penetration direction on physics interactor
	crsf::THandPhysicsInteractor* my_physics_interactor = hand_->FindPhysicsInteractor(my_model);
	my_physics_interactor->SetPenetrationDirection(direction);
	my_physics_interactor->SetPenetrationDirection(my_physics_interactor->GetPenetrationDirection().normalized());

	return false;
}

bool HandManager::object_collision_event(const std::shared_ptr<crsf::TCRModel>& my_model, const std::shared_ptr<crsf::TCRModel>& evented_model)
{
	// if my model collided with physics interactor,
	// set physics parameter to relaxing state
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
	// if my model is separated from physics interactor,
	// set damping to initial state (damping have similar effect to air resistence)
	crsf::TPhysicsManager::GetInstance()->SetDamping(my_model, 0, 0);
	return false;
}

bool HandManager::object_update_event(const std::shared_ptr<crsf::TCRModel>& my_model)
{
	if (!hand_)
		return true;

	crsf::TWorld* world = crsf::TGraphicRenderEngine::GetInstance()->GetWorld();

	// get contactinformation
	auto my_physics_model = my_model->GetPhysicsModel();
	auto my_model_contact_info = my_model->GetPhysicsModel()->GetContactInfo();
	const auto& contacted_models = my_model_contact_info->GetContactedModel();

	// init grasp state
	bool old_grasp_state = my_physics_model->GetIsGrasped();
	my_physics_model->SetIsGrasped(false);

	// current contacted physics particle
	std::vector<std::shared_ptr<crsf::TCRModel>> current_contacted_physics_interactor;
	current_contacted_physics_interactor.clear();

	// hand mocap vibration bit mask
    Hand_MoCAPInterface::FingerMask vibration_mask[2] = { Hand_MoCAPInterface::FingerMask::FINGER_NONE, Hand_MoCAPInterface::FingerMask::FINGER_NONE };
	
	// traverse contacted model
    for (const auto& contacted_model : contacted_models)
    {
        if (contacted_model->GetModelGroup() == crsf::EMODEL_GROUP_HANDPHYSICSINTERACTOR)
        {
			// set & get physics interactor information
			current_contacted_physics_interactor.push_back(contacted_model);
            auto intr = hand_->FindPhysicsInteractor(contacted_model);
            if (!intr)
                return false;
            auto tag = intr->GetConnectedJointTag();
			intr->SetIsTouched(true);

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

	// no physics particle collision - return
	if (current_contacted_physics_interactor.empty())
	{
		my_physics_model->SetPhysicsType(crsf::EPHYX_TYPE_RIGIDBODY_ACTIVE);
		hand_->SetIsTouched(false);
		return false;
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

	// test grasp kinematic feasibility
	LMatrix4f hand_to_world;
	int contacted_interactor_size = current_contacted_physics_interactor.size();
	crsf::THandPhysicsInteractor* intr1;
	crsf::THandPhysicsInteractor* intr2;
	LVecBase3 dir1, dir2;
	int joint1, joint2;
	for (int i = 0; i < contacted_interactor_size; i++)
	{
		intr1 = hand_->FindPhysicsInteractor(current_contacted_physics_interactor[i]);
		if (!intr1)
			return false;
		dir1 = intr1->GetPenetrationDirection();
		joint1 = intr1->GetConnectedJointTag();

		for (int j = i; j < contacted_interactor_size; j++)
		{
			intr2 = hand_->FindPhysicsInteractor(current_contacted_physics_interactor[j]);
			if (!intr2)
				return false;
			dir2 = intr2->GetPenetrationDirection();
			joint2 = intr2->GetConnectedJointTag();

			float cosangle = dir1.dot(dir2);
			if (cosangle < -0.7
				&& joint1 >= 0 && joint1 <= 21 && joint2 >= 0 && joint2 <= 21)
			{
				my_physics_model->SetIsGrasped(true);
				hand_to_world = hand_->Get3DModel_LeftWrist()->GetMatrix(world);

				break;
			}
			if (cosangle < -0.7
				&& joint1 >= 22 && joint1 <= 43 && joint2 >= 22 && joint2 <= 43)
			{
				my_physics_model->SetIsGrasped(true);
				hand_to_world = hand_->Get3DModel_RightWrist()->GetMatrix(world);

				break;
			}
		}
	}

	// init relative transform hand <-> object
	if (!old_grasp_state && my_physics_model->GetIsGrasped())
	{
		LVecBase3f scale;
		LVecBase3f shear;
		LVecBase3f hpr;
		LVecBase3f translate;
		decompose_matrix(hand_to_world, scale, shear, hpr, translate);

		LQuaternionf quat;
		quat.set_hpr(hpr);

		LMatrix4f rot_mat;
		quat.extract_to_matrix(rot_mat);
		rot_mat.set_row(3, translate);

		my_model->SetFixedRelativeTransform(my_model->GetMatrix(world) * invert(rot_mat));
	}
	else if (old_grasp_state && !my_physics_model->GetIsGrasped())
	{
		my_physics_model->SetPhysicsType(crsf::EPHYX_TYPE_RIGIDBODY_ACTIVE);
		hand_->SetIsTouched(false);
	}

	// set object transform to grasp state
	if (my_physics_model->GetIsGrasped())
	{
		//my_physics_model->SetPhysicsType(crsf::EPHYX_TYPE_RIGIDBODY_PASSIVE);

		LMatrix4f fixed_pose = my_model->GetFixedRelativeTransform();

		LVecBase3f scale;
		LVecBase3f shear;
		LVecBase3f hpr;
		LVecBase3f translate;
		decompose_matrix(hand_to_world, scale, shear, hpr, translate);

		LQuaternionf quat;
		quat.set_hpr(hpr);

		LMatrix4f rot_mat;
		quat.extract_to_matrix(rot_mat);
		rot_mat.set_row(3, translate);

		auto new_mat = fixed_pose * rot_mat;

		my_model->SetMatrix(new_mat, world, crsf::EMODEL_SETMODE_ONLY_PHYSICS);
	}
	else
	{
		my_physics_model->SetPhysicsType(crsf::EPHYX_TYPE_RIGIDBODY_ACTIVE);
		hand_->SetIsTouched(false);
	}

	return false;
}
