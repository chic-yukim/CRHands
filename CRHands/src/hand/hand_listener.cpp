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

#include <crsf/CRModel/TGroupedObjects.h>
#include <crsf/CRModel/TGroupedObjectsBase.h>


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
	auto my_physics_interactor = hand_->FindPhysicsInteractor(my_model);
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

		auto lin_vel = physics_manager->GetLinearVelocity(my_model.get());
		auto cur_lin_vel = lin_vel / 100.0;
		physics_manager->SetLinearVelocity(my_model.get(), cur_lin_vel);

		auto ang_vel = physics_manager->GetAngularVelocity(my_model.get());
		auto cur_ang_vel = ang_vel / 100.0;
		physics_manager->SetAngularVelocity(my_model.get(), cur_ang_vel);

		physics_manager->SetDamping(my_model.get(), 100.0, 100.0);
		physics_manager->ApplyImpulse(my_model.get(), LVecBase3(0), LVecBase3(0));
	}

	return false;
}

bool HandManager::object_separation_event(const std::shared_ptr<crsf::TCRModel>& my_model, const std::shared_ptr<crsf::TCRModel>& evented_model)
{
	// if my model is separated from physics interactor,
	// set damping to initial state (damping have similar effect to air resistence)
	crsf::TPhysicsManager::GetInstance()->SetDamping(my_model.get(), 0, 0);
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


bool HandManager::grouped_object_update_event_each(const std::shared_ptr<crsf::TCRModel>& my_model)
{
	my_model->contacted_hand_pointer.clear();
	my_model->is_contacted = false;
	for (auto & contacted_model : my_model->GetPhysicsModel()->GetContactInfo()->GetContactedModel())
	{
		auto interactor = std::dynamic_pointer_cast<crsf::THandPhysicsInteractor>(contacted_model);
		if (!interactor)
			continue;

		auto interactor_joint_tag = interactor->GetConnectedJointTag();
		crsf::TWorldObject* parent_hand_model = nullptr;
		if (interactor_joint_tag >= 0 && interactor_joint_tag <= 21)
			parent_hand_model = interactor->GetParentHandModel()->Get3DModel_LeftWrist();
		else if (interactor_joint_tag >= 22 && interactor_joint_tag <= 43)
			parent_hand_model = interactor->GetParentHandModel()->Get3DModel_RightWrist();
		my_model->contacted_hand_pointer.push_back(parent_hand_model);

		for (int i = 0; i < my_model->contacted_hand_pointer.size() - 1; i++)
		{
			if (my_model->contacted_hand_pointer[i] == parent_hand_model)
			{
				my_model->contacted_hand_pointer.pop_back();
			}
		}

		my_model->is_contacted = true;
		my_model->contacted_physics_particle.push_back(interactor.get());
	}
	return false;
}

bool HandManager::grouped_object_update_event(const std::shared_ptr<crsf::TCRModel>& my_model)
{
	auto world = crsf::TGraphicRenderEngine::GetInstance()->GetWorld();

	auto grouped_object_base = std::dynamic_pointer_cast<crsf::TGroupedObjectsBase>(my_model);

	if (!grouped_object_base)
	{
		global_logger->error("grouped_object_update_event should define on TGroupedObjectsBase.");
		return false;
	}

	// first, check each object's contacted state
	std::vector<shared_ptr<crsf::TCRModel>> contacted_children;

	for (int i = 0; i < grouped_object_base->GetChildren().size(); i++)
	{
		auto sub_group = std::dynamic_pointer_cast<crsf::TGroupedObjects>(grouped_object_base->GetChild(i)->shared_from_this());

		if (sub_group)
		{
			for (int j = 0; j < sub_group->GetChildren().size(); j++)
			{
				auto child_model = std::dynamic_pointer_cast<crsf::TCRModel>(sub_group->GetChild(j)->shared_from_this());

				if (!child_model)
					continue;

				if (child_model->is_contacted)
					contacted_children.push_back(child_model);
			}
		}
	}

	// distinguish Contacted Hand
	int number_of_hand = 2;
	if (grouped_object_base->is_multi_user_connect)
		number_of_hand = 4;

	std::vector<shared_ptr<crsf::TCRModel>> *contacted_hand = new std::vector<shared_ptr<crsf::TCRModel>>[number_of_hand];

	for (int i = 0; i < contacted_children.size(); i++)
	{
		for (int j = 0; j < contacted_children[i]->contacted_hand_pointer.size(); j++)
		{
			if (hand_pointer_[0] == contacted_children[i]->contacted_hand_pointer[j]) // right hand
			{
				contacted_hand[0].push_back(contacted_children[i]);
			}

			if (hand_pointer_[1] == contacted_children[i]->contacted_hand_pointer[j]) // left hand
			{
				contacted_hand[1].push_back(contacted_children[i]);
			}

			if (grouped_object_base->is_multi_user_connect)
			{
				// TBA, multi-user is not considered yet.
			}
		}
	}

	// determine grasping
	std::map<crsf::TWorldObject*, bool> hand_grasped;

	if (grouped_object_base->is_multi_mesh_group)
	{
		// TBA, multi-mesh group is not considered yet.
	}
	else
	{
		for (int n = 0; n < number_of_hand; n++)
		{
			hand_grasped[hand_pointer_[n]] = false;

			for (int i = 0; i < contacted_hand[n].size(); i++)
			{
				for (int j = 0; j < contacted_hand[n][i]->contacted_physics_particle.size(); j++)
				{
					for (int k = j; k < contacted_hand[n][i]->contacted_physics_particle.size(); k++)
					{
						auto object_1_contact_hand = contacted_hand[n][i]->contacted_physics_particle[j];
						auto object_2_contact_hand = contacted_hand[n][i]->contacted_physics_particle[k];

						LVecBase3 direction_1 = object_1_contact_hand->GetPenetrationDirection();
						LVecBase3 direction_2 = object_2_contact_hand->GetPenetrationDirection();

						float cos_angle = direction_1.dot(direction_2);

						if (cos_angle < -0.7)
						{
							if (!grouped_object_base->primary_grasped_hand_pointer)
							{
								grouped_object_base->primary_grasped_hand_pointer = hand_pointer_[n];
								grouped_object_base->primary_grasped_hand_number = n;
							}
							else
							{
								if (grouped_object_base->primary_grasped_hand_pointer != hand_pointer_[n])
								{
									grouped_object_base->secondary_grasped_hand_pointer = hand_pointer_[n];
									grouped_object_base->secondary_grasped_hand_number = n;
								}
							}

							hand_grasped[hand_pointer_[n]] = true;
							goto out1;
						}
					}
				}
			}
		out1:;
		}
	}

	// counting grasped hand
	int grasped_count = 0;
	for (int i = 0; i < number_of_hand; i++)
	{
		if (hand_grasped[hand_pointer_[i]])
		{
			grasped_count++;
		}
	}

	// two hand grasped case
	if (grasped_count >= 2)
	{
		if (grouped_object_base->release_object)
		{
			if (grouped_object_base->is_group_changeable) // grouping
			{
				//grouped_object_base->grouped_objects_grouping(contacted_hand);
			}

			if (grouped_object_base->grouped_object_data[0].axis_on_object != LVecBase3(-1) ||
				grouped_object_base->grouped_object_data[1].axis_on_object != LVecBase3(-1))
			{
				grouped_object_base->release_object = false;
			}
		}

		if (grouped_object_base->grouped_object_data[0].axis_on_object != LVecBase3(-1) ||
			grouped_object_base->grouped_object_data[1].axis_on_object != LVecBase3(-1))
		{
			int primary_hand_number = grouped_object_base->primary_grasped_hand_number;
			int secondary_hand_number = grouped_object_base->secondary_grasped_hand_number;
			grouped_object_base->grasped_hand[0] = (crsf::TWorldObject*)grouped_object_base->primary_grasped_hand_pointer;
			grouped_object_base->grasped_hand[1] = (crsf::TWorldObject*)grouped_object_base->secondary_grasped_hand_pointer;

			int sub_group_0 = 0, sub_group_1 = 0;
			for (int i = 0; i < contacted_hand[primary_hand_number].size(); i++)
			{
				if (grouped_object_base->sub_group[0] == std::dynamic_pointer_cast<crsf::TGroupedObjects>(contacted_hand[primary_hand_number][i]->GetParent()->shared_from_this()))
				{
					sub_group_0++;
				}
				else if (grouped_object_base->sub_group[1] == std::dynamic_pointer_cast<crsf::TGroupedObjects>(contacted_hand[primary_hand_number][i]->GetParent()->shared_from_this()))
				{
					sub_group_1++;
				}
			}

			if (sub_group_0 > sub_group_1)
			{
				grouped_object_base->grasped_object[0] = grouped_object_base->sub_group[0];
				grouped_object_base->grasped_object[1] = grouped_object_base->sub_group[1];

				grouped_object_base->grasped_object_data[0] = grouped_object_base->grouped_object_data[0];
				grouped_object_base->grasped_object_data[1] = grouped_object_base->grouped_object_data[1];
			}
			else if (sub_group_0 < sub_group_1)
			{
				grouped_object_base->grasped_object[0] = grouped_object_base->sub_group[1];
				grouped_object_base->grasped_object[1] = grouped_object_base->sub_group[0];

				grouped_object_base->grasped_object_data[0] = grouped_object_base->grouped_object_data[1];
				grouped_object_base->grasped_object_data[1] = grouped_object_base->grouped_object_data[0];
			}
			else if (sub_group_0 == sub_group_1)
			{
				unsigned long long number_of_sub_group_contacted_mesh[2] = { 0, };

				for (int i = 0; i < 2; i++)
				{
					for (int j = 0; j < grouped_object_base->sub_group[i]->GetChildren().size(); j++)
					{
						auto model = std::dynamic_pointer_cast<crsf::TCRModel>(grouped_object_base->sub_group[i]->GetChild(i)->shared_from_this());
						for (int k = 0; k < model->contacted_physics_particle.size(); k++)
						{
							auto interactor = model->contacted_physics_particle[k];
							auto interactor_joint_tag = interactor->GetConnectedJointTag();
							crsf::TWorldObject* parent_hand_model = nullptr;
							if (interactor_joint_tag >= 0 && interactor_joint_tag <= 21)
								parent_hand_model = interactor->GetParentHandModel()->Get3DModel_LeftWrist();
							else if (interactor_joint_tag >= 22 && interactor_joint_tag <= 43)
								parent_hand_model = interactor->GetParentHandModel()->Get3DModel_RightWrist();

							if (parent_hand_model == (crsf::TWorldObject*)grouped_object_base->primary_grasped_hand_pointer)
							{
								number_of_sub_group_contacted_mesh[i] += model->contacted_physics_particle.size();
							}
						}
					}
				}

				if (number_of_sub_group_contacted_mesh[0] > number_of_sub_group_contacted_mesh[1])
				{
					grouped_object_base->grasped_object[0] = grouped_object_base->sub_group[0];
					grouped_object_base->grasped_object[1] = grouped_object_base->sub_group[1];

					grouped_object_base->grasped_object_data[0] = grouped_object_base->grouped_object_data[0];
					grouped_object_base->grasped_object_data[1] = grouped_object_base->grouped_object_data[1];
				}
				else if (number_of_sub_group_contacted_mesh[0] < number_of_sub_group_contacted_mesh[1])
				{
					grouped_object_base->grasped_object[0] = grouped_object_base->sub_group[1];
					grouped_object_base->grasped_object[1] = grouped_object_base->sub_group[0];

					grouped_object_base->grasped_object_data[0] = grouped_object_base->grouped_object_data[1];
					grouped_object_base->grasped_object_data[1] = grouped_object_base->grouped_object_data[0];
				}
			}

			if (grouped_object_base->manipulated_object != grouped_object_base->grasped_object[1])
			{
				auto primary_hand = (crsf::TWorldObject*)grouped_object_base->primary_grasped_hand_pointer;
				LMatrix4f world_to_hand = primary_hand->GetMatrix(world);
				LMatrix4f world_to_group = grouped_object_base->GetMatrix(world);

				LMatrix4f transform_coordinate = LMatrix4f::ident_mat();
				transform_coordinate.set_row(3, grouped_object_base->grasped_object_data[0].pivot_on_object);
				LVecBase3 pivot_translation = grouped_object_base->grasped_object_data[0].pivot_on_object_in_group
					- grouped_object_base->grasped_object_data[0].pivot_on_object;
				LMatrix4f pivot_transform = LMatrix4f::ident_mat();
				pivot_transform.set_row(3, pivot_translation);
				LMatrix4f sub_group_to_sub_group_relative_transform = grouped_object_base->grasped_object[0]->GetMatrix();
				sub_group_to_sub_group_relative_transform = sub_group_to_sub_group_relative_transform * invert(pivot_transform);

				LMatrix4f hand_to_group = sub_group_to_sub_group_relative_transform * world_to_group * invert(world_to_hand);

				grouped_object_base->SetMatrix(hand_to_group);

				grouped_object_base->grasped_object[1]->SetMatrix(invert(grouped_object_base->grasped_object[0]->GetMatrix()));
				grouped_object_base->grasped_object[0]->SetMatrix(LMatrix4f::ident_mat());
			}

			grouped_object_base->manipulated_object = grouped_object_base->grasped_object[1];

			if (grouped_object_base->first_time)
			{
				for (int i = 0; i < 2; i++)
				{
					LMatrix4f world_to_hand = grouped_object_base->grasped_hand[i]->GetMatrix(world);
					LMatrix4f world_to_sub_group = grouped_object_base->grasped_object[i]->GetMatrix(world);

					grouped_object_base->fixed_relative_transform[i] = world_to_sub_group;
					grouped_object_base->fixed_relative_transform[i] = grouped_object_base->fixed_relative_transform[i] * invert(world_to_hand);
				}

				grouped_object_base->first_time = false;
			}

			for (int i = 0; i < 2; i++)
			{
				LMatrix4f world_to_hand = grouped_object_base->grasped_hand[i]->GetMatrix(world);

				grouped_object_base->current_hand_global_pose[i] = grouped_object_base->fixed_relative_transform[i];
				grouped_object_base->current_hand_global_pose[i] = grouped_object_base->current_hand_global_pose[i] * world_to_hand;
			}

			// TBA, 
			// TODO : constrained object control algorithm

		}
	}
	else
	{
		grouped_object_base->first_time = true;
	}

	if (grasped_count >= 1)
	{
		crsf::TWorldObject* primary_hand;
		if (grasped_count == 1)
		{
			for (int i = 0; i < number_of_hand; i++)
			{
				if (hand_grasped[hand_pointer_[i]])
				{
					primary_hand = hand_pointer_[i];
					grouped_object_base->primary_grasped_hand_pointer = primary_hand;
					grouped_object_base->primary_grasped_hand_number = i;

					break;
				}
			}
		}
		else
		{
			primary_hand = (crsf::TWorldObject*)grouped_object_base->primary_grasped_hand_pointer;
		}

		LMatrix4f world_to_hand = primary_hand->GetMatrix(world);
		LMatrix4f world_to_cube = grouped_object_base->GetMatrix(world);
		LMatrix4f hand_to_cube = world_to_cube * invert(world_to_hand);

		primary_hand->AddWorldObject(grouped_object_base);
		grouped_object_base->SetMatrix(hand_to_cube);
	}
	else
	{
		world->AddWorldObject(grouped_object_base);

		auto right_hand = hand_pointer_[0];
		auto left_hand = hand_pointer_[1];

		if (grouped_object_base->is_multi_user_connect)
		{
			// TBA, multi-user is not considered yet.
		}

		grouped_object_base->primary_grasped_hand_pointer = nullptr;
		grouped_object_base->secondary_grasped_hand_pointer = nullptr;

		grouped_object_base->release_object = true;
	}

	delete[] contacted_hand;

	return false;
}
