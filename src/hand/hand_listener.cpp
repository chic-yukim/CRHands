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
	auto world = crsf::TGraphicRenderEngine::GetInstance()->GetWorld();

	auto my_model_contact_info = my_model->GetPhysicsModel()->GetContactInfo();
	auto contacted_models = my_model_contact_info->GetContactedModel();

	std::vector<std::shared_ptr<crsf::TCRModel>> pContacted;
	bool is_contacted[6] = { 0 }; // 6 vibration bit - left 3 + right 3
	for (auto& contacted_model : contacted_models)
	{
		if (contacted_model->GetModelGroup() == crsf::EMODEL_GROUP_HANDPHYSICSINTERACTOR)
		{
			pContacted.push_back(contacted_model);
			auto intr = hand_->FindPhysicsInteractor(contacted_model);
			if (!intr)
				return false;
			auto tag = intr->GetConnectedJointTag();

			// vibration bit masking
			switch (tag)
			{
			case crsf::LEFT__MIDDLE_4:
				is_contacted[0] = true;
				break;
			case crsf::LEFT__INDEX_4:
				is_contacted[1] = true;
				break;
			case crsf::LEFT__THUMB_4:
				is_contacted[2] = true;
				break;
			case crsf::RIGHT__MIDDLE_4:
				is_contacted[3] = true;
				break;
			case crsf::RIGHT__INDEX_4:
				is_contacted[4] = true;
				break;
			case crsf::RIGHT__THUMB_4:
				is_contacted[5] = true;
				break;
			}
		}
	}

	if (pContacted.empty())
	{
		// stop vibration feedback
		if (interface_hand_mocap_)
		{
			interface_hand_mocap_->StopVibration(0);
			interface_hand_mocap_->StopVibration(1);
		}

		return false;
	}

	// do vibration feedback
	if (interface_hand_mocap_)
	{
		unsigned int left_bit = 0;
		unsigned int right_bit = 0;

		for (int i = 0; i < 2; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				if (i == 0)
				{
					left_bit += (is_contacted[i * 3 + j] ? 1 : 0) * pow(2, j);
				}
				else
				{
					right_bit += (is_contacted[i * 3 + j] ? 1 : 0) * pow(2, j);
				}
			}
		}

		std::vector<unsigned char> left_data = { 0x40, 0x01, 0x05, (unsigned char)left_bit, 0x05, 0x05, 0x4B, 0xA5 };
		std::vector<unsigned char> right_data = { 0x40, 0x01, 0x05, (unsigned char)right_bit, 0x05, 0x05, 0x4B, 0xA5 };

		interface_hand_mocap_->MakeVibration(0, left_data.data());
		interface_hand_mocap_->MakeVibration(1, right_data.data());
	}

	return false;
}