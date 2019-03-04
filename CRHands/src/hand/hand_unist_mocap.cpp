/**
* Coexistence Reality Software Framework (CRSF)
* Copyright (c) Center of Human-centered Interaction for Coexistence. All rights reserved.
* See the LICENSE.md file for more details.
*/

#include "hand_manager.hpp"

#include <render_pipeline/rppanda/showbase/showbase.hpp>
#include <render_pipeline/rpcore/globals.hpp>

#include <crsf/CoexistenceInterface/TDynamicStageMemory.h>

#include <crsf/RenderingEngine/TGraphicRenderEngine.h>
#include <crsf/CRModel/TWorld.h>
#include <crsf/CRModel/TWorldObject.h>
#include <crsf/CRModel/TCharacter.h>

#include <crsf/CRModel/TCRHand.h>

#include <crsf/CoexistenceInterface/TAvatarMemoryObject.h>
#include <crsf/System/TPose.h>

#include <openvr_module.h>

#if _MSC_VER > 1900
#include <rpplugins/openvr/plugin.hpp>
#else
#include <openvr_plugin.hpp>
#endif

void HandManager::render_unist_mocap(crsf::TAvatarMemoryObject *amo)
{
	crsf::TWorld* virtual_world = crsf::TGraphicRenderEngine::GetInstance()->GetWorld();

	//global_logger->info("hand mocap rendering process is running...");

	//////////////////////////////////////////////////////////////////////////
	// set right wrist pose = tracker pose
	if (module_open_vr_)
	{
		auto tracker_pos = module_open_vr_->GetDevicePosition(tracker_index_right_);
		auto tracker_quat = module_open_vr_->GetDeviceOrientation(tracker_index_right_);

		LQuaternionf a;
		a.set_from_axis_angle(90, LVecBase3(0, 1, 0));
		tracker_quat = a * tracker_quat;

		hand_->GetJointData(crsf::LEFT__WRIST)->Get3DModel()->SetPosition(LVecBase3(100), virtual_world);
		hand_->GetJointData(crsf::LEFT__WRIST)->SetPosition(LVecBase3(100));

		hand_->GetJointData(crsf::RIGHT__WRIST)->Get3DModel()->SetPosition(tracker_pos, virtual_world);
		hand_->GetJointData(crsf::RIGHT__WRIST)->SetPosition(tracker_pos);
		hand_->GetJointData(crsf::RIGHT__WRIST)->Get3DModel()->SetHPR(tracker_quat.get_hpr(), virtual_world);
		LQuaternionf d, e, ed;
		d.set_from_axis_angle(-90, LVecBase3(0, 0, 1));
		e.set_from_axis_angle(-90, LVecBase3(1, 0, 0));
		ed = e * d;
		tracker_quat = ed * tracker_quat;
		hand_->GetJointData(crsf::RIGHT__WRIST)->SetOrientation(tracker_quat);
	}

	//////////////////////////////////////////////////////////////////////////
	// render hand by mocap pose data
	if (hand_)
	{
		// # joint
		int nJointNumber = 28;

		// read data from AvatarMemory 
		for (int i = 0; i < nJointNumber; i++)
		{
			hand_mocap_data_[i] = amo->GetAvatarMemory().at(i).GetPosition()[0];
		}

		// Thumb, index, middle = 3
		for (int i = 0; i < 3; i++)
		{
			int start_index = 2 + i * 5;

			// 1st joint
			{
				LQuaternionf quat_result;

				// get sign of [aa]
				int sign;
				if (hand_mocap_data_[start_index] == 0)
					sign = 1;
				else if (hand_mocap_data_[start_index] == 2)
					sign = -1;

				// get [aa] rotation data
				float aa = sign * hand_mocap_data_[start_index + 1];
				// get [mcp] rotation data
				float mcp = hand_mocap_data_[start_index + 2];

				// calculate quaternion for hand model
				LQuaternionf a, b, ba;
				a.set_from_axis_angle(aa, LVecBase3(0, 0, 1));
				b.set_from_axis_angle(mcp, LVecBase3(0, -1, 0));
				ba = b * a;
				quat_result = ba;

				// thumb case
				if (i == 0)
				{
					// multiply quaternion for thumb rotation
					LQuaternionf c, d, e, edc;
					c.set_from_axis_angle(-1.406013982f * 180.0f / M_PI, LVecBase3(1, 0, 0));
					d.set_from_axis_angle(0.453798839f * 180.0f / M_PI, LVecBase3(0, 0, -1));
					e.set_from_axis_angle(0.905946589f * 180.0f / M_PI, LVecBase3(0, 1, 0));
					edc = e * d * c;
					LQuaternionf f;
					f.set_from_axis_angle(-45, LVecBase3(0, 1, 0));
					edc = f * edc;
					quat_result = quat_result * edc;
				}

				// set hpr
				LVecBase3 hpr = quat_result.get_hpr();
				int model_index = crsf::RIGHT__THUMB_2 + i * 4;
				hand_->GetJointData(model_index)->Get3DModel()->SetHPR(hpr);
			}

			// 2nd joint
			{
				LQuaternionf quat_result;

				// get [pip] data
				float pip = hand_mocap_data_[start_index + 3];

				// calculate quaternion for hand model
				LQuaternionf a;
				a.set_from_axis_angle(pip, LVecBase3(0, -1, 0));
				quat_result = a;

				// Set hpr
				LVecBase3 hpr = quat_result.get_hpr();
				int model_index = crsf::RIGHT__THUMB_3 + i * 4;
				hand_->GetJointData(model_index)->Get3DModel()->SetHPR(hpr);
			}

			// 3rd joint
			{
				LQuaternionf quat_result;

				// get [dip] data
				float dip = hand_mocap_data_[start_index + 4];

				// calculate quaternion for hand model
				LQuaternionf a;
				a.set_from_axis_angle(dip, LVecBase3(0, -1, 0));
				quat_result = a;

				// set hpr
				LVecBase3 hpr = quat_result.get_hpr();
				int model_index = crsf::RIGHT__THUMB_4 + i * 4;
				hand_->GetJointData(model_index)->Get3DModel()->SetHPR(hpr);
			}
		}
	}
}