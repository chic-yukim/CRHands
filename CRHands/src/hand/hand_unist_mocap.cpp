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

#include <kinesthethic_hand_mocap_interface.h>

#if _MSC_VER > 1900
#include <rpplugins/openvr/plugin.hpp>
#else
#include <openvr_plugin.hpp>
#endif

void HandManager::render_unist_mocap(crsf::TAvatarMemoryObject *amo)
{
	crsf::TWorld* virtual_world = crsf::TGraphicRenderEngine::GetInstance()->GetWorld();

    int tracker_index[HAND_INDEX_COUNT];

    tracker_index[HAND_INDEX_LEFT] = tracker_indices_[HAND_INDEX_LEFT];
    tracker_index[HAND_INDEX_RIGHT] = tracker_indices_[HAND_INDEX_RIGHT];

	// set wrist pose using tracker pose
	if (module_open_vr_)
	{
		if (unist_mocap_mode_ == "left")
		{
			auto tracker_pos = module_open_vr_->GetDevicePosition(tracker_index[HAND_INDEX_LEFT]);
			auto tracker_quat = module_open_vr_->GetDeviceOrientation(tracker_index[HAND_INDEX_LEFT]);

			// if tracker is not activated,
			// find tracker
			if (tracker_pos == LVecBase3(0))
			{
                find_trackers();
			}

			LQuaternionf a;
			a.set_from_axis_angle(-270, LVecBase3(0, 1, 0));
			tracker_quat = a * tracker_quat;

			hand_->GetJointData(crsf::RIGHT__WRIST)->Get3DModel()->SetPosition(LVecBase3(100), virtual_world);
			hand_->GetJointData(crsf::RIGHT__WRIST)->SetPosition(LVecBase3(100));

			hand_->GetJointData(crsf::LEFT__WRIST)->Get3DModel()->SetPosition(tracker_pos, virtual_world);
			hand_->GetJointData(crsf::LEFT__WRIST)->SetPosition(tracker_pos);
			hand_->GetJointData(crsf::LEFT__WRIST)->Get3DModel()->SetHPR(tracker_quat.get_hpr(), virtual_world);
		}
		else if (unist_mocap_mode_ == "right")
		{
			auto tracker_pos = module_open_vr_->GetDevicePosition(tracker_index[HAND_INDEX_RIGHT]);
			auto tracker_quat = module_open_vr_->GetDeviceOrientation(tracker_index[HAND_INDEX_RIGHT]);

			// if tracker is not activated,
			// find tracker
			if (tracker_pos == LVecBase3(0))
			{
				find_trackers();
			}

			LQuaternionf a;
			a.set_from_axis_angle(90, LVecBase3(0, 1, 0));
			tracker_quat = a * tracker_quat;

			hand_->GetJointData(crsf::LEFT__WRIST)->Get3DModel()->SetPosition(LVecBase3(100), virtual_world);
			hand_->GetJointData(crsf::LEFT__WRIST)->SetPosition(LVecBase3(100));

			hand_->GetJointData(crsf::RIGHT__WRIST)->Get3DModel()->SetPosition(tracker_pos, virtual_world);
			hand_->GetJointData(crsf::RIGHT__WRIST)->SetPosition(tracker_pos);
			hand_->GetJointData(crsf::RIGHT__WRIST)->Get3DModel()->SetHPR(tracker_quat.get_hpr(), virtual_world);
		}
	}

	// render hand by mocap pose data
	if (hand_)
	{
		// read data from AvatarMemory 
		for (int i = 0; i < unist_mocap_joint_number_; i++)
		{
			hand_mocap_data_[i] = amo->GetAvatarMemory().at(i).GetPosition()[0];
		}

		// thumb, index, middle = 3 fingers
		for (int i = 0; i < 3; i++)
		{
			// reference protocol
			int start_index = 2 + i * 5;

			// 1st joint = proximal phalanges
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
				if (i == 0) // thumb
				{
					/*LQuaternionf a_, b_, c_;
					a_.set_from_axis_angle(-mcp * 0.3f, LVecBase3(1, 0, 0));
					b_.set_from_axis_angle(aa - 30.0f, LVecBase3(0, 1, 0));
					c_.set_from_axis_angle(mcp, LVecBase3(0, 0, 1));
					quat_result = c_ * a_ * b_;*/

					/*LQuaternionf a_, b_;
					a_.set_from_axis_angle(aa - 20.0f, LVecBase3(0, 1, 0));
					b_.set_from_axis_angle(mcp - 30.0f, LVecBase3(0, 0, 1));
					quat_result = b_ * a_;*/

					LQuaternionf a_, b_;
					a_.set_from_axis_angle(aa * 1.1f, LVecBase3(0, 1, 0));
					b_.set_from_axis_angle(mcp * 1.4f + 30.0f, LVecBase3(0, 0, 1));
					quat_result = b_ * a_;
				}
				else // index, middle
				{
					LQuaternionf a_, b_;
					a_.set_from_axis_angle(aa, LVecBase3(0, 1, 0));
					b_.set_from_axis_angle(mcp, LVecBase3(0, 0, 1));
					quat_result = b_ * a_;
				}
				// rotate unist hand -> crsf hand
				quat_result = rotate_unist_to_crsf(quat_result);

				// thumb case
				if (i == 0)
				{
					// multiply quaternion for thumb rotation
					if (unist_mocap_mode_ == "left")
					{
						LQuaternionf c, d, e, edc;
						c.set_from_axis_angle(1.806013982f * 180.0f / M_PI, LVecBase3(-1, 0, 0));
						d.set_from_axis_angle(0.453798839f * 180.0f / M_PI, LVecBase3(0, 0, -1));
						e.set_from_axis_angle(0.905946589f * 180.0f / M_PI, LVecBase3(0, 1, 0));
						edc = e * d * c;
						LQuaternionf f;
						f.set_from_axis_angle(-45, LVecBase3(0, 1, 0));
						edc = f * edc;
						quat_result = quat_result * edc;
					}
					else if (unist_mocap_mode_ == "right")
					{
						LQuaternionf c, d, e, edc;
						c.set_from_axis_angle(-1.806013982f * 180.0f / M_PI, LVecBase3(1, 0, 0));
						d.set_from_axis_angle(0.453798839f * 180.0f / M_PI, LVecBase3(0, 0, -1));
						e.set_from_axis_angle(0.905946589f * 180.0f / M_PI, LVecBase3(0, 1, 0));
						edc = e * d * c;
						LQuaternionf f;
						f.set_from_axis_angle(-45, LVecBase3(0, 1, 0));
						edc = f * edc;
						quat_result = quat_result * edc;
					}
				}

				// set hpr
				LVecBase3 hpr = quat_result.get_hpr();
				int model_index;
				if (unist_mocap_mode_ == "left")
					model_index = crsf::LEFT__THUMB_2 + i * 4;
				else if (unist_mocap_mode_ == "right")
					model_index = crsf::RIGHT__THUMB_2 + i * 4;
				hand_->GetJointData(model_index)->Get3DModel()->SetHPR(hpr);
			}

			// 2nd joint = intermediate phalanges
			{
				LQuaternionf quat_result;

				// get [pip] data
				float pip = hand_mocap_data_[start_index + 3];
				if (i == 0) // thumb
				{
					//pip *= 1.0f;
					pip *= 1.2f;
				}
				else if (i == 1) // index
				{
					pip *= 1.2f;
				}
				else if (i == 2) // middle
				{
					pip *= 1.2f;
				}

				// calculate quaternion for hand model
				quat_result.set_from_axis_angle(pip, LVecBase3(0, 0, 1));
				// rotate unist hand -> crsf hand
				quat_result = rotate_unist_to_crsf(quat_result);

				// set hpr
				LVecBase3 hpr = quat_result.get_hpr();
				int model_index;
				if (unist_mocap_mode_ == "left")
					model_index = crsf::LEFT__THUMB_3 + i * 4;
				else if (unist_mocap_mode_ == "right")
					model_index = crsf::RIGHT__THUMB_3 + i * 4;
				hand_->GetJointData(model_index)->Get3DModel()->SetHPR(hpr);
			}

			// 3rd joint = distal phalanges
			{
				LQuaternionf quat_result;

				// get [dip] data
				float dip = hand_mocap_data_[start_index + 4];
				if (i == 0) // thumb
				{
					//dip = dip - 17.8f;
					dip *= 1.4f;
				}
				else if (i == 1) // index
				{
					dip *= 1.2f;
				}
				else if (i == 2) // middle
				{
					dip *= 1.2f;
				}

				// calculate quaternion for hand model
				quat_result.set_from_axis_angle(dip, LVecBase3(0, 0, 1));
				// rotate unist hand -> crsf hand
				quat_result = rotate_unist_to_crsf(quat_result);

				// set hpr
				LVecBase3 hpr = quat_result.get_hpr();
				int model_index;
				if (unist_mocap_mode_ == "left")
					model_index = crsf::LEFT__THUMB_4 + i * 4;
				else if (unist_mocap_mode_ == "right")
					model_index = crsf::RIGHT__THUMB_4 + i * 4;
				hand_->GetJointData(model_index)->Get3DModel()->SetHPR(hpr);
			}

			// do hand model scaling using input data
			if (props_.get("subsystem.unistmocap_scale", false))
			{
				// get finger length
				float offset = hand_mocap_data_[23 + i] / 1000.0f;
				float dist[3];

				if (i == 0) // thumb
				{
					/*std::cout << offset << std::endl;
					offset *= 0.66;
					dist[0] = offset / 2 * 5;
					dist[1] = offset / 2 * 3;
					dist[2] = offset;*/

					dist[0] = offset * 0.5;
					dist[1] = offset * 0.3;
					dist[2] = offset * 0.2;
				}
				else
				{
					dist[0] = offset * 0.5;
					dist[1] = offset * 0.3;
					dist[2] = offset * 0.2;
				}

				// do scaling following hand model hierarchy
				int h;
				if (unist_mocap_mode_ == "left")
					h = 0;
				else if (unist_mocap_mode_ == "right")
					h = 1;
				int f = i;
				for (int j = 0; j < 3; j++)
				{
					int index = 1 + (h * 22) + (f * 4) + j;

					LVecBase3 origin_scale;
					if (j == 0) // 1st joint = proximal phalanges
						origin_scale = hand_->GetJointData(index)->Get3DModelStandardPoseScale();
					else
						origin_scale = hand_->GetJointData(index)->Get3DModel()->GetScale();
					hand_->GetJointData(index)->SetSensorOffset(dist[j]);
					float modified_offset_ratio = hand_->GetJointData(index)->GetScalingRatio_Offset();
					//std::cout << "index[" << index << "]: " << modified_offset_ratio << std::endl;

					LVecBase3 modified_scale = LVecBase3(origin_scale[0] * modified_offset_ratio,
						origin_scale[1],
						origin_scale[2]);
					hand_->GetJointData(index)->Get3DModel()->SetScale(modified_scale);

					if (f == 2) // in middle case, ring and pinky follows middle finger's scale 
					{
						hand_->GetJointData(index + 4)->Get3DModel()->SetScale(modified_scale);
						hand_->GetJointData(index + 8)->Get3DModel()->SetScale(modified_scale);
					}

					// hierarchy tree scaling (local scale balancing)
					if (j != 2)
					{
						LVecBase3 nextJoint_scale = hand_->GetJointData(index + 1)->Get3DModelStandardPoseScale();
						nextJoint_scale[0] *= 1.0f / modified_offset_ratio;
						hand_->GetJointData(index + 1)->Get3DModel()->SetScale(nextJoint_scale);

						if (f == 2) // in middle case, ring and pinky follows middle finger's scale 
						{
							hand_->GetJointData(index + 1 + 4)->Get3DModel()->SetScale(nextJoint_scale);
							hand_->GetJointData(index + 1 + 8)->Get3DModel()->SetScale(nextJoint_scale);
						}
					}
				}
			}
		}
	}
}

LQuaternionf HandManager::rotate_unist_to_crsf(const LQuaternionf& quat)
{
	LQuaternionf result;

	// rotate unist coord -> panda coord
	LQuaternionf a, b, ba;
	a.set_from_axis_angle(180, LVecBase3(1, 0, 0));
	b.set_from_axis_angle(90, LVecBase3(0, 1, 0));
	ba = b * a;
	result = ba.conjugate() * quat * ba;

	// rotate to leap hand origin pose
	if (unist_mocap_mode_ == "left")
	{
		LQuaternionf c, d, dc;
		c.set_from_axis_angle(90, LVecBase3(0, 1, 0));
		d.set_from_axis_angle(-90, LVecBase3(0, 0, 1));
		dc = d * c;
		result = dc.conjugate() * result * dc;
	}
	else if (unist_mocap_mode_ == "right")
	{
		LQuaternionf c, d, dc;
		c.set_from_axis_angle(90, LVecBase3(0, 1, 0));
		d.set_from_axis_angle(-90, LVecBase3(0, 0, 1));
		dc = d * c;
		result = dc.conjugate() * result * dc;
	}

	return result;
}