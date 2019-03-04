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

void HandManager::render_hand_mocap(crsf::TAvatarMemoryObject *amo)
{
	crsf::TWorld* virtual_world = crsf::TGraphicRenderEngine::GetInstance()->GetWorld();

	if (module_open_vr_)
	{
		if (hand_mocap_mode_ == BOTH)
		{
			trakcers_pos_[0] = module_open_vr_->GetDevicePosition(tracker_index_left_);
			trakcers_pos_[1] = module_open_vr_->GetDevicePosition(tracker_index_right_);
			trackers_quat_[0] = module_open_vr_->GetDeviceOrientation(tracker_index_left_);
			trackers_quat_[1] = module_open_vr_->GetDeviceOrientation(tracker_index_right_);

			// left
			{
				LQuaternionf a, b, ba;
				a.set_from_axis_angle(90, LVecBase3(0, 0, 1));
				b.set_from_axis_angle(90, LVecBase3(0, 1, 0));
				ba = b * a;
				trackers_quat_[0] = ba * trackers_quat_[0];
			}

			// right
			{
				LQuaternionf a, b, c, cba;
				a.set_from_axis_angle(90, LVecBase3(0, 0, 1));
				b.set_from_axis_angle(180, LVecBase3(1, 0, 0));
				c.set_from_axis_angle(-90, LVecBase3(0, 1, 0));
				cba = c * b * a;
				trackers_quat_[1] = cba * trackers_quat_[1];
			}

			// left
			{
				hand_->GetJointData(21)->Get3DModel()->SetPosition(trakcers_pos_[0], virtual_world);
				hand_->GetJointData(21)->SetPosition(trakcers_pos_[0]);
				hand_->GetJointData(21)->Get3DModel()->SetHPR(trackers_quat_[0].get_hpr(), virtual_world);
				LQuaternionf a, b, ba;
				a.set_from_axis_angle(90, LVecBase3(0, 0, 1));
				b.set_from_axis_angle(90, LVecBase3(1, 0, 0));
				ba = b * a;
				trackers_quat_[0] = ba * trackers_quat_[0];
				hand_->GetJointData(21)->SetOrientation(trackers_quat_[0]);
			}

			// right
			{
				hand_->GetJointData(43)->Get3DModel()->SetPosition(trakcers_pos_[1], virtual_world);
				hand_->GetJointData(43)->SetPosition(trakcers_pos_[1]);
				hand_->GetJointData(43)->Get3DModel()->SetHPR(trackers_quat_[1].get_hpr(), virtual_world);
				LQuaternionf a, b, ba;
				a.set_from_axis_angle(90, LVecBase3(0, 1, 0));
				b.set_from_axis_angle(-90, LVecBase3(0, 0, 1));
				ba = b * a;
				trackers_quat_[1] = ba * trackers_quat_[1];
				hand_->GetJointData(43)->SetOrientation(trackers_quat_[1]);
			}
		}
		else
		{
			if (hand_mocap_mode_ == LEFT)
			{
				trakcer_pos_ = module_open_vr_->GetDevicePosition(tracker_index_left_);
				tracker_quat_ = module_open_vr_->GetDeviceOrientation(tracker_index_left_);

				LQuaternionf a, b, ba;
				a.set_from_axis_angle(90, LVecBase3(0, 0, 1));
				b.set_from_axis_angle(90, LVecBase3(0, 1, 0));
				ba = b * a;
				tracker_quat_ = ba * tracker_quat_;

				hand_->GetJointData(21)->Get3DModel()->SetPosition(trakcer_pos_, virtual_world);
				hand_->GetJointData(21)->SetPosition(trakcer_pos_);
				hand_->GetJointData(21)->Get3DModel()->SetHPR(tracker_quat_.get_hpr(), virtual_world);
				LQuaternionf c, d, dc;
				c.set_from_axis_angle(90, LVecBase3(0, 0, 1));
				d.set_from_axis_angle(90, LVecBase3(1, 0, 0));
				dc = d * c;
				tracker_quat_ = dc * tracker_quat_;
				hand_->GetJointData(21)->SetOrientation(tracker_quat_);

				hand_->GetJointData(43)->Get3DModel()->SetPosition(LVecBase3(100), virtual_world);
				hand_->GetJointData(43)->SetPosition(LVecBase3(100));
			}

			if (hand_mocap_mode_ == RIGHT)
			{
				trakcer_pos_ = module_open_vr_->GetDevicePosition(tracker_index_right_);
				tracker_quat_ = module_open_vr_->GetDeviceOrientation(tracker_index_right_);

				LQuaternionf a, b, c, cba;
				a.set_from_axis_angle(90, LVecBase3(0, 0, 1));
				b.set_from_axis_angle(180, LVecBase3(1, 0, 0));
				c.set_from_axis_angle(-90, LVecBase3(0, 1, 0));
				cba = c * b * a;
				tracker_quat_ = cba * tracker_quat_;

				hand_->GetJointData(21)->Get3DModel()->SetPosition(LVecBase3(100), virtual_world);
				hand_->GetJointData(21)->SetPosition(LVecBase3(100));

				hand_->GetJointData(43)->Get3DModel()->SetPosition(trakcer_pos_, virtual_world);
				hand_->GetJointData(43)->SetPosition(trakcer_pos_);
				hand_->GetJointData(43)->Get3DModel()->SetHPR(tracker_quat_.get_hpr(), virtual_world);
				LQuaternionf d, e, ed;
				d.set_from_axis_angle(-90, LVecBase3(0, 0, 1));
				e.set_from_axis_angle(-90, LVecBase3(1, 0, 0));
				ed = e * d;
				tracker_quat_ = ed * tracker_quat_;
				hand_->GetJointData(43)->SetOrientation(tracker_quat_);
			}
		}
	}

	// if calibration process is finished,
	if (is_hand_mocap_calibration_)
	{
		int index;
		int model_index;

		for (int h = 0; h < 2; h++)
		{
			if (hand_mocap_mode_ == RIGHT)
				h = 1;

			for (int f = 0; f < 3; f++)
			{
				for (int j = 0; j < 4; j++)
				{
					index = (h * 12) + (f * 4) + j;
					model_index = 1 + (h * 22) + (f * 4) + j;

					// Get TPose from avatar memory object
					crsf::TPose getTPose = amo->GetAvatarMemory().at(index);

					// <<Rotation>>
					// Get quaternion from sensor
					LQuaternionf a = getTPose.GetQuaternion();

					// Set quaternion onto joint data
					hand_->GetJointData(model_index)->SetOrientation(a);

					// Rotate to hand model origin pose
					LQuaternionf quat_result;

					if ((h == 0 && hand_mocap_mode_ == BOTH) || hand_mocap_mode_ == LEFT) // left
					{
						LQuaternionf aa;
						aa.set_from_axis_angle(180, LVecBase3(1, 0, 0));
						a = aa.conjugate() * a * aa;
					}

					if ((h == 0 && hand_mocap_mode_ == BOTH) || hand_mocap_mode_ == LEFT) // left
					{
						if (f == 0 && j == 0)
						{
							LQuaternionf b, c, cb;
							b.set_from_axis_angle(-90, LVecBase3(0, -1, 0));
							c.set_from_axis_angle(-90, LVecBase3(0, 0, 1));
							cb = c * b;
							quat_result = cb.conjugate() * a * cb;
						}
						else
						{
							LQuaternionf b, c, cb;
							b.set_from_axis_angle(90, LVecBase3(0, 1, 0));
							c.set_from_axis_angle(90, LVecBase3(0, 0, -1));
							cb = c * b;
							quat_result = cb.conjugate() * a * cb;
						}
					}
					if ((h == 1 && hand_mocap_mode_ == BOTH) || hand_mocap_mode_ == RIGHT) // right
					{
						LQuaternionf b, c, cb;
						b.set_from_axis_angle(90, LVecBase3(0, 1, 0));
						c.set_from_axis_angle(-90, LVecBase3(0, 0, 1));
						cb = c * b;
						quat_result = cb.conjugate() * a * cb;
					}

					if (((h == 0 && hand_mocap_mode_ == BOTH) || hand_mocap_mode_ == LEFT) && f == 0 && j == 0) // Left thumb - multiply initial quaternion
					{
						LQuaternionf e, f, g, gfe;
						e.set_from_axis_angle(1.39323033616515f * 180.0f / M_PI, LVecBase3(-1, 0, 0));
						f.set_from_axis_angle(0.533031068419669f * 180.0f / M_PI, LVecBase3(0, 0, -1));
						g.set_from_axis_angle(0.907932313878427f * 180.0f / M_PI, LVecBase3(0, 1, 0));
						gfe = g * f * e;
						LQuaternionf h;
						h.set_from_axis_angle(-45, LVecBase3(0, 1, 0));
						gfe = h * gfe;
						quat_result = quat_result * gfe;
					}
					if (((h == 1 && hand_mocap_mode_ == BOTH) || hand_mocap_mode_ == RIGHT) && f == 0 && j == 0) // Right thumb - multiply initial quaternion
					{
						LQuaternionf e, f, g, gfe;
						e.set_from_axis_angle(-1.39323033616515f * 180.0f / M_PI, LVecBase3(1, 0, 0));
						f.set_from_axis_angle(0.533031068419669f * 180.0f / M_PI, LVecBase3(0, 0, -1));
						g.set_from_axis_angle(0.907932313878427f * 180.0f / M_PI, LVecBase3(0, 1, 0));
						gfe = g * f * e;
						LQuaternionf h;
						h.set_from_axis_angle(-45, LVecBase3(0, 1, 0));
						gfe = h * gfe;
						quat_result = quat_result * gfe;
					}

					// Set rotation on 3d model
					auto hpr = quat_result.get_hpr();
					if (j != 3) // Not tip (tip does not have joint)
						hand_->GetJointData(model_index)->Get3DModel()->SetHPR(hpr);

					// Set rotation about ring & pinky joints same with middle joint
					if (f == 2 && j != 3)
					{
						hand_->GetJointData(model_index + 4)->Get3DModel()->SetHPR(hpr);
						hand_->GetJointData(model_index + 8)->Get3DModel()->SetHPR(hpr);
					}

					// <<Position>>
					// Get position from sensor (local coordinate from root(=wrist))
					LVecBase3 temp_pos = getTPose.GetPosition();
					temp_pos *= 0.001f;
					if (j != 3)
						hand_->GetJointData(model_index)->SetPosition(temp_pos); // Save the local position

					if (props_.get("subsystem.handmocap_position", false))
					{
						// Rotate to hand model origin pose
						LQuaternionf root_quat;
						if (h == 0)
							root_quat = hand_->GetJointData(21)->GetOrientation();
						else if (h == 1)
							root_quat = hand_->GetJointData(43)->GetOrientation();
						temp_pos = rotate_pos_by_quat(temp_pos, root_quat);

						// Set local position to world position
						LVecBase3 root_pos;
						if (h == 0)
							root_pos = hand_->GetJointData(21)->Get3DModel()->GetPosition(virtual_world);
						else if (h == 1)
							root_pos = hand_->GetJointData(43)->Get3DModel()->GetPosition(virtual_world);
						LVecBase3 new_pos;
						new_pos = root_pos + temp_pos;

						// Set position on 3d model
						if (j != 3)
							hand_->GetJointData(model_index)->Get3DModel()->SetPosition(new_pos, virtual_world);
					}
				}
			}

			if (hand_mocap_mode_ != BOTH)
				break;
		}

		if (props_.get("subsystem.handmocap_scale", false))
		{
			// Set sensor offset
			// Left palm
			if (hand_mocap_mode_ == BOTH || hand_mocap_mode_ == LEFT)
			{
				int i = 21;

				// offset
				{
					float dist = hand_->GetJointData(9)->GetPosition().length(); // middle finger_1
					hand_->GetJointData(i)->SetSensorOffset(dist);
				}
			}

			// Right palm
			// offset
			if (hand_mocap_mode_ == BOTH || hand_mocap_mode_ == RIGHT)
			{
				int i = 43;

				// offset
				{
					float dist = hand_->GetJointData(31)->GetPosition().length(); // middle finger_1
					hand_->GetJointData(i)->SetSensorOffset(dist);
				}

				// width
				{
					LVecBase3 middle_1 = hand_->GetJointData(31)->GetPosition();
					LVecBase3 thumb_1 = hand_->GetJointData(23)->GetPosition();
					float dist = dist_two_vector(middle_1, thumb_1);
					hand_->GetJointData(i)->SetSensorWidth(dist);
				}

				// thickness
				{

					float dist = hand_->GetJointData(23)->GetPosition().length();
					hand_->GetJointData(i)->SetSensorThickness(dist);
				}
			}

			// Finger bone
			for (int h = 0; h < 2; h++)
			{
				if (hand_mocap_mode_ == RIGHT)
					h = 1;

				for (int f = 0; f < 3; f++)
				{
					for (int j = 0; j < 3; j++)
					{
						index = (h * 12) + (f * 4) + j;
						model_index = 1 + (h * 22) + (f * 4) + j;

						// offset
						if (j != 2)
						{
							LVecBase3 pos_cur = hand_->GetJointData(model_index)->GetPosition();
							LVecBase3 pos_next = hand_->GetJointData(model_index + 1)->GetPosition();
							float vx = pos_next[0] - pos_cur[0];
							float vy = pos_next[1] - pos_cur[1];
							float vz = pos_next[2] - pos_cur[2];
							float dist = (float)sqrt(vx*vx + vy * vy + vz * vz);
							hand_->GetJointData(model_index)->SetSensorOffset(dist);
						}
						else
						{
							LVecBase3 pos_cur = hand_->GetJointData(model_index)->GetPosition();
							LVecBase3 pos_next = amo->GetAvatarMemory().at(index + 1).GetPosition() * 0.001f;
							float vx = pos_next[0] - pos_cur[0];
							float vy = pos_next[1] - pos_cur[1];
							float vz = pos_next[2] - pos_cur[2];
							float dist = (float)sqrt(vx*vx + vy * vy + vz * vz);
							hand_->GetJointData(model_index)->SetSensorOffset(dist);
						}
					}
				}

				if (hand_mocap_mode_ != BOTH)
					break;
			}

			// Do auto-scaling by offset ratio on mechanism
			// Left palm
			if (hand_mocap_mode_ == BOTH || hand_mocap_mode_ == LEFT)
			{
				int i = 21;

				LVecBase3 origin_scale = hand_->GetJointData(i)->Get3DModelStandardPoseScale();

				float modified_offset_ratio = hand_->GetJointData(i)->GetScalingRatio_Offset();
				float modified_width_ratio = hand_->GetJointData(i)->GetScalingRatio_Width();
				float modified_thickness_ratio = hand_->GetJointData(i)->GetScalingRatio_Thickness();

				LVecBase3 modified_scale = LVecBase3(origin_scale[0] * modified_offset_ratio,
					origin_scale[1] * modified_width_ratio,
					origin_scale[2] * modified_thickness_ratio);
				hand_->GetJointData(i)->Get3DModel()->SetScale(modified_scale);

				int thumb_start = 1;
				for (int j = 0; j < 5; j++)
				{
					int cur_index = thumb_start + (j * 4);
					// hierarchy tree scaling (local scale balancing)
					LVecBase3 nextJoint_scale = hand_->GetJointData(cur_index)->Get3DModelStandardPoseScale();
					nextJoint_scale[0] *= 1.0f / modified_offset_ratio;
					hand_->GetJointData(cur_index)->Get3DModel()->SetScale(nextJoint_scale);
				}
			}

			// Right palm
			if (hand_mocap_mode_ == BOTH || hand_mocap_mode_ == RIGHT)
			{
				int i = 43;

				LVecBase3 origin_scale = hand_->GetJointData(i)->Get3DModelStandardPoseScale();

				float modified_offset_ratio = hand_->GetJointData(i)->GetScalingRatio_Offset();
				float modified_width_ratio = hand_->GetJointData(i)->GetScalingRatio_Width();
				float modified_thickness_ratio = hand_->GetJointData(i)->GetScalingRatio_Thickness();

				LVecBase3 modified_scale = LVecBase3(origin_scale[0] * modified_offset_ratio,
					origin_scale[1],
					origin_scale[2]);
				hand_->GetJointData(i)->Get3DModel()->SetScale(modified_scale);

				int thumb_start = 23;
				for (int f = 0; f < 5; f++)
				{
					int cur_index = thumb_start + (f * 4);
					// hierarchy tree scaling (local scale balancing)
					LVecBase3 nextJoint_scale = hand_->GetJointData(cur_index)->Get3DModelStandardPoseScale();
					nextJoint_scale[0] *= 1.0f / modified_offset_ratio;
					hand_->GetJointData(cur_index)->Get3DModel()->SetScale(nextJoint_scale);
				}
			}

			// Finger bone
			for (int h = 0; h < 2; h++)
			{
				if (hand_mocap_mode_ == RIGHT)
					h = 1;

				for (int f = 0; f < 3; f++)
				{
					for (int j = 0; j < 3; j++)
					{
						int i = 1 + (h * 22) + (f * 4) + j;

						LVecBase3 origin_scale = hand_->GetJointData(i)->Get3DModel()->GetScale();

						float modified_offset_ratio = hand_->GetJointData(i)->GetScalingRatio_Offset();

						LVecBase3 modified_scale = LVecBase3(origin_scale[0] * modified_offset_ratio,
							origin_scale[1],
							origin_scale[2]);
						hand_->GetJointData(i)->Get3DModel()->SetScale(modified_scale);

						// hierarchy tree scaling (local scale balancing)
						if (j != 2)
						{
							LVecBase3 nextJoint_scale = hand_->GetJointData(i + 1)->Get3DModelStandardPoseScale();
							nextJoint_scale[0] *= 1.0f / modified_offset_ratio;
							hand_->GetJointData(i + 1)->Get3DModel()->SetScale(nextJoint_scale);

							if (f == 2)
							{
								hand_->GetJointData(i + 1 + 4)->Get3DModel()->SetScale(nextJoint_scale);
								hand_->GetJointData(i + 1 + 8)->Get3DModel()->SetScale(nextJoint_scale);
							}
						}
					}
				}

				if (hand_mocap_mode_ != BOTH)
					break;
			}
		}
	}
}