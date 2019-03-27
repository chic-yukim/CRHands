/**
* Coexistence Reality Software Framework (CRSF)
* Copyright (c) Center of Human-centered Interaction for Coexistence. All rights reserved.
* See the LICENSE.md file for more details.
*/

#include "hand_manager.hpp"

#include <render_pipeline/rppanda/showbase/showbase.hpp>
#include <render_pipeline/rpcore/globals.hpp>

#include <crsf/CoexistenceInterface/TDynamicStageMemory.h>
#include <crsf/CREngine/TDynamicModuleManager.h>

#include <crsf/RenderingEngine/TGraphicRenderEngine.h>
#include <crsf/CRModel/TWorld.h>
#include <crsf/CRModel/TWorldObject.h>
#include <crsf/CRModel/TCharacter.h>

#include <crsf/CRModel/TCRHand.h>

#include <crsf/CoexistenceInterface/TAvatarMemoryObject.h>
#include <crsf/System/TPose.h>

void HandManager::render_hand_leap(crsf::TAvatarMemoryObject *amo)
{
	crsf::TWorld* virtual_world = crsf::TGraphicRenderEngine::GetInstance()->GetWorld();

	// if- TCRHand,
	if (hand_)
	{
		// set matrix from user's origin to LEAP
		LMatrix4f origin_to_leap_mat = LMatrix4f::ident_mat();
		origin_to_leap_mat.set_translate_mat(hand_->GetHandProperty().m_vec3ZeroToSensor);

		// VR mode - leap local translation is 'HMD-to-LEAP'
		if (crsf::TDynamicModuleManager::GetInstance()->IsModuleEnabled("openvr"))
		{
			// set orbit matrix (following camera)
			auto cam_matrix = rpcore::Globals::base->get_cam().get_mat(rpcore::Globals::render);
			cam_matrix.set_row(3, cam_matrix.get_row3(3) / crsf::TGraphicRenderEngine::GetInstance()->GetRenderingScaleFactor());
			node_hmd_->SetMatrix(origin_to_leap_mat * cam_matrix);
		}
		else // mono mode - leap local translation is 'zero-to-LEAP'
		{
			node_hmd_->SetMatrix(origin_to_leap_mat);
		}

		// # joint
		int joint_number = hand_->GetJointNumber();

		// Loop all joint
		for (int i = 0; i < joint_number; i++)
		{
			// read joint TPose from AvatarMemory
			crsf::TPose get_avatar_pose = amo->GetAvatarMemory().at(i);
			// read joint TAvatarHeader from AvatarMemory
			crsf::TAvatarHeader get_avatar_header = amo->GetAvatarHeader().at(i);

			// [position]
			// 1. read joint position
			LVecBase3 joint_position = get_avatar_pose.GetPosition();

			// 2. register hand joint position
			hand_->GetJointData(i)->SetPosition(joint_position);

			// [quaternion]
			// 1. read joint quaternion
			LQuaternionf joint_quaternion = get_avatar_pose.GetQuaternion();

			// 2. register hand joint quaternion
			hand_->GetJointData(i)->SetOrientation(joint_quaternion);

			// [pose update]
			// update 3D model's pose
			if (hand_->GetHandProperty().m_bRender3DModel && hand_->GetHandProperty().m_p3DModel)
			{
				crsf::TWorldObject* joint_model = hand_->GetJointData(i)->Get3DModel();
				std::string joint_model_name = hand_->GetJointData(i)->Get3DModelNodeName();

				if (joint_model && joint_model_name != "")
				{
					LQuaternionf quat_result;

					// left
					if (i == 1) // thumb_1
					{
						// convert leap coordinate -> CRSF hand coordinate
						if (leap_mode_ == "HMD")
						{
							LQuaternionf a, b, ba;
							a.set_from_axis_angle(90, LVecBase3(0, 1, 0));
							b.set_from_axis_angle(-90, LVecBase3(0, 0, 1));
							ba = b * a;
							quat_result = ba.conjugate() * joint_quaternion * ba;
						}
						else if (leap_mode_ == "floor")
						{
							LQuaternionf a, b, ba;
							a.set_from_axis_angle(-90, LVecBase3(0, 0, 1));
							b.set_from_axis_angle(0, LVecBase3(0, 0, 1));
							ba = b * a;
							quat_result = ba.conjugate() * joint_quaternion * ba;
						}

						// multiply quaternion for thumb initial rotation
						LQuaternionf c, d, e, edc;
						c.set_from_axis_angle(1.406013982f * 180.0f / M_PI, LVecBase3(-1, 0, 0));
						d.set_from_axis_angle(0.453798839f * 180.0f / M_PI, LVecBase3(0, 0, -1));
						e.set_from_axis_angle(0.905946589f * 180.0f / M_PI, LVecBase3(0, 1, 0));
						edc = e * d * c;
						LQuaternionf f;
						f.set_from_axis_angle(-45, LVecBase3(0, 1, 0));
						edc = f * edc;
						quat_result = quat_result * edc;

						// set hpr
						LVecBase3 hpr = quat_result.get_hpr();
						joint_model->SetHPR(hpr);
					}
					else if (i >= 2 && i <= 20)
					{
						// convert leap coordinate -> CRSF hand coordinate
						if (leap_mode_ == "HMD")
						{
							LQuaternionf a, b, ba;
							a.set_from_axis_angle(90, LVecBase3(0, 1, 0));
							b.set_from_axis_angle(-90, LVecBase3(0, 0, 1));
							ba = b * a;
							quat_result = ba.conjugate() * joint_quaternion * ba;
						}
						else if (leap_mode_ == "floor")
						{
							LQuaternionf a, b, ba;
							a.set_from_axis_angle(-90, LVecBase3(0, 0, 1));
							b.set_from_axis_angle(0, LVecBase3(0, 0, 1));
							ba = b * a;
							quat_result = ba.conjugate() * joint_quaternion * ba;
						}

						// set hpr
						LVecBase3 hpr = quat_result.get_hpr();
						joint_model->SetHPR(hpr);
					}
					else if (i == 21) // root(wrist)
					{
						if (hand_character_->GetParent())
						{
							// set position
							joint_model->SetPosition(joint_position, hand_character_->GetParent());

							// rotate hand model to LEAP base
							if (leap_mode_ == "HMD")
							{
								LQuaternionf a, b, ba;
								a.set_from_axis_angle(-90, LVecBase3(0, 1, 0));
								b.set_from_axis_angle(-90, LVecBase3(1, 0, 0));
								ba = b * a;
								quat_result = ba * joint_quaternion;
							}
							else if (leap_mode_ == "floor")
							{
								LQuaternionf a, b, ba;
								a.set_from_axis_angle(90, LVecBase3(0, 0, 1));
								b.set_from_axis_angle(0, LVecBase3(1, 0, 0));
								ba = b * a;
								quat_result = ba * joint_quaternion;
							}
							
							// set hpr
							LVecBase3 hpr = quat_result.get_hpr();
							joint_model->SetHPR(hpr);
						}
					}

					// right
					else if (i == 23) // thumb_1
					{
						// convert leap coordinate -> CRSF hand coordinate
						if (leap_mode_ == "HMD")
						{
							LQuaternionf a, b, ba;
							a.set_from_axis_angle(90, LVecBase3(0, 1, 0));
							b.set_from_axis_angle(-90, LVecBase3(0, 0, 1));
							ba = b * a;
							quat_result = ba.conjugate() * joint_quaternion * ba;
						}
						else if (leap_mode_ == "floor")
						{
							LQuaternionf a, b, ba;
							a.set_from_axis_angle(-90, LVecBase3(0, 0, 1));
							b.set_from_axis_angle(180, LVecBase3(0, 1, 0));
							ba = b * a;
							quat_result = ba.conjugate() * joint_quaternion * ba;
						}
						
						// multiply quaternion for thumb initial rotation
						LQuaternionf c, d, e, edc;
						c.set_from_axis_angle(-1.406013982f * 180.0f / M_PI, LVecBase3(1, 0, 0));
						d.set_from_axis_angle(0.453798839f * 180.0f / M_PI, LVecBase3(0, 0, -1));
						e.set_from_axis_angle(0.905946589f * 180.0f / M_PI, LVecBase3(0, 1, 0));
						edc = e * d * c;
						LQuaternionf f;
						f.set_from_axis_angle(-45, LVecBase3(0, 1, 0));
						edc = f * edc;
						quat_result = quat_result * edc;

						// set hpr
						LVecBase3 hpr = quat_result.get_hpr();
						joint_model->SetHPR(hpr);
					}
					else if (i >= 24 && i <= 42)
					{
						// convert leap coordinate -> CRSF hand coordinate
						if (leap_mode_ == "HMD")
						{
							LQuaternionf a, b, ba;
							a.set_from_axis_angle(90, LVecBase3(0, 1, 0));
							b.set_from_axis_angle(-90, LVecBase3(0, 0, 1));
							ba = b * a;
							quat_result = ba.conjugate() * joint_quaternion * ba;
						}
						else if (leap_mode_ == "floor")
						{
							LQuaternionf a, b, ba;
							a.set_from_axis_angle(-90, LVecBase3(0, 0, 1));
							b.set_from_axis_angle(180, LVecBase3(0, 1, 0));
							ba = b * a;
							quat_result = ba.conjugate() * joint_quaternion * ba;
						}

						// set hpr
						LVecBase3 hpr = quat_result.get_hpr();
						joint_model->SetHPR(hpr);
					}
					else if (i == 43) // root(wrist)
					{
						if (hand_character_->GetParent())
						{
							// set position
							joint_model->SetPosition(joint_position, hand_character_->GetParent());

							// rotate hand model to LEAP base
							if (leap_mode_ == "HMD")
							{
								LQuaternionf a, b, ba;
								a.set_from_axis_angle(-90, LVecBase3(0, 1, 0));
								b.set_from_axis_angle(90, LVecBase3(1, 0, 0));
								ba = b * a;
								quat_result = ba * joint_quaternion;
							}
							else if (leap_mode_ == "floor")
							{
								LQuaternionf a, b, ba;
								a.set_from_axis_angle(90, LVecBase3(0, 0, 1));
								b.set_from_axis_angle(180, LVecBase3(1, 0, 0));
								ba = b * a;
								quat_result = ba * joint_quaternion;
							}

							// set hpr
							LVecBase3 hpr = quat_result.get_hpr();
							joint_model->SetHPR(hpr);
						}
					}
				}
			}
		}
	}
}