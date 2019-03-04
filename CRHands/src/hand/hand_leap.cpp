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

void HandManager::render_hand_leap(crsf::TAvatarMemoryObject *amo)
{
	crsf::TWorld* virtual_world = crsf::TGraphicRenderEngine::GetInstance()->GetWorld();

	//////////////////////////////////////////////////////////////////////////
	// if- TCRHand,
	if (hand_)
	{
		// # HMD
		//////////////////////////////////////////////////////////////////////////
		// <<< Orbit >>>
		// Translation to designated hand pose (set sensor's origin on CRSF)
		LMatrix4f zeroToSensorMatrix;
		zeroToSensorMatrix.set_translate_mat(hand_->GetHandProperty().m_vec3ZeroToSensor);

		// set orbit matrix
		auto cam_matrix = rpcore::Globals::base->get_cam().get_mat(rpcore::Globals::render);
		cam_matrix.set_row(3, cam_matrix.get_row3(3) / crsf::TGraphicRenderEngine::GetInstance()->GetRenderingScaleFactor());
		node_hmd_->SetMatrix(zeroToSensorMatrix * cam_matrix);

		// # joint
		int nJointNumber = hand_->GetJointNumber();

		//////////////////////////////////////////////////////////////////////////
		// Loop all joint
		for (int i = 0; i < nJointNumber; i++)
		{
			//////////////////////////////////////////////////////////////////////////
			// <<< Read joint TPose from AvatarMemory >>>
			crsf::TPose getTPose = amo->GetAvatarMemory().at(i);
			// <<< Read joint TAvatarHeader from AvatarMemory >>>
			crsf::TAvatarHeader getTAvatarHeader = amo->GetAvatarHeader().at(i);

			//////////////////////////////////////////////////////////////////////////
			// <<< Update flag >>>
			// 1. Read joint update flag
			bool bUpdate = getTAvatarHeader.m_bUpdate;

			// 2. Register hand joint update flag
			hand_->GetJointData(i)->SetIsUpdate(bUpdate);

			//////////////////////////////////////////////////////////////////////////
			// <<< Position >>>
			// 1. Read joint position
			LVecBase3 vec3Position = getTPose.GetPosition();

			// 2. Translation to designated hand pose (set sensor's origin on CRSF)
			vec3Position = vec3Position + hand_->GetHandProperty().m_vec3ZeroToSensor;

			// 3. Register hand joint position
			hand_->GetJointData(i)->SetPosition(vec3Position);

			//////////////////////////////////////////////////////////////////////////
			// <<< Orientation >>>
			// 1. Read joint orientation
			LQuaternionf quatOrientation = getTPose.GetQuaternion();

			// 2. Register hand joint orientation
			hand_->GetJointData(i)->SetOrientation(quatOrientation);

			// 3. Update 3D model's orientation
			if (hand_->GetHandProperty().m_bRender3DModel && hand_->GetHandProperty().m_p3DModel)
			{
				crsf::TWorldObject* pTextureModel = hand_->GetJointData(i)->Get3DModel();
				std::string strTextureName = hand_->GetJointData(i)->Get3DModelNodeName();

				if (pTextureModel && strTextureName != "")
				{
					//////////////////////////////////////////////////////////////////////////
					// Left
					if (i == 1) // Thumb_1
					{
						// Calculate quaternion for hand model
						LQuaternionf a, b, ba;
						a.set_from_axis_angle(-90, LVecBase3(0, -1, 0));
						b.set_from_axis_angle(-90, LVecBase3(0, 0, 1));
						ba = b * a;
						// Mapped quaternion result
						LQuaternionf quat_result = ba.conjugate() * quatOrientation * ba;
						// Multiply quaternion for thumb rotation
						LQuaternionf c, d, e, edc;
						c.set_from_axis_angle(1.406013982f * 180.0f / M_PI, LVecBase3(-1, 0, 0));
						d.set_from_axis_angle(0.453798839f * 180.0f / M_PI, LVecBase3(0, 0, -1));
						e.set_from_axis_angle(0.905946589f * 180.0f / M_PI, LVecBase3(0, 1, 0));
						edc = e * d * c;
						LQuaternionf f;
						f.set_from_axis_angle(-45, LVecBase3(0, 1, 0));
						edc = f * edc;
						quat_result = quat_result * edc;
						// Set hpr
						LVecBase3 hpr = quat_result.get_hpr();
						pTextureModel->SetHPR(hpr);
					}
					else if (i >= 2 && i <= 20)
					{
						// Calculate quaternion for hand model
						LQuaternionf a, b, ba;
						a.set_from_axis_angle(90, LVecBase3(0, 1, 0));
						b.set_from_axis_angle(90, LVecBase3(0, 0, -1));
						ba = b * a;
						// Mapped quaternion result
						LQuaternionf quat_result = ba.conjugate() * quatOrientation * ba;
						LVecBase3 hpr = quat_result.get_hpr();
						// Set hpr
						pTextureModel->SetHPR(hpr);
					}
					else if (i == 21) // Root(wrist)
					{
						// Set position
						if (hand_character_->GetParent())
						{
							pTextureModel->SetPosition(vec3Position, hand_character_->GetParent());

							// Rotate hand model to LEAP base
							LQuaternionf a, b, ba;
							a.set_from_axis_angle(-90, LVecBase3(0, 1, 0));
							b.set_from_axis_angle(-90, LVecBase3(1, 0, 0));
							ba = b * a;
							// Quaternion result
							LQuaternionf quat_result = ba * quatOrientation;
							LVecBase3 hpr = quat_result.get_hpr();
							// Set hpr
							pTextureModel->SetHPR(hpr);
						}
					}

					//////////////////////////////////////////////////////////////////////////
					// Right
					else if (i == 23) // Thumb_1
					{
						// Calculate quaternion for hand model
						LQuaternionf a, b, ba;
						a.set_from_axis_angle(90, LVecBase3(0, 1, 0));
						b.set_from_axis_angle(-90, LVecBase3(0, 0, 1));
						ba = b * a;
						// Mapped quaternion result
						LQuaternionf quat_result = ba.conjugate() * quatOrientation * ba;
						// Multiply quaternion for thumb rotation
						LQuaternionf c, d, e, edc;
						c.set_from_axis_angle(-1.406013982f * 180.0f / M_PI, LVecBase3(1, 0, 0));
						d.set_from_axis_angle(0.453798839f * 180.0f / M_PI, LVecBase3(0, 0, -1));
						e.set_from_axis_angle(0.905946589f * 180.0f / M_PI, LVecBase3(0, 1, 0));
						edc = e * d * c;
						LQuaternionf f;
						f.set_from_axis_angle(-45, LVecBase3(0, 1, 0));
						edc = f * edc;
						quat_result = quat_result * edc;
						// Set hpr
						LVecBase3 hpr = quat_result.get_hpr();
						pTextureModel->SetHPR(hpr);
					}
					else if (i >= 24 && i <= 42)
					{
						// Calculate quaternion for hand model
						LQuaternionf a, b, ba;
						a.set_from_axis_angle(90, LVecBase3(0, 1, 0));
						b.set_from_axis_angle(-90, LVecBase3(0, 0, 1));
						ba = b * a;
						// Mapped quaternion result
						LQuaternionf quat_result = ba.conjugate() * quatOrientation * ba;
						LVecBase3 hpr = quat_result.get_hpr();
						// Set hpr
						pTextureModel->SetHPR(hpr);
					}
					else if (i == 43) // Root(wrist)
					{
						// Set position
						if (hand_character_->GetParent())
						{
							pTextureModel->SetPosition(vec3Position, hand_character_->GetParent());

							// Rotate hand model to LEAP base
							LQuaternionf a, b, ba;
							a.set_from_axis_angle(-90, LVecBase3(0, 1, 0));
							b.set_from_axis_angle(90, LVecBase3(1, 0, 0));
							ba = b * a;
							// Quaternion result
							LQuaternionf quat_result = ba * quatOrientation;
							LVecBase3 hpr = quat_result.get_hpr();
							// Set hpr
							pTextureModel->SetHPR(hpr);
						}
					}
				}
			}


			//////////////////////////////////////////////////////////////////////////
			// <<< Width >>>
			// 1. Read joint width
			float fWidth = getTAvatarHeader.m_fWidth;

			// 2. Register hand joint width
			hand_->GetJointData(i)->SetWidth(fWidth);
		}
	}
}