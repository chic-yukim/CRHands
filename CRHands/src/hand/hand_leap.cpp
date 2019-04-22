/**
* Coexistence Reality Software Framework (CRSF)
* Copyright (c) Center of Human-centered Interaction for Coexistence. All rights reserved.
* See the LICENSE.md file for more details.
*/

#include "hand_leap.hpp"

#include <render_pipeline/rppanda/showbase/showbase.hpp>
#include <render_pipeline/rpcore/globals.hpp>

#include <crsf/CoexistenceInterface/TDynamicStageMemory.h>
#include <crsf/CoexistenceInterface/TAvatarMemoryObject.h>
#include <crsf/CREngine/TDynamicModuleManager.h>
#include <crsf/CRModel/TWorld.h>
#include <crsf/CRModel/TCharacter.h>
#include <crsf/CRModel/TCRHand.h>
#include <crsf/RealWorldInterface/TInterfaceManager.h>
#include <crsf/RenderingEngine/TGraphicRenderEngine.h>
#include <crsf/System/TPose.h>

#include <leapmotion_interface.h>

#include "hand/hand.hpp"

enum LeapMotionMode
{
    LEAP_MOTION_MODE_FLOOR,
    LEAP_MOTION_MODE_HMD
};

void render_hand_leap_local(Hand* hand, crsf::TAvatarMemoryObject* amo)
{
    static const double M_PI = std::acos(-1);

    if (!hand)
        return;

    auto crhand = hand->get_hand();

    if (!(crhand->GetHandProperty().m_bRender3DModel && crhand->GetHandProperty().m_p3DModel))
        return;

    auto leapmotion_interface = dynamic_cast<LeapMotionInterface*>(crsf::TInterfaceManager::GetInstance()->GetInputInterface("LeapMotion"));
    LeapMotionMode leap_mode = LEAP_MOTION_MODE_FLOOR;
    if (leapmotion_interface)
    {
        auto mode = leapmotion_interface->GetMode();
        if (mode == "HMD")
            leap_mode = LEAP_MOTION_MODE_HMD;
        else if (mode == "floor")
            leap_mode = LEAP_MOTION_MODE_FLOOR;
    }

    auto rendering_engine = crsf::TGraphicRenderEngine::GetInstance();
    auto world = rendering_engine->GetWorld();

    // set matrix from user's origin to LEAP
    LMatrix4f origin_to_leap_mat = LMatrix4f::ident_mat();
    origin_to_leap_mat.set_translate_mat(crhand->GetHandProperty().m_vec3ZeroToSensor);

    // VR mode - leap local translation is 'HMD-to-LEAP'
    if (crsf::TDynamicModuleManager::GetInstance()->IsModuleEnabled("openvr"))
    {
        // set orbit matrix (following camera)
        auto cam_matrix = rpcore::Globals::base->get_cam().get_mat(rpcore::Globals::render);
        cam_matrix.set_row(3, cam_matrix.get_row3(3) / rendering_engine->GetRenderingScaleFactor());
        hand->get_object()->SetMatrix(origin_to_leap_mat * cam_matrix);
    }
    // mono mode - leap local translation is 'zero-to-LEAP'
    else
    {
        hand->get_object()->SetMatrix(origin_to_leap_mat);
    }

    std::vector<crsf::TPose> dest_poses;
    auto dest_amo = hand->get_avatar_memory_object();
    if (dest_amo)
    {
        dest_poses = dest_amo->GetAvatarMemory();
    }

    // # joint
    const unsigned int joint_number = crhand->GetJointNumber();

    // Loop all joint
    for (unsigned int i = 0; i < joint_number; i++)
    {
        // read joint TPose from AvatarMemory
        const auto& get_avatar_pose = amo->GetAvatarMemory(i);
        // read joint TAvatarHeader from AvatarMemory
        const auto& get_avatar_header = amo->GetAvatarHeader(i);

        // [position]
        // 1. read joint position
        LVecBase3 joint_position = get_avatar_pose.GetPosition();

        // 2. register hand joint position
        crhand->GetJointData(i)->SetPosition(joint_position);

        // [quaternion]
        // 1. read joint quaternion
        LQuaternionf joint_quaternion = get_avatar_pose.GetQuaternion();

        // 2. register hand joint quaternion
        crhand->GetJointData(i)->SetOrientation(joint_quaternion);

        // [pose update]
        // update 3D model's pose
        crsf::TWorldObject* joint_model = crhand->GetJointData(i)->Get3DModel();
        if (joint_model)
        {
            LQuaternionf quat_result;

            // left
            if (i == 1) // thumb_1
            {
                // convert leap coordinate -> CRSF hand coordinate
                if (leap_mode == LEAP_MOTION_MODE_HMD)
                {
                    LQuaternionf a, b, ba;
                    a.set_from_axis_angle(90, LVecBase3(0, 1, 0));
                    b.set_from_axis_angle(-90, LVecBase3(0, 0, 1));
                    ba = b * a;
                    quat_result = ba.conjugate() * joint_quaternion * ba;
                }
                else if (leap_mode == LEAP_MOTION_MODE_FLOOR)
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
                if (leap_mode == LEAP_MOTION_MODE_HMD)
                {
                    LQuaternionf a, b, ba;
                    a.set_from_axis_angle(90, LVecBase3(0, 1, 0));
                    b.set_from_axis_angle(-90, LVecBase3(0, 0, 1));
                    ba = b * a;
                    quat_result = ba.conjugate() * joint_quaternion * ba;
                }
                else if (leap_mode == LEAP_MOTION_MODE_FLOOR)
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
                const auto& parent = crhand->Get3DModel()->GetParent();
                if (parent)
                {
                    // set position
                    joint_model->SetPosition(joint_position, parent);

                    // rotate hand model to LEAP base
                    if (leap_mode == LEAP_MOTION_MODE_HMD)
                    {
                        LQuaternionf a, b, ba;
                        a.set_from_axis_angle(-90, LVecBase3(0, 1, 0));
                        b.set_from_axis_angle(-90, LVecBase3(1, 0, 0));
                        ba = b * a;
                        quat_result = ba * joint_quaternion;
                    }
                    else if (leap_mode == LEAP_MOTION_MODE_FLOOR)
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
                if (leap_mode == LEAP_MOTION_MODE_HMD)
                {
                    LQuaternionf a, b, ba;
                    a.set_from_axis_angle(90, LVecBase3(0, 1, 0));
                    b.set_from_axis_angle(-90, LVecBase3(0, 0, 1));
                    ba = b * a;
                    quat_result = ba.conjugate() * joint_quaternion * ba;
                }
                else if (leap_mode == LEAP_MOTION_MODE_FLOOR)
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
                if (leap_mode == LEAP_MOTION_MODE_HMD)
                {
                    LQuaternionf a, b, ba;
                    a.set_from_axis_angle(90, LVecBase3(0, 1, 0));
                    b.set_from_axis_angle(-90, LVecBase3(0, 0, 1));
                    ba = b * a;
                    quat_result = ba.conjugate() * joint_quaternion * ba;
                }
                else if (leap_mode == LEAP_MOTION_MODE_FLOOR)
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
                const auto& parent = crhand->Get3DModel()->GetParent();
                if (parent)
                {
                    // set position
                    joint_model->SetPosition(joint_position, parent);

                    // rotate hand model to LEAP base
                    if (leap_mode == LEAP_MOTION_MODE_HMD)
                    {
                        LQuaternionf a, b, ba;
                        a.set_from_axis_angle(-90, LVecBase3(0, 1, 0));
                        b.set_from_axis_angle(90, LVecBase3(1, 0, 0));
                        ba = b * a;
                        quat_result = ba * joint_quaternion;
                    }
                    else if (leap_mode == LEAP_MOTION_MODE_FLOOR)
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

            if (dest_amo)
            {
                dest_poses[i].MakePosition(joint_model->GetPosition(world));
                dest_poses[i].SetQuaternion(joint_model->GetQuaternion(world));
            }
        }
    }

    if (dest_amo)
    {
        dest_amo->SetAvatarMemory(dest_poses);
        dest_amo->UpdateAvatarMemoryObject();
    }
}
