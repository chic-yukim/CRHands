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

void HandManager::render_hand_mocap_local(crsf::TCRHand* hand, crsf::TAvatarMemoryObject* amo, int hand_side)
{
    auto world = crsf::TGraphicRenderEngine::GetInstance()->GetWorld();

    for (int f = 0; f < 3; f++)
    {
        for (int j = 0; j < 4; j++)
        {
            const int index = (hand_side * 12) + (f * 4) + j;
            const int model_index = 1 + (hand_side * 22) + (f * 4) + j;

            // Get TPose from avatar memory object
            const auto& getTPose = amo->GetAvatarMemory().at(index);

            // <<Rotation>>
            // Get quaternion from sensor
            LQuaternionf a = getTPose.GetQuaternion();

            // Set quaternion onto joint data
            hand->GetJointData(model_index)->SetOrientation(a);

            // Rotate to hand model origin pose
            LQuaternionf quat_result;

            if (hand_side == 0) // left
            {
                {
                    LQuaternionf aa;
                    aa.set_from_axis_angle(180, LVecBase3(1, 0, 0));
                    a = aa.conjugate() * a * aa;
                }

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
            else if (hand_side == 1) // right
            {
                LQuaternionf b, c, cb;
                b.set_from_axis_angle(90, LVecBase3(0, 1, 0));
                c.set_from_axis_angle(-90, LVecBase3(0, 0, 1));
                cb = c * b;
                quat_result = cb.conjugate() * a * cb;
            }

            if ((hand_side == 0) && f == 0 && j == 0) // Left thumb - multiply initial quaternion
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
            else if ((hand_side == 1) && f == 0 && j == 0) // Right thumb - multiply initial quaternion
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
                hand->GetJointData(model_index)->Get3DModel()->SetHPR(hpr);

            // Set rotation about ring & pinky joints same with middle joint
            if (f == 2 && j != 3)
            {
                hand->GetJointData(model_index + 4)->Get3DModel()->SetHPR(hpr);
                hand->GetJointData(model_index + 8)->Get3DModel()->SetHPR(hpr);
            }

            // <<Position>>
            // Get position from sensor (local coordinate from root(=wrist))
            LVecBase3 temp_pos = getTPose.GetPosition();
            temp_pos *= 0.001f;
            if (j != 3)
                hand->GetJointData(model_index)->SetPosition(temp_pos); // Save the local position

            if (props_.get("subsystem.handmocap_position", false))
            {
                // Rotate to hand model origin pose
                LQuaternionf root_quat;
                if (hand_side == 0)
                    root_quat = hand->GetJointData(21)->GetOrientation();
                else if (hand_side == 1)
                    root_quat = hand->GetJointData(43)->GetOrientation();
                temp_pos = rotate_pos_by_quat(temp_pos, root_quat);

                // Set local position to world position
                LVecBase3 root_pos;
                if (hand_side == 0)
                    root_pos = hand->GetJointData(21)->Get3DModel()->GetPosition(world);
                else if (hand_side == 1)
                    root_pos = hand->GetJointData(43)->Get3DModel()->GetPosition(world);
                LVecBase3 new_pos;
                new_pos = root_pos + temp_pos;

                // Set position on 3d model
                if (j != 3)
                    hand->GetJointData(model_index)->Get3DModel()->SetPosition(new_pos, world);
            }
        }
    }

    if (props_.get("subsystem.handmocap_scale", false))
    {
        // Set sensor offset
        if (hand_side == 0)
        {
            // Left palm
            const int i = 21;

            // offset
            {
                float dist = hand->GetJointData(9)->GetPosition().length(); // middle finger_1
                hand->GetJointData(i)->SetSensorOffset(dist);
            }
        }
        else if (hand_side == 1)
        {
            // Right palm
            const int i = 43;

            // offset
            {
                float dist = hand->GetJointData(31)->GetPosition().length(); // middle finger_1
                hand->GetJointData(i)->SetSensorOffset(dist);
            }

            // width
            {
                LVecBase3 middle_1 = hand->GetJointData(31)->GetPosition();
                LVecBase3 thumb_1 = hand->GetJointData(23)->GetPosition();
                float dist = dist_two_vector(middle_1, thumb_1);
                hand->GetJointData(i)->SetSensorWidth(dist);
            }

            // thickness
            {
                float dist = hand->GetJointData(23)->GetPosition().length();
                hand->GetJointData(i)->SetSensorThickness(dist);
            }
        }

        // Finger bone
        for (int f = 0; f < 3; f++)
        {
            for (int j = 0; j < 3; j++)
            {
                const int index = (hand_side * 12) + (f * 4) + j;
                const int model_index = 1 + (hand_side * 22) + (f * 4) + j;

                // offset
                if (j != 2)
                {
                    const auto& pos_cur = hand->GetJointData(model_index)->GetPosition();
                    const auto& pos_next = hand->GetJointData(model_index + 1)->GetPosition();
                    float dist = (pos_cur - pos_next).length();
                    hand->GetJointData(model_index)->SetSensorOffset(dist);
                }
                else
                {
                    const auto& pos_cur = hand->GetJointData(model_index)->GetPosition();
                    const auto& pos_next = amo->GetAvatarMemory().at(index + 1).GetPosition() * 0.001f;
                    float dist = (pos_cur - pos_next).length();
                    hand->GetJointData(model_index)->SetSensorOffset(dist);
                }
            }
        }

        // Do auto-scaling by offset ratio on mechanism
        // Left palm
        if (hand_side == 0)
        {
            int i = 21;

            LVecBase3 origin_scale = hand->GetJointData(i)->Get3DModelStandardPoseScale();

            float modified_offset_ratio = hand->GetJointData(i)->GetScalingRatio_Offset();
            float modified_width_ratio = hand->GetJointData(i)->GetScalingRatio_Width();
            float modified_thickness_ratio = hand->GetJointData(i)->GetScalingRatio_Thickness();

            LVecBase3 modified_scale = LVecBase3(origin_scale[0] * modified_offset_ratio,
                origin_scale[1] * modified_width_ratio,
                origin_scale[2] * modified_thickness_ratio);
            hand->GetJointData(i)->Get3DModel()->SetScale(modified_scale);

            int thumb_start = 1;
            for (int j = 0; j < 5; j++)
            {
                int cur_index = thumb_start + (j * 4);
                // hierarchy tree scaling (local scale balancing)
                LVecBase3 nextJoint_scale = hand->GetJointData(cur_index)->Get3DModelStandardPoseScale();
                nextJoint_scale[0] *= 1.0f / modified_offset_ratio;
                hand->GetJointData(cur_index)->Get3DModel()->SetScale(nextJoint_scale);
            }
        }
        else if (hand_side == 1)    // Right palm
        {
            int i = 43;

            LVecBase3 origin_scale = hand->GetJointData(i)->Get3DModelStandardPoseScale();

            float modified_offset_ratio = hand->GetJointData(i)->GetScalingRatio_Offset();
            float modified_width_ratio = hand->GetJointData(i)->GetScalingRatio_Width();
            float modified_thickness_ratio = hand->GetJointData(i)->GetScalingRatio_Thickness();

            LVecBase3 modified_scale = LVecBase3(origin_scale[0] * modified_offset_ratio,
                origin_scale[1],
                origin_scale[2]);
            hand->GetJointData(i)->Get3DModel()->SetScale(modified_scale);

            int thumb_start = 23;
            for (int f = 0; f < 5; f++)
            {
                int cur_index = thumb_start + (f * 4);
                // hierarchy tree scaling (local scale balancing)
                LVecBase3 nextJoint_scale = hand->GetJointData(cur_index)->Get3DModelStandardPoseScale();
                nextJoint_scale[0] *= 1.0f / modified_offset_ratio;
                hand->GetJointData(cur_index)->Get3DModel()->SetScale(nextJoint_scale);
            }
        }

        // Finger bone
        for (int f = 0; f < 3; f++)
        {
            for (int j = 0; j < 3; j++)
            {
                const int i = 1 + (hand_side * 22) + (f * 4) + j;

                LVecBase3 origin_scale = hand->GetJointData(i)->Get3DModel()->GetScale();

                float modified_offset_ratio = hand->GetJointData(i)->GetScalingRatio_Offset();

                LVecBase3 modified_scale = LVecBase3(origin_scale[0] * modified_offset_ratio,
                    origin_scale[1],
                    origin_scale[2]);
                hand->GetJointData(i)->Get3DModel()->SetScale(modified_scale);

                // hierarchy tree scaling (local scale balancing)
                if (j != 2)
                {
                    LVecBase3 nextJoint_scale = hand->GetJointData(i + 1)->Get3DModelStandardPoseScale();
                    nextJoint_scale[0] *= 1.0f / modified_offset_ratio;
                    hand->GetJointData(i + 1)->Get3DModel()->SetScale(nextJoint_scale);

                    if (f == 2)
                    {
                        hand->GetJointData(i + 1 + 4)->Get3DModel()->SetScale(nextJoint_scale);
                        hand->GetJointData(i + 1 + 8)->Get3DModel()->SetScale(nextJoint_scale);
                    }
                }
            }
        }
    }
}

void HandManager::render_hand_mocap(crsf::TAvatarMemoryObject* amo)
{
	crsf::TWorld* virtual_world = crsf::TGraphicRenderEngine::GetInstance()->GetWorld();

    int tracker_index[HAND_INDEX_COUNT];

    tracker_index[HAND_INDEX_LEFT] = tracker_indices_[HAND_INDEX_LEFT];
    tracker_index[HAND_INDEX_RIGHT] = tracker_indices_[HAND_INDEX_RIGHT];

    if (module_open_vr_)
    {
        if ((hand_mocap_mode_ & HAND_MOCAP_MODE_LEFT) == 0)
        {
            hand_->GetJointData(21)->Get3DModel()->SetPosition(LVecBase3(100), virtual_world);
            hand_->GetJointData(21)->SetPosition(LVecBase3(100));
        }
        else
        {
            auto trakcer_pos = module_open_vr_->GetDevicePosition(tracker_index[HAND_INDEX_LEFT]);
            auto tracker_quat = module_open_vr_->GetDeviceOrientation(tracker_index[HAND_INDEX_LEFT]);

            LQuaternionf a, b, ba;
            a.set_from_axis_angle(90, LVecBase3(0, 0, 1));
            b.set_from_axis_angle(90, LVecBase3(0, 1, 0));
            ba = b * a;
            tracker_quat = ba * tracker_quat;

            hand_->GetJointData(21)->Get3DModel()->SetPosition(trakcer_pos, virtual_world);
            hand_->GetJointData(21)->SetPosition(trakcer_pos);
            hand_->GetJointData(21)->Get3DModel()->SetHPR(tracker_quat.get_hpr(), virtual_world);
            LQuaternionf c, d, dc;
            c.set_from_axis_angle(90, LVecBase3(0, 0, 1));
            d.set_from_axis_angle(90, LVecBase3(1, 0, 0));
            dc = d * c;
            tracker_quat = dc * tracker_quat;
            hand_->GetJointData(21)->SetOrientation(tracker_quat);
        }

        if ((hand_mocap_mode_ & HAND_MOCAP_MODE_RIGHT) == 0)
        {
            hand_->GetJointData(43)->Get3DModel()->SetPosition(LVecBase3(100), virtual_world);
            hand_->GetJointData(43)->SetPosition(LVecBase3(100));
        }
        else
        {
            auto trakcer_pos = module_open_vr_->GetDevicePosition(tracker_index[HAND_INDEX_RIGHT]);
            auto tracker_quat = module_open_vr_->GetDeviceOrientation(tracker_index[HAND_INDEX_RIGHT]);

            LQuaternionf a, b, c, cba;
            a.set_from_axis_angle(90, LVecBase3(0, 0, 1));
            b.set_from_axis_angle(180, LVecBase3(1, 0, 0));
            c.set_from_axis_angle(-90, LVecBase3(0, 1, 0));
            cba = c * b * a;
            tracker_quat = cba * tracker_quat;

            hand_->GetJointData(43)->Get3DModel()->SetPosition(trakcer_pos, virtual_world);
            hand_->GetJointData(43)->SetPosition(trakcer_pos);
            hand_->GetJointData(43)->Get3DModel()->SetHPR(tracker_quat.get_hpr(), virtual_world);
            LQuaternionf d, e, ed;
            d.set_from_axis_angle(-90, LVecBase3(0, 0, 1));
            e.set_from_axis_angle(-90, LVecBase3(1, 0, 0));
            ed = e * d;
            tracker_quat = ed * tracker_quat;
            hand_->GetJointData(43)->SetOrientation(tracker_quat);
        }
    }

    // if calibration process is finished,
    if (is_hand_mocap_calibration_)
    {
        if ((hand_mocap_mode_ & HAND_MOCAP_MODE_LEFT) != 0)
            render_hand_mocap_local(hand_, amo, 0);

        if ((hand_mocap_mode_ & HAND_MOCAP_MODE_RIGHT) != 0)
            render_hand_mocap_local(hand_, amo, 1);
    }
}
