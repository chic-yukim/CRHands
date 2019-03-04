#include "main_gui.hpp"

#include <imgui.h>

#include <crsf/RealWorldInterface/TInterfaceManager.h>

#include "hand_mocap_interface.h"

void MainGUI::setup_hand_mocap()
{
    auto hand_mocap_interface = dynamic_cast<Hand_MoCAPInterface*>(crsf::TInterfaceManager::GetInstance()->GetInputInterface("Hand_MoCAP"));
    if (!hand_mocap_interface)
        return;

    for (auto hand_index : { Hand_MoCAPInterface::HAND_LEFT, Hand_MoCAPInterface::HAND_RIGHT })
    {
        hand_mocap_status_[hand_index].status = Hand_MoCAPInterface::Status::STATUS_DISCONNECTED;
    }

    accept(hand_mocap_interface->get_status_changed_event_name(), [this](const Event* ev) {
        hand_mocap_status_[ev->get_parameter(0).get_int_value()].status = ev->get_parameter(1).get_int_value();
    });
}

void MainGUI::ui_hand_mocap()
{
    auto hand_mocap_interface = dynamic_cast<Hand_MoCAPInterface*>(crsf::TInterfaceManager::GetInstance()->GetInputInterface("Hand_MoCAP"));
    if (!hand_mocap_interface)
        return;

    if (!ImGui::CollapsingHeader("Hand MoCAP"))
        return;

    ImGui::LabelText("Mode", hand_mocap_interface->GetMoCAPMode().c_str());

    for (auto hand_index : { Hand_MoCAPInterface::HAND_LEFT, Hand_MoCAPInterface::HAND_RIGHT })
    {
        if (!ImGui::CollapsingHeader(hand_index == Hand_MoCAPInterface::HAND_LEFT ? "Left" : "Right"))
            continue;

        ImGui::PushID(hand_index);

        bool& thumb = hand_mocap_status_[hand_index].thumb;
        bool& index = hand_mocap_status_[hand_index].index;
        bool& middle = hand_mocap_status_[hand_index].middle;

        ImGui::LabelText("Status", hand_mocap_interface->GetStatusString(Hand_MoCAPInterface::Status(hand_mocap_status_[hand_index].status)));

        ImGui::TextUnformatted("Vibration:");
        ImGui::BeginGroup();

        bool is_changed = false;
        is_changed = ImGui::Checkbox("Thumb", &thumb)   || is_changed;    ImGui::SameLine();
        is_changed = ImGui::Checkbox("Index", &index)   || is_changed;    ImGui::SameLine();
        is_changed = ImGui::Checkbox("Middle", &middle) || is_changed;

        if (ImGui::Button("All Fingers"))
        {
            thumb = index = middle = true;
            is_changed = true;
        }

        ImGui::SameLine();

        if (ImGui::Button("Stop"))
        {
            thumb = index = middle = false;
            is_changed = true;
        }

        if (is_changed)
        {
            Hand_MoCAPInterface::FingerMask mask = Hand_MoCAPInterface::FingerMask::FINGER_NONE;
            if (thumb)
                mask |= Hand_MoCAPInterface::FingerMask::FINGER_THUMB;

            if (index)
                mask |= Hand_MoCAPInterface::FingerMask::FINGER_INDEX;

            if (middle)
                mask |= Hand_MoCAPInterface::FingerMask::FINGER_MIDDLE;

            hand_mocap_interface->SetVibration(hand_index, static_cast<Hand_MoCAPInterface::FingerMask>(mask));
        }

        ImGui::EndGroup();

        ImGui::PopID();
    }
}
