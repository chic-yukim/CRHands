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
        hand_mocap_status_[hand_index] = Hand_MoCAPInterface::Status::STATUS_DISCONNECTED;
    }

    accept(hand_mocap_interface->get_status_changed_event_name(), [this](const Event* ev) {
        hand_mocap_status_[ev->get_parameter(0).get_int_value()] = ev->get_parameter(1).get_int_value();
    });
}

void MainGUI::ui_hand_mocap()
{
    auto hand_mocap_interface = dynamic_cast<Hand_MoCAPInterface*>(crsf::TInterfaceManager::GetInstance()->GetInputInterface("Hand_MoCAP"));
    if (!hand_mocap_interface)
        return;

    if (!ImGui::CollapsingHeader("Hand MoCAP"))
        return;

    for (auto hand_index : { Hand_MoCAPInterface::HAND_LEFT, Hand_MoCAPInterface::HAND_RIGHT })
    {
        if (!ImGui::CollapsingHeader(hand_index == Hand_MoCAPInterface::HAND_LEFT ? "Left" : "Right"))
            continue;

        ImGui::LabelText("Status", hand_mocap_interface->get_status_string(Hand_MoCAPInterface::Status(hand_mocap_status_[hand_index])));
    }
}
