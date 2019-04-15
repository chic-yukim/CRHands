#pragma once

#include <render_pipeline/rppanda/showbase/direct_object.hpp>

class CRHands;

class MainGUI : public rppanda::DirectObject
{
public:
    MainGUI(CRHands& app);

    virtual ~MainGUI();

    void setup_hand_mocap();
    void ui_hand_mocap();
	void ui_unist_mocap();

private:
    void on_imgui_new_frame();

    CRHands& app_;

    struct HandMocapStatus
    {
        int status = 0;
        bool thumb = false;
        bool index = false;
        bool middle = false;
    };
    HandMocapStatus hand_mocap_status_[2];
};
