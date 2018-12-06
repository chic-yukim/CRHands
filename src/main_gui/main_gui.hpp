#pragma once

#include <render_pipeline/rppanda/showbase/direct_object.hpp>

class CRHands;

class MainGUI : public rppanda::DirectObject
{
public:
    MainGUI(CRHands& app);

    virtual ~MainGUI();

private:
    void on_imgui_new_frame();

	CRHands& app_;

    std::string today_date_;
};
