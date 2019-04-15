#include "main_gui/main_gui.hpp"

#include <numeric>

#include <imgui.h>

#include <fmt/format.h>

#include <render_pipeline/rppanda/showbase/showbase.hpp>
#include <render_pipeline/rpcore/pluginbase/manager.hpp>
#include <render_pipeline/rpcore/util/primitives.hpp>
#include <render_pipeline/rpcore/render_pipeline.hpp>
#include "main.hpp"

MainGUI::MainGUI(CRHands& app) : app_(app)
{
	std::cout << std::endl << " Main GUI! " << std::endl << std::endl;
	std::cout << std::endl << " Main GUI! " << std::endl << std::endl;

    rppanda::Messenger::get_global_instance()->send(
        "imgui-setup-context",
        EventParameter(new rppanda::FunctionalTask([this](rppanda::FunctionalTask* task) {

            ImGui::SetCurrentContext(std::static_pointer_cast<ImGuiContext>(task->get_user_data()).get());
            accept("imgui-new-frame", [this](const Event*) { on_imgui_new_frame(); });
            return AsyncTask::DS_done;
        }, "MainApp::setup-imgui"))
    );

    setup_hand_mocap();
}

MainGUI::~MainGUI() = default;

void MainGUI::on_imgui_new_frame()
{
    static bool window = true;

	std::cout << std::endl << " on IMGUI! " << std::endl << std::endl;
	std::cout << std::endl << " on IMGUI! " << std::endl << std::endl;
	std::cout << std::endl << " on IMGUI! " << std::endl << std::endl;
	std::cout << std::endl << " on IMGUI! " << std::endl << std::endl;

    ImGui::Begin("CRHands", &window);

	
	static float f1 = 0.123f, f2 = 0.0f;
	ImGui::SliderFloat("slider float", &f1, 0.0f, 1.0f, "ratio = %.3f");
	//ImGui::SliderFloat("A", )

    ui_hand_mocap();

    ImGui::End();
}