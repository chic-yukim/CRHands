#include "main.hpp"

#include <spdlog/spdlog.h>

#include "hand/hand_manager.hpp"
#include "object/jewelry.hpp"

#include <crsf/CREngine/TDynamicModuleManager.h>

#include <render_pipeline/rpcore/render_pipeline.hpp>

#include <crsf/CoexistenceInterface/TDynamicStageMemory.h>
#include <crsf/RenderingEngine/TGraphicRenderEngine.h>
#include <crsf/CRModel/TWorld.h>
#include <crsf/CRModel/TCRHand.h>
#include <crsf/System/TCRProperty.h>

#include <crsf/CREngine/TPhysicsManager.h>

#include "hand/hand.hpp"
#include "main_gui/main_gui.hpp"
#include "local_user.hpp"

CRSEEDLIB_MODULE_CREATOR(MainApp);

spdlog::logger* global_logger = nullptr;

//////////////////////////////////////////////////////////////////////////
MainApp::MainApp(void) : crsf::TDynamicModuleInterface(CRMODULE_ID_STRING)
{
    global_logger = m_logger.get();

    rendering_engine_ = crsf::TGraphicRenderEngine::GetInstance();
    pipeline_ = rendering_engine_->GetRenderPipeline();
    dsm_ = crsf::TDynamicStageMemory::GetInstance();

	setup_physics();
}

MainApp::~MainApp() = default;

void MainApp::OnLoad(void)
{
	rendering_engine_->SetWindowTitle(CRMODULE_ID_STRING);
	/*if (!crsf::TDynamicModuleManager::GetInstance()->IsModuleEnabled("openvr"))
	{
		rendering_engine_->EnableControl();
		rendering_engine_->SetControllerInitialPosHpr(
			LVecBase3(0.0f, -2.2f, 1.0f),
			LVecBase3(0.0f, -25.0f, 0.0f));
		rendering_engine_->ResetControllerInitial();
	}*/

	rendering_engine_->EnableControl();
	rendering_engine_->SetControllerInitialPosHpr(
		LVecBase3(0.0f, -2.2f, 1.0f),
		LVecBase3(0.0f, -25.0f, 0.0f));
	rendering_engine_->ResetControllerInitial();
}

void MainApp::OnStart(void)
{
	srand((unsigned int)time(NULL));

    user_ = std::make_unique<LocalUser>();

	setup_event();
	setup_hand();
	setup_scene();

    main_gui_ = std::make_unique<MainGUI>(*this);

	/*do_method_later(1.0f, [this](rppanda::FunctionalTask* task) {
		physics_manager_->Start();
		return AsyncTask::DoneStatus::DS_done;
	}, "MainApp::start_physics");*/
}

void MainApp::OnExit(void)
{
    main_gui_.reset();

	remove_all_tasks();

	physics_manager_->Exit();

	hand_manager_.reset();

    user_.reset();
}

void MainApp::setup_event()
{
	accept("f1", [this](const Event*) {
		rendering_engine_->GetRenderNode()->GetNodePath().ls();
	});

	accept("0", [this](const Event*) {
		physics_manager_->Start();
	});

	accept("9", [this](const Event*) {
        // reset cubes position
        reset_cubes_position();

		jewelry_->SetPosition(LVecBase3(0, 0, 1), rendering_engine_->GetWorld());

		auto sub_group_box_0 = dynamic_cast<crsf::TCRModel*>(jewelry_->GetChild(0)->GetChild(0));
		auto sub_group_box_1 = dynamic_cast<crsf::TCRModel*>(jewelry_->GetChild(1)->GetChild(0));
		sub_group_box_0->SetPosition(0, 0, -0.0095);
		sub_group_box_1->SetPosition(0, 0, 0.0095);
		physics_manager_->SetLinearVelocity(sub_group_box_0, LVecBase3(0));
		physics_manager_->SetAngularVelocity(sub_group_box_0, LVecBase3(0));
		physics_manager_->SetLinearVelocity(sub_group_box_1, LVecBase3(0));
		physics_manager_->SetAngularVelocity(sub_group_box_1, LVecBase3(0));

		jewelry_->set_hinge_rotation(45);
	});
}

void MainApp::setup_physics()
{
	physics_manager_ = crsf::TPhysicsManager::GetInstance();
	physics_manager_->Init(crsf::EPHYX_ENGINE_BULLET);
	physics_manager_->SetGravity(LVecBase3(0.0f, 0.0f, -0.98f));
	physics_manager_->SetInternalStep_FPS(60, true, false);
}

void MainApp::setup_hand()
{
    hand_manager_ = std::make_unique<HandManager>(*this, m_property);
    hand_manager_->setup_hand(user_.get());
}

void MainApp::setup_scene()
{
	if (m_property.get("object.create.ground", false))
		setup_ground();
	if (m_property.get("object.create.table", false))
		setup_table();
	if (m_property.get("object.create.cubes", false))
		setup_cubes();
	if (m_property.get("object.create.jewelry", false))
		setup_jewelry();
	if (m_property.get("object.create.twisty_puzzle", false))
		setup_twisty_puzzle();
}