#include "main.hpp"

#include <spdlog/spdlog.h>

#include "hand/hand_manager.hpp"

#include <crsf/CREngine/TDynamicModuleManager.h>

#include <render_pipeline/rpcore/render_pipeline.hpp>

#include <crsf/RenderingEngine/TGraphicRenderEngine.h>
#include <crsf/CRModel/TWorld.h>

#include <crsf/CREngine/TPhysicsManager.h>

#include "main_gui/main_gui.hpp"

CRSEEDLIB_MODULE_CREATOR(CRHands);

spdlog::logger* global_logger = nullptr;

//////////////////////////////////////////////////////////////////////////
CRHands::CRHands(void) : crsf::TDynamicModuleInterface(CRMODULE_ID_STRING)
{
	global_logger = m_logger.get();

	setup_physics();
}

CRHands::~CRHands() = default;

void CRHands::OnLoad(void)
{
	rendering_engine_ = crsf::TGraphicRenderEngine::GetInstance();
	pipeline_ = rendering_engine_->GetRenderPipeline();

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

void CRHands::OnStart(void)
{
	srand((unsigned int)time(NULL));

	setup_event();
	setup_hand();
	setup_scene();


	std::cout << std::endl << " On Start! " << std::endl << std::endl;
	std::cout << std::endl << " On Start! " << std::endl << std::endl;

    main_gui_ = std::make_unique<MainGUI>(*this);

	/*do_method_later(1.0f, [this](rppanda::FunctionalTask* task) {
		physics_manager_->Start();
		return AsyncTask::DoneStatus::DS_done;
	}, "CRHands::start_physics");*/
}

void CRHands::OnExit(void)
{
    main_gui_.reset();

	remove_all_tasks();

	physics_manager_->Exit();

	hand_manager_.reset();
}

void CRHands::setup_event()
{
	accept("f1", [this](const Event*) {
		crsf::TGraphicRenderEngine* rendering_engine = crsf::TGraphicRenderEngine::GetInstance();
		rendering_engine->GetRenderNode()->GetNodePath().ls();
	});

	accept("0", [this](const Event*) {
		physics_manager_->Start();
	});
}

void CRHands::setup_physics()
{
	physics_manager_ = crsf::TPhysicsManager::GetInstance();
	physics_manager_->Init(crsf::EPHYX_ENGINE_BULLET);
	physics_manager_->SetGravity(LVecBase3(0.0f, 0.0f, -0.98f));
	physics_manager_->SetInternalStep_FPS(60, true, false);
}

void CRHands::setup_hand()
{
	hand_manager_ = std::make_unique<HandManager>(*this, m_property);
}

void CRHands::setup_scene()
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