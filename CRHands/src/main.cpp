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
    auto hand = user_->make_hand();
    auto crhand = hand->get_hand();

    // update hand property
    auto hand_prop = crhand->GetHandProperty();
    float leap_local_translation_x, leap_local_translation_y, leap_local_translation_z;
    // VR mode - leap local translation is 'HMD-to-LEAP'
    if (crsf::TDynamicModuleManager::GetInstance()->IsModuleEnabled("openvr"))
    {
        leap_local_translation_x = m_property.get("hand.HMD_to_LEAP_x", 0.0f);
        leap_local_translation_y = m_property.get("hand.HMD_to_LEAP_y", 0.0f);
        leap_local_translation_z = m_property.get("hand.HMD_to_LEAP_z", 0.0f);
    }
    else // mono mode - leap local translation is 'zero-to-LEAP'
    {
        leap_local_translation_x = m_property.get("hand.zero_to_LEAP_x", 0.0f);
        leap_local_translation_y = m_property.get("hand.zero_to_LEAP_y", 0.0f);
        leap_local_translation_z = m_property.get("hand.zero_to_LEAP_z", 0.0f);
    }
    hand_prop.m_vec3ZeroToSensor = LVecBase3(leap_local_translation_x, leap_local_translation_y, leap_local_translation_z);
    crhand->SetHandProperty(hand_prop);

    auto node_hmd = crsf::CreateObject<crsf::TWorldObject>("HMD");
    rendering_engine_->GetWorld()->AddWorldObject(node_hmd);
    node_hmd->AddWorldObject(hand->get_object());

	hand_manager_ = std::make_unique<HandManager>(*this, m_property);
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