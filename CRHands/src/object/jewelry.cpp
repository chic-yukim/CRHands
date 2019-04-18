#include "jewelry.hpp"

#include "main.hpp"

#include "hand/hand_manager.hpp"

#include <render_pipeline/rpcore/util/rpmaterial.hpp>

#include <crsf/RenderingEngine/TGraphicRenderEngine.h>
#include <crsf/CRModel/TWorld.h>

#include <crsf/CREngine/TPhysicsManager.h>

#include <crsf/CRModel/TGraphicModel.h>
#include <crsf/CRModel/TPhysicsModel.h>
#include <crsf/CRModel/TCRModel.h>
#include <crsf/CRModel/TCube.h>
#include <crsf/CRModel/TCompound.h>
#include <crsf/CRModel/TImportedModel.h>
#include <crsf/CRModel/TGroupedObjects.h>

#include <crsf/Utility/TOpenFrameworksMath.h>

void MainApp::setup_jewelry()
{
	auto world = rendering_engine_->GetWorld();

	jewelry_ = std::make_shared<Jewelry>("jewerly_");
	world->AddWorldObject(jewelry_);

	jewelry_->initialize_grouped_objects(0.001, LVecBase3(0, 0, 1));

	jewelry_->set_hinge_rotation(45);

	jewelry_->attach_collision_listener_each(std::bind(&HandManager::object_collision_event, hand_manager_.get(), std::placeholders::_1, std::placeholders::_2));
	jewelry_->attach_inside_listener_each(std::bind(&HandManager::object_collision_event, hand_manager_.get(), std::placeholders::_1, std::placeholders::_2));
	jewelry_->attach_update_listener_each(std::bind(&HandManager::grouped_object_update_event_each, hand_manager_.get(), std::placeholders::_1));
	jewelry_->attach_update_listener(std::bind(&HandManager::grouped_object_update_event, hand_manager_.get(), std::placeholders::_1));
}

Jewelry::Jewelry(const std::string& name) : TGroupedObjectsBase(name)
{
	is_group_changeable = false;
	is_multi_mesh_group = false;
}

Jewelry::~Jewelry()
{

}

void Jewelry::initialize_grouped_objects(double scale, const LVecBase3& pos)
{
	auto world = crsf::TGraphicRenderEngine::GetInstance()->GetWorld();

	TGroupedObjectsBase::initialize_grouped_objects(scale, pos);

	grouped_object_data[0].object = sub_group[0];
	grouped_object_data[0].axis_on_object = LVecBase3(1, 0, 0);
	grouped_object_data[0].pivot_on_object = LVecBase3(0, -0.033, 0.0025);
	grouped_object_data[0].pivot_on_object_in_group = preMult(sub_group[0]->GetMatrix(), grouped_object_data[0].pivot_on_object);
	grouped_object_data[0].has_limit_angle = true;
	grouped_object_data[0].limit_angle[0] = 0;
	grouped_object_data[0].limit_angle[1] = 89;

	grouped_object_data[1].object = sub_group[1];
	grouped_object_data[1].axis_on_object = LVecBase3(1, 0, 0);
	grouped_object_data[1].pivot_on_object = LVecBase3(0, -0.033, 0.0025);
	grouped_object_data[1].pivot_on_object_in_group = preMult(sub_group[1]->GetMatrix(), grouped_object_data[1].pivot_on_object);
	grouped_object_data[1].has_limit_angle = true;
	grouped_object_data[1].limit_angle[0] = -89;
	grouped_object_data[1].limit_angle[1] = 0;

	{
		// load compound's graphic model
		auto compound_graphic = world->LoadModel("resources/models/jewelry/jewelry_bottom.egg");
		compound_graphic->SetScale(0.001, world);

		// create compound's physics model
		auto compound_physics = crsf::CreateObject<crsf::TWorldObject>("jewelry_bottom_physics");
		world->AddWorldObject(compound_physics);
		auto cube = crsf::CreateObject<crsf::TCube>("jewelry_bottom_physics_cube", LVecBase3(0), LVecBase3(0.03, 0.032, 0.01));
		cube->CreateGraphicModel();
		compound_physics->AddWorldObject(cube);
		cube->SetPosition(LVecBase3(0));

		// create compound
		auto compound = crsf::CreateObject<crsf::TCompound>("jewelry_bottom_compounds",
			compound_graphic, compound_physics.get(), crsf::ECOMPOUND_MODEL);
		world->AddWorldObject(compound);

		// create physics model of compound
		crsf::TPhysicsModel::Parameters params;
		params.m_fMass = sub_group[0]->mass;
		params.m_fFriction = sub_group[0]->friction;
		params.m_bHandInteractable = sub_group[0]->hand_interactive;
		compound->CreatePhysicsModel(params);
		crsf::TPhysicsManager::GetInstance()->AddModel(compound);

		sub_group[0]->AddWorldObject(compound);
		compound->SetPosition(LVecBase3(0, 0, -0.0095));
	}

	{
		// load compound's graphic model
		auto compound_graphic = world->LoadModel("resources/models/jewelry/jewelry_top.egg");
		compound_graphic->SetScale(0.001, world);

		// create compound's physics model
		auto compound_physics = crsf::CreateObject<crsf::TWorldObject>("jewelry_top_physics");
		world->AddWorldObject(compound_physics);
		auto cube = crsf::CreateObject<crsf::TCube>("jewelry_top_physics_cube", LVecBase3(0), LVecBase3(0.03, 0.032, 0.0055));
		cube->CreateGraphicModel();
		compound_physics->AddWorldObject(cube);
		cube->SetPosition(LVecBase3(0));

		// create compound
		auto compound = crsf::CreateObject<crsf::TCompound>("jewelry_top_compounds",
			compound_graphic, compound_physics.get(), crsf::ECOMPOUND_MODEL);
		world->AddWorldObject(compound);

		// create physics model of compound
		crsf::TPhysicsModel::Parameters params;
		params.m_fMass = sub_group[1]->mass;
		params.m_fFriction = sub_group[1]->friction;
		params.m_bHandInteractable = sub_group[1]->hand_interactive;
		compound->CreatePhysicsModel(params);
		crsf::TPhysicsManager::GetInstance()->AddModel(compound);

		sub_group[1]->AddWorldObject(compound);
		compound->SetPosition(LVecBase3(0, 0, 0.0095));
	}
}

void Jewelry::set_hinge_rotation(float angle)
{
	LMatrix4f matrix;

	if (angle >= 0)
	{
		matrix = calculate_rotation_matrix(&grouped_object_data[0], angle);
		sub_group[0]->SetMatrix(matrix);
		sub_group[1]->SetMatrix(LMatrix4f::ident_mat());

		object_angle = angle;
		manipulated_object = sub_group[0];
	}
	else
	{
		matrix = calculate_rotation_matrix(&grouped_object_data[1], angle);
		sub_group[0]->SetMatrix(LMatrix4f::ident_mat());
		sub_group[1]->SetMatrix(matrix);

		object_angle = angle;
		manipulated_object = sub_group[1];
	}
}