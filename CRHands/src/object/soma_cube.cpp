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

void MainApp::setup_cubes()
{
	auto world = rendering_engine_->GetWorld();

	float init_position_x = m_property.get("object.cubes.init_position_x", 0.0f);
	float init_position_y = m_property.get("object.cubes.init_position_y", 0.0f);
	float init_position_z = m_property.get("object.cubes.init_position_z", 0.0f);
	float halfExtent = m_property.get("object.cubes.halfExtent", 0.0f);
	float count = m_property.get("object.cubes.count", 1);

	for (int i = 0; i < count; i++)
	{
		// cube instance
		crsf::TCube::Parameters params;
		params.m_strName = "cubes_" + std::to_string(i);
		params.m_vec3Origin = LVecBase3(init_position_x + (i / 12) * 0.2, init_position_y, init_position_z + (i % 12) * 0.1);
		params.m_vec3HalfExtent = LVecBase3(halfExtent);
		auto cube = crsf::CreateObject<crsf::TCube>(params);
		cube->SetHeight(i);

		cube->DisableTestBounding();

		// graphic
		auto graphic_model = cube->CreateGraphicModel();
		rpcore::RPMaterial mat(graphic_model->GetMaterial());
		mat.set_roughness(1.0f);
		mat.set_base_color(LColorf((rand() % 100 + 1) * 0.01f, (rand() % 100 + 1) * 0.01f, (rand() % 100 + 1) * 0.01f, 1));
		graphic_model->SetMaterial(mat.get_material());

		world->AddWorldObject(cube);

		// physics
		crsf::TPhysicsModel::Parameters phyx_params;
		phyx_params.m_fMass = 10.0f;
		phyx_params.m_fFriction = 10.0f;
		phyx_params.m_bHandInteractable = true;
		auto physics_model = cube->CreatePhysicsModel(phyx_params);
		physics_manager_->AddModel(cube);

		// add listener
		physics_model->AttachCollisionListener(std::bind(&HandManager::object_collision_event, hand_manager_.get(), std::placeholders::_1, std::placeholders::_2), "soma_cube_collision_" + std::to_string(i));
		physics_model->AttachSeparationListener(std::bind(&HandManager::object_separation_event, hand_manager_.get(), std::placeholders::_1, std::placeholders::_2), "soma_cube_separation_" + std::to_string(i));
		physics_model->AttachUpdateListener(std::bind(&HandManager::object_update_event, hand_manager_.get(), std::placeholders::_1), "soma_cube_update_" + std::to_string(i));

		//
		cubes_.push_back(cube);
	}
}

void MainApp::reset_cubes_position()
{
	float count = m_property.get("object.cubes.count", 1);

	for (int i = 0; i < count; i++)
	{
		if (!cubes_.empty())
		{
			cubes_[i]->Reset_to_Origin();
		}
	}
}