#include "main.hpp"

#include <render_pipeline/rpcore/util/rpmaterial.hpp>

#include <crsf/RenderingEngine/TGraphicRenderEngine.h>
#include <crsf/CRModel/TWorld.h>

#include <crsf/CREngine/TPhysicsManager.h>

#include <crsf/CRModel/TGraphicModel.h>
#include <crsf/CRModel/TPhysicsModel.h>
#include <crsf/CRModel/TCRModel.h>
#include <crsf/CRModel/TCube.h>
#include <crsf/CRModel/TCompound.h>

void MainApp::setup_ground()
{
	auto world = rendering_engine_->GetWorld();
	
	// ground instance
	ground_ = crsf::CreateObject<crsf::TCube>("ground_", LVecBase3(0, 0, -0.02), LVecBase3(2.0, 2.0, 0.02));

	// graphic
	auto graphic_model = ground_->CreateGraphicModel();
	rpcore::RPMaterial mat(graphic_model->GetMaterial());
	mat.set_roughness(1.0f);
	mat.set_base_color(LColorf(1, 1, 1, 1));
	graphic_model->SetMaterial(mat.get_material());

	world->AddWorldObject(ground_);

	// physics
	crsf::TPhysicsModel::Parameters params;
	params.m_fMass = 0.0f;
	ground_->CreatePhysicsModel(params);
	physics_manager_->AddModel(ground_);
}

void MainApp::setup_table()
{
	auto world = rendering_engine_->GetWorld();

	// table(graphic) instance
	table_ = world->LoadModel("resources/models/Ikea_Lisabo_table/Ikea_Lisabo_table.bam");

	float init_position_x = m_property.get("object.table.init_position_x", 0.0f);
	float init_position_y = m_property.get("object.table.init_position_y", 0.0f);
	float init_position_z = m_property.get("object.table.init_position_z", 0.0f);
	table_->SetPosition(LVecBase3(init_position_x, init_position_y, init_position_z));
	float scale = m_property.get("object.table.scale", 0.0f);
	table_->SetScale(scale);

	table_->DisableTestBounding();

	// set table(physics) -> compound model
	if (m_property.get("object.table.is_compound", false))
	{
		// create compound physics model's root
		table_compound_ = crsf::CreateObject<crsf::TWorldObject>("table_compound_");
		world->AddWorldObject(table_compound_);

		float phyxInit_position_x = m_property.get("object.table.table_phyx.phyxInit_position_x", 0.0f);
		float phyxInit_position_y = m_property.get("object.table.table_phyx.phyxInit_position_y", 0.0f);
		float phyxInit_position_z = m_property.get("object.table.table_phyx.phyxInit_position_z", 0.0f);
		float phyxHalfExtent_x = m_property.get("object.table.table_phyx.phyxHalfExtent_x", 0.0f);
		float phyxHalfExtent_y = m_property.get("object.table.table_phyx.phyxHalfExtent_y", 0.0f);
		float phyxHalfExtent_z = m_property.get("object.table.table_phyx.phyxHalfExtent_z", 0.0f);
		float legHalfExtent_x = 0.02f;
		float legHalfExtent_y = 0.02f;
		float legHalfExtent_z = (phyxInit_position_z - phyxHalfExtent_z - init_position_z) / 2.0f;

		// set children
		// table board
		{
			auto table_board = crsf::CreateObject<crsf::TCube>("table_board",
				LVecBase3(phyxInit_position_x, phyxInit_position_y, phyxInit_position_z),
				LVecBase3(phyxHalfExtent_x, phyxHalfExtent_y, phyxHalfExtent_z));

			table_board->CreateGraphicModel();
			table_compound_->AddWorldObject(table_board);
		}

		// 4 table legs
		for (int i = 0; i < 4; i++)
		{
			LVecBase3 pos;
			switch (i)
			{
			case 0:
				pos = LVecBase3(phyxInit_position_x + phyxHalfExtent_x - legHalfExtent_x,
					phyxInit_position_y - phyxHalfExtent_y + legHalfExtent_y,
					legHalfExtent_z + init_position_z);
				break;
			case 1:
				pos = LVecBase3(phyxInit_position_x + phyxHalfExtent_x - legHalfExtent_x,
					phyxInit_position_y + phyxHalfExtent_y - legHalfExtent_y,
					legHalfExtent_z + init_position_z);
				break;
			case 2:
				pos = LVecBase3(phyxInit_position_x - phyxHalfExtent_x + legHalfExtent_x,
					phyxInit_position_y - phyxHalfExtent_y + legHalfExtent_y,
					legHalfExtent_z + init_position_z);
				break;
			case 3:
				pos = LVecBase3(phyxInit_position_x - phyxHalfExtent_x + legHalfExtent_x,
					phyxInit_position_y + phyxHalfExtent_y - legHalfExtent_y,
					legHalfExtent_z + init_position_z);
				break;
			}
			auto table_leg = crsf::CreateObject<crsf::TCube>("table_leg_" + std::to_string(i),
				pos, LVecBase3(legHalfExtent_x, legHalfExtent_y, legHalfExtent_z));

			table_leg->CreateGraphicModel();
			table_compound_->AddWorldObject(table_leg);
		}

		// create compound model instance
		auto compound = crsf::CreateObject<crsf::TCompound>("table_compound_model_", table_, table_compound_.get(), crsf::ECOMPOUND_MODEL);

		// create compound physics model
		crsf::TPhysicsModel::Parameters params;
		params.m_fMass = 10.0f;
		params.m_fFriction = 10.0f;
		compound->CreatePhysicsModel(params);
		physics_manager_->AddModel(compound);
	}
	else // set table(physics) -> cube, only board, no legs
	{
		// table(physics) instance
		float init_position_x = m_property.get("object.table.table_phyx.phyxInit_position_x", 0.0f);
		float init_position_y = m_property.get("object.table.table_phyx.phyxInit_position_y", 0.0f);
		float init_position_z = m_property.get("object.table.table_phyx.phyxInit_position_z", 0.0f);
		float phyxHalfExtent_x = m_property.get("object.table.table_phyx.phyxHalfExtent_x", 0.0f);
		float phyxHalfExtent_y = m_property.get("object.table.table_phyx.phyxHalfExtent_y", 0.0f);
		float phyxHalfExtent_z = m_property.get("object.table.table_phyx.phyxHalfExtent_z", 0.0f);
		table_physics_ = crsf::CreateObject<crsf::TCube>("table_physics_", LVecBase3(init_position_x, init_position_y, init_position_z), LVecBase3(phyxHalfExtent_x, phyxHalfExtent_y, phyxHalfExtent_z));

		// graphic
		auto table_cube_graphic = table_physics_->CreateGraphicModel();
		
		world->AddWorldObject(table_physics_);
		table_physics_->Hide();

		// physics
		crsf::TPhysicsModel::Parameters params;
		params.m_fMass = 0.0f;
		params.m_fFriction = 10.0f;
		auto physics_model = table_physics_->CreatePhysicsModel(params);
		physics_manager_->AddModel(table_physics_);
	}
}