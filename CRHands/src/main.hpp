#pragma once

#include <render_pipeline/rppanda/showbase/direct_object.hpp>
#include <crsf/CRAPI/TDynamicModuleInterface.h>

namespace rpcore {
	class RenderPipeline;
}

namespace crsf {
	class TGraphicRenderEngine;
	class TPhysicsManager;
	class TCube;
	class TWorldObject;
}

class HandManager;
class Jewelry;

class CRHands: public crsf::TDynamicModuleInterface, public rppanda::DirectObject
{
public:
	CRHands();
	virtual ~CRHands();

	void OnLoad(void) override;
	void OnStart(void) override;
	void OnExit(void) override;

	void setup_event();
	void setup_physics();
	void setup_hand();

	void setup_scene();
	void setup_ground();
	void setup_table();
	void setup_cubes();
	void setup_jewelry();
	void setup_twisty_puzzle();

	void reset_cubes_position();

private:
    friend class MainGUI;

	crsf::TGraphicRenderEngine* rendering_engine_;
	rpcore::RenderPipeline* pipeline_;

    std::unique_ptr<MainGUI> main_gui_;

	crsf::TPhysicsManager* physics_manager_;

	friend class HandManager;
	std::unique_ptr<HandManager> hand_manager_;

	// [OBJECTS]
	// base object
	std::shared_ptr<crsf::TCube> ground_ = nullptr;
	crsf::TWorldObject* table_ = nullptr;
	std::shared_ptr<crsf::TCube> table_physics_ = nullptr;
	std::shared_ptr<crsf::TWorldObject> table_compound_ = nullptr;

	// soma cube
	std::vector<std::shared_ptr<crsf::TCube>> cubes_;

	// jewelry
	std::shared_ptr<Jewelry> jewelry_ = nullptr;
};
