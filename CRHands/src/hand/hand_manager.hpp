/**
* Coexistence Reality Software Framework (CRSF)
* Copyright (c) Center of Human-centered Interaction for Coexistence. All rights reserved.
* See the LICENSE.md file for more details.
*/

#pragma once

#include <memory>

#include <util/math.hpp>

#include <boost/property_tree/ptree.hpp>

#include <render_pipeline/rppanda/showbase/direct_object.hpp>

namespace crsf {
	class TCube;
	class TAvatarMemoryObject;
	class TCRHand;
	class TCharacter;
	class TWorldObject;
	class TCRModel;
}

class MainApp;

class OpenVRModule;
class LeapMotionInterface;
class Hand_MoCAPInterface;
class Kinesthetic_HandMoCAPInterface;

class HandManager : public rppanda::DirectObject
{
public:
	HandManager(MainApp& app, const boost::property_tree::ptree& props);
	virtual ~HandManager();

	// hand
	crsf::TCRHand* get_hand() const;
	crsf::TWorldObject* get_hand_object() const;
	crsf::TCharacter* get_hand_character() const;

	void setup_hand(void);
	void setup_hand_event(void);

	// leap
	void render_hand_leap(crsf::TAvatarMemoryObject *amo);

	// CHIC mocap
	void render_hand_mocap(crsf::TAvatarMemoryObject *amo);

	// UNIST mocap
	void render_unist_mocap(crsf::TAvatarMemoryObject *amo);
	LQuaternionf rotate_unist_to_crsf(const LQuaternionf& quat);

	// VIVE
	void get_open_vr_module_data();

	// listener
	bool interactor_collision_event(const std::shared_ptr<crsf::TCRModel>& my_model, const std::shared_ptr<crsf::TCRModel>& evented_model);

	bool object_collision_event(const std::shared_ptr<crsf::TCRModel>& my_model, const std::shared_ptr<crsf::TCRModel>& evented_model);
	bool object_separation_event(const std::shared_ptr<crsf::TCRModel>& my_model, const std::shared_ptr<crsf::TCRModel>& evented_model);
	bool object_update_event(const std::shared_ptr<crsf::TCRModel>& my_model);

	bool grouped_object_update_event_each(const std::shared_ptr<crsf::TCRModel>& my_model);
	bool grouped_object_update_event(const std::shared_ptr<crsf::TCRModel>& my_model);

private:
	MainApp& app_;

	const boost::property_tree::ptree& props_;

	const double M_PI = std::acos(-1);

	// hand model
	crsf::TCRHand* hand_ = nullptr;
	crsf::TWorldObject* hand_object_ = nullptr;
	crsf::TCharacter* hand_character_ = nullptr;
	
	// LEAP
	LeapMotionInterface* interface_leap_ = nullptr;
	std::string leap_mode_ = "HMD";

	// CHIC mocap
	Hand_MoCAPInterface* interface_hand_mocap_ = nullptr;

	float hand_model_joint_offset_[2][5][4];

	bool is_hand_mocap_calibration_ = false;

	enum e_hand_mocap_mode_
	{
		BOTH, LEFT, RIGHT
	};
	e_hand_mocap_mode_ hand_mocap_mode_;

	unsigned int last_hand_mocap_vibrations_[2];

	// UNIST mocap
	Kinesthetic_HandMoCAPInterface* interface_unist_mocap_ = nullptr;
	int unist_mocap_joint_number_ = 28;

	float hand_mocap_data_[28];

	std::string unist_mocap_mode_ = "right";

	// VIVE
	std::shared_ptr<OpenVRModule> module_open_vr_ = nullptr;

	std::shared_ptr<crsf::TWorldObject> node_hmd_ = nullptr;

	std::string left_wrist_tracker_serial_;
	std::string right_wrist_tracker_serial_;

	int tracker_index_left_ = -1;
	int tracker_index_right_ = -1;

	LVecBase3 trakcer_pos_;
	LQuaternionf tracker_quat_;
	LVecBase3 trakcers_pos_[2];
	LQuaternionf trackers_quat_[2];

	// physics particle
	float particle_radius_ = 0.0025f;

	// grasp algorithm
	std::vector<crsf::TWorldObject*> hand_pointer_;
};

inline crsf::TCRHand* HandManager::get_hand() const
{
	return hand_;
}

inline crsf::TWorldObject* HandManager::get_hand_object() const
{
	return hand_object_;
}

inline crsf::TCharacter* HandManager::get_hand_character() const
{
	return hand_character_;
}
