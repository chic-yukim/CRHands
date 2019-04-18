/**
* Coexistence Reality Software Framework (CRSF)
* Copyright (c) Center of Human-centered Interaction for Coexistence. All rights reserved.
* See the LICENSE.md file for more details.
*/

#pragma once

#include <memory>

namespace crsf {
class TCRHand;
class TWorldObject;
class TCharacter;
class TCRProperty;
class THandInteractionEngineConnector;
class TCRModel;
}

class Hand
{
public:
    Hand(const crsf::TCRProperty& props, crsf::TWorldObject* hand_model);
    ~Hand();

    crsf::TCRHand* get_hand() const;
    crsf::TWorldObject* get_object() const;
    crsf::TCharacter* get_character() const;
    crsf::THandInteractionEngineConnector* get_hand_connector() const;

    void setup_physics_interactor(float particle_radius = 0.0025f);

private:
    bool interactor_collision_event(const std::shared_ptr<crsf::TCRModel>& my_model, const std::shared_ptr<crsf::TCRModel>& evented_model);

    std::unique_ptr<crsf::TCRHand> hand_;
    crsf::TWorldObject* hand_object_ = nullptr;
    std::unique_ptr<crsf::THandInteractionEngineConnector> hand_connector_;
};

inline crsf::TCRHand* Hand::get_hand() const
{
    return hand_.get();
}

inline crsf::TWorldObject* Hand::get_object() const
{
    return hand_object_;
}

inline crsf::THandInteractionEngineConnector* Hand::get_hand_connector() const
{
    return hand_connector_.get();
}
