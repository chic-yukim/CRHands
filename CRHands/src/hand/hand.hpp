/**
* Coexistence Reality Software Framework (CRSF)
* Copyright (c) Center of Human-centered Interaction for Coexistence. All rights reserved.
* See the LICENSE.md file for more details.
*/

#pragma once

#include <memory>
#include <functional>

namespace crsf {
class TCRHand;
class TWorldObject;
class TCharacter;
class TCRProperty;
class THandInteractionEngineConnector;
class TCRModel;
class TAvatarMemoryObject;
}

class Hand
{
public:
    // void(target_hand, source_amo, dest_amo)
    using RenderMethodType = std::function<void(Hand*, crsf::TAvatarMemoryObject*)>;

public:
    Hand(const crsf::TCRProperty& props, crsf::TWorldObject* hand_model);
    ~Hand();

    crsf::TCRHand* get_hand() const;
    crsf::TWorldObject* get_object() const;
    crsf::TCharacter* get_character() const;
    crsf::THandInteractionEngineConnector* get_hand_connector() const;

    void setup_physics_interactor(float particle_radius = 0.0025f);

    crsf::TAvatarMemoryObject* get_avatar_memory_object() const;
    void set_avatar_memory_object(crsf::TAvatarMemoryObject* amo);

    void set_render_method(crsf::TAvatarMemoryObject* source_amo, const RenderMethodType& render_method);

private:
    bool interactor_collision_event(const std::shared_ptr<crsf::TCRModel>& my_model, const std::shared_ptr<crsf::TCRModel>& evented_model);

    std::unique_ptr<crsf::TCRHand> hand_;
    crsf::TWorldObject* hand_object_ = nullptr;
    std::unique_ptr<crsf::THandInteractionEngineConnector> hand_connector_;

    RenderMethodType render_method_;

    crsf::TAvatarMemoryObject* hand_amo_ = nullptr;
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

inline crsf::TAvatarMemoryObject* Hand::get_avatar_memory_object() const
{
    return hand_amo_;
}
