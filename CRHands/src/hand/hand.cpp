#include "hand.hpp"

#include <crsf/CRModel/TCharacter.h>
#include <crsf/CRModel/TCRHand.h>
#include <crsf/CRModel/THandPhysicsInteractor.h>
#include <crsf/CRModel/TCRModel.h>
#include <crsf/System/TCRProperty.h>
#include <crsf/CREngine/THandInteractionEngineConnector.h>

Hand::Hand(const crsf::TCRProperty& props, crsf::TWorldObject* hand_model) : hand_object_(hand_model)
{
    hand_ = std::make_unique<crsf::TCRHand>(props);

    hand_connector_ = std::make_unique<crsf::THandInteractionEngineConnector>();
    hand_connector_->Init(hand_.get());
}

Hand::~Hand() = default;

crsf::TCharacter* Hand::get_character() const
{
    return hand_->Get3DModel()->GetPtrOf<crsf::TCharacter>();
}

void Hand::setup_physics_interactor(float particle_radius)
{
    hand_->ConstructPhysicsInteractor_FixedVertex_Sphere("resources/models/hands/PhysicsInteractorIndex_full_new.txt", particle_radius, false, false, "both");

    hand_->AttachPhysicsInteractor_CollisionListener([this](const std::shared_ptr<crsf::TCRModel>& my_model, const std::shared_ptr<crsf::TCRModel>& evented_model) {
        return interactor_collision_event(my_model, evented_model);
    });

    hand_->AttachPhysicsInteractor_InsideListener([this](const std::shared_ptr<crsf::TCRModel>& my_model, const std::shared_ptr<crsf::TCRModel>& evented_model) {
        return interactor_collision_event(my_model, evented_model);
    });
}

bool Hand::interactor_collision_event(const std::shared_ptr<crsf::TCRModel>& my_model, const std::shared_ptr<crsf::TCRModel>& evented_model)
{
    // if contacted model is not interactable, return
    if (!evented_model->GetPhysicsModel()->GetIsHandInteractable())
        return false;

    // get contact information
    crsf::TContactInfo* my_contact_info = my_model->GetPhysicsModel()->GetContactInfo();

    auto physics_particle = hand_->FindPhysicsInteractor(my_model);
    physics_particle->SetPenetrationDirection(my_contact_info->GetNormalWorldOnB());
    physics_particle->SetPenetrationDirection(physics_particle->GetPenetrationDirection().normalized());

    return false;
}
