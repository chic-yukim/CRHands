#include "hand.hpp"

#include <crsf/CoexistenceInterface/TAvatarMemoryObject.h>
#include <crsf/CRModel/TCharacter.h>
#include <crsf/CRModel/TCRHand.h>
#include <crsf/CRModel/THandPhysicsInteractor.h>
#include <crsf/CRModel/TCRModel.h>
#include <crsf/CRModel/TWorld.h>
#include <crsf/System/TCRProperty.h>
#include <crsf/CREngine/THandInteractionEngineConnector.h>
#include <crsf/RenderingEngine/TGraphicRenderEngine.h>

Hand::Hand(const crsf::TCRProperty& props, crsf::TWorldObject* hand_model) : hand_object_(hand_model)
{
    hand_ = std::make_unique<crsf::TCRHand>(props);

    hand_connector_ = std::make_unique<crsf::THandInteractionEngineConnector>();
    hand_connector_->Init(hand_.get());
}

Hand::~Hand()
{
    hand_connector_.reset();
}

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

void Hand::set_avatar_memory_object(crsf::TAvatarMemoryObject* amo)
{
    if (amo && hand_->GetJointNumber() != amo->GetProperty().m_propAvatar.m_nJointNumber)
        return;

    hand_amo_ = amo;
}

void Hand::set_render_method(crsf::TAvatarMemoryObject* source_amo, const RenderMethodType& render_method)
{
    render_method_ = render_method;
    hand_connector_->ConnectHand([this](crsf::TAvatarMemoryObject* amo) {
        render_method_(this, amo);
    }, source_amo);
}

// ************************************************************************************************

void render_hand(Hand* hand, crsf::TAvatarMemoryObject* amo)
{
    if (!hand)
        return;

    auto crhand = hand->get_hand();

    if (!(crhand->GetHandProperty().m_bRender3DModel && crhand->GetHandProperty().m_p3DModel))
        return;

    auto world = crsf::TGraphicRenderEngine::GetInstance()->GetWorld();

    // # joint
    const unsigned int joint_number = crhand->GetJointNumber();

    // loop left joint
    const auto left_joint_number = (std::min)(joint_number, 22u);
    for (unsigned int i = 1; i < left_joint_number; ++i)
    {
        // update 3D model's pose
        crsf::TWorldObject* joint_model = crhand->GetJointData(i)->Get3DModel();
        if (joint_model)
        {
            const auto& get_avatar_pose = amo->GetAvatarMemory(i);

            joint_model->SetHPR(get_avatar_pose.GetQuaternion().get_hpr(), world);
            if (i == 21) // root(wrist)
                joint_model->SetPosition(get_avatar_pose.GetPosition(), world);
        }
    }

    // loop right joint
    const auto right_joint_number = (std::min)(joint_number, 44u);
    for (unsigned int i = 23; i < right_joint_number; ++i)
    {
        // update 3D model's pose
        crsf::TWorldObject* joint_model = crhand->GetJointData(i)->Get3DModel();
        if (joint_model)
        {
            const auto& get_avatar_pose = amo->GetAvatarMemory(i);

            joint_model->SetHPR(get_avatar_pose.GetQuaternion().get_hpr(), world);
            if (i == 43) // root(wrist)
                joint_model->SetPosition(get_avatar_pose.GetPosition(), world);
        }
    }
}
