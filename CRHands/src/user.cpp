#include "user.hpp"

#include <crsf/CRModel/TWorld.h>
#include <crsf/CRModel/TCharacter.h>
#include <crsf/CRModel/TCRHand.h>
#include <crsf/RenderingEngine/TGraphicRenderEngine.h>
#include <crsf/System/TCRProperty.h>

#include "hand/hand.hpp"

User::User(unsigned int system_index) : system_index_(system_index)
{
    auto rendering_engine = crsf::TGraphicRenderEngine::GetInstance();
    auto world = rendering_engine->GetWorld();

    auto user = crsf::CreateObject<crsf::TWorldObject>("User" + std::to_string(system_index_));
    root_ = user.get();
    world->AddWorldObject(user);
}

User::~User()
{
    stop_voice();

    root_->DetachWorldObject();
}

void User::set_avatar(crsf::TWorldObject* model)
{
    if (avatar_)
        avatar_->Hide();
    avatar_ = model;
}

void User::set_voice(crsf::TSoundMemoryObject* voice_mo)
{
    voice_mo_ = voice_mo;
}

Hand* User::make_hand()
{
    auto rendering_engine = crsf::TGraphicRenderEngine::GetInstance();
    auto world = rendering_engine->GetWorld();

    // set hand property
    crsf::TCRProperty hand_property;
    hand_property.m_propAvatar.SetJointNumber(44);
    hand_property.m_propHand.m_strName = "Hand" + std::to_string(system_index_);
    hand_property.m_propHand.SetRenderMode(false, false, true);

    // load hand model
    auto hand_object = world->LoadModel("resources/models/hands/hand.egg");
    auto hand_character = hand_object->GetChild("leap_hand")->GetPtrOf<crsf::TCharacter>();
    hand_character->MakeAllControlJoint();
    hand_property.m_propHand.m_p3DModel = hand_character;
    hand_property.m_propHand.m_p3DModel_LeftWrist = hand_character->FindByName("Bip01FBXASC032RFBXASC032Hand001");
    hand_property.m_propHand.m_p3DModel_RightWrist = hand_character->FindByName("Bip01FBXASC032RFBXASC032Hand");
    LVecBase3 character_scale = hand_character->GetScale(world);
    hand_character->SetScale(character_scale * 0.01f, world);

    hand_object->DisableTestBounding();

    hand_ = std::make_unique<Hand>(hand_property, hand_object);

    hand_->get_hand()->Set3DModel_StandardPose();

    return hand_.get();
}
