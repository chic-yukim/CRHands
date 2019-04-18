#include "local_user.hpp"

#include <crsf/CoexistenceInterface/TDynamicStageMemory.h>
#include <crsf/CoexistenceInterface/TSoundMemoryObject.h>

LocalUser::LocalUser() : User(crsf::TDynamicStageMemory::GetInstance()->GetSystemIndex())
{
}

LocalUser::~LocalUser()
{
    auto dsm = crsf::TDynamicStageMemory::GetInstance();

    if (voice_mo_)
        dsm->DisableNetworking(voice_mo_);
}

void LocalUser::set_voice(crsf::TSoundMemoryObject* voice_mo)
{
    User::set_voice(voice_mo);

    if (!voice_mo_)
        return;

    crsf::TDynamicStageMemory::GetInstance()->EnableNetworking(voice_mo_, 2);
}
