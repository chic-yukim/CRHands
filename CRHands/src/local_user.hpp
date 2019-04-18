#pragma once

#include "user.hpp"

class LocalUser : public User
{
public:
    LocalUser();
    ~LocalUser() override;

    void set_voice(crsf::TSoundMemoryObject* voice_mo) override;
};
