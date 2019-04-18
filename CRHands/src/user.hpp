#pragma once

#include <memory>

#include <render_pipeline/rppanda/showbase/direct_object.hpp>

namespace crsf {
class TPointMemoryObject;
class TSoundMemoryObject;
class TWorldObject;
}

class Hand;

class User : public rppanda::DirectObject
{
public:
    virtual ~User();

    unsigned int get_system_index() const;

    crsf::TWorldObject* get_avatar() const;
    virtual void set_avatar(crsf::TWorldObject* model);

    virtual void set_voice(crsf::TSoundMemoryObject* voice_mo);
    virtual void play_voice() {}
    virtual void stop_voice() {}

    virtual Hand* make_hand();
    Hand* get_hand() const;

protected:
    User(unsigned int system_index);

    const unsigned int system_index_;

    crsf::TWorldObject* root_;
    crsf::TWorldObject* avatar_ = nullptr;

    crsf::TPointMemoryObject* point_mo_ = nullptr;
    crsf::TSoundMemoryObject* voice_mo_ = nullptr;

    std::unique_ptr<Hand> hand_;
};

// ************************************************************************************************

inline unsigned int User::get_system_index() const
{
    return system_index_;
}

inline crsf::TWorldObject* User::get_avatar() const
{
    return avatar_;
}

inline Hand* User::get_hand() const
{
    return hand_.get();
}
