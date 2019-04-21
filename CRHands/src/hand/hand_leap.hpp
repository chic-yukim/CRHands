/**
* Coexistence Reality Software Framework (CRSF)
* Copyright (c) Center of Human-centered Interaction for Coexistence. All rights reserved.
* See the LICENSE.md file for more details.
*/

namespace crsf {
class TAvatarMemoryObject;
}

class Hand;

void render_hand_leap_local(Hand* hand, crsf::TAvatarMemoryObject* amo);
void render_hand_leap_remote(Hand* hand, crsf::TAvatarMemoryObject* amo);
