#pragma once

#include <luse.h>

LVecBase3 rotate_pos_by_quat(const LVecBase3& pos, const LQuaternionf& quat);
float dist_two_vector(const LVecBase3& v1, const LVecBase3& v2);