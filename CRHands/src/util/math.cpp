#include "math.hpp"

LVecBase3 rotate_pos_by_quat(const LVecBase3& pos, const LQuaternionf& quat)
{
	LQuaternionf m_quat = quat;

	// LQuaternionf * LVecBase3
	// nVidia SDK implementation
	LVecBase3 uv, uuv;
	LVecBase3 qvec(m_quat[1], m_quat[2], m_quat[3]);
	uv = qvec.cross(pos); // uv = qvec ^ pos
	uuv = qvec.cross(uv); // uuv = qvec ^ uv
	uv *= (2.0f * m_quat[0]);
	uuv *= 2.0f;
	LVecBase3 result = pos + uv + uuv;

	return result;
}

float dist_two_vector(const LVecBase3& v1, const LVecBase3& v2)
{
	float vx = v1[0] - v2[0];
	float vy = v1[1] - v2[1];
	float vz = v1[2] - v2[2];
	float dist = (float)sqrt(vx * vx + vy * vy + vz * vz);

	return dist;
}