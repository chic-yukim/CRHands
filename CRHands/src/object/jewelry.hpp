#pragma once

#include "main.hpp"

#include <crsf/CRModel/TGroupedObjectsBase.h>

class Jewelry : public crsf::TGroupedObjectsBase
{
public:
	Jewelry(const std::string& name);
	~Jewelry();

	void initialize_grouped_objects(double scale, const LVecBase3& pos);

	void set_hinge_rotation(float angle);
};