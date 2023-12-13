#pragma once
#include "ViWoRoot.h"

class PolyRegionCalHelper
{
public:
	//����
	static VPE::dvec2 getCentroid(const std::vector<VPE::dvec2>& poly_region);
	//����
	static VPE::dvec3 getBaryCenter(const std::vector<VPE::dvec3>& poly_region);
	//����������α߳�
	static VPE::dvec2 getBoundingSquare(const std::vector<VPE::dvec2>& poly_region);
};

