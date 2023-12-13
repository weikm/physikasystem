#pragma once
#include "ViWoRoot.h"

class PolyRegionCalHelper
{
public:
	//形心
	static VPE::dvec2 getCentroid(const std::vector<VPE::dvec2>& poly_region);
	//重心
	static VPE::dvec3 getBaryCenter(const std::vector<VPE::dvec3>& poly_region);
	//求外接正方形边长
	static VPE::dvec2 getBoundingSquare(const std::vector<VPE::dvec2>& poly_region);
};

