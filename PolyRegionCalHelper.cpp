#include "PolyRegionCalHelper.h"
//形心
VPE::dvec2 PolyRegionCalHelper::getCentroid(const std::vector<VPE::dvec2>& poly_region)
{
    if (poly_region.empty()) {
        return VPE::dvec2();
    }
    double min_x = poly_region[0].x, max_x = poly_region[0].x;
    double min_y = poly_region[0].y, max_y = poly_region[0].y;
    for (auto& v : poly_region) {
        min_x = std::min(min_x, v.x);
        min_y = std::min(min_y, v.y);
        max_x = std::max(max_x, v.x);
        max_y = std::max(max_y, v.y);
    }
    return VPE::dvec2((min_x + max_x) / 2, (min_y + max_y) / 2);
}

//重心
VPE::dvec3 PolyRegionCalHelper::getBaryCenter(const std::vector<VPE::dvec3>& poly_region)
{

}

//外接xy平面正方形边长
VPE::dvec2 PolyRegionCalHelper::getBoundingSquare(const std::vector<VPE::dvec2>& poly_region)
{
    if (poly_region.empty()) {
        return VPE::dvec2();
    }

    double min_x = poly_region[0].x, max_x = poly_region[0].x;
    double min_y = poly_region[0].y, max_y = poly_region[0].y;
    for (auto& v : poly_region) {
        min_x = std::min(min_x, v.x);
        min_y = std::min(min_y, v.y);
        max_x = std::max(max_x, v.x);
        max_y = std::max(max_y, v.y);
    }
    double dx = fabs(max_x - min_x), dy = fabs(max_y - min_y);
    return VPE::dvec2(dx, dy);
}