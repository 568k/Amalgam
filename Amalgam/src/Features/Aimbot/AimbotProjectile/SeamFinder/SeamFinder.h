#pragma once
#include "../../../../SDK/SDK.h"

struct SeamEdge {
	Vec3 m_vPoint1;
	Vec3 m_vPoint2;
};

class CSeamFinder
{
public:
	std::vector<SeamEdge> FindSeams(const std::vector<int>& vBrushes);
	std::vector<SeamEdge> FindSeamsInRadius(const Vec3& vOrigin, float flRadius);

private:
	std::vector<SeamEdge> GetBrushEdges(const std::vector<cplane_t>& vPlanes);
	bool FitEdgeToBrush(SeamEdge& tEdge, const std::vector<cplane_t>& vPlanes);
	
	// Math helpers
	bool IntersectPlanes(const cplane_t& p0, const cplane_t& p1, const cplane_t& p2, Vec3& vOut);
	bool IntersectLinePlane(const Vec3& vLineDir, const Vec3& vLinePoint, const Vec3& vPlaneNormal, float flPlaneDist, Vec3& vOut);
};

ADD_FEATURE(CSeamFinder, SeamFinder);
