#include "SeamFinder.h"

#define EPSILON 0.001f

bool CSeamFinder::IntersectPlanes(const cplane_t& p0, const cplane_t& p1, const cplane_t& p2, Vec3& vOut)
{
	Vec3 n0 = p0.normal;
	Vec3 n1 = p1.normal;
	Vec3 n2 = p2.normal;

	Vec3 vCross01 = n0.Cross(n1);
	float flDet = vCross01.Dot(n2);

	if (fabsf(flDet) < EPSILON)
		return false;

	Vec3 vCross12 = n1.Cross(n2);
	Vec3 vCross20 = n2.Cross(n0);

	vOut = (vCross12 * p0.dist + vCross20 * p1.dist + vCross01 * p2.dist) / flDet;
	return true;
}

bool CSeamFinder::IntersectLinePlane(const Vec3& vLineDir, const Vec3& vLinePoint, const Vec3& vPlaneNormal, float flPlaneDist, Vec3& vOut)
{
	float flDot = vPlaneNormal.Dot(vLineDir);
	if (fabsf(flDot) < EPSILON)
		return false;

	float t = (flPlaneDist - vPlaneNormal.Dot(vLinePoint)) / flDot;
	vOut = vLinePoint + vLineDir * t;
	return true;
}

std::vector<SeamEdge> CSeamFinder::GetBrushEdges(const std::vector<cplane_t>& vPlanes)
{
	std::vector<SeamEdge> vEdges;

	for (size_t i = 0; i < vPlanes.size(); i++)
	{
		for (size_t j = i + 1; j < vPlanes.size(); j++)
		{
			const auto& p0 = vPlanes[i];
			const auto& p1 = vPlanes[j];

			if ((p0.normal - p1.normal).Length() < EPSILON || (p0.normal + p1.normal).Length() < EPSILON)
				continue;

			Vec3 vLineDir = p0.normal.Cross(p1.normal);
			vLineDir.Normalize();

			// Find a point on the line (intersection of p0, p1, and a plane perpendicular to line)
			// Or just use IntersectPlanes with a 3rd plane.
			// Construct a 3rd plane perpendicular to line_dir passing through origin? 
			// No, easier way:
			// Solve for point on line.
			// 3rd plane: Normal = vLineDir, Dist = 0.
			cplane_t p2;
			p2.normal = vLineDir;
			p2.dist = 0.f;

			Vec3 vLinePoint;
			if (!IntersectPlanes(p0, p1, p2, vLinePoint))
				continue;

			float flMinDist = -FLT_MAX;
			float flMaxDist = FLT_MAX;
			bool bValid = true;

			for (size_t k = 0; k < vPlanes.size(); k++)
			{
				if (k == i || k == j)
					continue;

				const auto& pk = vPlanes[k];
				float flDot = pk.normal.Dot(vLineDir);
				float flDist = pk.dist - pk.normal.Dot(vLinePoint); // Distance from point to plane? No.
				// Logic from Zig:
				// t = (pc.dist - vec.dot(pc.norm, line_point)) / vec.dot(pc.norm, line_dir);
				// Wait, Source plane equation: dot(n, p) = d
				// So dist from plane is dot(n, p) - d
				// If dot(n, p) - d > 0, point is in front (outside).
				// We want the segment INSIDE the brush, so dot(n, p) - d <= 0 for all planes.
				
				// Intersection t:
				// dot(n, p + dir*t) = d
				// dot(n, p) + dot(n, dir)*t = d
				// t = (d - dot(n, p)) / dot(n, dir)
				
				if (fabsf(flDot) < EPSILON)
				{
					// Parallel
					// If outside, whole line is invalid
					if (pk.normal.Dot(vLinePoint) - pk.dist > EPSILON)
					{
						bValid = false;
						break;
					}
					continue;
				}

				float t = (pk.dist - pk.normal.Dot(vLinePoint)) / flDot;

				if (flDot > 0) // Normal pointing same direction as line
				{
					// If we go further +t, we go further outside (dot increases)
					// So t is an upper bound?
					// If dot > 0, as t increases, dot(n, p+dir*t) increases.
					// We want dot(n, p+dir*t) <= d
					// So we want to be "below" t.
					// max_dist = min(max_dist, t)
					if (t < flMaxDist) flMaxDist = t;
				}
				else // Normal pointing opposite
				{
					// If we go further +t, dot decreases.
					// We want dot <= d.
					// So we want to be "above" t?
					// wait, if dot < 0, as t increases, dot(n, p+dir*t) decreases.
					// so eventually it becomes <= d.
					// so t is a lower bound.
					// min_dist = max(min_dist, t)
					if (t > flMinDist) flMinDist = t;
				}
			}

			if (!bValid || flMaxDist <= flMinDist)
				continue;

			// Add edge
			SeamEdge edge;
			edge.m_vPoint1 = vLinePoint + vLineDir * flMinDist;
			edge.m_vPoint2 = vLinePoint + vLineDir * flMaxDist;
			vEdges.push_back(edge);
		}
	}
	return vEdges;
}

bool CSeamFinder::FitEdgeToBrush(SeamEdge& tEdge, const std::vector<cplane_t>& vPlanes)
{
	SeamEdge tAdjusted = tEdge;
	Vec3 vLineDir = tAdjusted.m_vPoint2 - tAdjusted.m_vPoint1;
	vLineDir.Normalize();
	Vec3 vLinePoint = tAdjusted.m_vPoint1;

	for (const auto& plane : vPlanes)
	{
		float d1 = plane.normal.Dot(tAdjusted.m_vPoint1) - plane.dist;
		float d2 = plane.normal.Dot(tAdjusted.m_vPoint2) - plane.dist;

		if (d1 > EPSILON && d2 > EPSILON)
			return false; // Completely outside

		if (d1 > 0)
		{
			// Clip p1
			Vec3 vIntersection;
			if (IntersectLinePlane(vLineDir, vLinePoint, plane.normal, plane.dist, vIntersection))
				tAdjusted.m_vPoint1 = vIntersection;
			else if (d1 > EPSILON)
				return false;
		}

		if (d2 > 0)
		{
			// Clip p2
			Vec3 vIntersection;
			if (IntersectLinePlane(vLineDir, vLinePoint, plane.normal, plane.dist, vIntersection))
				tAdjusted.m_vPoint2 = vIntersection;
			else if (d2 > EPSILON)
				return false;
		}
		
		if ((tAdjusted.m_vPoint1 - tAdjusted.m_vPoint2).Length() < EPSILON)
			return false;
	}

	tEdge = tAdjusted;
	return true;
}

std::vector<SeamEdge> CSeamFinder::FindSeams(const std::vector<int>& vBrushes)
{
	std::vector<SeamEdge> vSeams;
	std::vector<std::vector<cplane_t>> vBrushPlanes;

	// Cache planes
	for (int iBrush : vBrushes)
	{
		CUtlVector<Vector4D> vPlanesOut;
		int iContents = 0;
		if (I::EngineTrace->GetBrushInfo(iBrush, &vPlanesOut, &iContents))
		{
			if (!(iContents & MASK_SOLID)) // Skip non-solid brushes
				continue;

			std::vector<cplane_t> vPlanes;
			for (int i = 0; i < vPlanesOut.Count(); i++)
			{
				cplane_t plane;
				plane.normal = { vPlanesOut[i].x, vPlanesOut[i].y, vPlanesOut[i].z };
				plane.dist = vPlanesOut[i].w;
				vPlanes.push_back(plane);
			}
			vBrushPlanes.push_back(vPlanes);
		}
	}

	for (size_t i = 0; i < vBrushPlanes.size(); i++)
	{
		auto vEdges = GetBrushEdges(vBrushPlanes[i]);
		
		for (size_t j = 0; j < vBrushPlanes.size(); j++)
		{
			if (i == j) continue;

			for (auto& edge : vEdges)
			{
				SeamEdge tAdjusted = edge;
				if (FitEdgeToBrush(tAdjusted, vBrushPlanes[j]))
				{
					vSeams.push_back(tAdjusted);
				}
			}
		}
	}

	return vSeams;
}

std::vector<SeamEdge> CSeamFinder::FindSeamsInRadius(const Vec3& vOrigin, float flRadius)
{
	Vector vMins = vOrigin - Vec3(flRadius, flRadius, flRadius);
	Vector vMaxs = vOrigin + Vec3(flRadius, flRadius, flRadius);

	CUtlVector<int> vBrushIndices;
	I::EngineTrace->GetBrushesInAABB(vMins, vMaxs, &vBrushIndices, MASK_SOLID);

	std::vector<int> vBrushes;
	for (int i = 0; i < vBrushIndices.Count(); i++)
	{
		vBrushes.push_back(vBrushIndices[i]);
	}

	return FindSeams(vBrushes);
}
