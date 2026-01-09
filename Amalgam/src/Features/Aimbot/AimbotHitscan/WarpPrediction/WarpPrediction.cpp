#include "WarpPrediction.h"
#include "../../../Simulation/MovementSimulation/MovementSimulation.h"

namespace WarpPrediction // i dont even know if this works as intended, but its worth a try
{
	bool ShouldPredict(CTFPlayer* target)
	{
		if (!target)
			return false;
		return H::Entities.GetLagCompensation(target->entindex());
	}

	Vec3 PredictDelta(CTFPlayer* target, const Vec3& eyePos, const Vec3& origin)
	{
		if (!target || !ShouldPredict(target))
			return {};

		float flDelta = target->m_flSimulationTime() - target->m_flOldSimulationTime();
		int nTicks = TIME_TO_TICKS(flDelta);

		if (nTicks <= 0 || nTicks > 22)
			return {};

		MoveStorage tStorage;
		if (!F::MoveSim.Initialize(target, tStorage))
			return {};

		for (int i = 0; i < nTicks; i++)
			F::MoveSim.RunTick(tStorage);

		Vec3 vPredictedPos = tStorage.m_MoveData.m_vecAbsOrigin;
		F::MoveSim.Restore(tStorage);

		return vPredictedPos - origin;
	}
}
