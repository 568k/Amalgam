#include "MovementSimulation.h"

#include "../../EnginePrediction/EnginePrediction.h"
#include <numeric>

namespace MoveSimConstants
{
	constexpr float HULL_PADDING = 0.0625f;
	constexpr float MAX_MOVEMENT_SPEED = 450.f;
	constexpr float MAX_RECORD_GAP = 0.35f;
	constexpr float WEIGHT_EXP_DECAY = 1.5f;
	constexpr float VELOCITY_THRESHOLD = 0.015f;

	// TF2 physics
	constexpr float AIR_SPEED_CAP = 30.f;
	constexpr float DEFAULT_SV_AIRACCELERATE = 10.f;
	constexpr float DEFAULT_SV_ACCELERATE = 10.f;
	constexpr float DEFAULT_SV_FRICTION = 4.f;
	constexpr float DEFAULT_SV_STOPSPEED = 100.f;
}

// forward decl
static inline float CalculateExpectedYawGain(float flSpeed, float flMaxSpeed, float flAirSpeedCap = MoveSimConstants::AIR_SPEED_CAP);

static CUserCmd s_tDummyCmd = {};

void CMovementSimulation::Store(MoveStorage& tStorage)
{
	auto pMap = tStorage.m_pPlayer->GetPredDescMap();
	if (!pMap)
		return;

	size_t iSize = tStorage.m_pPlayer->GetIntermediateDataSize();
	tStorage.m_pData = reinterpret_cast<byte*>(I::MemAlloc->Alloc(iSize));

	CPredictionCopy copy = { PC_NETWORKED_ONLY, tStorage.m_pData, PC_DATA_PACKED, tStorage.m_pPlayer, PC_DATA_NORMAL };
	copy.TransferData("MovementSimulationStore", tStorage.m_pPlayer->entindex(), pMap);
}

void CMovementSimulation::Reset(MoveStorage& tStorage)
{
	if (tStorage.m_pData)
	{
		auto pMap = tStorage.m_pPlayer->GetPredDescMap();
		if (!pMap)
			return;

		CPredictionCopy copy = { PC_NETWORKED_ONLY, tStorage.m_pPlayer, PC_DATA_NORMAL, tStorage.m_pData, PC_DATA_PACKED };
		copy.TransferData("MovementSimulationReset", tStorage.m_pPlayer->entindex(), pMap);

		I::MemAlloc->Free(tStorage.m_pData);
		tStorage.m_pData = nullptr;
	}
}

void CMovementSimulation::Store()
{
	for (auto pEntity : H::Entities.GetGroup(EntityEnum::PlayerAll))
	{
		auto pPlayer = pEntity->As<CTFPlayer>();
		const int iIndex = pPlayer->entindex();
		auto& vRecords = m_mRecords[iIndex];

		if (!pPlayer->IsAlive() || pPlayer->IsAGhost())
		{
			vRecords.clear();
			continue;
		}

		bool bLocal = pPlayer->entindex() == I::EngineClient->GetLocalPlayer() && !I::EngineClient->IsPlayingDemo();
		Vec3 vVelocity = bLocal ? F::EnginePrediction.m_vVelocity : pPlayer->m_vecVelocity();
		Vec3 vOrigin = bLocal ? F::EnginePrediction.m_vOrigin : pPlayer->m_vecOrigin();
		if (vVelocity.IsZero())
		{
			vRecords.clear();
			continue;
		}
		float flMaxSpeed = SDK::MaxSpeed(pPlayer);
		Vec3 vDirection = vVelocity.To2D().IsZero() ? Vec3() : vVelocity.Normalized2D() * flMaxSpeed;
		if (pPlayer->IsSwimming())
			vDirection.z = vVelocity.z;

		MoveData* pLastRecord = !vRecords.empty() ? &vRecords.front() : nullptr;
		const float flSimTime = pPlayer->m_flSimulationTime();
		const int iMode = pPlayer->IsSwimming() ? 2 : pPlayer->IsOnGround() ? 0 : 1;
		if (pLastRecord && pLastRecord->m_flSimTime == flSimTime)
		{
			pLastRecord->m_vDirection = vDirection;
			pLastRecord->m_iMode = iMode;
			pLastRecord->m_vVelocity = vVelocity;
			pLastRecord->m_vOrigin = vOrigin;
			continue;
		}
		vRecords.emplace_front(
			vDirection,
			flSimTime,
			iMode,
			vVelocity,
			vOrigin
		);
		if (vRecords.size() > 66)
			vRecords.pop_back();

		if (pLastRecord)
		{
			CGameTrace trace = {};
			CTraceFilterWorldAndPropsOnly filter = {};
			SDK::TraceHull(pLastRecord->m_vOrigin, pLastRecord->m_vOrigin + pLastRecord->m_vVelocity * TICK_INTERVAL, pPlayer->m_vecMins() + MoveSimConstants::HULL_PADDING, pPlayer->m_vecMaxs() - MoveSimConstants::HULL_PADDING, pPlayer->SolidMask(), &filter, &trace);
			if (trace.DidHit() && trace.plane.normal.z < 0.5f)
				vRecords.clear();
		}
	}

	for (auto pEntity : H::Entities.GetGroup(EntityEnum::PlayerAll))
	{
		auto pPlayer = pEntity->As<CTFPlayer>();
		const int iIndex = pPlayer->entindex();
		auto& vSimTimes = m_mSimTimes[iIndex];

		if (pEntity->entindex() == I::EngineClient->GetLocalPlayer() || !pPlayer->IsAlive() || pPlayer->IsAGhost())
		{
			vSimTimes.clear();
			continue;
		}

		float flDeltaTime = H::Entities.GetDeltaTime(iIndex);
		if (!flDeltaTime)
			continue;

		vSimTimes.push_front(flDeltaTime);
		if (vSimTimes.size() > Vars::Aimbot::Projectile::DeltaCount.Value)
			vSimTimes.pop_back();
	}
}

bool CMovementSimulation::Initialize(CBaseEntity* pEntity, MoveStorage& tStorage, bool bHitchance, bool bStrafe)
{
	if (!pEntity || !pEntity->IsPlayer() || !pEntity->As<CTFPlayer>()->IsAlive())
	{
		tStorage.m_bInitFailed = tStorage.m_bFailed = true;
		return false;
	}

	tStorage.m_flAverageYaw = 0.f; // reset any stale strafe estimation

	auto pPlayer = pEntity->As<CTFPlayer>();
	tStorage.m_pPlayer = pPlayer;

	// store vars
	m_bOldInPrediction = I::Prediction->m_bInPrediction;
	m_bOldFirstTimePredicted = I::Prediction->m_bFirstTimePredicted;
	m_flOldFrametime = I::GlobalVars->frametime;

	// store restore data
	Store(tStorage);

	// the hacks that make it work
	I::MoveHelper->SetHost(pPlayer);
	pPlayer->m_pCurrentCommand() = &s_tDummyCmd;

	if (auto pAvgVelocity = H::Entities.GetAvgVelocity(pPlayer->entindex()))
		pPlayer->m_vecVelocity() = *pAvgVelocity; // only use average velocity here

	if (pPlayer->m_bDucked() = pPlayer->IsDucking())
	{
		pPlayer->m_fFlags() &= ~FL_DUCKING; // breaks origin's z if FL_DUCKING is not removed
		pPlayer->m_flDucktime() = 0.f;
		pPlayer->m_flDuckJumpTime() = 0.f;
		pPlayer->m_bDucking() = false;
		pPlayer->m_bInDuckJump() = false;
	}

	if (pPlayer != H::Entities.GetLocal())
	{
		pPlayer->m_vecBaseVelocity() = Vec3(); // residual basevelocity causes issues
		if (pPlayer->IsOnGround())
			pPlayer->m_vecVelocity().z = std::min(pPlayer->m_vecVelocity().z, 0.f); // step fix
		else
			pPlayer->m_hGroundEntity() = nullptr; // fix for velocity.z being set to 0 even if in air
	}
	else if (Vars::Misc::Movement::Bunnyhop.Value && G::OriginalCmd.buttons & IN_JUMP)
		tStorage.m_bBunnyHop = true;

	// setup move data
	if (!SetupMoveData(tStorage))
	{
		tStorage.m_bFailed = true;
		return false;
	}

	// reserve path storage to avoid per tick reallocations when recording paths
	tStorage.m_vPath.clear();
	if (tStorage.m_vPath.capacity() < 128)
		tStorage.m_vPath.reserve(128);

	const int iStrafeSamples = tStorage.m_bDirectMove
		? Vars::Aimbot::Projectile::GroundSamples.Value
		: Vars::Aimbot::Projectile::AirSamples.Value;

	bool bCalculated = bStrafe ? StrafePrediction(tStorage, iStrafeSamples) : false;

	if (bHitchance && bCalculated && !pPlayer->m_vecVelocity().IsZero() && Vars::Aimbot::Projectile::HitChance.Value)
	{
		const auto& vRecords = m_mRecords[pPlayer->entindex()];
		const auto iSamples = vRecords.size();
		if (vRecords.empty())
		{
			tStorage.m_bFailed = true;
			return false;
		}

		float flLegacyChance = 1.f;
		double dbAverageYaw = 0.0;
		double dbTotalTime = 0.0;

		float flYaw1 = Math::VectorAngles(vRecords[0].m_vDirection).y;
		for (size_t i = 0; i < iSamples; i++)
		{
			if (vRecords.size() <= i + 2)
				break;

			const auto& pRecord1 = vRecords[i], &pRecord2 = vRecords[i + 1];
			if (pRecord1.m_iMode != pRecord2.m_iMode)
			{
				flYaw1 = Math::VectorAngles(pRecord2.m_vDirection).y;
				continue;
			}

			const float flYaw2 = Math::VectorAngles(pRecord2.m_vDirection).y;
			const float flTime1 = pRecord1.m_flSimTime, flTime2 = pRecord2.m_flSimTime;
			const float flDelta = flTime1 - flTime2;
			if (flDelta <= 0.f)
			{
				flYaw1 = flYaw2;
				continue;
			}
			const int iTicks = std::max(TIME_TO_TICKS(flDelta), 1);

			float flYaw = Math::NormalizeAngle(flYaw1 - flYaw2);
			dbAverageYaw += flYaw;
			dbTotalTime += std::max(flDelta, 0.f);

			flYaw /= iTicks;

			if (tStorage.m_MoveData.m_flMaxSpeed)
				flYaw *= std::clamp(pRecord1.m_vVelocity.Length2D() / tStorage.m_MoveData.m_flMaxSpeed, 0.f, 1.f);

			if ((i + 1) % iStrafeSamples == 0 || i == iSamples - 1)
			{
				float flAverageYaw = dbAverageYaw / std::max(TIME_TO_TICKS(dbTotalTime), 1);
				if (fabsf(tStorage.m_flAverageYaw - flAverageYaw) > 0.5f)
					flLegacyChance -= 1.f / ((iSamples - 1) / float(iStrafeSamples) + 1);

				dbAverageYaw = 0.0;
				dbTotalTime = 0.0;
			}

			flYaw1 = flYaw2;
		}

		double dbTotalWeight = 0.0;
		double dbDevWeight = 0.0;
		const float flYawTolerance = 0.6f;

		flYaw1 = Math::VectorAngles(vRecords[0].m_vDirection).y;
		for (size_t i = 0; i + 1 < iSamples; i++)
		{
			const auto& pRecord1 = vRecords[i];
			const auto& pRecord2 = vRecords[i + 1];
			if (pRecord1.m_iMode != pRecord2.m_iMode)
			{
				flYaw1 = Math::VectorAngles(pRecord2.m_vDirection).y;
				continue;
			}

			const float flYaw2 = Math::VectorAngles(pRecord2.m_vDirection).y;
			const float flDeltaTime = std::max(pRecord1.m_flSimTime - pRecord2.m_flSimTime, 0.f);
			if (flDeltaTime <= 0.f)
			{
				flYaw1 = flYaw2;
				continue;
			}

			const int iTicks = std::max(TIME_TO_TICKS(flDeltaTime), 1);
			const float flSpeed = pRecord1.m_vVelocity.Length2D();

			float flYawRate = Math::NormalizeAngle(flYaw1 - flYaw2) / iTicks;
			
			// scale to match prediction's compensated yaw
			if (pRecord1.m_iMode == 1 && flSpeed > MoveSimConstants::AIR_SPEED_CAP && tStorage.m_MoveData.m_flMaxSpeed > 0.f)
			{
				float flExpectedGain = CalculateExpectedYawGain(flSpeed, tStorage.m_MoveData.m_flMaxSpeed);
				if (flExpectedGain > 0.01f)
				{
					float flScaleFactor = std::min(fabsf(flYawRate) / flExpectedGain, 3.f);
					if (flScaleFactor > 1.f)
						flYawRate *= std::sqrt(flScaleFactor);
				}
			}
			
			if (tStorage.m_MoveData.m_flMaxSpeed)
				flYawRate *= std::clamp(flSpeed / tStorage.m_MoveData.m_flMaxSpeed, 0.f, 1.f);

			const double dbWeight = flSpeed * flDeltaTime;
			dbTotalWeight += dbWeight;
			if (fabsf(flYawRate - tStorage.m_flAverageYaw) > flYawTolerance)
				dbDevWeight += dbWeight;

			flYaw1 = flYaw2;
		}

		float flNewChance = 1.f;
		if (dbTotalWeight > 0.0)
			flNewChance = static_cast<float>(1.0 - std::clamp(dbDevWeight / dbTotalWeight, 0.0, 1.0));
		
		float flCurrentChance = flLegacyChance;
		if (dbTotalWeight > 0.0)
		{
			constexpr float flBlend = 0.35f;
			flCurrentChance = std::clamp(flLegacyChance + (flNewChance - flLegacyChance) * flBlend, 0.f, 1.f);
		}
		if (flCurrentChance < Vars::Aimbot::Projectile::HitChance.Value / 100)
		{
			if (Vars::Debug::Logging.Value)
			{
				SDK::Output("MovementSimulation", std::format("Hitchance (current {}% < {}%, legacy {}%, new {}%)", flCurrentChance * 100, Vars::Aimbot::Projectile::HitChance.Value, flLegacyChance * 100, flNewChance * 100).c_str(), { 80, 200, 120 }, true);
			}

			tStorage.m_bFailed = true;
			return false;
		}
	}

	for (int i = 0; i < H::Entities.GetChoke(pPlayer->entindex()); i++)
		RunTick(tStorage);

	return true;
}

bool CMovementSimulation::SetupMoveData(MoveStorage& tStorage)
{
	if (!tStorage.m_pPlayer)
		return false;

	tStorage.m_MoveData.m_bFirstRunOfFunctions = false;
	tStorage.m_MoveData.m_bGameCodeMovedPlayer = false;
	tStorage.m_MoveData.m_nPlayerHandle = reinterpret_cast<IHandleEntity*>(tStorage.m_pPlayer)->GetRefEHandle();

	tStorage.m_MoveData.m_vecAbsOrigin = tStorage.m_pPlayer->m_vecOrigin();
	tStorage.m_MoveData.m_vecVelocity = tStorage.m_pPlayer->m_vecVelocity();
	tStorage.m_MoveData.m_flMaxSpeed = SDK::MaxSpeed(tStorage.m_pPlayer);
	tStorage.m_MoveData.m_flClientMaxSpeed = tStorage.m_MoveData.m_flMaxSpeed;

	if (!tStorage.m_MoveData.m_vecVelocity.To2D().IsZero())
	{
		int iIndex = tStorage.m_pPlayer->entindex();
		if (iIndex == I::EngineClient->GetLocalPlayer() && G::CurrentUserCmd)
			tStorage.m_MoveData.m_vecViewAngles = G::CurrentUserCmd->viewangles;
		else
		{
			if (!tStorage.m_pPlayer->InCond(TF_COND_SHIELD_CHARGE))
				tStorage.m_MoveData.m_vecViewAngles = { 0.f, Math::VectorAngles(tStorage.m_MoveData.m_vecVelocity).y, 0.f };
			else
				tStorage.m_MoveData.m_vecViewAngles = H::Entities.GetEyeAngles(iIndex);
		}

		const auto& vRecords = m_mRecords[tStorage.m_pPlayer->entindex()];
		if (!vRecords.empty())
		{
			auto& tRecord = vRecords.front();
			if (!tRecord.m_vDirection.IsZero())
			{
				Vec3 vForward = {}, vRight = {};
				Math::AngleVectors(tStorage.m_MoveData.m_vecViewAngles, &vForward, &vRight);
				vForward = vForward.To2D();
				vRight = vRight.To2D();
				const Vec3 vWish = tRecord.m_vDirection;
				tStorage.m_MoveData.m_flForwardMove = vWish.Dot(vForward);
				tStorage.m_MoveData.m_flSideMove = vWish.Dot(vRight);
				tStorage.m_MoveData.m_flUpMove = vWish.z;
			}
		}
	}

	tStorage.m_MoveData.m_vecAngles = tStorage.m_MoveData.m_vecOldAngles = tStorage.m_MoveData.m_vecViewAngles;
	if (auto pConstraintEntity = tStorage.m_pPlayer->m_hConstraintEntity().Get())
		tStorage.m_MoveData.m_vecConstraintCenter = pConstraintEntity->GetAbsOrigin();
	else
		tStorage.m_MoveData.m_vecConstraintCenter = tStorage.m_pPlayer->m_vecConstraintCenter();
	tStorage.m_MoveData.m_flConstraintRadius = tStorage.m_pPlayer->m_flConstraintRadius();
	tStorage.m_MoveData.m_flConstraintWidth = tStorage.m_pPlayer->m_flConstraintWidth();
	tStorage.m_MoveData.m_flConstraintSpeedFactor = tStorage.m_pPlayer->m_flConstraintSpeedFactor();

	tStorage.m_flPredictedDelta = GetPredictedDelta(tStorage.m_pPlayer);
	tStorage.m_flSimTime = tStorage.m_pPlayer->m_flSimulationTime();
	tStorage.m_flPredictedSimTime = tStorage.m_flSimTime + tStorage.m_flPredictedDelta;
	tStorage.m_vPredictedOrigin = tStorage.m_MoveData.m_vecAbsOrigin;
	tStorage.m_bDirectMove = tStorage.m_pPlayer->IsOnGround() || tStorage.m_pPlayer->IsSwimming();

	return true;
}

static inline float GetGravity(CBaseEntity* pEntity = nullptr)
{
	static auto sv_gravity = H::ConVars.FindVar("sv_gravity");
	float flGravity = sv_gravity->GetFloat();

	if (pEntity && pEntity->IsPlayer() && pEntity->As<CTFPlayer>()->InCond(TF_COND_PARACHUTE_DEPLOYED))
		flGravity *= 0.5f;

	return flGravity;
}

// friction scale for air strafing
static inline float GetFrictionScale(float flVelocityXY, float flTurn, float flVelocityZ, float flMin = 50.f, float flMax = 150.f)
{
	if (flVelocityZ <= 0.f || flVelocityZ > 250.f)
		return 1.f;

	static auto sv_airaccelerate = H::ConVars.FindVar("sv_airaccelerate");
	float flAirAccel = sv_airaccelerate ? sv_airaccelerate->GetFloat() : MoveSimConstants::DEFAULT_SV_AIRACCELERATE;

	float flScaledMin = flMin * std::max(flAirAccel, 1.f);
	float flScaledMax = flMax * std::max(flAirAccel, 1.f);
	float flTurnMagnitude = fabsf(flVelocityXY * DEG2RAD(flTurn));

	return Math::RemapVal(flTurnMagnitude, flScaledMin, flScaledMax, 1.f, 0.25f);
}

// expected yaw gain per tick from air accel (accounts for 30u/s cap)
static inline float CalculateExpectedYawGain(float flSpeed, float flMaxSpeed, float flAirSpeedCap)
{
	if (flSpeed <= 0.f)
		return 0.f;

	static auto sv_airaccelerate = H::ConVars.FindVar("sv_airaccelerate");
	float flAirAccel = sv_airaccelerate ? sv_airaccelerate->GetFloat() : MoveSimConstants::DEFAULT_SV_AIRACCELERATE;

	float flWishSpd = std::min(flMaxSpeed, flAirSpeedCap);
	float flAccelSpeed = flAirAccel * flMaxSpeed * TICK_INTERVAL;
	float flAddSpeed = std::min(flAccelSpeed, flWishSpd);

	if (flSpeed > flAddSpeed)
		return RAD2DEG(asinf(std::min(flAddSpeed / flSpeed, 1.f)));
	return 0.f;
}

// wish direction from view angles
static inline void CalculateWishDirection(const Vec3& vViewAngles, float flForwardMove, float flSideMove,
	Vec3& vWishDir, float& flWishSpeed)
{
	Vec3 vForward, vRight;
	Math::AngleVectors(vViewAngles, &vForward, &vRight);
	
	vForward.z = 0.f;
	vRight.z = 0.f;
	vForward.Normalize();
	vRight.Normalize();

	Vec3 vWishVel = vForward * flForwardMove + vRight * flSideMove;
	vWishVel.z = 0.f;

	flWishSpeed = vWishVel.Length();
	if (flWishSpeed > 0.f)
		vWishDir = vWishVel / flWishSpeed;
	else
		vWishDir = Vec3();
}

class CScopedBounds
{
public:
	CScopedBounds(CTFPlayer* pPlayer) : m_pPlayer(pPlayer)
	{
		if (!m_pPlayer || m_pPlayer->entindex() == I::EngineClient->GetLocalPlayer())
			return;

		if (auto pGameRules = I::TFGameRules())
		{
			if (auto pViewVectors = pGameRules->GetViewVectors())
			{
				m_pViewVectors = pViewVectors;
				m_vHullMin = pViewVectors->m_vHullMin;
				m_vHullMax = pViewVectors->m_vHullMax;
				m_vDuckHullMin = pViewVectors->m_vDuckHullMin;
				m_vDuckHullMax = pViewVectors->m_vDuckHullMax;

				pViewVectors->m_vHullMin = Vec3(-24, -24, 0) + 0.125f;
				pViewVectors->m_vHullMax = Vec3(24, 24, 82) - 0.125f;
				pViewVectors->m_vDuckHullMin = Vec3(-24, -24, 0) + 0.125f;
				pViewVectors->m_vDuckHullMax = Vec3(24, 24, 62) - 0.125f;
			}
		}
	}

	~CScopedBounds()
	{
		if (m_pViewVectors)
		{
			m_pViewVectors->m_vHullMin = m_vHullMin;
			m_pViewVectors->m_vHullMax = m_vHullMax;
			m_pViewVectors->m_vDuckHullMin = m_vDuckHullMin;
			m_pViewVectors->m_vDuckHullMax = m_vDuckHullMax;
		}
	}

private:
	CTFPlayer* m_pPlayer = nullptr;
	CViewVectors* m_pViewVectors = nullptr;
	Vec3 m_vHullMin, m_vHullMax, m_vDuckHullMin, m_vDuckHullMax;
};

struct StrafeDataState
{
	int iChanges = 0;
	int iStart = 0;
	int iStaticSign = 0;
	bool bStaticZero = false;
};

static inline bool GetYawDifference(CBaseEntity* pEntity, MoveData& tRecord1, MoveData& tRecord2, StrafeDataState& state, bool bStart, float* pYaw, float flYaw1, float flYaw2, float flStraightFuzzyValue, int iMaxChanges = 0, int iMaxChangeTime = 0, float flMaxSpeed = 0.f)
{
	const float flTime1 = tRecord1.m_flSimTime, flTime2 = tRecord2.m_flSimTime;
	const float flDeltaTime = flTime1 - flTime2;
	const int iTicks = std::max(TIME_TO_TICKS(flDeltaTime), 1);

	*pYaw = Math::NormalizeAngle(flYaw1 - flYaw2);
	
	const bool bInAir = tRecord1.m_iMode == 1;
	const float flSpeed = tRecord1.m_vVelocity.Length2D();
	
	// velocity turns slower than view at high speed, compensate for that
	if (bInAir && flSpeed > MoveSimConstants::AIR_SPEED_CAP)
	{
		float flExpectedGain = CalculateExpectedYawGain(flSpeed, flMaxSpeed > 0.f ? flMaxSpeed : 320.f);
		
		if (flExpectedGain > 0.01f)
		{
			float flObservedPerTick = fabsf(*pYaw) / iTicks;
			float flScaleFactor = std::min(flObservedPerTick / flExpectedGain, 3.f);
			if (flScaleFactor > 1.f)
				*pYaw *= std::sqrt(flScaleFactor);
		}
	}
	else if (!bInAir && flMaxSpeed > 0.f)
	{
		*pYaw *= std::clamp(flSpeed / flMaxSpeed, 0.f, 1.f);
	}

	if (Vars::Aimbot::Projectile::MovesimFrictionFlags.Value & Vars::Aimbot::Projectile::MovesimFrictionFlagsEnum::CalculateIncrease && bInAir)
	{
		float flFutureZ = tRecord1.m_vVelocity.z + GetGravity(pEntity) * TICK_INTERVAL;
		*pYaw /= GetFrictionScale(flSpeed, *pYaw, flFutureZ, 0.f, 56.f);
	}

	if (fabsf(*pYaw) > 45.f)
		return false;

	const bool iLastZero = state.bStaticZero;
	const bool iCurrZero = fabsf(*pYaw) < 0.001f;
	const int iLastSign = state.iStaticSign;
	const int iCurrSign = iCurrZero ? 0 : sign(*pYaw);
	state.bStaticZero = iCurrZero;
	state.iStaticSign = iCurrSign ? iCurrSign : state.iStaticSign;

	const bool bChanged = (!iCurrZero && iLastSign && iCurrSign && iCurrSign != iLastSign) || (iCurrZero != iLastZero);
	const bool bStraight = fabsf(*pYaw) * tRecord1.m_vVelocity.Length2D() * iTicks < flStraightFuzzyValue; // dumb way to get straight bool

	if (bStart)
	{
		state.iChanges = 0; state.iStart = TIME_TO_TICKS(flTime1);
		if (bStraight && ++state.iChanges > iMaxChanges)
			return false;
		return true;
	}
	else
	{
		if ((bChanged || bStraight) && ++state.iChanges > iMaxChanges)
			return false;
		return state.iChanges && state.iStart - TIME_TO_TICKS(flTime2) > iMaxChangeTime ? false : true;
	}
}

void CMovementSimulation::GetAverageYaw(MoveStorage& tStorage, int iSamples)
{
	auto pPlayer = tStorage.m_pPlayer;
	auto& vRecords = m_mRecords[pPlayer->entindex()];
	if (vRecords.empty())
		return;

	tStorage.m_flAverageYaw = 0.f;

	const float flMinSpeed2D = 6.f;
	const float flMaxRecordGap = MoveSimConstants::MAX_RECORD_GAP;

	bool bGround = tStorage.m_bDirectMove; int iMinimumStrafes = 4;
	float flMaxSpeed = SDK::MaxSpeed(tStorage.m_pPlayer, false, true);

	float flLowMinimumDistance = bGround ? Vars::Aimbot::Projectile::GroundLowMinimumDistance.Value : Vars::Aimbot::Projectile::AirLowMinimumDistance.Value;
	float flLowMinimumSamples = bGround ? Vars::Aimbot::Projectile::GroundLowMinimumSamples.Value : Vars::Aimbot::Projectile::AirLowMinimumSamples.Value;
	float flHighMinimumDistance = bGround ? Vars::Aimbot::Projectile::GroundHighMinimumDistance.Value : Vars::Aimbot::Projectile::AirHighMinimumDistance.Value;
	float flHighMinimumSamples = bGround ? Vars::Aimbot::Projectile::GroundHighMinimumSamples.Value : Vars::Aimbot::Projectile::AirHighMinimumSamples.Value;

	double dbWeightedYaw = 0.0;
	double dbTotalWeight = 0.0;
	double dbAccumulatedTime = 0.0;
	StrafeDataState state = {};

	iSamples = std::min(iSamples, int(vRecords.size()));
	size_t i = 1;
	int iValidStrafes = 0;

	float flYaw1 = Math::VectorAngles(vRecords[0].m_vDirection).y;
	for (; i < iSamples; i++)
	{
		auto& tRecord1 = vRecords[i - 1];
		auto& tRecord2 = vRecords[i];
		if (tRecord1.m_iMode != tRecord2.m_iMode)
		{
			state = {};
			flYaw1 = Math::VectorAngles(tRecord2.m_vDirection).y;
			continue;
		}

		const float flDeltaTime = std::max(tRecord1.m_flSimTime - tRecord2.m_flSimTime, 0.f);
		if (flDeltaTime <= 0.f)
			continue;
		if (flDeltaTime > flMaxRecordGap)
		{
			state = {};
			flYaw1 = Math::VectorAngles(tRecord2.m_vDirection).y;
			continue;
		}

		float flYaw2 = Math::VectorAngles(tRecord2.m_vDirection).y;

		bGround = tRecord1.m_iMode != 1;
		float flStraightFuzzyValue = bGround ? Vars::Aimbot::Projectile::GroundStraightFuzzyValue.Value : Vars::Aimbot::Projectile::AirStraightFuzzyValue.Value;
		int iMaxChanges = bGround ? Vars::Aimbot::Projectile::GroundMaxChanges.Value : Vars::Aimbot::Projectile::AirMaxChanges.Value;
		int iMaxChangeTime = bGround ? Vars::Aimbot::Projectile::GroundMaxChangeTime.Value : Vars::Aimbot::Projectile::AirMaxChangeTime.Value;
		if (!bGround && tRecord2.m_iMode == 0 && flDeltaTime <= TICK_INTERVAL * 2)
			bGround = false;
		iMinimumStrafes = 4 + iMaxChanges;

		const float flSpeed = tRecord1.m_vVelocity.Length2D();
		if (flSpeed < flMinSpeed2D)
		{
			flYaw1 = flYaw2;
			continue;
		}

		float flYaw = 0.f;
		bool bResult = GetYawDifference(pPlayer, tRecord1, tRecord2, state, state.iStart == 0, &flYaw, flYaw1, flYaw2, flStraightFuzzyValue, iMaxChanges, iMaxChangeTime, flMaxSpeed);

		flYaw1 = flYaw2; // Cache for next iteration

		if (!bResult)
		{
			state = {};
			continue;
		}

		const int iTicks = std::max(TIME_TO_TICKS(flDeltaTime), 1);

		const float flYawRate = flYaw / iTicks;
		++iValidStrafes;
		const double dbWeight = flSpeed * flDeltaTime * std::exp(-dbAccumulatedTime * MoveSimConstants::WEIGHT_EXP_DECAY); // weight yaw by speed and time
		dbWeightedYaw += flYawRate * dbWeight;
		dbTotalWeight += dbWeight;
		dbAccumulatedTime += flDeltaTime;
	}

#ifdef VISUALIZE_RECORDS
	size_t i2 = i; for (; i2 < iSamples; i2++)
	{
		auto& tRecord1 = vRecords[i2 - 1];
		auto& tRecord2 = vRecords[i2];

		float flStraightFuzzyValue = bGround ? Vars::Aimbot::Projectile::GroundStraightFuzzyValue.Value : Vars::Aimbot::Projectile::AirStraightFuzzyValue.Value;
		VisualizeRecords(tRecord1, tRecord2, { 0, 0, 0, 100 }, flStraightFuzzyValue);
	}
#endif
	if (iValidStrafes < iMinimumStrafes) // valid strafes not high enough
		return;

	int iMinimum = flLowMinimumSamples;
	if (pPlayer->entindex() != I::EngineClient->GetLocalPlayer())
	{
		float flDistance = 0.f;
		if (auto pLocal = H::Entities.GetLocal())
			flDistance = pLocal->m_vecOrigin().DistTo(tStorage.m_pPlayer->m_vecOrigin());
		iMinimum = flDistance < flLowMinimumDistance ? flLowMinimumSamples : Math::RemapVal(flDistance, flLowMinimumDistance, flHighMinimumDistance, flLowMinimumSamples + 1, flHighMinimumSamples);
	}

	// demand both weight and meaningful time covered; clamp minimum by ticks of collected weight
	const int iCollectedTicks = std::max(TIME_TO_TICKS(float(dbTotalWeight) / std::max(flMaxSpeed, 1.f)), 1);
	if (iCollectedTicks < iMinimum || dbTotalWeight <= 0.0)
		return;

	float flAverageYaw = static_cast<float>(dbWeightedYaw / dbTotalWeight);
	
	if (fabsf(flAverageYaw) < 0.2f)
		return;
	
	tStorage.m_flAverageYaw = flAverageYaw;
	SDK::Output("MovementSimulation", std::format("flAverageYaw calculated to {} with weight {:.2f} (min {} {})", flAverageYaw, dbTotalWeight, iMinimum, pPlayer->entindex() == I::EngineClient->GetLocalPlayer() ? "local" : "remote").c_str(), { 100, 255, 150 }, Vars::Debug::Logging.Value);
}

bool CMovementSimulation::StrafePrediction(MoveStorage& tStorage, int iSamples)
{
	if (tStorage.m_bDirectMove
		? !(Vars::Aimbot::Projectile::StrafePrediction.Value & Vars::Aimbot::Projectile::StrafePredictionEnum::Ground)
		: !(Vars::Aimbot::Projectile::StrafePrediction.Value & Vars::Aimbot::Projectile::StrafePredictionEnum::Air))
		return false;

	GetAverageYaw(tStorage, iSamples);
	return true;
}

bool CMovementSimulation::SetDuck(MoveStorage& tStorage, bool bDuck) // this only touches origin, bounds
{
	if (bDuck == tStorage.m_pPlayer->m_bDucked())
		return true;

	auto pGameRules = I::TFGameRules();
	auto pViewVectors = pGameRules ? pGameRules->GetViewVectors() : nullptr;
	float flScale = tStorage.m_pPlayer->m_flModelScale();

	if (!tStorage.m_pPlayer->IsOnGround())
	{
		Vec3 vHullMins = (pViewVectors ? pViewVectors->m_vHullMin : Vec3(-24, -24, 0)) * flScale;
		Vec3 vHullMaxs = (pViewVectors ? pViewVectors->m_vHullMax : Vec3(24, 24, 82)) * flScale;
		Vec3 vDuckHullMins = (pViewVectors ? pViewVectors->m_vDuckHullMin : Vec3(-24, -24, 0)) * flScale;
		Vec3 vDuckHullMaxs = (pViewVectors ? pViewVectors->m_vDuckHullMax : Vec3(24, 24, 62)) * flScale;

		if (bDuck)
			tStorage.m_MoveData.m_vecAbsOrigin += (vHullMaxs - vHullMins) - (vDuckHullMaxs - vDuckHullMins);
		else
		{
			Vec3 vOrigin = tStorage.m_MoveData.m_vecAbsOrigin - ((vHullMaxs - vHullMins) - (vDuckHullMaxs - vDuckHullMins));

			CGameTrace trace = {};
			CTraceFilterWorldAndPropsOnly filter = {};
			SDK::TraceHull(vOrigin, vOrigin, vHullMins, vHullMaxs, tStorage.m_pPlayer->SolidMask(), &filter, &trace);
			if (trace.DidHit())
				return false;

			tStorage.m_MoveData.m_vecAbsOrigin = vOrigin;
		}
	}
	tStorage.m_pPlayer->m_bDucked() = bDuck;

	return true;
}


void CMovementSimulation::RunTick(MoveStorage& tStorage, bool bPath, std::function<void(CMoveData&)>* pCallback)
{
	if (tStorage.m_bFailed || !tStorage.m_pPlayer || !tStorage.m_pPlayer->IsPlayer())
		return;

	const bool bIsLocalPlayer = tStorage.m_pPlayer->entindex() == I::EngineClient->GetLocalPlayer();

	if (bPath)
		tStorage.m_vPath.push_back(tStorage.m_MoveData.m_vecAbsOrigin);

	// make sure frametime and prediction vars are right
	I::Prediction->m_bInPrediction = true;
	I::Prediction->m_bFirstTimePredicted = false;
	I::GlobalVars->frametime = I::Prediction->m_bEnginePaused ? 0.f : TICK_INTERVAL;
	
	CScopedBounds scopedBounds(tStorage.m_pPlayer);

	float flAppliedYaw = 0.f;
	if (tStorage.m_flAverageYaw)
	{
		float flMult = 1.f;
		if (!tStorage.m_bDirectMove && !tStorage.m_pPlayer->InCond(TF_COND_SHIELD_CHARGE)
			&& (Vars::Aimbot::Projectile::MovesimFrictionFlags.Value & Vars::Aimbot::Projectile::MovesimFrictionFlagsEnum::RunReduce))
		{
			flMult = GetFrictionScale(tStorage.m_MoveData.m_vecVelocity.Length2D(), tStorage.m_flAverageYaw, tStorage.m_MoveData.m_vecVelocity.z + GetGravity(tStorage.m_pPlayer) * TICK_INTERVAL);
		}

		flAppliedYaw = tStorage.m_flAverageYaw * flMult;
		tStorage.m_MoveData.m_vecViewAngles.y += flAppliedYaw;
		
		// update movement inputs for air strafe
		if (!tStorage.m_bDirectMove && !tStorage.m_MoveData.m_vecVelocity.To2D().IsZero())
		{
			Vec3 vVelDir = tStorage.m_MoveData.m_vecVelocity.Normalized2D();
			Vec3 vForward, vRight;
			Math::AngleVectors(tStorage.m_MoveData.m_vecViewAngles, &vForward, &vRight);
			vForward.z = 0.f; vForward.Normalize();
			vRight.z = 0.f; vRight.Normalize();

			float flStrafeSign = (tStorage.m_flAverageYaw > 0.f) ? 1.f : -1.f;
			Vec3 vPerpendicular = Vec3(-vVelDir.y, vVelDir.x, 0.f) * flStrafeSign;
			
			tStorage.m_MoveData.m_flForwardMove = vPerpendicular.Dot(vForward) * MoveSimConstants::MAX_MOVEMENT_SPEED;
			tStorage.m_MoveData.m_flSideMove = vPerpendicular.Dot(vRight) * MoveSimConstants::MAX_MOVEMENT_SPEED;
		}
	}
	else if (!tStorage.m_bDirectMove)
		tStorage.m_MoveData.m_flForwardMove = tStorage.m_MoveData.m_flSideMove = 0.f;

	float flOldSpeed = tStorage.m_MoveData.m_flClientMaxSpeed;
	if (tStorage.m_pPlayer->m_bDucked() && tStorage.m_pPlayer->IsOnGround() && !tStorage.m_pPlayer->IsSwimming())
		tStorage.m_MoveData.m_flClientMaxSpeed /= 3;

	if (tStorage.m_bBunnyHop && tStorage.m_pPlayer->IsOnGround() && !tStorage.m_pPlayer->m_bDucked())
	{
		tStorage.m_MoveData.m_nOldButtons = 0;
		tStorage.m_MoveData.m_nButtons |= IN_JUMP;
	}

	I::GameMovement->ProcessMovement(tStorage.m_pPlayer, &tStorage.m_MoveData);
	if (pCallback)
		(*pCallback)(tStorage.m_MoveData);

	tStorage.m_MoveData.m_flClientMaxSpeed = flOldSpeed;

	tStorage.m_flSimTime += TICK_INTERVAL;
	tStorage.m_bPredictNetworked = tStorage.m_flSimTime >= tStorage.m_flPredictedSimTime;
	if (tStorage.m_bPredictNetworked)
	{
		tStorage.m_vPredictedOrigin = tStorage.m_MoveData.m_vecAbsOrigin;
		tStorage.m_flPredictedSimTime += tStorage.m_flPredictedDelta;
	}
	bool bLastbDirectMove = tStorage.m_bDirectMove;
	tStorage.m_bDirectMove = tStorage.m_pPlayer->IsOnGround() || tStorage.m_pPlayer->IsSwimming();

	if (tStorage.m_flAverageYaw)
		tStorage.m_MoveData.m_vecViewAngles.y -= flAppliedYaw;
	else if (tStorage.m_bDirectMove && !bLastbDirectMove
		&& !tStorage.m_MoveData.m_flForwardMove && !tStorage.m_MoveData.m_flSideMove
		&& tStorage.m_MoveData.m_vecVelocity.Length2D() > tStorage.m_MoveData.m_flMaxSpeed * MoveSimConstants::VELOCITY_THRESHOLD)
	{
		Vec3 vDirection = tStorage.m_MoveData.m_vecVelocity.Normalized2D() * MoveSimConstants::MAX_MOVEMENT_SPEED;
		s_tDummyCmd.forwardmove = vDirection.x, s_tDummyCmd.sidemove = -vDirection.y;
		SDK::FixMovement(&s_tDummyCmd, {}, tStorage.m_MoveData.m_vecViewAngles);
		tStorage.m_MoveData.m_flForwardMove = s_tDummyCmd.forwardmove, tStorage.m_MoveData.m_flSideMove = s_tDummyCmd.sidemove;
	}
}

void CMovementSimulation::RunTick(MoveStorage& tStorage, bool bPath, std::function<void(CMoveData&)> fCallback)
{
	RunTick(tStorage, bPath, &fCallback);
}

void CMovementSimulation::Restore(MoveStorage& tStorage)
{
	if (tStorage.m_bInitFailed || !tStorage.m_pPlayer)
		return;

	I::MoveHelper->SetHost(nullptr);
	tStorage.m_pPlayer->m_pCurrentCommand() = nullptr;

	Reset(tStorage);

	I::Prediction->m_bInPrediction = m_bOldInPrediction;
	I::Prediction->m_bFirstTimePredicted = m_bOldFirstTimePredicted;
	I::GlobalVars->frametime = m_flOldFrametime;
}

float CMovementSimulation::GetPredictedDelta(CBaseEntity* pEntity)
{
	auto& vSimTimes = m_mSimTimes[pEntity->entindex()];
	if (!vSimTimes.empty())
	{
		switch (Vars::Aimbot::Projectile::DeltaMode.Value)
		{
		case 0: return std::reduce(vSimTimes.begin(), vSimTimes.end()) / vSimTimes.size();
		case 1: return *std::max_element(vSimTimes.begin(), vSimTimes.end());
		}
	}
	return TICK_INTERVAL;
}