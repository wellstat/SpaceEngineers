using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using Sandbox.ModAPI.Ingame;
using SpaceEngineers.Game.ModAPI.Ingame;
using VRageMath;
using VRage;
using VRage.Game;
using VRage.Game.ModAPI.Ingame;
using VRage.Game.ModAPI.Ingame.Utilities;
using VRage.ModAPI;
using Sandbox.Game.GameSystems;

namespace AdvancedGuidedMissile
{
	public class Program : MyGridProgram
	{
//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
#region Script
//------------------------------------------------------------
// ADN - Advanced Guided Missile v7.1
//------------------------------------------------------------

//------------------ Default Settings ------------------
#region Default Settings
const int DEF_NAVIGATION_STRATEGY = 0;

const double DEF_PROP_NAV_CONSTANT = 5;
const double DEF_PROP_NAV_ACCEL_CONSTANT = 0.05;
const double DEF_PROP_NAV_MULTIPLIER_IF_ACCEL = 1.25;

const double DEF_MIN_THRUST_ACCELERATION = 1.0;
const double DEF_THRUST_ESTIMATE_FACTOR = 1.0;

const double DEF_PD_P_GAIN = 30;
const double DEF_PD_D_GAIN = 10;
const double DEF_PD_AIM_LIMIT = 6.3;

const double DEF_LAUNCH_DELAY_TIME = 1;
const bool DEF_COMPENSATE_DAMPENER = true;

const bool DEF_PERFORM_FAN_OUT = false;
const int DEF_FAN_OUT_DURATION_TICKS = 120;
const bool DEF_PERFORM_TURRET_DODGE = false;
const double DEF_TURRET_DODGE_START_DISTANCE = 700;
const double DEF_TURRET_DODGE_END_DISTANCE = 250;

const bool DEF_ACTIVE_HOMING_ENABLED = true;
const bool DEF_RAYCAST_EXCLUDE_FRIENDLY = true;
const bool DEF_RAYCAST_EXCLUDE_NEUTRAL = false;
const bool DEF_RAYCAST_LOCK_NEARBY = true;

const bool DEF_RAYCAST_OFFSET_TARGETING = false;
const double DEF_RAYCAST_RANDOM_OFFSET_AMOUNT = 0;

const double DEF_POS_RAYCAST_EXTRA_DIST = 5;

const double DEF_RAYCAST_MIN_DIST = 50;
const double DEF_RAYCAST_MAX_DIST = 3000;

const bool DEF_USE_INERTIAL_TRIPWIRE = true;
const double INERTIAL_TRIPWIRE_MAX_OFFSET = 7;
const int TRIPWIRE_PREMATURE_TICKS = 0;

const int DEF_TRIPWIRE_TYPE = 0;

const double DEF_IDLE_DISABLE_SECONDS = 60;

const int DEF_DETACH_TYPE = 0;
#endregion
//------------------ Script Constants ------------------
#region Script Constants
const string DISPLAY_VERSION = "7.1";

const int REFRESH_RATE = 10;

const int DETACH_MAX_WAIT_INTERVAL = 60;

const int THRUST_ESTIMATE_INTERVAL = 120;
const int GRAVITY_UPDATE_INTERVAL = 180;

const double EPSILON = 0.00001;
const double ONE_TICK = 1.0 / 60.0;
const double ONE_SECOND = 60.0;
const double STEP_DELTA = 1.0 / REFRESH_RATE;
const int TICKS_PER_REFRESH = (60 / REFRESH_RATE);

const string INI_SECTION = "AGM";
const string INI_SAVED_BLOCKS_SECTION = "AGMSAVE";
const string LAUNCH_COMMAND = "FIRE";
const string PROXIMITY_CAMERA_TAG = "PROX";

const int IDLE_CHECK_INTERVAL = 60;

const int TURRET_DODGE_FULL_SPIN_TICKS = 60;
const double TURRET_DODGE_RADIUS_DIST_RATIO = 0.0625;

const string PARTIAL_TAG = "[NOTREADY]";

const string IGC_MSG_LAUNCH_COMMAND = "IGCMSG_FIRE";

const string IGC_MSG_TARGET_TRACKS = "IGCMSG_TR_TK";
const string IGC_MSG_TARGET_DATALINK = "IGCMSG_TR_DL";
const string IGC_MSG_TARGET_SWITCH_LOST = "IGCMSG_TR_SW";
const string IGC_MSG_MISSILE_COMMAND = "IGCMSG_MS_CM";

const string CGE_MSG_TARGET_DATALINK = "CGE_MSG_TR_DL";

const string FAKE = "SIMJ";

const int INIT_STAGE_ACCEPTING_CMD = 0;
const int INIT_STAGE_PROCESSING_CMD = 1;
const int INIT_STAGE_PENDING_DETACH = 2;
const int INIT_STAGE_COMPLETED = 3;

readonly char[] delimiters = { ',' };
#endregion
//------------------ Global Variables ------------------
#region Global Variables
int initStage = INIT_STAGE_ACCEPTING_CMD;
string commandToBeProcessed = string.Empty;

int detachWaitCount = 0;

Action currentStateAction;
Action fastStateAction;

IMyGridTerminalSystem GTS;

int loadedCustomDataHashCode = 0;

ScriptConfiguration config;
MyIni iniConfig;

MissileInformation missileInfo;
TargetInformation targetInfo;

ThrusterControl thrusterControl;
GyroControl gyroControl;
PDController yawController;
PDController pitchController;
PDController rollController;

RaycastTracker raycastTracker;

List<IMyCameraBlock> cameras;
ProximitySensor proxSensor;

Vector3D? fanOutVector;
bool prevPerformedTurretDodge;

int lastMissileIntactClock = 0;
int lastMissileCommandClock = 0;

Dictionary<string, Action<string>> commandsLookup;
//Dictionary<string, Action> igcMsgLookup;

IMyBroadcastListener igcTargetTracksListener;
IMyBroadcastListener igcTargetDatalinkListener;
IMyBroadcastListener igcTargetSwitchLostListener;
IMyBroadcastListener igcMissileCommandListener;

MyTuple<long, long, long, int, float>? launchByIGCData = null;

int initMsCounter = 0;
double initAverageMs = 0;
double runTotalMs = 0;

long launchControlId = 0;
long timeSinceLastRun = 0;

int clock = 0;
bool init = false;
#endregion
//-------------------- Main Program --------------------

Program()
{
	ForceJITCompilation();

	IGC.UnicastListener.SetMessageCallback(IGC_MSG_LAUNCH_COMMAND);

	if (config == null)
	{
		loadedCustomDataHashCode = 0;
	}

	InitConfiguration();

	bool tagAdded = false;
	try
	{
		if (config.HaveSavedBlocks)
		{
			if (Me.CustomName.IndexOf(PARTIAL_TAG, StringComparison.OrdinalIgnoreCase) == -1)
			{
				Me.CustomName += " " + PARTIAL_TAG;
				tagAdded = true;
			}
		}
	}
	catch (Exception) { }

	string message;
	if (config.HaveSavedBlocks)
	{
		if (tagAdded)
		{
			message = $"SaveMissile configuration present, {PARTIAL_TAG} tag added to signal for build completion checks.";
		}
		else
		{
			message = "SaveMissile configuration present.";
		}
	}
	else
	{
		message = "Missile will be initialized on detach.";
	}
	Echo($"Advanced Guided Missile v{DISPLAY_VERSION}\n\n<<Config Loaded>>\n\n{message}");
}

public void Main(string args, UpdateType updateType)
{
	if (!init)
	{
		long initStartTicks = DateTime.Now.Ticks;
		if (!InitLoop(args, updateType))
		{
			initAverageMs += (DateTime.Now.Ticks - initStartTicks) * 0.0001;
			initMsCounter++;
			return;
		}
		initAverageMs += (DateTime.Now.Ticks - initStartTicks) * 0.0001;
		initMsCounter++;

		initAverageMs /= initMsCounter;
		runTotalMs -= (DateTime.Now.Ticks - initStartTicks) * 0.0001;

		init = true;
	}

	if (args.Length > 0)
	{
		if (args.StartsWith(CGE_MSG_TARGET_DATALINK) && args.Length > CGE_MSG_TARGET_DATALINK.Length)
		{
			ProcessCGETargetDatalinkMessage(args);
			return;
		}
	}

	ProcessIGCTargetDatalinkMessage();
	ProcessIGCTargetSwitchLostMessage();
	ProcessIGCTargetTracksMessage();
	ProcessIGCMissileCommandMessage();

	timeSinceLastRun += Runtime.TimeSinceLastRun.Ticks;

	if ((updateType & UpdateType.Update1) == 0 || timeSinceLastRun == 0)
	{
		return;
	}
	timeSinceLastRun = 0;

	clock++;
	
	runTotalMs += Runtime.LastRunTimeMs;
	if (clock % 60 == 0)
	{
		Echo("Init Average: " + Math.Round(initAverageMs, 4) + "ms\nRuntime Average: " + Math.Round(runTotalMs / clock, 4) + "ms");
	}

	fastStateAction?.Invoke();

	if (clock % TICKS_PER_REFRESH != 0)
	{
		return;
	}

	currentStateAction?.Invoke();
}

#region State Processing

void StateLaunching()
{
	switch (missileInfo.SubState)
	{
		case 0:
			MyMath.InitializeFastSin();

			thrusterControl.FireThrusters(true);
			gyroControl.SetGyroOverride(true);

			missileInfo.MissileRadius = Me.CubeGrid.WorldAABB.HalfExtents.Length();
			missileInfo.MissileAcceleration = CalculateThrustAcceleration(missileInfo.RemoteControl, thrusterControl.Thrusters);

			missileInfo.SubCounter = clock + (int)(config.LaunchDelayTime * ONE_SECOND);
			missileInfo.SubState = 1;
			break;
		case 1:
			if (clock >= missileInfo.SubCounter)
			{
				if (targetInfo.EntityId == 0)
				{
					targetInfo.Position = missileInfo.ForwardBlock.WorldMatrix.Translation + (missileInfo.ForwardBlock.WorldMatrix.Forward * (missileInfo.ForwardBlockReversed ? -1000000 : 1000000));
				}

				missileInfo.SubCounter = 0;
				currentStateAction = StateCommandGuidance;

				if (proxSensor != null)
				{
					RefreshMissileInfo();
					fastStateAction = CheckProximitySensor;
				}
			}
			break;
	}
}

void StateCommandGuidance()
{
	targetInfo.GuessPosition = targetInfo.Position + (targetInfo.Velocity * ONE_TICK * (clock - targetInfo.LastDetectedClock));

	if (config.UseInertialTripwire)
	{
		targetInfo.GuessTripwirePosition = targetInfo.GuessPosition;
	}

	RefreshMissileInfo();
	RefreshExtendedTargetInfo();

	switch (config.NavigationStrategy)
	{
	case 1:
		RefreshNavigationPI();
		break;
	default:
		RefreshNavigationPN();
		break;
	}

	AimAtTarget();

	if (missileInfo.DampenerBlock != null && missileInfo.NaturalGravityLength > 0 && !missileInfo.IsSpinning)
	{
		AimAtGravity();
	}

	if (raycastTracker != null)
	{
		AttemptActiveHomingSwitch();
	}

	if (config.UseInertialTripwire)
	{
		CheckInertialTripwire();
	}

	if (clock % IDLE_CHECK_INTERVAL == 0)
	{
		CheckMissileIdle();
	}
}

void StateActiveHoming()
{
	raycastTracker.Update(clock);

	if (raycastTracker.CurrentRaycastState == RaycastTracker.RaycastState.NO_TARGET)
	{
		missileInfo.SubCounter = 0;
		currentStateAction = StateCommandGuidance;
	}
	else
	{
		if (targetInfo.PreviousVelocityClock != targetInfo.LastDetectedClock)
		{
			targetInfo.PreviousVelocity = targetInfo.Velocity;
			targetInfo.PreviousVelocityClock = targetInfo.LastDetectedClock;
		}

		raycastTracker.GetRaycastTarget(out targetInfo.GuessPosition, out targetInfo.Velocity, out targetInfo.EntityId);

		targetInfo.LastDetectedClock = clock;

		if (config.UseInertialTripwire)
		{
			if (config.OffsetTargeting)
			{
				targetInfo.GuessTripwirePosition = targetInfo.GuessPosition;
			}
			else
			{
				raycastTracker.GetRaycastHitPosition(out targetInfo.GuessTripwirePosition);
			}
		}
	}

	RefreshMissileInfo();
	RefreshExtendedTargetInfo();

	switch (config.NavigationStrategy)
	{
	case 1:
		RefreshNavigationPI();
		break;
	default:
		RefreshNavigationPN();
		break;
	}

	AimAtTarget();

	if (missileInfo.DampenerBlock != null && missileInfo.NaturalGravityLength > 0 && !missileInfo.IsSpinning)
	{
		AimAtGravity();
	}

	if (config.UseInertialTripwire)
	{
		CheckInertialTripwire();
	}

	if (clock % IDLE_CHECK_INTERVAL == 0)
	{
		CheckMissileIdle();
	}
}

#endregion

#region Fast State Processing

void TripwireCountdown()
{
	if (clock >= missileInfo.TripwireClock)
	{
		switch (config.TripwireType)
		{
			case 1:
				CommandArmWarheads(null);
				CommandCountdownWarheads(null);
				break;
			default:
				CommandDetonateWarheads(null);
				break;
		}
		CommandAbort(null);
	}
}

void CheckProximitySensor()
{
	if (proxSensor.CheckProximity(missileInfo.Speed * ONE_TICK, targetInfo.EntityId))
	{
		switch (config.TripwireType)
		{
			case 1:
				CommandArmWarheads(null);
				CommandCountdownWarheads(null);
				break;
			default:
				CommandDetonateWarheads(null);
				break;
		}
		CommandAbort(null);
	}
}

#endregion

#region Guidance Processing

void AttemptActiveHomingSwitch()
{
	raycastTracker.Update(clock);

	if (raycastTracker.CurrentRaycastState == RaycastTracker.RaycastState.TARGET_LOCKED)
	{
		missileInfo.SubCounter = 0;
		missileInfo.ActiveSwitchCount++;
		currentStateAction = StateActiveHoming;
	}
	else
	{
		if (targetInfo.EntityId != 0)
		{
			raycastTracker.StartTrackingByPosition(targetInfo.GuessPosition, targetInfo.EntityId);
		}
		else
		{
			raycastTracker.StartTrackingByVector(missileInfo.ForwardBlock, config.RaycastMaxDistance, RaycastTracker.RaycastStrategy.GHOST_CIRCULAR);
		}
	}
}

void CheckInertialTripwire()
{
	if (targetInfo.EntityId != 0 && missileInfo.IsApproaching)
	{
		Vector3D rangeVector = targetInfo.GuessTripwirePosition - missileInfo.Position;
		rangeVector -= (missileInfo.ForwardBlockReversed ? missileInfo.ForwardBlock.WorldMatrix.Backward : missileInfo.ForwardBlock.WorldMatrix.Forward) * missileInfo.MissileRadius;
		Vector3D approachVector = Vector3D.ProjectOnVector(ref rangeVector, ref missileInfo.Velocity);

		double coverDistanceSq = STEP_DELTA * STEP_DELTA * missileInfo.Speed * missileInfo.Speed;
		if (coverDistanceSq >= approachVector.LengthSquared())
		{
			if ((missileInfo.Position + approachVector - targetInfo.GuessTripwirePosition).LengthSquared() <= (INERTIAL_TRIPWIRE_MAX_OFFSET * INERTIAL_TRIPWIRE_MAX_OFFSET))
			{
				missileInfo.TripwireClock = clock + (int)(Math.Sqrt(approachVector.LengthSquared() / coverDistanceSq) * TICKS_PER_REFRESH) - TRIPWIRE_PREMATURE_TICKS;
				fastStateAction = TripwireCountdown;

				TripwireCountdown();
			}
		}
	}
}

void CheckMissileIdle()
{
	if (gyroControl.HasUsableGyro() && thrusterControl.HasUsableThruster() && (missileInfo.ForwardBlock?.WorldMatrix.Translation ?? Vector3D.Zero) != Vector3D.Zero)
	{
		lastMissileIntactClock = clock;
	}
	
	int earliestThreshold = Math.Min(Math.Max(targetInfo.LastDetectedClock, lastMissileCommandClock), lastMissileIntactClock);
	if (clock - earliestThreshold > config.IdleDisableSeconds * 60)
	{
		Runtime.UpdateFrequency = UpdateFrequency.None;
	}
}

void RefreshMissileInfo()
{
	MatrixD refWorldMatrix = missileInfo.ForwardBlock.WorldMatrix;
	if (missileInfo.ForwardBlockReversed)
	{
		refWorldMatrix.Forward = refWorldMatrix.Backward;
		refWorldMatrix.Left = refWorldMatrix.Right;
	}
	MatrixD.Transpose(ref refWorldMatrix, out missileInfo.ViewMatrix);

	if (missileInfo.RemoteControl != null)
	{
		missileInfo.Position = missileInfo.RemoteControl.CenterOfMass;
		missileInfo.Velocity = missileInfo.RemoteControl.GetShipVelocities().LinearVelocity;
	}
	else
	{
		missileInfo.Position = Me.WorldMatrix.Translation;
		missileInfo.Velocity = Vector3D.Zero;
	}

	double newSpeed = missileInfo.Velocity.Length();
	missileInfo.IsAccelerating = (newSpeed - missileInfo.Speed) > 1;
	missileInfo.Speed = newSpeed;

	missileInfo.NaturalGravity = (missileInfo.RemoteControl?.GetNaturalGravity() ?? Vector3D.Zero);
	if (clock >= missileInfo.NextUpdateNaturalGravity)
	{
		missileInfo.NextUpdateNaturalGravity += GRAVITY_UPDATE_INTERVAL;
		missileInfo.NaturalGravityLength = missileInfo.NaturalGravity.Length();
		if (Math.Abs(missileInfo.NaturalGravityLength) < EPSILON)
		{
			missileInfo.NaturalGravityLength = 0;
		}
	}

	if (clock >= missileInfo.NextThrustEstimate && missileInfo.NaturalGravityLength > 0)
	{
		missileInfo.NextThrustEstimate += THRUST_ESTIMATE_INTERVAL;
		missileInfo.MissileAcceleration = CalculateThrustAcceleration(missileInfo.RemoteControl, thrusterControl.Thrusters);
	}
}

void RefreshExtendedTargetInfo()
{
	if (targetInfo.LastDetectedClock > targetInfo.PreviousVelocityClock)
	{
		targetInfo.GuessAcceleration = (targetInfo.Velocity - targetInfo.PreviousVelocity) * ONE_SECOND / (targetInfo.LastDetectedClock - targetInfo.PreviousVelocityClock);
	}
	else
	{
		targetInfo.GuessAcceleration = Vector3D.Zero;
	}

	targetInfo.RangeVector = targetInfo.GuessPosition - missileInfo.Position;
	targetInfo.RelativeVelocity = targetInfo.Velocity - missileInfo.Velocity;

	missileInfo.IsApproaching = (targetInfo.RelativeVelocity.Dot(ref targetInfo.RangeVector) < 0);

	if (raycastTracker != null && raycastTracker.CurrentRaycastState != RaycastTracker.RaycastState.NO_TARGET)
	{
		if (config.PerformFanOut)
		{
			if (clock <= config.FanOutDurationTicks + (int)(config.LaunchDelayTime * 60))
			{
				if (fanOutVector == null)
				{
					Random rnd = new Random((int)Me.EntityId);
					float angle = (float)rnd.NextDouble() * MathHelper.TwoPi;

					Vector3D rangeVectorNormal = targetInfo.RangeVector;
					double rangeVectorLength = rangeVectorNormal.Normalize();

					Vector3D guideVector = (rangeVectorNormal.Dot(Vector3D.Up) > 0.7 ? Vector3D.Left : Vector3D.Up);
					Vector3D upVector = Vector3D.Normalize(guideVector - Vector3D.ProjectOnVector(ref guideVector, ref rangeVectorNormal));
					Vector3D leftVector = upVector.Cross(rangeVectorNormal);
				
					fanOutVector = ((upVector * MyMath.FastCos(angle)) + (leftVector * MyMath.FastSin(angle))) * rangeVectorLength;
				}

				targetInfo.RangeVector += fanOutVector.Value;
			}
			else
			{
				config.PerformFanOut = false;
			}
		}
		else if (config.PerformTurretDodge)
		{
			double rangeSq = targetInfo.RangeVector.LengthSquared();
			if (rangeSq <= config.TurretDodgeStartDistance * config.TurretDodgeStartDistance && rangeSq >= config.TurretDodgeEndDistance * config.TurretDodgeEndDistance && missileInfo.IsApproaching)
			{
				prevPerformedTurretDodge = true;
				
				float angle = (clock % TURRET_DODGE_FULL_SPIN_TICKS) / (float)TURRET_DODGE_FULL_SPIN_TICKS * MathHelper.TwoPi;

				Vector3D rangeVectorNormal = targetInfo.RangeVector;
				double rangeVectorLength = rangeVectorNormal.Normalize();

				Vector3D guideVector = (rangeVectorNormal.Dot(Vector3D.Up) > 0.7 ? Vector3D.Left : Vector3D.Up);
				Vector3D upVector = Vector3D.Normalize(guideVector - Vector3D.ProjectOnVector(ref guideVector, ref rangeVectorNormal));
				Vector3D leftVector = upVector.Cross(rangeVectorNormal);
		
				Vector3D deviation = (upVector * MyMath.FastCos(angle)) + (leftVector * MyMath.FastSin(angle));
				
				targetInfo.RangeVector += deviation * (rangeVectorLength * TURRET_DODGE_RADIUS_DIST_RATIO);
			}
			else if (prevPerformedTurretDodge)
			{
				config.PerformTurretDodge = false;
			}
		}
	}
}

void RefreshNavigationPN()
{
	Vector3D rangeVectorNormal = Vector3D.Normalize(targetInfo.RangeVector);

    Vector3D rotationVector = Vector3D.Cross(targetInfo.RangeVector, targetInfo.RelativeVelocity) / Math.Max(targetInfo.RangeVector.LengthSquared(), 1);
    Vector3D compensateVector = Vector3D.Cross(rotationVector, rangeVectorNormal) * targetInfo.RelativeVelocity.Length();

	Vector3D targetANVector = targetInfo.GuessAcceleration - (targetInfo.GuessAcceleration.Dot(ref rangeVectorNormal) * rangeVectorNormal);

	Vector3D gravityANVector;
	if (missileInfo.DampenerBlock == null && missileInfo.NaturalGravityLength > 0 && config.CompensateDampener)
	{
		gravityANVector = missileInfo.NaturalGravity * Math.Abs(missileInfo.NaturalGravity.Dot(ref rangeVectorNormal)) / missileInfo.NaturalGravityLength;
	}
	else
	{
		gravityANVector = Vector3D.Zero;
	}

	Vector3D accelerationVector = (config.PropNavConstant * compensateVector) + (config.PropNavAccelConstant * targetANVector) - gravityANVector;
	if (missileInfo.IsAccelerating && missileInfo.Speed > 105)
	{
		accelerationVector *= config.ProvNavMultiplierIfAccel;
	}

	double adjustedRangeLength = (missileInfo.MissileAcceleration * missileInfo.MissileAcceleration) - accelerationVector.LengthSquared();
	missileInfo.TargetVector = Vector3D.TransformNormal((rangeVectorNormal * Math.Sqrt(Math.Max(adjustedRangeLength, 0))) + accelerationVector, ref missileInfo.ViewMatrix);
}

void RefreshNavigationPI()
{
    Vector3D accelerationVector = targetInfo.GuessAcceleration;
    if (missileInfo.DampenerBlock == null && missileInfo.NaturalGravityLength > 0.1 && config.CompensateDampener)
    {
        accelerationVector -= missileInfo.NaturalGravity * missileInfo.NaturalGravityLength;
    }
    
    double a = 0.25 * (accelerationVector.LengthSquared() - (missileInfo.MissileAcceleration * missileInfo.MissileAcceleration));
    double b = accelerationVector.Dot(targetInfo.RelativeVelocity);
    double c = accelerationVector.Dot(targetInfo.RangeVector) + targetInfo.RelativeVelocity.LengthSquared();
    double d = 2 * targetInfo.RangeVector.Dot(targetInfo.RelativeVelocity);
    double e = targetInfo.RangeVector.LengthSquared();

    double t = FastSolver.Solve(a, b, c, d, e);
    if (t == double.MaxValue || double.IsNaN(t)) t = 1000;

    Vector3D interceptPoint = targetInfo.GuessPosition + (targetInfo.RelativeVelocity * t) + (0.5 * accelerationVector * t * t);
    missileInfo.TargetVector = Vector3D.TransformNormal(interceptPoint - missileInfo.Position, ref missileInfo.ViewMatrix);
}

void AimAtTarget()
{
	//---------- Activate Gyroscopes To Turn Towards Target ----------

	double absX = Math.Abs(missileInfo.TargetVector.X);
	double absY = Math.Abs(missileInfo.TargetVector.Y);
	double absZ = Math.Abs(missileInfo.TargetVector.Z);

	double yawInput, pitchInput;
	if (absZ < 0.00001)
	{
		yawInput = pitchInput = MathHelperD.PiOver2;
	}
	else
	{
		bool flipYaw = absX > absZ;
		bool flipPitch = absY > absZ;

		yawInput = FastAT(Math.Max(flipYaw ? (absZ / absX) : (absX / absZ), 0.00001));
		pitchInput = FastAT(Math.Max(flipPitch ? (absZ / absY) : (absY / absZ), 0.00001));

		if (flipYaw) yawInput = MathHelperD.PiOver2 - yawInput;
		if (flipPitch) pitchInput = MathHelperD.PiOver2 - pitchInput;

		if (missileInfo.TargetVector.Z > 0)
		{
			yawInput = (Math.PI - yawInput);
			pitchInput = (Math.PI - pitchInput);
		}
	}

	//---------- PID Controller Adjustment ----------

	if (double.IsNaN(yawInput)) yawInput = 0;
	if (double.IsNaN(pitchInput)) pitchInput = 0;

	yawInput *= Math.Sign(missileInfo.TargetVector.X);
	pitchInput *= Math.Sign(missileInfo.TargetVector.Y);

	yawInput = yawController.Filter(yawInput, 2);
	pitchInput = pitchController.Filter(pitchInput, 2);

	if (Math.Abs(yawInput) + Math.Abs(pitchInput) > config.PDAimLimit)
	{
		double adjust = config.PDAimLimit / (Math.Abs(yawInput) + Math.Abs(pitchInput));
		yawInput *= adjust;
		pitchInput *= adjust;
	}

	//---------- Set Gyroscope Parameters ----------

	gyroControl.CheckGyro();
	gyroControl.SetGyroYaw((float)yawInput);
	gyroControl.SetGyroPitch((float)pitchInput);
}

void AimAtGravity()
{
	Vector3D vector = Vector3D.TransformNormal(missileInfo.NaturalGravity, missileInfo.ViewMatrix);
	double valueX = missileInfo.FuncDownGetRight(ref vector);
	double valueZ = missileInfo.FuncDownGetForward(ref vector);

	double absX = Math.Abs(valueX);
	double absZ = Math.Abs(valueZ);

	double rollInput;
	if (absZ < 0.00001)
	{
		rollInput = MathHelperD.PiOver2;
	}
	else
	{
		bool flipRoll = absX > absZ;

		rollInput = FastAT(Math.Max(flipRoll ? absZ / absX : absX / absZ, 0.00001));

		if (flipRoll) rollInput = MathHelperD.PiOver2 - rollInput;

		if (valueZ > 0)
		{
			rollInput = (Math.PI - rollInput);
		}
	}

	//---------- PID Controller Adjustment ----------

	if (double.IsNaN(rollInput)) rollInput = 0;

	rollInput *= Math.Sign(valueX);

	rollInput = rollController.Filter(rollInput, 2);

	//---------- Set Gyroscope Parameters ----------

	gyroControl.SetGyroRoll((float)rollInput);
}

#endregion

#region Inter Grid Communications Processing

void ProcessIGCTargetDatalinkMessage()
{
	while (igcTargetDatalinkListener.HasPendingMessage)
	{
		object data = igcTargetDatalinkListener.AcceptMessage().Data;
		if (data is MyTuple<bool, long, long, long, Vector3D, Vector3D>)
		{
			//[Notes] TargetDatalinkData(IsGroupRecipient, RecipientId, SenderId, TargetEntityId, TargetPosition, TargetVelocity) => MyTuple<bool, long, long, long, Vector3D, Vector3D>
			MyTuple<bool, long, long, long, Vector3D, Vector3D> targetInfoData = (MyTuple<bool, long, long, long, Vector3D, Vector3D>)data;
			if (IsTargetRecipient(targetInfoData.Item1, targetInfoData.Item2, targetInfoData.Item3))
			{
				if (targetInfo.PreviousVelocityClock != targetInfo.LastDetectedClock)
				{
					targetInfo.PreviousVelocity = targetInfo.Velocity;
					targetInfo.PreviousVelocityClock = targetInfo.LastDetectedClock;
				}

				targetInfo.EntityId = targetInfoData.Item4;
				targetInfo.Position = targetInfoData.Item5;
				targetInfo.Velocity = targetInfoData.Item6;
				targetInfo.LastDetectedClock = clock;
			}
		}
	}
}

void ProcessIGCTargetSwitchLostMessage()
{
	if (raycastTracker != null && raycastTracker.CurrentRaycastState == RaycastTracker.RaycastState.NO_TARGET && missileInfo.ActiveSwitchCount > 0)
	{
		while (igcTargetSwitchLostListener.HasPendingMessage)
		{
			object data = igcTargetSwitchLostListener.AcceptMessage().Data;
			if (data is MyTuple<bool, long, long, long, Vector3D, Vector3D>)
			{
				//[Notes] TargetDatalinkData(IsGroupRecipient, RecipientId, SenderId, TargetEntityId, TargetPosition, TargetVelocity) => MyTuple<bool, long, long, long, Vector3D, Vector3D>
				MyTuple<bool, long, long, long, Vector3D, Vector3D> targetInfoData = (MyTuple<bool, long, long, long, Vector3D, Vector3D>)data;
				if (IsTargetRecipient(targetInfoData.Item1, targetInfoData.Item2, targetInfoData.Item3))
				{
					if (targetInfo.PreviousVelocityClock != targetInfo.LastDetectedClock)
					{
						targetInfo.PreviousVelocity = targetInfo.Velocity;
						targetInfo.PreviousVelocityClock = targetInfo.LastDetectedClock;
					}

					targetInfo.EntityId = targetInfoData.Item4;
					targetInfo.Position = targetInfoData.Item5;
					targetInfo.Velocity = targetInfoData.Item6;
					targetInfo.LastDetectedClock = clock;
				}
			}
		}
	}
	else
	{
		while (igcTargetSwitchLostListener.HasPendingMessage)
		{
			igcTargetSwitchLostListener.AcceptMessage();
		}
	}
}

void ProcessIGCTargetTracksMessage()
{
	while (igcTargetTracksListener.HasPendingMessage)
	{
		object data = igcTargetTracksListener.AcceptMessage().Data;
		if (data is MyTuple<long, long, Vector3D, Vector3D, double>)
		{
			//[Notes] TargetTracksData(SenderId, TargetEntityId, TargetPosition, TargetVelocity, TargetSizeSq) => MyTuple<long, long, Vector3D, Vector3D, double>
			MyTuple<long, long, Vector3D, Vector3D, double> tracksData = (MyTuple<long, long, Vector3D, Vector3D, double>)data;

			if (targetInfo.EntityId == tracksData.Item2)
			{
				if (raycastTracker != null && raycastTracker.CurrentRaycastState != RaycastTracker.RaycastState.TARGET_LOCKED)
				{
					if (targetInfo.PreviousVelocityClock != targetInfo.LastDetectedClock)
					{
						targetInfo.PreviousVelocity = targetInfo.Velocity;
						targetInfo.PreviousVelocityClock = targetInfo.LastDetectedClock;
					}

					targetInfo.Position = tracksData.Item3;
					targetInfo.Velocity = tracksData.Item4;
					targetInfo.LastDetectedClock = clock;

					if (raycastTracker.CurrentRaycastState == RaycastTracker.RaycastState.TARGET_SLIPPED)
					{
						raycastTracker.StopTracking();
					}
				}
			}
		}
	}
}

void ProcessIGCMissileCommandMessage()
{
	while (igcMissileCommandListener.HasPendingMessage)
	{
		object data = igcMissileCommandListener.AcceptMessage().Data;
		if (data is MyTuple<bool, long, long, string>)
		{
			//[Notes] MissileCommandData(GroupRecipient, RecipientId, SenderId, MissileCommand) => MyTuple<bool, long, long, string>
			MyTuple<bool, long, long, string> missileCommandData = (MyTuple<bool, long, long, string>)data;
			if (missileCommandData.Item4 != null && IsTargetRecipient(missileCommandData.Item1, missileCommandData.Item2, missileCommandData.Item3))
			{
				int pos = missileCommandData.Item4.IndexOf(':');
				if (pos > -1)
				{
					ExecuteMissileCommand(missileCommandData.Item4.Substring(0, pos), missileCommandData.Item4.Substring(pos + 1));
				}
				else
				{
					ExecuteMissileCommand(missileCommandData.Item4, string.Empty);
				}

				lastMissileCommandClock = clock;
			}
		}
	}
}

bool IsTargetRecipient(bool groupRecipient, long recipientId, long senderId)
{
	return ((groupRecipient ? config.GroupId : config.UniqueId) == recipientId) && (config.SenderId == 0 || config.SenderId == senderId);
}

void ProcessCGETargetDatalinkMessage(string args)
{
	string msg = args.Substring(CGE_MSG_TARGET_DATALINK.Length);
	MyIni cgeDatalink = new MyIni();
	if (cgeDatalink.TryParse(msg))
	{
		if (targetInfo.PreviousVelocityClock != targetInfo.LastDetectedClock)
		{
			targetInfo.PreviousVelocity = targetInfo.Velocity;
			targetInfo.PreviousVelocityClock = targetInfo.LastDetectedClock;
		}

		targetInfo.EntityId = cgeDatalink.Get(CGE_MSG_TARGET_DATALINK, "EntityId").ToInt64(0);
		targetInfo.Position.X = cgeDatalink.Get(CGE_MSG_TARGET_DATALINK, "PositionX").ToDouble(0);
		targetInfo.Position.Y = cgeDatalink.Get(CGE_MSG_TARGET_DATALINK, "PositionY").ToDouble(0);
		targetInfo.Position.Z = cgeDatalink.Get(CGE_MSG_TARGET_DATALINK, "PositionZ").ToDouble(0);
		targetInfo.Velocity.X = cgeDatalink.Get(CGE_MSG_TARGET_DATALINK, "VelocityX").ToDouble(0);
		targetInfo.Velocity.Y = cgeDatalink.Get(CGE_MSG_TARGET_DATALINK, "VelocityY").ToDouble(0);
		targetInfo.Velocity.Z = cgeDatalink.Get(CGE_MSG_TARGET_DATALINK, "VelocityZ").ToDouble(0);
		targetInfo.LastDetectedClock = clock;
	}
}

#endregion

#region Commands Processing

void ExecuteMissileCommand(string command, string args)
{
	Action<string> action;
	if (commandsLookup.TryGetValue(command, out action))
	{
		action(args);
	}
}

void CommandAbort(string args)
{
	currentStateAction = null;
	fastStateAction = null;

	Runtime.UpdateFrequency = UpdateFrequency.None;
}

void CommandSpin(string args)
{
	float value;
	if (float.TryParse(args, out value))
	{
		gyroControl?.SetGyroRoll(value * MathHelper.RPMToRadiansPerSecond);

		missileInfo.IsSpinning = (value != 0);
	}
}

void CommandDetonateWarheads(string args)
{
	List<IMyWarhead> blocks = new List<IMyWarhead>(0);
	GTS?.GetBlocksOfType(blocks, (b) =>
	{
		b.IsArmed = true;
		b.Detonate();
		return false;
	});
}

void CommandArmWarheads(string args)
{
	List<IMyWarhead> blocks = new List<IMyWarhead>(0);
	GTS?.GetBlocksOfType(blocks, (b) =>
	{
		b.IsArmed = true;
		return false;
	});
}

void CommandCountdownWarheads(string args)
{
	List<IMyWarhead> blocks = new List<IMyWarhead>(0);
	GTS?.GetBlocksOfType(blocks, (b) =>
	{
		b.StartCountdown();
		return false;
	});
}

void CommandMultiple(string args)
{
	string[] groups = args.Split(',');
	for (int i = 0; i < groups.Length; i++)
	{
		int pos = groups[i].IndexOf(':');
		if (pos > -1 && pos < groups[i].Length - 1)
		{
			ExecuteMissileCommand(groups[i].Substring(0, pos), groups[i].Substring(pos + 1));
		}
		else
		{
			ExecuteMissileCommand(groups[i], string.Empty);
		}
	}
}

#endregion

#region Initialization

bool InitLoop(string args, UpdateType updateType)
{
	switch (initStage)
	{
	case INIT_STAGE_ACCEPTING_CMD:
		
		if (args?.Length > 0)
		{
			if ((updateType & UpdateType.IGC) > 0 && args.Equals(IGC_MSG_LAUNCH_COMMAND) && IGC.UnicastListener.HasPendingMessage)
			{
				launchByIGCData = null;

				while (IGC.UnicastListener.HasPendingMessage)
				{
					//[Notes] TargetDatalinkData(LaunchControlId, SetUniqueId, SetGroupId, SetBitMaskOptions(1=OffsetTargeting), SetRandomOffsetAmount) => MyTuple<long, long, long, int, float>
					MyIGCMessage message = IGC.UnicastListener.AcceptMessage();
					if (message.Data is MyTuple<long, long, long, int, float>)
					{
						IGC.UnicastListener.DisableMessageCallback();

						launchByIGCData = (MyTuple<long, long, long, int, float>)message.Data;

						initStage = INIT_STAGE_PROCESSING_CMD;
						commandToBeProcessed = message.Tag;

						Runtime.UpdateFrequency = UpdateFrequency.Update1;

						break;
					}
				}
			}
			else
			{
				launchByIGCData = null;

				initStage = INIT_STAGE_PROCESSING_CMD;
				commandToBeProcessed = args;

				Runtime.UpdateFrequency = UpdateFrequency.Update1;
			}
		}

		return false;
		
	case INIT_STAGE_PROCESSING_CMD:
		
		if (launchByIGCData != null || commandToBeProcessed.StartsWith(LAUNCH_COMMAND, StringComparison.OrdinalIgnoreCase))
		{
			if (launchByIGCData == null)
			{
				if (commandToBeProcessed.Length > LAUNCH_COMMAND.Length)
				{
					string[] tokens = commandToBeProcessed.Split(':');
					if (tokens.Length > 1)
					{
						if (!long.TryParse(tokens[1], out launchControlId)) launchControlId = 0;
					}
				}
			}
			else
			{
				launchControlId = launchByIGCData.Value.Item1;
			}

			commandToBeProcessed = string.Empty;

			if ((Runtime.UpdateFrequency & UpdateFrequency.Update1) == 0)
			{
				Runtime.UpdateFrequency = UpdateFrequency.Update1;
			}

			InitVariables();
			InitConfiguration();

			//[Notes] TargetDatalinkData(LaunchControlId, SetUniqueId, SetGroupId, SetOffsetTargeting, SetRandomOffsetAmount) => MyTuple<long, long, long, bool, float>
			if (launchByIGCData != null)
			{
				config.UniqueId = launchByIGCData.Value.Item2;
				config.GroupId = launchByIGCData.Value.Item3;
				if ((launchByIGCData.Value.Item4 & (int)AGMLaunchOptionEnum.OffsetTargeting) > 0) config.OffsetTargeting = true;
				if (launchByIGCData.Value.Item5 > 0) config.RandomOffsetAmount = launchByIGCData.Value.Item5;
			}

			if (config.HaveSavedBlocks)
			{
				if (InitSavedMissile())
				{
					DetachMissile();

					detachWaitCount = 0;

					initStage = INIT_STAGE_PENDING_DETACH;

					iniConfig = null;
				}
				else
				{
					initStage = INIT_STAGE_ACCEPTING_CMD;

					Runtime.UpdateFrequency = UpdateFrequency.None;

					Echo("\n<< Missile Incomplete >>");

					iniConfig = null;
				}
			}
			else
			{
				PopulateDetachBlock();
				if (missileInfo.DetachBlock == null)
				{
					Echo("\nWarning: Detach Not Found");
				}

				DetachMissile();

				detachWaitCount = 0;

				initStage = INIT_STAGE_PENDING_DETACH;
			}
		}
		else
		{
			RunConfigAndDianostics(commandToBeProcessed);

			initStage = INIT_STAGE_ACCEPTING_CMD;

			commandToBeProcessed = string.Empty;

			Runtime.UpdateFrequency = UpdateFrequency.None;
		}

		return false;

	case INIT_STAGE_PENDING_DETACH:
		
		if (config.HaveSavedBlocks)
		{
			if (CheckDetachSuccessful())
			{
				if (cameras != null && cameras.Count > 0)
				{
					foreach (IMyCameraBlock camera in cameras)
					{
						if (camera.CustomName.IndexOf(PROXIMITY_CAMERA_TAG, StringComparison.OrdinalIgnoreCase) > -1)
						{
							if (proxSensor == null)
							{
								proxSensor = new ProximitySensor();
								proxSensor.ExcludeFriendly = config.ExcludeFriendly;
								proxSensor.ExcludeNeutral = config.ExcludeNeutral;
							}
							proxSensor.AddCamera(camera);
						}
					}

					InitRaycastTracker(cameras);
				}

				currentStateAction = StateLaunching;

				initStage = INIT_STAGE_COMPLETED;

				return true;
			}
			else
			{
				detachWaitCount++;

				if (detachWaitCount > DETACH_MAX_WAIT_INTERVAL)
				{
					initStage = INIT_STAGE_ACCEPTING_CMD;

					Runtime.UpdateFrequency = UpdateFrequency.None;

					Echo("\n<< Detach Failed >>");
				}
			}
		}
		else
		{
			if (CheckDetachSuccessful())
			{
				if (InitMissile())
				{
					currentStateAction = StateLaunching;

					initStage = INIT_STAGE_COMPLETED;

					iniConfig = null;

					return true;
				}
				else
				{
					initStage = INIT_STAGE_ACCEPTING_CMD;

					Runtime.UpdateFrequency = UpdateFrequency.None;

					Echo("\n<< Missile Incomplete >>");

					iniConfig = null;
				}
			}
			else
			{
				detachWaitCount++;

				if (detachWaitCount > DETACH_MAX_WAIT_INTERVAL)
				{
					initStage = INIT_STAGE_ACCEPTING_CMD;

					Runtime.UpdateFrequency = UpdateFrequency.None;

					Echo("\n<< Detach Failed >>");

					iniConfig = null;
				}
			}
		}

		return false;
	}
	
	return false;
}

void DetachMissile()
{
	switch (missileInfo.DetachType)
	{
	case DetachTypeEnum.Merge:
		IMyShipMergeBlock mergeBlock = missileInfo.DetachBlock as IMyShipMergeBlock;
		if (mergeBlock?.Enabled ?? false)
		{
			mergeBlock.Enabled = false;
		}
		return;
	case DetachTypeEnum.RotorTop:
	case DetachTypeEnum.RotorBase:
		IMyMotorStator rotorBlock = missileInfo.DetachBlock as IMyMotorStator;
		rotorBlock?.Detach();
		return;
	case DetachTypeEnum.Connector:
		IMyShipConnector connectorBlock = missileInfo.DetachBlock as IMyShipConnector;
		connectorBlock?.Disconnect();
		return;
	}
}

public bool CheckDetachSuccessful()
{
	if (missileInfo.DetachBlock != null)
	{
		switch (missileInfo.DetachType)
		{
		case DetachTypeEnum.Merge:
			IMyShipMergeBlock mergeBlock = missileInfo.DetachBlock as IMyShipMergeBlock;
			if (mergeBlock != null && !mergeBlock.IsConnected)
			{
				return true;
			}
			break;
		case DetachTypeEnum.RotorTop:
		case DetachTypeEnum.RotorBase:
			IMyMotorStator rotorBlock = missileInfo.DetachBlock as IMyMotorStator;
			if (rotorBlock != null && !rotorBlock.IsAttached)
			{
				return true;
			}
			break;
		case DetachTypeEnum.Connector:
			IMyShipConnector connectorBlock = missileInfo.DetachBlock as IMyShipConnector;
			if (connectorBlock != null && connectorBlock.Status != MyShipConnectorStatus.Connected)
			{
				return true;
			}
			break;
		}
	}

	if (launchControlId > 0)
	{
		if (GTS?.GetBlockWithId(launchControlId) == null)
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	return false;
}

public bool InitMissile()
{
	cameras = new List<IMyCameraBlock>();

	List<IMyGyro> gyros = new List<IMyGyro>();
	
	Vector3D gridDiagonalVector = ComputeBlockGridDiagonalVector(Me);
	ThrusterGroup[] thrusterGroups = new ThrusterGroup[6];

	bool haveThruster = false;
	bool hasForwardReference = IsNotEmpty(config.ForwardReferenceName);
	bool hasDampenerReference = IsNotEmpty(config.DampenerReferenceName);

	List<IMyTerminalBlock> blocks = new List<IMyTerminalBlock>(0);
	GTS?.GetBlocksOfType(blocks, (b) =>
	{
		if (b.CubeGrid == Me.CubeGrid)
		{
			if (b is IMyGyro)
			{
				gyros.Add(b as IMyGyro);
			}
			else if (b is IMyThrust)
			{
				IMyThrust thruster = b as IMyThrust;

				int index = (int)thruster.Orientation.Forward;
				if (thrusterGroups[index] == null)
				{
					thrusterGroups[index] = new ThrusterGroup(index, gridDiagonalVector.Dot(thruster.WorldMatrix.Forward));
					thrusterGroups[index].Thrusters = new List<IMyThrust>();
				}
				thrusterGroups[index].Thrusters.Add(thruster);
				thrusterGroups[index].TotalThrust += Math.Max(thruster.MaxEffectiveThrust, 0.00001f);

				haveThruster = true;
			}
			else if (b is IMyCameraBlock)
			{
				if (config.ActiveHomingEnabled)
				{
					cameras.Add(b as IMyCameraBlock);
				}

				if (b.CustomName.IndexOf(PROXIMITY_CAMERA_TAG, StringComparison.OrdinalIgnoreCase) > -1)
				{
					if (proxSensor == null)
					{
						proxSensor = new ProximitySensor();
						proxSensor.ExcludeFriendly = config.ExcludeFriendly;
						proxSensor.ExcludeNeutral = config.ExcludeNeutral;
					}
					proxSensor.AddCamera(b as IMyCameraBlock);
				}
			}
			else if (b is IMyRemoteControl)
			{
				missileInfo.RemoteControl = b as IMyRemoteControl;
			}
			else if (b is IMyRadioAntenna)
			{
				missileInfo.CommsAntenna = b as IMyRadioAntenna;
			}

			if (hasForwardReference && missileInfo.ForwardBlock == null)
			{
				if (NameContains(b, config.ForwardReferenceName))
				{
					missileInfo.ForwardBlock = b;
					missileInfo.ForwardBlockReversed = (b is IMyThrust);
				}
			}

			if (hasDampenerReference && missileInfo.DampenerBlock == null && b is IMyThrust)
			{
				if (NameContains(b, config.DampenerReferenceName))
				{
					missileInfo.DampenerBlock = b;
				}
			}
		}
		return false;
	});

	if (GTS == null)
	{
		return false;
	}

	if (gyros.Count == 0)
	{
		Echo("Error: No Gyroscopes Found.");
		return false;
	}
	if (!haveThruster)
	{
		Echo("Error: No Thrusters Found.");
		return false;
	}
	if (missileInfo.RemoteControl == null)
	{
		Echo("Error: No Remote Control Found.");
		return false;
	}

	InitThrusters(thrusterGroups);
	thrusterControl.Enabled(true);

	MatrixD refWorldMatrix = missileInfo.ForwardBlock.WorldMatrix;
	if (missileInfo.ForwardBlockReversed)
	{
		refWorldMatrix.Forward = refWorldMatrix.Backward;
		refWorldMatrix.Left = refWorldMatrix.Right;
	}

	InitGyroControls(gyros, ref refWorldMatrix);
	gyroControl.Enabled(true);

	InitPDControllers();

	InitCommsAntenna();

	InitCommunications();

	InitRaycastTracker(cameras);

	return true;
}

public bool InitSavedMissile()
{
	Vector3I[] vec = { -Base6Directions.GetIntVector(Me.Orientation.Left), Base6Directions.GetIntVector(Me.Orientation.Up), -Base6Directions.GetIntVector(Me.Orientation.Forward) };

	string detachBlockStr = iniConfig.Get(INI_SAVED_BLOCKS_SECTION, "DetachBlock").ToString();
	if (IsNotEmpty(detachBlockStr))
	{
		Base64BytePosToBlock(detachBlockStr, Me, ref vec, out missileInfo.DetachBlock);
		if (missileInfo.DetachBlock == null || !(missileInfo.DetachBlock is IMyShipMergeBlock || missileInfo.DetachBlock is IMyMotorStator || missileInfo.DetachBlock is IMyShipConnector))
		{
			return false;
		}
		missileInfo.DetachType = GetDetachTypeOfBlock(missileInfo.DetachBlock, Me.CubeGrid);
	}
	else
	{
		PopulateDetachBlock(true);
		if (missileInfo.DetachBlock == null && config.DetachType != DetachTypeEnum.Standard && config.DetachType != DetachTypeEnum.Sprue)
		{
			return false;
		}
	}

	if (!Base64BytePosToBlock(iniConfig.Get(INI_SAVED_BLOCKS_SECTION, "CommsAntenna").ToString(), Me, ref vec, out missileInfo.CommsAntenna))
	{
		return false;
	}

	if (!Base64BytePosToBlock(iniConfig.Get(INI_SAVED_BLOCKS_SECTION, "DampenerBlock").ToString(), Me, ref vec, out missileInfo.DampenerBlock))
	{
		return false;
	}

	Base64BytePosToBlock(iniConfig.Get(INI_SAVED_BLOCKS_SECTION, "ForwardBlock").ToString(), Me, ref vec, out missileInfo.ForwardBlock);
	if (missileInfo.ForwardBlock == null)
	{
		return false;
	}

	missileInfo.ForwardBlockReversed = iniConfig.Get(INI_SAVED_BLOCKS_SECTION, "ForwardBlockReversed").ToBoolean(false);

	Type dirType = new Base6Directions.Direction().GetType();

	try
	{
		missileInfo.DownDirectionForward = (Base6Directions.Direction)Enum.Parse(dirType, iniConfig.Get(INI_SAVED_BLOCKS_SECTION, "DownDirectionForward").ToString());
	}
	catch
	{
		return false;
	}
	missileInfo.FuncDownGetForward = GetDampenerValueExtractor(missileInfo.DownDirectionForward);

	try
	{
		missileInfo.DownDirectionRight = (Base6Directions.Direction)Enum.Parse(dirType, iniConfig.Get(INI_SAVED_BLOCKS_SECTION, "DownDirectionRight").ToString());
	}
	catch
	{
		return false;
	}
	missileInfo.FuncDownGetRight = GetDampenerValueExtractor(missileInfo.DownDirectionRight);

	Base64BytePosToBlock(iniConfig.Get(INI_SAVED_BLOCKS_SECTION, "RemoteControl").ToString(), Me, ref vec, out missileInfo.RemoteControl);
	if (missileInfo.RemoteControl == null)
	{
		return false;
	}

	List<IMyGyro> gyros;
	Base64BytePosToBlockList(iniConfig.Get(INI_SAVED_BLOCKS_SECTION, "Gyroscopes").ToString(), Me, ref vec, out gyros);
	if (gyros == null || gyros.Count == 0)
	{
		return false;
	}

	List<IMyThrust> thrusters;
	Base64BytePosToBlockList(iniConfig.Get(INI_SAVED_BLOCKS_SECTION, "Thrusters").ToString(), Me, ref vec, out thrusters);
	if (thrusters == null || thrusters.Count == 0)
	{
		return false;
	}

	List<IMyPowerProducer> power;
	if (!Base64BytePosToBlockList(iniConfig.Get(INI_SAVED_BLOCKS_SECTION, "PowerBlocks").ToString(), Me, ref vec, out power))
	{
		return false;
	}

	if (!Base64BytePosToBlockList(iniConfig.Get(INI_SAVED_BLOCKS_SECTION, "RaycastCameras").ToString(), Me, ref vec, out cameras))
	{
		return false;
	}

	if (GTS == null)
	{
		return false;
	}

	if (power != null && power.Count > 0)
	{
		foreach (IMyPowerProducer block in power)
		{
			if (!block.Enabled) block.Enabled = true;
		}
	}

	MatrixD refWorldMatrix = missileInfo.ForwardBlock.WorldMatrix;
	if (missileInfo.ForwardBlockReversed)
	{
		refWorldMatrix.Forward = refWorldMatrix.Backward;
		refWorldMatrix.Left = refWorldMatrix.Right;
	}

	thrusterControl = new ThrusterControl(thrusters);
	thrusterControl.Enabled(true);

	InitGyroControls(gyros, ref refWorldMatrix);
	gyroControl.Enabled(true);

	InitPDControllers();

	InitCommsAntenna();

	InitCommunications();

	return true;
}

void InitVariables()
{
	GTS = GridTerminalSystem;

	if (loadedCustomDataHashCode == 0)
	{
		config = new ScriptConfiguration();
	}

	missileInfo = new MissileInformation();
	targetInfo = new TargetInformation();

	thrusterControl = null;
	gyroControl = null;
	yawController = null;
	pitchController = null;
	rollController = null;

	raycastTracker = null;

	proxSensor = null;

	commandsLookup = new Dictionary<string, Action<string>>(StringComparer.OrdinalIgnoreCase);
	commandsLookup["ABORT"] = CommandAbort;
	commandsLookup["SPIN"] = CommandSpin;
	commandsLookup["DETONATE"] = CommandDetonateWarheads;
	commandsLookup["ARM"] = CommandArmWarheads;
	commandsLookup["COUNTDOWN"] = CommandCountdownWarheads;
	commandsLookup["MULTI"] = CommandMultiple;
}

void InitConfiguration()
{
	int latestHashCode = Me.CustomData.GetHashCode();
	if (loadedCustomDataHashCode == 0 || loadedCustomDataHashCode != latestHashCode)
	{
		loadedCustomDataHashCode = latestHashCode;
	
		bool validConfigLoaded = false;
	
		config = new ScriptConfiguration();

		iniConfig = new MyIni();
		if (iniConfig.TryParse(Me.CustomData))
		{
			if (iniConfig.ContainsSection(INI_SECTION))
			{
				config.UniqueId = iniConfig.Get(INI_SECTION, "UniqueId").ToInt64(0);
				config.GroupId = iniConfig.Get(INI_SECTION, "GroupId").ToInt64(0);
				config.SenderId = iniConfig.Get(INI_SECTION, "SenderId").ToInt64(0);
				config.NavigationStrategy = iniConfig.Get(INI_SECTION, "NavigationStrategy").ToInt32(DEF_NAVIGATION_STRATEGY);
				config.PropNavConstant = iniConfig.Get(INI_SECTION, "PropNavConstant").ToDouble(DEF_PROP_NAV_CONSTANT);
				config.PropNavAccelConstant = iniConfig.Get(INI_SECTION, "PropNavAccelConstant").ToDouble(DEF_PROP_NAV_ACCEL_CONSTANT);
				config.ProvNavMultiplierIfAccel = iniConfig.Get(INI_SECTION, "ProvNavMultiplierIfAccel").ToDouble(DEF_PROP_NAV_MULTIPLIER_IF_ACCEL);
				config.MinimumThrustAcceleration = iniConfig.Get(INI_SECTION, "MinimumThrustAcceleration").ToDouble(DEF_MIN_THRUST_ACCELERATION);
				config.ThrustEstimateFactor = iniConfig.Get(INI_SECTION, "ThrustEstimateFactor").ToDouble(DEF_THRUST_ESTIMATE_FACTOR);
				config.PDGainP = iniConfig.Get(INI_SECTION, "PDGainP").ToDouble(DEF_PD_P_GAIN);
				config.PDGainD = iniConfig.Get(INI_SECTION, "PDGainD").ToDouble(DEF_PD_D_GAIN);
				config.PDAimLimit = iniConfig.Get(INI_SECTION, "PDAimLimit").ToDouble(DEF_PD_AIM_LIMIT);
				config.LaunchDelayTime = iniConfig.Get(INI_SECTION, "LaunchDelayTime").ToDouble(DEF_LAUNCH_DELAY_TIME);
				config.PerformFanOut = iniConfig.Get(INI_SECTION, "PerformFanOut").ToBoolean(DEF_PERFORM_FAN_OUT);
				config.FanOutDurationTicks = iniConfig.Get(INI_SECTION, "FanOutDurationTicks").ToInt32(DEF_FAN_OUT_DURATION_TICKS);
				config.PerformTurretDodge = iniConfig.Get(INI_SECTION, "PerformTurretDodge").ToBoolean(DEF_PERFORM_TURRET_DODGE);
				config.TurretDodgeStartDistance = iniConfig.Get(INI_SECTION, "TurretDodgeStartDistance").ToDouble(DEF_TURRET_DODGE_START_DISTANCE);
				config.TurretDodgeEndDistance = iniConfig.Get(INI_SECTION, "TurretDodgeEndDistance").ToDouble(DEF_TURRET_DODGE_END_DISTANCE);
				config.CompensateDampener = iniConfig.Get(INI_SECTION, "CompensateDampener").ToBoolean(DEF_COMPENSATE_DAMPENER);
				config.ActiveHomingEnabled = iniConfig.Get(INI_SECTION, "ActiveHomingEnabled").ToBoolean(DEF_ACTIVE_HOMING_ENABLED);
				config.ExcludeFriendly = iniConfig.Get(INI_SECTION, "ExcludeFriendly").ToBoolean(DEF_RAYCAST_EXCLUDE_FRIENDLY);
				config.ExcludeNeutral = iniConfig.Get(INI_SECTION, "ExcludeNeutral").ToBoolean(DEF_RAYCAST_EXCLUDE_NEUTRAL);
				config.RaycastRelockNearby = iniConfig.Get(INI_SECTION, "RaycastRelockNearby").ToBoolean(DEF_RAYCAST_LOCK_NEARBY);
				config.OffsetTargeting = iniConfig.Get(INI_SECTION, "OffsetTargeting").ToBoolean(DEF_RAYCAST_OFFSET_TARGETING);
				config.RandomOffsetAmount = iniConfig.Get(INI_SECTION, "RandomOffsetAmount").ToDouble(DEF_RAYCAST_RANDOM_OFFSET_AMOUNT);
				config.ProbeOffsetVector = iniConfig.Get(INI_SECTION, "ProbeOffsetVector").ToString();
				config.RaycastMinDistance = iniConfig.Get(INI_SECTION, "RaycastMinDistance").ToDouble(DEF_RAYCAST_MIN_DIST);
				config.RaycastMaxDistance = iniConfig.Get(INI_SECTION, "RaycastMaxDistance").ToDouble(DEF_RAYCAST_MAX_DIST);
				config.ForwardReferenceName = iniConfig.Get(INI_SECTION, "ForwardReferenceName").ToString();
				config.DampenerReferenceName = iniConfig.Get(INI_SECTION, "DampenerReferenceName").ToString();
				config.TripwireType = iniConfig.Get(INI_SECTION, "TripwireType").ToInt32(DEF_TRIPWIRE_TYPE);
				config.UseInertialTripwire = iniConfig.Get(INI_SECTION, "UseInertialTripwire").ToBoolean(DEF_USE_INERTIAL_TRIPWIRE);
				config.IdleDisableSeconds = iniConfig.Get(INI_SECTION, "IdleDisableSeconds").ToDouble(DEF_IDLE_DISABLE_SECONDS);
				config.DetachGroupName = iniConfig.Get(INI_SECTION, "DetachGroupName").ToString();
				config.DetachBlockTag = iniConfig.Get(INI_SECTION, "DetachBlockTag").ToString();
				config.DetachType = GetDetachTypeFromString(iniConfig.Get(INI_SECTION, "DetachType").ToString().Trim());
				
				if (config.TurretDodgeStartDistance < config.TurretDodgeEndDistance)
				{
					double startDist = config.TurretDodgeEndDistance;
					config.TurretDodgeEndDistance = config.TurretDodgeStartDistance;
					config.TurretDodgeStartDistance = startDist;
				}

				validConfigLoaded = true;
			}

			if (iniConfig.ContainsSection(INI_SAVED_BLOCKS_SECTION))
			{
				config.HaveSavedBlocks = true;
			}
		}
	
		if (!validConfigLoaded)
		{
			config.UniqueId = 0;
			config.GroupId = 0;
			config.SenderId = 0;
			config.NavigationStrategy = DEF_NAVIGATION_STRATEGY;
			config.PropNavConstant = DEF_PROP_NAV_CONSTANT;
			config.PropNavAccelConstant = DEF_PROP_NAV_ACCEL_CONSTANT;
			config.ProvNavMultiplierIfAccel = DEF_PROP_NAV_MULTIPLIER_IF_ACCEL;
			config.MinimumThrustAcceleration = DEF_MIN_THRUST_ACCELERATION;
			config.ThrustEstimateFactor = DEF_THRUST_ESTIMATE_FACTOR;
			config.PDGainP = DEF_PD_P_GAIN;
			config.PDGainD = DEF_PD_D_GAIN;
			config.PDAimLimit = DEF_PD_AIM_LIMIT;
			config.LaunchDelayTime = DEF_LAUNCH_DELAY_TIME;
			config.PerformFanOut = DEF_PERFORM_FAN_OUT;
			config.FanOutDurationTicks = DEF_FAN_OUT_DURATION_TICKS;
			config.PerformTurretDodge = DEF_PERFORM_TURRET_DODGE;
			config.TurretDodgeStartDistance = DEF_TURRET_DODGE_START_DISTANCE;
			config.TurretDodgeEndDistance = DEF_TURRET_DODGE_END_DISTANCE;
			config.CompensateDampener = DEF_COMPENSATE_DAMPENER;
			config.ActiveHomingEnabled = DEF_ACTIVE_HOMING_ENABLED;
			config.ExcludeFriendly = DEF_RAYCAST_EXCLUDE_FRIENDLY;
			config.ExcludeNeutral = DEF_RAYCAST_EXCLUDE_NEUTRAL;
			config.RaycastRelockNearby = DEF_RAYCAST_LOCK_NEARBY;
			config.OffsetTargeting = DEF_RAYCAST_OFFSET_TARGETING;
			config.RandomOffsetAmount = DEF_RAYCAST_RANDOM_OFFSET_AMOUNT;
			config.RaycastMinDistance = DEF_RAYCAST_MIN_DIST;
			config.RaycastMaxDistance = DEF_RAYCAST_MAX_DIST;
			config.ForwardReferenceName = string.Empty;
			config.DampenerReferenceName = string.Empty;
			config.TripwireType = DEF_TRIPWIRE_TYPE;
			config.UseInertialTripwire = DEF_USE_INERTIAL_TRIPWIRE;
			config.IdleDisableSeconds = DEF_IDLE_DISABLE_SECONDS;
			config.DetachGroupName = string.Empty;
			config.DetachBlockTag = string.Empty;
			config.DetachType = DEF_DETACH_TYPE;
		}
	}
}

void ForceJITCompilation()
{
	try
	{
		Main(FAKE, UpdateType.Update1);
		
		InitLoop(FAKE, UpdateType.Update1);
		InitVariables();
		InitConfiguration();

		GTS = null;
		
		thrusterControl = null;
		InitMissile();
		thrusterControl = null;
		InitSavedMissile();
		thrusterControl = null;

		InitThrusters(new ThrusterGroup[0]);
		InitGyroControls(new List<IMyGyro>(), ref MatrixD.Identity);
		InitPDControllers();
		InitCommsAntenna();
		InitCommunications();
		
		List<IMyCameraBlock> cameraList = new List<IMyCameraBlock>(0);
		InitRaycastTracker(cameraList);
		raycastTracker = new RaycastTracker(cameraList);
		raycastTracker.EnableAllCameras();

		GridTerminalSystem.GetBlocksOfType(cameraList, (b) => { return (cameraList.Count == 0); });
		raycastTracker.Cameras = cameraList;
		if (raycastTracker.Cameras.Count > 0)
		{
			IMyTerminalBlock fakeTargetBlock = (raycastTracker.Cameras.Count > 0 ? raycastTracker.Cameras[0] : (IMyTerminalBlock)Me);
			Vector3D fakePosition = fakeTargetBlock.WorldMatrix.Translation + (fakeTargetBlock.WorldMatrix.Forward * 0.1);
			MyDetectedEntityInfo fakeEntityInfo = new MyDetectedEntityInfo(9801, FAKE, MyDetectedEntityType.LargeGrid, fakePosition, fakeTargetBlock.WorldMatrix, Vector3D.Zero, MyRelationsBetweenPlayerAndBlock.Enemies, fakeTargetBlock.WorldAABB.Inflate(100), 1);
			raycastTracker.StartTrackingByPosition(fakePosition, 9801);
			raycastTracker.StartTrackingByVector(fakeTargetBlock, 0.1, RaycastTracker.RaycastStrategy.GHOST_CIRCULAR);
			raycastTracker.CurrentRaycastState = RaycastTracker.RaycastState.TARGET_LOCKED;
			raycastTracker.LatestRaycastEntityInfo = fakeEntityInfo;
			raycastTracker.LatestRaycastTriggeredClock = 1;
			raycastTracker.GetRaycastTarget(out targetInfo.GuessPosition, out targetInfo.Velocity, out targetInfo.EntityId);
			raycastTracker.GetRaycastHitPosition(out targetInfo.GuessTripwirePosition);
			raycastTracker.Update(2);
			raycastTracker.Update(3);
		}
		/*
		CheckSave();
		*/
		igcTargetDatalinkListener = igcTargetSwitchLostListener = igcTargetTracksListener = igcMissileCommandListener = IGC.RegisterBroadcastListener("A-B-C-DEFG-9-8-7");

		ProcessIGCTargetDatalinkMessage();
		ProcessIGCTargetSwitchLostMessage();
		ProcessIGCTargetTracksMessage();
		ProcessIGCMissileCommandMessage();
		ProcessCGETargetDatalinkMessage(CGE_MSG_TARGET_DATALINK);

		IGC.DisableBroadcastListener(igcTargetDatalinkListener);
		igcTargetDatalinkListener = igcTargetSwitchLostListener = igcTargetTracksListener = igcMissileCommandListener = null;

		StateLaunching();
		StateCommandGuidance();
		StateActiveHoming();

		proxSensor = new ProximitySensor();

		TripwireCountdown();
		CheckInertialTripwire();
		CheckProximitySensor();
		
		missileInfo.FuncDownGetForward = GetDampenerValueExtractor(Base6Directions.Direction.Forward);
		missileInfo.FuncDownGetRight = GetDampenerValueExtractor(Base6Directions.Direction.Right);

		AttemptActiveHomingSwitch();
		RefreshMissileInfo();
		RefreshExtendedTargetInfo();
		RefreshNavigationPN();
		RefreshNavigationPI();
		AimAtTarget();
		AimAtGravity();

		missileInfo.DetachBlock = null;
		missileInfo.DetachType = DetachTypeEnum.Sprue;
		
		DetachMissile();
		PopulateDetachBlock(true);
		CheckDetachSuccessful();
	}
	catch (Exception) { }

	loadedCustomDataHashCode = 0;
	Runtime.UpdateFrequency = UpdateFrequency.None;
}

void PopulateDetachBlock(bool supressErrors = false)
{
	missileInfo.DetachBlock = null;
	missileInfo.DetachType = DetachTypeEnum.Sprue;

	if (config.DetachType == DetachTypeEnum.Sprue)
	{
		return;
	}

	List<IMyTerminalBlock> blocks = new List<IMyTerminalBlock>(0);
	IMyTerminalBlock closestBlock = null;
	double closest = double.MaxValue;

	if (IsNotEmpty(config.DetachGroupName))
	{
		IMyBlockGroup group = GetGroupWithName(config.DetachGroupName);
		group?.GetBlocksOfType(blocks, (b) =>
		{
			if (IsValidDetachBlockType(b))
			{
				double dist = (Me.WorldMatrix.Translation - b.WorldMatrix.Translation).LengthSquared();
				if (dist < closest)
				{
					closestBlock = b;
					closest = dist;
				}
			}
			return false;
		});

		if (closestBlock == null)
		{
			if (!supressErrors)
			{
				Echo("Error: Cannot find suitable Detach Merge or Rotor Block in Group Named \"" + config.DetachGroupName + "\". Unable to detach missile.");
			}
		}
		else
		{
			missileInfo.DetachBlock = closestBlock;
			missileInfo.DetachType = GetDetachTypeOfBlock(closestBlock, Me.CubeGrid);
		}
	}
	else
	{
		bool haveTag = IsNotEmpty(config.DetachBlockTag);
		GTS?.GetBlocksOfType(blocks, (b) =>
		{
			if (IsValidDetachBlockType(b) && (!haveTag || NameContains(b, config.DetachBlockTag)))
			{
				double dist = (Me.WorldMatrix.Translation - b.WorldMatrix.Translation).LengthSquared();
				if (dist < closest)
				{
					closestBlock = b;
					closest = dist;
				}
			}
			return false;
		});

		if (closestBlock == null)
		{
			if (!supressErrors)
			{
				Echo("Error: Cannot find suitable Detach Merge or Rotor Block with Tag \"" + config.DetachBlockTag + "\". Unable to detach missile.");
			}
		}
		else
		{
			missileInfo.DetachBlock = closestBlock;
			missileInfo.DetachType = GetDetachTypeOfBlock(closestBlock, Me.CubeGrid);
		}
	}
}

bool IsValidDetachBlockType(IMyTerminalBlock block)
{
	DetachTypeEnum detachType = GetDetachTypeOfBlock(block, Me.CubeGrid);
	
	switch (detachType)
	{
	case DetachTypeEnum.Merge:
		return (config.DetachType == DetachTypeEnum.Standard || config.DetachType == DetachTypeEnum.Merge);
	case DetachTypeEnum.RotorBase:
		return (config.DetachType == DetachTypeEnum.Standard || config.DetachType == DetachTypeEnum.RotorBase);
	case DetachTypeEnum.RotorTop:
		return ((config.DetachType == DetachTypeEnum.Standard || config.DetachType == DetachTypeEnum.RotorTop) && ((IMyMotorStator)block).TopGrid == Me.CubeGrid);
	case DetachTypeEnum.Connector:
		return (config.DetachType == DetachTypeEnum.Standard || config.DetachType == DetachTypeEnum.Connector);
	default:
		return false;
	}
}

void InitThrusters(ThrusterGroup[] thrusterGroups)
{
	SortedList<ThrusterGroup, bool> reverseSortedTG = new SortedList<ThrusterGroup, bool>(new ThrusterGroupComparer());
	foreach (ThrusterGroup tg in thrusterGroups)
	{
		if (tg != null) reverseSortedTG.Add(tg, true);
	}
	IList<ThrusterGroup> sortedThrusterList = reverseSortedTG.Keys;

	if (missileInfo.ForwardBlock == null)
	{
		missileInfo.ForwardBlock = (sortedThrusterList.Count > 0 ? sortedThrusterList[0].Thrusters[0] : null);
		missileInfo.ForwardBlockReversed = true;

		if (missileInfo.ForwardBlock != null)
		{
			thrusterControl = new ThrusterControl(sortedThrusterList[0].Thrusters);
		}
		else
		{
			missileInfo.ForwardBlock = (missileInfo.RemoteControl == null ? Me : (IMyTerminalBlock)missileInfo.RemoteControl);
			missileInfo.ForwardBlockReversed = false;
		}
	}

	if (thrusterControl == null)
	{
		foreach (ThrusterGroup tg in sortedThrusterList)
		{
			if (tg.Thrusters[0].WorldMatrix.Forward.Dot(missileInfo.ForwardBlock.WorldMatrix.Forward) >= 0.9)
			{
				thrusterControl = new ThrusterControl(tg.Thrusters);
				break;
			}
		}
	}

	if (missileInfo.DampenerBlock == null)
	{
		for (int i = 0; i < sortedThrusterList.Count; i++)
		{
			if (Math.Abs(sortedThrusterList[i].Thrusters[0].WorldMatrix.Forward.Dot(missileInfo.ForwardBlock.WorldMatrix.Forward)) <= 0.1)
			{
				missileInfo.DampenerBlock = sortedThrusterList[i].Thrusters[0];
				break;
			}
		}
	}

	if (missileInfo.DampenerBlock != null)
	{
		MatrixD refWorldMatrix = missileInfo.ForwardBlock.WorldMatrix;
		if (missileInfo.ForwardBlockReversed)
		{
			refWorldMatrix.Forward = refWorldMatrix.Backward;
			refWorldMatrix.Left = refWorldMatrix.Right;
		}

		MatrixD viewMatrix;
		MatrixD.Transpose(ref refWorldMatrix, out viewMatrix);
		Vector3 refDownLocalVector = Vector3D.TransformNormal(missileInfo.DampenerBlock.WorldMatrix.Forward, viewMatrix);

		missileInfo.DownDirectionForward = Base6Directions.GetDirection(ref refDownLocalVector);
		missileInfo.FuncDownGetForward = GetDampenerValueExtractor(missileInfo.DownDirectionForward);

		missileInfo.DownDirectionRight = Base6Directions.GetDirection(Vector3.Forward.Cross(refDownLocalVector));
		missileInfo.FuncDownGetRight = GetDampenerValueExtractor(missileInfo.DownDirectionRight);
	}

	if (thrusterControl == null) thrusterControl = new ThrusterControl(null);
}

void InitGyroControls(List<IMyGyro> gyros, ref MatrixD refWorldMatrix)
{
	gyroControl = new GyroControl(gyros);
	gyroControl.Init(ref refWorldMatrix);
}

void InitPDControllers()
{
	yawController = new PDController(config.PDGainP, config.PDGainD, REFRESH_RATE);
	pitchController = new PDController(config.PDGainP, config.PDGainD, REFRESH_RATE);
	rollController = new PDController(config.PDGainP, config.PDGainD, REFRESH_RATE);
}

void InitCommsAntenna()
{
	if (GTS != null)
	{
		if (missileInfo.CommsAntenna != null)
		{
			if (!missileInfo.CommsAntenna.Enabled) missileInfo.CommsAntenna.Enabled = true;
			if (!missileInfo.CommsAntenna.EnableBroadcasting) missileInfo.CommsAntenna.EnableBroadcasting = true;
		}
	}
}

void InitCommunications()
{
	if (GTS != null)
	{
		//igcMsgLookup = new Dictionary<string, Action>();

		igcTargetTracksListener = IGC.RegisterBroadcastListener(IGC_MSG_TARGET_TRACKS);
		//igcTargetTracksListener.SetMessageCallback(IGC_MSG_TARGET_TRACKS);
		//igcMsgLookup.Add(IGC_MSG_TARGET_TRACKS, ProcessIGCTargetTracksMessage);

		igcTargetDatalinkListener = IGC.RegisterBroadcastListener(IGC_MSG_TARGET_DATALINK);
		//igcTargetDatalinkListener.SetMessageCallback(IGC_MSG_TARGET_DATALINK);
		//igcMsgLookup.Add(IGC_MSG_TARGET_DATALINK, ProcessIGCTargetDatalinkMessage);

		igcTargetSwitchLostListener = IGC.RegisterBroadcastListener(IGC_MSG_TARGET_SWITCH_LOST);
		//igcTargetSwitchLostListener.SetMessageCallback(IGC_MSG_TARGET_SWITCH_LOST);
		//igcMsgLookup.Add(IGC_MSG_TARGET_SWITCH_LOST, ProcessIGCTargetSwitchLostMessage);

		igcMissileCommandListener = IGC.RegisterBroadcastListener(IGC_MSG_MISSILE_COMMAND);
		//igcMissileCommandListener.SetMessageCallback(IGC_MSG_MISSILE_COMMAND);
		//igcMsgLookup.Add(IGC_MSG_MISSILE_COMMAND, ProcessIGCMissileCommandMessage);
	}
}

void InitRaycastTracker(List<IMyCameraBlock> cameras)
{
	if (cameras.Count > 0 && config.ActiveHomingEnabled)
	{
		raycastTracker = new RaycastTracker(cameras, true);
		raycastTracker.ExcludeFriendly = config.ExcludeFriendly;
		raycastTracker.ExcludeNeutral = config.ExcludeNeutral;
		raycastTracker.UseOffsetTargeting = config.OffsetTargeting;
		raycastTracker.ExcludedEntityIDs = new HashSet<long>();
		raycastTracker.ExcludedEntityIDs.Add(Me.CubeGrid.EntityId);
		raycastTracker.RelockCloseByEntityType = (config.RaycastRelockNearby ? RaycastTracker.RelockCloseByType.AFTER_RELOCK : RaycastTracker.RelockCloseByType.DISABLED);

		if (config.RandomOffsetAmount > 0)
		{
			raycastTracker.RandomOffsetAmount = (float)config.RandomOffsetAmount;
		}

		if (IsNotEmpty(config.ProbeOffsetVector))
		{
			raycastTracker.ProbeOffsetVector = Base64ByteToVector3(config.ProbeOffsetVector);
		}
	}
	else
	{
		raycastTracker = null;
		config.ActiveHomingEnabled = false;
	}
}

Vector3D ComputeBlockGridDiagonalVector(IMyTerminalBlock block)
{
	IMyCubeGrid cubeGrid = block.CubeGrid;

	Vector3D minVector = cubeGrid.GridIntegerToWorld(cubeGrid.Min);
	Vector3D maxVector = cubeGrid.GridIntegerToWorld(cubeGrid.Max);

	return (minVector - maxVector);
}

public delegate double DampenerFuncDelegate(ref Vector3D v);

double DampenerFuncUp(ref Vector3D v) { return -v.Y; }
double DampenerFuncDown(ref Vector3D v) { return v.Y; }
double DampenerFuncLeft(ref Vector3D v) { return v.X; }
double DampenerFuncRight(ref Vector3D v) { return -v.X; }
double DampenerFuncNone(ref Vector3D v) { return 0; }

DampenerFuncDelegate GetDampenerValueExtractor(Base6Directions.Direction dir)
{
	switch (dir)
	{
		case Base6Directions.Direction.Up: return DampenerFuncUp;
		case Base6Directions.Direction.Down: return DampenerFuncDown;
		case Base6Directions.Direction.Left: return DampenerFuncLeft;
		case Base6Directions.Direction.Right: return DampenerFuncRight;
	}
	return DampenerFuncNone;
}

#endregion

#region Configuration And Dianostics

void RunConfigAndDianostics(string args)
{
	string[] tokens = args.Split(':');
	if (tokens.Length > 0)
	{
		switch (tokens[0].Trim().ToUpper())
		{
		case "CHECKDETACH":
			CheckDetach(false);
			break;
		case "TESTDETACH":
			CheckDetach(true);
			break;
		case "CHECKMISSILE":
			CheckMissile(false);
			break;
		case "SAVEMISSILE":
			CheckMissile(true);
			break;
		case "CHECKSAVE":
			bool verified = CheckSave();
			if (verified)
			{
				Me.CustomName = Me.CustomName.Replace(PARTIAL_TAG, "");
				Echo("<< Missile Verified >>");
			}
			else
			{
				Echo("<< Missile Incomplete >>");
			}
			break;
	}
	}
}

void PopulateMissileBlocks()
{
	InitVariables();
	InitConfiguration();

	iniConfig = null;

	List<IMyGyro> gyros = new List<IMyGyro>();
	List<IMyCameraBlock> cameras = new List<IMyCameraBlock>();

	Vector3D gridDiagonalVector = ComputeBlockGridDiagonalVector(Me);
	ThrusterGroup[] thrusterGroups = new ThrusterGroup[6];

	bool hasForwardReference = IsNotEmpty(config.ForwardReferenceName);
	bool hasDampenerReference = IsNotEmpty(config.DampenerReferenceName);

	List<IMyTerminalBlock> blocks = new List<IMyTerminalBlock>(0);
	GTS?.GetBlocksOfType(blocks, (b) =>
	{
		if (b.CubeGrid == Me.CubeGrid)
		{
			if (b is IMyGyro)
			{
				gyros.Add(b as IMyGyro);
			}
			else if (b is IMyThrust)
			{
				IMyThrust thruster = b as IMyThrust;

				int index = (int)thruster.Orientation.Forward;
				if (thrusterGroups[index] == null)
				{
					thrusterGroups[index] = new ThrusterGroup(index, gridDiagonalVector.Dot(thruster.WorldMatrix.Forward));
					thrusterGroups[index].Thrusters = new List<IMyThrust>();
				}
				thrusterGroups[index].Thrusters.Add(thruster);
				thrusterGroups[index].TotalThrust += Math.Max(thruster.MaxEffectiveThrust, 0.00001f);
			}
			else if (b is IMyCameraBlock)
			{
				if (config.ActiveHomingEnabled)
				{
					cameras.Add(b as IMyCameraBlock);
				}

				if (b.CustomName.IndexOf(PROXIMITY_CAMERA_TAG, StringComparison.OrdinalIgnoreCase) > -1)
				{
					if (proxSensor == null)
					{
						proxSensor = new ProximitySensor();
						proxSensor.ExcludeFriendly = config.ExcludeFriendly;
						proxSensor.ExcludeNeutral = config.ExcludeNeutral;
					}
					proxSensor.AddCamera(b as IMyCameraBlock);
				}
			}
			else if (b is IMyRemoteControl)
			{
				missileInfo.RemoteControl = b as IMyRemoteControl;
			}
			else if (b is IMyRadioAntenna)
			{
				missileInfo.CommsAntenna = b as IMyRadioAntenna;
			}
			else if (b is IMyPowerProducer)
			{
				if (missileInfo.PowerBlocks == null)
				{
					missileInfo.PowerBlocks = new List<IMyPowerProducer>();
				}
				missileInfo.PowerBlocks.Add(b as IMyPowerProducer);
			}

			if (hasForwardReference && missileInfo.ForwardBlock == null)
			{
				if (NameContains(b, config.ForwardReferenceName))
				{
					missileInfo.ForwardBlock = b;
					missileInfo.ForwardBlockReversed = (b is IMyThrust);
				}
			}

			if (hasDampenerReference && missileInfo.DampenerBlock == null && b is IMyThrust)
			{
				if (NameContains(b, config.DampenerReferenceName))
				{
					missileInfo.DampenerBlock = b;
				}
			}
		}
		return false;
	});

	InitThrusters(thrusterGroups);

	MatrixD refWorldMatrix = missileInfo.ForwardBlock.WorldMatrix;
	if (missileInfo.ForwardBlockReversed)
	{
		refWorldMatrix.Forward = refWorldMatrix.Backward;
		refWorldMatrix.Left = refWorldMatrix.Right;
	}

	InitGyroControls(gyros, ref refWorldMatrix);

	InitRaycastTracker(cameras);
}

void CheckDetach(bool detach = false)
{
	InitVariables();
	InitConfiguration();

	PopulateDetachBlock(true);

	Echo("----- Missile Detach -----\n");

	if (IsNotEmpty(config.DetachGroupName))
	{
		Echo("[By Detach Group]");
		Echo("=> " + config.DetachGroupName + "\n");
	}
	else if (IsNotEmpty(config.DetachBlockTag))
	{
		Echo("[By Detach Name Tag]");
		Echo("=> " + config.DetachBlockTag + "\n");
	}

	if (missileInfo.DetachBlock != null)
	{
		Echo("Missile Detach Block:\n=> " + missileInfo.DetachBlock.CustomName);
		if (detach)
		{
			if (missileInfo.DetachBlock is IMyShipMergeBlock)
			{
				((IMyShipMergeBlock)missileInfo.DetachBlock).Enabled = false;
			}
			else if (missileInfo.DetachBlock is IMyMotorStator)
			{
				((IMyMotorStator)missileInfo.DetachBlock).Detach();
			}
		}
	}
	else
	{
		Echo("Warning: No Detach Block Found. Detach is assumed to be Sprue");
	}
}

void CheckMissile(bool save = false)
{
	PopulateMissileBlocks();

	Echo("----- Missile Issues -----");

	bool haveIssues = false;

	if (gyroControl.Gyroscopes == null || gyroControl.Gyroscopes.Count == 0)
	{
		haveIssues = true;
		Echo("\n=> No Gyroscopes");
	}

	if (thrusterControl.Thrusters == null || thrusterControl.Thrusters.Count == 0)
	{
		haveIssues = true;
		Echo("\n=> No Thrusters");
	}

	if (missileInfo.RemoteControl == null)
	{
		haveIssues = true;
		Echo("\n=> No Remote Control");
	}

	if (missileInfo.CommsAntenna == null)
	{
		Echo("\n=> No Antenna");
	}

	Echo("\n----- Missile Parameters -----");

	if (thrusterControl.Thrusters != null && thrusterControl.Thrusters.Count > 0)
	{
		Echo("\nOne of the Forward Thrusters:\n=> " + thrusterControl.Thrusters[0].CustomName);
	}

	if (missileInfo.DampenerBlock != null)
	{
		Echo("\nOne of the Gravity Dampeners:\n=> " + missileInfo.DampenerBlock.CustomName);
	}

	Echo("\nFire And Forget: " + StrYesNo(config.ActiveHomingEnabled));
	Echo("\nProximity Tripwire: " + StrYesNo(proxSensor != null));
	Echo("\nInertial Tripwire: " + StrYesNo(config.UseInertialTripwire));
	Echo("\nTripwire Type: " + (config.TripwireType == 1 ? "Arm & Countdown" : "Detonate"));

	PopulateDetachBlock(true);
	if (missileInfo.DetachBlock != null && missileInfo.DetachBlock.CubeGrid != Me.CubeGrid)
	{
		missileInfo.DetachBlock = null;
		missileInfo.DetachType = DetachTypeEnum.Sprue;
	}

	Echo("\nMissile Detach Block:\n=> " + (missileInfo.DetachBlock?.CustomName ?? "<None>"));

	if (save)
	{
		Echo("\n----- Save Configuration -----");

		if (haveIssues)
		{
			Echo("\nError : Unable to Save due to Missile Issues");
		}
		else
		{
			Vector3I[] vec = { -Base6Directions.GetIntVector(Me.Orientation.Left), Base6Directions.GetIntVector(Me.Orientation.Up), -Base6Directions.GetIntVector(Me.Orientation.Forward) };
			Vector3I[] inv = { new Vector3I(vec[0].X, vec[1].X, vec[2].X), new Vector3I(vec[0].Y, vec[1].Y, vec[2].Y), new Vector3I(vec[0].Z, vec[1].Z, vec[2].Z) };

			iniConfig = new MyIni();
			iniConfig.TryParse(Me.CustomData);

			iniConfig.Set(INI_SAVED_BLOCKS_SECTION, "DetachBlock", BlockBytePosToBase64(missileInfo.DetachBlock, Me, ref inv));
			iniConfig.Set(INI_SAVED_BLOCKS_SECTION, "CommsAntenna", BlockBytePosToBase64(missileInfo.CommsAntenna, Me, ref inv));
			iniConfig.Set(INI_SAVED_BLOCKS_SECTION, "DampenerBlock", BlockBytePosToBase64(missileInfo.DampenerBlock, Me, ref inv));
			iniConfig.Set(INI_SAVED_BLOCKS_SECTION, "ForwardBlock", BlockBytePosToBase64(missileInfo.ForwardBlock, Me, ref inv));
			iniConfig.Set(INI_SAVED_BLOCKS_SECTION, "ForwardBlockReversed", missileInfo.ForwardBlockReversed);
			iniConfig.Set(INI_SAVED_BLOCKS_SECTION, "DownDirectionForward", missileInfo.DownDirectionForward.ToString());
			iniConfig.Set(INI_SAVED_BLOCKS_SECTION, "DownDirectionRight", missileInfo.DownDirectionRight.ToString());
			iniConfig.Set(INI_SAVED_BLOCKS_SECTION, "RemoteControl", BlockBytePosToBase64(missileInfo.RemoteControl, Me, ref inv));
			iniConfig.Set(INI_SAVED_BLOCKS_SECTION, "Gyroscopes", BlockListBytePosToBase64(gyroControl.Gyroscopes, Me, ref inv));
			iniConfig.Set(INI_SAVED_BLOCKS_SECTION, "Thrusters", BlockListBytePosToBase64(thrusterControl.Thrusters, Me, ref inv));
			iniConfig.Set(INI_SAVED_BLOCKS_SECTION, "PowerBlocks", BlockListBytePosToBase64(missileInfo.PowerBlocks, Me, ref inv));
			iniConfig.Set(INI_SAVED_BLOCKS_SECTION, "RaycastCameras", BlockListBytePosToBase64(raycastTracker?.Cameras, Me, ref inv));

			Me.CustomData = iniConfig.ToString();

			Echo("\n<< Save Missile Successful >>");
		}
	}

	missileInfo = new MissileInformation();
}

bool CheckSave()
{
	Vector3I[] vec = { -Base6Directions.GetIntVector(Me.Orientation.Left), Base6Directions.GetIntVector(Me.Orientation.Up), -Base6Directions.GetIntVector(Me.Orientation.Forward) };

	iniConfig = new MyIni();
	iniConfig.TryParse(Me.CustomData);
	if (!CheckConfigSavedBlockList("DetachBlock", ref vec, true)) return false;
	if (!CheckConfigSavedBlockList("CommsAntenna", ref vec, false))
	{
		Echo("Warning: No Antenna detected.\n[CGE] name tag in PB required.\n");
	}
	if (!CheckConfigSavedBlockList("DampenerBlock", ref vec, true)) return false;
	if (!CheckConfigSavedBlockList("ForwardBlock", ref vec, false)) return false;
	if (!CheckConfigSavedBlockList("RemoteControl", ref vec, false)) return false;
	if (!CheckConfigSavedBlockList("Gyroscopes", ref vec, false)) return false;
	if (!CheckConfigSavedBlockList("Thrusters", ref vec, false)) return false;
	if (!CheckConfigSavedBlockList("PowerBlocks", ref vec, true)) return false;
	if (!CheckConfigSavedBlockList("RaycastCameras", ref vec, true)) return false;

	return true;
}

bool CheckConfigSavedBlockList(string key, ref Vector3I[] vec, bool acceptEmpty)
{
	string[] values = iniConfig.Get(INI_SAVED_BLOCKS_SECTION, key).ToString().Split(delimiters, StringSplitOptions.RemoveEmptyEntries);
	return ((values.Length > 0 || acceptEmpty) && VerifyBlocksExist(values, Me, ref vec));
}

bool VerifyBlocksExist(string[] input, IMyTerminalBlock origin, ref Vector3I[] vec)
{
	foreach (string line in input)
	{
		if (line != null && line.Length == 12)
		{
			Vector3I result = new Vector3I();
			result.X = BitConverter.ToInt16(Convert.FromBase64String(line.Substring(0, 4)), 0);
			result.Y = BitConverter.ToInt16(Convert.FromBase64String(line.Substring(4, 4)), 0);
			result.Z = BitConverter.ToInt16(Convert.FromBase64String(line.Substring(8, 4)), 0);

			result = (result.X * vec[0]) + (result.Y * vec[1]) + (result.Z * vec[2]);
			result += origin.Position;

			if (!origin.CubeGrid.CubeExists(result))
			{
				return false;
			}
		}
		else
		{
			return false;
		}
	}
	return true;
}

#endregion

#region Helper Methods

private double CalculateThrustAcceleration(IMyRemoteControl remoteControl, List<IMyThrust> thrusters)
{
	if (remoteControl == null || thrusters == null)
	{
		return config.MinimumThrustAcceleration;
	}
	
	float totalThrust = 0;
	foreach (IMyThrust thruster in thrusters)
	{
		totalThrust += thruster.MaxEffectiveThrust;
	}
        
	double thrustAcceleration = totalThrust / remoteControl.CalculateShipMass().TotalMass;
	if (double.IsNaN(thrustAcceleration) || double.IsInfinity(thrustAcceleration) || thrustAcceleration < config.MinimumThrustAcceleration)
	{
		return config.MinimumThrustAcceleration;
	}
	else
	{
		return thrustAcceleration * config.ThrustEstimateFactor;
	}
}

double FastAT(double x)
{
	//Removed Math.Abs() since x is always positive in this script
	return 0.785375 * x - x * (x - 1.0) * (0.2447 + 0.0663 * x);
}

string BlockListBytePosToBase64<T>(List<T> blocks, IMyTerminalBlock origin, ref Vector3I[] vec) where T: class, IMyTerminalBlock
{
	if (blocks == null || blocks.Count == 0)
	{
		return "";
	}
	else
	{
		StringBuilder sb = new StringBuilder(blocks.Count * 9);
		foreach (IMyTerminalBlock block in blocks)
		{
			sb.Append(BlockBytePosToBase64(block, origin, ref vec)).Append(delimiters[0]);
		}
		return sb.ToString();
	}
}

string BlockBytePosToBase64(IMyTerminalBlock block, IMyTerminalBlock origin, ref Vector3I[] vec)
{
	if (block == null)
	{
		return "";
	}
	else
	{
		Vector3I input = block.Position - origin.Position;
		input = (input.X * vec[0]) + (input.Y * vec[1]) + (input.Z * vec[2]);

		return Convert.ToBase64String(BitConverter.GetBytes((short)(input.X))) +
			Convert.ToBase64String(BitConverter.GetBytes((short)(input.Y))) +
			Convert.ToBase64String(BitConverter.GetBytes((short)(input.Z)));
	}
}

bool Base64BytePosToBlockList<T>(string input, IMyTerminalBlock origin, ref Vector3I[] vec, out List<T> result) where T: class, IMyTerminalBlock
{
	if (input == null)
	{
		result = null;
		return true;
	}

	string[] values = input.Split(delimiters, StringSplitOptions.RemoveEmptyEntries);
	result = new List<T>(values.Length);

	foreach (string line in values)
	{
		if (line.Length == 12)
		{
			IMySlimBlock slim = origin.CubeGrid.GetCubeBlock(Base64ByteToVector3I(line, Me, ref vec));
			if (slim != null)
			{
				T block =  slim.FatBlock as T;
				if (block != null)
				{
					result.Add(block);
				}
				else
				{
					return false;
				}
			}
			else
			{
				return false;
			}
		}
	}

	return true;
}

bool Base64BytePosToBlock<T>(string input, IMyTerminalBlock origin, ref Vector3I[] vec, out T result) where T: class, IMyTerminalBlock
{
	if (input != null && input.Length == 12)
	{
		IMySlimBlock slim = origin.CubeGrid.GetCubeBlock(Base64ByteToVector3I(input, Me, ref vec));
		if (slim != null)
		{
			result = slim.FatBlock as T;
			if (result != null) return true;
		}
		result = null;
		return false;
	}
	else
	{
		result = null;
		return true;
	}
}

Vector3I Base64ByteToVector3I(string input, IMyTerminalBlock origin, ref Vector3I[] vec)
{
	Vector3I result = new Vector3I();
	if (input != null && input.Length == 12)
	{
		result.X = BitConverter.ToInt16(Convert.FromBase64String(input.Substring(0, 4)), 0);
		result.Y = BitConverter.ToInt16(Convert.FromBase64String(input.Substring(4, 4)), 0);
		result.Z = BitConverter.ToInt16(Convert.FromBase64String(input.Substring(8, 4)), 0);

		result = (result.X * vec[0]) + (result.Y * vec[1]) + (result.Z * vec[2]);
		result += origin.Position;
	}
	return result;
}

Vector3? Base64ByteToVector3(string input)
{
	if (input != null && input.Length == 24)
	{
		Vector3 result = new Vector3();

		result.X = BitConverter.ToSingle(Convert.FromBase64String(input.Substring(0, 8)), 0);
		result.Y = BitConverter.ToSingle(Convert.FromBase64String(input.Substring(8, 8)), 0);
		result.Z = BitConverter.ToSingle(Convert.FromBase64String(input.Substring(16, 8)), 0);

		return result;
	}

	return null;
}

string StrYesNo(bool yes)
{
	return (yes ? "Yes" : "No");
}

IMyBlockGroup GetGroupWithName(string name)
{
	List<IMyBlockGroup> groups = new List<IMyBlockGroup>(2);
	GTS?.GetBlockGroups(groups, (b) => { return b.Name.IndexOf(name, StringComparison.OrdinalIgnoreCase) > -1; });
	return (groups.Count > 0 ? groups[0] : null);
}

DetachTypeEnum GetDetachTypeFromString(string detachTypeStr)
{
	if (IsNotEmpty(detachTypeStr))
	{
		if (StringEqualsIgnoreCase(detachTypeStr, "Merge"))
		{
			return DetachTypeEnum.Merge;
		}
		else if (StringEqualsIgnoreCase(detachTypeStr, "RotorTop") || StringEqualsIgnoreCase(detachTypeStr, "Rotor"))
		{
			return DetachTypeEnum.RotorTop;
		}
		else if (StringEqualsIgnoreCase(detachTypeStr, "RotorBase"))
		{
			return DetachTypeEnum.RotorBase;
		}
		else if (StringEqualsIgnoreCase(detachTypeStr, "Connector"))
		{
			return DetachTypeEnum.Connector;
		}
		else if (StringEqualsIgnoreCase(detachTypeStr, "Sprue"))
		{
			return DetachTypeEnum.Sprue;
		}
		else
		{
			return DetachTypeEnum.Standard;
		}
	}
	else
	{
		return DEF_DETACH_TYPE;
	}
}

DetachTypeEnum GetDetachTypeOfBlock(IMyTerminalBlock block, IMyCubeGrid missileGrid)
{
	if (block is IMyShipMergeBlock)
	{
		if (block.CubeGrid == missileGrid)
		{
			return DetachTypeEnum.Merge;
		}
	}
	else if (block is IMyMotorStator)
	{
		return (block.CubeGrid == missileGrid ? DetachTypeEnum.RotorBase : DetachTypeEnum.RotorTop);
	}
	else if (block is IMyShipConnector)
	{
		if (block.CubeGrid == missileGrid)
		{
			return DetachTypeEnum.Connector;
		}
	}
	return DetachTypeEnum.Sprue;
}

bool IsNotEmpty(string line)
{
	return (line != null && line.Length > 0);
}

bool NameContains(IMyTerminalBlock block, string nameTag)
{
	return (block.CustomName.IndexOf(nameTag, StringComparison.OrdinalIgnoreCase) > -1);
}

bool StringEqualsIgnoreCase(string input, string compare)
{
	return input.Equals(compare, StringComparison.OrdinalIgnoreCase);
}

#endregion

#region Helper Enums

public enum AGMLaunchOptionEnum
{
	OffsetTargeting = 1
}

public enum DetachTypeEnum
{
	Standard = 0,
	Merge = 1,
	RotorTop = 2,
	RotorBase = 3,
	Connector = 4,
	Sprue = 5
}

#endregion

#region Helper Classes

public class ScriptConfiguration
{
	public long UniqueId;
	public long GroupId;
	public long SenderId;

	public string ProbeOffsetVector;

	public int NavigationStrategy;

	public double PropNavConstant;
	public double PropNavAccelConstant;
	public double ProvNavMultiplierIfAccel;

	public double MinimumThrustAcceleration;
	public double ThrustEstimateFactor;

	public double PDGainP;
	public double PDGainD;
	public double PDAimLimit;

	public double LaunchDelayTime;
	public bool CompensateDampener;

	public bool PerformFanOut;
	public int FanOutDurationTicks;
	public bool PerformTurretDodge;
	public double TurretDodgeStartDistance;
	public double TurretDodgeEndDistance;

	public bool ActiveHomingEnabled;
	public bool ExcludeFriendly;
	public bool ExcludeNeutral;
	public bool RaycastRelockNearby;

	public bool OffsetTargeting;
	public double RandomOffsetAmount;

	public double RaycastMinDistance;
	public double RaycastMaxDistance;

	public string ForwardReferenceName;
	public string DampenerReferenceName;

	public int TripwireType;
	public bool UseInertialTripwire;

	public double IdleDisableSeconds;

	public bool HaveSavedBlocks;

	public string DetachGroupName;
	public string DetachBlockTag;
	public DetachTypeEnum DetachType;
}

public class MissileInformation
{
	public int SubState;
	public int SubCounter;

	public int ActiveSwitchCount;

	public IMyTerminalBlock DetachBlock;
	public DetachTypeEnum DetachType;

	public IMyRemoteControl RemoteControl;

	public IMyRadioAntenna CommsAntenna;

	public IMyTerminalBlock ForwardBlock;
	public bool ForwardBlockReversed;

	public IMyTerminalBlock DampenerBlock;
	public Base6Directions.Direction DownDirectionForward;
	public Base6Directions.Direction DownDirectionRight;
	public DampenerFuncDelegate FuncDownGetForward;
	public DampenerFuncDelegate FuncDownGetRight;

	public List<IMyPowerProducer> PowerBlocks;

	public Vector3D Position;
	public Vector3D Velocity;

	public MatrixD ViewMatrix;
	public Vector3D TargetVector;

	public Vector3D NaturalGravity;
	public double NaturalGravityLength;

	public double MissileAcceleration;
	public int NextThrustEstimate;

	public double MissileRadius;
	public double Speed;

	public bool IsAccelerating;
	public bool IsApproaching;
	public bool IsSpinning;

	public int NextUpdateNaturalGravity;

	public int TripwireClock;
}

public class TargetInformation
{
	public long EntityId;
	public Vector3D Position;
	public Vector3D Velocity;

	public Vector3D GuessPosition;
	public Vector3D GuessAcceleration;
	public Vector3D GuessTripwirePosition;
	public Vector3D PreviousVelocity;
	public int PreviousVelocityClock;

	public int LastDetectedClock;

	public Vector3D RangeVector;
	public Vector3D RelativeVelocity;
}

public class ProximitySensor
{
	List<ProxCamera> proxCameras;

	public bool ExcludeFriendly;
	public bool ExcludeNeutral;

	public ProximitySensor()
	{
		proxCameras = new List<ProxCamera>();
	}

	public void AddCamera(IMyCameraBlock camera)
	{
		ProxCamera prox = new ProxCamera();
		prox.Camera = camera;

		bool hasCustomExtend = false;

		if (camera.CustomData.Length > 0)
		{
			MyIni config = new MyIni();
			if (config.TryParse(camera.CustomData))
			{
				prox.Extend = config.Get(PROXIMITY_CAMERA_TAG, "Extend").ToDouble(0);

				hasCustomExtend = true;
			}
		}

		if (!hasCustomExtend)
		{
			Vector3I localForward = Base6Directions.GetIntVector(camera.Orientation.Forward);
			int sumForward = localForward.X + localForward.Y + localForward.Z;

			localForward *= (sumForward < 0 ? camera.CubeGrid.Min : camera.CubeGrid.Max) - camera.Position;
			sumForward = localForward.X + localForward.Y + localForward.Z;

			if (sumForward > 0)
			{
				prox.Extend = sumForward * camera.CubeGrid.GridSize;
			}
		}

		proxCameras.Add(prox);
	}

	public bool CheckProximity(double distance, long includeEntityId)
	{
		foreach (ProxCamera prox in proxCameras)
		{
			if (prox.Camera.IsWorking)
			{
				MyDetectedEntityInfo detected = prox.Camera.Raycast(distance + prox.Extend);
				if (detected.EntityId != 0 && (detected.EntityId == includeEntityId || IsValidTarget(ref detected)))
				{
					return true;
				}
			}
		}
		return false;
	}

	bool IsValidTarget(ref MyDetectedEntityInfo detected)
	{
		if (detected.Type == MyDetectedEntityType.LargeGrid || detected.Type == MyDetectedEntityType.SmallGrid)
		{
			switch (detected.Relationship)
			{
				case MyRelationsBetweenPlayerAndBlock.Owner:
				case MyRelationsBetweenPlayerAndBlock.FactionShare:
					return !ExcludeFriendly;
				case MyRelationsBetweenPlayerAndBlock.Neutral:
				case MyRelationsBetweenPlayerAndBlock.NoOwnership:
					return !ExcludeNeutral;
				default:
					return true;
			}
		}
		return false;
	}

	class ProxCamera
	{
		public IMyCameraBlock Camera;
		public double Extend;
	}
}

public class RaycastTracker
{
	//Minimum number of ticks the script must wait before using each raycast. This is for performance purposes
	public const int MINIMUM_RAYCAST_TICKS = 6;

	//Number of ticks to check raycast again if no cameras are usable
	public const int RAYCAST_COOLDOWN_RECHECK_TICKS = 12;

	//Number of ticks before refreshing nextRaycastReadyRefresh
	public const int RAYCAST_READY_REFRESH_TICKS = 30;

	//Maximum Number of ticks allowed for a slipped target before declaring lock is lost
	public const int SLIP_LOST_THRESHOLD_TICKS = 120;

	//Threshold number of relock-on offset raycast hits to switch to use offset raycasting permanently
	public const int USE_OFFSET_THRESHOLD_HIT_COUNT = 5;

	//Number of raycsat probe tests before probe converges to center point
	public const int PROBE_CONVERGE_MAX_COUNT = 25;

	//Whether to exclude friendly targets
	public bool ExcludeFriendly { get; set; } = true;

	//Whether to exclude neutral targets
	public bool ExcludeNeutral { get; set; } = false;

	//Set of entity ids to prevent locking-on to
	public HashSet<long> ExcludedEntityIDs { get; set; }

	//Minimum target cross section to lock-on as valid target
	public float MinimumCrossSection { get; set; } = 0f;

	//Whether to lock-on to an offset position instead of center point of the target
	public bool UseOffsetTargeting { get; set; } = true;

	public float RandomOffsetAmount { get; set; } = 0f;

	//Probe for valid offset spot from this target's local vector to the target's center. If a valid offset is found, it is copied to OffsetTargetVector and this will be cleared
	public Vector3D? ProbeOffsetVector
	{
		get
		{
			return probeOffsetVector;
		}
		set
		{
			probeOffsetVector = value;
			probeOffsetTriggeredCount = 0;
		}
	}
	int probeOffsetTriggeredCount;
	Vector3D? probeOffsetVector;

	//Percentage of the estimated available raycast to utilize. The remaining are reserved for emergency relock-on operations
	public float RaycastUsagePercent { get; set; } = 0.9f;

	//Percentage of the estimated reserved raycast to utilize for relock-on operations
	public float RelockUsagePercent { get; set; } = 0.5f;

	//Maximum frames to relock-on (1 frame is 8 points. Example 3 frames = Max 24 raycast to spam for relock-on)
	public int RelockMaxFrameDepth { get; set; } = 3;

	//Whether to allow lock-on to a close by target if initial raycast or relock failed to aquire same target
	public RelockCloseByType RelockCloseByEntityType { get; set; } = RelockCloseByType.AFTER_RELOCK;

	//Latest raycast info returned from the camera raycast method
	public MyDetectedEntityInfo LatestRaycastEntityInfo { get; set; }

	//Clock value of the last triggering of a raycast operation
	public int LatestRaycastTriggeredClock { get; set; } = -1;

	//Clock value of the first lock on via start tracking methods
	public int FirstRaycastLockedClock { get; set; } = -1;

	//Clock value of when the next raycast will be ready
	public int NextRaycastReadyClock { get; set; }

	//Current state of the raycast tracking
	public RaycastState CurrentRaycastState { get; set; }

	//The offset vector relative to the target's orientation. Used for offset targeting
	public Vector3D OffsetTargetVector { get; set; }

	//Tag object for external custom use
	public object Tag { get; set; }

	//Cameras to be used for raycast lock-on
	List<IMyCameraBlock> m_cameras;
	public List<IMyCameraBlock> Cameras
	{
		get { return m_cameras; }
		set
		{
			m_cameras = value;

			NextRaycastReadyClock = 0;

			CompileCameraGroups();

			if (m_cameras.Count > 0)
			{
				gridReference = m_cameras[0].CubeGrid;
			}
		}
	}
	List<CameraGroup> m_cameraGroups;

	readonly int[,] relockPattern = new int[,] {{1,0},{-1,0},{0,1},{0,-1},{-1,-1},{1,-1},{-1,1},{1,1}};
	readonly Random rnd = new Random();

	//Radius of the target
	double targetRadius;

	//Adjustment to raycast for a longer distance to account for target radius
	double raycastRangeFactor;

	//The offset vector relative to the target's orientation. Used for offset raycasting
	Vector3D offsetRaycastVector;
	bool useOffsetRaycasting;

	//Clock value of when the next raycast ready clock and nextRaycastDelay will be refreshed
	int nextRaycastReadyRefresh;

	//Estimated number of ticks to wait for next raycast
	int nextRaycastDelay;

	//Estimated number of usable charges reserved
	int reservedRaycastCharges;

	//Temp values for non default RaycastStrategy for initial lock-on
	int t_AimReserves;
	Vector3D t_AimVector;
	int t_AimClock;

	//Temp values for threshold of switching to offset raycasting
	int t_offsetHitCount;

	//For remembering previously used CameraGroup
	public CameraGroup currentCameraGroup;

	//For quick working camera reference
	IMyCubeGrid gridReference;

	//Precalculated scalars for checking raycastable camera angles
	float coneLimit;
	double sideScale;
	bool noFwdScale;

	//Number of ticks to charge 1 meter of raycast per camera
	float ticksRatio;

	//Internal clock
	int clock;

	public const float TICK_DELTA = 1f / 60f;

	//Instantiate new class with the predefined set of cameras
	public RaycastTracker(List<IMyCameraBlock> cameras, bool enableAllCameras = true)
	{
		if (cameras.Count > 0)
		{
			ticksRatio = cameras[0].TimeUntilScan(cameras[0].AvailableScanRange + 1000) * 0.00006f;

			coneLimit = cameras[0].RaycastConeLimit;
			if (coneLimit == 0f || coneLimit == 180f) sideScale = double.NaN;
			else sideScale = Math.Tan(MathHelper.ToRadians(90 - coneLimit));

			noFwdScale = double.IsNaN(sideScale) || double.IsInfinity(sideScale);
			if (noFwdScale) sideScale = 1;
		}
		else
		{
			//Default value of 60 ticks per 2km (1 sec = 60 ticks)
			ticksRatio = 0.03f;

			coneLimit = 45;
			sideScale = 1;
			noFwdScale = false;
		}

		Cameras = cameras;
		if (enableAllCameras)
		{
			EnableAllCameras();
		}

		MyMath.InitializeFastSin();
	}

	private void CompileCameraGroups()
	{
		if (coneLimit <= 0 || coneLimit >= 180)
		{
			m_cameraGroups = new List<CameraGroup>();
			return;
		}

		Dictionary<string, CameraGroup> cameraGroupsLookup = new Dictionary<string, CameraGroup>();

		foreach (IMyCameraBlock camera in m_cameras)
		{
			string key = camera.CubeGrid.EntityId.ToString() + "-" + ((int)camera.Orientation.Forward).ToString();

			CameraGroup cameraGroup;
			if (cameraGroupsLookup.ContainsKey(key))
			{
				cameraGroup = cameraGroupsLookup[key];
			}
			else
			{
				cameraGroup = new CameraGroup();
				cameraGroup.Cameras = new List<IMyCameraBlock>();
				cameraGroup.SideScale = sideScale;
				cameraGroup.NoFwdScale = noFwdScale;

				cameraGroupsLookup[key] = cameraGroup;
			}

			cameraGroup.Cameras.Add(camera);
		}

		m_cameraGroups = cameraGroupsLookup.Values.ToList();

		foreach (CameraGroup cameraGroup in m_cameraGroups)
		{
			cameraGroup.Grid = cameraGroup.Cameras[0].CubeGrid;

			int minX = int.MaxValue, maxX = int.MinValue, minY = int.MaxValue, maxY = int.MinValue, minZ = int.MaxValue, maxZ = int.MinValue;

			foreach (IMyCameraBlock camera in cameraGroup.Cameras)
			{
				minX = Math.Min(minX, camera.Position.X);
				maxX = Math.Max(maxX, camera.Position.X);
				minY = Math.Min(minY, camera.Position.Y);
				maxY = Math.Max(maxY, camera.Position.Y);
				minZ = Math.Min(minZ, camera.Position.Z);
				maxZ = Math.Max(maxZ, camera.Position.Z);
			}

			Base6Directions.Direction gridUp = cameraGroup.Grid.WorldMatrix.GetClosestDirection(cameraGroup.Cameras[0].WorldMatrix.Up);
			Base6Directions.Direction gridLeft = cameraGroup.Grid.WorldMatrix.GetClosestDirection(cameraGroup.Cameras[0].WorldMatrix.Left);
			Base6Directions.Direction gridForward = cameraGroup.Grid.WorldMatrix.GetClosestDirection(cameraGroup.Cameras[0].WorldMatrix.Forward);

			cameraGroup.DirectionUp = CameraGroup.GetDirectionFunction(gridUp);
			cameraGroup.DirectionLeft = CameraGroup.GetDirectionFunction(gridLeft);
			cameraGroup.DirectionForward = CameraGroup.GetDirectionFunction(gridForward);

			cameraGroup.PointUp = CameraGroup.GetReferencePoint(gridUp, minX, maxX, minY, maxY, minZ, maxZ);
			cameraGroup.PointDown = CameraGroup.GetReferencePoint(Base6Directions.GetOppositeDirection(gridUp), minX, maxX, minY, maxY, minZ, maxZ);
			cameraGroup.PointLeft = CameraGroup.GetReferencePoint(gridLeft, minX, maxX, minY, maxY, minZ, maxZ);
			cameraGroup.PointRight = CameraGroup.GetReferencePoint(Base6Directions.GetOppositeDirection(gridLeft), minX, maxX, minY, maxY, minZ, maxZ);
		}
	}

	public void EnableAllCameras()
	{
		if (m_cameras != null)
		{
			foreach (IMyCameraBlock camera in m_cameras)
			{
				if (!camera.Enabled) camera.Enabled = true;
				if (!camera.EnableRaycast) camera.EnableRaycast = true;
			}
		}
	}

	//Start tracking the target using reference from aimingBlock and specified distance for initial lock-on. If aimingBlock is a turret, its azimuth and elevation is used instead
	public RaycastResult StartTrackingByVector(IMyTerminalBlock aimingBlock, double distance, RaycastStrategy raycastStrategy = RaycastStrategy.DEFAULT, int extraRaycastCount = 3, double raycastModValue = 0, bool bypassRefresh = false)
	{
		MatrixD worldMatrix = aimingBlock.WorldMatrix;

		if (aimingBlock is IMyLargeTurretBase)
		{
			IMyLargeTurretBase turret = aimingBlock as IMyLargeTurretBase;

			Vector3D direction;
			Vector3D.CreateFromAzimuthAndElevation(turret.Azimuth, turret.Elevation, out direction);

			direction = Vector3D.TransformNormal(direction, worldMatrix);

			return StartTrackingByVector(worldMatrix.Translation, direction, distance, raycastStrategy, extraRaycastCount, raycastModValue, bypassRefresh);
		}
		else if (aimingBlock is IMyThrust)
		{
			return StartTrackingByVector(worldMatrix.Translation, -worldMatrix.Forward, distance, raycastStrategy, extraRaycastCount, raycastModValue, bypassRefresh);
		}
		else
		{
			return StartTrackingByVector(worldMatrix.Translation, worldMatrix.Forward, distance, raycastStrategy, extraRaycastCount, raycastModValue, bypassRefresh);
		}
	}
	//Start tracking the target using world direction provided by aimStartPosition and aimForwardVector plus specified distance for initial lock-on. extraRaycastCount and raycastModValue is ONLY used when raycastStrategy is NOT default
	public RaycastResult StartTrackingByVector(Vector3D aimStartPosition, Vector3D aimForwardVector, double distance, RaycastStrategy raycastStrategy = RaycastStrategy.DEFAULT, int extraRaycastCount = 3, double raycastModValue = 0, bool bypassRefresh = false)
	{
		if (CurrentRaycastState != RaycastState.NO_TARGET)
		{
			StopTracking();
		}

		if (clock >= NextRaycastReadyClock || bypassRefresh)
		{
			Vector3D targetPosition = aimStartPosition + (aimForwardVector * distance);

			RefreshRaycastReadyClock(ref targetPosition, 1, (raycastStrategy == RaycastStrategy.DEFAULT ? 1 : 2));

			IMyCameraBlock camera;
			if (GetRaycastable(ref targetPosition, 1, out camera, out targetPosition))
			{
				MyDetectedEntityInfo entityInfo = Raycast(camera, ref targetPosition);
				if (!entityInfo.IsEmpty())
				{
					if (ValidTarget(ref entityInfo))
					{
						//Raycast successful
						SetInitialRaycastTargetInfo(ref entityInfo);

						return RaycastResult.RAYCAST_SUCCESS;
					}
					else
					{
						return RaycastResult.NO_TARGET_FOUND;
					}
				}
				else if (raycastStrategy != RaycastStrategy.DEFAULT)
				{
					if (t_AimReserves >= extraRaycastCount)
					{
						t_AimReserves = 0;

						if (extraRaycastCount < 1) extraRaycastCount = 1;

						List<Vector3D> testPositions = new List<Vector3D>(extraRaycastCount);
						if (raycastStrategy == RaycastStrategy.GHOST_AHEAD)
						{
							double dot = aimForwardVector.Dot(ref t_AimVector);
							if (dot > -1 && dot < 1)
							{
								if (raycastModValue <= 0) raycastModValue = 0.5;

								Vector3D aimSideVector = (dot * aimForwardVector) - t_AimVector;
								double angle = FastAC(dot);
								aimSideVector /= MyMath.FastSin((float)angle);
								angle = angle / ((clock - t_AimClock) * extraRaycastCount) * 60f * raycastModValue;
								for (int i = 1; i <= extraRaycastCount; i++)
								{
									testPositions.Add(aimStartPosition + (((aimForwardVector * MyMath.FastCos((float)angle * i)) + (aimSideVector * MyMath.FastSin((float)angle * i))) * distance));
								}
							}
						}
						else //RaycastStrategy.GHOST_CIRCULAR
						{
							if (raycastModValue <= 0) raycastModValue = 0.2;

							MatrixD aimViewMatrix = MatrixD.CreateFromDir(aimForwardVector);
							double changeDistance = distance * MyMath.FastSin((float)MathHelperD.ToRadians(raycastModValue));
							double changeAngle = Math.PI * 2 / extraRaycastCount;
							for (int i = 1; i <= extraRaycastCount; i++)
							{
								Vector3D changeVector = new Vector3D(MyMath.FastSin((float)changeAngle * i), MyMath.FastCos((float)changeAngle * i), 0);
								testPositions.Add(aimStartPosition + (aimForwardVector * distance) + (Vector3D.TransformNormal(changeVector, aimViewMatrix) * changeDistance));
							}
						}

						for (int i = 0; i < testPositions.Count; i++)
						{
							targetPosition = testPositions[i];
							if (GetRaycastable(ref targetPosition, 1, out camera, out targetPosition))
							{
								entityInfo = Raycast(camera, ref targetPosition);
								if (!entityInfo.IsEmpty())
								{
									if (ValidTarget(ref entityInfo))
									{
										//Raycast successful
										SetInitialRaycastTargetInfo(ref entityInfo);

										return RaycastResult.RAYCAST_SUCCESS;
									}
								}
							}
							else
							{
								break;
							}
						}
						return RaycastResult.NO_TARGET_FOUND;
					}
					else
					{
						t_AimVector = aimForwardVector;
						t_AimClock = clock;

						t_AimReserves++;
					}
				}
				else
				{
					return RaycastResult.NO_TARGET_FOUND;
				}
			}
		}
		return RaycastResult.RAYCAST_COOLDOWN;
	}

	//Start tracking the target, using the targetPosition for first lock-on
	public RaycastResult StartTrackingByPosition(Vector3D targetPosition, long forceEntityId = -1, bool bypassRefresh = false)
	{
		if (CurrentRaycastState != RaycastState.NO_TARGET)
		{
			StopTracking();
		}

		if (clock >= NextRaycastReadyClock || bypassRefresh)
		{
			double extraRangeFactor = 1;
			if (gridReference != null)
			{
				extraRangeFactor += (DEF_POS_RAYCAST_EXTRA_DIST / (targetPosition - gridReference.WorldAABB.Center).Length());
			}

			RefreshRaycastReadyClock(ref targetPosition, extraRangeFactor);

			IMyCameraBlock camera;
			if (GetRaycastable(ref targetPosition, extraRangeFactor, out camera, out targetPosition))
			{
				MyDetectedEntityInfo entityInfo = Raycast(camera, ref targetPosition);
				if (!entityInfo.IsEmpty() && ValidTarget(ref entityInfo) && (forceEntityId == -1 || forceEntityId == entityInfo.EntityId))
				{
					//Raycast successful
					SetInitialRaycastTargetInfo(ref entityInfo);

					return RaycastResult.RAYCAST_SUCCESS;
				}
				else
				{
					return RaycastResult.NO_TARGET_FOUND;
				}
			}
		}
		return RaycastResult.RAYCAST_COOLDOWN;
	}

	void SetInitialRaycastTargetInfo(ref MyDetectedEntityInfo entityInfo)
	{
		LatestRaycastEntityInfo = entityInfo;
		LatestRaycastTriggeredClock = clock;
		FirstRaycastLockedClock = clock;

		OffsetTargetVector = offsetRaycastVector = GetOffsetVector(ref entityInfo);
		useOffsetRaycasting = false;

		targetRadius = entityInfo.BoundingBox.HalfExtents.Length();
		raycastRangeFactor = 1;

		//Check if target center is raycastable
		Vector3D targetPosition = entityInfo.Position;
		IMyCameraBlock camera;
		if (reservedRaycastCharges > 0 && GetRaycastable(ref targetPosition, 1, out camera, out targetPosition))
		{
			MyDetectedEntityInfo syncEntityInfo = Raycast(camera, ref targetPosition);
			if (syncEntityInfo.IsEmpty() || syncEntityInfo.EntityId != entityInfo.EntityId)
			{
				//Hollow center, switch to offset raycasting
				useOffsetRaycasting = true;
			}
			reservedRaycastCharges--;
		}

		CurrentRaycastState = RaycastState.TARGET_LOCKED;
	}

	//Stop tracking the target
	public void StopTracking()
	{
		LatestRaycastEntityInfo = new MyDetectedEntityInfo();
		LatestRaycastTriggeredClock = -1;

		CleanupStateValues();

		CurrentRaycastState = RaycastState.NO_TARGET;
	}

	void CleanupStateValues()
	{
		NextRaycastReadyClock = nextRaycastReadyRefresh = t_AimReserves = t_offsetHitCount = 0;
	}

	//Use this method and pass in the Runtime object from your main script to allow this class to take care of its internal clock and timing
	public void Update(IMyGridProgramRuntimeInfo runtime)
	{
		long ticks = runtime.TimeSinceLastRun.Ticks;
		if (ticks > 0)
		{
			clock += (int)Math.Round(ticks * 0.000006f);
			Update();
		}
	}

	//In case any Update() is missed, use this method to recorrect the internal clock
	public void Update(int newClock)
	{
		if (newClock > clock)
		{
			clock = newClock - 1;
			Update();
		}
	}

	//Perform the required processing. This method MUST be called ONCE and only ONCE everytime your Main() script runs
	public void Update()
	{
		clock++;

		if (CurrentRaycastState != RaycastState.NO_TARGET)
		{
			if (clock >= NextRaycastReadyClock)
			{
				Vector3D travelVector = LatestRaycastEntityInfo.Velocity * TICK_DELTA * (clock - LatestRaycastTriggeredClock);
				Vector3D targetPosition = LatestRaycastEntityInfo.Position + travelVector;
				if (useOffsetRaycasting)
				{
					targetPosition += Vector3D.TransformNormal(offsetRaycastVector, LatestRaycastEntityInfo.Orientation);
				}

				if (gridReference != null)
				{
					raycastRangeFactor = 1 + (targetRadius / (targetPosition - gridReference.WorldAABB.Center).Length());
				}

				RefreshRaycastReadyClock(ref targetPosition, raycastRangeFactor);

				IMyCameraBlock camera;
				if (GetRaycastable(ref targetPosition, raycastRangeFactor, out camera, out targetPosition))
				{
					MyDetectedEntityInfo entityInfo = Raycast(camera, ref targetPosition);
					if (!entityInfo.IsEmpty() && entityInfo.EntityId == LatestRaycastEntityInfo.EntityId)
					{
						//Raycast successfully maintained on target
						SetRaycastTargetInfo(ref entityInfo, useOffsetRaycasting);
					}
					else if (RelockUsagePercent > 0) //Target not found. Performing aggressive relock-on
					{
						PerformRelockOperations(camera, ref entityInfo, ref targetPosition, ref travelVector);
					}
				}
				else
				{
					CurrentRaycastState = RaycastState.TARGET_SLIPPED;
				}
			}
		}
	}

	void PerformRelockOperations(IMyCameraBlock camera, ref MyDetectedEntityInfo entityInfo, ref Vector3D targetPosition, ref Vector3D travelVector)
	{
		if (!entityInfo.IsEmpty())
		{
			if ((LatestRaycastEntityInfo.Position + travelVector - entityInfo.Position).LengthSquared() >= (targetRadius * targetRadius))
			{
				//Outside bounding sphere of expected target
				entityInfo = new MyDetectedEntityInfo();
			}
		}

		RaycastResult relockStatus = RaycastResult.NO_TARGET_FOUND;
		Vector3D origin = LatestRaycastEntityInfo.Position + travelVector;

		if (useOffsetRaycasting) //Using offset previously
		{
			relockStatus = AttemptRelock(origin, raycastRangeFactor, ref entityInfo); //Try without offset
		}
		else //Not using offset previously
		{
			relockStatus = AttemptRelock(origin + Vector3D.TransformNormal(offsetRaycastVector, LatestRaycastEntityInfo.Orientation), raycastRangeFactor, ref entityInfo); //Try with offset
		}

		if (relockStatus == RaycastResult.RAYCAST_SUCCESS)
		{
			//Raycast successfully maintained on target
			if (!useOffsetRaycasting)
			{
				t_offsetHitCount++;
				if (t_offsetHitCount >= USE_OFFSET_THRESHOLD_HIT_COUNT)
				{
					useOffsetRaycasting = true;
				}
			}

			SetRaycastTargetInfo(ref entityInfo, useOffsetRaycasting);
		}
		else
		{
			relockStatus = AttemptRelock(LatestRaycastEntityInfo.Position, raycastRangeFactor, ref entityInfo); //Try previous position
			if (relockStatus == RaycastResult.RAYCAST_SUCCESS)
			{
				//Raycast successfully maintained on target
				SetRaycastTargetInfo(ref entityInfo, useOffsetRaycasting);
			}
		}

		if (relockStatus == RaycastResult.NO_TARGET_FOUND)
		{
			double travelDistance = travelVector.Length();
			if (travelDistance < targetRadius)
			{
				if (Vector3D.IsZero(travelVector))
				{
					travelVector = LatestRaycastEntityInfo.Orientation.Forward * targetRadius;
				}
				else
				{
					travelVector = travelVector / travelDistance * targetRadius;
				}
				travelDistance = targetRadius;
			}
			Vector3D verticalVector = Vector3D.Normalize(travelVector.Cross(targetPosition - camera.GetPosition())) * travelDistance * 0.5;
			int frames = Math.Max((int)Math.Ceiling(travelDistance / targetRadius * 1.3), 1);

			travelVector /= frames;
			verticalVector /= frames;

			if (frames > RelockMaxFrameDepth)
			{
				frames = RelockMaxFrameDepth;
			}

			for (int i = 1; i <= frames; i++)
			{
				int max = relockPattern.GetLength(0);
				for (int j = 0; j < max; j++)
				{
					relockStatus = AttemptRelock(origin + (travelVector * i * relockPattern[j,0]) + (verticalVector * i * relockPattern[j,1]), raycastRangeFactor, ref entityInfo);
					if (relockStatus != RaycastResult.NO_TARGET_FOUND) break;
				}
				if (relockStatus != RaycastResult.NO_TARGET_FOUND) break;
			}

			if (relockStatus == RaycastResult.RAYCAST_SUCCESS)
			{
				//Raycast successfully maintained on target
				SetRaycastTargetInfo(ref entityInfo, useOffsetRaycasting);
			}
		}

		if (relockStatus != RaycastResult.RAYCAST_SUCCESS)
		{
			if (RelockCloseByEntityType == RelockCloseByType.AFTER_RELOCK && !entityInfo.IsEmpty())
			{
				//Use close by target
				SetRaycastTargetInfo(ref entityInfo, true);
			}
			else if (CurrentRaycastState == RaycastState.TARGET_SLIPPED && clock > LatestRaycastTriggeredClock + SLIP_LOST_THRESHOLD_TICKS)
			{
				CleanupStateValues();

				CurrentRaycastState = RaycastState.NO_TARGET;
			}
			else
			{
				CurrentRaycastState = RaycastState.TARGET_SLIPPED;
			}
		}
	}

	RaycastResult AttemptRelock(Vector3D targetPosition, double raycastRangeFactor, ref MyDetectedEntityInfo closestEntityInfo)
	{
		if (reservedRaycastCharges > 0)
		{
			IMyCameraBlock camera;
			if (GetRaycastable(ref targetPosition, raycastRangeFactor, out camera, out targetPosition))
			{
				MyDetectedEntityInfo entityInfo = Raycast(camera, ref targetPosition);
				reservedRaycastCharges--;

				if (!entityInfo.IsEmpty() && entityInfo.EntityId == LatestRaycastEntityInfo.EntityId)
				{
					closestEntityInfo = entityInfo;
					return RaycastResult.RAYCAST_SUCCESS;
				}
				else
				{
					if (RelockCloseByEntityType != RelockCloseByType.DISABLED)
					{
						if (!entityInfo.IsEmpty() && ValidTarget(ref entityInfo))
						{
							double distSq = (entityInfo.Position - targetPosition).LengthSquared();
							double radiusSq = (closestEntityInfo.IsEmpty() ? (targetRadius * targetRadius) : (closestEntityInfo.Position - targetPosition).LengthSquared());
							if (distSq < radiusSq) //Within bounding sphere of expected target and closer than previous closest match
							{
								closestEntityInfo = entityInfo;
								if (RelockCloseByEntityType == RelockCloseByType.IMMEDIATE)
								{
									//Use close by target
									return RaycastResult.RAYCAST_SUCCESS;
								}
							}
						}
					}
					return RaycastResult.NO_TARGET_FOUND;
				}
			}
			else
			{
				reservedRaycastCharges = 0;
			}
		}
		return RaycastResult.RAYCAST_COOLDOWN;
	}

	void SetRaycastTargetInfo(ref MyDetectedEntityInfo entityInfo, bool refreshRaycastOffset)
	{
		LatestRaycastEntityInfo = entityInfo;
		LatestRaycastTriggeredClock = clock;

		if (refreshRaycastOffset)
		{
			offsetRaycastVector = GetOffsetVector(ref entityInfo);
		}

		CurrentRaycastState = RaycastState.TARGET_LOCKED;

		if (RandomOffsetAmount > 0)
		{	
			ProbeForValidRandomOffsetVector();
		}

		if (probeOffsetVector != null)
		{
			ProbeForValidOffsetVector();
		}
	}

	void ProbeForValidRandomOffsetVector()
	{
		Vector3D partialOffsetVector = new Vector3D(((rnd.NextDouble() * 2) - 1) * RandomOffsetAmount, ((rnd.NextDouble() * 2) - 1) * RandomOffsetAmount, ((rnd.NextDouble() * 2) - 1) * RandomOffsetAmount);
		Vector3D targetPosition = LatestRaycastEntityInfo.Position + Vector3D.TransformNormal(partialOffsetVector, LatestRaycastEntityInfo.Orientation);

		IMyCameraBlock camera;
		if (GetRaycastable(ref targetPosition, raycastRangeFactor, out camera, out targetPosition))
		{
			MyDetectedEntityInfo entityInfo = Raycast(camera, ref targetPosition);
			if (!entityInfo.IsEmpty() && entityInfo.EntityId == LatestRaycastEntityInfo.EntityId)
			{
				OffsetTargetVector = GetOffsetVector(ref entityInfo);
				UseOffsetTargeting = true;

				ProbeOffsetVector = null;
				RandomOffsetAmount = 0f;
			}
		}
	}

	void ProbeForValidOffsetVector()
	{
		if (probeOffsetTriggeredCount < PROBE_CONVERGE_MAX_COUNT)
		{
			Vector3D partialOffsetVector = (double)(PROBE_CONVERGE_MAX_COUNT - probeOffsetTriggeredCount) / PROBE_CONVERGE_MAX_COUNT * probeOffsetVector.Value;
			Vector3D targetPosition = LatestRaycastEntityInfo.Position + Vector3D.TransformNormal(partialOffsetVector, LatestRaycastEntityInfo.Orientation);

			IMyCameraBlock camera;
			if (GetRaycastable(ref targetPosition, raycastRangeFactor, out camera, out targetPosition))
			{
				MyDetectedEntityInfo entityInfo = Raycast(camera, ref targetPosition);
				if (!entityInfo.IsEmpty() && entityInfo.EntityId == LatestRaycastEntityInfo.EntityId)
				{
					OffsetTargetVector = GetOffsetVector(ref entityInfo);
					UseOffsetTargeting = true;

					ProbeOffsetVector = null;
					RandomOffsetAmount = 0f;
				}
				else
				{
					probeOffsetTriggeredCount++;
				}
			}
		}
		else
		{
			probeOffsetVector = null;
		}
	}

	void RefreshRaycastReadyClock(ref Vector3D targetPosition, double raycastRangeFactor, int scaleDelay = 1)
	{
		if (clock >= nextRaycastReadyRefresh)
		{
			nextRaycastReadyRefresh = clock + RAYCAST_READY_REFRESH_TICKS;

			int count = 0;
			int charges = 0;
			double maxDist = 0;

			foreach (CameraGroup tryGroup in m_cameraGroups)
			{
				if (tryGroup.WithinLimits(ref targetPosition))
				{
					double distance = (targetPosition - tryGroup.Cameras[0].GetPosition()).Length() * raycastRangeFactor;
					double invD = 1.0 / distance;

					foreach (IMyCameraBlock camera in tryGroup.Cameras)
					{
						if (camera.IsWorking && camera.AvailableScanRange >= distance)
						{
							maxDist = Math.Max(maxDist, distance);
							count++;
							charges += (int)Math.Floor(camera.AvailableScanRange * invD);
						}
					}
				}
			}

			if (maxDist == 0 && gridReference != null)
			{
				maxDist = (targetPosition - gridReference.WorldAABB.Center).Length() * raycastRangeFactor;
			}

			if (count > 0)
			{
				float available = count * RaycastUsagePercent;
				float factor = ticksRatio / available;

				if (ProbeOffsetVector != null)
				{
					factor *= 2;
				}

				nextRaycastDelay = (int)Math.Ceiling(maxDist * factor);
				reservedRaycastCharges = (int)Math.Floor((charges - available) * RelockUsagePercent);
			}
			else
			{
				nextRaycastDelay = RAYCAST_COOLDOWN_RECHECK_TICKS;
				reservedRaycastCharges = 0;
			}
		}

		NextRaycastReadyClock = clock + Math.Max(MINIMUM_RAYCAST_TICKS, nextRaycastDelay * scaleDelay);
	}

	Vector3D GetOffsetVector(ref MyDetectedEntityInfo entityInfo)
	{
		if (entityInfo.HitPosition.HasValue)
		{
			return Vector3D.TransformNormal(entityInfo.HitPosition.Value - entityInfo.Position, MatrixD.Transpose(entityInfo.Orientation));
		}
		else
		{
			return Vector3D.Zero;
		}
	}

	bool GetRaycastable(ref Vector3D targetPosition, double raycastRangeFactor, out IMyCameraBlock readyCamera, out Vector3D newTargetPosition)
	{
		CameraGroup currentGroup = null;

		if (currentCameraGroup != null)
		{
			currentGroup = currentCameraGroup;
			if (GetFromCameraGroup(currentGroup, ref targetPosition, raycastRangeFactor, out readyCamera, out newTargetPosition))
			{
				currentCameraGroup = currentGroup;
				return true;
			}
		}

		foreach (CameraGroup tryGroup in m_cameraGroups)
		{
			if (tryGroup != currentGroup && tryGroup.WithinLimits(ref targetPosition))
			{
				if (GetFromCameraGroup(tryGroup, ref targetPosition, raycastRangeFactor, out readyCamera, out newTargetPosition))
				{
					currentCameraGroup = tryGroup;
					return true;
				}
			}
		}

		newTargetPosition = targetPosition;
		readyCamera = null;
		return false;
	}

	bool GetFromCameraGroup(CameraGroup group, ref Vector3D targetPosition, double raycastRangeFactor, out IMyCameraBlock readyCamera, out Vector3D newTargetPosition)
	{
		bool checkGroupLimit = true;

		for (int i = 0; i < group.Cameras.Count; i++)
		{
			if (group.StaggerIndex >= group.Cameras.Count)
			{
				group.StaggerIndex = 0;
			}
			IMyCameraBlock camera = group.Cameras[group.StaggerIndex++];
			if (camera.IsWorking)
			{
				Vector3D position = ((targetPosition - camera.GetPosition()) * raycastRangeFactor) + camera.GetPosition();
				if (CanScan(camera, ref position))
				{
					newTargetPosition = position;
					readyCamera = camera;
					return true;
				}
				else if (checkGroupLimit)
				{
					checkGroupLimit = false;
					if (!group.WithinLimits(ref targetPosition))
					{
						break;
					}
				}
			}
		}

		newTargetPosition = targetPosition;
		readyCamera = null;
		return false;
	}

	bool CanScan(IMyCameraBlock camera, ref Vector3D position)
	{
		Vector3D forward = (noFwdScale ? Vector3D.Zero : camera.WorldMatrix.Forward);
		Vector3D scaleLeft = camera.WorldMatrix.Left;
		Vector3D scaleUp = camera.WorldMatrix.Up;

		Vector3D direction = position - camera.WorldMatrix.Translation;

		if (sideScale >= 0)
		{
			return (camera.AvailableScanRange * camera.AvailableScanRange >= direction.LengthSquared()) &&
				direction.Dot(forward + scaleLeft) >= 0 &&
				direction.Dot(forward - scaleLeft) >= 0 &&
				direction.Dot(forward + scaleUp) >= 0 &&
				direction.Dot(forward - scaleUp) >= 0;
		}
		else
		{
			return (camera.AvailableScanRange * camera.AvailableScanRange >= direction.LengthSquared()) &&
				(direction.Dot(forward + scaleLeft) >= 0 ||
				direction.Dot(forward - scaleLeft) >= 0 ||
				direction.Dot(forward + scaleUp) >= 0 ||
				direction.Dot(forward - scaleUp) >= 0);
		}
	}

	void GetRaycastParameters(IMyCameraBlock camera, ref Vector3D position, out double distance, out double pitch, out double yaw)
	{
		Vector3D targetVector = position - camera.WorldMatrix.Translation;
		targetVector = Vector3D.TransformNormal(targetVector, MatrixD.Transpose(camera.WorldMatrix));

		Vector3D yawBaseVector = Vector3D.Normalize(new Vector3D(targetVector.X, 0, targetVector.Z));

		distance = targetVector.Normalize();
		
		yaw = MathHelper.ToDegrees(Math.Acos(MathHelper.Clamp(yawBaseVector.Dot(Vector3D.Forward), -1, 1)) * Math.Sign(targetVector.X));
		pitch = MathHelper.ToDegrees(Math.Acos(MathHelper.Clamp(yawBaseVector.Dot(targetVector), -1, 1)) * Math.Sign(targetVector.Y));
	}

	MyDetectedEntityInfo Raycast(IMyCameraBlock camera, ref Vector3D position)
	{
		double raycastDistance, raycastPitch, raycastYaw;
		GetRaycastParameters(camera, ref position, out raycastDistance, out raycastPitch, out raycastYaw);
		return camera.Raycast(raycastDistance, (float)raycastPitch, (float)raycastYaw);
	}

	bool ValidTarget(ref MyDetectedEntityInfo entityInfo)
	{
		if ((entityInfo.Type == MyDetectedEntityType.LargeGrid || entityInfo.Type == MyDetectedEntityType.SmallGrid) &&
			(ExcludedEntityIDs == null || !ExcludedEntityIDs.Contains(entityInfo.EntityId)) &&
			(MinimumCrossSection <= 0f || entityInfo.BoundingBox.Extents.LengthSquared() >= MinimumCrossSection * MinimumCrossSection))
		{
			switch (entityInfo.Relationship)
			{
				case MyRelationsBetweenPlayerAndBlock.Owner:
				case MyRelationsBetweenPlayerAndBlock.FactionShare:
					return !ExcludeFriendly;
				case MyRelationsBetweenPlayerAndBlock.Neutral:
				case MyRelationsBetweenPlayerAndBlock.NoOwnership:
					return !ExcludeNeutral;
				default:
					return true;
			}
		}
		return false;
	}

	//Get raycast target information. You should only call this after the Update() call has been made, to ensure latest information
	public RaycastState GetRaycastTarget(out Vector3D outTargetPosition, out Vector3D outTargetVelocity, out long outEntityId)
	{
		outEntityId = LatestRaycastEntityInfo.EntityId;
		return GetRaycastTarget(out outTargetPosition, out outTargetVelocity);
	}
	public RaycastState GetRaycastTarget(out Vector3D outTargetPosition, out Vector3D outTargetVelocity)
	{
		if (CurrentRaycastState != RaycastState.NO_TARGET)
		{
			outTargetVelocity = LatestRaycastEntityInfo.Velocity;
			outTargetPosition = LatestRaycastEntityInfo.Position + (outTargetVelocity * TICK_DELTA * (clock - LatestRaycastTriggeredClock));
			if (UseOffsetTargeting && OffsetTargetVector != Vector3D.Zero)
			{
				outTargetPosition += Vector3D.TransformNormal(OffsetTargetVector, LatestRaycastEntityInfo.Orientation);
			}
		}
		else
		{
			outTargetPosition = outTargetVelocity = Vector3D.Zero;
		}
		return CurrentRaycastState;
	}

	public RaycastState GetRaycastHitPosition(out Vector3D outHitPosition)
	{
		if (CurrentRaycastState != RaycastState.NO_TARGET)
		{
			Vector3D hitPosition = (LatestRaycastEntityInfo.HitPosition.HasValue ? LatestRaycastEntityInfo.HitPosition.Value : LatestRaycastEntityInfo.Position);
			outHitPosition = hitPosition + (LatestRaycastEntityInfo.Velocity * TICK_DELTA * (clock - LatestRaycastTriggeredClock));
		}
		else
		{
			outHitPosition = Vector3D.Zero;
		}
		return CurrentRaycastState;
	}

	public double FastAC(double x)
	{
	   return (-0.69813170079773212 * x * x - 0.87266462599716477) * x + 1.5707963267948966;
	}

	public class CameraGroup
	{
		public List<IMyCameraBlock> Cameras;
		public int StaggerIndex;

		public double SideScale;
		public bool NoFwdScale;

		public IMyCubeGrid Grid;

		public Vector3I PointUp;
		public Vector3I PointDown;
		public Vector3I PointLeft;
		public Vector3I PointRight;

		public Func<IMyCubeGrid, Vector3D> DirectionUp;
		public Func<IMyCubeGrid, Vector3D> DirectionLeft;
		public Func<IMyCubeGrid, Vector3D> DirectionForward;

		public static Vector3D GetDirectionUp(IMyCubeGrid grid) { return grid.WorldMatrix.Up; }
		public static Vector3D GetDirectionDown(IMyCubeGrid grid) { return grid.WorldMatrix.Down; }
		public static Vector3D GetDirectionLeft(IMyCubeGrid grid) { return grid.WorldMatrix.Left; }
		public static Vector3D GetDirectionRight(IMyCubeGrid grid) { return grid.WorldMatrix.Right; }
		public static Vector3D GetDirectionForward(IMyCubeGrid grid) { return grid.WorldMatrix.Forward; }
		public static Vector3D GetDirectionBackward(IMyCubeGrid grid) { return grid.WorldMatrix.Backward; }

		public static Func<IMyCubeGrid, Vector3D> GetDirectionFunction(Base6Directions.Direction dir)
		{
			switch (dir)
			{
				case Base6Directions.Direction.Up: return GetDirectionUp;
				case Base6Directions.Direction.Down: return GetDirectionDown;
				case Base6Directions.Direction.Left: return GetDirectionLeft;
				case Base6Directions.Direction.Right: return GetDirectionRight;
				case Base6Directions.Direction.Forward: return GetDirectionForward;
				case Base6Directions.Direction.Backward: return GetDirectionBackward;
				default: return GetDirectionForward;
			}
		}

		public static Vector3I GetReferencePoint(Base6Directions.Direction dir, int minX, int maxX, int minY, int maxY, int minZ, int maxZ)
		{

			switch (dir)
			{
				case Base6Directions.Direction.Up: return new Vector3I((minX + maxX) / 2, maxY, (minZ + maxZ) / 2);
				case Base6Directions.Direction.Down: return new Vector3I((minX + maxX) / 2, minY, (minZ + maxZ) / 2);
				case Base6Directions.Direction.Left: return new Vector3I(minX, (minY + maxY) / 2, (minZ + maxZ) / 2);
				case Base6Directions.Direction.Right: return new Vector3I(maxX, (minY + maxY) / 2, (minZ + maxZ) / 2);
				case Base6Directions.Direction.Forward: return new Vector3I((minX + maxX) / 2, (minY + maxY) / 2, minZ);
				case Base6Directions.Direction.Backward: return new Vector3I((minX + maxX) / 2, (minY + maxY) / 2, maxZ);
				default: return new Vector3I((minX + maxX) / 2, (minY + maxY) / 2, minZ);
			}
		}

		Vector3D GetAim(ref Vector3D position, ref Vector3I refPoint)
		{
			return position - Grid.GridIntegerToWorld(refPoint);
		}

		public bool WithinLimits(ref Vector3D position)
		{
			Vector3D forward = (NoFwdScale ? Vector3D.Zero : DirectionForward(Grid));
			Vector3D scaleLeft = SideScale * DirectionLeft(Grid);
			Vector3D scaleUp = SideScale * DirectionUp(Grid);

			if (SideScale >= 0)
			{
				return (GetAim(ref position, ref PointRight).Dot(forward + scaleLeft) >= 0 &&
						GetAim(ref position, ref PointLeft).Dot(forward - scaleLeft) >= 0 &&
						GetAim(ref position, ref PointDown).Dot(forward + scaleUp) >= 0 &&
						GetAim(ref position, ref PointUp).Dot(forward - scaleUp) >= 0);
			}
			else
			{
				return (GetAim(ref position, ref PointRight).Dot(forward + scaleLeft) >= 0 ||
						GetAim(ref position, ref PointLeft).Dot(forward - scaleLeft) >= 0 ||
						GetAim(ref position, ref PointDown).Dot(forward + scaleUp) >= 0 ||
						GetAim(ref position, ref PointUp).Dot(forward - scaleUp) >= 0);
			}
		}
	}

	public enum RaycastStrategy
	{
		DEFAULT,		//Shoot only one single raycast
		GHOST_AHEAD,	//Shoot additional raycast ahead to compensate for lag
		GHOST_CIRCULAR  //Shoot additional raycast around aim point to compensate for jitter
	}

	public enum RelockCloseByType
	{
		DISABLED,	   //No relocking to close by target
		IMMEDIATE,	  //Relock to close by target upon detecting suitable
		AFTER_RELOCK	//Relock to close by only when all relock attempts failed
	}

	public enum RaycastState
	{
		NO_TARGET,
		TARGET_LOCKED,
		TARGET_SLIPPED
	}

	public enum RaycastResult
	{
		NO_TARGET_FOUND,
		RAYCAST_SUCCESS,
		RAYCAST_COOLDOWN
	}
}

public class FastSolver
{
    public static readonly double epsilon = 0.000001;
    
    public static readonly double cos120d = -0.5;
    public static readonly double sin120d = Math.Sin(Math.PI / 3.0);
    public static readonly double root3 = Math.Sqrt(3.0);

    public static readonly double inv3 = 1.0 / 3.0;
    public static readonly double inv9 = 1.0 / 9.0;
    public static readonly double inv54 = 1.0 / 54.0;

    //Shortcut Ignoring Of Complex Values And Return Smallest Real Number
    public static double Solve(double a, double b, double c, double d, double e)
    {
        if (Math.Abs(a) < epsilon) a = (a >= 0 ? epsilon : -epsilon);
        double inva = 1 / a;

        b *= inva;
        c *= inva;
        d *= inva;
        e *= inva;
        
        double a3 = -c;
        double b3 =  b * d - 4 * e;
        double c3 = -b * b * e - d * d + 4 * c * e;
        
        double[] result;
        bool chooseMaximal = SolveCubic(a3, b3, c3, out result);
        double y = result[0];
        if (chooseMaximal)
        {
            if (Math.Abs(result[1]) > Math.Abs(y)) y = result[1];
            if (Math.Abs(result[2]) > Math.Abs(y)) y = result[2];
        }

        double q1, q2, p1, p2, squ;
        
        double u = y * y - 4 * e;
        if (Math.Abs(u) < epsilon)
        {
            q1 = q2 = y * 0.5;
            u = b * b - 4 * (c - y);

            if (Math.Abs(u) < epsilon)
            {
                p1 = p2 = b * 0.5;
            }
            else
            {
                squ = Math.Sqrt(u);
                p1 = (b + squ) * 0.5;
                p2 = (b - squ) * 0.5;
            }
        }
        else
        {
            squ = Math.Sqrt(u);
            q1 = (y + squ) * 0.5;
            q2 = (y - squ) * 0.5;

            double dm = 1 / (q1 - q2);
            p1 = (b * q1 - d) * dm;
            p2 = (d - b * q2) * dm;
        }
        
        double v1, v2;

        u = p1 * p1 - 4 * q1;
        if (u < 0)
        {
            v1 = double.MaxValue;
        }
        else
        {
            squ = Math.Sqrt(u);
            v1 = MinPosNZ(-p1 + squ, -p1 - squ) * 0.5;
        }
        
        u = p2 * p2 - 4 * q2;
        if (u < 0)
        {
            v2 = double.MaxValue;
        }
        else
        {
            squ = Math.Sqrt(u);
            v2 = MinPosNZ(-p2 + squ, -p2 - squ) * 0.5;
        }

        return MinPosNZ(v1, v2);
    }

    private static bool SolveCubic(double a, double b, double c, out double[] result)
    {
        result = new double[4];

	    double a2 = a * a;
    	double q = (a2 - 3 * b) * inv9;
	    double r = (a * (2 * a2 - 9 * b) + 27 * c) * inv54;
        double r2 = r * r;
	    double q3 = q * q * q;
        
        if (r2 < q3)
        {
            double sqq = Math.Sqrt(q);
            
            double t = r / (sqq * sqq * sqq);
            if (t < -1) t = -1;
            else if (t > 1) t = 1;

            t = Math.Acos(t);

            a *= inv3;
            q = -2 * sqq;

            double costv3 = Math.Cos(t * inv3);
            double sintv3 = Math.Sin(t * inv3);
            
            result[0] = q * costv3 - a;
            result[1] = q * ((costv3 * cos120d) - (sintv3 * sin120d)) - a;
            result[2] = q * ((costv3 * cos120d) + (sintv3 * sin120d)) - a;

            return true;
        } 
        else 
        {
            double g = -Math.Pow(Math.Abs(r) + Math.Sqrt(r2 - q3), inv3);
            if (r < 0) g = -g;

            double h = (g == 0 ? 0 : q / g);
            
            a *= inv3;

            result[0] = (g + h) - a;
            result[1] = -0.5 * (g + h) - a;
            result[2] = 0.5 * root3 * (g - h);

            if (Math.Abs(result[2]) < epsilon)
            {
                result[2] = result[1];
                return true;
            }
		    else
            {
                return false;
            }
        }
    }

    private static double MinPosNZ(double a, double b)
    {
        if (a <= 0) return (b > 0 ? b : double.MaxValue);
        else if (b <= 0) return a;
        else return Math.Min(a, b);
    }
}

public class PDController
{
	double lastInput;

	public double gain_p;
	public double gain_d;

	double second;

	public PDController(double pGain, double dGain, float stepsPerSecond = 60f)
	{
		gain_p = pGain;
		gain_d = dGain;
		second = stepsPerSecond;
	}

	public double Filter(double input, int round_d_digits)
	{
		double roundedInput = Math.Round(input, round_d_digits);

		double derivative = (roundedInput - lastInput) * second;
		lastInput = roundedInput;

		return (gain_p * input) + (gain_d * derivative);
	}

	public void Reset()
	{
		lastInput = 0;
	}
}

public class GyroControl
{
	Action<IMyGyro, float>[] profiles =
	{
		(g, v) => { g.Yaw = -v; },
		(g, v) => { g.Yaw = v; },
		(g, v) => { g.Pitch = -v; },
		(g, v) => { g.Pitch = v; },
		(g, v) => { g.Roll = -v; },
		(g, v) => { g.Roll = v; }
	};

	public List<IMyGyro> Gyroscopes { get; protected set; }

	byte[] gyroYaw;
	byte[] gyroPitch;
	byte[] gyroRoll;

	int activeGyro = 0;

	public GyroControl(List<IMyGyro> newGyros)
	{
		Gyroscopes = newGyros;
	}

	public void Init(ref MatrixD refWorldMatrix)
	{
		if (Gyroscopes == null)
		{
			Gyroscopes = new List<IMyGyro>();
		}

		gyroYaw = new byte[Gyroscopes.Count];
		gyroPitch = new byte[Gyroscopes.Count];
		gyroRoll = new byte[Gyroscopes.Count];

		for (int i = 0; i < Gyroscopes.Count; i++)
		{
			gyroYaw[i] = SetRelativeDirection(Gyroscopes[i].WorldMatrix.GetClosestDirection(refWorldMatrix.Up));
			gyroPitch[i] = SetRelativeDirection(Gyroscopes[i].WorldMatrix.GetClosestDirection(refWorldMatrix.Left));
			gyroRoll[i] = SetRelativeDirection(Gyroscopes[i].WorldMatrix.GetClosestDirection(refWorldMatrix.Forward));
		}

		activeGyro = 0;
	}

	public byte SetRelativeDirection(Base6Directions.Direction dir)
	{
		switch (dir)
		{
			case Base6Directions.Direction.Up:
				return 1;
			case Base6Directions.Direction.Down:
				return 0;
			case Base6Directions.Direction.Left:
				return 2;
			case Base6Directions.Direction.Right:
				return 3;
			case Base6Directions.Direction.Forward:
				return 4;
			case Base6Directions.Direction.Backward:
				return 5;
		}
		return 0;
	}

	public void Enabled(bool enabled)
	{
		foreach (IMyGyro gyro in Gyroscopes)
		{
			if (gyro.Enabled != enabled) gyro.Enabled = enabled;
		}
	}

	public void SetGyroOverride(bool bOverride)
	{
		CheckGyro();

		for (int i = 0; i < Gyroscopes.Count; i++)
		{
			if (i == activeGyro)
			{
				if (Gyroscopes[i].GyroOverride != bOverride)
				{
					Gyroscopes[i].GyroOverride = bOverride;
				}
			}
			else
			{
				if (Gyroscopes[i].GyroOverride)
				{
					Gyroscopes[i].GyroOverride = false;
				}
			}
		}
	}

	public void SetGyroYaw(float yawRate)
	{
		if (activeGyro < Gyroscopes.Count)
		{
			profiles[gyroYaw[activeGyro]](Gyroscopes[activeGyro], yawRate);
		}
	}

	public void SetGyroPitch(float pitchRate)
	{
		if (activeGyro < Gyroscopes.Count)
		{
			profiles[gyroPitch[activeGyro]](Gyroscopes[activeGyro], pitchRate);
		}
	}

	public void SetGyroRoll(float rollRate)
	{
		if (activeGyro < Gyroscopes.Count)
		{
			profiles[gyroRoll[activeGyro]](Gyroscopes[activeGyro], rollRate);
		}
	}

	public void ZeroTurnGyro()
	{
		for (int i = 0; i < Gyroscopes.Count; i++)
		{
			profiles[gyroYaw[i]](Gyroscopes[i], 0f);
			profiles[gyroPitch[i]](Gyroscopes[i], 0f);
		}
	}

	public void ResetGyro()
	{
		foreach (IMyGyro gyro in Gyroscopes)
		{
			gyro.Yaw = gyro.Pitch = gyro.Roll = 0f;
		}
	}

	public void CheckGyro()
	{
		if (activeGyro < Gyroscopes.Count)
		{
			if (Gyroscopes[activeGyro].IsFunctional)
			{
				return;
			}
			else
			{
				activeGyro++;

				while (activeGyro < Gyroscopes.Count)
				{
					IMyGyro gyro = Gyroscopes[activeGyro];

					if (gyro.IsFunctional)
					{
						if (!gyro.GyroOverride) gyro.GyroOverride = true;

						break;
					}
					else
					{
						if (gyro.GyroOverride) gyro.GyroOverride = false;

						activeGyro++;
					}
				}
			}
		}
	}

	public bool HasUsableGyro()
	{
		foreach (IMyGyro gyro in Gyroscopes)
		{
			if (gyro.IsFunctional)
			{
				return true;
			}
		}
		return false;
	}
}

public class ThrusterControl
{
	public List<IMyThrust> Thrusters { get; protected set; }

	public ThrusterControl(List<IMyThrust> forward)
	{
		Thrusters = forward;
	}

	public void Enabled(bool enabled)
	{
		if (Thrusters != null)
		{
			foreach (IMyThrust th in Thrusters)
			{
				if (th.Enabled != enabled) th.Enabled = enabled;
			}
		}
	}

	public void FireThrusters(bool switchOn)
	{
		if (Thrusters != null)
		{
			foreach (IMyThrust th in Thrusters)
			{
				th.ThrustOverridePercentage = (switchOn ? 1f : 0f);
			}
		}
	}

	public bool HasUsableThruster()
	{
		foreach (IMyThrust th in Thrusters)
		{
			if (th.IsFunctional)
			{
				return true;
			}
		}
		return false;
	}
}

public class ThrusterGroup
{
	public List<IMyThrust> Thrusters;
	public float TotalThrust;

	public int ThrustGroupIndex;
	public double ThrustDotGridDiagonal;

	public ThrusterGroup(int thrustGroupIndex, double thrustDotGridDiagonal)
	{
		ThrustGroupIndex = thrustGroupIndex;
		ThrustDotGridDiagonal = thrustDotGridDiagonal;
	}
}

public class ThrusterGroupComparer : IComparer<ThrusterGroup>
{
	public int Compare(ThrusterGroup x, ThrusterGroup y)
	{
		if (x.TotalThrust > y.TotalThrust) return -1;
		else if (x.TotalThrust < y.TotalThrust) return 1;
		else
		{
			if (x.ThrustDotGridDiagonal > y.ThrustDotGridDiagonal) return -1;
			else if (x.ThrustDotGridDiagonal < y.ThrustDotGridDiagonal) return 1;
			else
			{
				if (x.ThrustGroupIndex > y.ThrustGroupIndex) return -1;
				else if (x.ThrustGroupIndex < y.ThrustGroupIndex) return 1;
				else return 0;
			}
		}
	}
}

#endregion

#endregion
//-------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
	}
}
