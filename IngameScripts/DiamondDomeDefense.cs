using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using VRageMath;
using VRage;
using VRage.Game;
using VRage.Game.ModAPI.Ingame;
using VRage.Game.ModAPI.Ingame.Utilities;
using SpaceEngineers.Game.ModAPI.Ingame;
using VRage.Game.GUI.TextPanel;

namespace DiamondDomeDefense
{
	public class Program : MyGridProgram
	{
//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
#region Script
//------------------------------------------------------------
// ADN - Diamond Dome Defense Script v5.1
//------------------------------------------------------------

//------------------ Default Settings ------------------
#region General Settings
public class GeneralSettings
{
	public int MainBlocksReloadTicks = 600;

	public int TargetTracksTransmitIntervalTicks = 45;

	public int ManualAimBroadcastDurationTicks = 3600;
	public double ManualAimRaycastDistance = 5000;
	public int ManualAimRaycastRefreshInterval = 30;

	public int MaxDesignatorUpdatesPerTick = 1;

	public float MaxPDCUpdatesPerTick = 1;
	public int MinPDCRefreshRate = 15;

	public bool UseDesignatorReset = true;
	public int DesignatorResetInterval = 45;

	public bool UseRangeSweeper = true;
	public int RangeSweeperInterval = 2;
	public int RangeSweeperPerTick = 3;

	public int TargetFoundHoldTicks = 60;

	public bool UsePDCSpray = true;
	public double PDCSprayMinTargetSize = 12;

	public int RandomOffsetProbeInterval = 15;

	public double MaxRaycastTrackingDistance = 3000;

	public int RaycastTargetRefreshTicks = 15;
	public int RaycastGlobalRefreshTicks = 3;

	public int PriorityMinRefreshTicks = 15;
	public int PriorityMaxRefreshTicks = 90;
	public int PriorityGlobalRefreshTicks = 2;

	public int TargetSlippedTicks = 15;
	public int TargetLostTicks = 60;

	public double RaycastExtensionDistance = 5;

	public double MinTargetSizeEngage = 2;
	public double MinTargetSizePriority = 4;

	public double MissileMinTargetSize = 12;
	public double MissileCountPerSize = 36;

	public double MaxMissileLaunchDistance = 2000;
	public double MissileOffsetRadiusFactor = 0.5;
	public double MissileOffsetProbability = 0.75;

	public double PriorityDowngradeConstant = 800;

	public int MissileStaggerWaitTicks = 60;
	public int MissileReassignIntervalTicks = 600;
	public int MissilePBGridReloadTicks = 300;
	public int MissileTransmitDurationTicks = 600;
	public int MissileTransmitIntervalTicks = 15;

	public double MissileLaunchSpeedLimit = 60;

	public double PDCFireDotLimit = 0.995;
	public bool ConstantFireMode = true;

	public bool RotorUseLimitSnap = false;

	public float RotorCtrlDeltaGain = 1f;
	public float RotorCtrlOutputGain = 60f;
	public float RotorCtrlOutputLimit = 60f;
	
	public float RotorSnapVelocityGain = 50f;
	public float RotorSnapSpeedLimit = 60f;

	public int ReloadCheckTicks = 90;
	public int ReloadedCooldownTicks = 300;

	public double ReloadMaxAngle = 88;
	public double ReloadLockStateAngle = 79;

	public int DisplaysRefreshInterval = 30;

	public int AllyTrackLostTicks = 120;

	public bool CheckSelfOcclusion = true;
	public bool UseAABBOcclusionChecker = false;
	public float OcclusionExtraClearance = 0f;
	public int OcclusionCheckerInitBlockLimit = 500;
}
#endregion
//------------------ Weapon Profiles ------------------
#region Weapon Profiles
const double GATLING_INITIAL_SPEED = 400;
const double GATLING_ACCELERATION = 0;
const double GATLING_MAX_SPEED = 400;
const double GATLING_MAX_RANGE = 800;
const double GATLING_SPAWN_OFFSET = 0;
const double GATLING_RELOAD_TIME = 0;
const bool GATLING_IS_CAPPED_SPEED = false;
const bool GATLING_USE_SALVO = false;

const double ROCKET_INITIAL_SPEED = 100;
const double ROCKET_ACCELERATION = 600;
const double ROCKET_MAX_SPEED = 200;
const double ROCKET_MAX_RANGE = 800;
const double ROCKET_SPAWN_OFFSET = 4;
const double ROCKET_RELOAD_TIME = 1;
const bool ROCKET_IS_CAPPED_SPEED = true;
const bool ROCKET_USE_SALVO = true;

public class DefaultWeaponProfiles
{
	public WeaponProfile gatlingProfile = new WeaponProfile(GATLING_INITIAL_SPEED, GATLING_ACCELERATION, GATLING_MAX_SPEED, GATLING_MAX_RANGE, GATLING_SPAWN_OFFSET, GATLING_RELOAD_TIME, GATLING_IS_CAPPED_SPEED, GATLING_USE_SALVO);
	public WeaponProfile rocketProfile = new WeaponProfile(ROCKET_INITIAL_SPEED, ROCKET_ACCELERATION, ROCKET_MAX_SPEED, ROCKET_MAX_RANGE, ROCKET_SPAWN_OFFSET, ROCKET_RELOAD_TIME, ROCKET_IS_CAPPED_SPEED, ROCKET_USE_SALVO);
}
#endregion
//------------------ Script Constants ------------------
#region Script Constants
const string DISPLAY_VERSION = "5.0";
const string DISPLAY_SCRIPT_NAME = "Diamond Dome";

const string INI_SECTION = "DDS";
const string MISSILE_INI_SECTION = "AGM";

const string DESIGNATOR_GRP_TAG = "DDS Designator";
const string RAYCAST_CAMERA_GRP_TAG = "DDS Camera";
const string MANUAL_AIMING_GRP_TAG = "DDS Aiming";
const string SILO_DOOR_GRP_TAG = "DDS Door";
const string DISPLAY_GRP_TAG = "DDS Display";

const string PDC_GRP_TAG = "DDS Turret";
const string AZIMUTH_ROTOR_TAG = "Azimuth";
const string ELEVATION_ROTOR_TAG = "Elevation";
const string AIM_BLOCK_TAG = "Aiming";
const string RELOAD_CONNECTOR_TAG = "Reload";

const string MANUAL_AIMING_AIMBLOCK_TAG = "Forward";
const string MANUAL_AIMING_STATUS_TAG = "Status";
const string MANUAL_AIMING_ALERT_TAG = "Alert";
const string MANUAL_AIMING_GRP_PREFIX = "AIM";

const string MISSILE_PB_GRP_TAG = "DDS Missile";
const string TORPEDO_PB_GRP_TAG = "DDS Torpedo";

const string USE_CGE_TAG = "[CGE]";

const string MANUAL_LAUNCH_CMD = "FIRE";
const string MANUAL_LAUNCH_LARGEST_TAG = "LARGEST";
const string MANUAL_LAUNCH_TORPEDO_TAG = "BIG";
const string MANUAL_LAUNCH_EXTENDED_TAG = "EXTEND";
const string MANUAL_AIM_TRACK_TAG = "TRACK";
const string MANUAL_AIM_RELEASE_TAG = "RELEASE";
const string MANUAL_AIM_VALUE_SET_TAG = "SET";
const string MANUAL_AIM_VALUE_SET_CENTER_TAG = "CENTER";
const string MANUAL_AIM_VALUE_SET_OFFSET_TAG = "OFFSET";
const string MANUAL_AIM_VALUE_SET_RANDOM_TAG = "RANDOM";
const string MANUAL_AIM_VALUE_SET_RANGE_TAG = "RANGE";
const string MANUAL_AIM_VALUE_INC_RANGE_TAG = "INCRANGE";
const string MANUAL_AIM_VALUE_DEC_RANGE_TAG = "DECRANGE";
const string MANUAL_AIM_VALUE_CYCLE_OFFSET_TAG = "CYCLEOFFSET";

const string CMD_TOGGLE_ON_OFF = "TOGGLE";
const string CMD_TOGGLE_ON = "ENABLE";
const string CMD_TOGGLE_OFF = "DISABLE";
const string CMD_DEBUG_MODE = "DEBUGMODE";

const double COS_45_DEGREES = 0.707;

const int OFFSET_POINTS_MAX_COUNT = 7;
const int OFFSET_POINTS_MOVE_ANGLE_PER_SECOND = 30;
const int OFFSET_POINTS_EXPIRE_TICKS = 240;
const int TARGET_ORIENTATION_EXPIRE_TICKS = 120;
const double OFFSET_PROBE_RANDOM_FACTOR = 0.75;

const int STATUS_REFRESH_INTERVAL = 60;

const double PROFILER_NEW_VALUE_FACTOR = 0.005;
const int PROFILER_HISTORY_COUNT = (int)(1 / PROFILER_NEW_VALUE_FACTOR);

const double INV_ONE_TICK = 1.0 / 60.0;
const int TICKS_PER_SECOND = 60;

const UpdateType ARG_COMMAND_FLAGS = UpdateType.Terminal | UpdateType.Trigger | UpdateType.Script;

const string IGC_MSG_TRACKS_INFO = "IGCMSG_TK_IF";
const string IGC_MSG_TARGET_TRACKS = "IGCMSG_TR_TK";
const string IGC_MSG_TARGET_DATALINK = "IGCMSG_TR_DL";
const string IGC_MSG_TARGET_SWITCH_LOST = "IGCMSG_TR_SW";

const string CGE_MSG_TARGET_DATALINK = "CGE_MSG_TR_DL";

const string FAKE = "SIMJ";

const float standardTextSizeFactor = 0.032f;

readonly Color statusThisScriptTextColor = Color.White;
readonly Color statusThisScriptBoxColor = new Color(40, 5, 100);
readonly Color statusAimIdTextColor = new Color(245, 230, 255);
readonly Color statusAimIdBoxColor = new Color(40, 15, 5);

readonly Color statusTitleTextColor = Color.White;
readonly Color statusNoTargetTextColor = new Color(0, 0, 0);
readonly Color statusNoTargetBoxColor = new Color(50, 50, 50);
readonly Color statusSeekingTextColor = new Color(255, 255, 255);
readonly Color statusSeekingBoxColor = new Color(100, 100, 0);
readonly Color statusLockedTextColor = Color.White;
readonly Color statusLockedBoxColor = Color.Green;
readonly Color statusCurrentTargetTextColor = Color.White;
readonly Color statusCurrentTargetBoxColor = Color.DarkOrchid;
readonly Color statusOptionsTextColor = Color.White;
readonly Color statusOptionsBoxColor = new Color(0, 0, 90);

readonly char[] splitDelimiterCommand = new char[]{':'};

readonly char[] progressIcons = new char[]{ '\u2014', '\\', '|', '/', '\u2014', '\\', '|', '/' };

readonly IComparer<PDCTarget> sortCommsTargetPriority = new PDCTargetCommsSorting();

readonly Random rnd = new Random();
readonly List<IMyTerminalBlock> dummyBlocks = new List<IMyTerminalBlock>(0);
readonly StringBuilder sb = new StringBuilder();
#endregion
//------------------ Global Variables ------------------
#region Global Variables
int loadedCustomDataHashCode = 0;

GeneralSettings settings;
DefaultWeaponProfiles weaponProfiles;

IMyRemoteControl remote;

List<PDCTurret> pdcList;
RoundRobin<PDCTurret> pdcAssignRR;
RoundRobin<PDCTurret> pdcFireRR;

List<IMyTextSurface> displayPanels;

int curPDCUpdatesPerTick = 1;
int curPDCUpdatesSkipTicks = 0;
int curPDCNextUpdateClock = 0;

List<Designator> designators;
RoundRobin<Designator> designatorTargetRR;
RoundRobin<Designator> designatorOperationRR;
int designatorTargetAcquiredClock = -10000;

List<IMyCameraBlock> raycastCameras;
RaycastHandler raycastHandler;
int nextRaycastGlobalClock;

TargetManager targetManager;

AllyManager allyManager;

SortedDictionary<double, PDCTarget> sortedEntityIds;
double maxPriorityValue = 0;
int nextPriorityRefreshClock;
bool curHaveTargets = false;
int assignmentState = 0;

List<IMyProgrammableBlock> missileComputers;
List<IMyProgrammableBlock> torpedoComputers;

List<ManualPDCTarget> manualTargeters;
Dictionary<string, ManualPDCTarget> manualTargetersLookup;

byte[] genUniqueIdBuffer = new byte[8];
long[] genUniqueIdResult = new long[1];

int nextAutoMissileLaunchClock = 0;

IPDCOcclusionChecker occlusionChecker;
IEnumerator<int> iterOcclusionCreator;

IMyBroadcastListener igcTargetTracksListener;
IMyBroadcastListener igcTracksInfoListener;

Queue<MissileCommsTarget> guidanceCommsTargets;
Queue<PDCTarget> tracksCommsTargets;
int curCommsQueuePriority = 0;
bool haveIGCTransmitted = false;

double gridSpeedLimit = 100;
double shipRadius = 0;

int nextTracksTranmissionClock = 0;
int nextMyShipTranmissionClock = 0;

long timeSinceLastRun = 0;
bool switchedOn = true;

Profiler profiler;
bool debugMode;

int progressIconIndex = 0;

int clock = 0;
int subMode = 0;
bool init = false;
#endregion
//-------------------- Main Program --------------------

Program()
{
	Runtime.UpdateFrequency = UpdateFrequency.Update1;

	profiler = new Profiler(Runtime, PROFILER_HISTORY_COUNT, PROFILER_NEW_VALUE_FACTOR);

	settings = new GeneralSettings();
	weaponProfiles = new DefaultWeaponProfiles();
}

public void Main(string args, UpdateType updateType)
{
	if (!init)
	{
		if (!InitLoop())
		{
			return;
		}

		nextPriorityRefreshClock = -100000;
		assignmentState = 0;

		switchedOn = true;
		debugMode = false;

		profiler.Clear();

		clock = -1;

		init = true;
		return;
	}

	profiler.UpdateRuntime();

	if (debugMode) profiler.StartSectionWatch("InterGridComms");

	if (args.Length > 0)
	{
		if (args.Equals(IGC_MSG_TARGET_TRACKS))
		{
			ProcessCommsMessage();
		}
		else
		{
			if ((updateType & ARG_COMMAND_FLAGS) != 0)
			{
				ProcessCommands(args);
			}
		}
	}

	if (debugMode) profiler.StopSectionWatch("InterGridComms");

	timeSinceLastRun += Runtime.TimeSinceLastRun.Ticks;

	if ((updateType & UpdateType.Update1) == 0 || timeSinceLastRun == 0)
	{
		return;
	}
	timeSinceLastRun = 0;

	clock++;

	if (!switchedOn)
	{
		if (clock % STATUS_REFRESH_INTERVAL == 0)
		{
			DisplayStatus();
		}
		return;
	}

	if (debugMode) profiler.StartSectionWatch("AutoMissileLaunch");

	if (clock >= nextAutoMissileLaunchClock)
	{
		LaunchAutomaticMissiles();
	}

	if (debugMode) profiler.StopSectionWatch("AutoMissileLaunch");
	if (debugMode) profiler.StartSectionWatch("MissileReload");

	if (settings.MissilePBGridReloadTicks > 0 && clock % settings.MissilePBGridReloadTicks == 0)
	{
		ReloadMissilesAndTorpedos();
	}

	if (debugMode) profiler.StopSectionWatch("MissileReload");
	if (debugMode) profiler.StartSectionWatch("ManualAimReload");

	if (settings.MainBlocksReloadTicks > 0 && clock % settings.MainBlocksReloadTicks == 0)
	{
		ReloadMainBlocks();
	}

	if (debugMode) profiler.StopSectionWatch("ManualAimReload");
	if (debugMode) profiler.StartSectionWatch("ManualRaycast");

	UpdateManualAimRaycast();

	if (debugMode) profiler.StopSectionWatch("ManualRaycast");
	if (debugMode) profiler.StartSectionWatch("Designator");

	UpdateDesignatorTargets();

	if (debugMode) profiler.StopSectionWatch("Designator");
	if (debugMode) profiler.StartSectionWatch("Allies");

	UpdateAllies();

	if (debugMode) profiler.StopSectionWatch("Allies");

	if (targetManager.Count() > 0)
	{
		curHaveTargets = true;

		if (debugMode) profiler.StartSectionWatch("RaycastTracking");

		if (clock >= nextRaycastGlobalClock)
		{
			UpdateRaycastTargets();
		}
		
		if (debugMode) profiler.StopSectionWatch("RaycastTracking");
		if (debugMode) profiler.StartSectionWatch("AssignTargets");

		if (clock % settings.PriorityGlobalRefreshTicks == 0)
		{
			AssignTargets();
		}

		if (debugMode) profiler.StopSectionWatch("AssignTargets");
		if (debugMode) profiler.StartSectionWatch("PDCAimFireReload");

		AimFireReloadPDC();

		if (debugMode) profiler.StopSectionWatch("PDCAimFireReload");
	}
	else if (curHaveTargets)
	{
		curHaveTargets = false;

		foreach (PDCTurret pdcReset in pdcList)
		{
			pdcReset.TargetInfo = null;
			pdcReset.ReleaseWeapons();
			pdcReset.ResetRotors();
		}

		assignmentState = 0;
	}

	if (debugMode) profiler.StartSectionWatch("TransmitIGCMessages");

	if (targetManager.Count() > 0 || guidanceCommsTargets.Count() > 0)
	{
		TransmitIGCMessages();
	}

	TransmitMyShipInformation();

	if (debugMode) profiler.StopSectionWatch("TransmitIGCMessages");

	if (clock % settings.DisplaysRefreshInterval == 0)
	{
		RefreshDisplays();
	}

	if (clock % STATUS_REFRESH_INTERVAL == 0)
	{
		DisplayStatus();
	}

	profiler.UpdateComplexity();
}

void ForceJITCompilation()
{
	try
	{
		PDCTarget fakeTarget;
		IMyTerminalBlock fakeTargetBlock = (raycastHandler.Cameras.Count > 0 ? raycastHandler.Cameras[0] : (IMyTerminalBlock)Me);
		MyDetectedEntityInfo fakeEntityInfo = new MyDetectedEntityInfo(9801, FAKE, MyDetectedEntityType.LargeGrid, fakeTargetBlock.WorldMatrix.Translation + (fakeTargetBlock.WorldMatrix.Forward * 0.1), fakeTargetBlock.WorldMatrix, Vector3D.Zero, MyRelationsBetweenPlayerAndBlock.Enemies, fakeTargetBlock.WorldAABB.Inflate(100), 1);
		targetManager.UpdateTarget(ref fakeEntityInfo, 1, out fakeTarget);
		
		ProcessCommsMessage();
		ProcessCommands(FAKE);

		LaunchAutomaticMissiles();

		LaunchMissileForTarget(fakeTarget, false, true);
		LaunchManualForTarget(torpedoComputers, fakeTarget, TargetingPointTypeEnum.Center, null, false, true);

		double savedValue = settings.ManualAimRaycastDistance;
		settings.ManualAimRaycastDistance = 0.1f;
		manualTargeters = new List<ManualPDCTarget>();
		manualTargeters.Add(new ManualPDCTarget("AIM0", Me, null, null));
		UpdateManualAimRaycast();
		settings.ManualAimRaycastDistance = savedValue;
		manualTargeters = null;

		UpdateRaycastTargets();
		AssignTargets();
		AimFireReloadPDC();

		UpdateRaycastTargets();
		AssignTargets();
		AimFireReloadPDC();

		targetManager.ClearBlackList();
		targetManager.ClearTargets();
		
		curHaveTargets = false;

		foreach (PDCTurret pdcReset in pdcList)
		{
			pdcReset.TargetInfo = null;
			pdcReset.ReleaseWeapons();
			pdcReset.ResetRotors();
		}

		assignmentState = 0;

		guidanceCommsTargets.Clear();
		tracksCommsTargets.Clear();

		TransmitIGCMessages();
		TransmitIGCMessages();

		guidanceCommsTargets.Clear();
		tracksCommsTargets.Clear();

		haveIGCTransmitted = false;
		curCommsQueuePriority = 0;
	}
	catch (Exception) { }
}

bool InitLoop()
{
	if (subMode == 0)
	{
		InitConfiguration();

		List<IMyRemoteControl> remoteBlocks = new List<IMyRemoteControl>(0);
		GridTerminalSystem.GetBlocksOfType(remoteBlocks, (b) => { if (remote == null) { remote = b; } return false; });

		if (remote != null)
		{
			float currentValue = remote.SpeedLimit;
			remote.SpeedLimit = float.MaxValue;
			gridSpeedLimit = remote.SpeedLimit;
			remote.SpeedLimit = currentValue;
		}

		raycastHandler = new RaycastHandler(new List<IMyCameraBlock>(0));
		
		targetManager = new TargetManager();

		allyManager = new AllyManager();

		sortedEntityIds = new SortedDictionary<double, PDCTarget>();

		ReloadMainBlocks();

		ReloadMissilesAndTorpedos();

		guidanceCommsTargets = new Queue<MissileCommsTarget>();
		tracksCommsTargets = new Queue<PDCTarget>();

		settings.ManualAimRaycastDistance = Math.Min(Math.Max(settings.ManualAimRaycastDistance, 1000), 100000);

		igcTargetTracksListener = IGC.RegisterBroadcastListener(IGC_MSG_TARGET_TRACKS);
		igcTracksInfoListener = IGC.RegisterBroadcastListener(IGC_MSG_TRACKS_INFO);

		shipRadius = Me.CubeGrid.WorldAABB.HalfExtents.Length();

		if (settings.CheckSelfOcclusion)
		{
			List<IMyTerminalBlock> blocks = new List<IMyTerminalBlock>();
			GridTerminalSystem.GetBlocksOfType(blocks, (b) => { return (b is IMyMechanicalConnectionBlock || b is IMyShipConnector); });

			List<PDCOcclusionGrid> refBlocks = new List<PDCOcclusionGrid>();
			foreach (IMyTerminalBlock block in blocks)
			{
				if (block is IMyMechanicalConnectionBlock)
				{
					IMyMechanicalConnectionBlock mech = block as IMyMechanicalConnectionBlock;
					if (mech.Top != null)
					{
						refBlocks.Add(new PDCOcclusionGrid(mech.Top.CubeGrid, mech.Top.Position));
					}
				}
				else
				{
					IMyShipConnector connector = block as IMyShipConnector;
					if (connector != null && connector.OtherConnector != null)
					{
						refBlocks.Add(new PDCOcclusionGrid(connector.OtherConnector.CubeGrid, connector.OtherConnector.Position));
					}
				}
			}

			if (settings.UseAABBOcclusionChecker)
			{
				AABBPDCOcclusionChecker aabbChecker = new AABBPDCOcclusionChecker();
				iterOcclusionCreator = aabbChecker.Init(new PDCOcclusionGrid(Me.CubeGrid, Me.Position), refBlocks, settings.OcclusionExtraClearance, settings.OcclusionCheckerInitBlockLimit);

				occlusionChecker = aabbChecker;

				subMode = 1;
			}
			else
			{
				CubeExistsPDCOcclusionChecker cubeExistsChecker = new CubeExistsPDCOcclusionChecker(new PDCOcclusionGrid(Me.CubeGrid, Me.Position), refBlocks, settings.OcclusionExtraClearance);

				occlusionChecker = cubeExistsChecker;

				foreach (PDCTurret pdc in pdcList)
				{
					pdc.GridClearanceCheck = occlusionChecker;
				}

				subMode = 2;
			}
		}

		ForceJITCompilation();
	}

	if (subMode == 1)
	{
		if (iterOcclusionCreator.MoveNext())
		{
			Echo("--- Creating Occlusion Checker ---\nBlocks Processed:" + iterOcclusionCreator.Current);
			return false;
		}
		else
		{
			Echo("--- Occlusion Checker Created ---");

			iterOcclusionCreator.Dispose();
			iterOcclusionCreator = null;

			foreach (PDCTurret pdc in pdcList)
			{
				pdc.GridClearanceCheck = occlusionChecker;
			}

			subMode = 2;
		}
	}

	return true;
}

void InitConfiguration()
{
	int latestHashCode = Me.CustomData.GetHashCode();
	if (loadedCustomDataHashCode == 0 || loadedCustomDataHashCode != latestHashCode)
	{
		loadedCustomDataHashCode = latestHashCode;

		MyIni iniConfig = new MyIni();
		if (iniConfig.TryParse(Me.CustomData))
		{
			if (iniConfig.ContainsSection(INI_SECTION))
			{
				settings.MainBlocksReloadTicks = iniConfig.Get(INI_SECTION, "MainBlocksReloadTicks").ToInt32(settings.MainBlocksReloadTicks);

				settings.TargetTracksTransmitIntervalTicks = iniConfig.Get(INI_SECTION, "TargetTracksTransmitIntervalTicks").ToInt32(settings.TargetTracksTransmitIntervalTicks);

				settings.ManualAimBroadcastDurationTicks = iniConfig.Get(INI_SECTION, "ManualAimBroadcastDurationTicks").ToInt32(settings.ManualAimBroadcastDurationTicks);
				settings.ManualAimRaycastDistance = iniConfig.Get(INI_SECTION, "ManualAimRaycastDistance").ToDouble(settings.ManualAimRaycastDistance);
				settings.ManualAimRaycastRefreshInterval = iniConfig.Get(INI_SECTION, "ManualAimRaycastRefreshInterval").ToInt32(settings.ManualAimRaycastRefreshInterval);

				settings.MaxDesignatorUpdatesPerTick = iniConfig.Get(INI_SECTION, "MaxDesignatorUpdatesPerTick").ToInt32(settings.MaxDesignatorUpdatesPerTick);

				settings.MaxPDCUpdatesPerTick = iniConfig.Get(INI_SECTION, "MaxPDCUpdatesPerTick").ToSingle(settings.MaxPDCUpdatesPerTick);
				settings.MinPDCRefreshRate = iniConfig.Get(INI_SECTION, "MinPDCRefreshRate").ToInt32(settings.MinPDCRefreshRate);

				settings.UseDesignatorReset = iniConfig.Get(INI_SECTION, "UseDesignatorReset").ToBoolean(settings.UseDesignatorReset);
				settings.DesignatorResetInterval = iniConfig.Get(INI_SECTION, "DesignatorResetInterval").ToInt32(settings.DesignatorResetInterval);

				settings.UseRangeSweeper = iniConfig.Get(INI_SECTION, "UseRangeSweeper").ToBoolean(settings.UseRangeSweeper);
				settings.RangeSweeperPerTick = iniConfig.Get(INI_SECTION, "RangeSweeperPerTick").ToInt32(settings.RangeSweeperPerTick);
				settings.RangeSweeperInterval = iniConfig.Get(INI_SECTION, "RangeSweeperInterval").ToInt32(settings.RangeSweeperInterval);

				settings.TargetFoundHoldTicks = iniConfig.Get(INI_SECTION, "TargetFoundHoldTicks").ToInt32(settings.TargetFoundHoldTicks);

				settings.UsePDCSpray = iniConfig.Get(INI_SECTION, "UsePDCSpray").ToBoolean(settings.UsePDCSpray);
				settings.PDCSprayMinTargetSize = iniConfig.Get(INI_SECTION, "PDCSprayMinTargetSize").ToDouble(settings.PDCSprayMinTargetSize);

				settings.MaxRaycastTrackingDistance = iniConfig.Get(INI_SECTION, "MaxRaycastTrackingDistance").ToDouble(settings.MaxRaycastTrackingDistance);

				settings.RaycastTargetRefreshTicks = iniConfig.Get(INI_SECTION, "RaycastTargetRefreshTicks").ToInt32(settings.RaycastTargetRefreshTicks);
				settings.RaycastGlobalRefreshTicks = Math.Max(iniConfig.Get(INI_SECTION, "RaycastGlobalRefreshTicks").ToInt32(settings.RaycastGlobalRefreshTicks), 1);
			
				settings.PriorityMinRefreshTicks = iniConfig.Get(INI_SECTION, "PriorityMinRefreshTicks").ToInt32(settings.PriorityMinRefreshTicks);
				settings.PriorityMaxRefreshTicks = iniConfig.Get(INI_SECTION, "PriorityMaxRefreshTicks").ToInt32(settings.PriorityMaxRefreshTicks);
				settings.PriorityGlobalRefreshTicks = Math.Max(iniConfig.Get(INI_SECTION, "priorityGlobalRefreshTicks").ToInt32(settings.PriorityGlobalRefreshTicks), 1);

				settings.TargetSlippedTicks = iniConfig.Get(INI_SECTION, "TargetSlippedTicks").ToInt32(settings.TargetSlippedTicks);
				settings.TargetLostTicks = iniConfig.Get(INI_SECTION, "TargetLostTicks").ToInt32(settings.TargetLostTicks);

				settings.RandomOffsetProbeInterval = iniConfig.Get(INI_SECTION, "RandomOffsetProbeInterval").ToInt32(settings.RandomOffsetProbeInterval);

				settings.RaycastExtensionDistance = iniConfig.Get(INI_SECTION, "RaycastExtensionDistance").ToDouble(settings.RaycastExtensionDistance);

				settings.MinTargetSizeEngage = iniConfig.Get(INI_SECTION, "MinTargetSizeEngage").ToDouble(settings.MinTargetSizeEngage);
				settings.MinTargetSizePriority = iniConfig.Get(INI_SECTION, "MinTargetSizePriority").ToDouble(settings.MinTargetSizePriority);

				settings.MissileMinTargetSize = iniConfig.Get(INI_SECTION, "MissileMinTargetSize").ToDouble(settings.MissileMinTargetSize);
				settings.MissileCountPerSize = iniConfig.Get(INI_SECTION, "MissileCountPerSize").ToDouble(settings.MissileCountPerSize);

				settings.MaxMissileLaunchDistance = iniConfig.Get(INI_SECTION, "MaxMissileLaunchDistance").ToDouble(settings.MaxMissileLaunchDistance);
				settings.MissileOffsetRadiusFactor = iniConfig.Get(INI_SECTION, "MissileOffsetRadiusFactor").ToDouble(settings.MissileOffsetRadiusFactor);
				settings.MissileOffsetProbability = iniConfig.Get(INI_SECTION, "MissileOffsetProbability").ToDouble(settings.MissileOffsetProbability);

				settings.MissileStaggerWaitTicks = iniConfig.Get(INI_SECTION, "MissileStaggerWaitTicks").ToInt32(settings.MissileStaggerWaitTicks);
				settings.MissileReassignIntervalTicks = iniConfig.Get(INI_SECTION, "MissileReassignIntervalTicks").ToInt32(settings.MissileReassignIntervalTicks);
				settings.MissilePBGridReloadTicks = iniConfig.Get(INI_SECTION, "MissilePBGridReloadTicks").ToInt32(settings.MissilePBGridReloadTicks);
				settings.MissileTransmitDurationTicks = iniConfig.Get(INI_SECTION, "MissileTransmitDurationTicks").ToInt32(settings.MissileTransmitDurationTicks);
				settings.MissileTransmitIntervalTicks = iniConfig.Get(INI_SECTION, "MissileTransmitIntervalTicks").ToInt32(settings.MissileTransmitIntervalTicks);

				settings.MissileLaunchSpeedLimit = iniConfig.Get(INI_SECTION, "MissileLaunchSpeedLimit").ToDouble(settings.MissileLaunchSpeedLimit);

				settings.PDCFireDotLimit = iniConfig.Get(INI_SECTION, "PDCFireDotLimit").ToDouble(settings.PDCFireDotLimit);
				settings.ConstantFireMode = iniConfig.Get(INI_SECTION, "ConstantFireMode").ToBoolean(settings.ConstantFireMode);

				settings.RotorUseLimitSnap = iniConfig.Get(INI_SECTION, "RotorUseLimitSnap").ToBoolean(settings.RotorUseLimitSnap);

				settings.RotorCtrlDeltaGain = iniConfig.Get(INI_SECTION, "RotorCtrlDeltaGain").ToSingle(settings.RotorCtrlDeltaGain);
			
				settings.ReloadCheckTicks = iniConfig.Get(INI_SECTION, "ReloadCheckTicks").ToInt32(settings.ReloadCheckTicks);
				settings.ReloadedCooldownTicks = iniConfig.Get(INI_SECTION, "ReloadedCooldownTicks").ToInt32(settings.ReloadedCooldownTicks);

				settings.ReloadMaxAngle = iniConfig.Get(INI_SECTION, "ReloadMaxAngle").ToDouble(settings.ReloadMaxAngle);
				settings.ReloadLockStateAngle = iniConfig.Get(INI_SECTION, "ReloadLockStateAngle").ToDouble(settings.ReloadLockStateAngle);

				settings.DisplaysRefreshInterval = Math.Max(iniConfig.Get(INI_SECTION, "DisplaysRefreshInterval").ToInt32(settings.DisplaysRefreshInterval), 1);

				settings.AllyTrackLostTicks = iniConfig.Get(INI_SECTION, "AllyTrackLostTicks").ToInt32(settings.AllyTrackLostTicks);

				settings.CheckSelfOcclusion = iniConfig.Get(INI_SECTION, "CheckSelfOcclusion").ToBoolean(settings.CheckSelfOcclusion);
				settings.UseAABBOcclusionChecker = iniConfig.Get(INI_SECTION, "UseAABBOcclusionChecker").ToBoolean(settings.UseAABBOcclusionChecker);
				settings.OcclusionExtraClearance = iniConfig.Get(INI_SECTION, "OcclusionExtraClearance").ToSingle(settings.OcclusionExtraClearance);
				settings.OcclusionCheckerInitBlockLimit = iniConfig.Get(INI_SECTION, "OcclusionCheckerInitBlockLimit").ToInt32(settings.OcclusionCheckerInitBlockLimit);
			}
		}
	}
}

void ReloadMainBlocks()
{
	CompileDesignators();

	CompileRaycastCameras();
	
	CompilePDCGroups();

	CompileManualTargeters();

	CompileDisplayPanels();
}

void CompileDesignators()
{
	List<IMyBlockGroup> groups = new List<IMyBlockGroup>();
	List<Designator> newDesignators = new List<Designator>();
	GridTerminalSystem.GetBlockGroups(groups, (b) => { return b.Name.IndexOf(DESIGNATOR_GRP_TAG, StringComparison.OrdinalIgnoreCase) > -1; });
	foreach (IMyBlockGroup group in groups)
	{
		group.GetBlocksOfType<IMyLargeTurretBase>(dummyBlocks, (block) =>
		{
			Designator designator = new Designator(block as IMyLargeTurretBase);
			newDesignators.Add(designator);
			return false;
		});
		break;
	}
	designators = newDesignators;
	designatorTargetRR = new RoundRobin<Designator>(newDesignators, FuncDesignatorHasTarget);
	designatorOperationRR = new RoundRobin<Designator>(newDesignators, FuncDesignatorIsWorking);
}

void CompileRaycastCameras()
{
	List<IMyBlockGroup> groups = new List<IMyBlockGroup>();
	List<IMyCameraBlock> newRaycastCameras = new List<IMyCameraBlock>();
	GridTerminalSystem.GetBlockGroups(groups, (b) => { return b.Name.IndexOf(RAYCAST_CAMERA_GRP_TAG, StringComparison.OrdinalIgnoreCase) > -1; });
	foreach (IMyBlockGroup group in groups)
	{
		group.GetBlocksOfType(newRaycastCameras, (b) => { b.EnableRaycast = true; b.Enabled = true; return true; });
		break;
	}
	raycastCameras = newRaycastCameras;
	raycastHandler.Cameras = raycastCameras;
}

void CompilePDCGroups()
{
	Dictionary<long, PDCTurret> previousTurrets = null;
	{
		if (pdcList != null && pdcList.Count > 0)
		{
			previousTurrets = new Dictionary<long, PDCTurret>();
			foreach (PDCTurret pdc in pdcList)
			{
				if (!previousTurrets.ContainsKey(pdc.AzimuthRotor.EntityId))
				{
					previousTurrets.Add(pdc.AzimuthRotor.EntityId, pdc);
				}
			}
		}
	}

	HashSet<IMyTerminalBlock> uniqueFilter = new HashSet<IMyTerminalBlock>();

	List<IMyMotorStator> pdcAzimuthRotors = new List<IMyMotorStator>();
	Dictionary<long, List<IMyMotorStator>> pdcElevationRotors = new Dictionary<long, List<IMyMotorStator>>();
	Dictionary<long, List<IMyUserControllableGun>> pdcWeapons = new Dictionary<long, List<IMyUserControllableGun>>();
	Dictionary<long, IMyTerminalBlock> pdcAimBlock = new Dictionary<long, IMyTerminalBlock>();
	Dictionary<long, IMyShipController> pdcRemote = new Dictionary<long, IMyShipController>();
	Dictionary<long, IMyShipConnector> pdcConnector = new Dictionary<long, IMyShipConnector>();

	List<IMyBlockGroup> groups = new List<IMyBlockGroup>();
	List<PDCTurret> newPDCList = new List<PDCTurret>();
	GridTerminalSystem.GetBlockGroups(groups, (b) => { return b.Name.IndexOf(PDC_GRP_TAG, StringComparison.OrdinalIgnoreCase) > -1; });
	foreach (IMyBlockGroup group in groups)
	{
		group.GetBlocksOfType(dummyBlocks, (block) =>
		{
			if (Me.IsSameConstructAs(block) && uniqueFilter.Add(block))
			{
				if (block is IMyMotorStator)
				{
					if (NameContains(block, AZIMUTH_ROTOR_TAG))
					{
						pdcAzimuthRotors.Add(block as IMyMotorStator);
					}
					else if (NameContains(block, ELEVATION_ROTOR_TAG))
					{
						if (pdcElevationRotors.ContainsKey(block.CubeGrid.EntityId))
						{
							pdcElevationRotors[block.CubeGrid.EntityId].Add(block as IMyMotorStator);
						}
						else
						{
							List<IMyMotorStator> list = new List<IMyMotorStator>();
							list.Add(block as IMyMotorStator);
							pdcElevationRotors.Add(block.CubeGrid.EntityId, list);
						}
					}
				}
				else if (block is IMyUserControllableGun)
				{
					if (pdcWeapons.ContainsKey(block.CubeGrid.EntityId))
					{
						pdcWeapons[block.CubeGrid.EntityId].Add(block as IMyUserControllableGun);
					}
					else
					{
						List<IMyUserControllableGun> list = new List<IMyUserControllableGun>();
						list.Add(block as IMyUserControllableGun);
						pdcWeapons.Add(block.CubeGrid.EntityId, list);
					}
				}
				else if (block is IMyShipController)
				{
					if (!pdcRemote.ContainsKey(block.CubeGrid.EntityId))
					{
						pdcRemote.Add(block.CubeGrid.EntityId, block as IMyShipController);
					}
				}
				else if (block is IMyShipConnector)
				{
					if (!pdcConnector.ContainsKey(block.CubeGrid.EntityId))
					{
						if (NameContains(block, RELOAD_CONNECTOR_TAG))
						{
							pdcConnector.Add(block.CubeGrid.EntityId, block as IMyShipConnector);
						}
					}
				}
				else if (NameContains(block, AIM_BLOCK_TAG))
				{
					if (!pdcAimBlock.ContainsKey(block.CubeGrid.EntityId))
					{
						pdcAimBlock.Add(block.CubeGrid.EntityId, block);
					}
				}
			}
			return false;
		});
	}

	foreach (IMyMotorStator azimuthRotor in pdcAzimuthRotors)
	{
		if (azimuthRotor.TopGrid != null)
		{
			List<IMyMotorStator> checkElevationRotors;
			if (pdcElevationRotors.TryGetValue(azimuthRotor.TopGrid.EntityId, out checkElevationRotors))
			{
				List<IMyMotorStator> elevationRotors = new List<IMyMotorStator>();
				List<List<IMyUserControllableGun>> weapons = new List<List<IMyUserControllableGun>>();
				List<IMyTerminalBlock> aimBlocks = new List<IMyTerminalBlock>();

				WeaponProfile profile = null;

				IMyTerminalBlock connectorRefBlock = null;

				foreach (IMyMotorStator elevationRotor in checkElevationRotors)
				{
					if (elevationRotor.TopGrid != null)
					{
						List<IMyUserControllableGun> checkWeapons;
						if (pdcWeapons.TryGetValue(elevationRotor.TopGrid.EntityId, out checkWeapons))
						{
							IMyTerminalBlock aimBlock;
							if (pdcAimBlock.ContainsKey(elevationRotor.TopGrid.EntityId))
							{
								aimBlock = pdcAimBlock[elevationRotor.TopGrid.EntityId];
							}
							else
							{
								aimBlock = checkWeapons[0];
							}

							elevationRotors.Add(elevationRotor);
							weapons.Add(checkWeapons);
							aimBlocks.Add(aimBlock);

							if (profile == null && checkWeapons.Count > 0)
							{
								profile = GetWeaponProfile(checkWeapons[0]);
							}

							if (connectorRefBlock == null)
							{
								connectorRefBlock = aimBlock;
							}
						}
					}
				}

				if (elevationRotors.Count > 0)
				{
					IMyShipController controller;
					if (pdcRemote.ContainsKey(elevationRotors[0].TopGrid.EntityId))
					{
						controller = pdcRemote[elevationRotors[0].TopGrid.EntityId];
					}
					else
					{
						controller = remote;
					}

					if (profile == null)
					{
						profile = weaponProfiles.gatlingProfile;
					}

					PDCTurret pdc = new PDCTurret(azimuthRotor.CustomName, azimuthRotor, elevationRotors, aimBlocks, controller, weapons, profile, settings);

					IMyShipConnector baseConnector = null;
					if (pdcConnector.ContainsKey(elevationRotors[0].CubeGrid.EntityId))
					{
						baseConnector = pdcConnector[elevationRotors[0].CubeGrid.EntityId];
					}

					IMyShipConnector turretConnector = null;
					if (pdcConnector.ContainsKey(connectorRefBlock.CubeGrid.EntityId))
					{
						turretConnector = pdcConnector[connectorRefBlock.CubeGrid.EntityId];
					}

					if (baseConnector != null && turretConnector != null)
					{
						pdc.ReloadBaseConnector = baseConnector;
						pdc.ReloadTurretConnector = turretConnector;
					}

					if (previousTurrets != null && previousTurrets.ContainsKey(pdc.AzimuthRotor.EntityId))
					{
						pdc.TransferTarget(previousTurrets[pdc.AzimuthRotor.EntityId]);
					}
					else
					{
						pdc.ReleaseWeapons(true);
						pdc.ResetRotors();
					}

					newPDCList.Add(pdc);
				}
			}
		}
	}

	if (settings.MaxPDCUpdatesPerTick == 0)
	{
		curPDCUpdatesPerTick = Math.Max((int)Math.Ceiling(settings.MinPDCRefreshRate * newPDCList.Count / (double)TICKS_PER_SECOND), 1);
		curPDCUpdatesSkipTicks = 0;
	}
	else
	{
		if (settings.MaxPDCUpdatesPerTick < 1f && settings.MaxPDCUpdatesPerTick > 0)
		{
			curPDCUpdatesPerTick = 1;
			curPDCUpdatesSkipTicks = Math.Min((int)Math.Floor(1f / settings.MaxPDCUpdatesPerTick), TICKS_PER_SECOND);
		}
		else
		{
			curPDCUpdatesPerTick = Math.Max(Math.Min(newPDCList.Count, (int)Math.Ceiling(settings.MaxPDCUpdatesPerTick)), 1);
			curPDCUpdatesSkipTicks = 0;
		}
	}

	double refreshPerPass = (double)newPDCList.Count / curPDCUpdatesPerTick;
	double targetedCountGainPerHit = refreshPerPass / Math.Max(settings.PriorityMaxRefreshTicks, 1);

	foreach (PDCTurret pdc in newPDCList)
	{
		pdc.TargetedCountGainPerHit = targetedCountGainPerHit;
		pdc.ReloadMaxAngleRadians =  MathHelperD.ToRadians(settings.ReloadMaxAngle);
		pdc.ReloadLockStateAngleRadians =  MathHelperD.ToRadians(settings.ReloadLockStateAngle);
	}

	pdcList = newPDCList;
	pdcAssignRR = new RoundRobin<PDCTurret>(newPDCList, FuncPDCIsWorking);
	pdcFireRR = new RoundRobin<PDCTurret>(newPDCList, FuncPDCIsWorking);
}

void CompileManualTargeters()
{
	List<ManualPDCTarget> newManualTargeters = new List<ManualPDCTarget>();
	Dictionary<string, ManualPDCTarget> newManualTargetersLookup = new Dictionary<string, ManualPDCTarget>();

	List<IMyBlockGroup> groups = new List<IMyBlockGroup>();
	GridTerminalSystem.GetBlockGroups(groups, (b) => { return NameContains(b, MANUAL_AIMING_GRP_TAG); });
	foreach (IMyBlockGroup group in groups)
	{
		group.GetBlocksOfType(dummyBlocks, (b) =>
		{
			int codeId = 1;
			int pos = b.CustomName.IndexOf(MANUAL_AIMING_GRP_PREFIX, StringComparison.OrdinalIgnoreCase);
			if (pos > -1)
			{
				if (pos + MANUAL_AIMING_AIMBLOCK_TAG.Length < b.CustomName.Length)
				{
					if (int.TryParse($"{b.CustomName[pos + MANUAL_AIMING_GRP_PREFIX.Length]}", out codeId))
					{
						if (codeId < 1 || codeId > 9)
						{
							codeId = 1;
						}
					}
					else
					{
						codeId = 1;
					}
				}
			}

			string codeIdStr = MANUAL_AIMING_GRP_PREFIX + codeId;
			ManualPDCTarget manualTarget;

			if (newManualTargetersLookup.ContainsKey(codeIdStr))
			{
				manualTarget = newManualTargetersLookup[codeIdStr];
			}
			else
			{
				if (manualTargetersLookup != null && manualTargetersLookup.ContainsKey(codeIdStr))
				{
					manualTarget = manualTargetersLookup[codeIdStr];
				}
				else
				{
					manualTarget = new ManualPDCTarget(codeIdStr, null, null, null);
					manualTarget.MaxManualRaycastDistance = settings.ManualAimRaycastDistance;
				}

				newManualTargeters.Add(manualTarget);
				newManualTargetersLookup.Add(codeIdStr, manualTarget);
			}

			if (NameContains(b, MANUAL_AIMING_AIMBLOCK_TAG))
			{
				manualTarget.AimingBlock = b;
				manualTarget.AimingTurret = b as IMyLargeTurretBase;
			}
			else if (NameContains(b, MANUAL_AIMING_ALERT_TAG))
			{
				manualTarget.AlertBlock = b;
			}
			else if (NameContains(b, MANUAL_AIMING_STATUS_TAG))
			{
				IMyTextSurfaceProvider surfaceProvider = b as IMyTextSurfaceProvider;
				if (surfaceProvider != null)
				{
					try { manualTarget.DisplayStatus = surfaceProvider.GetSurface(0); } catch (Exception) { }
				}
			}
			else
			{
				if (manualTarget.AimingBlock == null)
				{
					manualTarget.AimingBlock = b;
					manualTarget.AimingTurret = b as IMyLargeTurretBase;
				}

				if (manualTarget.DisplayStatus == null && b is IMyTextSurfaceProvider)
				{
					try { manualTarget.DisplayStatus = ((IMyTextSurfaceProvider)b).GetSurface(0); } catch (Exception) { }
				}
			}

			return false;
		});

		break;
	}

	int index = 0;

	while (index < newManualTargeters.Count)
	{
		ManualPDCTarget manualTarget = newManualTargeters[index];
		if (manualTarget.AimingBlock == null)
		{
			if (index + 1 == newManualTargeters.Count)
			{
				newManualTargeters.RemoveAt(index);
			}
			else
			{
				newManualTargeters[index] = newManualTargeters[newManualTargeters.Count - 1];
				newManualTargeters.RemoveAt(newManualTargeters.Count - 1);
			}
			newManualTargetersLookup.Remove(manualTarget.CodeId);
		}

		index++;
	}

	manualTargeters = newManualTargeters;
	manualTargetersLookup = newManualTargetersLookup;
}

void CompileDisplayPanels()
{
	List<IMyBlockGroup> groups = new List<IMyBlockGroup>();
	displayPanels = new List<IMyTextSurface>();
	GridTerminalSystem.GetBlockGroups(groups, (b) => { return b.Name.IndexOf(DISPLAY_GRP_TAG, StringComparison.OrdinalIgnoreCase) > -1; });
	foreach (IMyBlockGroup group in groups)
	{
		group.GetBlocksOfType(displayPanels, (b) =>
		{
			return true;
		});
		break;
	}
}

void ReloadMissilesAndTorpedos()
{
	List<IMyBlockGroup> groups = new List<IMyBlockGroup>();

	missileComputers = new List<IMyProgrammableBlock>();
	GridTerminalSystem.GetBlockGroups(groups, (b) => { return NameContains(b, MISSILE_PB_GRP_TAG); });
	foreach (IMyBlockGroup group in groups)
	{
		group.GetBlocksOfType(missileComputers, (b) =>
		{
			if (b.Enabled && Me.IsSameConstructAs(b))
			{
				return AGMChecker.CheckSave(b);
			}
			else
			{
				return false;
			}
		});
		break;
	}

	groups.Clear();
	torpedoComputers = new List<IMyProgrammableBlock>();
	GridTerminalSystem.GetBlockGroups(groups, (b) => { return NameContains(b, TORPEDO_PB_GRP_TAG); });
	foreach (IMyBlockGroup group in groups)
	{
		group.GetBlocksOfType(torpedoComputers, (b) =>
		{
			if (b.Enabled && Me.IsSameConstructAs(b))
			{
				return AGMChecker.CheckSave(b);
			}
			else
			{
				return false;
			}
		});
		break;
	}
}

void ProcessCommsMessage()
{
	while (igcTargetTracksListener.HasPendingMessage)
	{
		object data = igcTargetTracksListener.AcceptMessage().Data;
		if (data is MyTuple<long, long, Vector3D, Vector3D, double>)
		{
			//[Notes] TargetTracksData(SenderId, TargetEntityId, TargetPosition, TargetVelocity, TargetSizeSq) => MyTuple<long, long, Vector3D, Vector3D, double>
			MyTuple<long, long, Vector3D, Vector3D, double> targetTracksData = (MyTuple<long, long, Vector3D, Vector3D, double>)data;
			if (!targetManager.TargetExists(targetTracksData.Item2) || clock - targetManager.GetTarget(targetTracksData.Item2).LastDetectedClock >= settings.TargetSlippedTicks)
			{
				TargetData targetData = new TargetData();
				targetData.EntityId = targetTracksData.Item2;
				targetData.Position = targetTracksData.Item3;
				targetData.Velocity = targetTracksData.Item4;
				targetManager.UpdateTarget(targetData, clock - 1, false);

				if (targetTracksData.Item5 > 0)
				{
					PDCTarget target = targetManager.GetTarget(targetData.EntityId);
					if (target != null && target.TargetSizeSq == 0)
					{
						target.TargetSizeSq = targetTracksData.Item5;
					}
				}
			}
		}
	}

	while (igcTracksInfoListener.HasPendingMessage)
	{
		object data = igcTracksInfoListener.AcceptMessage().Data;
		if (data is MyTuple<long, long, Vector3D, Vector3D, double>)
		{
			//[Notes] TargetTracksData(TargetEntityId, TargetPosition, TargetVelocity, TargetSizeSq, Flags[IsFriendly,IsLargeGrid], Timestamp) => MyTuple<long, Vector3D, Vector3D, double, int, long>
			MyTuple<long, Vector3D, Vector3D, double, int, long> tracksInfoData = (MyTuple<long, Vector3D, Vector3D, double, int, long>)data;
			if (!targetManager.TargetExists(tracksInfoData.Item1) || clock - targetManager.GetTarget(tracksInfoData.Item1).LastDetectedClock >= settings.TargetSlippedTicks)
			{
				if ((tracksInfoData.Item5 & (int)TrackTypeEnum.IsFriendly) == 0)
				{
					TargetData targetData = new TargetData();
					targetData.EntityId = tracksInfoData.Item1;
					targetData.Position = tracksInfoData.Item2;
					targetData.Velocity = tracksInfoData.Item3;

					targetManager.UpdateTarget(targetData, clock - 1, false);

					if (tracksInfoData.Item4 > 0)
					{
						PDCTarget target = targetManager.GetTarget(targetData.EntityId);
						if (target != null)
						{
							if (target.TargetSizeSq == 0)
							{
								target.TargetSizeSq = tracksInfoData.Item4;
							}

							target.IsLargeGrid = (tracksInfoData.Item5 & (int)TrackTypeEnum.IsLargeGrid) > 0;
						}
					}
				}
				else
				{
					AllyTrack ally = new AllyTrack(tracksInfoData.Item1);
					ally.Position = tracksInfoData.Item2;
					ally.Velocity = tracksInfoData.Item3;
					ally.SizeSq = tracksInfoData.Item4;
					ally.IsLargeGrid = (tracksInfoData.Item5 & (int)TrackTypeEnum.IsLargeGrid) > 0;
					
					allyManager.UpdateAlly(ally, clock);
				}
			}
		}
	}
}

void ProcessCommands(string arguments)
{
	string[] tokens = arguments.Split(splitDelimiterCommand, StringSplitOptions.RemoveEmptyEntries);
	if (tokens.Length == 0) return;

	string command = tokens[0].Trim().ToUpper();

	ManualPDCTarget manualTarget = null;

	if (command.StartsWith(MANUAL_AIMING_GRP_PREFIX, StringComparison.OrdinalIgnoreCase))
	{
		int codeId;
		if (command.Length == MANUAL_AIMING_GRP_PREFIX.Length)
		{
			codeId = 1;
		}
		else if (!int.TryParse(command.Substring(MANUAL_AIMING_GRP_PREFIX.Length).Trim(), out codeId))
		{
			codeId = 0;
		}

		if (codeId >= 1 && manualTargetersLookup.ContainsKey(MANUAL_AIMING_GRP_PREFIX + codeId))
		{
			manualTarget = manualTargetersLookup[MANUAL_AIMING_GRP_PREFIX + codeId];
		}

		if (manualTarget != null && tokens.Length <= 1)
		{
			return;
		}
	}

	if (manualTarget != null)
	{
		command = tokens[1].Trim().ToUpper();
	}
	else if (manualTargeters?.Count > 0)
	{
		manualTarget = manualTargeters[0];
	}

	switch (command)
	{
	case MANUAL_LAUNCH_CMD:
		bool useTorpedo = TokenContainsMatch(tokens, MANUAL_LAUNCH_TORPEDO_TAG);
		List<IMyProgrammableBlock> guidanceComputers = (useTorpedo ? torpedoComputers : missileComputers);

		if (TokenContainsMatch(tokens, MANUAL_LAUNCH_LARGEST_TAG))
		{
			PDCTarget largestTarget = targetManager.FindLargestTarget();
			if (largestTarget != null)
			{
				LaunchManualForTarget(guidanceComputers, largestTarget, manualTarget?.TargetingPointType ?? TargetingPointTypeEnum.Center, manualTarget?.OffsetPoint, useTorpedo);
			}
		}
		else if (manualTarget != null)
		{
			bool useExtendedMissileDistance = TokenContainsMatch(tokens, MANUAL_LAUNCH_EXTENDED_TAG);

			if (manualTarget.SelectedEntityId > 0 && targetManager.TargetExists(manualTarget.SelectedEntityId))
			{
				PDCTarget currentTarget = targetManager.GetTarget(manualTarget.SelectedEntityId);
				if (currentTarget != null)
				{
					if (useExtendedMissileDistance)
					{
						currentTarget.MaxAllowedMissileLaunchDistance = Math.Max(manualTarget.MaxManualRaycastDistance, settings.MaxMissileLaunchDistance);
					}
					LaunchManualForTarget(guidanceComputers, currentTarget, manualTarget.TargetingPointType, manualTarget.OffsetPoint, useTorpedo);
				}
			}
			else
			{
				manualTarget.Enabled = true;
				manualTarget.SelectedEntityId = -1;
				manualTarget.OffsetPoint = Vector3D.Zero;
				manualTarget.Position = manualTarget.AimingBlock.WorldMatrix.Translation + (manualTarget.GetForwardViewDirection() * manualTarget.MaxManualRaycastDistance);
				manualTarget.MaxAllowedRaycastDistance = Math.Max(manualTarget.MaxManualRaycastDistance, settings.MaxRaycastTrackingDistance);
				manualTarget.MaxAllowedMissileLaunchDistance = (useExtendedMissileDistance ? Math.Max(manualTarget.MaxManualRaycastDistance, settings.MaxMissileLaunchDistance) : 0);

				LaunchManualForTarget(guidanceComputers, manualTarget, manualTarget.TargetingPointType, manualTarget.OffsetPoint, useTorpedo);
			}
		}
		break;
	case MANUAL_AIM_TRACK_TAG:
		if (manualTarget != null)
		{
			if (manualTarget.Enabled && manualTarget.SelectedEntityId == -1)
			{
				manualTarget.Enabled = false;
				manualTarget.OffsetPoint = Vector3D.Zero;
			}
			else
			{
				bool useExtendedMissileDistance = TokenContainsMatch(tokens, MANUAL_LAUNCH_EXTENDED_TAG);

				manualTarget.Enabled = true;
				manualTarget.SelectedEntityId = -1;
				manualTarget.OffsetPoint = Vector3D.Zero;
				manualTarget.Position = manualTarget.AimingBlock.WorldMatrix.Translation + (manualTarget.GetForwardViewDirection() * manualTarget.MaxManualRaycastDistance);
				manualTarget.MaxAllowedRaycastDistance = Math.Max(manualTarget.MaxManualRaycastDistance, settings.MaxRaycastTrackingDistance);
				manualTarget.MaxAllowedMissileLaunchDistance = (useExtendedMissileDistance ? Math.Max(manualTarget.MaxManualRaycastDistance, settings.MaxMissileLaunchDistance) : 0);
			}
		}
		break;
	case MANUAL_AIM_RELEASE_TAG:
		if (manualTarget != null)
		{
			manualTarget.Enabled = false;
			manualTarget.SelectedEntityId = -1;
			manualTarget.OffsetPoint = Vector3D.Zero;
		}
		break;
	case MANUAL_AIM_VALUE_SET_TAG:
		if (manualTarget != null && tokens.Length >= 3)
		{
			switch (tokens[2].ToUpper().Trim())
			{
			case MANUAL_AIM_VALUE_SET_CENTER_TAG:
				manualTarget.TargetingPointType = TargetingPointTypeEnum.Center;
				break;
			case MANUAL_AIM_VALUE_SET_OFFSET_TAG:
				manualTarget.TargetingPointType = TargetingPointTypeEnum.Offset;
				break;
			case MANUAL_AIM_VALUE_SET_RANDOM_TAG:
				manualTarget.TargetingPointType = TargetingPointTypeEnum.Random;
				break;
			case MANUAL_AIM_VALUE_SET_RANGE_TAG:
				if (tokens.Length >= 4)
				{
					double value;
					if (double.TryParse(tokens[3].Trim(), out value))
					{
						manualTarget.MaxManualRaycastDistance = Math.Min(Math.Max(value, 1000), 100000);;
					}
				}
				break;
			}
		}
		break;
	case MANUAL_AIM_VALUE_CYCLE_OFFSET_TAG:
		if (manualTarget != null)
		{
			manualTarget.TargetingPointType = (TargetingPointTypeEnum)(((int)manualTarget.TargetingPointType + 1) % 3);
		}
		break;
	case MANUAL_AIM_VALUE_INC_RANGE_TAG:
		if (manualTarget != null)
		{
			manualTarget.MaxManualRaycastDistance = Math.Min(Math.Max(manualTarget.MaxManualRaycastDistance + 1000, 1000), 100000);;
		}
		break;
	case MANUAL_AIM_VALUE_DEC_RANGE_TAG:
		if (manualTarget != null)
		{
			manualTarget.MaxManualRaycastDistance = Math.Min(Math.Max(manualTarget.MaxManualRaycastDistance - 1000, 1000), 100000);;
		}
		break;
	case CMD_TOGGLE_ON_OFF:
		switchedOn = !switchedOn;

		if (!switchedOn)
		{
			foreach (PDCTurret pdc in pdcList)
			{
				pdc.TargetInfo = null;
				pdc.ReleaseWeapons(true);
				pdc.ResetRotors();
			}
		}
		break;
	case CMD_TOGGLE_ON:
		switchedOn = true;
		break;
	case CMD_TOGGLE_OFF:
		switchedOn = false;

		foreach (PDCTurret pdc in pdcList)
		{
			pdc.TargetInfo = null;
			pdc.ReleaseWeapons(true);
			pdc.ResetRotors();
		}
		break;
	case CMD_DEBUG_MODE:
		debugMode = !debugMode;
		break;
	}
}

void UpdateManualAimRaycast()
{
	if (manualTargeters?.Count > 0 && settings.ManualAimRaycastRefreshInterval > 0 && (clock % settings.ManualAimRaycastRefreshInterval < manualTargeters.Count))
	{
		ManualPDCTarget manualTarget = manualTargeters[clock % settings.ManualAimRaycastRefreshInterval];
		
		if (IsWorking(manualTarget.AimingBlock) && manualTarget.Enabled)
		{
			Vector3D aimVector = manualTarget.GetForwardViewDirection();
			Vector3D targetPosition = manualTarget.AimingBlock.WorldMatrix.Translation + (aimVector * manualTarget.MaxManualRaycastDistance);

			MyDetectedEntityInfo entityInfo;
			raycastHandler.Raycast(ref targetPosition, out entityInfo, settings.RaycastExtensionDistance);

			if (!entityInfo.IsEmpty() && IsValidTarget(ref entityInfo))
			{
				manualTarget.SelectedEntityId = entityInfo.EntityId;
				manualTarget.Position = (entityInfo.HitPosition.HasValue ? entityInfo.HitPosition.Value : entityInfo.Position);
				manualTarget.Velocity = entityInfo.Velocity;
				manualTarget.DetectedClock = clock;
				
				Vector3D estimatedTargetOffsetPoint = manualTarget.Position - manualTarget.AimingBlock.WorldMatrix.Translation;
				estimatedTargetOffsetPoint = Vector3D.ProjectOnVector(ref estimatedTargetOffsetPoint, ref aimVector) + manualTarget.AimingBlock.WorldMatrix.Translation;
				manualTarget.OffsetPoint = Vector3D.TransformNormal(estimatedTargetOffsetPoint - entityInfo.Position, MatrixD.Transpose(entityInfo.Orientation));

				//Vector3ToBase64

				PDCTarget target;
				targetManager.UpdateTarget(ref entityInfo, clock, out target);

				if (target != null)
				{
					manualTarget.Enabled = false;
					
					if (IsWorking(manualTarget.AlertBlock))
					{
						if (manualTarget.AlertBlock is IMySoundBlock)
						{
							((IMySoundBlock)manualTarget.AlertBlock).Play();
						}
						else if (manualTarget.AlertBlock is IMyTimerBlock)
						{
							((IMyTimerBlock)manualTarget.AlertBlock).Trigger();
						}
						else
						{
							IMyFunctionalBlock funcBlock = manualTarget.AlertBlock as IMyFunctionalBlock;
							if (funcBlock != null && !funcBlock.Enabled)
							{
								funcBlock.Enabled = true;
							}
						}
					}

					target.MaxAllowedRaycastDistance = manualTarget.MaxAllowedRaycastDistance;
					target.MaxAllowedMissileLaunchDistance = manualTarget.MaxAllowedMissileLaunchDistance;

					foreach (MissileCommsTarget commsTarget in guidanceCommsTargets)
					{
						if (commsTarget.Target == manualTarget)
						{
							commsTarget.Target = target;
							commsTarget.TransmitUntilClock = clock + settings.MissileTransmitDurationTicks;
						}
					}
				}
			}
		}
		else if (manualTarget.SelectedEntityId > 0 && !targetManager.TargetExists(manualTarget.SelectedEntityId))
		{
			manualTarget.SelectedEntityId = -1;
		}
	}
}

void UpdateDesignatorTargets()
{
	designatorTargetRR.Begin();

	int maxCount = (settings.MaxDesignatorUpdatesPerTick == 0 ? designators.Count : settings.MaxDesignatorUpdatesPerTick);
	for (int i = 0; i < maxCount; i++)
	{
		Designator designator = designatorTargetRR.GetNext();
		if (designator != null)
		{
			MyDetectedEntityInfo entityInfo = designator.Turret.GetTargetedEntity();

			if (targetManager.UpdateTarget(ref entityInfo, clock))
			{
				nextPriorityRefreshClock = clock + settings.PriorityMinRefreshTicks;
			}

			designatorTargetAcquiredClock = clock;

			if (settings.UseDesignatorReset)
			{
				if (clock >= designator.NextResetClock)
				{
					designator.Turret.ResetTargetingToDefault();
					designator.Turret.EnableIdleRotation = false;

					designator.NextResetClock = clock + settings.DesignatorResetInterval;
				}
			}
		}
		else
		{
			break;
		}
	}

	if (settings.UseRangeSweeper)
	{
		if (clock <= designatorTargetAcquiredClock + settings.TargetFoundHoldTicks)
		{
			designatorOperationRR.Begin();

			maxCount = (settings.RangeSweeperPerTick == 0 ? designators.Count : settings.RangeSweeperPerTick);
			for (int i = 0; i < maxCount; i++)
			{
				Designator designator = designatorOperationRR.GetNext();
				if (designator != null)
				{
					if (clock >= designator.NextSweeperClock)
					{
						designator.SetMaxRange();

						designator.NextSweeperClock = clock + settings.RangeSweeperInterval;
					}
				}
				else
				{
					break;
				}
			}
		}
	}
}

void UpdateRaycastTargets()
{
	PDCTarget target = targetManager.GetOldestRaycastUpdatedTarget();
	if (target != null)
	{
		if (clock - target.RaycastRefreshClock >= settings.RaycastTargetRefreshTicks)
		{
			if (raycastHandler.Cameras.Count > 0)
			{
				double maxAllowedRaycastDistance = (target.MaxAllowedRaycastDistance == 0 ? settings.MaxRaycastTrackingDistance : target.MaxAllowedRaycastDistance);
				
				if ((target.Position - Me.WorldMatrix.Translation).LengthSquared() <= maxAllowedRaycastDistance * maxAllowedRaycastDistance)
				{
					Vector3D targetPosition = target.Position + (target.Velocity * (clock - target.DetectedClock) * INV_ONE_TICK);

					MyDetectedEntityInfo entityInfo;
					if (raycastHandler.Raycast(ref targetPosition, out entityInfo, settings.RaycastExtensionDistance))
					{
						HandleRaycastParameters(ref entityInfo);
					}

					nextRaycastGlobalClock = clock + settings.RaycastGlobalRefreshTicks;
				}
			}

			targetManager.UpdateRaycastRefreshClock(target.EntityId, clock);

			HandlePriorityAssigment(target);
		}
	}

	if (settings.UsePDCSpray && settings.RandomOffsetProbeInterval > 0 && clock % settings.RandomOffsetProbeInterval == 0)
	{
		PDCTarget largestTarget = targetManager.FindLargestTarget();
		if (largestTarget != null && largestTarget.Orientation != null && largestTarget.TargetSizeSq >= settings.PDCSprayMinTargetSize * settings.PDCSprayMinTargetSize)
		{
			double maxAllowedRaycastDistance = (largestTarget.MaxAllowedRaycastDistance == 0 ? settings.MaxRaycastTrackingDistance : largestTarget.MaxAllowedRaycastDistance);
			Vector3D largestPosition = largestTarget.CenterPosition + (largestTarget.Velocity * (clock - largestTarget.DetectedClock) * INV_ONE_TICK);

			if ((largestPosition - Me.WorldMatrix.Translation).LengthSquared() <= maxAllowedRaycastDistance * maxAllowedRaycastDistance)
			{
				double probeRadius = Math.Sqrt(largestTarget.TargetSizeSq) * 0.5 * OFFSET_PROBE_RANDOM_FACTOR;

				Vector3D partialOffsetVector = new Vector3D(((rnd.NextDouble() * 2) - 1) * probeRadius, ((rnd.NextDouble() * 2) - 1) * probeRadius, ((rnd.NextDouble() * 2) - 1) * probeRadius);
				Vector3D targetPosition = largestPosition + Vector3D.TransformNormal(partialOffsetVector, largestTarget.Orientation.Value);

				MyDetectedEntityInfo entityInfo;
				if (raycastHandler.Raycast(ref targetPosition, out entityInfo, settings.RaycastExtensionDistance))
				{
					HandleRaycastParameters(ref entityInfo);
				}

				nextRaycastGlobalClock = clock + settings.RaycastGlobalRefreshTicks;
			}
		}
	}
}

void HandleRaycastParameters(ref MyDetectedEntityInfo entityInfo)
{
	if (entityInfo.IsEmpty())
	{
		return;
	}

	if (IsValidTarget(ref entityInfo))
	{

		PDCTarget target;
		targetManager.UpdateTarget(ref entityInfo, clock, out target);

		if (target == null)
		{
			return;
		}

		target.TargetSizeSq = entityInfo.BoundingBox.Extents.LengthSquared();

		if (target.CheckTargetSizeSq == 0)
		{
			target.CheckTargetSizeSq = target.TargetSizeSq;
		}

		if (settings.UsePDCSpray && target.TargetSizeSq >= settings.PDCSprayMinTargetSize * settings.PDCSprayMinTargetSize && entityInfo.HitPosition.HasValue)
		{
			if (target.OffsetPoints == null)
			{
				target.OffsetPoints = new List<OffsetPoint>(OFFSET_POINTS_MAX_COUNT);
			}

			Vector3D offsetPoint = Vector3D.TransformNormal(entityInfo.HitPosition.Value - entityInfo.Position, MatrixD.Transpose(entityInfo.Orientation));
								
			if (target.OffsetPoints.Count >= OFFSET_POINTS_MAX_COUNT)
			{
				int selectedIndex = 0;
				double selectedDistanceSq = double.MaxValue;
				for (int i = 0; i < target.OffsetPoints.Count; i++)
				{
					if (clock > target.OffsetPoints[i].LastUpdatedClock + OFFSET_POINTS_EXPIRE_TICKS)
					{
						selectedIndex = i;
						selectedDistanceSq = 0;
						break;
					}
										
					double distance = (target.OffsetPoints[i].Point - offsetPoint).LengthSquared();
					if (distance < selectedDistanceSq)
					{
						selectedIndex = i;
						selectedDistanceSq = distance;
					}
				}
				if (selectedDistanceSq < double.MaxValue)
				{
					target.OffsetPoints[selectedIndex] = new OffsetPoint(ref offsetPoint, clock);
				}
			}
			else
			{
				target.OffsetPoints.Add(new OffsetPoint(ref offsetPoint, clock));
			}
		}
	}
	else
	{
		targetManager.AddToBlackList(entityInfo.EntityId);
		targetManager.RemoveTarget(entityInfo.EntityId);
	}
}

void HandlePriorityAssigment(PDCTarget target)
{
	Vector3D shipPosition = Me.GetPosition();
	Vector3D shipVelocity = (remote != null ? remote.GetShipVelocities().LinearVelocity : Vector3D.Zero);

	if (target.TargetSizeSq > 0 && target.TargetSizeSq < settings.MinTargetSizeEngage * settings.MinTargetSizeEngage)
	{
		targetManager.AddToBlackList(target.EntityId);
		targetManager.RemoveTarget(target.EntityId);
	}
	else if (clock - target.DetectedClock <= settings.TargetLostTicks)
	{
		double priorityValue = ComputePriority(shipRadius, shipPosition, shipVelocity, target);
		priorityValue += rnd.NextDouble() * 0.000000000001;

		target.PriorityValue = priorityValue;

		maxPriorityValue = Math.Max(priorityValue, maxPriorityValue);

		if (remote == null || remote.GetShipVelocities().LinearVelocity.LengthSquared() <= settings.MissileLaunchSpeedLimit * settings.MissileLaunchSpeedLimit)
		{
			if (target.TargetSizeSq >= settings.MissileMinTargetSize * settings.MissileMinTargetSize)
			{
				if (target.MissileLaunchLastClock == 0 || clock >= target.MissileLaunchLastClock + settings.MissileReassignIntervalTicks)
				{
					target.MissileLaunchLastClock = clock;

					double maxAllowedMissileLaunchDistance = (target.MaxAllowedMissileLaunchDistance == 0 ? settings.MaxMissileLaunchDistance : target.MaxAllowedMissileLaunchDistance);

					if ((target.Position - Me.WorldMatrix.Translation).LengthSquared() <= maxAllowedMissileLaunchDistance * maxAllowedMissileLaunchDistance)
					{
						if (target.TargetSizeSq == 0)
						{
							target.MissileRemainingCount = 1;
						}
						else
						{
							target.MissileRemainingCount = (int)Math.Ceiling(Math.Sqrt(target.TargetSizeSq) / Math.Max(settings.MissileCountPerSize, 1));
						}
					}
				}
			}
		}
	}
	else
	{
		targetManager.RemoveTarget(target.EntityId);
	}
}

void UpdateAllies()
{
	AllyTrack ally = allyManager.GetOldestUpdatedAlly();
	if (ally != null)
	{
		if (clock - ally.LastDetectedClock > settings.AllyTrackLostTicks)
		{
			allyManager.RemoveAlly(ally.EntityId);
		}
	}
}

void AssignTargets()
{
	switch (assignmentState)
	{
		case 0:
			if (clock >= nextPriorityRefreshClock)
			{
				if (targetManager.Count() > 0)
				{
					nextPriorityRefreshClock = clock + settings.PriorityMaxRefreshTicks;

					sortedEntityIds.Clear();
					maxPriorityValue = 0;

					List<PDCTarget> pdcTargets = targetManager.GetAllTargets();
					foreach (PDCTarget target in pdcTargets)
					{
						if (!sortedEntityIds.ContainsKey(target.PriorityValue))
						{
							sortedEntityIds.Add(target.PriorityValue, target);

							maxPriorityValue = Math.Max(target.PriorityValue, maxPriorityValue);
						}
					}

					pdcAssignRR.Reset();
					pdcAssignRR.Begin();

					assignmentState = 1;
				}
			}
			break;
		case 1:
			PDCTurret pdc = pdcAssignRR.GetNext();
			if (pdc != null)
			{
				if (pdc != null && !pdc.IsDamaged)
				{
					double selectedPriority = 0;
					PDCTarget selectedTarget = null;

					double preferredPriority = 0;
					PDCTarget preferredTarget = null;

					foreach (KeyValuePair<double, PDCTarget> keyTarget in sortedEntityIds)
					{
						if (pdc.IsTargetable(keyTarget.Value, clock))
						{
							selectedPriority = keyTarget.Key;
							selectedTarget = keyTarget.Value;

							if (pdc.TurretPrioritySize > 0)
							{
								if (selectedTarget.TargetSizeSq >= pdc.TurretPrioritySize * pdc.TurretPrioritySize)
								{
									preferredPriority = selectedPriority;
									preferredTarget = selectedTarget;

									break;
								}
							}
							else
							{
								break;
							}
						}
					}

					if (preferredTarget != null)
					{
						selectedPriority = preferredPriority;
						selectedTarget = preferredTarget;
					}

					if (selectedTarget != null)
					{
						pdc.TargetInfo = selectedTarget;

						if (sortedEntityIds.ContainsKey(selectedPriority))
						{
							sortedEntityIds.Remove(selectedPriority);

							maxPriorityValue += 1000;
							selectedPriority = maxPriorityValue;

							sortedEntityIds.Add(selectedPriority, selectedTarget);
						}
					}
					else
					{
						if (pdc.TargetInfo != null && !targetManager.TargetExists(pdc.TargetInfo.EntityId))
						{
							pdc.TargetInfo = null;
						}
						pdc.ReleaseWeapons();
						pdc.ResetRotors();
					}
				}
			}
			else
			{
				assignmentState = 0;
			}
			break;
	}
}

double ComputePriority(double shipRadius, Vector3D shipPosition, Vector3D shipVelocity, PDCTarget target)
{
	Vector3D rangeVector = target.Position - shipPosition;
	double priorityValue = rangeVector.Length();

	PlaneD plane;
	if (shipVelocity.LengthSquared() < 0.01)
	{
		plane = new PlaneD(shipPosition, Vector3D.Normalize(rangeVector));
	}
	else
	{
		plane = new PlaneD(shipPosition, shipPosition + shipVelocity, shipPosition + rangeVector.Cross(shipVelocity));
	}
	Vector3D intersectPoint = plane.Intersection(ref target.Position, ref target.Velocity);
	Vector3D targetTravelVector = intersectPoint - target.Position;

	if (targetTravelVector.Dot(ref target.Velocity) < 0)
	{
		priorityValue += settings.PriorityDowngradeConstant * 4;
	}
	else
	{
		double t = Math.Sqrt(targetTravelVector.LengthSquared() / Math.Max(target.Velocity.LengthSquared(), 0.000000000000001));
		if ((intersectPoint - (shipPosition + (shipVelocity * t))).LengthSquared() > shipRadius * shipRadius)
		{
			priorityValue += settings.PriorityDowngradeConstant * 2;
		}
		else if (target.TargetSizeSq <= 0)
		{
			priorityValue += settings.PriorityDowngradeConstant;
		}
		else if (target.TargetSizeSq < settings.MinTargetSizePriority * settings.MinTargetSizePriority)
		{
			priorityValue += settings.PriorityDowngradeConstant * 3;
		}
	}

	if (target.CheckTargetSizeSq > target.TargetSizeSq)
	{
		priorityValue += settings.PriorityDowngradeConstant * Math.Max(target.PDCTargetedCount, 1);
	}
	else
	{
		priorityValue += settings.PriorityDowngradeConstant * Math.Min(target.PDCTargetedCount, 1);
	}

	return priorityValue;
}

void AimFireReloadPDC()
{
	if (curPDCUpdatesSkipTicks > 0)
	{
		if (clock >= curPDCNextUpdateClock)
		{
			curPDCNextUpdateClock = clock + curPDCUpdatesSkipTicks;
		}
		else
		{
			return;
		}
	}
	
	pdcFireRR.Begin();

	int maxCount = (curPDCUpdatesPerTick == 0 ? pdcList.Count : curPDCUpdatesPerTick);
	for (int i = 0; i < maxCount; i++)
	{
		PDCTurret pdc = pdcFireRR.GetNext();
		if (pdc != null)
		{
			if (clock >= pdc.LastReloadCheckClock + settings.ReloadCheckTicks)
			{
				pdc.LastReloadCheckClock = clock;

				if (pdc.CheckReloadRequired())
				{
					if (clock >= pdc.LastReloadOperationCheck + settings.ReloadedCooldownTicks)
					{
						pdc.LastReloadOperationCheck = clock;

						pdc.PerformReloadProcedure(clock);
					}
				}
			}

			if (pdc.CurrentReloadState == PDCTurret.ReloadState.NONE)
			{
				if (pdc.TargetInfo != null)
				{
					pdc.AimAndFire(pdc.TargetInfo, clock);
				}
			}
			else
			{
				pdc.PerformReloadProcedure(clock);
			}
		}
		else
		{
			break;
		}
	}
}

void LaunchAutomaticMissiles()
{
	if (targetManager.Count() > 0)
	{
		PDCTarget target = targetManager.GetOldestRaycastUpdatedTarget();
		if (target != null)
		{
			if (target.MissileRemainingCount > 0 && targetManager.TargetExists(target.EntityId))
			{
				target.MissileRemainingCount--;

				bool useOffsetTargeting = (target.MissileRemainingCount <= 0 ? true : (rnd.NextDouble() <= settings.MissileOffsetProbability));
				LaunchMissileForTarget(target, useOffsetTargeting);

				nextAutoMissileLaunchClock = clock + settings.MissileStaggerWaitTicks;
			}
		}
	}
}

void LaunchMissileForTarget(PDCTarget target, bool useOffsetTargeting = false, bool fakeSimulation = false)
{
	IMyProgrammableBlock missileComputer = null;
	double closest = double.MaxValue;

	foreach (IMyProgrammableBlock block in missileComputers)
	{
		if (block.IsWorking && GridTerminalSystem.GetBlockWithId(block.EntityId) != null)
		{
			double distanceSq = (block.WorldMatrix.Translation - target.Position).LengthSquared();
			if (distanceSq < closest)
			{
				closest = distanceSq;
				missileComputer = block;
			}
		}
	}

	if (missileComputer != null && !fakeSimulation)
	{
		MissileCommsTarget commsTarget = new MissileCommsTarget();
		commsTarget.TransmitUniqueId = GenerateUniqueId();
		commsTarget.Target = target;
		commsTarget.TransmitUntilClock = clock + settings.MissileTransmitDurationTicks;
		commsTarget.UseOffsetTargeting = useOffsetTargeting;
		guidanceCommsTargets.Enqueue(commsTarget);

		MissileCommsTarget switchLostTarget = new MissileCommsTarget();
		switchLostTarget.Target = target;
		switchLostTarget.TransmitUntilClock = int.MaxValue;
		switchLostTarget.TransmitAsSwitchLost = true;
		guidanceCommsTargets.Enqueue(switchLostTarget);

		//[Notes] TargetDatalinkData(LaunchControlId, SetUniqueId, SetGroupId, SetBitMaskOptions(1=OffsetTargeting), SetRandomOffsetAmount) => MyTuple<long, long, long, int, float>
		MyTuple<long, long, long, int, float> launchData = new MyTuple<long, long, long, int, float>();

		launchData.Item1 = Me.EntityId;
		launchData.Item2 = commsTarget.TransmitUniqueId;
		launchData.Item3 = Me.EntityId;
		launchData.Item4 |= (int)AGMLaunchOptionEnum.OffsetTargeting;
		launchData.Item5 = (useOffsetTargeting ? (float)(Math.Sqrt(target.TargetSizeSq) * 0.5 * settings.MissileOffsetRadiusFactor) : 0f);

		bool sentIGCLaunchSuccess = IGC.SendUnicastMessage(missileComputer.EntityId, "", launchData);

		if (!sentIGCLaunchSuccess)
		{
			MyIni config = new MyIni();
			if (missileComputer.CustomData.Length > 0) config.TryParse(missileComputer.CustomData);
			config.Set(MISSILE_INI_SECTION, "UniqueId", launchData.Item2);
			config.Set(MISSILE_INI_SECTION, "GroupId", launchData.Item3);
			if (useOffsetTargeting)
			{
				config.Set(MISSILE_INI_SECTION, "OffsetTargeting", launchData.Item4);
				config.Set(MISSILE_INI_SECTION, "RandomOffsetAmount", launchData.Item5);
			}
			missileComputer.CustomData = config.ToString();
			
			missileComputer.TryRun("FIRE:" + Me.EntityId);
		}

		missileComputers.Remove(missileComputer);
	}
}

void LaunchManualForTarget(List<IMyProgrammableBlock> guidanceComputers, PDCTarget target, TargetingPointTypeEnum targetingPointType = TargetingPointTypeEnum.Center, Vector3D? offsetPoint = null, bool checkUseCGE = false, bool fakeSimulation = false)
{
	for (int i = 0; i < guidanceComputers.Count; i++)
	{
		IMyProgrammableBlock computer = guidanceComputers[i];

		if (computer.IsWorking && GridTerminalSystem.GetBlockWithId(computer.EntityId) != null && !fakeSimulation)
		{
			MissileCommsTarget commsTarget = new MissileCommsTarget();
			commsTarget.TransmitUniqueId = GenerateUniqueId();
			commsTarget.Target = target;
			commsTarget.TransmitUntilClock = clock + settings.ManualAimBroadcastDurationTicks;
			if (checkUseCGE && NameContains(computer, USE_CGE_TAG)) commsTarget.CacheMissilePB = computer;
			guidanceCommsTargets.Enqueue(commsTarget);

			MyIni config = null;

			//[Notes] TargetDatalinkData(LaunchControlId, SetUniqueId, SetGroupId, SetBitMaskOptions(1=OffsetTargeting), SetRandomOffsetAmount) => MyTuple<long, long, long, int, float>
			MyTuple<long, long, long, int, float> launchData = new MyTuple<long, long, long, int, float>();

			launchData.Item1 = Me.EntityId;
			launchData.Item2 = commsTarget.TransmitUniqueId;
			launchData.Item3 = Me.EntityId;

			switch (targetingPointType)
			{
			case TargetingPointTypeEnum.Offset:
				launchData.Item4 |= (int)AGMLaunchOptionEnum.OffsetTargeting;

				config = new MyIni();
				if (computer.CustomData.Length > 0) config.TryParse(computer.CustomData);
				config.Set(MISSILE_INI_SECTION, "ProbeOffsetVector", Vector3ToBase64(offsetPoint != null ? offsetPoint.Value : Vector3D.Zero));
				computer.CustomData = config.ToString();

				break;
			case TargetingPointTypeEnum.Random:
				launchData.Item4 |= (int)AGMLaunchOptionEnum.OffsetTargeting;
				launchData.Item5 = (float)(Math.Sqrt(target.TargetSizeSq) * 0.5 * settings.MissileOffsetRadiusFactor);

				break;
			default:
				launchData.Item5 = 0f;

				break;
			}

			bool sentIGCLaunchSuccess = IGC.SendUnicastMessage(computer.EntityId, "", launchData);

			if (!sentIGCLaunchSuccess)
			{
				//IGC Unicast Failed. Script may be older version
				if (config == null)
				{
					config = new MyIni();
					if (computer.CustomData.Length > 0) config.TryParse(computer.CustomData);
				}
				config.Set(MISSILE_INI_SECTION, "UniqueId", launchData.Item2);
				config.Set(MISSILE_INI_SECTION, "GroupId", launchData.Item3);
				switch (targetingPointType)
				{
				case TargetingPointTypeEnum.Offset:
					config.Set(MISSILE_INI_SECTION, "OffsetTargeting", launchData.Item4);
					config.Set(MISSILE_INI_SECTION, "ProbeOffsetVector", Vector3ToBase64(offsetPoint != null ? offsetPoint.Value : Vector3D.Zero));

					break;
				case TargetingPointTypeEnum.Random:
					config.Set(MISSILE_INI_SECTION, "OffsetTargeting", launchData.Item4);
					config.Set(MISSILE_INI_SECTION, "RandomOffsetAmount", launchData.Item5);

					break;
				}
				computer.CustomData = config.ToString();

				computer.TryRun("FIRE:" + Me.EntityId);
			}

			if (guidanceComputers.Count == 1)
			{
				guidanceComputers.Clear();
			}
			else
			{
				guidanceComputers[i] = guidanceComputers[guidanceComputers.Count - 1];
				guidanceComputers.RemoveAt(guidanceComputers.Count - 1);
			}

			break;
		}
	}
}

void TransmitIGCMessages()
{
	haveIGCTransmitted = false;

	if (curCommsQueuePriority == 0)
	{
		TransmitMissileInformation();
		if (!haveIGCTransmitted)
		{
			TransmitTargetTracksInformation();
		}

		curCommsQueuePriority = 1;
	}
	else
	{
		TransmitTargetTracksInformation();
		if (!haveIGCTransmitted)
		{
			TransmitMissileInformation();
		}

		curCommsQueuePriority = 0;
	}
}

void TransmitMissileInformation()
{
	while (guidanceCommsTargets.Count > 0)
	{
		MissileCommsTarget current = guidanceCommsTargets.Dequeue();

		if (clock <= current.TransmitUntilClock)
		{
			if (!(current.Target.EntityId == -1 || targetManager.TargetExists(current.Target.EntityId)))
			{
				break;
			}

			if (clock >= current.NextTransmitClock)
			{
				if (current.CacheMissilePB == null)
				{
					//[Notes] TargetDatalinkData(IsGroupRecipient, RecipientId, SenderId, TargetEntityId, TargetPosition, TargetVelocity) => MyTuple<bool, long, long, long, Vector3D, Vector3D>
					MyTuple<bool, long, long, long, Vector3D, Vector3D> targetDatalinkData = new MyTuple<bool, long, long, long, Vector3D, Vector3D>();
					targetDatalinkData.Item1 = current.TransmitAsSwitchLost;
					targetDatalinkData.Item2 = (current.TransmitAsSwitchLost ? Me.EntityId : current.TransmitUniqueId);
					targetDatalinkData.Item3 = 0;
					targetDatalinkData.Item4 = current.Target.EntityId;
					targetDatalinkData.Item5 = current.Target.Position + (current.Target.Velocity * (clock - current.Target.DetectedClock + 1) * INV_ONE_TICK);
					targetDatalinkData.Item6 = current.Target.Velocity;

					IGC.SendBroadcastMessage(current.TransmitAsSwitchLost ? IGC_MSG_TARGET_SWITCH_LOST : IGC_MSG_TARGET_DATALINK, targetDatalinkData);
				}
				else
				{
					Vector3D targetPosition = current.Target.Position + (current.Target.Velocity * (clock - current.Target.DetectedClock + 1) * INV_ONE_TICK);
					MyIni cgeDatalink = new MyIni();
					cgeDatalink.Set(CGE_MSG_TARGET_DATALINK, "EntityId", current.Target.EntityId);
					cgeDatalink.Set(CGE_MSG_TARGET_DATALINK, "PositionX", targetPosition.X);
					cgeDatalink.Set(CGE_MSG_TARGET_DATALINK, "PositionY", targetPosition.Y);
					cgeDatalink.Set(CGE_MSG_TARGET_DATALINK, "PositionZ", targetPosition.Z);
					cgeDatalink.Set(CGE_MSG_TARGET_DATALINK, "VelocityX", current.Target.Velocity.X);
					cgeDatalink.Set(CGE_MSG_TARGET_DATALINK, "VelocityY", current.Target.Velocity.Y);
					cgeDatalink.Set(CGE_MSG_TARGET_DATALINK, "VelocityZ", current.Target.Velocity.Z);

					current.CacheMissilePB.TryRun(CGE_MSG_TARGET_DATALINK + cgeDatalink.ToString());
				}

				current.NextTransmitClock = clock + settings.MissileTransmitIntervalTicks;
				haveIGCTransmitted = true;
			}

			if (!current.TransmitAsSwitchLost)
			{
				guidanceCommsTargets.Enqueue(current);
			}

			break;
		}
	}
}

void TransmitTargetTracksInformation()
{
	if (clock >= nextTracksTranmissionClock && tracksCommsTargets.Count == 0)
	{
		if (targetManager.Count() > 0)
		{
			List<PDCTarget> pdcTargets = targetManager.GetAllTargets();
			pdcTargets.Sort(sortCommsTargetPriority);

			foreach (PDCTarget target in pdcTargets)
			{
				tracksCommsTargets.Enqueue(target);
			}
		}

		nextTracksTranmissionClock = clock + settings.TargetTracksTransmitIntervalTicks;
	}

	while (tracksCommsTargets.Count > 0)
	{
		PDCTarget target = tracksCommsTargets.Dequeue();

		if (targetManager.TargetUpToLocalDate(target.EntityId, clock - settings.TargetLostTicks))
		{
			//[Notes] TargetTracksData(TargetEntityId, TargetPosition, TargetVelocity, TargetSizeSq, Flags[IsFriendly,IsLargeGrid], Timestamp) => MyTuple<long, Vector3D, Vector3D, double, int, long>
			MyTuple<long, Vector3D, Vector3D, double, int, long> tracksInfoData = new MyTuple<long, Vector3D, Vector3D, double, int, long>();
			tracksInfoData.Item1 = target.EntityId;
			tracksInfoData.Item2 = target.Position + (target.Velocity * (clock - target.DetectedClock + 1) * INV_ONE_TICK);
			tracksInfoData.Item3 = target.Velocity;
			tracksInfoData.Item4 = target.TargetSizeSq;
			
			if (target.IsLargeGrid) tracksInfoData.Item5 |= (int)TrackTypeEnum.IsLargeGrid;

			tracksInfoData.Item6 = 0;

			IGC.SendBroadcastMessage(IGC_MSG_TRACKS_INFO, tracksInfoData);
			haveIGCTransmitted = true;

			break;
		}
	}
}

void TransmitMyShipInformation()
{
	if (clock >= nextMyShipTranmissionClock)
	{
		//[Notes] TargetTracksData(TargetEntityId, TargetPosition, TargetVelocity, TargetSizeSq, Flags[IsFriendly,IsLargeGrid], Timestamp) => MyTuple<long, Vector3D, Vector3D, double, int, long>
		MyTuple<long, Vector3D, Vector3D, double, int, long> tracksInfoData = new MyTuple<long, Vector3D, Vector3D, double, int, long>();
		tracksInfoData.Item1 = Me.CubeGrid.EntityId;
		tracksInfoData.Item2 = Me.CubeGrid.WorldAABB.Center;
		tracksInfoData.Item3 = (remote != null ? remote.GetShipVelocities().LinearVelocity : Vector3D.Zero);
		tracksInfoData.Item4 = shipRadius * shipRadius * 4;
		
		tracksInfoData.Item5 |= (int)TrackTypeEnum.IsFriendly;
		if (Me.CubeGrid.GridSizeEnum == MyCubeSize.Large) tracksInfoData.Item5 |= (int)TrackTypeEnum.IsLargeGrid;

		tracksInfoData.Item6 = 0;

		IGC.SendBroadcastMessage(IGC_MSG_TRACKS_INFO, tracksInfoData);
		
		nextMyShipTranmissionClock = clock + settings.TargetTracksTransmitIntervalTicks;
	}
}

void RefreshDisplays()
{
	foreach (ManualPDCTarget manualTarget in manualTargeters)
	{
		if (manualTarget.DisplayStatus != null)
		{
			IMyTextSurface surface = manualTarget.DisplayStatus;
			
			//Always ensure square drawing panel, and there are 1024 logical pixels on each sizes
			Vector2 screenOffset;
			Vector2 stepSize;
			if (surface.SurfaceSize.X == surface.SurfaceSize.Y)
			{
				screenOffset = (surface.TextureSize - surface.SurfaceSize) * 0.5f;
				stepSize = new Vector2(surface.SurfaceSize.X) * 0.0009765625f;
			}
			else if (surface.SurfaceSize.X > surface.SurfaceSize.Y)
			{
				screenOffset = (surface.TextureSize - surface.SurfaceSize + new Vector2(surface.SurfaceSize.X - surface.SurfaceSize.Y, 0f)) * 0.5f;
				stepSize = new Vector2(surface.SurfaceSize.Y) * 0.0009765625f;
			}
			else
			{
				screenOffset = (surface.TextureSize - surface.SurfaceSize + new Vector2(0f, surface.SurfaceSize.Y - surface.SurfaceSize.X)) * 0.5f;
				stepSize = new Vector2(surface.SurfaceSize.Y) * 0.0009765625f;
			}

			List<MySprite> sprites = new List<MySprite>();

			sprites.Add(new MySprite(SpriteType.TEXTURE, "SquareSimple", surface.TextureSize * 0.5f, null, Color.Black));
			
			Vector2 fullBarSize = stepSize * new Vector2(1024f, 100f);
			Vector2 fullBarMidWidth = new Vector2(fullBarSize.X * 0.5f, 0f);
			Vector2 fullBarMidPosition = fullBarSize * 0.5f;
			Vector2 verticalSpacing = new Vector2(0f, fullBarSize.Y);
			Vector2 verticalGap = new Vector2(0f, stepSize.Y * 20f);
			Vector2 halfBarSize = stepSize * new Vector2(480f, 100f);
			Vector2 halfBarMidWidth = new Vector2(halfBarSize.X * 0.5f, 0f);
			Vector2 halfBarMidPosition = halfBarSize * 0.5f;
			Vector2 horizontalSpacing = stepSize * new Vector2(544f, 0f);

			float textSize = standardTextSizeFactor * fullBarSize.Y;

			Vector2 currentOffset = screenOffset + verticalGap;

			string text;
			Color textColor;
			Color boxColor;

			sprites.Add(new MySprite(SpriteType.TEXTURE, "SquareSimple", currentOffset + fullBarMidPosition, fullBarSize, statusThisScriptBoxColor));
			sprites.Add(new MySprite(SpriteType.TEXT, $"{DISPLAY_SCRIPT_NAME} v{DISPLAY_VERSION}", currentOffset + fullBarMidWidth, null, statusThisScriptTextColor, "DEBUG", TextAlignment.CENTER, textSize));

			currentOffset += verticalSpacing;

			sprites.Add(new MySprite(SpriteType.TEXTURE, "SquareSimple", currentOffset + fullBarMidPosition, fullBarSize, statusAimIdBoxColor));
			sprites.Add(new MySprite(SpriteType.TEXT, $"Manual Targeter {manualTarget.CodeId}", currentOffset + fullBarMidWidth, null, statusAimIdTextColor, "DEBUG", TextAlignment.CENTER, textSize));

			currentOffset += verticalSpacing + verticalGap;

			sprites.Add(new MySprite(SpriteType.TEXT, "Status", currentOffset + fullBarMidWidth, null, statusTitleTextColor, "DEBUG", TextAlignment.CENTER, textSize));

			currentOffset += verticalSpacing;

			if (manualTarget.SelectedEntityId > 0) { text = "Target Locked"; textColor = statusLockedTextColor; boxColor = statusLockedBoxColor; }
			else if (manualTarget.Enabled) { text = "Seeking"; textColor = statusSeekingTextColor; boxColor = statusSeekingBoxColor; }
			else { text = "No Target"; textColor = statusNoTargetTextColor; boxColor = statusNoTargetBoxColor; }
			sprites.Add(new MySprite(SpriteType.TEXTURE, "SquareSimple", currentOffset + fullBarMidPosition, fullBarSize, boxColor));
			sprites.Add(new MySprite(SpriteType.TEXT, text, currentOffset + fullBarMidWidth, null, textColor, "DEBUG", TextAlignment.CENTER, textSize));

			currentOffset += verticalSpacing + verticalGap;

			sprites.Add(new MySprite(SpriteType.TEXT, "Current Target", currentOffset + fullBarMidWidth, null, statusTitleTextColor, "DEBUG", TextAlignment.CENTER, textSize));

			currentOffset += verticalSpacing;

			if (manualTarget.SelectedEntityId > 0) { text = GetTargetCode(manualTarget.SelectedEntityId); textColor = statusCurrentTargetTextColor; boxColor = statusCurrentTargetBoxColor; }
			else { text = "-"; textColor = statusNoTargetTextColor; boxColor = statusNoTargetBoxColor; }
			sprites.Add(new MySprite(SpriteType.TEXTURE, "SquareSimple", currentOffset + fullBarMidPosition, fullBarSize, boxColor));
			sprites.Add(new MySprite(SpriteType.TEXT, text, currentOffset + fullBarMidWidth, null, textColor, "DEBUG", TextAlignment.CENTER, textSize));

			currentOffset += verticalSpacing + verticalGap;

			sprites.Add(new MySprite(SpriteType.TEXT, "Range", currentOffset + halfBarMidWidth, null, statusTitleTextColor, "DEBUG", TextAlignment.CENTER, textSize));

			sprites.Add(new MySprite(SpriteType.TEXT, "Hit Point", currentOffset + horizontalSpacing + halfBarMidWidth, null, statusTitleTextColor, "DEBUG", TextAlignment.CENTER, textSize));

			currentOffset += verticalSpacing;

			text = $"{Math.Round(manualTarget.MaxManualRaycastDistance * 0.001, 1):n1}km";
			textColor = statusOptionsTextColor;
			boxColor = statusOptionsBoxColor;
			sprites.Add(new MySprite(SpriteType.TEXTURE, "SquareSimple", currentOffset + halfBarMidPosition, halfBarSize, boxColor));
			sprites.Add(new MySprite(SpriteType.TEXT, text, currentOffset + halfBarMidWidth, null, textColor, "DEBUG", TextAlignment.CENTER, textSize));

			text = manualTarget.GetTargetingPointTypeName();
			textColor = statusOptionsTextColor;
			boxColor = statusOptionsBoxColor;
			sprites.Add(new MySprite(SpriteType.TEXTURE, "SquareSimple", currentOffset + horizontalSpacing + halfBarMidPosition, halfBarSize, boxColor));
			sprites.Add(new MySprite(SpriteType.TEXT, text, currentOffset + horizontalSpacing + halfBarMidWidth, null, textColor, "DEBUG", TextAlignment.CENTER, textSize));

			surface.ContentType = ContentType.SCRIPT;
			surface.Script = "";

			MySpriteDrawFrame frame = surface.DrawFrame();
			frame.AddRange(sprites);

			frame.Dispose();
		}
	}
}

void DisplayStatus()
{
	sb.Clear();

	progressIconIndex = (progressIconIndex + 1) % 8;

	sb.AppendLine($"====[ Diamond Dome System ]==={progressIcons[progressIconIndex]}");
	sb.AppendLine($"<<Version {DISPLAY_VERSION}>>\n");

	sb.AppendLine($"PDCs : {pdcList.Count(b => { return !b.IsDamaged; })}");
	sb.AppendLine($"Designators : {designators.Count}");
	sb.AppendLine($"Raycast Cameras : {raycastCameras.Count}");
	sb.AppendLine($"Guided Missiles : {missileComputers.Count}");
	sb.AppendLine($"Guided Torpedos : {torpedoComputers.Count}");

	if (switchedOn)
	{
		sb.AppendLine($"Tracked Targets : {targetManager.Count()}");
	}
	else
	{
		sb.AppendLine("\n---< DD Disabled >---");
	}

	sb.AppendLine("\n---- Runtime Performance ---\n");
	profiler.PrintPerformance(sb);

	if (debugMode)
	{
		sb.AppendLine("\n>>>>>>> Debug Mode <<<<<<<");
		sb.AppendLine("\n---- Debug Performance ---\n");
		profiler.PrintSectionBreakdown(sb);
	}

	Echo(sb.ToString());
}

#region Helper Methods

long GenerateUniqueId()
{
	rnd.NextBytes(genUniqueIdBuffer);
	Buffer.BlockCopy(genUniqueIdBuffer, 0, genUniqueIdResult, 0, 8);
	return genUniqueIdResult[0];
}

string GetTargetCode(long entityId)
{
	return $"T{entityId % 100000:00000}";
}

string Vector3ToBase64(Vector3D vector)
{
	return Convert.ToBase64String(BitConverter.GetBytes((float)vector.X)) +
		Convert.ToBase64String(BitConverter.GetBytes((float)vector.Y)) +
		Convert.ToBase64String(BitConverter.GetBytes((float)vector.Z));
}

bool IsWorking(IMyTerminalBlock block)
{
	return (block != null && block.IsWorking);
}

bool IsValidTarget(ref MyDetectedEntityInfo entityInfo)
{
	if (entityInfo.Type == MyDetectedEntityType.LargeGrid || entityInfo.Type == MyDetectedEntityType.SmallGrid)
	{
		return IsHostileTarget(ref entityInfo);
	}
	return false;
}

bool IsHostileTarget(ref MyDetectedEntityInfo entityInfo)
{
	return !(entityInfo.Relationship == MyRelationsBetweenPlayerAndBlock.Owner || entityInfo.Relationship == MyRelationsBetweenPlayerAndBlock.FactionShare);
}

WeaponProfile GetWeaponProfile(IMyUserControllableGun weapon)
{
	if (weapon.BlockDefinition.SubtypeId.Contains("Gatling"))
	{
		return weaponProfiles.gatlingProfile;
	}
	else if (weapon.BlockDefinition.SubtypeId.Contains("Missile") || weapon.BlockDefinition.SubtypeId.Contains("Rocket"))
	{
		return weaponProfiles.rocketProfile;
	}
	return weaponProfiles.gatlingProfile;
}

bool FuncDesignatorHasTarget(Designator designator)
{
	return designator.Turret.HasTarget;
}

bool FuncDesignatorIsWorking(Designator designator)
{
	return designator.Turret.IsWorking;
}

bool FuncPDCIsWorking(PDCTurret pdc)
{
	return !pdc.IsDamaged;
}

bool NameContains(IMyTerminalBlock block, string tag)
{
	return (block.CustomName.IndexOf(tag, StringComparison.OrdinalIgnoreCase) > -1);
}
bool NameContains(IMyBlockGroup group, string tag)
{
	return (group.Name.IndexOf(tag, StringComparison.OrdinalIgnoreCase) > -1);
}

bool TokenContainsMatch(string[] tokens, string compare)
{
	foreach (string token in tokens)
	{
		if (token.Trim().Equals(compare, StringComparison.OrdinalIgnoreCase))
		{
			return true;
		}
	}
	return false;
}

#endregion

#region Helper Enums

public enum AGMLaunchOptionEnum
{
	OffsetTargeting = 1
}

public enum TargetingPointTypeEnum
{
	Center = 0,
	Offset = 1,
	Random = 2
}

public enum TrackTypeEnum
{
	IsFriendly = 1,
	IsLargeGrid = 2
}

#endregion

#region Helper Classes

public abstract class CompoundSprite
{
	public abstract bool CreateSprites(List<MySprite> spritesToBeAdded);
}

public static class AGMChecker
{
	const string INI_SAVED_BLOCKS_SECTION = "AGMSAVE";
	const string PARTIAL_TAG = "[NOTREADY]";

	public static bool CheckSave(IMyProgrammableBlock missilePB)
	{
		if (missilePB != null && missilePB.CustomName.IndexOf(PARTIAL_TAG, StringComparison.OrdinalIgnoreCase) > -1)
		{
			Vector3I[] vec = { -Base6Directions.GetIntVector(missilePB.Orientation.Left), Base6Directions.GetIntVector(missilePB.Orientation.Up), -Base6Directions.GetIntVector(missilePB.Orientation.Forward) };

			MyIni iniConfig = new MyIni();
			if (iniConfig.TryParse(missilePB.CustomData) && iniConfig.ContainsSection(INI_SAVED_BLOCKS_SECTION))
			{
				char[] delimiters = { ',' };

				if (!CheckConfigSavedBlockList("DetachBlock", missilePB, iniConfig, delimiters, ref vec, true)) return false;
				if (!CheckConfigSavedBlockList("DampenerBlock", missilePB, iniConfig, delimiters, ref vec, true)) return false;
				if (!CheckConfigSavedBlockList("ForwardBlock", missilePB, iniConfig, delimiters, ref vec, false)) return false;
				if (!CheckConfigSavedBlockList("RemoteControl", missilePB, iniConfig, delimiters, ref vec, false)) return false;
				if (!CheckConfigSavedBlockList("Gyroscopes", missilePB, iniConfig, delimiters, ref vec, false)) return false;
				if (!CheckConfigSavedBlockList("Thrusters", missilePB, iniConfig, delimiters, ref vec, false)) return false;
				if (!CheckConfigSavedBlockList("PowerBlocks", missilePB, iniConfig, delimiters, ref vec, true)) return false;
				if (!CheckConfigSavedBlockList("RaycastCameras", missilePB, iniConfig, delimiters, ref vec, true)) return false;
			}

			missilePB.CustomName = missilePB.CustomName.Replace(PARTIAL_TAG, "").Trim();
		}

		return true;
	}

	static bool CheckConfigSavedBlockList(string key, IMyTerminalBlock origin, MyIni iniConfig, char[] delimiters, ref Vector3I[] vec, bool acceptEmpty)
	{
		string[] values = iniConfig.Get(INI_SAVED_BLOCKS_SECTION, key).ToString().Split(delimiters, StringSplitOptions.RemoveEmptyEntries);
		return ((values.Length > 0 || acceptEmpty) && VerifyBlocksExist(values, origin, ref vec));
	}

	static bool VerifyBlocksExist(string[] input, IMyTerminalBlock origin, ref Vector3I[] vec)
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
}

public class RoundRobin<T>
{
	private List<T> items;
	private Func<T, bool> isReady;

	private int start;
	private int current;
	private bool available;

	public RoundRobin(List<T> dispatchItems, Func<T, bool> isReadyFunc = null)
	{
		items = dispatchItems;
		isReady = isReadyFunc;

		start = current = 0;
		available = false;

		if (items == null) items = new List<T>();
	}

	public void Reset()
	{
		start = current = 0;
	}

	public void Begin()
	{
		start = current;
		available = (items.Count > 0);
	}

	public T GetNext()
	{
		if (start >= items.Count) start = 0;
		if (current >= items.Count)
		{
			current = 0;
			available = (items.Count > 0);
		}

		T result = default(T);

		while (available)
		{
			T item = items[current++];

			if (current >= items.Count) current = 0;
			if (current == start) available = false;

			if (isReady == null || isReady(item))
			{
				result = item;
				break;
			}
		}

		return result;
	}

	public void ReloadList(List<T> dispatchItems)
	{
		items = dispatchItems;
		if (items == null) items = new List<T>();

		if (start >= items.Count) start = 0;
		if (current >= items.Count) current = 0;

		available = false;
	}
}

public class PDCTargetCommsSorting : IComparer<PDCTarget>
{
	public int Compare(PDCTarget x, PDCTarget y)
	{
		if (x == null) return (y == null ? 0 : 1);
		else if (y == null) return -1;
		else return (x.DetectedClock < y.DetectedClock ? -1 : (x.DetectedClock > y.DetectedClock ? 1 : (x.EntityId < y.EntityId ? -1 : (x.EntityId > y.EntityId ? 1 : 0))));
	}
}

public class TargetManager
{
	Dictionary<long, Wrapper> targetsLookup;
	SortedSet<Wrapper> targetsByRaycastRefresh;

	HashSet<long> targetsBlackList;

	public TargetManager()
	{
		targetsLookup = new Dictionary<long, Wrapper>();
		targetsByRaycastRefresh = new SortedSet<Wrapper>(new RaycastRefreshComparer());

		targetsBlackList = new HashSet<long>();
	}

	public bool UpdateTarget(ref MyDetectedEntityInfo entityInfo, int clock)
	{
		PDCTarget dummyTarget;
		return UpdateTarget(ref entityInfo, clock, out dummyTarget);
	}
	public bool UpdateTarget(ref MyDetectedEntityInfo entityInfo, int clock, out PDCTarget target)
	{
		if (targetsBlackList.Contains(entityInfo.EntityId))
		{
			target = null;
			return false;
		}

		if (targetsLookup.ContainsKey(entityInfo.EntityId))
		{
			Wrapper wrapper = targetsLookup[entityInfo.EntityId];
			target = wrapper.Target;

			int deltaTicks = Math.Max(clock - target.DetectedClock, 1);
			double deltaTimeR = 1.0 / deltaTicks;

			target.LastVelocity = target.Velocity;
			target.LastDetectedClock = target.DetectedClock;

			if (entityInfo.HitPosition != null)
			{
				target.Position = entityInfo.HitPosition.Value;

				target.CenterPosition = entityInfo.Position;
				target.RaycastDetectedClock = clock;

				target.Orientation = entityInfo.Orientation;
				target.OrientationUpdatedClock = clock;
			}
			else
			{
				target.Position = entityInfo.Position;
			}

			target.IsLargeGrid = (entityInfo.Type == MyDetectedEntityType.LargeGrid);

			target.Velocity = entityInfo.Velocity;
			target.DetectedClock = clock;

			Vector3D acceleration = (target.Velocity - target.LastVelocity) * deltaTimeR * TICKS_PER_SECOND;
			if (acceleration.LengthSquared() > 1)
			{
				target.Acceleration = (target.Acceleration * 0.25) + (acceleration * 0.75);
			}
			else
			{
				target.Acceleration = Vector3D.Zero;
			}

			wrapper.LastLocalDetectedClock = clock;

			return false;
		}
		else
		{
			target = new PDCTarget(entityInfo.EntityId);

			target.Position = entityInfo.Position;
			target.Velocity = target.LastVelocity = entityInfo.Velocity;
			target.DetectedClock = target.LastDetectedClock = clock;

			Wrapper wrapper = new Wrapper();
			wrapper.EntityId = target.EntityId;
			wrapper.Target = target;

			wrapper.LastLocalDetectedClock = clock;

			targetsLookup.Add(entityInfo.EntityId, wrapper);
			targetsByRaycastRefresh.Add(wrapper);

			return true;
		}
	}
	public bool UpdateTarget(TargetData targetData, int clock, bool isLocal = true)
	{
		if (targetsBlackList.Contains(targetData.EntityId))
		{
			return false;
		}

		if (targetsLookup.ContainsKey(targetData.EntityId))
		{
			Wrapper wrapper = targetsLookup[targetData.EntityId];
			PDCTarget target = wrapper.Target;

			int deltaTicks = Math.Max(clock - target.DetectedClock, 1);
			double deltaTimeR = 1.0 / deltaTicks;

			target.LastVelocity = target.Velocity;
			target.LastDetectedClock = target.DetectedClock;

			target.Position = targetData.Position;
			target.Velocity = targetData.Velocity;
			target.DetectedClock = clock;

			Vector3D acceleration = (target.Velocity - target.LastVelocity) * deltaTimeR * TICKS_PER_SECOND;
			if (acceleration.LengthSquared() > 1)
			{
				target.Acceleration = (target.Acceleration * 0.25) + (acceleration * 0.75);
			}
			else
			{
				target.Acceleration = Vector3D.Zero;
			}

			if (isLocal)
			{
				wrapper.LastLocalDetectedClock = clock;
			}

			return false;
		}
		else
		{
			PDCTarget target = new PDCTarget(targetData.EntityId);

			target.Position = targetData.Position;
			target.Velocity = target.LastVelocity = targetData.Velocity;
			target.DetectedClock = target.LastDetectedClock = clock;

			Wrapper wrapper = new Wrapper();
			wrapper.EntityId = target.EntityId;
			wrapper.Target = target;

			if (isLocal)
			{
				wrapper.LastLocalDetectedClock = clock;
			}

			targetsLookup.Add(targetData.EntityId, wrapper);
			targetsByRaycastRefresh.Add(wrapper);

			return true;
		}
	}

	public void UpdateRaycastRefreshClock(long entityId, int clock)
	{
		if (targetsLookup.ContainsKey(entityId))
		{
			Wrapper wrapper = targetsLookup[entityId];

			wrapper.Target.RaycastRefreshClock = clock;

			targetsByRaycastRefresh.Remove(wrapper);
			wrapper.RaycastRefreshClock = clock;

			targetsByRaycastRefresh.Add(wrapper);
		}
	}

	public bool TargetExists(long entityId)
	{
		return targetsLookup.ContainsKey(entityId);
	}

	public bool TargetUpToLocalDate(long entityId, int minimumLastClock)
	{
		if (targetsLookup.ContainsKey(entityId))
		{
			Wrapper wrapper = targetsLookup[entityId];
			return (wrapper.LastLocalDetectedClock >= minimumLastClock);
		}
		else
		{
			return false;
		}
	}

	public int Count()
	{
		return targetsLookup.Count;
	}

	public PDCTarget GetTarget(long entityId)
	{
		Wrapper wrapper;
		if (targetsLookup.TryGetValue(entityId, out wrapper)) return wrapper.Target;
		else return null;
	}

	public List<PDCTarget> GetAllTargets()
	{
		List<PDCTarget> targetList = new List<PDCTarget>(targetsLookup.Count);
		foreach (Wrapper wrapper in targetsByRaycastRefresh)
		{
			targetList.Add(wrapper.Target);
		}
		return targetList;
	}

	public PDCTarget GetOldestRaycastUpdatedTarget()
	{
		if (targetsByRaycastRefresh.Count == 0) return null;
		else return targetsByRaycastRefresh.Min.Target;
	}

	public PDCTarget FindLargestTarget()
	{
		double largestSizeSq = double.MinValue;
		PDCTarget largestTarget = null;

		foreach (Wrapper wrapper in targetsByRaycastRefresh)
		{
			if (wrapper.Target.TargetSizeSq > largestSizeSq)
			{
				largestSizeSq = wrapper.Target.TargetSizeSq;
				largestTarget = wrapper.Target;
			}
		}

		return largestTarget;
	}

	public void RemoveTarget(long entityId)
	{
		if (targetsLookup.ContainsKey(entityId))
		{
			targetsByRaycastRefresh.Remove(targetsLookup[entityId]);
			targetsLookup.Remove(entityId);
		}
	}

	public void ClearTargets()
	{
		targetsLookup.Clear();
		targetsByRaycastRefresh.Clear();
	}

	public void AddToBlackList(long entityId)
	{
		targetsBlackList.Add(entityId);
	}

	public void ClearBlackList()
	{
		targetsBlackList.Clear();
	}

	class Wrapper
	{
		public long EntityId;

		public int RaycastRefreshClock;
		public int LastLocalDetectedClock;

		public PDCTarget Target;
	}

	class RaycastRefreshComparer : IComparer<Wrapper>
	{
		public int Compare(Wrapper x, Wrapper y)
		{
			if (x == null) return (y == null ? 0 : 1);
			else if (y == null) return -1;
			else return (x.RaycastRefreshClock < y.RaycastRefreshClock ? -1 : (x.RaycastRefreshClock > y.RaycastRefreshClock ? 1 : (x.EntityId < y.EntityId ? -1 : (x.EntityId > y.EntityId ? 1 : 0))));
		}
	}
}

public class AllyManager
{
	Dictionary<long, Wrapper> alliesLookup;
	SortedSet<Wrapper> alliesByLastUpdate;

	public AllyManager()
	{
		alliesLookup = new Dictionary<long, Wrapper>();
		alliesByLastUpdate = new SortedSet<Wrapper>(new LastUpdateComparer());
	}

	public bool UpdateAlly(ref MyDetectedEntityInfo entityInfo, int clock)
	{
		if (alliesLookup.ContainsKey(entityInfo.EntityId))
		{
			Wrapper wrapper = alliesLookup[entityInfo.EntityId];

			wrapper.AllyTrackData.Position = entityInfo.Position;
			wrapper.AllyTrackData.Velocity = entityInfo.Velocity;
			wrapper.AllyTrackData.IsLargeGrid = (entityInfo.Type == MyDetectedEntityType.LargeGrid);
			wrapper.AllyTrackData.SizeSq = entityInfo.BoundingBox.Extents.LengthSquared();
			wrapper.AllyTrackData.LastDetectedClock = clock;

			alliesByLastUpdate.Remove(wrapper);
			
			wrapper.LastUpdatedClock = clock;

			alliesByLastUpdate.Add(wrapper);

			return false;
		}
		else
		{
			AllyTrack ally = new AllyTrack(entityInfo.EntityId);

			ally.Position = entityInfo.Position;
			ally.Velocity = entityInfo.Velocity;
			ally.LastDetectedClock = clock;

			Wrapper wrapper = new Wrapper();
			wrapper.EntityId = ally.EntityId;
			wrapper.AllyTrackData = ally;

			wrapper.LastUpdatedClock = clock;

			alliesLookup.Add(entityInfo.EntityId, wrapper);
			alliesByLastUpdate.Add(wrapper);

			return true;
		}
	}
	public bool UpdateAlly(AllyTrack allyTrack, int clock)
	{
		if (alliesLookup.ContainsKey(allyTrack.EntityId))
		{
			Wrapper wrapper = alliesLookup[allyTrack.EntityId];

			wrapper.AllyTrackData.Position = allyTrack.Position;
			wrapper.AllyTrackData.Velocity = allyTrack.Velocity;
			wrapper.AllyTrackData.LastDetectedClock = clock;

			alliesByLastUpdate.Remove(wrapper);
			
			wrapper.LastUpdatedClock = clock;

			alliesByLastUpdate.Add(wrapper);

			return false;
		}
		else
		{
			allyTrack.LastDetectedClock = clock;

			Wrapper wrapper = new Wrapper();
			wrapper.EntityId = allyTrack.EntityId;
			wrapper.AllyTrackData = allyTrack;

			wrapper.LastUpdatedClock = clock;

			alliesLookup.Add(allyTrack.EntityId, wrapper);
			alliesByLastUpdate.Add(wrapper);

			return true;
		}
	}

	public void UpdateLastUpdatedClock(long entityId, int clock)
	{
		if (alliesLookup.ContainsKey(entityId))
		{
			Wrapper wrapper = alliesLookup[entityId];

			alliesByLastUpdate.Remove(wrapper);
			
			wrapper.LastUpdatedClock = clock;

			alliesByLastUpdate.Add(wrapper);
		}
	}

	public bool AllyExists(long entityId)
	{
		return alliesLookup.ContainsKey(entityId);
	}

	public int Count()
	{
		return alliesLookup.Count;
	}

	public AllyTrack GetAlly(long entityId)
	{
		Wrapper wrapper;
		if (alliesLookup.TryGetValue(entityId, out wrapper)) return wrapper.AllyTrackData;
		else return null;
	}

	public List<AllyTrack> GetAllAllies()
	{
		List<AllyTrack> targetList = new List<AllyTrack>(alliesLookup.Count);
		foreach (Wrapper wrapper in alliesByLastUpdate)
		{
			targetList.Add(wrapper.AllyTrackData);
		}
		return targetList;
	}

	public AllyTrack GetOldestUpdatedAlly()
	{
		if (alliesByLastUpdate.Count == 0) return null;
		else return alliesByLastUpdate.Min.AllyTrackData;
	}

	public void RemoveAlly(long entityId)
	{
		if (alliesLookup.ContainsKey(entityId))
		{
			alliesByLastUpdate.Remove(alliesLookup[entityId]);
			alliesLookup.Remove(entityId);
		}
	}

	public void ClearAllies()
	{
		alliesLookup.Clear();
		alliesByLastUpdate.Clear();
	}

	class Wrapper
	{
		public long EntityId;
		public int LastUpdatedClock;

		public AllyTrack AllyTrackData;
	}

	class LastUpdateComparer : IComparer<Wrapper>
	{
		public int Compare(Wrapper x, Wrapper y)
		{
			if (x == null) return (y == null ? 0 : 1);
			else if (y == null) return -1;
			else return (x.LastUpdatedClock < y.LastUpdatedClock ? -1 : (x.LastUpdatedClock > y.LastUpdatedClock ? 1 : (x.EntityId < y.EntityId ? -1 : (x.EntityId > y.EntityId ? 1 : 0))));
		}
	}
}

public class ManualPDCTarget : PDCTarget
{
	public readonly string CodeId;

	public IMyTerminalBlock AimingBlock;
	public IMyLargeTurretBase AimingTurret;
	
	public IMyTextSurface DisplayStatus;
	public IMyTerminalBlock AlertBlock;

	public bool Enabled = false;
	public long SelectedEntityId = -1;

	public Vector3D? OffsetPoint;
	public TargetingPointTypeEnum TargetingPointType = TargetingPointTypeEnum.Center;

	public double MaxManualRaycastDistance;

	public ManualPDCTarget(string codeId, IMyTerminalBlock aimingBlock, IMyTextSurface displayStatus, IMyTerminalBlock alertBlock) : base(-1)
	{
		CodeId = codeId;

		AimingBlock = aimingBlock;
		AimingTurret = aimingBlock as IMyLargeTurretBase;

		DisplayStatus = displayStatus;
		AlertBlock = alertBlock;
	}

	public Vector3D GetForwardViewDirection()
	{
		if (AimingBlock == null)
		{
			return Vector3D.Zero;
		}

		if (AimingTurret != null)
		{
			Vector3D direction;
			Vector3D.CreateFromAzimuthAndElevation(AimingTurret.Azimuth, AimingTurret.Elevation, out direction);

			return Vector3D.TransformNormal(direction, AimingTurret.WorldMatrix);
		}
		else
		{
			return AimingBlock.WorldMatrix.Forward;
		}
	}

	public string GetTargetingPointTypeName()
	{
		switch (TargetingPointType)
		{
		case TargetingPointTypeEnum.Center: return "Center";
		case TargetingPointTypeEnum.Offset: return "Offset";
		case TargetingPointTypeEnum.Random: return "Random";
		}
		return "-";
	}
}
public class PDCTarget
{
	public readonly long EntityId;

	public Vector3D Position;
	public int DetectedClock;

	public Vector3D CenterPosition;
	public int RaycastDetectedClock;
	public int RaycastRefreshClock;

	public Vector3D Velocity;
	public Vector3D Acceleration;

	public Vector3D LastVelocity;
	public int LastDetectedClock;

	public MatrixD? Orientation;
	public int OrientationUpdatedClock;

	public bool IsLargeGrid;

	public double TargetSizeSq;
	public double PriorityValue;
	public double CheckTargetSizeSq;

	public double MaxAllowedRaycastDistance;
	public double MaxAllowedMissileLaunchDistance;

	public double PDCTargetedCount;

	public int MissileRemainingCount;
	public int MissileLaunchLastClock;

	public List<OffsetPoint> OffsetPoints;

	public PDCTarget(long entityId)
	{
		EntityId = entityId;
	}
}

public struct OffsetPoint
{
	public Vector3D Point;
	public int LastUpdatedClock;

	public OffsetPoint(ref Vector3D point, int currentClock)
	{
		Point = point;
		LastUpdatedClock = currentClock;
	}
}

public class Designator
{
	public IMyLargeTurretBase Turret;
	public float MaxDesignatorRange;
	public ITerminalProperty<float> RangeProperty;

	public int NextResetClock = 0;
	public int NextSweeperClock = 0;

	Random rnd = new Random();

	public Designator(IMyLargeTurretBase turret)
	{
		Turret = turret;
		MaxDesignatorRange = turret.GetMaximum<float>("Range");
		RangeProperty = turret.GetProperty("Range").As<float>();
	}

	public void SetMaxRange()
	{
		RangeProperty.SetValue(Turret, MaxDesignatorRange - 0.01f);
		RangeProperty.SetValue(Turret, MaxDesignatorRange);
	}
}

public class WeaponProfile
{
	public double InitialSpeed = 400;
	public double Acceleration = 0;
	public double MaxSpeed = 400;
	public double MaxRange = 850;
	public double SpawnOffset = 0;
	public double ReloadTime = 0;
	public bool IsCappedSpeed = false;
	public bool UseSalvo = false;

	public WeaponProfile(double initialSpeed, double acceleration, double maxSpeed, double maxRange, double spawnOffset, double reloadTime, bool isCappedSpeed, bool useSalvo)
	{
		InitialSpeed = initialSpeed;
		Acceleration = acceleration;
		MaxSpeed = maxSpeed;
		MaxRange = maxRange;
		SpawnOffset = spawnOffset;
		ReloadTime = reloadTime;
		IsCappedSpeed = isCappedSpeed;
		UseSalvo = useSalvo;
	}

	public Vector3D ComputeInterceptPoint(ref Vector3D targetLocation, ref Vector3D targetVelocity, ref Vector3D projectileLocation, ref Vector3D shipDirection, PDCTarget target)
	{
		Vector3D relativeVelocity = (IsCappedSpeed ? targetVelocity : targetVelocity - shipDirection);

		Vector3D z = targetLocation - projectileLocation;
		double k = (Acceleration == 0 ? 0 : (MaxSpeed - InitialSpeed) / Acceleration);
		double p = (0.5 * Acceleration * k * k) + (InitialSpeed * k) - (MaxSpeed * k);

		double a = (MaxSpeed * MaxSpeed) - relativeVelocity.LengthSquared();
		double b = 2 * ((p * MaxSpeed) - relativeVelocity.Dot(z));
		double c = (p * p) - z.LengthSquared();

		double t = SolveQuadratic(a, b, c);

		if (double.IsNaN(t) || t < 0)
		{
			return new Vector3D(double.NaN);
		}

		Vector3D targetPoint;

		if (target.Acceleration.LengthSquared() > 0.1)
		{
			targetPoint = targetLocation + (relativeVelocity * t) + (0.5 * target.Acceleration * t * t);
		}
		else
		{
			targetPoint = targetLocation + (relativeVelocity * t);
		}

		if (IsCappedSpeed && MaxSpeed > 0)
		{
			int u = (int)Math.Ceiling(t * 60);

			Vector3D aimDirection;
			Vector3D stepAcceleration;
			Vector3D currentPosition;
			Vector3D currentDirection;

			aimDirection = Vector3D.Normalize(targetPoint - projectileLocation);
			stepAcceleration = (aimDirection * Acceleration) / 60;

			currentPosition = projectileLocation;
			currentDirection = shipDirection + (aimDirection * InitialSpeed);

			for (int i = 0; i < u; i++)
			{
				currentDirection += stepAcceleration;

				double speed = currentDirection.Length();
				if (speed > MaxSpeed)
				{
					currentDirection = currentDirection / speed * MaxSpeed;
				}

				currentPosition += (currentDirection / 60);

				if ((i + 1) % 60 == 0)
				{
					if (Vector3D.Distance(projectileLocation, currentPosition) > MaxRange)
					{
						return targetPoint;
					}
				}
			}

			return targetPoint + targetPoint - currentPosition;
		}
		else
		{
			return targetPoint;
		}
	}

	public double SolveQuadratic(double a, double b, double c)
	{
		if (a == 0)
		{
			return -(c / b);
		}

		double u = (b * b) - (4 * a * c);
		if (u < 0)
		{
			return Double.NaN;
		}
		u = Math.Sqrt(u);

		double t1 = (-b + u) / (2 * a);
		double t2 = (-b - u) / (2 * a);
		return (t1 > 0 ? (t2 > 0 ? (t1 < t2 ? t1 : t2) : t1) : t2);
	}
}

public class RaycastHandler
{
	//Precalculated scalars for checking raycastable camera angles
	float coneLimit;
	double sideScale;
	bool noFwdScale;

	//Cameras to be used for raycast lock-on
	List<IMyCameraBlock> m_cameras;
	public List<IMyCameraBlock> Cameras
	{
		get { return m_cameras; }
		set
		{
			foreach (IMyCameraBlock camera in value)
			{
				camera.Enabled = true;
				camera.EnableRaycast = true;
			}
			m_cameras = value;

			CompileCameraGroups();
		}
	}
	List<CameraGroup> m_cameraGroups;

	public RaycastHandler(List<IMyCameraBlock> cameras)
	{
		if (cameras.Count > 0)
		{
			coneLimit = cameras[0].RaycastConeLimit;
			if (coneLimit == 0f || coneLimit == 180f) sideScale = double.NaN;
			else sideScale = Math.Tan(MathHelper.ToRadians(90 - coneLimit));

			noFwdScale = double.IsNaN(sideScale) || double.IsInfinity(sideScale);
			if (noFwdScale) sideScale = 1;
		}
		else
		{
			coneLimit = 45;
			sideScale = 1;
			noFwdScale = false;
		}

		Cameras = cameras;

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

	//Get available camera and perform raycast
	public bool Raycast(ref Vector3D targetPosition, out MyDetectedEntityInfo entityInfo, double extraDistance = 0)
	{
		IMyCameraBlock camera = GetRaycastable(ref targetPosition);
		if (camera != null)
		{
			if (extraDistance == 0)
			{
				entityInfo = Raycast(camera, ref targetPosition);
			}
			else
			{
				Vector3D targetRangeVector = targetPosition - camera.WorldMatrix.Translation;
				Vector3D adjustedPosition = targetPosition + ((extraDistance / Math.Max(targetRangeVector.Length(), 0.000000000000001)) * targetRangeVector);
				entityInfo = Raycast(camera, ref adjustedPosition);
			}
			return true;
		}
		else
		{
			entityInfo = default(MyDetectedEntityInfo);
			return false;
		}
	}

	IMyCameraBlock GetRaycastable(ref Vector3D targetPosition)
	{
		foreach (CameraGroup tryGroup in m_cameraGroups)
		{
			if (tryGroup.WithinLimits(ref targetPosition))
			{
				return GetFromCameraGroup(tryGroup, ref targetPosition);
			}
		}

		return null;
	}

	IMyCameraBlock GetFromCameraGroup(CameraGroup group, ref Vector3D targetPosition)
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
				if (CanScan(camera, ref targetPosition))
				{
					return camera;
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

		return null;
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
}

public class MissileCommsTarget
{
	public long TransmitUniqueId;

	public PDCTarget Target;
	public int TransmitUntilClock;

	public int NextTransmitClock;

	public bool UseOffsetTargeting;

	public bool TransmitAsSwitchLost;

	public IMyProgrammableBlock CacheMissilePB;
}

public class PDCTurret
{
	public GeneralSettings Settings;

	public MyIni TurretConfig;

	public string GroupName;

	public IMyMotorStator AzimuthRotor;
	public double AngleOffsetX;
	public double LowerLimitX;
	public double UpperLimitX;
	public bool HaveLimitX;
	
	public double ActualAzimuth;

	public PredictiveController AzimuthRotorCtrl;

	public ElevationGroup[] ElevationRotors;
	public WeaponProfile WeaponStats;

	public double TurretPrioritySize;

	public float RotorSpeedLimit;

	public IMyShipController Controller;

	public ITerminalProperty<bool> WeaponShootProperty;
	public ITerminalAction WeaponShootAction;

	public IPDCOcclusionChecker GridClearanceCheck;

	public double TargetedCountGainPerHit;

	public PDCTarget TargetInfo;

	public bool IsDamaged;

	public IMyShipConnector ReloadBaseConnector;
	public IMyShipConnector ReloadTurretConnector;

	public int LastReloadConnectAttempt;

	public int LastReloadCheckClock;
	public int LastReloadOperationCheck;

	public ReloadState CurrentReloadState;
	public double ReloadMaxAngleRadians;
	public double ReloadLockStateAngleRadians;

	long previousAimEntityId;
	Vector3D previousAverageAimVector;

	double offsetMoveAngleTanPerSecond = Math.Tan(MathHelperD.ToRadians(OFFSET_POINTS_MOVE_ANGLE_PER_SECOND));

	long currentOffsetEntityId;
	int currentOffsetIndex;
	Vector3D currentOffsetStart;
	Vector3D currentOffsetEnd;
	double currentOffsetLastClock;
	double currentOffsetMoveFactorPerTick;
	double currentOffsetAmount;

	public PDCTurret(string groupName, IMyMotorStator azimuthRotor, IMyMotorStator elevationRotor, IMyTerminalBlock aimBlock, IMyShipController controller, List<IMyUserControllableGun> weapons, WeaponProfile weaponStats, GeneralSettings settings, IPDCOcclusionChecker gridClearanceCheck = null)
		: this(groupName, azimuthRotor, new List<IMyMotorStator>(new IMyMotorStator[] { elevationRotor }), new List<IMyTerminalBlock>(new IMyTerminalBlock[] { aimBlock }), controller, new List<List<IMyUserControllableGun>>() { weapons }, weaponStats, settings, gridClearanceCheck) { }
	public PDCTurret(string groupName, IMyMotorStator azimuthRotor, List<IMyMotorStator> elevationRotors, List<IMyTerminalBlock> aimBlocks, IMyShipController controller, List<List<IMyUserControllableGun>> weapons, WeaponProfile weaponStats, GeneralSettings settings, IPDCOcclusionChecker gridClearanceCheck = null)
	{
		Settings = settings;

		GroupName = groupName;

		Controller = controller;
		WeaponStats = weaponStats;
		
		RotorSpeedLimit = MathHelper.RPMToRadiansPerSecond * (Settings.RotorUseLimitSnap ? Settings.RotorSnapSpeedLimit : Settings.RotorCtrlOutputLimit);

		GridClearanceCheck = gridClearanceCheck;

		TurretConfig = new MyIni();
		if (TurretConfig.TryParse(azimuthRotor.CustomData))
		{
			if (TurretConfig.ContainsSection(INI_SECTION))
			{
				double weaponInitialSpeed = TurretConfig.Get(INI_SECTION, "WeaponInitialSpeed").ToDouble(0);
				double weaponAcceleration = TurretConfig.Get(INI_SECTION, "WeaponAcceleration").ToDouble(0);
				double weaponMaxSpeed = TurretConfig.Get(INI_SECTION, "WeaponMaxSpeed").ToDouble(0);
				double weaponMaxRange = TurretConfig.Get(INI_SECTION, "WeaponMaxRange").ToDouble(0);
				double weaponSpawnOffset = TurretConfig.Get(INI_SECTION, "WeaponSpawnOffset").ToDouble(0);
				double weaponReloadTime = TurretConfig.Get(INI_SECTION, "WeaponReloadTime").ToDouble(0);
				bool weaponIsCappedSpeed = TurretConfig.Get(INI_SECTION, "WeaponIsCappedSpeed").ToBoolean(false);
				bool weaponUseSalvo = TurretConfig.Get(INI_SECTION, "WeaponUseSalvo").ToBoolean(false);
				
				TurretPrioritySize = TurretConfig.Get(INI_SECTION, "TurretPrioritySize").ToDouble(0);

				if (weaponMaxRange > 0 && (weaponInitialSpeed > 0 || weaponAcceleration > 0))
				{
					weaponInitialSpeed = MathHelper.Clamp(weaponInitialSpeed, 0, 1000000000);
					weaponAcceleration = MathHelper.Clamp(weaponAcceleration, 0, 1000000000);

					weaponMaxSpeed = MathHelper.Clamp(weaponMaxSpeed, 0, 1000000000);
					if (weaponMaxSpeed == 0) weaponMaxSpeed = 1000000000;

					weaponMaxRange = MathHelper.Clamp(weaponMaxRange, 0, 1000000000);

					WeaponStats = new WeaponProfile(weaponInitialSpeed, weaponAcceleration, weaponMaxSpeed, weaponMaxRange, weaponSpawnOffset, weaponReloadTime, weaponIsCappedSpeed, weaponUseSalvo);
				}
			}
		}
		else
		{
			TurretPrioritySize = 0;
		}
		
		if (weapons.Count > 0 && weapons[0].Count > 0)
		{
			WeaponShootProperty = weapons[0][0].GetProperty("Shoot").As<bool>();
			WeaponShootAction = weapons[0][0].GetActionWithName("ShootOnce");
		}

		AzimuthRotor = azimuthRotor;
		GetOrSetRotorLimitConfig(AzimuthRotor, out LowerLimitX, out UpperLimitX);
		HaveLimitX = (LowerLimitX > double.MinValue && UpperLimitX < double.MaxValue);

		if (!Settings.RotorUseLimitSnap)
		{
			AzimuthRotorCtrl = new PredictiveController(Settings.RotorCtrlOutputGain, Settings.RotorCtrlDeltaGain, RotorSpeedLimit);
			SetRotorLimits(AzimuthRotor, LowerLimitX, UpperLimitX);
		}

		ElevationRotors = new ElevationGroup[elevationRotors.Count];
		for (int i = 0; i < elevationRotors.Count; i++)
		{
			ElevationGroup elevationRotor = new ElevationGroup();
			ElevationRotors[i] = elevationRotor;

			elevationRotor.Rotor = elevationRotors[i];
			elevationRotor.AimBlock = aimBlocks[i];

			double lowerLimitY, upperLimitY;
			GetOrSetRotorLimitConfig(elevationRotor.Rotor, out lowerLimitY, out upperLimitY);
			elevationRotor.HaveLimitY = (lowerLimitY > double.MinValue && upperLimitY < double.MaxValue);
			elevationRotor.LowerLimitY = lowerLimitY;
			elevationRotor.UpperLimitY = upperLimitY;

			SetElevationOffsetAndReverse(elevationRotor);

			if (!Settings.RotorUseLimitSnap)
			{
				elevationRotor.RotorCtrl = new PredictiveController(Settings.RotorCtrlOutputGain, Settings.RotorCtrlDeltaGain, RotorSpeedLimit);
				SetRotorLimits(elevationRotor.Rotor, lowerLimitY, upperLimitY);
			}

			elevationRotor.Weapons = weapons[i];

			if (WeaponStats.UseSalvo)
			{
				elevationRotor.WeaponStaggerInterval = (int)Math.Ceiling((WeaponStats.ReloadTime * TICKS_PER_SECOND) / Math.Max(elevationRotor.Weapons.Count, 1));
			}
		}

		if (ElevationRotors.Length > 0)
		{
			SetAzimuthOffsetAndReverse(AzimuthRotor, ElevationRotors[0], ref AngleOffsetX);
		}
	}

	public void TransferTarget(PDCTurret prevPDC)
	{
		TargetInfo = prevPDC.TargetInfo;

		previousAimEntityId = prevPDC.previousAimEntityId;
		previousAverageAimVector = prevPDC.previousAverageAimVector;
                                    
		currentOffsetEntityId = prevPDC.currentOffsetEntityId;
		currentOffsetIndex = prevPDC.currentOffsetIndex;
		currentOffsetStart = prevPDC.currentOffsetStart;
		currentOffsetEnd = prevPDC.currentOffsetEnd;
		currentOffsetLastClock = prevPDC.currentOffsetLastClock;
		currentOffsetMoveFactorPerTick = prevPDC.currentOffsetMoveFactorPerTick;
		currentOffsetAmount = prevPDC.currentOffsetAmount;
	}

	public void SetAzimuthOffsetAndReverse(IMyMotorStator azimuthRotor, ElevationGroup pilotElevationRotor, ref double angleOffsetX)
	{
		IMyMotorStator elevationRotor = pilotElevationRotor.Rotor;

		double cosAngle = Math.Cos(elevationRotor.Angle);
		double sinAngle = Math.Sin(elevationRotor.Angle);
		Vector3D elevGridBackward = (elevationRotor.WorldMatrix.Backward * cosAngle) + (elevationRotor.WorldMatrix.Left * sinAngle);
		Vector3D elevGridLeft = (elevationRotor.WorldMatrix.Left * cosAngle) + (elevationRotor.WorldMatrix.Forward * sinAngle);

		double elevationZero = pilotElevationRotor.AngleOffsetY + ComputeOffsetFromVector(pilotElevationRotor.AimBlock.WorldMatrix.Forward, elevGridBackward, elevGridLeft);
		if (elevationZero >= MathHelperD.TwoPi) elevationZero -= MathHelperD.TwoPi;

		cosAngle = Math.Cos(elevationZero);
		sinAngle = Math.Sin(elevationZero);
		Vector3D elevationAimForward = (pilotElevationRotor.Rotor.WorldMatrix.Backward * cosAngle) + (pilotElevationRotor.Rotor.WorldMatrix.Left * sinAngle);
		
		cosAngle = Math.Cos(azimuthRotor.Angle);
		sinAngle = Math.Sin(azimuthRotor.Angle);
		Vector3D azimuthGridBackward = (azimuthRotor.WorldMatrix.Backward * cosAngle) + (azimuthRotor.WorldMatrix.Left * sinAngle);
		Vector3D azimuthGridLeft = (azimuthRotor.WorldMatrix.Left * cosAngle) + (azimuthRotor.WorldMatrix.Forward * sinAngle);

		angleOffsetX = -ComputeOffsetFromVector(elevationAimForward, azimuthGridBackward, azimuthGridLeft);
		angleOffsetX = (Math.Round(angleOffsetX / MathHelper.PiOver2) % 4) * MathHelper.PiOver2;
	}

	public void SetElevationOffsetAndReverse(ElevationGroup elevationRotor)
	{
		IMyMotorStator rotor = elevationRotor.Rotor;
		
		double cosAngle = Math.Cos(rotor.Angle);
		double sinAngle = Math.Sin(rotor.Angle);
		Vector3D elevGridBackward = (rotor.WorldMatrix.Backward * cosAngle) + (rotor.WorldMatrix.Left * sinAngle);
		Vector3D elevGridLeft = (rotor.WorldMatrix.Left * cosAngle) + (rotor.WorldMatrix.Forward * sinAngle);

		elevationRotor.AngleOffsetY = ComputeOffsetFromVector(elevationRotor.AimBlock.WorldMatrix.Forward, elevGridBackward, elevGridLeft);

		double lowerLimit = rotor.LowerLimitRad;
		double upperLimit = rotor.UpperLimitRad;

		double midAngle;
		if (rotor.LowerLimitDeg == float.MinValue || rotor.UpperLimitDeg == float.MaxValue)
		{
			midAngle = rotor.Angle + elevationRotor.AngleOffsetY;
		}
		else
		{
			midAngle = ((upperLimit + lowerLimit) * 0.5) + elevationRotor.AngleOffsetY;
		}
		Limit2PILite(ref midAngle);

		cosAngle = Math.Cos(midAngle);
		sinAngle = Math.Sin(midAngle);
		Vector3D adjustedVector = (rotor.WorldMatrix.Backward * cosAngle) + (rotor.WorldMatrix.Left * sinAngle);

		Vector3D crossForward = rotor.WorldMatrix.Up.Cross(AzimuthRotor.WorldMatrix.Up);
		elevationRotor.ReverseY = (crossForward.Dot(adjustedVector) < 0);

		double rotorToForwardDeviation = ComputeOffsetFromVector(rotor.WorldMatrix.Backward, elevationRotor.ReverseY ? -crossForward : crossForward, AzimuthRotor.WorldMatrix.Up);
		if (elevationRotor.ReverseY) rotorToForwardDeviation = MathHelper.TwoPi - rotorToForwardDeviation;
		elevationRotor.AngleOffsetY = - rotorToForwardDeviation - elevationRotor.AngleOffsetY;
		elevationRotor.AngleOffsetY = (Math.Round(elevationRotor.AngleOffsetY / MathHelper.PiOver2) % 4) * MathHelper.PiOver2;
	}

	public double ComputeOffsetFromVector(Vector3D checkVector, Vector3D baseRef0DegVector, Vector3D baseRef90DegVector)
	{
		double offsetDotCheck = Math.Round(baseRef0DegVector.Dot(checkVector));
		if (offsetDotCheck == 0)
		{
			if (baseRef90DegVector.Dot(checkVector) > 0) return MathHelperD.PiOver2;
			else return MathHelperD.PiOver2 + MathHelperD.Pi;
		}
		else
		{
			if (offsetDotCheck > 0) return 0;
			else return MathHelperD.Pi;
		}
	}

	public void ResetRotors()
	{
		AzimuthRotor.TargetVelocityRad = 0f;
		for (int i = 0; i < ElevationRotors.Length; i++)
		{
			ElevationRotors[i].Rotor.TargetVelocityRad = 0f;
		}
	}

	public bool IsAzimuthRotorIntact()
	{
		return (AzimuthRotor.IsWorking && AzimuthRotor.IsAttached);
	}

	public bool IsElevationRotorIntact(ElevationGroup elevationRotor)
	{
		return (elevationRotor.Rotor.IsWorking && elevationRotor.Rotor.IsAttached && elevationRotor.AimBlock.IsFunctional);
	}

	public bool IsElevationRotorIntact()
	{
		foreach (ElevationGroup elevationRotor in ElevationRotors)
		{
			if (elevationRotor.Rotor.IsWorking && elevationRotor.Rotor.IsAttached && elevationRotor.AimBlock.IsFunctional)
			{
				return true;
			}
		}
		return false;
	}

	public ElevationGroup GetPilotElevationRotor()
	{
		foreach (ElevationGroup elevationRotor in ElevationRotors)
		{
			if (elevationRotor.Rotor.IsWorking && elevationRotor.Rotor.IsAttached && elevationRotor.AimBlock.IsFunctional)
			{
				return elevationRotor;
			}
		}
		return ElevationRotors[0];
	}

	private bool ComputeTargetVectorAndDistance(PDCTarget target, int currentClock, out Vector3D targetVector, out double targetDistance)
	{
		Vector3D shipPosition = ElevationRotors[0].AimBlock.WorldMatrix.Translation;

		Vector3D selectedPosition;
		if (target.OffsetPoints != null)
		{
			selectedPosition = SelectOffsetPosition(target, currentClock);
		}
		else
		{
			selectedPosition = target.Position;
		}

		Vector3D targetPosition = (target.DetectedClock == currentClock ? selectedPosition : selectedPosition + ((currentClock - target.DetectedClock) * INV_ONE_TICK * target.Velocity));
		Vector3D shipDirection = (Controller == null ? Vector3D.Zero : Controller.GetShipVelocities().LinearVelocity);

		if (WeaponStats.SpawnOffset != 0)
		{
			shipPosition += ElevationRotors[0].AimBlock.WorldMatrix.Forward * WeaponStats.SpawnOffset;
		}

		targetPosition = WeaponStats.ComputeInterceptPoint(ref targetPosition, ref target.Velocity, ref shipPosition, ref shipDirection, target);

		if (double.IsNaN(targetPosition.Sum))
		{
			targetVector = Vector3D.Zero;
			targetDistance = 0;
			return false;
		}
		else
		{
			targetVector = targetPosition - shipPosition;
			targetDistance = targetVector.Length();
			targetVector = (targetDistance == 0 ? Vector3D.Zero : targetVector / targetDistance);
			return true;
		}
	}

	private Vector3D SelectOffsetPosition(PDCTarget target, int currentClock)
	{
		if (target.OffsetPoints?.Count == 0 || currentClock > target.OrientationUpdatedClock + TARGET_ORIENTATION_EXPIRE_TICKS || target.Orientation == null)
		{
			return target.Position;
		}

		bool resetMoveAmount = false;

		if (currentOffsetEntityId != target.EntityId)
		{
			currentOffsetEntityId = target.EntityId;
			currentOffsetIndex = 0;

			currentOffsetStart = Vector3D.TransformNormal(target.Position - target.CenterPosition, MatrixD.Transpose(target.Orientation.Value));
			currentOffsetEnd = target.OffsetPoints[0].Point;

			resetMoveAmount = true;
		}
		else if (currentOffsetAmount > 1)
		{
			currentOffsetIndex++;
			if (currentOffsetIndex >= target.OffsetPoints.Count)
			{
				currentOffsetIndex = 0;
			}

			currentOffsetStart = currentOffsetEnd;
			currentOffsetEnd = target.OffsetPoints[currentOffsetIndex].Point;

			resetMoveAmount = true;
		}

		if (resetMoveAmount)
		{
			currentOffsetLastClock = currentClock;

			double offsetApartLength = (currentOffsetStart - currentOffsetEnd).Length();

			if (offsetApartLength < 1)
			{
				currentOffsetMoveFactorPerTick = INV_ONE_TICK;
			}
			else
			{
				currentOffsetMoveFactorPerTick = INV_ONE_TICK / offsetApartLength * (offsetMoveAngleTanPerSecond * (target.CenterPosition - AzimuthRotor.WorldMatrix.Translation).Length());
			}
			currentOffsetAmount = 0;
		}

		currentOffsetAmount += (currentClock - currentOffsetLastClock) * currentOffsetMoveFactorPerTick;
		currentOffsetLastClock = currentClock;

		return target.CenterPosition + Vector3D.TransformNormal(Vector3D.Lerp(currentOffsetStart, currentOffsetEnd, currentOffsetAmount), target.Orientation.Value);
	}

	private bool CheckAndSetAllRotors(double aimAzimuth, double aimElevation, int currentClock, bool testOnly = false)
	{
		aimAzimuth += AngleOffsetX;
		if (aimAzimuth >= MathHelperD.TwoPi) aimAzimuth -= MathHelperD.TwoPi;

		double yaw;
		if (HaveLimitX)
		{
			if (!GetClampedCorrectionValue(out yaw, aimAzimuth, ActualAzimuth, LowerLimitX, UpperLimitX))
			{
				return false;
			}
		}
		else
		{
			yaw = aimAzimuth - ActualAzimuth;

			if (yaw > Math.PI) yaw -= MathHelperD.TwoPi;
			else if (yaw < -Math.PI) yaw += MathHelperD.TwoPi;
		}

		if (!testOnly)
		{
			if (AzimuthRotorCtrl == null)
			{
				AzimuthRotor.TargetVelocityRad = Math.Min(RotorSpeedLimit, Math.Max(-RotorSpeedLimit, (float)(yaw * Settings.RotorSnapVelocityGain)));
				SnapRotorLimitsToAngle(AzimuthRotor, yaw);
			}
			else
			{
				AzimuthRotor.TargetVelocityRad = (float)AzimuthRotorCtrl.Filter(ActualAzimuth, aimAzimuth, currentClock);
			}
		}

		for (int i = 0; i < ElevationRotors.Length; i++)
		{
			ElevationGroup ElevationRotor = ElevationRotors[i];

			if (!IsElevationRotorIntact(ElevationRotor)) continue;

			double curElevation;
			curElevation = ElevationRotor.CurrentElevation;

			double aimElevLocal = aimElevation;

			if (ElevationRotor.ReverseY)
			{
				curElevation = ElevationRotor.AngleOffsetY - curElevation;
				if (curElevation < -MathHelperD.TwoPi) curElevation += MathHelperD.TwoPi;

				aimElevLocal = ElevationRotor.AngleOffsetY - aimElevLocal;
				if (aimElevLocal < -MathHelperD.TwoPi) aimElevLocal += MathHelperD.TwoPi;
			}
			else
			{
				curElevation += ElevationRotor.AngleOffsetY;
				if (curElevation >= MathHelperD.TwoPi) curElevation -= MathHelperD.TwoPi;

				aimElevLocal += ElevationRotor.AngleOffsetY;
				if (aimElevLocal >= MathHelperD.TwoPi) aimElevLocal -= MathHelperD.TwoPi;
			}

			double pitch;
			if (ElevationRotor.HaveLimitY)
			{
				if (!GetClampedCorrectionValue(out pitch, aimElevLocal, curElevation, ElevationRotor.LowerLimitY, ElevationRotor.UpperLimitY))
				{
					return false;
				}
			}
			else
			{
				pitch = aimElevLocal - curElevation;

				if (pitch > Math.PI) pitch -= MathHelperD.TwoPi;
				else if (pitch < -Math.PI) pitch += MathHelperD.TwoPi;
			}

			if (!testOnly)
			{
				if (ElevationRotor.RotorCtrl == null)
				{
					ElevationRotor.Rotor.TargetVelocityRad = Math.Min(RotorSpeedLimit, Math.Max(-RotorSpeedLimit, (float)(pitch * Settings.RotorSnapVelocityGain)));
					SnapRotorLimitsToAngle(ElevationRotor.Rotor, pitch);
				}
				else
				{
					ElevationRotor.Rotor.TargetVelocityRad = (float)ElevationRotor.RotorCtrl.Filter(curElevation, aimElevLocal, currentClock);
				}
			}
			else
			{
				break;
			}
		}

		return true;
	}

	public bool CheckReloadRequired()
	{
		long startTiming = DateTime.Now.Ticks;

		if (ReloadBaseConnector == null || ReloadTurretConnector == null)
		{
			return false;
		}

		if ((ReloadBaseConnector.GetInventory()?.IsItemAt(0) ?? false) && (!ReloadTurretConnector.GetInventory()?.IsItemAt(0) ?? false))
		{
			return true;
		}

		return false;
	}

	public bool PerformReloadProcedure(int currentClock)
	{
		if (ReloadBaseConnector == null || ReloadTurretConnector == null)
		{
			return true;
		}

		if (IsDamaged)
		{
			CurrentReloadState = ReloadState.NONE;
			return true;
		}
		else if (!IsAzimuthRotorIntact() || !IsElevationRotorIntact())
		{
			CurrentReloadState = ReloadState.NONE;
			IsDamaged = true;
			return true;
		}

		if (CurrentReloadState == ReloadState.NONE)
		{
			ReleaseWeapons(true);
			CurrentReloadState = ReloadState.MAX_ANGLE;
		}

		ElevationGroup pilotElevationRotor = GetPilotElevationRotor();

		double aimElevation;

		switch (CurrentReloadState)
		{
			case ReloadState.MAX_ANGLE:
				UpdateRotorBearings(ElevationRotors, pilotElevationRotor, AzimuthRotor);
				aimElevation = ReloadMaxAngleRadians;

				if (Math.Abs(aimElevation - pilotElevationRotor.CurrentElevation) > 0.0018)
				{
					CheckAndSetAllRotors(ActualAzimuth, aimElevation, currentClock, false);
				}
				else
				{
					CurrentReloadState = ReloadState.LOCK_STATE_ANGLE;
				}

				break;
			case ReloadState.LOCK_STATE_ANGLE:
				UpdateRotorBearings(ElevationRotors, pilotElevationRotor, AzimuthRotor);
				aimElevation = ReloadLockStateAngleRadians;

				if (Math.Abs(aimElevation - pilotElevationRotor.CurrentElevation) > 0.0018)
				{
					CheckAndSetAllRotors(ActualAzimuth, aimElevation, currentClock, false);
				}
				else
				{
					CurrentReloadState = ReloadState.LOCK_CONNECTORS;
				}

				break;
			case ReloadState.LOCK_CONNECTORS:
				if (ReloadTurretConnector.Status == MyShipConnectorStatus.Connected)
				{
					CurrentReloadState = ReloadState.TRANSFER_ITEMS;
				}
				else if (ReloadTurretConnector.Status == MyShipConnectorStatus.Connectable)
				{
					if (currentClock >= LastReloadConnectAttempt + 30)
					{
						ReloadTurretConnector.Connect();

						LastReloadConnectAttempt = currentClock;
					}
				}
				break;
			case ReloadState.TRANSFER_ITEMS:
				if (ReloadBaseConnector.GetInventory()?.IsItemAt(0) ?? false)
				{
					ReloadTurretConnector.GetInventory()?.TransferItemFrom(ReloadBaseConnector.GetInventory(), 0, 0, true);
				}
				ReloadTurretConnector.Disconnect();
				ReloadBaseConnector.Disconnect();
				CurrentReloadState = ReloadState.NONE;
				return true;
		}

		return false;
	}

	public bool IsTargetable(PDCTarget target, int currentClock)
	{
		if (IsDamaged)
		{
			return false;
		}
		else if (!IsAzimuthRotorIntact() || !IsElevationRotorIntact())
		{
			IsDamaged = true;
			return false;
		}

		if (CurrentReloadState != ReloadState.NONE)
		{
			return false;
		}

		Vector3D targetVector;
		double targetDistance;
		bool interceptable = ComputeTargetVectorAndDistance(target, currentClock, out targetVector, out targetDistance);

		if (interceptable)
		{
			ElevationGroup pilotElevationRotor = GetPilotElevationRotor();

			if (HaveLimitX || pilotElevationRotor.HaveLimitY)
			{
				double aimAzimuth, aimElevation;
				UpdateRotorBearings(ElevationRotors, pilotElevationRotor, AzimuthRotor);
				GetAzimuthElevation(targetVector, AzimuthRotor, out aimAzimuth, out aimElevation);

				if (!CheckAndSetAllRotors(aimAzimuth, aimElevation, currentClock, true))
				{
					return false;
				}
			}

			if (GridClearanceCheck != null && GridClearanceCheck.TestWorldRayHit(pilotElevationRotor.AimBlock.WorldMatrix.Translation, targetVector))
			{
				return false;
			}

			if (targetDistance <= WeaponStats.MaxRange)
			{
				return true;
			}
		}

		return false;
	}

	public bool AimAndFire(PDCTarget target, int currentClock)
	{
		if (IsDamaged)
		{
			ReleaseWeapons(true);
			ResetRotors();
			return false;
		}
		else if (!IsAzimuthRotorIntact() || !IsElevationRotorIntact())
		{
			ReleaseWeapons(true);
			ResetRotors();
			IsDamaged = true;
			return false;
		}

		if (CurrentReloadState != ReloadState.NONE)
		{
			return false;
		}

		Vector3D targetVector;
		double targetDistance;
		bool interceptable = ComputeTargetVectorAndDistance(target, currentClock, out targetVector, out targetDistance);

		if (interceptable)
		{
			ElevationGroup pilotElevationRotor = GetPilotElevationRotor();

			double aimAzimuth, aimElevation;
			UpdateRotorBearings(ElevationRotors, pilotElevationRotor, AzimuthRotor);
			GetAzimuthElevation(targetVector, AzimuthRotor, out aimAzimuth, out aimElevation);

			if (CheckAndSetAllRotors(aimAzimuth, aimElevation, currentClock, false))
			{
				if (GridClearanceCheck != null && GridClearanceCheck.TestWorldRayHit(pilotElevationRotor.AimBlock.WorldMatrix.Translation, targetVector))
				{
					ReleaseWeapons();
					ResetRotors();
					return false;
				}

				if (targetDistance <= WeaponStats.MaxRange)
				{
					bool haveAimedWeapons = false;

					foreach (ElevationGroup elevationRotor in ElevationRotors)
					{
						double dotAngle = elevationRotor.AimBlock.WorldMatrix.Forward.Dot(targetVector);
						bool withinRange = (dotAngle >= Settings.PDCFireDotLimit);
						if (!withinRange && dotAngle >= COS_45_DEGREES && previousAimEntityId == target.EntityId)
						{
							double dotAngle2 = elevationRotor.AimBlock.WorldMatrix.Forward.Dot(previousAverageAimVector);
							double dotAngle3 = targetVector.Dot(previousAverageAimVector);

							withinRange = (dotAngle2 >= Settings.PDCFireDotLimit) || (dotAngle >= dotAngle3 && dotAngle2 >= dotAngle3);
						}
						
						if (withinRange)
						{
							FireWeapons(elevationRotor, currentClock, WeaponStats.UseSalvo);
							haveAimedWeapons = true;
						}
						else
						{
							ReleaseWeapons(elevationRotor);
						}
					}

					if (haveAimedWeapons)
					{
						target.PDCTargetedCount += TargetedCountGainPerHit;
					}

					previousAimEntityId = target.EntityId;
					previousAverageAimVector = targetVector;
					
					return true;
				}
			}
		}

		ReleaseWeapons();
		ResetRotors();
		return false;
	}

	public void UpdateRotorBearings(ElevationGroup[] elevationRotors, ElevationGroup pilotElevationRotor, IMyMotorStator azRotor)
	{
		ActualAzimuth = azRotor.Angle;
		Limit2PILite(ref ActualAzimuth);

		foreach (ElevationGroup elevationRotor in elevationRotors)
		{
			if (elevationRotor == pilotElevationRotor || IsElevationRotorIntact(elevationRotor))
			{
				double dot = elevationRotor.AimBlock.WorldMatrix.Forward.Dot(azRotor.WorldMatrix.Up);
				elevationRotor.CurrentElevation = MathHelperD.PiOver2 - Math.Acos(MathHelper.Clamp(dot, -1, 1));
			}
		}
	}

	public void GetAzimuthElevation(Vector3D aimVector, IMyMotorStator azRotor, out double azimuth, out double elevation)
	{
		double dot = aimVector.Dot(azRotor.WorldMatrix.Up);
		elevation = MathHelperD.PiOver2 - Math.Acos(MathHelper.Clamp(dot, -1, 1));

		double cosAngle = Math.Cos(elevation);
		Vector3D azForward = (cosAngle == 0 ? Vector3D.Zero : (aimVector - (azRotor.WorldMatrix.Up * dot)) / cosAngle);
		azimuth = Math.Acos(MathHelper.Clamp(azRotor.WorldMatrix.Backward.Dot(azForward), -1, 1));
		if (azRotor.WorldMatrix.Right.Dot(azForward) > 0) azimuth = MathHelperD.TwoPi - azimuth;
	}

	public bool GetClampedCorrectionValue(out double correction, double desiredAngle, double currentAngle, double lowerLimit, double upperLimit)
	{
		upperLimit -= lowerLimit;
		desiredAngle -= lowerLimit;
		currentAngle -= lowerLimit;

		Limit2PILite(ref upperLimit);
		Limit2PILite(ref desiredAngle);
		Limit2PILite(ref currentAngle);

		if (desiredAngle >= 0 && desiredAngle <= upperLimit)
		{
			if (currentAngle > upperLimit)
			{
				currentAngle = (currentAngle - upperLimit <= MathHelperD.TwoPi - currentAngle ? upperLimit : 0);
			}

			correction = desiredAngle - currentAngle;
			return true;
		}
		else
		{
			correction = 0;
			return false;
		}
	}

	public void SnapRotorLimitsToAngle(IMyMotorStator rotor, double diffAngle)
	{
		float desiredAngle = rotor.Angle + (float)diffAngle;

		if (desiredAngle < -MathHelper.TwoPi || desiredAngle > MathHelper.TwoPi)
		{
			rotor.SetValueFloat("LowerLimit", float.MinValue);
			rotor.SetValueFloat("UpperLimit", float.MaxValue);
		}
		else if (diffAngle < 0)
		{
			rotor.UpperLimitRad = rotor.Angle;
			rotor.LowerLimitRad = desiredAngle;
		}
		else if (diffAngle > 0)
		{
			rotor.LowerLimitRad = rotor.Angle;
			rotor.UpperLimitRad = desiredAngle;
		}
	}

	public void GetOrSetRotorLimitConfig(IMyMotorStator rotor, out double lowerLimit, out double upperLimit)
	{
		double lowerLimitDeg = double.MinValue;
		double upperLimitDeg = double.MaxValue;

		if (Settings.RotorUseLimitSnap)
		{
			MyIni cfg = new MyIni();
			cfg.TryParse(rotor.CustomData);

			lowerLimitDeg = cfg.Get(INI_SECTION, "LowerLimit").ToDouble(lowerLimitDeg);
			upperLimitDeg = cfg.Get(INI_SECTION, "UpperLimit").ToDouble(upperLimitDeg);

			if (lowerLimitDeg == double.MinValue || upperLimitDeg == double.MaxValue)
			{
				lowerLimitDeg = MathHelper.Clamp(Math.Round(rotor.LowerLimitDeg, 3), -361, 361);
				upperLimitDeg = MathHelper.Clamp(Math.Round(rotor.UpperLimitDeg, 3), -361, 361);

				cfg.Set(INI_SECTION, "LowerLimit", lowerLimitDeg);
				cfg.Set(INI_SECTION, "UpperLimit", upperLimitDeg);

				rotor.CustomData = cfg.ToString();
			}
		}
		else
		{
			lowerLimitDeg = MathHelper.Clamp(Math.Round(rotor.LowerLimitDeg, 3), -361, 361);
			upperLimitDeg = MathHelper.Clamp(Math.Round(rotor.UpperLimitDeg, 3), -361, 361);
		}

		if (lowerLimitDeg < -360) lowerLimitDeg = double.MinValue;
		if (upperLimitDeg > 360) upperLimitDeg = double.MaxValue;

		if (upperLimitDeg < lowerLimitDeg || upperLimitDeg - lowerLimitDeg >= 360)
		{
			lowerLimitDeg = double.MinValue;
			upperLimitDeg = double.MaxValue;
		}

		if (lowerLimitDeg > double.MinValue && upperLimitDeg < double.MaxValue)
		{
			lowerLimit = MathHelperD.ToRadians(lowerLimitDeg);
			upperLimit = MathHelperD.ToRadians(upperLimitDeg);
		}
		else
		{
			lowerLimit = double.MinValue;
			upperLimit = double.MaxValue;
		}
	}

	public void Limit2PILite(ref double value)
	{
		if (value < 0)
		{
			if (value <= -MathHelperD.TwoPi) value += MathHelperD.FourPi;
			else value += MathHelperD.TwoPi;
		}
		else if (value >= MathHelperD.TwoPi)
		{
			if (value >= MathHelperD.FourPi) value -= MathHelperD.FourPi;
			else value -= MathHelperD.TwoPi;
		}
	}

	public void SetRotorLimits(IMyMotorStator rotor, double lowerLimit, double upperLimit)
	{
		if (lowerLimit == double.MinValue) rotor.SetValueFloat("LowerLimit", float.MinValue);
		else rotor.LowerLimitRad = (float)lowerLimit;

		if (upperLimit == double.MaxValue) rotor.SetValueFloat("UpperLimit", float.MaxValue);
		else rotor.UpperLimitRad = (float)upperLimit;
	}

	public void FireWeapons(ElevationGroup elevationRotor, int clock, bool isSalvo)
	{
		if (isSalvo)
		{
			if (elevationRotor.Weapons.Count > 0 && elevationRotor.WeaponNextFireClock <= clock)
			{
				if (elevationRotor.WeaponStaggerIndex >= elevationRotor.Weapons.Count)
				{
					elevationRotor.WeaponStaggerIndex = 0;
				}

				IMyUserControllableGun weapon = elevationRotor.Weapons[elevationRotor.WeaponStaggerIndex];

				WeaponShootAction.Apply(weapon);

				elevationRotor.WeaponNextFireClock = clock + elevationRotor.WeaponStaggerInterval;
				elevationRotor.WeaponStaggerIndex++;
			}
		}
		else
		{
			if (!elevationRotor.FireLatch)
			{
				foreach (IMyUserControllableGun weapon in elevationRotor.Weapons)
				{
					WeaponShootProperty.SetValue(weapon, true);
				}
				elevationRotor.FireLatch = true;
			}
		}
	}

	public void ReleaseWeapons(bool force = false)
	{
		foreach (ElevationGroup elevationRotor in ElevationRotors)
		{
			ReleaseWeapons(elevationRotor, force);
		}
	}
	public void ReleaseWeapons(ElevationGroup elevationRotor, bool force = false)
	{
		if (elevationRotor.FireLatch || force)
		{
			foreach (IMyUserControllableGun weapon in elevationRotor.Weapons)
			{
				WeaponShootProperty.SetValue(weapon, false);
			}
			elevationRotor.FireLatch = false;
		}
	}

	public class ElevationGroup
	{
		public IMyMotorStator Rotor;
		public double AngleOffsetY;
		public double LowerLimitY;
		public double UpperLimitY;
		public bool HaveLimitY;
		public bool ReverseY;

		public IMyTerminalBlock AimBlock;
		public double CurrentElevation;

		public PredictiveController RotorCtrl;

		public List<IMyUserControllableGun> Weapons;
		public bool FireLatch;

		public int WeaponStaggerIndex;
		public int WeaponNextFireClock;
		public int WeaponStaggerInterval;
	}

	public enum ReloadState
	{
		NONE,
		MAX_ANGLE,
		LOCK_STATE_ANGLE,
		LOCK_CONNECTORS,
		TRANSFER_ITEMS
	}
}

public class PDCOcclusionGrid
{
	public IMyCubeGrid Grid;
	public Vector3I StartPosition;

	public PDCOcclusionGrid(IMyCubeGrid grid, Vector3I startPosition)
	{
		Grid = grid;
		StartPosition = startPosition;
	}
}

public interface IPDCOcclusionChecker
{
	bool TestWorldRayHit(Vector3 startPosition, Vector3 aimVector);
}

public class AABBPDCOcclusionChecker : IPDCOcclusionChecker
{
	public Vector3I[] ADJACENT_VECTORS = { Vector3I.Left, Vector3I.Right, Vector3I.Up, Vector3I.Down, Vector3I.Forward, Vector3I.Backward };

	public MyDynamicAABBTree AABBTree;
	public IMyCubeGrid Grid;

	public AABBPDCOcclusionChecker() { }

	public IEnumerator<int> Init(PDCOcclusionGrid mainGrid, List<PDCOcclusionGrid> subGrids, float occlusionExtraClearance, int limit)
	{
		if (limit <= 0) limit = 1000000000;

		int counter = 0;

		Grid = mainGrid.Grid;
		AABBTree = new MyDynamicAABBTree();

		Vector3 offset = new Vector3(0.5f * Grid.GridSize);
		if (occlusionExtraClearance != 0f)
		{
			offset += new Vector3(occlusionExtraClearance);
		}

		Stack<Vector3I> remaining = new Stack<Vector3I>();
		HashSet<Vector3I> tested = new HashSet<Vector3I>();
		BoundingBox hitBox;

		remaining.Push(mainGrid.StartPosition);
		tested.Add(mainGrid.StartPosition);

		hitBox = new BoundingBox((mainGrid.StartPosition * Grid.GridSize) - offset, (mainGrid.StartPosition * Grid.GridSize) + offset);
		AABBTree.AddProxy(ref hitBox, hitBox, 0);

		Vector3I next;

		while (remaining.Count > 0)
		{
			Vector3I current = remaining.Pop();

			for (int i = 0; i < 6; i++)
			{
				next = current + ADJACENT_VECTORS[i];
				if (!tested.Contains(next))
				{
					tested.Add(next);
					if (Grid.CubeExists(next))
					{
						remaining.Push(next);

						hitBox = new BoundingBox((next * Grid.GridSize) - offset, (next * Grid.GridSize) + offset);
						AABBTree.AddProxy(ref hitBox, hitBox, 0);
					}
				}
			}

			counter++;
			if (counter % limit == 0)
			{
				yield return counter;
			}
		}

		Dictionary<IMyCubeGrid, PDCOcclusionGrid> uniqueSubGrids = new Dictionary<IMyCubeGrid, PDCOcclusionGrid>();

		foreach (PDCOcclusionGrid subGrid in subGrids)
		{
			if (!uniqueSubGrids.ContainsKey(subGrid.Grid))
			{
				uniqueSubGrids.Add(subGrid.Grid, subGrid);
			}
		}

		MatrixD matrix = Grid.WorldMatrix;
		MatrixD.Transpose(ref matrix, out matrix);

		foreach (PDCOcclusionGrid subGrid in uniqueSubGrids.Values)
		{
			if (subGrid.Grid != Grid)
			{
				Vector3 min = Vector3D.TransformNormal((subGrid.Grid.WorldAABB.Min - Grid.WorldMatrix.Translation), ref matrix);
				Vector3 max = Vector3D.TransformNormal((subGrid.Grid.WorldAABB.Max - Grid.WorldMatrix.Translation), ref matrix);

				hitBox = new BoundingBox(min - offset, max + offset);
				AABBTree.AddProxy(ref hitBox, hitBox, 0);
			}
		}
	}

	public bool TestWorldRayHit(Vector3 startPosition, Vector3 aimVector)
	{
		MatrixD matrix = Grid.WorldMatrix;
		MatrixD.Transpose(ref matrix, out matrix);

		Line line = new Line(Vector3D.TransformNormal(startPosition - Grid.WorldMatrix.Translation, ref matrix), Vector3D.TransformNormal(aimVector, ref matrix) * 1000);

		return TestLocalRayHit(ref line);
	}

	public bool TestLocalRayHit(ref Line line)
	{
		List<MyLineSegmentOverlapResult<BoundingBox>> result = new List<MyLineSegmentOverlapResult<BoundingBox>>(0);
		AABBTree.OverlapAllLineSegment(ref line, result);

		foreach (MyLineSegmentOverlapResult<BoundingBox> hit in result)
		{
			if (hit.Element.Extents.LengthSquared() < (line.From - hit.Element.Center).LengthSquared())
			{
				return true;
			}
		}
		return false;
	}
}

public class CubeExistsPDCOcclusionChecker : IPDCOcclusionChecker
{
	private readonly int MAX_ITERATIONS = 40;

	public IMyCubeGrid Grid;
	public BoundingBox GriddBB;

	double InvGridSize;

	public List<IMyCubeGrid> SubGrids;

	public float AdjustedClearance;
	public bool HaveAdjustedClearance;
	
	public CubeExistsPDCOcclusionChecker(PDCOcclusionGrid mainGrid, List<PDCOcclusionGrid> subGrids, float occlusionExtraClearance)
	{
		Grid = mainGrid.Grid;
		GriddBB = new BoundingBox(Grid.Min, Grid.Max);

		InvGridSize = 1.0 / Grid.GridSize;

		if (occlusionExtraClearance != 0)
		{
			HaveAdjustedClearance = true;
			AdjustedClearance = Grid.GridSize * 0.5f + occlusionExtraClearance;
		}
		else
		{
			HaveAdjustedClearance = false;
			AdjustedClearance = 0f;
		}
		
		Dictionary<IMyCubeGrid, PDCOcclusionGrid> uniqueSubGrids = new Dictionary<IMyCubeGrid, PDCOcclusionGrid>();

		foreach (PDCOcclusionGrid subGrid in subGrids)
		{
			if (!uniqueSubGrids.ContainsKey(subGrid.Grid))
			{
				uniqueSubGrids.Add(subGrid.Grid, subGrid);
			}
		}

		SubGrids = new List<IMyCubeGrid>(uniqueSubGrids.Count);

		MatrixD matrix = Grid.WorldMatrix;
		MatrixD.Transpose(ref matrix, out matrix);

		Vector3 offset = new Vector3(AdjustedClearance);
		
		foreach (PDCOcclusionGrid subGrid in uniqueSubGrids.Values)
		{
			if (subGrid.Grid != Grid)
			{
				SubGrids.Add(subGrid.Grid);
			}
		}
	}

	public bool TestWorldRayHit(Vector3 startPosition, Vector3 aimVector)
	{
		float startToWallAmount;
		if (Grid.WorldAABB.Contains(startPosition) == ContainmentType.Disjoint)
		{
			double? result = Grid.WorldAABB.Intersects(new Ray(startPosition, aimVector));
			if (result == null)
			{
				return false;
			}
			startToWallAmount = (float)result.Value;
		}
		else
		{
			double? result = Grid.WorldAABB.Intersects(new Ray(startPosition + (aimVector * 5000), -aimVector));
			if (result == null)
			{
				return false;
			}
			startToWallAmount = 5000f - (float)result.Value;
		}

		MatrixD matrix = Grid.WorldMatrix;
		MatrixD.Transpose(ref matrix, out matrix);

		Line line = new Line(Vector3D.TransformNormal(startPosition - Grid.WorldMatrix.Translation, ref matrix) * InvGridSize, Vector3D.TransformNormal(startPosition + (aimVector * startToWallAmount) - Grid.WorldMatrix.Translation, ref matrix) * InvGridSize);

		if (TestLocalRayHit(ref line))
		{
			return true;
		}

		if (SubGrids.Count > 0)
		{
			RayD ray = new RayD(startPosition, aimVector);

			foreach (IMyCubeGrid subGrid in SubGrids)
			{
				if (subGrid.WorldAABB.Intersects(ray) != null)
				{
					if (subGrid.WorldAABB.Extents.LengthSquared() < (ray.Position - subGrid.WorldAABB.Center).LengthSquared())
					{
						return true;
					}
				}
			}
		}

		return false;
	}

	public bool TestLocalRayHit(ref Line line)
	{
		int division = Math.Min((int)Math.Ceiling(line.Length), MAX_ITERATIONS);
		float increment = 1.0f / division * line.Length;

		Vector3I startPosition = new Vector3I((int)Math.Round(line.From.X), (int)Math.Round(line.From.Y), (int)Math.Round(line.From.Z));

		for (int i = 1; i <= division; i++)
		{
			Vector3 rawPosition = line.From + (line.Direction * increment * i);
			Vector3I testPosition = new Vector3I((int)Math.Round(rawPosition.X), (int)Math.Round(rawPosition.Y), (int)Math.Round(rawPosition.Z));

			if (Grid.CubeExists(testPosition))
			{
				if (testPosition != startPosition)
				{
					if (HaveAdjustedClearance)
					{
						double closest = Math.Min(Math.Abs(rawPosition.X - testPosition.X), Math.Min(Math.Abs(rawPosition.Y - testPosition.Y), Math.Abs(rawPosition.Z - testPosition.Z)));
						if (closest <= AdjustedClearance)
						{
							return true;
						}
					}
					else
					{
						return true;
					}
				}
			}
		}

		return false;
	}
}

public class TargetData
{
	public long EntityId;
	public Vector3D Position;
	public Vector3D Velocity;
}

public class AllyTrack
{
	public long EntityId;
	public Vector3D Position;
	public Vector3D Velocity;
	public bool IsLargeGrid;
	public double SizeSq;

	public int LastDetectedClock;

	public AllyTrack(long entityId)
	{
		EntityId = entityId;
	}
}

public class PredictiveController
{
	const int MIN_REFRESH_INTERVAL = 3;	//Hard Limit To Avoid Spasm
	
	double gain_output;
	double gain_predict;
	double output_limit;
	
	double predictedDelta;

	double prevExpected;
	int prevClock;

	public PredictiveController(double outputGain, double predictiveGain, double outputLimit)
	{
		gain_output = outputGain;
		gain_predict = predictiveGain;
		output_limit = outputLimit;
	}

	public double Filter(double current, double expected, int clock)
	{
		int deltaSteps = Math.Max(clock - prevClock, 1);

		double currentDelta = expected - prevExpected;

		if (deltaSteps < MIN_REFRESH_INTERVAL)
		{
			currentDelta *= (double)MIN_REFRESH_INTERVAL / deltaSteps;
			deltaSteps = MIN_REFRESH_INTERVAL;
		}

		AdjustAnglePI(ref currentDelta);

		if (predictedDelta * currentDelta < 0)	//Sign Reversal
		{
			predictedDelta = (gain_predict * currentDelta);
		}
		else
		{
			predictedDelta = ((1 - gain_predict) * predictedDelta) + (gain_predict * currentDelta);
		}

        double delta = expected - current + predictedDelta;
		AdjustAnglePI(ref delta);

		prevExpected = expected;
		prevClock = Math.Max(clock, prevClock);

		return MathHelper.Clamp(delta * gain_output / deltaSteps, -output_limit, output_limit);
	}

	public void AdjustAnglePI(ref double value)
	{
		if (value < -Math.PI)
		{
			value += MathHelperD.TwoPi;
			if (value < -Math.PI) value += MathHelperD.TwoPi;
		}
		else if (value > Math.PI)
		{
			value -= MathHelperD.TwoPi;
			if (value > Math.PI) value -= MathHelperD.TwoPi;
		}
	}

	public void Reset()
	{
		prevClock = 0;
		predictedDelta = prevExpected = 0;
	}
}

public class Profiler
{
	public int HistoryMaxCount;
	public double NewValueFactor;

	public double AverageRuntime;
	public double PeakRuntime;

	public double AverageComplexity;
	public double PeakComplexity;

	public IMyGridProgramRuntimeInfo Runtime { get; private set; }
	public readonly Queue<double> HistoryRuntime = new Queue<double>();
	public readonly Queue<double> HistoryComplexity = new Queue<double>();
	public readonly Dictionary<string, SectionValues> AverageBreakdown = new Dictionary<string, SectionValues>();

	private double invMaxRuntimePercent;
	private double invMaxInstCountPercent;

	public Profiler(IMyGridProgramRuntimeInfo runtime, int historyMaxCount, double newValueFactor)
	{
		Runtime = runtime;
		HistoryMaxCount = historyMaxCount;
		NewValueFactor = newValueFactor;

		invMaxRuntimePercent = 6;
		invMaxInstCountPercent = 100.0 / (Runtime.MaxInstructionCount == 0 ? 50000 : Runtime.MaxInstructionCount);
	}

	public void Clear()
	{
		AverageRuntime = 0;
		HistoryRuntime.Clear();
		PeakRuntime = 0;

		AverageComplexity = 0;
		HistoryComplexity.Clear();
		PeakComplexity = 0;
	}

	public void UpdateRuntime()
	{
		double runtime = Runtime.LastRunTimeMs;
		AverageRuntime += (runtime - AverageRuntime) * NewValueFactor;

		HistoryRuntime.Enqueue(runtime);
		if (HistoryRuntime.Count > HistoryMaxCount) HistoryRuntime.Dequeue();
		PeakRuntime = HistoryRuntime.Max();
	}

	public void UpdateComplexity()
	{
		double complexity = Runtime.CurrentInstructionCount;
		AverageComplexity += (complexity - AverageComplexity) * NewValueFactor;

		HistoryComplexity.Enqueue(complexity);
		if (HistoryComplexity.Count > HistoryMaxCount) HistoryComplexity.Dequeue();
		PeakComplexity = HistoryComplexity.Max();
	}

	public void PrintPerformance(StringBuilder sb)
	{
		sb.AppendLine($"Avg Runtime = {AverageRuntime:0.0000}ms   ({AverageRuntime * invMaxRuntimePercent:0.00}%)");
		sb.AppendLine($"Peak Runtime = {PeakRuntime:0.0000}ms\n");
		sb.AppendLine($"Avg Complexity = {AverageComplexity:0.00}   ({AverageComplexity * invMaxInstCountPercent:0.00}%)");
		sb.AppendLine($"Peak Complexity = {PeakComplexity:0.00}");
	}

	public void StartSectionWatch(string section)
	{
		SectionValues sectionValues;
		if (AverageBreakdown.ContainsKey(section))
		{
			sectionValues = AverageBreakdown[section];
		}
		else
		{
			sectionValues = new SectionValues();
			AverageBreakdown[section] = sectionValues;
		}

		sectionValues.StartTicks = DateTime.Now.Ticks;
	}

	public void StopSectionWatch(string section)
	{
		SectionValues sectionValues;
		if (AverageBreakdown.TryGetValue(section, out sectionValues))
		{
			long current = DateTime.Now.Ticks;
			double runtime = (current - sectionValues.StartTicks) * 0.0001;

			sectionValues.AccumulatedCount++;
			sectionValues.AccumulatedRuntime += runtime;
			sectionValues.StartTicks = current;
		}
	}

	public void PrintSectionBreakdown(StringBuilder sb)
	{
		foreach (KeyValuePair<string, SectionValues> entry in AverageBreakdown)
		{
			double runtime = (entry.Value.AccumulatedCount == 0 ? 0 : entry.Value.AccumulatedRuntime / entry.Value.AccumulatedCount);
			sb.AppendLine($"{entry.Key} = {runtime:0.0000}ms");
		}
	}

	public class SectionValues
	{
		public long AccumulatedCount;
		public double AccumulatedRuntime;
		public long StartTicks;
	}
}

#endregion

#endregion
//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
	}
}