//============================================================
// Raw version of the published script on my workshop.
//============================================================

//------------------------------------------------------------
// ADN - Easy Lidar Homing Script v16.1
//------------------------------------------------------------

//----- Refer To Steam Workshop Discussion Section For Variables Definition -----

//Sets the default missile homing mode. See http://steamcommunity.com/workshop/filedetails/discussion/807454034/1457328392110577137/ for more information.
int missileLaunchType = 0;

//Type of block to disconnect missile from launching ship: 0 = Merge Block, 1 = Rotor, 2 = Connector, 3 = Merge Block And Any Locked Connectors, 4 = Rotor And Any Locked Connectors, 99 = No detach required
int missileDetachPortType = 0;

//Spin Missile By This RPM After Launch Clearance
int spinAmount = 0;

//Whether to perform a vertical takeoff for the launching procedure
bool verticalTakeoff = false;

//Whether to exclude locking-on to friendly targets. Recommended to set to true after testing
bool excludeFriendly = false;

//Whether to adjust thrust based on intended and actual aim vector
bool adjustThrustBasedOnAim = false;

//Whether to fly straight first until LOCK command is given via R_TARGET Text Panel to start homing. Activating this will ignore launchSeconds variable
bool waitForHomingTrigger = false;

//Whether to allow missile to read Custom Data of the R_TARGET to get command during missile homing
bool enableMissileCommand = true;

//The amount of spread for 5 point raycast for easier initial lock-on. Zero to use normal single beam lock-on
float fivePointInitialLockDist = 0f;

//Raycast ahead of of your initial aiming, to mitigate high ping and gyro sync lag. Zero to use normal single beam lock-on
int raycastAheadCount = 0;
float raycastAheadSeconds = 0f;

//Whether to Offset Targeting based on first Lidar Hit Position
bool useOffsetTargeting = false;

//Whether to Offset Raycasting based on first Lidar Hit Position
bool useOffsetRaycasting = false;

//For AI Turret lock-on modes. Whether to switch to raycast after AI turret lock-on
bool switchTurretToRaycast = false;

//------------------------------ Inter Grid Communications Configuration ------------------------------

string missileId = null;
string missileGroup = null;
string allowedSenderId = null;

//------------------------------ Reference Block Name Configuration ------------------------------

string strShipRefBlockGroup = "GRP_ELH_SHIP";
string strDetachBlockGroup = "GRP_ELH_DETACH";

string strShipRefLidar = "R_LIDAR";
string strShipRefFwd = "R_FORWARD";
string strShipRefPanel = "R_TARGET";
string strShipRefTurrets = "R_DESIGNATOR";

string strShipRefNotMissileTag = "NOT_MISSILE";

string missileActivationCommands = "";

string missileTriggerCommands = "";

string proximityTriggerCommands = "SETWB:Warhead:Safety:True,ACTW:Warhead:Detonate";

string failunsafeTriggerCommands = "SETWB:Warhead:Safety:True";

string strGyroscopesTag = "";
string strThrustersTag = "";
string strDetachPortTag = "";
string strDirectionRefBlockTag = "";

string strProximitySensorTag = "PROXIMITY";

string strLockTriggerBlockTag = "R_ALERT";
string strLockTriggerAction = "PlaySound";

string strStatusDisplayPrefix = "<D>";

//------------------------------ Lidar Lock On Configuration ------------------------------

float LIDAR_MIN_LOCK_DISTANCE = 50;
float LIDAR_MAX_LOCK_DISTANCE = 3000;

int LIDAR_REFRESH_INTERVAL = 0;

float LIDAR_REFRESH_CALC_FACTOR = 0.85f;

//------------------------------ Missile Handling Configuration ------------------------------

double launchSeconds = 1;

bool? boolDrift = null;
bool? boolLeadTarget = null;
bool? boolNaturalDampener = null;

//------------------------------ Above Is User Configuration Section. This Section Is For PID Tuning ------------------------------

double AIM_P = 0;
double AIM_I = 0;
double AIM_D = 0;
double AIM_LIMIT = 6.3;

double INTEGRAL_WINDUP_UPPER_LIMIT = 0;
double INTEGRAL_WINDUP_LOWER_LIMIT = 0;

//------------------------------ Script Parameters Configuration ------------------------------

int MERGE_SEPARATE_WAIT_THRESHOLD = 60;

bool outputMissileStatus = true;

//------------------------------ Important Constants ------------------------------

const double DEF_SMALL_GRID_P = 31.42;
const double DEF_SMALL_GRID_I = 0;
const double DEF_SMALL_GRID_D = 10.48;

const double DEF_BIG_GRID_P = 15.71;
const double DEF_BIG_GRID_I = 0;
const double DEF_BIG_GRID_D = 7.05;

const double RPM_FACTOR = 1800 / Math.PI;

const float MIN_THRUST_ACCELERATION = 10f;
const int MIN_ACCEL_RECOUNT_TICKS = 120;
const int MIN_LIDAR_RECOUNT_TICKS = 15;
const float SECOND = 60f;

//------------------------------ Below Is Main Script Body ------------------------------

List<IMyCameraBlock> shipRefLidars;
IMyTerminalBlock shipRefFwd;
IMyTextPanel shipRefPanel;

List<IMyLargeTurretBase> shipRefTurrets;

List<IMyCameraBlock> missileLidars;
IMyShipController remoteControl;
IMyTerminalBlock refFwdBlock;
IMyTerminalBlock refDwdBlock;

IMyLargeTurretBase homingTurret;

bool hasProximitySensors;
List<ProximitySensor> proximitySensors;
int failunsafeGrpCnt;

bool isLidarMode;

IMyTerminalBlock alertBlock;
IMyTerminalBlock statusDisplay;

IMyTerminalBlock notMissile;
double notMissileRadius;

bool homingReleaseLock;

bool commsPositionSet;
Vector3D commsPosition;

bool commsFwdSet;
RayD commsFwd;

bool commsLidarTargetSet;
MyDetectedEntityInfo commsLidarTarget;

GyroControl gyroControl;

List<IMyThrust> thrusters;
float[] thrustValues;

List<IMyThrust> launchThrusters;

int adjustThrustState;

MatrixD refWorldMatrix;
MatrixD refViewMatrixUnclean4thCol;  //Used by TransformNormal only, therefore its Transposed from WorldMatrix without cleanup of 4th Column
bool refFwdReverse;

bool fwdIsTurret;

Vector3D refFwdPosition;
Vector3D refFwdVector;
bool refFwdSet;

Vector3D naturalGravity;
double naturalGravityLength;

Vector3D midPoint;
Vector3D shipVelocity;
double speed;
double rpm;

double thrustAcceleration;

Vector3D lastMidPoint;
Vector3D lastNormal;

MyDetectedEntityInfo lidarTargetInfo;

Vector3D lidarLastForward;
Vector3D lidarAimChangeVector;
int lidarLastForwardClock;

Vector3D offsetTargetPosition;
Vector3D offsetRaycastPosition;

Vector3D targetPosition;
Vector3D lastTargetPosition;

double lastTimeToIntercept;

bool targetPositionSet;
int lastTargetPositionClock;

Vector3D targetVector;
double distToTarget;

Vector3D targetDirection;
double targetSpeed;
double targetRadius;

PIDController yawController;
PIDController pitchController;
PIDController rollController;

int subCounter = 0;
int subMode = 0;
int mode = -2;
int clock = 0;
bool init = false;

IMyTerminalBlock detachBlock;
int detachBlockType = -1;

List<KeyValuePair<double, string[]>> rpmTriggerList;
List<KeyValuePair<double, string[]>> distTriggerList;
List<KeyValuePair<int, string[]>> timeTriggerList;
bool haveTriggerCommands;

Random rnd = new Random();

double ticksRatio = 0;
double ticksFactor = 0;

int lidarStaggerIndex = 0;

int nextLidarTriggerTicks = 0;
int nextLidarRecountTicks = 0;

int nextAccelRecountTicks = 0;

void Main(string arguments, UpdateType updateSource)
{
    //---------- Initialization And General Controls ----------

    if (!init)
    {
        if (subMode == 0)  //Check for configuration command
        {
            if (updateSource == UpdateType.Antenna)
            {
                return;
            }

            subMode = 1;

            missileId = Me.GetId().ToString();
            missileGroup = null;

            if (Me.CustomData.Length > 0)
            {
                ProcessCustomConfiguration();
            }

            if (raycastAheadCount > 0 && raycastAheadSeconds == 0f)
            {
                raycastAheadSeconds = 0.5f;
            }

            if (arguments.Length > 0)
            {
                ProcessConfigurationCommand(arguments);
            }

            if (subMode == 1)   //Not Diagnostic Commands
            {
                Runtime.UpdateFrequency = UpdateFrequency.Update1;
            }
            else
            {
                return;
            }
        }

        if (subMode == 1)  //Missile still on launching ship's grid
        {
            InitLaunchingShipRefBlocks();

            if (shipRefFwd != null)
            {
                refFwdPosition = shipRefFwd.WorldMatrix.Translation;
                refFwdVector = (fwdIsTurret ? CalculateTurretViewVector(shipRefFwd as IMyLargeTurretBase) : shipRefFwd.WorldMatrix.Forward);
            }

            if (!DetachFromGrid())
            {
                throw new Exception("--- Initialization Failed ---");
            }

            subCounter = 0;
            subMode = (missileDetachPortType == 99 ? 3 : 2);
            return;
        }
        else if (subMode == 2)  //Missile waiting for successful detachment from launching ship
        {
            bool isDetached = false;

            if (notMissile != null)
            {
                isDetached = (notMissile.CubeGrid != Me.CubeGrid);
            }
            else if (detachBlockType == 0)
            {
                isDetached = !((detachBlock as IMyShipMergeBlock).IsConnected);
            }
            else if (detachBlockType == 1)
            {
                isDetached = !((detachBlock as IMyMotorBase).IsAttached);
            }
            else if (detachBlockType == 2)
            {
                isDetached = ((detachBlock as IMyShipConnector).Status != MyShipConnectorStatus.Connected);
            }

            if (isDetached)
            {
                subMode = 3;
                return;
            }
            else
            {
                subCounter++;

                if (subCounter >= MERGE_SEPARATE_WAIT_THRESHOLD)
                {
                    Echo("Error: Missile detach failed.");
                    throw new Exception("--- Initialization Failed ---");
                }

                return;
            }
        }
        else if (subMode == 3)  //Missile successfully detached and currently initializing
        {
            if (missileDetachPortType == 3 || missileDetachPortType == 4)
            {
                DetachLockedConnectors();
            }

            if (notMissile != null)
            {
                notMissileRadius = ComputeBlockGridDiagonalVector(notMissile).Length() / 2.0;
            }

            if (!InitMissileBlocks())
            {
                throw new Exception("--- Initialization Failed ---");
            }
        }

        if (missileLaunchType == 99)
        {
            Runtime.UpdateFrequency = UpdateFrequency.None;
            mode = 99;
        }
        else
        {
            nextLidarTriggerTicks = LIDAR_REFRESH_INTERVAL;

            if (waitForHomingTrigger)
            {
                subCounter = int.MaxValue;
            }
            else
            {
                if (missileLaunchType == 6)
                {
                    refFwdSet = true;
                }

                subCounter = (int)(launchSeconds * SECOND);
            }

            FireThrusters(verticalTakeoff ? launchThrusters : thrusters, true);

            mode = -1;
        }

        if (IsNotEmpty(missileActivationCommands))
        {
            ExecuteTriggerCommand(missileActivationCommands);
        }

        isLidarMode = "0,1,2,5,7,8,9,10".Contains(missileLaunchType.ToString());

        subMode = 0;
        clock = 0;

        init = true;
        return;
    }

    //---------- Modes And Controls ----------

    if (arguments.Length > 0)
    {
        ProcessCommunicationMessage(arguments);
    }

    if (enableMissileCommand)
    {
        ProcessMissileCommand(shipRefPanel.CustomData);
    }

    if ((updateSource & UpdateType.Update1) == 0)
    {
        return;
    }

    clock++;

    CalculateParameters();

    if (mode == -1)  //Launching
    {
        if (waitForHomingTrigger)
        {
            if (homingReleaseLock)
            {
                subCounter = 0;
            }
        }

        if (subCounter > 0)
        {
            subCounter -= 1;
        }
        else
        {
            if (verticalTakeoff)
            {
                FireThrusters(launchThrusters, false);
                FireThrusters(thrusters, true);
            }

            gyroControl.SetGyroOverride(true);

            if (spinAmount > 0)
            {
                gyroControl.SetGyroRoll(spinAmount);
            }

            distToTarget = 1000000;

            if (missileLaunchType == 3 || missileLaunchType == 4 || missileLaunchType == 6)
            {
                if (IsNotEmpty(missileTriggerCommands))
                {
                    ExecuteTriggerCommand(missileTriggerCommands);
                }
            }

            lastTargetPosition = targetPosition = ((refWorldMatrix.Forward * 1000000) + midPoint);

            subCounter = 0;
            subMode = 0;
            mode = missileLaunchType;
        }
    }
    else if (mode == 0)  //Lidar Homing With Initial Shipborne Lidar Lock-On
    {
        if (subMode == 0)  //Initial Lock-On
        {
            if (nextLidarTriggerTicks <= clock || commsLidarTargetSet)
            {
                bool targetFound = false;

                if (commsLidarTargetSet)
                {
                    commsLidarTargetSet = false;

                    targetFound = CheckAndSetValidLidarTarget(ref commsLidarTarget, ref refWorldMatrix);
                }
                else if (shipRefFwd != null)
                {
                    targetFound = PerformPreLock(shipRefLidars, shipRefFwd, fwdIsTurret);
                }

                if (targetFound)
                {
                    targetFound = false;

                    double overshootDistance = targetRadius / 2;
                    Vector3D lidarPosition = (useOffsetRaycasting ? lidarTargetInfo.HitPosition.Value : lidarTargetInfo.Position);

                    IMyCameraBlock syncLidar = GetLidarAndRecountTicks(missileLidars, ref lidarPosition, overshootDistance, lidarStaggerIndex++, ref refWorldMatrix);
                    if (syncLidar != null)
                    {
                        lidarPosition += (Vector3D.Normalize(lidarPosition - syncLidar.GetPosition()) * overshootDistance);

                        MyDetectedEntityInfo entityInfo = syncLidar.Raycast(lidarPosition);
                        if (!entityInfo.IsEmpty())
                        {
                            if (entityInfo.EntityId == lidarTargetInfo.EntityId)
                            {
                                targetFound = true;
                            }
                        }

                        if (!targetFound && !useOffsetRaycasting)
                        {
                            lidarPosition = lidarTargetInfo.HitPosition.Value;

                            syncLidar = GetLidarAndRecountTicks(missileLidars, ref lidarPosition, overshootDistance, lidarStaggerIndex++, ref refWorldMatrix);
                            if (syncLidar != null)
                            {
                                lidarPosition += (Vector3D.Normalize(lidarPosition - syncLidar.GetPosition()) * overshootDistance);

                                entityInfo = syncLidar.Raycast(lidarPosition);
                                if (!entityInfo.IsEmpty())
                                {
                                    if (entityInfo.EntityId == lidarTargetInfo.EntityId)
                                    {
                                        useOffsetRaycasting = true;
                                        targetFound = true;
                                    }
                                }
                            }
                        }
                    }

                    if (targetFound)
                    {
                        TransitToFullLock();
                    }
                }
            }

            if (targetPositionSet)
            {
                targetPosition = lidarTargetInfo.Position + (lidarTargetInfo.Velocity / SECOND * (clock - lastTargetPositionClock));

                if (useOffsetTargeting)
                {
                    targetPosition += Vector3D.TransformNormal(offsetTargetPosition, lidarTargetInfo.Orientation);
                }

                if (boolLeadTarget == true)
                {
                    CalculateLeadParameters();
                }
            }
        }
        else if (subMode == 1)  //Lidar Homing
        {
            PerformLidarLogic();

            if (boolLeadTarget == true)
            {
                CalculateLeadParameters();
            }
        }

        PerformCommonOperations();
    }
    else if (mode == 1)  //Lidar Homing With Initial Shipborne GPS Coordinates Lock-On
    {
        if (subMode == 0)  //Initial Lock-On
        {
            if (commsPositionSet)
            {
                commsPositionSet = false;

                targetPosition = commsPosition;
                targetPositionSet = true;
            }
            else
            {
                Vector3D parsedVector;
                if (shipRefPanel != null && ParseCoordinates(shipRefPanel.GetPublicTitle(), out parsedVector))
                {
                    targetPosition = parsedVector;
                    targetPositionSet = true;
                }
                else
                {
                    lastTargetPosition = targetPosition = GetFlyStraightVector();
                    targetPositionSet = false;
                }
            }

            if (targetPositionSet && (GetMissileMidPoint() - targetPosition).Length() < 1)
            {
                lastTargetPosition = targetPosition = GetFlyStraightVector();
                targetPositionSet = false;
            }

            if (targetPositionSet)
            {
                if (nextLidarTriggerTicks <= clock)
                {
                    double overshootDistance = targetRadius / 2;

                    IMyCameraBlock syncLidar = GetLidarAndRecountTicks(missileLidars, ref targetPosition, overshootDistance, lidarStaggerIndex++, ref refWorldMatrix);
                    if (syncLidar != null)
                    {
                        Vector3D testTargetPosition = targetPosition + (Vector3D.Normalize(targetPosition - syncLidar.GetPosition()) * overshootDistance);

                        MyDetectedEntityInfo entityInfo = syncLidar.Raycast(testTargetPosition);
                        if (!entityInfo.IsEmpty())
                        {
                            if (CheckAndSetValidLidarTarget(ref entityInfo, ref refWorldMatrix))
                            {
                                TransitToFullLock();
                            }
                        }
                    }
                }
            }

            if (boolLeadTarget == true)
            {
                CalculateTargetInfo();
                CalculateLeadParameters();
            }
        }
        else if (subMode == 1)  //Lidar Homing
        {
            PerformLidarLogic();

            if (boolLeadTarget == true)
            {
                CalculateLeadParameters();
            }
        }

        PerformCommonOperations();
    }
    else if (mode == 2)  //Lidar Homing With Initial Shipborne Camera Guidance
    {
        if (subMode == 0)  //Initial Camera Guidance
        {
            if (commsFwdSet)
            {
                commsFwdSet = false;

                refFwdPosition = commsFwd.Position;
                refFwdVector = commsFwd.Direction;
            }
            else if (shipRefFwd != null)
            {
                refFwdPosition = shipRefFwd.WorldMatrix.Translation;
                refFwdVector = (fwdIsTurret ? CalculateTurretViewVector(shipRefFwd as IMyLargeTurretBase) : shipRefFwd.WorldMatrix.Forward);
            }

            SetCameraGuidedTargetPosition();

            if (nextLidarTriggerTicks <= clock)
            {
                Vector3D shipRefTargetPosition = refFwdPosition + (refFwdVector * LIDAR_MAX_LOCK_DISTANCE);

                if (PerformPreLock(missileLidars, ref refWorldMatrix, ref refFwdVector, ref shipRefTargetPosition))
                {
                    TransitToFullLock();
                }
            }
        }
        else if (subMode == 1)  //Lidar Homing
        {
            PerformLidarLogic();

            if (boolLeadTarget == true)
            {
                CalculateLeadParameters();
            }
        }

        PerformCommonOperations();
    }
    else if (mode == 3)  //Camera Guided Mode
    {
        if (commsFwdSet)
        {
            commsFwdSet = false;

            refFwdPosition = commsFwd.Position;
            refFwdVector = commsFwd.Direction;
        }
        else if (shipRefFwd != null)
        {
            refFwdPosition = shipRefFwd.WorldMatrix.Translation;
            refFwdVector = (fwdIsTurret ? CalculateTurretViewVector(shipRefFwd as IMyLargeTurretBase) : shipRefFwd.WorldMatrix.Forward);
        }

        SetCameraGuidedTargetPosition();

        PerformCommonOperations();
    }
    else if (mode == 4)  //Cruise Mode
    {
        if (commsPositionSet)
        {
            commsPositionSet = false;

            targetPosition = commsPosition;
            targetPositionSet = true;
        }
        else if (shipRefPanel != null)
        {
            Vector3D parsedVector;
            if (ParseCoordinates(shipRefPanel.GetPublicTitle(), out parsedVector))
            {
                targetPosition = parsedVector;
                targetPositionSet = true;
            }
        }

        if (boolLeadTarget == true)
        {
            CalculateTargetInfo();
            CalculateLeadParameters();
        }

        PerformCommonOperations();
    }
    else if (mode == 6)  //Fixed Glide Mode
    {
        if (!refFwdSet)
        {
            if (commsFwdSet)
            {
                commsFwdSet = false;

                refFwdPosition = commsFwd.Position;
                refFwdVector = commsFwd.Direction;
                refFwdSet = true;
            }
            else if (shipRefFwd != null)
            {
                refFwdPosition = shipRefFwd.WorldMatrix.Translation;
                refFwdVector = (fwdIsTurret ? CalculateTurretViewVector(shipRefFwd as IMyLargeTurretBase) : shipRefFwd.WorldMatrix.Forward);
                refFwdSet = true;
            }
        }

        SetCameraGuidedTargetPosition();

        PerformCommonOperations();
    }
    else if (mode == 7)  //Lidar Homing With Shipborne Lidar Lock-On (Semi-Active Style Guidance)
    {
        if (subMode == 0)  //Initial Lock-On
        {
            if (nextLidarTriggerTicks <= clock || commsLidarTargetSet)
            {
                if (commsLidarTargetSet)
                {
                    commsLidarTargetSet = false;

                    if (CheckAndSetValidLidarTarget(ref commsLidarTarget, ref refWorldMatrix))
                    {
                        TransitToFullLock();
                    }
                }
                else if (shipRefFwd != null)
                {
                    if (PerformPreLock(shipRefLidars, shipRefFwd, fwdIsTurret))
                    {
                        TransitToFullLock();
                    }
                }
            }

            if (targetPositionSet)
            {
                targetPosition = lidarTargetInfo.Position + (lidarTargetInfo.Velocity / SECOND * (clock - lastTargetPositionClock));

                if (useOffsetTargeting)
                {
                    targetPosition += Vector3D.TransformNormal(offsetTargetPosition, lidarTargetInfo.Orientation);
                }

                if (boolLeadTarget == true)
                {
                    CalculateLeadParameters();
                }
            }
        }
        else if (subMode == 1)  //Lidar Homing
        {
            targetPosition = lidarTargetInfo.Position + (lidarTargetInfo.Velocity / SECOND * (clock - lastTargetPositionClock));

            if (nextLidarTriggerTicks <= clock || commsLidarTargetSet)
            {
                bool targetFound = false;
                double overshootDistance = targetRadius / 2;

                if (commsLidarTargetSet)
                {
                    commsLidarTargetSet = false;

                    CheckAndUpdateLidarTarget(ref commsLidarTarget, ref targetFound);
                }
                else if (shipRefFwd != null)
                {
                    MatrixD shipRefWorldMatrix = shipRefFwd.WorldMatrix;
                    Vector3D lidarPosition = (useOffsetRaycasting ? targetPosition + Vector3D.TransformNormal(offsetRaycastPosition, lidarTargetInfo.Orientation) : targetPosition);

                    IMyCameraBlock aimLidar = GetLidarAndRecountTicks(shipRefLidars, ref lidarPosition, overshootDistance, lidarStaggerIndex++, ref shipRefWorldMatrix);
                    if (aimLidar != null)
                    {
                        lidarPosition += (Vector3D.Normalize(lidarPosition - aimLidar.GetPosition()) * overshootDistance);

                        MyDetectedEntityInfo entityInfo = aimLidar.Raycast(lidarPosition);
                        if (!entityInfo.IsEmpty())
                        {
                            CheckAndUpdateLidarTarget(ref entityInfo, ref targetFound);
                        }

                        if (!targetFound && !useOffsetRaycasting)
                        {
                            lidarPosition = targetPosition + Vector3D.TransformNormal(offsetRaycastPosition, lidarTargetInfo.Orientation);

                            aimLidar = GetLidarAndRecountTicks(shipRefLidars, ref lidarPosition, overshootDistance, lidarStaggerIndex++, ref shipRefWorldMatrix);
                            if (aimLidar != null)
                            {
					            lidarPosition += (Vector3D.Normalize(lidarPosition - aimLidar.GetPosition()) * overshootDistance);

					            entityInfo = aimLidar.Raycast(lidarPosition);
                                if (!entityInfo.IsEmpty())
                                {
                                    CheckAndUpdateLidarTarget(ref entityInfo, ref targetFound);
                                    if (targetFound)
                                    {
                                        useOffsetRaycasting = true;
                                    }
                                }
                            }
                        }
                    }
                }

                targetPositionSet = targetFound;
            }

            if (useOffsetTargeting)
            {
                targetPosition += Vector3D.TransformNormal(offsetTargetPosition, lidarTargetInfo.Orientation);
            }

            if (boolLeadTarget == true)
            {
                CalculateLeadParameters();
            }
        }

        PerformCommonOperations();
    }
    else if (mode == 8 || mode == 9 || mode == 5)  //Turret AI Homing
    {
        if (subMode == 0)  //Initialization
        {
            if (mode == 5)
            {
                if (shipRefTurrets.Count > 0)
                {
                    homingTurret = shipRefTurrets[0];

                    subCounter = 0;
                    subMode = 1;
                }
            }
            else if (homingTurret != null)
            {
                homingTurret.EnableIdleRotation = false;
                homingTurret.SetValueFloat("Range", homingTurret.GetMaximum<float>("Range"));
                homingTurret.Enabled = true;

                if (refFwdVector.Sum == 0)
                {
                    targetPosition = refWorldMatrix.Translation + (refWorldMatrix.Forward * 1000000);
                }
                else
                {
                    targetPosition = refFwdPosition + (refFwdVector * 1000000);
                }

                subCounter = 0;
                subMode = 1;
            }
        }
        else if (subMode == 1)  //Seeking For Target
        {
            if (mode == 5)
            {
                for (int i = 0; i < shipRefTurrets.Count; i++)
                {
                    if (shipRefTurrets[i].HasTarget)
                    {
                        homingTurret = shipRefTurrets[i];
                        break;
                    }
                }
            }

            if (homingTurret.HasTarget)
            {
                SetHomingTurretLidarTarget();

                offsetTargetPosition = offsetRaycastPosition = Vector3D.Zero;

                if (IsNotEmpty(missileTriggerCommands))
                {
                    ExecuteTriggerCommand(missileTriggerCommands);
                }

                subCounter = 0;
                subMode = 2;

                if (boolLeadTarget == true)
                {
                    CalculateLeadParameters();
                }
            }
            else if (mode == 9)
            {
                if (commsFwdSet)
                {
                    commsFwdSet = false;

                    refFwdPosition = commsFwd.Position;
                    refFwdVector = commsFwd.Direction;
                }
                else if (shipRefFwd != null)
                {
                    refFwdPosition = shipRefFwd.WorldMatrix.Translation;
                    refFwdVector = (fwdIsTurret ? CalculateTurretViewVector(shipRefFwd as IMyLargeTurretBase) : shipRefFwd.WorldMatrix.Forward);
                }

                SetCameraGuidedTargetPosition();
            }
        }
        else if (subMode == 2)  //Turret Target Locked
        {
            if (switchTurretToRaycast)
            {
                PerformLidarLogic();

                if (!targetPositionSet && homingTurret.HasTarget)
                {
                    SetHomingTurretLidarTarget();
                }
            }
            else
            {
                if (homingTurret.HasTarget)
                {
                    SetHomingTurretLidarTarget();
                }
                else
                {
                    targetPosition = lidarTargetInfo.Position + (lidarTargetInfo.Velocity / SECOND * (clock - lastTargetPositionClock));
                }
            }

            if (boolLeadTarget == true)
            {
                CalculateLeadParameters();
            }
        }

        PerformCommonOperations();
    }

    if (statusDisplay != null)
    {
        if (mode == -2)
        {
            DisplayStatus("Idle");
        }
        else if (mode == -1)
        {
            DisplayStatus("Launching");
        }
        else if (mode == 0 || mode == 1 || mode == 7)
        {
            if (subMode == 0)
            {
                DisplayStatus("Initial Lock");
            }
            else if (subMode == 1)
            {
                DisplayStatus((targetPositionSet ? "Lock" : "Trace") + ": [" + Math.Round(targetPosition.GetDim(0), 2) + "," + Math.Round(targetPosition.GetDim(1), 2) + "," + Math.Round(targetPosition.GetDim(2), 2) + "]");
            }
            else
            {
                DisplayStatus("-");
            }
        }
        else if (mode == 2)
        {
            if (subMode == 0)
            {
                DisplayStatus("Initial Camera Lock");
            }
            else if (subMode == 1)
            {
                DisplayStatus((targetPositionSet ? "Lock" : "Trace") + ": [" + Math.Round(targetPosition.GetDim(0), 2) + "," + Math.Round(targetPosition.GetDim(1), 2) + "," + Math.Round(targetPosition.GetDim(2), 2) + "]");
            }
            else
            {
                DisplayStatus("-");
            }
        }
        else if (mode == 3)
        {
            DisplayStatus("Camera");
        }
        else if (mode == 4)
        {
            DisplayStatus("Cruise: [" + Math.Round(targetPosition.GetDim(0), 2) + "," + Math.Round(targetPosition.GetDim(1), 2) + "," + Math.Round(targetPosition.GetDim(2), 2) + "]");
        }
        else if (mode == 6)
        {
            DisplayStatus("Fixed Glide");
        }
        else if (mode == 5 || mode == 8 || mode == 9)
        {
            if (subMode == 2)
            {
                DisplayStatus("Turret Locked");
            }
            else if (subMode == 0 || subMode == 1)
            {
                DisplayStatus("Initial Lock");
            }
            else
            {
                DisplayStatus("-");
            }
        }
        else
        {
            DisplayStatus("-");
        }
    }

    if (outputMissileStatus)
    {
        string statusCode;
        switch (mode)
        {
        case -2:
            statusCode = "-";
            break;
        case -1:
            statusCode = (waitForHomingTrigger ? "W" : (subCounter > 0 ? "F" : "K"));
            break;
        case 0:
        case 1:
        case 5:
        case 7:
            statusCode = (subMode == 0 ? "K" : (targetPositionSet ? "L" : "T"));
            break;
        case 2:
            statusCode = (subMode == 0 ? "C" : (targetPositionSet ? "L" : "T"));
            break;
        case 3:
            statusCode = "C";
            break;
        case 4:
            statusCode = "D";
            break;
        case 6:
            statusCode = "G";
            break;
        case 8:
            statusCode = (subMode == 2 ? "U" : "K");
            break;
        default:
            statusCode = "-";
            break;
        }
        Echo("ST:" + mode + ":" + subMode + ":" + (waitForHomingTrigger ? 0 : subCounter) + ":" + clock + ":" + statusCode + ":" +
            Math.Round(targetPosition.GetDim(0), 5) + ":" + Math.Round(targetPosition.GetDim(1), 5) + ":" + Math.Round(targetPosition.GetDim(2), 5) + ":" +
            Math.Round(targetRadius, 5) + ":");
    }
}

//------------------------------ Miscellaneous Methods ------------------------------

void DisplayStatus(string statusMsg)
{
    if (statusDisplay != null)
    {
        statusDisplay.CustomName = strStatusDisplayPrefix + " Mode: " + mode + ", " + statusMsg;
    }
}

void TriggerLockAlert()
{
    if (alertBlock != null)
    {
        if (alertBlock.HasAction(strLockTriggerAction))
        {
            alertBlock.ApplyAction(strLockTriggerAction);
        }
    }
}

Vector3D GetMissileMidPoint()
{
    return (Me.CubeGrid.GridIntegerToWorld(Me.CubeGrid.Min) + Me.CubeGrid.GridIntegerToWorld(Me.CubeGrid.Max)) / 2;
}

Vector3D GetFlyStraightVector()
{
    return (shipVelocity * 1000000) + midPoint;
}

bool IsNotEmpty(string line)
{
    return (line != null && line.Length > 0);
}

bool NameContains(IMyTerminalBlock block, string nameTag)
{
    return (block.CustomName.IndexOf(nameTag, StringComparison.OrdinalIgnoreCase) > -1);
}

//------------------------------ Missile And Target Information Methods ------------------------------

void CalculateParameters()
{
    //---------- Calculate Missile Related Variables ----------

    refWorldMatrix = refFwdBlock.WorldMatrix;
    if (refFwdReverse)
    {
        refWorldMatrix.Forward = refWorldMatrix.Backward;
        refWorldMatrix.Left = refWorldMatrix.Right;
    }
    refViewMatrixUnclean4thCol = MatrixD.Transpose(refWorldMatrix);

    if (remoteControl != null)
    {
        MyShipVelocities velocity = remoteControl.GetShipVelocities();

        midPoint = remoteControl.CenterOfMass;
        shipVelocity = velocity.LinearVelocity;
        speed = shipVelocity.Length();

        naturalGravity = remoteControl.GetNaturalGravity();
        naturalGravityLength = naturalGravity.Length();
        naturalGravity = (naturalGravityLength > 0 ? naturalGravity / naturalGravityLength : Vector3D.Zero);

        rpm = velocity.AngularVelocity.Dot(refWorldMatrix.Forward) * RPM_FACTOR;
    }
    else
    {
        midPoint = GetMissileMidPoint();
        shipVelocity = midPoint - lastMidPoint;
        speed = shipVelocity.Length() * SECOND;
        lastMidPoint = midPoint;

        naturalGravity = shipVelocity;
        naturalGravityLength = naturalGravity.Length();
        naturalGravity = (naturalGravityLength > 0 ? naturalGravity / naturalGravityLength : Vector3D.Zero);

        rpm = Math.Acos(lastNormal.Dot(refWorldMatrix.Up)) * RPM_FACTOR;
        lastNormal = refWorldMatrix.Up;
    }
}

void CalculateTargetInfo()
{
    if (targetPositionSet)
    {
        targetDirection = targetPosition - lastTargetPosition;
        targetSpeed = targetDirection.Length();

        if (targetSpeed > 0)
        {
            targetDirection = targetDirection / targetSpeed;
            targetSpeed = targetSpeed * SECOND / (clock - lastTargetPositionClock);
        }

        lastTargetPosition = targetPosition;
        lastTargetPositionClock = clock;

        targetPositionSet = false;
    }
    else
    {
        targetPosition = lastTargetPosition + ((targetDirection * targetSpeed) / SECOND * (clock - lastTargetPositionClock));
    }
}

void CalculateLeadParameters()
{
    if (targetSpeed > 0)
    {
        ComputeInterceptPoint(ref targetDirection, ref targetPosition, targetSpeed, ref midPoint, Math.Max(speed, targetSpeed * 0.5), out targetPosition, ref lastTimeToIntercept);
    }
}

void CalculateTargetParameters()
{
    //---------- Calculate Target Parameters ----------

    targetVector = targetPosition - midPoint;
    distToTarget = targetVector.Length();
    targetVector = targetVector / distToTarget;

    if (boolDrift == true)
    {
        Vector3D sideDrift = Vector3D.Reject(shipVelocity, targetVector);
        double sideSpeed = sideDrift.Length();
        if (sideSpeed >= 0.0001)
        {
            sideDrift = sideDrift / sideSpeed;

            double acceleration = CalculateThrustAcceleration();
            if (refDwdBlock != null && naturalGravityLength > 0.01)
            {
                acceleration -= Math.Max(sideDrift.Dot(naturalGravity) * naturalGravityLength, 0);
                if (acceleration < MIN_THRUST_ACCELERATION)
                {
                    acceleration = MIN_THRUST_ACCELERATION;
                }
            }
            double dotProduct = Math.Min((sideSpeed * speed * 2) / (acceleration * distToTarget), 0.999);

            targetVector -= (sideDrift * dotProduct) / Math.Sqrt(1 - (dotProduct * dotProduct));
            targetVector.Normalize();
        }
    }

    targetVector = Vector3D.TransformNormal(targetVector, refViewMatrixUnclean4thCol);
    targetVector.Normalize();

    if (Double.IsNaN(targetVector.Sum))
    {
        targetVector = Vector3D.Forward;
    }
}

double CalculateThrustAcceleration()
{
    if (nextAccelRecountTicks <= clock)
    {
        nextAccelRecountTicks = clock + MIN_ACCEL_RECOUNT_TICKS;

        if (remoteControl == null)
        {
            thrustAcceleration = MIN_THRUST_ACCELERATION;
        }
        else
        {
            float totalThrust = 0;
            for (int i = 0; i < thrusters.Count; i++)
            {
                totalThrust += thrusters[i].MaxEffectiveThrust;
            }

            thrustAcceleration = totalThrust / remoteControl.CalculateShipMass().TotalMass;
            if (double.IsNaN(thrustAcceleration) || double.IsInfinity(thrustAcceleration) || thrustAcceleration < MIN_THRUST_ACCELERATION)
            {
                thrustAcceleration = MIN_THRUST_ACCELERATION;
            }
        }
    }

    return thrustAcceleration;
}

Vector3D CalculateTurretViewVector(IMyLargeTurretBase turret)
{
    Vector3D direction;
    Vector3D.CreateFromAzimuthAndElevation(turret.Azimuth, turret.Elevation, out direction);

    return Vector3D.TransformNormal(direction, turret.WorldMatrix);
}

Vector3D[] SpreadRaycastPoint(ref Vector3D targetPoint, ref Vector3D forwardVector, float distance, int lidarCount)
{
    Vector3D[] refPoints;

    if (lidarCount > 1)
    {
        if (raycastAheadCount > 0)
        {
            lidarAimChangeVector = (forwardVector * distance) - lidarLastForward;
            lidarAimChangeVector = lidarAimChangeVector / (clock - lidarLastForwardClock) * SECOND * raycastAheadSeconds / raycastAheadCount;
            lidarLastForward = (forwardVector * distance);
            lidarLastForwardClock = clock;

            refPoints = new Vector3D[lidarCount];
            for (int i = 0; i < lidarCount; i++)
            {
                refPoints[i] = targetPoint + (lidarAimChangeVector * i);
            }
        }
        else
        {
            MatrixD matrix = MatrixD.CreateFromDir(forwardVector);
            refPoints = new Vector3D[5] { targetPoint,
                                            targetPoint + (matrix.Up * fivePointInitialLockDist),
                                            targetPoint + (matrix.Up * -fivePointInitialLockDist),
                                            targetPoint + (matrix.Left * fivePointInitialLockDist),
                                            targetPoint + (matrix.Left * -fivePointInitialLockDist) };
        }
    }
    else
    {
        refPoints = new Vector3D[1] { targetPoint };
    }

    return refPoints;
}

//------------------------------ Missile Lock-On And Leading Methods ------------------------------

void SetCameraGuidedTargetPosition()
{
    Vector3D shipToMissileVector = midPoint - refFwdPosition;
    Vector3D missileToViewLineVector = -Vector3D.Reject(shipToMissileVector, refFwdVector);

    double extraDistanceExtend = (speed * 2) + (shipToMissileVector + missileToViewLineVector).Length();

    if (refDwdBlock == null && naturalGravityLength > 0.1)
    {
        missileToViewLineVector -= Vector3D.Reject(missileToViewLineVector, naturalGravity);
        if (missileToViewLineVector.Dot(naturalGravity) < 0)
        {
	        targetPosition = refFwdPosition + (refFwdVector * extraDistanceExtend) - (naturalGravity * naturalGravityLength);
        }
        else
        {
	        targetPosition = refFwdPosition + (refFwdVector * extraDistanceExtend);
        }
    }
    else
    {
        targetPosition = refFwdPosition + (refFwdVector * extraDistanceExtend);
    }

    targetPositionSet = true;
}

bool PerformPreLock(List<IMyCameraBlock> lidars, IMyTerminalBlock refFwd, bool isTurret)
{
    MatrixD worldMatrix = refFwd.WorldMatrix;
    Vector3D fwdVector = (isTurret ? CalculateTurretViewVector(refFwd as IMyLargeTurretBase) : worldMatrix.Forward);
    Vector3D targetPoint = worldMatrix.Translation + (fwdVector * LIDAR_MAX_LOCK_DISTANCE);

    return PerformPreLock(lidars, ref worldMatrix, ref fwdVector, ref targetPoint);
}
bool PerformPreLock(List<IMyCameraBlock> lidars, ref MatrixD worldMatrix, ref Vector3D fwdVector, ref Vector3D targetPoint)
{
    if (lidarTargetInfo.IsEmpty())
    {
        int lidarCount = raycastAheadCount > 0 ? raycastAheadCount + 1 : (fivePointInitialLockDist > 0 ? 5 : 1);
        List<IMyCameraBlock> aimLidars = GetLidarsAndRecountTicks(lidars, ref targetPoint, 0, lidarStaggerIndex++, lidarCount, ref worldMatrix);
        if (aimLidars.Count > 0)
        {
            if (aimLidars.Count < lidarCount)
            {
                lidarCount = 1;
            }
            Vector3D[] refPoints = SpreadRaycastPoint(ref targetPoint, ref fwdVector, LIDAR_MAX_LOCK_DISTANCE, lidarCount);

            for (int i = 0; i < lidarCount; i++)
            {
                MyDetectedEntityInfo entityInfo = aimLidars[i].Raycast(refPoints[i]);
                if (!entityInfo.IsEmpty())
                {
                    if (CheckAndSetValidLidarTarget(ref entityInfo, ref worldMatrix))
                    {
                        return true;
                    }
                }
            }
        }
    }
    else
    {
        targetPosition = lidarTargetInfo.Position + (lidarTargetInfo.Velocity / SECOND * (clock - lastTargetPositionClock));
        double overshootDistance = targetRadius / 2;

        IMyCameraBlock aimLidar = GetLidarAndRecountTicks(lidars, ref targetPosition, overshootDistance, lidarStaggerIndex++, ref worldMatrix);
        if (aimLidar != null)
        {
            MyDetectedEntityInfo entityInfo = aimLidar.Raycast(targetPoint);
            if (!entityInfo.IsEmpty())
            {
                if (CheckAndSetValidLidarTarget(ref entityInfo, ref worldMatrix))
                {
                    return true;
                }
                else
                {
                    lidarTargetInfo = new MyDetectedEntityInfo();
                    targetPositionSet = false;
                }
            }
        }
    }
    return false;
}

void PerformLidarLogic()
{
    targetPosition = lidarTargetInfo.Position + (lidarTargetInfo.Velocity / SECOND * (clock - lastTargetPositionClock));

    if (nextLidarTriggerTicks <= clock)
    {
        bool targetFound = false;
        double overshootDistance = targetRadius / 2;

        Vector3D lidarPosition = (useOffsetRaycasting ? targetPosition + Vector3D.TransformNormal(offsetRaycastPosition, lidarTargetInfo.Orientation) : targetPosition);

        IMyCameraBlock aimLidar = GetLidarAndRecountTicks(missileLidars, ref lidarPosition, overshootDistance, lidarStaggerIndex++, ref refWorldMatrix);
        if (aimLidar != null)
        {
            lidarPosition += (Vector3D.Normalize(lidarPosition - aimLidar.GetPosition()) * overshootDistance);

            MyDetectedEntityInfo entityInfo = aimLidar.Raycast(lidarPosition);
            if (!entityInfo.IsEmpty())
            {
                CheckAndUpdateLidarTarget(ref entityInfo, ref targetFound);
            }

            if (!targetFound && !useOffsetRaycasting)
            {
                lidarPosition = targetPosition + Vector3D.TransformNormal(offsetRaycastPosition, lidarTargetInfo.Orientation);

                aimLidar = GetLidarAndRecountTicks(missileLidars, ref lidarPosition, overshootDistance, lidarStaggerIndex++, ref refWorldMatrix);
                if (aimLidar != null)
                {
					lidarPosition += (Vector3D.Normalize(lidarPosition - aimLidar.GetPosition()) * overshootDistance);

					entityInfo = aimLidar.Raycast(lidarPosition);
                    if (!entityInfo.IsEmpty())
                    {
                        CheckAndUpdateLidarTarget(ref entityInfo, ref targetFound);
                        if (targetFound)
                        {
                            useOffsetRaycasting = true;
                        }
                    }
                }
            }
        }

        targetPositionSet = targetFound;
    }

    if (useOffsetTargeting)
    {
        targetPosition += Vector3D.TransformNormal(offsetTargetPosition, lidarTargetInfo.Orientation);
    }
}

bool CheckAndSetValidLidarTarget(ref MyDetectedEntityInfo entityInfo, ref MatrixD shipRefWorldMatrix)
{
    if (IsValidLidarTarget(ref entityInfo, ref shipRefWorldMatrix))
    {
        distToTarget = Vector3D.Distance(entityInfo.Position, refWorldMatrix.Translation);
        targetSpeed = entityInfo.Velocity.Length();
        targetDirection = (targetSpeed > 0 ? new Vector3D(entityInfo.Velocity) / targetSpeed : new Vector3D());
        targetRadius = Vector3D.Distance(entityInfo.BoundingBox.Min, entityInfo.BoundingBox.Max);

        if (entityInfo.HitPosition.HasValue)
        {
            offsetTargetPosition = offsetRaycastPosition = Vector3D.TransformNormal(entityInfo.HitPosition.Value - entityInfo.Position, MatrixD.Transpose(entityInfo.Orientation));
        }
        else
        {
            offsetTargetPosition = offsetRaycastPosition = Vector3D.Zero;
        }

        lidarTargetInfo = entityInfo;
        lastTargetPositionClock = clock;

        targetPositionSet  = true;

        return true;
    }
    else
    {
        return false;
    }
}

void CheckAndUpdateLidarTarget(ref MyDetectedEntityInfo entityInfo, ref bool targetFound)
{
    if (entityInfo.EntityId == lidarTargetInfo.EntityId)
    {
        distToTarget = Vector3D.Distance(entityInfo.Position, refWorldMatrix.Translation);
        targetSpeed = entityInfo.Velocity.Length();
        targetDirection = (targetSpeed > 0 ? new Vector3D(entityInfo.Velocity) / targetSpeed : new Vector3D());
        targetRadius = Vector3D.Distance(entityInfo.BoundingBox.Min, entityInfo.BoundingBox.Max);

        if (useOffsetRaycasting && entityInfo.HitPosition.HasValue)
        {
            offsetRaycastPosition = Vector3D.TransformNormal(entityInfo.HitPosition.Value - entityInfo.Position, MatrixD.Transpose(entityInfo.Orientation));
        }

        lidarTargetInfo = entityInfo;
        lastTargetPositionClock = clock;

        targetPosition = entityInfo.Position;
        targetFound = true;
    }
}

void SetHomingTurretLidarTarget()
{
    MyDetectedEntityInfo entityInfo = homingTurret.GetTargetedEntity();

    distToTarget = Vector3D.Distance(entityInfo.Position, refWorldMatrix.Translation);
    targetSpeed = entityInfo.Velocity.Length();
    targetDirection = (targetSpeed > 0 ? new Vector3D(entityInfo.Velocity) / targetSpeed : new Vector3D());

    lidarTargetInfo = entityInfo;
    lastTargetPositionClock = clock;

    targetPosition = entityInfo.Position;
    targetPositionSet = true;
}

void TransitToFullLock()
{
    TriggerLockAlert();

    if (IsNotEmpty(missileTriggerCommands))
    {
        ExecuteTriggerCommand(missileTriggerCommands);
    }

    subCounter = 0;
    subMode = 1;
}

void PerformCommonOperations()
{
    CalculateTargetParameters();
    AimAtTarget();

    if (adjustThrustBasedOnAim)
    {
        AdjustThrustBasedOnAim();
    }

    if (boolNaturalDampener == true)
    {
        AimAtNaturalGravity();
    }

    if (haveTriggerCommands)
    {
        ProcessTriggerCommands();
    }

    if (hasProximitySensors)
    {
        CheckProximitySensors();
    }
}

void ComputeInterceptPoint(ref Vector3D targetDirection, ref Vector3D targetLocation, double targetSpeed, ref Vector3D currentLocation, double currentSpeed, out Vector3D interceptPoint, ref double timeToIntercept)
{
    //---------- Calculate Impact Point ----------

    //targetDirection Must Be Normalized
    double a = (targetSpeed * targetSpeed) - (currentSpeed * currentSpeed);
    double b = (2 * targetDirection.Dot(targetLocation - currentLocation) * targetSpeed);
    double c = (targetLocation - currentLocation).LengthSquared();

    double t;

    if (a == 0)
    {
        t = -c / a;
    }
    else
    {
        //Use Formula To Find Root: t = ( -b +- sqrt(b^2 - 4ac) ) / 2a
        double u = (b * b) - (4 * a * c);
        if (u < 0)
        {
            t = -1;
        }
        else
        {
            u = Math.Sqrt(u);

            double t1 = (-b + u) / (2 * a);
            double t2 = (-b - u) / (2 * a);

            t = (t1 > 0 ? (t2 > 0 ? (t1 < t2 ? t1 : t2) : t1) : t2);
        }
    }

    if (t < 0)
    {
        t = Math.Max(Math.Sqrt(c) / currentSpeed, timeToIntercept);
    }

    interceptPoint = targetLocation + (targetDirection * targetSpeed * t);
    timeToIntercept = t;
}

List<IMyCameraBlock> GetAvailableLidars(List<IMyCameraBlock> lidars, ref Vector3D aimPoint, double overshootDistance, int indexOffset, int lidarCount)
{
    List<IMyCameraBlock> result = new List<IMyCameraBlock>(lidarCount);

    for (int i = 0; i < lidars.Count; i++)
    {
        IMyCameraBlock lidar = lidars[(i + indexOffset) % lidars.Count];

        Vector3D adjustedAimPoint;
        if (overshootDistance == 0)
        {
            adjustedAimPoint = aimPoint;
        }
        else
        {
            Vector3D aimVector = aimPoint - lidar.WorldMatrix.Translation;
            double distance = aimVector.Length();
            adjustedAimPoint = aimPoint + (aimVector / distance * overshootDistance);
        }

        if (lidar.IsWorking && lidar.CanScan(adjustedAimPoint))
        {
            result.Add(lidar);

            if (result.Count >= lidarCount) break;
        }
    }

    return result;
}

IMyCameraBlock GetLidarAndRecountTicks(List<IMyCameraBlock> lidars, ref Vector3D aimPoint, double overshootDistance, int indexOffset, ref MatrixD recountRefMatrix)
{
    List<IMyCameraBlock> result = GetLidarsAndRecountTicks(lidars, ref aimPoint, overshootDistance, indexOffset, 1, ref recountRefMatrix);
    return (result.Count > 0 ? result[0] : null);
}
List<IMyCameraBlock> GetLidarsAndRecountTicks(List<IMyCameraBlock> lidars, ref Vector3D aimPoint, double overshootDistance, int indexOffset, int count, ref MatrixD recountRefMatrix)
{
    List<IMyCameraBlock> result;

    if (nextLidarRecountTicks <= clock)
    {
        result = GetAvailableLidars(lidars, ref aimPoint, overshootDistance, indexOffset, lidars.Count);

        ticksFactor = ticksRatio / (Math.Max((float)result.Count / (count > 1 ? 2 : 1), 1) * LIDAR_REFRESH_CALC_FACTOR);
        nextLidarRecountTicks = clock + MIN_LIDAR_RECOUNT_TICKS;

        if (result.Count > count)
        {
            result = result.GetRange(0, count);
        }
    }
    else
    {
        result = GetAvailableLidars(lidars, ref aimPoint, overshootDistance, indexOffset, count);
    }

    if (result.Count >= (count > 1 ? 2 : 1))
    {
        RecountLidarTicks(Vector3D.Distance(aimPoint, recountRefMatrix.Translation) + overshootDistance, ticksFactor);
    }

    return result;
}

void RecountLidarTicks(double distance, double factor)
{
    if (LIDAR_REFRESH_INTERVAL == 0)
    {
        nextLidarTriggerTicks = clock + (int)Math.Ceiling(distance * factor);
    }
    else
    {
        nextLidarTriggerTicks = clock + LIDAR_REFRESH_INTERVAL;
    }
}

bool IsValidLidarTarget(ref MyDetectedEntityInfo entityInfo, ref MatrixD referenceWorldMatrix)
{
    if (entityInfo.Type == MyDetectedEntityType.LargeGrid || entityInfo.Type == MyDetectedEntityType.SmallGrid)
    {
        if (Vector3D.Distance(entityInfo.Position, referenceWorldMatrix.Translation) > LIDAR_MIN_LOCK_DISTANCE)
        {
            if (!excludeFriendly || IsNotFriendly(entityInfo.Relationship))
            {
                if (notMissile == null || (entityInfo.Position - ComputeBlockGridMidPoint(notMissile)).Length() > notMissileRadius)
                {
                    if ((entityInfo.Position - referenceWorldMatrix.Translation).Length() >= LIDAR_MIN_LOCK_DISTANCE && (GetMissileMidPoint() - entityInfo.Position).Length() >= 1)
                    {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

bool IsValidProximityTarget(ref MyDetectedEntityInfo detected)
{
    bool matchAll = (!isLidarMode && lidarTargetInfo.EntityId <= 0);

    if (detected.EntityId > 0 && (matchAll || lidarTargetInfo.EntityId == detected.EntityId))
    {
        if (!excludeFriendly || IsNotFriendly(detected.Relationship))
        {
            return true;
        }
    }
    return false;
}

bool IsNotFriendly(MyRelationsBetweenPlayerAndBlock relationship)
{
    return (relationship != MyRelationsBetweenPlayerAndBlock.FactionShare && relationship != MyRelationsBetweenPlayerAndBlock.Owner);
}

//------------------------------ Missile Aiming Methods ------------------------------

int GetMultiplierSign(double value)
{
    return (value < 0 ? -1 : 1);
}

void AimAtTarget()
{
    //---------- Activate Gyroscopes To Turn Towards Target ----------

    Vector3D yawVector = new Vector3D(targetVector.GetDim(0), 0, targetVector.GetDim(2));
    Vector3D pitchVector = new Vector3D(0, targetVector.GetDim(1), targetVector.GetDim(2));
    yawVector.Normalize();
    pitchVector.Normalize();

    //This is NOT True Azimuth and Elevation. Pitch can be more than 90 degrees for improved oversteer.
    double yawInput = Math.Acos(yawVector.Dot(Vector3D.Forward)) * GetMultiplierSign(targetVector.GetDim(0));
    double pitchInput = Math.Acos(pitchVector.Dot(Vector3D.Forward)) * GetMultiplierSign(targetVector.GetDim(1));

    //---------- PID Controller Adjustment ----------

    yawInput = yawController.Filter(yawInput, 2);
    pitchInput = pitchController.Filter(pitchInput, 2);

    if (Math.Abs(yawInput) + Math.Abs(pitchInput) > AIM_LIMIT)
    {
        double adjust = AIM_LIMIT / (Math.Abs(yawInput) + Math.Abs(pitchInput));
        yawInput *= adjust;
        pitchInput *= adjust;
    }

    //---------- Set Gyroscope Parameters ----------

    gyroControl.SetGyroYaw((float)yawInput);
    gyroControl.SetGyroPitch((float)pitchInput);
}

void AimAtNaturalGravity()
{
    //---------- Activate Gyroscopes To Aim Dampener At Natural Gravity ----------

    if (refDwdBlock == null || naturalGravityLength < 0.01)
    {
        return;
    }

    MatrixD dampenerLookAtMatrix = MatrixD.CreateLookAt(Vector3D.Zero, refDwdBlock.WorldMatrix.Forward, refWorldMatrix.Forward);

    Vector3D gravityVector = Vector3D.TransformNormal(naturalGravity, dampenerLookAtMatrix);
    gravityVector.SetDim(1, 0);
    gravityVector.Normalize();

    if (Double.IsNaN(gravityVector.Sum))
    {
        gravityVector = Vector3D.Forward;
    }

    double rollInput = Math.Acos(gravityVector.Dot(Vector3D.Forward)) * GetMultiplierSign(gravityVector.GetDim(0));

    //---------- PID Controller Adjustment ----------

    rollInput = rollController.Filter(rollInput, 2);

    //---------- Set Gyroscope Parameters ----------

    gyroControl.SetGyroRoll((float)rollInput);
}

void AdjustThrustBasedOnAim()
{
    double dot = Vector3D.Forward.Dot(targetVector);

    if (dot > 0.707)
    {
        if (adjustThrustState != 0)
        {
            FireThrusters(thrusters, true);
            adjustThrustState = 0;
        }
    }
    else if (dot <= 0)
    {
        if (adjustThrustState != 2)
        {
            FireThrusters(thrusters, false);
            adjustThrustState = 2;
        }
    }
    else
    {
        AdjustThrusters(thrusters, (float)(dot * 1.414));
        adjustThrustState = 1;
    }
}

//------------------------------ Missile Separation Methods ------------------------------

bool DetachFromGrid(bool testOnly = false)
{
    List<IMyTerminalBlock> blocks;

    switch (missileDetachPortType)
    {
        case 0:
        case 3:
            blocks = GetDetachBlocks<IMyShipMergeBlock>();
            detachBlock = GetClosestBlockFromReference(blocks, Me, true);

            if (!testOnly)
            {
                if (detachBlock == null)
                {
                    Echo("Error: Missing Merge Block" + (IsNotEmpty(strDetachPortTag) ? " with tag " + strDetachPortTag : "") + " to detach.");
                    return false;
                }
                detachBlockType = 0;

                ((IMyShipMergeBlock)detachBlock).Enabled = false;
            }
            return true;
        case 1:
        case 4:
            blocks = GetDetachBlocks<IMyMotorBase>();
            for (int i = 0; i < blocks.Count; i++)
            {
                IMyCubeGrid grid = ((IMyMotorBase)blocks[i]).TopGrid;
                if (grid != null && grid == Me.CubeGrid)
                {
                    detachBlock = blocks[i];
                    break;
                }
            }

            if (!testOnly)
            {
                if (detachBlock == null)
                {
                    Echo("Error: Missing Rotor" + (IsNotEmpty(strDetachPortTag) ? " with tag " + strDetachPortTag : "") + " to detach.");
                    return false;
                }
                detachBlockType = 1;

                ((IMyMotorBase)detachBlock).Detach();
            }
            return true;
        case 2:
            blocks = GetDetachBlocks<IMyShipConnector>();
            detachBlock = GetClosestBlockFromReference(blocks, Me, true);

            if (!testOnly)
            {
                if (detachBlock == null)
                {
                    Echo("Error: Missing Connector" + (IsNotEmpty(strDetachPortTag) ? " with tag " + strDetachPortTag : "") + " to detach.");
                    return false;
                }
                detachBlockType = 2;

                ((IMyShipConnector)detachBlock).Disconnect();
            }
            return true;
        case 99:
            return true;
        default:
            if (!testOnly)
            {
                Echo("Error: Unknown missileDetachPortType - " + missileDetachPortType + ".");
            }
            return false;
    }
}

List<IMyTerminalBlock> GetDetachBlocks<T>() where T: class, IMyTerminalBlock
{
    List<IMyTerminalBlock> blocks = new List<IMyTerminalBlock>();
    IMyBlockGroup group = (IsNotEmpty(strDetachBlockGroup) ? GridTerminalSystem.GetBlockGroupWithName(strDetachBlockGroup) : null);

    if (group == null)
    {
        if (IsNotEmpty(strDetachPortTag))
        {
            GridTerminalSystem.GetBlocksOfType<T>(blocks, MatchDetachPortTag);
        }
        else
        {
            GridTerminalSystem.GetBlocksOfType<T>(blocks);
        }
    }
    else
    {
        if (IsNotEmpty(strDetachPortTag))
        {
            group.GetBlocksOfType<T>(blocks, MatchDetachPortTag);
        }
        else
        {
            group.GetBlocksOfType<T>(blocks);
        }
    }

    return blocks;
}

bool MatchDetachPortTag(IMyTerminalBlock block)
{
    return (block.CustomName.IndexOf(strDetachPortTag, StringComparison.OrdinalIgnoreCase) > -1);
}

IMyTerminalBlock GetClosestBlockFromReference(List<IMyTerminalBlock> checkBlocks, IMyTerminalBlock referenceBlock, bool sameGridCheck = false)
{
    IMyTerminalBlock checkBlock = null;
    double prevCheckDistance = Double.MaxValue;

    for (int i = 0; i < checkBlocks.Count; i++)
    {
        if (!sameGridCheck || checkBlocks[i].CubeGrid == referenceBlock.CubeGrid)
        {
            double currCheckDistance = (checkBlocks[i].GetPosition() - referenceBlock.GetPosition()).Length();
            if (currCheckDistance < prevCheckDistance)
            {
                prevCheckDistance = currCheckDistance;
                checkBlock = checkBlocks[i];
            }
        }
    }

    return checkBlock;
}

void DetachLockedConnectors()
{
    List<IMyTerminalBlock> blocks = new List<IMyTerminalBlock>();
    GridTerminalSystem.GetBlocksOfType<IMyShipConnector>(blocks);

    for (int i = 0; i < blocks.Count; i++)
    {
        if (blocks[i].CubeGrid == Me.CubeGrid)
        {
            IMyShipConnector otherConnector = ((IMyShipConnector)blocks[i]).OtherConnector;
            if (otherConnector == null || blocks[i].CubeGrid != otherConnector.CubeGrid)
            {
                blocks[i].ApplyAction("Unlock");
            }
        }
    }
}

//------------------------------ String Parsing Methods ------------------------------

bool ParseMatrix(string[] tokens, out MatrixD parsedMatrix, int start = 0, bool isOrientation = false)
{
    if (tokens.Length < start + (isOrientation ? 9 : 16))
    {
        parsedMatrix = new MatrixD();
        return false;
    }

    double v;
    double[] r = new double[isOrientation ? 9 : 16];

    for (int i = start; i < start + r.Length; i++)
    {
        if (Double.TryParse(tokens[i], out v))
        {
            r[i] = v;
        }
    }

    if (isOrientation)
    {
        parsedMatrix = new MatrixD(r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8]);
    }
    else
    {
        parsedMatrix = new MatrixD(r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8], r[9], r[10], r[11], r[12], r[13], r[14], r[15]);
    }

    return true;
}

bool ParseVector(string[] tokens, out Vector3D parsedVector, int start = 0)
{
    parsedVector = new Vector3D();

    if (tokens.Length < start + 3)
    {
        return false;
    }

    double result;

    if (Double.TryParse(tokens[start], out result))
    {
        parsedVector.SetDim(0, result);
    }
    else
    {
        return false;
    }

    if (Double.TryParse(tokens[start + 1], out result))
    {
        parsedVector.SetDim(1, result);
    }
    else
    {
        return false;
    }

    if (Double.TryParse(tokens[start + 2], out result))
    {
        parsedVector.SetDim(2, result);
    }
    else
    {
        return false;
    }

    return true;
}

bool ParseCoordinates(string coordinates, out Vector3D parsedVector)
{
    parsedVector = new Vector3D();
    coordinates = coordinates.Trim();

    double result;
    string[] tokens = coordinates.Split(':');

    if (coordinates.StartsWith("GPS") && tokens.Length >= 5)
    {
        if (Double.TryParse(tokens[2], out result))
        {
            parsedVector.SetDim(0, result);
        }
        else
        {
            return false;
        }

        if (Double.TryParse(tokens[3], out result))
        {
            parsedVector.SetDim(1, result);
        }
        else
        {
            return false;
        }

        if (Double.TryParse(tokens[4], out result))
        {
            parsedVector.SetDim(2, result);
        }
        else
        {
            return false;
        }

        return true;
    }
    else if (coordinates.StartsWith("[T:") && tokens.Length >= 4)
    {
        if (Double.TryParse(tokens[1], out result))
        {
            parsedVector.SetDim(0, result);
        }
        else
        {
            return false;
        }

        if (Double.TryParse(tokens[2], out result))
        {
            parsedVector.SetDim(1, result);
        }
        else
        {
            return false;
        }

        if (Double.TryParse(tokens[3].Substring(0, tokens[3].Length - 1), out result))
        {
            parsedVector.SetDim(2, result);
        }
        else
        {
            return false;
        }

        return true;
    }
    else
    {
        return false;
    }
}

//------------------------------ Command Processing Methods ------------------------------

void ProcessCustomConfiguration()
{
    CustomConfiguration cfg = new CustomConfiguration(Me);
    cfg.Load();

    cfg.Get("missileLaunchType", ref missileLaunchType);
    cfg.Get("missileDetachPortType", ref missileDetachPortType);
    cfg.Get("spinAmount", ref spinAmount);
    cfg.Get("verticalTakeoff", ref verticalTakeoff);
    cfg.Get("excludeFriendly", ref excludeFriendly);
    cfg.Get("adjustThrustBasedOnAim", ref adjustThrustBasedOnAim);
    cfg.Get("waitForHomingTrigger", ref waitForHomingTrigger);
    cfg.Get("enableMissileCommand", ref enableMissileCommand);
    cfg.Get("fivePointInitialLockDist", ref fivePointInitialLockDist);
    cfg.Get("raycastAheadCount", ref raycastAheadCount);
    cfg.Get("raycastAheadSeconds", ref raycastAheadSeconds);
    cfg.Get("useOffsetTargeting", ref useOffsetTargeting);
    cfg.Get("useOffsetRaycasting", ref useOffsetRaycasting);
    cfg.Get("switchTurretToRaycast", ref switchTurretToRaycast);
    cfg.Get("missileId", ref missileId);
    cfg.Get("missileGroup", ref missileGroup);
    cfg.Get("allowedSenderId", ref allowedSenderId);
    cfg.Get("strShipRefLidar", ref strShipRefLidar);
    cfg.Get("strShipRefBlockGroup", ref strShipRefBlockGroup);
    cfg.Get("strDetachBlockGroup", ref strDetachBlockGroup);
    cfg.Get("strShipRefForward", ref strShipRefFwd);
    cfg.Get("strShipRefTargetPanel", ref strShipRefPanel);
    cfg.Get("strShipRefTurrets", ref strShipRefTurrets);
    cfg.Get("strShipRefNotMissileTag", ref strShipRefNotMissileTag);
    cfg.Get("missileActivationCommands", ref missileActivationCommands);
    cfg.Get("missileTriggerCommands", ref missileTriggerCommands);
    cfg.Get("proximityTriggerCommands", ref proximityTriggerCommands);
    cfg.Get("failunsafeTriggerCommands", ref failunsafeTriggerCommands);
    cfg.Get("strGyroscopesTag", ref strGyroscopesTag);
    cfg.Get("strThrustersTag", ref strThrustersTag);
    cfg.Get("strDetachPortTag", ref strDetachPortTag);
    cfg.Get("strDirectionRefBlockTag", ref strDirectionRefBlockTag);
    cfg.Get("strProximitySensorTag", ref strProximitySensorTag);
    cfg.Get("strLockTriggerBlockTag", ref strLockTriggerBlockTag);
    cfg.Get("strLockTriggerAction", ref strLockTriggerAction);
    cfg.Get("strStatusDisplayPrefix", ref strStatusDisplayPrefix);
    cfg.Get("launchSeconds", ref launchSeconds);
    cfg.Get("boolDrift", ref boolDrift);
    cfg.Get("boolLeadTarget", ref boolLeadTarget);
    cfg.Get("boolNaturalDampener", ref boolNaturalDampener);
    cfg.Get("LIDAR_MIN_LOCK_DISTANCE", ref LIDAR_MIN_LOCK_DISTANCE);
    cfg.Get("LIDAR_MAX_LOCK_DISTANCE", ref LIDAR_MAX_LOCK_DISTANCE);
    cfg.Get("LIDAR_REFRESH_INTERVAL", ref LIDAR_REFRESH_INTERVAL);
    cfg.Get("LIDAR_REFRESH_CALC_FACTOR", ref LIDAR_REFRESH_CALC_FACTOR);
    cfg.Get("AIM_P", ref AIM_P);
    cfg.Get("AIM_I", ref AIM_I);
    cfg.Get("AIM_D", ref AIM_D);
    cfg.Get("AIM_LIMIT", ref AIM_LIMIT);
    cfg.Get("INTEGRAL_WINDUP_UPPER_LIMIT", ref INTEGRAL_WINDUP_UPPER_LIMIT);
    cfg.Get("INTEGRAL_WINDUP_LOWER_LIMIT", ref INTEGRAL_WINDUP_LOWER_LIMIT);
    cfg.Get("MERGE_SEPARATE_WAIT_THRESHOLD", ref MERGE_SEPARATE_WAIT_THRESHOLD);
    cfg.Get("outputMissileStatus", ref outputMissileStatus);
}

void ProcessConfigurationCommand(string commandLine)
{
    string[] keyValues = commandLine.Split(',');

    for (int i = 0; i < keyValues.Length; i++)
    {
        string[] tokens = keyValues[i].Trim().Split(':');
        if (tokens.Length > 0)
        {
            ProcessSingleConfigCommand(tokens);
        }
    }
}

void ProcessSingleConfigCommand(string[] tokens)
{
    string cmdToken = tokens[0].Trim().ToUpper();
    if (cmdToken.Equals("MODE") && tokens.Length >= 2)
    {
        int modeValue;
        if (Int32.TryParse(tokens[1], out modeValue))
        {
            missileLaunchType = modeValue;
        }
    }
    else if (cmdToken.Equals("R_LDR") && tokens.Length >= 2)
    {
        strShipRefLidar = tokens[1];
    }
    else if (cmdToken.Equals("R_TAR") && tokens.Length >= 2)
    {
        strShipRefPanel = tokens[1];
    }
    else if (cmdToken.Equals("R_FWD") && tokens.Length >= 2)
    {
        strShipRefFwd = tokens[1];
    }
    else if (cmdToken.Equals("R_DES") && tokens.Length >= 2)
    {
        strShipRefTurrets = tokens[1];
    }
    else if (cmdToken.Equals("V_LS") && tokens.Length >= 2)
    {
        double lsValue;
        if (Double.TryParse(tokens[1], out lsValue))
        {
            launchSeconds = lsValue;
        }
    }
    else if (cmdToken.Equals("V_DRIFT") && tokens.Length >= 2)
    {
        bool driftValue;
        if (bool.TryParse(tokens[1], out driftValue))
        {
            boolDrift = driftValue;
        }
    }
    else if (cmdToken.Equals("V_LEAD") && tokens.Length >= 2)
    {
        bool leadValue;
        if (bool.TryParse(tokens[1], out leadValue))
        {
            boolLeadTarget = leadValue;
        }
    }
    else if (cmdToken.Equals("V_DAMP") && tokens.Length >= 2)
    {
        bool dampenerValue;
        if (bool.TryParse(tokens[1], out dampenerValue))
        {
            boolNaturalDampener = dampenerValue;
        }
    }
    else if (cmdToken.Equals("P_VT") && tokens.Length >= 2)
    {
        bool vtValue;
        if (bool.TryParse(tokens[1], out vtValue))
        {
            verticalTakeoff = vtValue;
        }
    }
    else if (cmdToken.Equals("P_WFT") && tokens.Length >= 2)
    {
        bool wftValue;
        if (bool.TryParse(tokens[1], out wftValue))
        {
            waitForHomingTrigger = wftValue;
        }
    }
    else if (cmdToken.Equals("P_EMC") && tokens.Length >= 2)
    {
        bool emcValue;
        if (bool.TryParse(tokens[1], out emcValue))
        {
            enableMissileCommand = emcValue;
        }
    }
    else if (cmdToken.Equals("SPIN") && tokens.Length >= 2)
    {
        double spinValue;
        if (Double.TryParse(tokens[1], out spinValue))
        {
            spinAmount = (int)spinValue;
        }
    }
    else if (cmdToken.Equals("CHECK") || cmdToken.Equals("CHECKMISSILE"))
    {
        subMode = 0;

        CheckMissile();
    }
    else if (cmdToken.Equals("CHECKSHIP"))
    {
        subMode = 0;

        CheckLaunchingShip();
    }
}

void ProcessTriggerCommands()
{
    if (rpmTriggerList != null && rpmTriggerList.Count > 0)
    {
        int i = 0;
        while (i < rpmTriggerList.Count)
        {
            if (rpmTriggerList[i].Key <= rpm)
            {
                ProcessSingleMissileCommand(rpmTriggerList[i].Value);
                rpmTriggerList.RemoveAt(i);
            }
            else
            {
                i++;
            }
        }
    }

    if (distTriggerList != null && distTriggerList.Count > 0)
    {
        int i = 0;
        while (i < distTriggerList.Count)
        {
            if (distTriggerList[i].Key >= distToTarget)
            {
                ProcessSingleMissileCommand(distTriggerList[i].Value);
                distTriggerList.RemoveAt(i);
            }
            else
            {
                i++;
            }
        }
    }

    if (timeTriggerList != null && timeTriggerList.Count > 0)
    {
        int i = 0;
        while (i < timeTriggerList.Count)
        {
            if (timeTriggerList[i].Key <= clock)
            {
                ProcessSingleMissileCommand(timeTriggerList[i].Value);
                timeTriggerList.RemoveAt(i);
            }
            else
            {
                i++;
            }
        }
    }
}

void CheckProximitySensors()
{
    int curfailunsafe = 0;


    for (int i = 0; i < proximitySensors.Count; i++)
    {
        ProximitySensor sensor = proximitySensors[i];
        if (sensor.lidar == null) continue;

        double dist = sensor.distance;
        if (dist <= 0)
        {
            dist = speed / SECOND;
        }

        if (sensor.lidar.IsWorking && sensor.lidar.CanScan(dist))
        {
            if (sensor.dmsrange > dist && sensor.lidar.CanScan(sensor.dmsrange))
            {
                MyDetectedEntityInfo detected = sensor.lidar.Raycast(sensor.dmsrange, sensor.pitch, sensor.yaw);
                if (IsValidProximityTarget(ref detected))
                {
                    double raycastDist = Vector3D.Distance((detected.HitPosition.HasValue ? detected.HitPosition.Value : detected.Position), sensor.lidar.GetPosition());
                    if (raycastDist <= dist)
                    {
                        ProcessMissileCommand(sensor.proximityTriggerCommands);
                        sensor.lidar = null;
                        return;
                    }

                    if (detected.HitPosition.HasValue)
                    {
                        raycastDist = Vector3D.Distance(detected.Position, sensor.lidar.GetPosition());
                    }

                    if (sensor.dmsActive)
                    {
                        if (raycastDist > sensor.dmsPrevDist)
                        {
                            ProcessMissileCommand(sensor.proximityTriggerCommands);
                            sensor.lidar = null;
                            return;
                        }
                        sensor.dmsPrevDist = raycastDist;
                    }
                    else
                    {
                        sensor.dmsPrevDist = raycastDist;
                        sensor.dmsActive = true;
                    }
                }
                else if (sensor.dmsActive)
                {
                    ProcessMissileCommand(sensor.proximityTriggerCommands);
                    sensor.lidar = null;
                    return;
                }
            }
            else
            {
                MyDetectedEntityInfo detected = sensor.lidar.Raycast(dist, sensor.pitch, sensor.yaw);
                if (IsValidProximityTarget(ref detected))
                {
                    ProcessMissileCommand(sensor.proximityTriggerCommands);
                    sensor.lidar = null;
                    return;
                }
            }
        }
        else if (sensor.dmsActive)
        {
            ProcessMissileCommand(sensor.proximityTriggerCommands);
            sensor.lidar = null;
            return;
        }
        else if (sensor.failunsafe)
        {
            if (failunsafeGrpCnt > 0)
            {
                curfailunsafe++;
            }
            else
            {
                ProcessMissileCommand(sensor.failunsafeTriggerCommands);
                sensor.lidar = null;
                return;
            }
        }
    }

    if (failunsafeGrpCnt > 0 && curfailunsafe >= failunsafeGrpCnt)
    {
        string failunsafeCmd = null;
        for (int i = 0; i < proximitySensors.Count; i++)
        {
            ProximitySensor sensor = proximitySensors[i];
            if (sensor.lidar != null && sensor.failasgroup && sensor.failunsafe)
            {
                failunsafeCmd = sensor.failunsafeTriggerCommands;
                sensor.lidar = null;
            }
        }

        if (failunsafeCmd != null)
        {
            ProcessMissileCommand(failunsafeCmd);
            return;
        }
    }
}

void ProcessCommunicationMessage(string message)
{
    string[] msgTokens = message.Split(new char[] {'\r','\n'}, StringSplitOptions.RemoveEmptyEntries);

    for (int i = 0; i < msgTokens.Length; i++)
    {
        string msg = msgTokens[i];

        string recipient;
        string sender;
        string options;     //Not Supported Yet, For Future Use

        int start = msg.IndexOf("MSG;", 0, StringComparison.OrdinalIgnoreCase);
        if (start > -1)
        {
            start += 4;

            recipient = NextToken(msg, ref start, ';');
            sender = NextToken(msg, ref start, ';');
            options = NextToken(msg, ref start, ';');

            if (IsValidRecipient(recipient) && IsValidSender(sender))
            {
                if (msg.Length > start)
                {
                    ProcessMissileCommand(msg.Substring(start));
                }
            }
        }
    }
}

bool IsValidRecipient(string recipient)
{
    if (recipient.Length == 0)
    {
        return true;
    }

    int code = (recipient[0] == '*' ? 1 : 0) + (recipient[recipient.Length - 1] == '*' ? 2 : 0);
    switch (code)
    {
        case 0:
            return missileId.Equals(recipient, StringComparison.OrdinalIgnoreCase) ||
                    (missileGroup != null && missileGroup.Equals(recipient, StringComparison.OrdinalIgnoreCase));
        case 1:
            return missileId.EndsWith(recipient.Substring(1), StringComparison.OrdinalIgnoreCase) ||
                    (missileGroup != null && missileGroup.EndsWith(recipient.Substring(1), StringComparison.OrdinalIgnoreCase));
        case 2:
            return missileId.StartsWith(recipient.Substring(0, recipient.Length - 1), StringComparison.OrdinalIgnoreCase) ||
                    (missileGroup != null && missileGroup.StartsWith(recipient.Substring(0, recipient.Length - 1), StringComparison.OrdinalIgnoreCase));
        default:
            return (recipient.Length == 1) || (missileId.IndexOf(recipient.Substring(1, recipient.Length - 2), StringComparison.OrdinalIgnoreCase) > -1) ||
                    (missileGroup != null && (missileGroup.IndexOf(recipient.Substring(1, recipient.Length - 2), StringComparison.OrdinalIgnoreCase) > -1));
    }
}

bool IsValidSender(string sender)
{
    if (allowedSenderId == null || allowedSenderId.Length == 0)
    {
        return true;
    }

    int code = (allowedSenderId[0] == '*' ? 1 : 0) + (allowedSenderId[allowedSenderId.Length - 1] == '*' ? 2 : 0);
    switch (code)
    {
        case 0:
            return sender.Equals(allowedSenderId, StringComparison.OrdinalIgnoreCase);
        case 1:
            return sender.EndsWith(allowedSenderId.Substring(1), StringComparison.OrdinalIgnoreCase);
        case 2:
            return sender.StartsWith(allowedSenderId.Substring(0, allowedSenderId.Length - 1), StringComparison.OrdinalIgnoreCase);
        default:
            return (allowedSenderId.Length == 1) || (sender.IndexOf(allowedSenderId.Substring(1, allowedSenderId.Length - 2), StringComparison.OrdinalIgnoreCase) > -1);
    }
}

string NextToken(string line, ref int start, char delim)
{
    if (line.Length > start)
    {
        int end = line.IndexOf(delim, start);
        if (end > -1)
        {
            string result = line.Substring(start, end - start);
            start = end + 1;
            return result;
        }
    }
    start = line.Length;
    return "";
}

void ProcessMissileCommand(string commandLine)
{
    string[] keyValues = commandLine.Split(',');

    for (int i = 0; i < keyValues.Length; i++)
    {
        string[] tokens = keyValues[i].Trim().Split(':');
        if (tokens.Length > 0)
        {
            ProcessSingleMissileCommand(tokens);
        }
    }
}

void ProcessSingleMissileCommand(string[] tokens)
{
    string cmdToken = tokens[0].Trim().ToUpper();
    if (cmdToken.Equals("GPS"))
    {
        if (tokens.Length >= 4)
        {
            Vector3D parsedVector;
            if (ParseVector(tokens, out parsedVector, (tokens.Length == 4 ? 1 : 2)))
            {
                commsPosition = parsedVector;
                commsPositionSet = true;
            }
        }
    }
    else if (cmdToken.Equals("FWD"))
    {
        if (tokens.Length >= 4)
        {
            Vector3D parsedVector1;
            if (ParseVector(tokens, out parsedVector1, 1))
            {
                Vector3D parsedVector2;
                if (tokens.Length >= 7)
                {
                    if (!ParseVector(tokens, out parsedVector2, 4))
                    {
                        parsedVector2 = new Vector3D();
                    }
                }
                else
                {
                    parsedVector2 = new Vector3D();
                }
                commsFwd = new RayD(parsedVector2, parsedVector1);
                commsFwdSet = true;
            }
        }
    }
    else if (cmdToken.Equals("LDR"))
    {
        if (tokens.Length >= 2)
        {
            long entityId;
            if (!long.TryParse(tokens[1], out entityId))
            {
                entityId = -1;
            }

            Vector3D position;
            if (!(tokens.Length >= 5 && ParseVector(tokens, out position, 2)))
            {
                position = new Vector3D();
            }

            Vector3D velocity;
            if (!(tokens.Length >= 8 && ParseVector(tokens, out velocity, 5)))
            {
                velocity = new Vector3D();
            }

            Vector3D hitPosition;
            if (!(tokens.Length >= 11 && ParseVector(tokens, out hitPosition, 8)))
            {
                hitPosition = position;
            }

            Vector3D boxMin;
            if (!(tokens.Length >= 14 && ParseVector(tokens, out boxMin, 11)))
            {
                boxMin = position + new Vector3D(-1.25, -1.25, -1.25);
            }

            MatrixD orientation;
            if (!(tokens.Length >= 23 && ParseMatrix(tokens, out orientation, 14, true)))
            {
                orientation = new MatrixD();
            }

            int value;
            MyDetectedEntityType targetType;
            if (!(tokens.Length >= 24 && !int.TryParse(tokens[23], out value)))
            {
                value = 3;
            }
            try { targetType = (MyDetectedEntityType)value; }
            catch { targetType = MyDetectedEntityType.LargeGrid; }

            MyRelationsBetweenPlayerAndBlock targetRelationship;
            if (!(tokens.Length >= 25 && !int.TryParse(tokens[24], out value)))
            {
                value = 3;
            }
            try { targetRelationship = (MyRelationsBetweenPlayerAndBlock)value; }
            catch { targetRelationship = MyRelationsBetweenPlayerAndBlock.Neutral; }

            long timestamp;
            if (!(tokens.Length >= 26 && !long.TryParse(tokens[25], out timestamp)))
            {
                timestamp = DateTime.Now.Ticks;
            }

            BoundingBoxD boundingBox = new BoundingBoxD(boxMin, position + position - boxMin);

            commsLidarTarget = new MyDetectedEntityInfo(entityId, (tokens.Length >= 27 ? tokens[26] : ""), targetType, hitPosition, orientation, velocity, targetRelationship, boundingBox, timestamp);
            commsLidarTargetSet = true;
        }
    }
    else if (cmdToken.Equals("LOCK"))
    {
        homingReleaseLock = true;
    }
    else if (cmdToken.Equals("ABORT"))
    {
        gyroControl.ZeroTurnGyro();

        Runtime.UpdateFrequency = UpdateFrequency.None;
        mode = 99;
    }
    else if (cmdToken.Equals("SEVER"))
    {
        if (shipRefLidars != null)
        {
            shipRefLidars.Clear();
        }

        shipRefFwd = null;
        shipRefPanel = null;

        enableMissileCommand = false;
    }
    else if (cmdToken.Equals("CRUISE"))
    {
        if (verticalTakeoff)
        {
            FireThrusters(launchThrusters, false);
            FireThrusters(thrusters, true);
        }

        gyroControl.ResetGyro();
        gyroControl.SetGyroOverride(true);

        if (spinAmount > 0)
        {
            gyroControl.SetGyroRoll(spinAmount);
        }

        lastTargetPosition = targetPosition = GetFlyStraightVector();

        subCounter = 0;
        subMode = 0;
        mode = 4;
    }
    else if (cmdToken.Equals("GLIDE"))
    {
        if (verticalTakeoff)
        {
            FireThrusters(launchThrusters, false);
            FireThrusters(thrusters, true);
        }

        gyroControl.ResetGyro();
        gyroControl.SetGyroOverride(true);

        if (spinAmount > 0)
        {
            gyroControl.SetGyroRoll(spinAmount);
        }

        if (mode == -1)
        {
            refFwdPosition = midPoint;
            refFwdVector = Vector3D.Normalize(shipVelocity);
            refFwdSet = true;
        }
        else if (mode == 8 && homingTurret != null)
        {
            refFwdPosition = midPoint;
            refFwdVector = CalculateTurretViewVector(homingTurret);
            refFwdSet = true;
        }
        else if ((mode == 2 && subMode == 0) || (mode == 3))
        {
            if (commsFwdSet)
            {
                commsFwdSet = false;

                refFwdPosition = commsFwd.Position;
                refFwdVector = commsFwd.Direction;
                refFwdSet = true;
            }
            else if (shipRefFwd != null)
            {
                refFwdPosition = shipRefFwd.WorldMatrix.Translation;
                refFwdVector = (fwdIsTurret ? CalculateTurretViewVector(shipRefFwd as IMyLargeTurretBase) : shipRefFwd.WorldMatrix.Forward);
                refFwdSet = true;
            }
        }
        else
        {
            refFwdPosition = midPoint;
            refFwdVector = Vector3D.Normalize(targetPosition - midPoint);
            refFwdSet = true;
        }

        subCounter = 0;
        subMode = 0;
        mode = 6;
    }
    else if (cmdToken.StartsWith("ACT") && tokens.Length >= 3)
    {
        char opCode = (cmdToken.Length >= 4 ? cmdToken[3] : 'B');
        List<IMyTerminalBlock> triggerBlocks = null;
        switch (opCode)
        {
        case 'B':
            triggerBlocks = GetBlocksWithName<IMyTerminalBlock>(tokens[1], 3);
            break;
        case 'P':
            triggerBlocks = GetBlocksWithName<IMyTerminalBlock>(tokens[1], 1);
            break;
        case 'S':
            triggerBlocks = GetBlocksWithName<IMyTerminalBlock>(tokens[1], 2);
            break;
        case 'W':
            triggerBlocks = GetBlocksWithName<IMyTerminalBlock>(tokens[1], 0);
            break;
        }

        if (triggerBlocks != null)
        {
            for (int i = 0; i < triggerBlocks.Count; i++)
            {
                ITerminalAction action = triggerBlocks[i].GetActionWithName(tokens[2]);
                if (action != null)
                {
                    action.Apply(triggerBlocks[i]);
                }
            }
        }
    }
    else if (cmdToken.StartsWith("SET") && tokens.Length >= 3)
    {
        char opCode = (cmdToken.Length >= 4 ? cmdToken[3] : 'B');
        List<IMyTerminalBlock> triggerBlocks = null;
        switch (opCode)
        {
        case 'B':
            triggerBlocks = GetBlocksWithName<IMyTerminalBlock>(tokens[1], 3);
            break;
        case 'P':
            triggerBlocks = GetBlocksWithName<IMyTerminalBlock>(tokens[1], 1);
            break;
        case 'S':
            triggerBlocks = GetBlocksWithName<IMyTerminalBlock>(tokens[1], 2);
            break;
        case 'W':
            triggerBlocks = GetBlocksWithName<IMyTerminalBlock>(tokens[1], 0);
            break;
        }

        char propCode = (cmdToken.Length >= 5 ? cmdToken[4] : 'P');

        if (triggerBlocks != null)
        {
            for (int i = 0; i < triggerBlocks.Count; i++)
            {
                switch (propCode)
                {
                case 'P':
                    triggerBlocks[i].SetValueFloat(tokens[2], float.Parse(tokens[3]));
                    break;
                case 'B':
                    triggerBlocks[i].SetValueBool(tokens[2], bool.Parse(tokens[3]));
                    break;
                case 'D':
                    triggerBlocks[i].SetValueFloat(tokens[2], (float)distToTarget / float.Parse(tokens[3]));
                    break;
                case 'S':
                    triggerBlocks[i].SetValueFloat(tokens[2], (float)speed / float.Parse(tokens[3]));
                    break;
                case 'T':
                    triggerBlocks[i].SetValueFloat(tokens[2], (float)(distToTarget / speed) / float.Parse(tokens[3]));
                    break;
                case 'A':
                    triggerBlocks[i].SetValueFloat(tokens[2], triggerBlocks[i].GetValueFloat(tokens[2]) + float.Parse(tokens[3]));
                    break;
                case 'M':
                    triggerBlocks[i].SetValueFloat(tokens[2], triggerBlocks[i].GetValueFloat(tokens[2]) * float.Parse(tokens[3]));
                    break;
                }
            }
        }
    }
    else if (cmdToken.Equals("SPIN") && tokens.Length >= 1)
    {
        gyroControl.SetGyroRoll(tokens.Length >= 2 ? Int32.Parse(tokens[1]) : 30);
    }
}

void ExecuteTriggerCommand(string commandLine)
{
    int startIndex = commandLine.IndexOf('[') + 1;
    int endIndex = commandLine.LastIndexOf(']');

    string command = (startIndex > 0 && endIndex > -1 ? commandLine.Substring(startIndex, endIndex - startIndex) : commandLine);
    string[] keyValues = command.Split(',');

    for (int i = 0; i < keyValues.Length; i++)
    {
        string[] tokens = keyValues[i].Trim().Split(':');
        if (tokens.Length > 0)
        {
            string cmdToken = tokens[0].Trim();

            bool resolved = false;
            if (cmdToken.Length == 3 && cmdToken.StartsWith("TG"))
            {
                int type;
                double value = 0;
                bool parsed = (tokens.Length >= 3 ? double.TryParse(tokens[1], out value) : false);

                switch (cmdToken[2])
                {
                    case 'R':
                        type = 1;
                        break;
                    case 'D':
                        type = 2;
                        break;
                    case 'E':
                        type = 2;
                        value = distToTarget - value;
                        break;
                    case 'V':
                        type = 2;
                        value += targetRadius;
                        break;
                    case 'T':
                        type = 3;
                        value = (value * SECOND) + clock;
                        break;
                    default:
                        type = 0;
                        break;
                }

                if (type > 0)
                {
                    resolved = true;

                    if (parsed)
                    {
                        string[] items = new string[tokens.Length - 2];
                        Array.Copy(tokens, 2, items, 0, items.Length);

                        switch (type)
                        {
                            case 1:
                                if (rpmTriggerList == null) rpmTriggerList = new List<KeyValuePair<double, string[]>>();
                                rpmTriggerList.Add(new KeyValuePair<double, string[]>(value, items));
                                break;
                            case 2:
                                if (distTriggerList == null) distTriggerList = new List<KeyValuePair<double, string[]>>();
                                distTriggerList.Add(new KeyValuePair<double, string[]>(value, items));
                                break;
                            case 3:
                                if (timeTriggerList == null) timeTriggerList = new List<KeyValuePair<int, string[]>>();
                                timeTriggerList.Add(new KeyValuePair<int, string[]>((int)value, items));
                                break;
                        }

                        haveTriggerCommands = true;
                    }
                }
            }

            if (!resolved)
            {
                ProcessSingleMissileCommand(tokens);
            }
        }
    }
}

//------------------------------ Script Debugging Methods ------------------------------

void CheckMissile()
{
    Echo("----- Missile Issues -----\n");

    shipRefLidars = new List<IMyCameraBlock>(1);
    InitMissileBlocks(true);

    Echo("\n----- Missile Parameters -----");

    Echo("\n[Compatible Homing Modes]:");
    Echo((missileLidars.Count > 0 ? "0,1,2,3,4,5,6,7" : "3,4,5,6,7") + (homingTurret != null ? ",8,9" : ""));
    Echo("\nDrift Compensation: " + (boolDrift == true ? "Yes" : "No"));
    Echo("Gravity Dampeners: " + (boolNaturalDampener == true ? "Yes" : "No"));
    Echo("Target Leading: " + (boolLeadTarget == true ? "Yes" : "No"));
    Echo("\nVertical Takeoff: " + (verticalTakeoff == true ? "Yes" : "No"));
    Echo("\nProximity Sensors: " + (proximitySensors != null ? "Yes" : "No"));
    Echo("\n<<Below lists the Detected Blocks.\nSet to Show On HUD for checking>>");

    Echo("\nOne of the Forward Thrusters:");
    if (thrusters.Count > 0)
    {
        Echo(thrusters[0].CustomName);
        thrusters[0].ShowOnHUD = true;
    }
    else
    {
        Echo("<NONE>");
    }

    Echo("\nOne of the Gravity Dampeners:");
    if (refDwdBlock != null)
    {
        Echo(refDwdBlock.CustomName);
        refDwdBlock.ShowOnHUD = true;
    }
    else
    {
        Echo("<NONE>");
    }

    if (thrusters.Count > 0)
    {
        bool haveFwdLidars = false;
        for (int i = 0; i < missileLidars.Count; i++)
        {
            if (missileLidars[i].WorldMatrix.Forward.Dot(thrusters[0].WorldMatrix.Backward) > 0.99)
            {
                haveFwdLidars = true;
                break;
            }
        }
        if (!haveFwdLidars)
        {
            Echo("\nWarning: Missing Forward Facing Cameras.");
        }
    }

    Echo("\n--- End Of Check, Recompile Script & Remove CHECK Argument ---");
}

void CheckLaunchingShip()
{
    Echo("----- Launching Ship Warnings -----\n");

    InitLaunchingShipRefBlocks(true);

    Echo("\n----- Launching Ship Parameters -----");

    Echo("\n<<Below lists the Detected Blocks.\nSet to Show On HUD for checking>>");

    Echo("\nR_FORWARD Aiming Block:");
    if (shipRefFwd != null)
    {
        Echo(shipRefFwd.CustomName);
        shipRefFwd.ShowOnHUD = true;
    }
    else
    {
        Echo("<NONE>");
    }

    Echo("\nOne of the R_LIDAR Cameras:");
    if (shipRefLidars.Count > 0)
    {
        Echo(shipRefLidars[0].CustomName);
        shipRefLidars[0].ShowOnHUD = true;
    }
    else
    {
        Echo("<NONE>");
    }

    Echo("\nR_TARGET GPS Text Panel:");
    if (shipRefPanel != null)
    {
        Echo(shipRefPanel.CustomName);
        shipRefPanel.ShowOnHUD = true;
    }
    else
    {
        Echo("<NONE>");
    }

    Echo("\nLock-On Alert Sound Block:");
    if (alertBlock != null)
    {
        Echo(alertBlock.CustomName);
        alertBlock.ShowOnHUD = true;
    }
    else
    {
        Echo("<NONE>");
    }

    DetachFromGrid(true);

    Echo("\nBlock to Detach on Launch:");
    if (detachBlock != null)
    {
        Echo(detachBlock.CustomName);
        detachBlock.ShowOnHUD = true;
    }
    else
    {
        Echo("<WARNING NOT FOUND>");
    }

    Echo("\n--- End Of Check, Recompile Script & Remove CHECKSHIP Argument ---");
}

//------------------------------ Initialization Methods ------------------------------

bool InitLaunchingShipRefBlocks(bool testOnly = false)
{
    bool multiShipRefPanel = false;
    bool multiShipRefFwd = false;

    shipRefPanel = null;
    shipRefLidars = new List<IMyCameraBlock>();

    if (missileLaunchType == 5)
    {
        shipRefTurrets = new List<IMyLargeTurretBase>();
    }

    List<IMyTerminalBlock> blocks = new List<IMyTerminalBlock>();
    IMyBlockGroup group = (IsNotEmpty(strShipRefBlockGroup) ? GridTerminalSystem.GetBlockGroupWithName(strShipRefBlockGroup) : null);
    if (group != null)
    {
        group.GetBlocks(blocks);
    }
    else
    {
        GridTerminalSystem.GetBlocks(blocks);
    }

    foreach (IMyTerminalBlock block in blocks)
    {
        if (block is IMyCameraBlock)
        {
            if (NameContains(block, strShipRefLidar))
            {
                shipRefLidars.Add(block as IMyCameraBlock);
            }
        }
        else if (block is IMyTextPanel)
        {
            if (NameContains(block, strShipRefPanel))
            {
                if (shipRefPanel == null)
                {
                    shipRefPanel = block as IMyTextPanel;
                }
                else
                {
                    multiShipRefPanel = true;
                }
            }
        }

        if (NameContains(block, strShipRefFwd))
        {
            if (shipRefFwd == null)
            {
                shipRefFwd = block;
            }
            else
            {
                multiShipRefFwd = true;
            }
        }

        if (alertBlock == null && NameContains(block, strLockTriggerBlockTag))
        {
            alertBlock = block;
        }

        if (notMissile == null && NameContains(block, strShipRefNotMissileTag))
        {
            notMissile = block;
        }

        if (missileLaunchType == 5)
        {
            if (block is IMyLargeTurretBase && NameContains(block, strShipRefTurrets))
            {
                shipRefTurrets.Add(block as IMyLargeTurretBase);
            }
        }
    }

    if (shipRefLidars.Count == 0)
    {
        Echo("Warning: Missing Camera Lidars with tag " + strShipRefLidar);
    }
    else if (!testOnly)
    {
        foreach (IMyCameraBlock camera in shipRefLidars)
        {
            camera.Enabled = true;
            camera.EnableRaycast = true;
        }
    }

    if (shipRefPanel == null)
    {
        Echo("Warning: Missing Text Panel with tag " + strShipRefPanel);

        enableMissileCommand = false;
    }
    else if (multiShipRefPanel)
    {
        Echo("Warning: Multiple Text Panel with tag " + strShipRefPanel + " found. Using first panel - " + shipRefPanel.CustomName);
    }

    if (shipRefFwd == null)
    {
        IMyCockpit cockpit = null;
        foreach (IMyTerminalBlock block in blocks)
        {
            if (block is IMyCockpit)
            {
                if (cockpit == null || (!cockpit.IsMainCockpit && (block as IMyCockpit).IsMainCockpit))
                {
                    cockpit = block as IMyCockpit;
                }
            }
        }

        if (cockpit == null)
        {
            Echo("Warning: Missing Forward Block with tag " + strShipRefFwd);
        }
        else
        {
            Echo("Warning: Missing Forward Block, Using Cockpit - " + cockpit.CustomName);

            shipRefFwd = cockpit;
        }
    }
    else if (multiShipRefFwd)
    {
        Echo("Warning: Multiple Forward Block with tag " + strShipRefFwd + " found. Using first block - " + shipRefFwd.CustomName);
    }

    fwdIsTurret = ((shipRefFwd as IMyLargeTurretBase) != null);

    return true;
}

bool InitMissileBlocks(bool testOnly = false)
{
    List<IMyTerminalBlock> gyros = new List<IMyTerminalBlock>();
    thrusters = new List<IMyThrust>();
    missileLidars = new List<IMyCameraBlock>();
    remoteControl = null;
    refFwdBlock = null;
    homingTurret = null;
    statusDisplay = null;

    bool isFixedDirection = IsNotEmpty(strDirectionRefBlockTag);
    bool needStatusBlock = IsNotEmpty(strStatusDisplayPrefix);

    List<IMyTerminalBlock> blocks = new List<IMyTerminalBlock>();
    GridTerminalSystem.GetBlocks(blocks);

    foreach (IMyTerminalBlock block in blocks)
    {
        if (block.CubeGrid == Me.CubeGrid)
        {
            if (block is IMyGyro)
            {
                if (strGyroscopesTag == null || strGyroscopesTag.Length == 0 || NameContains(block, strGyroscopesTag))
                {
                    gyros.Add(block);
                }
            }
            else if (block is IMyThrust)
            {
                if (strThrustersTag == null || strThrustersTag.Length == 0 || NameContains(block, strThrustersTag))
                {
                    thrusters.Add(block as IMyThrust);
                }
            }
            else if (block is IMyCameraBlock)
            {
                missileLidars.Add(block as IMyCameraBlock);
            }
            else if (block is IMyRemoteControl)
            {
                remoteControl = block as IMyRemoteControl;
            }

            if (isFixedDirection && refFwdBlock == null)
            {
                if (NameContains(block, strDirectionRefBlockTag))
                {
                    refFwdBlock = block;
                }
            }
        }

        if (needStatusBlock && statusDisplay == null)
        {
            if (block.CustomName.StartsWith(strStatusDisplayPrefix, StringComparison.OrdinalIgnoreCase))
            {
                statusDisplay = block;

                if (!testOnly)
                {
                    if (statusDisplay.HasAction("OnOff_On"))
                    {
                        statusDisplay.ApplyAction("OnOff_On");

                        IMyRadioAntenna radioAntenna = statusDisplay as IMyRadioAntenna;
                        if (radioAntenna != null && !radioAntenna.IsBroadcasting)
                        {
                            radioAntenna.EnableBroadcasting = true;
                        }
                    }
                }
            }
        }
    }

    if (missileLaunchType == 8 || missileLaunchType == 9)
    {
        foreach (IMyTerminalBlock block in blocks)
        {
            if (block is IMyLargeTurretBase)
            {
                homingTurret = block as IMyLargeTurretBase;
                break;
            }
        }
    }

    if (gyros.Count == 0) Echo("Error: Missing Gyroscopes.");
    if (thrusters.Count == 0) Echo("Warning: Missing Thrusters.");
    if (remoteControl == null) Echo("Error: Missing Remote Control.");

    proximitySensors = null;
    failunsafeGrpCnt = 0;

    for (int i = 0; i < missileLidars.Count; i++)
    {
        if (!testOnly)
        {
            missileLidars[i].Enabled = true;
            missileLidars[i].EnableRaycast = true;
        }

        int startIndex = missileLidars[i].CustomName.IndexOf(strProximitySensorTag, StringComparison.OrdinalIgnoreCase);
        if (startIndex > -1)
        {
            if (proximitySensors == null)
            {
                proximitySensors = new List<ProximitySensor>();
                hasProximitySensors = true;
            }

            ProximitySensor proxSensor = new ProximitySensor(missileLidars[i]);
            proximitySensors.Add(proxSensor);

            double proximityDist = 0;
            startIndex += strProximitySensorTag.Length + 1;
            if (missileLidars[i].CustomName.Length > startIndex)
            {
                if (missileLidars[i].CustomName[startIndex - 1] == '_')
                {
                    string proximityDistStr;
                    int endIndex = missileLidars[i].CustomName.IndexOf(" ", startIndex, StringComparison.OrdinalIgnoreCase);
                    if (endIndex == -1)
                    {
                        proximityDistStr = missileLidars[i].CustomName.Substring(startIndex).Trim();
                    }
                    else
                    {
                        proximityDistStr = missileLidars[i].CustomName.Substring(startIndex, endIndex - startIndex).Trim();
                    }

                    if (!double.TryParse(proximityDistStr, out proximityDist))
                    {
                        proximityDist = 0;
                    }
                }
            }
            proxSensor.distance = proximityDist;

            CustomConfiguration cfg = new CustomConfiguration(proxSensor.lidar);
            cfg.Load();

            float yaw = 0, pitch = 0;
            bool failunsafe = false, failasgroup = false;
            double distance = 0, dmsrange = 0;
            string failunsafeTriggerCmds = failunsafeTriggerCommands, proximityTriggerCmds = proximityTriggerCommands;

            cfg.Get("yaw", ref yaw);
            cfg.Get("pitch", ref pitch);
            cfg.Get("failunsafe", ref failunsafe);
            cfg.Get("failasgroup", ref failasgroup);
            cfg.Get("distance", ref distance);
            cfg.Get("dmsrange", ref dmsrange);
            cfg.Get("failunsafeTriggerCommands", ref failunsafeTriggerCmds);
            cfg.Get("proximityTriggerCommands", ref proximityTriggerCmds);

            proxSensor.yaw = yaw;
            proxSensor.pitch = pitch;
            proxSensor.failunsafe = failunsafe;
            proxSensor.failasgroup = failasgroup;
            proxSensor.distance = distance;
            proxSensor.dmsrange = dmsrange;
            proxSensor.failunsafeTriggerCommands = failunsafeTriggerCmds;
            proxSensor.proximityTriggerCommands = proximityTriggerCmds;

            if (failasgroup && failunsafe)
            {
                failunsafeGrpCnt++;
            }
        }
    }

    if (missileLidars.Count > 0 || shipRefLidars.Count > 0)
    {
        IMyCameraBlock lidar = (missileLidars.Count > 0 ? missileLidars[0] : shipRefLidars[0]);

        ticksRatio = lidar.TimeUntilScan(lidar.AvailableScanRange + 1000) * 0.00006;
        ticksFactor = ticksRatio / Math.Max((int)Math.Floor(missileLidars.Count * LIDAR_REFRESH_CALC_FACTOR), 1);
    }
    else
    {
        ticksRatio = 0.03;
        ticksFactor = 0.03;
    }

    if (boolLeadTarget == null)
    {
        boolLeadTarget = true;
    }

    if (spinAmount > 0)
    {
        boolNaturalDampener = false;
    }

    isFixedDirection = (refFwdBlock != null);

    if (refFwdBlock == null || boolNaturalDampener == null || boolDrift == null || verticalTakeoff)
    {
        thrustValues = ComputeMaxThrustValues(thrusters);
    }

    if (refFwdBlock == null)
    {
        refFwdBlock = ComputeHighestThrustReference(thrusters, thrustValues);
        refFwdReverse = true;
    }

    if (refFwdBlock == null)
    {
        Echo("Warning: Missing Reference Blocks or Forward Thrusters. Using " + (remoteControl == null ? "Programmable Block" : "Remote Control") + " for Reference.");

        refFwdBlock = (remoteControl == null ? Me : (IMyTerminalBlock)remoteControl);
        refFwdReverse = false;
    }

    refWorldMatrix = refFwdBlock.WorldMatrix;
    if (refFwdReverse)
    {
        refWorldMatrix.Forward = refWorldMatrix.Backward;
        refWorldMatrix.Left = refWorldMatrix.Right;
    }

    gyroControl = new GyroControl(gyros, ref refWorldMatrix);
    if (!testOnly)
    {
        gyroControl.Enabled(true);
    }

    InitThrusters(isFixedDirection, testOnly);
    thrustValues = null;

    InitPIDControllers();

    return true;
}

void InitPIDControllers()
{
    //---------- Setup PID Controller ----------

    if (AIM_P + AIM_I + AIM_D < 0.001)
    {
        if (Me.CubeGrid.ToString().Contains("Large"))
        {
            AIM_P = DEF_BIG_GRID_P;
            AIM_I = DEF_BIG_GRID_I;
            AIM_D = DEF_BIG_GRID_D;
        }
        else
        {
            AIM_P = DEF_SMALL_GRID_P;
            AIM_I = DEF_SMALL_GRID_I;
            AIM_D = DEF_SMALL_GRID_D;
            AIM_LIMIT *= 2;
        }
    }

    yawController = new PIDController(AIM_P, AIM_I, AIM_D, INTEGRAL_WINDUP_UPPER_LIMIT, INTEGRAL_WINDUP_LOWER_LIMIT, SECOND);
    pitchController = new PIDController(AIM_P, AIM_I, AIM_D, INTEGRAL_WINDUP_UPPER_LIMIT, INTEGRAL_WINDUP_LOWER_LIMIT, SECOND);
    rollController = new PIDController(AIM_P, AIM_I, AIM_D, INTEGRAL_WINDUP_UPPER_LIMIT, INTEGRAL_WINDUP_LOWER_LIMIT, SECOND);
}

void InitThrusters(bool isFixedDirection, bool testOnly = false)
{
    //---------- Find Forward Thrusters ----------

    List<IMyThrust> checkThrusters = thrusters;
    thrusters = new List<IMyThrust>();

    if (!isFixedDirection || boolNaturalDampener == null || boolDrift == null || verticalTakeoff)
    {
        IMyThrust leftThruster = null;
        IMyThrust rightThruster = null;
        IMyThrust upThruster = null;
        IMyThrust downThruster = null;

        float leftThrustTotal = 0;
        float rightThrustTotal = 0;
        float upThrustTotal = 0;
        float downThrustTotal = 0;

        for (int i = 0; i < checkThrusters.Count; i++)
        {
            Base6Directions.Direction thrusterDirection = refWorldMatrix.GetClosestDirection(checkThrusters[i].WorldMatrix.Backward);
            switch (thrusterDirection)
            {
            case Base6Directions.Direction.Forward:
                thrusters.Add(checkThrusters[i]);
                break;
            case Base6Directions.Direction.Left:
                leftThruster = checkThrusters[i];
                leftThrustTotal += thrustValues[i];
                break;
            case Base6Directions.Direction.Right:
                rightThruster = checkThrusters[i];
                rightThrustTotal += thrustValues[i];
                break;
            case Base6Directions.Direction.Up:
                upThruster = checkThrusters[i];
                upThrustTotal += thrustValues[i];
                if (isFixedDirection)
                {
                    refDwdBlock = upThruster;
                }
                break;
            case Base6Directions.Direction.Down:
                downThruster = checkThrusters[i];
                downThrustTotal += thrustValues[i];
                break;
            }

            if (!testOnly)
            {
                checkThrusters[i].Enabled = true;
            }
        }

        float highestThrust = Math.Max(Math.Max(Math.Max(leftThrustTotal, rightThrustTotal), upThrustTotal), downThrustTotal);
        if (highestThrust == 0)
        {
            if (boolNaturalDampener == true)
            {
                Echo("Warning: Natural Gravity Dampener feature not possible as there are no Downward Thrusters found.");
            }
            boolNaturalDampener = false;

            if (boolDrift == null)
            {
                boolDrift = true;
            }
        }
        else
        {
            if (!isFixedDirection)
            {
                if (leftThrustTotal == highestThrust)
                {
                    refDwdBlock = leftThruster;
                }
                else if (rightThrustTotal == highestThrust)
                {
                    refDwdBlock = rightThruster;
                }
                else if (upThrustTotal == highestThrust)
                {
                    refDwdBlock = upThruster;
                }
                else
                {
                    refDwdBlock = downThruster;
                }
            }
            boolNaturalDampener = (refDwdBlock != null);

            if (boolDrift == null)
            {
                float lowestThrust = Math.Min(Math.Min(Math.Min(leftThrustTotal, rightThrustTotal), upThrustTotal), downThrustTotal);
                boolDrift = (highestThrust > lowestThrust * 2);
            }
        }

        if (verticalTakeoff && refDwdBlock != null)
        {
            launchThrusters = new List<IMyThrust>();

            for (int i = 0; i < checkThrusters.Count; i++)
            {
                if (refDwdBlock.WorldMatrix.Forward.Dot(checkThrusters[i].WorldMatrix.Forward) >= 0.9)
                {
                    launchThrusters.Add(checkThrusters[i]);
                }
            }
        }
    }
    else
    {
        for (int i = 0; i < checkThrusters.Count; i++)
        {
            Base6Directions.Direction thrusterDirection = refWorldMatrix.GetClosestDirection(checkThrusters[i].WorldMatrix.Backward);
            if (thrusterDirection == Base6Directions.Direction.Forward)
            {
                thrusters.Add(checkThrusters[i]);
            }
            else if (boolNaturalDampener == true && thrusterDirection == Base6Directions.Direction.Up)
            {
                refDwdBlock = checkThrusters[i];
            }

            if (!testOnly)
            {
                checkThrusters[i].Enabled = true;
            }
        }

        if (boolNaturalDampener == true && refDwdBlock == null)
        {
            Echo("Warning: Natural Gravity Dampener feature not possible as there are no Downward Thrusters found.");
            boolNaturalDampener = false;
        }
    }
}

float[] ComputeMaxThrustValues(List<IMyThrust> checkThrusters)
{
    float[] thrustValues = new float[checkThrusters.Count];

    for (int i = 0; i < checkThrusters.Count; i++)
    {
        thrustValues[i] = Math.Max(checkThrusters[i].MaxEffectiveThrust, 0.00001f);
    }

    return thrustValues;
}

IMyTerminalBlock ComputeHighestThrustReference(List<IMyThrust> checkThrusters, float[] thrustValues)
{
    if (checkThrusters.Count == 0)
    {
        return null;
    }

    IMyThrust fwdThruster = null;
    IMyThrust bwdThruster = null;
    IMyThrust leftThruster = null;
    IMyThrust rightThruster = null;
    IMyThrust upThruster = null;
    IMyThrust downThruster = null;

    float fwdThrustTotal = 0;
    float bwdThrustTotal = 0;
    float leftThrustTotal = 0;
    float rightThrustTotal = 0;
    float upThrustTotal = 0;
    float downThrustTotal = 0;

    for (int i = 0; i < checkThrusters.Count; i++)
    {
        Base6Directions.Direction thrusterDirection = Me.WorldMatrix.GetClosestDirection(checkThrusters[i].WorldMatrix.Backward);
        switch (thrusterDirection)
        {
        case Base6Directions.Direction.Forward:
            fwdThruster = checkThrusters[i];
            fwdThrustTotal += thrustValues[i];
            break;
        case Base6Directions.Direction.Backward:
            bwdThruster = checkThrusters[i];
            bwdThrustTotal += thrustValues[i];
            break;
        case Base6Directions.Direction.Left:
            leftThruster = checkThrusters[i];
            leftThrustTotal += thrustValues[i];
            break;
        case Base6Directions.Direction.Right:
            rightThruster = checkThrusters[i];
            rightThrustTotal += thrustValues[i];
            break;
        case Base6Directions.Direction.Up:
            upThruster = checkThrusters[i];
            upThrustTotal += thrustValues[i];
            break;
        case Base6Directions.Direction.Down:
            downThruster = checkThrusters[i];
            downThrustTotal += thrustValues[i];
            break;
        }
    }

    List<IMyTerminalBlock> highestThrustReferences = new List<IMyTerminalBlock>(2);

    float highestThrust = Math.Max(Math.Max(Math.Max(Math.Max(Math.Max(fwdThrustTotal, bwdThrustTotal), leftThrustTotal), rightThrustTotal), upThrustTotal), downThrustTotal);
    if (fwdThrustTotal == highestThrust && fwdThruster != null)
    {
        highestThrustReferences.Add(fwdThruster);
    }
    if (bwdThrustTotal == highestThrust && bwdThruster != null)
    {
        highestThrustReferences.Add(bwdThruster);
    }
    if (leftThrustTotal == highestThrust && leftThruster != null)
    {
        highestThrustReferences.Add(leftThruster);
    }
    if (rightThrustTotal == highestThrust && rightThruster != null)
    {
        highestThrustReferences.Add(rightThruster);
    }
    if (upThrustTotal == highestThrust && upThruster != null)
    {
        highestThrustReferences.Add(upThruster);
    }
    if (downThrustTotal == highestThrust && downThruster != null)
    {
        highestThrustReferences.Add(downThruster);
    }

    if (highestThrustReferences.Count == 1)
    {
        return highestThrustReferences[0];
    }
    else
    {
        Vector3D diagonalVector = ComputeBlockGridDiagonalVector(Me);

        IMyTerminalBlock closestToLengthRef = highestThrustReferences[0];
        double closestToLengthValue = 0;

        for (int i = 0; i < highestThrustReferences.Count; i++)
        {
            double dotLength = Math.Abs(diagonalVector.Dot(highestThrustReferences[i].WorldMatrix.Forward));
            if (dotLength > closestToLengthValue)
            {
                closestToLengthValue = dotLength;
                closestToLengthRef = highestThrustReferences[i];
            }
        }

        return closestToLengthRef;
    }
}

Vector3D ComputeBlockGridDiagonalVector(IMyTerminalBlock block)
{
    IMyCubeGrid cubeGrid = block.CubeGrid;

    Vector3D minVector = cubeGrid.GridIntegerToWorld(cubeGrid.Min);
    Vector3D maxVector = cubeGrid.GridIntegerToWorld(cubeGrid.Max);

    return (minVector - maxVector);
}

Vector3D ComputeBlockGridMidPoint(IMyTerminalBlock block)
{
    return (block.CubeGrid.GridIntegerToWorld(block.CubeGrid.Min) + block.CubeGrid.GridIntegerToWorld(block.CubeGrid.Max)) / 2;
}

//------------------------------ Thruster Control Methods ------------------------------

void FireThrusters(List<IMyThrust> listThrusters, bool overrideMode)
{
    if (listThrusters != null)
    {
        for (int i = 0; i < listThrusters.Count; i++)
        {
            listThrusters[i].SetValue("Override", (overrideMode ? listThrusters[i].GetMaximum<float>("Override") : 0f));
        }
    }
}

void AdjustThrusters(List<IMyThrust> listThrusters, float scale)
{
    if (listThrusters != null)
    {
        for (int i = 0; i < listThrusters.Count; i++)
        {
            listThrusters[i].SetValue("Override", listThrusters[i].GetMaximum<float>("Override") * scale);
        }
    }
}

//------------------------------ Block Finder ------------------------------

List<IMyTerminalBlock> GetBlocksWithName<T>(string name, int matchType = 0) where T: class, IMyTerminalBlock
{
    List<IMyTerminalBlock> blocks = new List<IMyTerminalBlock>();
    GridTerminalSystem.SearchBlocksOfName(name, blocks);

    List<IMyTerminalBlock> filteredBlocks = new List<IMyTerminalBlock>();
    for (int i = 0; i < blocks.Count; i++)
    {
        if (matchType > 0)
        {
            bool isMatch = false;

            switch (matchType)
            {
                case 1:
                    if (blocks[i].CustomName.StartsWith(name, StringComparison.OrdinalIgnoreCase))
                    {
                        isMatch = true;
                    }
                    break;
                case 2:
                    if (blocks[i].CustomName.EndsWith(name, StringComparison.OrdinalIgnoreCase))
                    {
                        isMatch = true;
                    }
                    break;
                case 3:
                    if (blocks[i].CustomName.Equals(name, StringComparison.OrdinalIgnoreCase))
                    {
                        isMatch = true;
                    }
                    break;
                default:
                    isMatch = true;
                    break;
            }

            if (!isMatch)
            {
                continue;
            }
        }

        IMyTerminalBlock block = blocks[i] as T;
        if (block != null)
        {
            filteredBlocks.Add(block);
        }
    }

    return filteredBlocks;
}

//------------------------------ Custom Classes ------------------------------

public class GyroControl
{
    string[] profiles = {"Yaw","Yaw","Pitch","Pitch","Roll","Roll"};

    List<IMyGyro> gyros;

    private byte[] gyroYaw;
    private byte[] gyroPitch;
    private byte[] gyroRoll;

    public GyroControl(List<IMyTerminalBlock> newGyros, ref MatrixD refWorldMatrix)
    {
        gyros = new List<IMyGyro>(newGyros.Count);

        gyroYaw = new byte[newGyros.Count];
        gyroPitch = new byte[newGyros.Count];
        gyroRoll = new byte[newGyros.Count];

        int index = 0;
        foreach (IMyTerminalBlock block in newGyros)
        {
            IMyGyro gyro = block as IMyGyro;
            if (gyro != null)
            {
                gyroYaw[index] = SetRelativeDirection(gyro.WorldMatrix.GetClosestDirection(refWorldMatrix.Up));
                gyroPitch[index] = SetRelativeDirection(gyro.WorldMatrix.GetClosestDirection(refWorldMatrix.Left));
                gyroRoll[index] = SetRelativeDirection(gyro.WorldMatrix.GetClosestDirection(refWorldMatrix.Forward));

                gyros.Add(gyro);

                index++;
            }
        }

    }

    public byte SetRelativeDirection(Base6Directions.Direction dir)
    {
        switch (dir)
        {
            case Base6Directions.Direction.Up:
                return 0;
            case Base6Directions.Direction.Down:
                return 1;
            case Base6Directions.Direction.Left:
                return 2;
            case Base6Directions.Direction.Right:
                return 3;
            case Base6Directions.Direction.Forward:
                return 5;
            case Base6Directions.Direction.Backward:
                return 4;
        }
        return 0;
    }

    public void Enabled(bool enabled)
    {
        foreach (IMyGyro gyro in gyros)
        {
            gyro.Enabled = enabled;
        }
    }

    public void SetGyroOverride(bool bOverride)
    {
        foreach (IMyGyro gyro in gyros)
        {
            gyro.GyroOverride = bOverride;
        }
    }

    public void SetGyroYaw(float yawRate)
    {
        for (int i = 0; i < gyros.Count; i++)
        {
            byte index = gyroYaw[i];
            gyros[i].SetValue(profiles[index], (index % 2 == 0 ? yawRate : -yawRate));
        }
    }

    public void SetGyroPitch(float pitchRate)
    {
        for (int i = 0; i < gyros.Count; i++)
        {
            byte index = gyroPitch[i];
            gyros[i].SetValue(profiles[index], (index % 2 == 0 ? pitchRate : -pitchRate));
        }
    }

    public void SetGyroRoll(float rollRate)
    {
        for (int i = 0; i < gyros.Count; i++)
        {
            byte index = gyroRoll[i];
            gyros[i].SetValue(profiles[index], (index % 2 == 0 ? rollRate : -rollRate));
        }
    }

    public void ZeroTurnGyro()
    {
        for (int i = 0; i < gyros.Count; i++)
        {
            gyros[i].SetValue(profiles[gyroYaw[i]], 0f);
            gyros[i].SetValue(profiles[gyroPitch[i]], 0f);
        }
    }

    public void ResetGyro()
    {
        foreach (IMyGyro gyro in gyros)
        {
            gyro.SetValue("Yaw", 0f);
            gyro.SetValue("Pitch", 0f);
            gyro.SetValue("Roll", 0f);
        }
    }
}

public class PIDController
{
    double integral;
    double lastInput;

    double gain_p;
    double gain_i;
    double gain_d;
    double upperLimit_i;
    double lowerLimit_i;
    double second;

    public PIDController(double pGain, double iGain, double dGain, double iUpperLimit = 0, double iLowerLimit = 0, float stepsPerSecond = 60f)
    {
        gain_p = pGain;
        gain_i = iGain;
        gain_d = dGain;
        upperLimit_i = iUpperLimit;
        lowerLimit_i = iLowerLimit;
        second = stepsPerSecond;
    }

    public double Filter(double input, int round_d_digits)
    {
        double roundedInput = Math.Round(input, round_d_digits);

        integral = integral + (input / second);
        integral = (upperLimit_i > 0 && integral > upperLimit_i ? upperLimit_i : integral);
        integral = (lowerLimit_i < 0 && integral < lowerLimit_i ? lowerLimit_i : integral);

        double derivative = (roundedInput - lastInput) * second;
        lastInput = roundedInput;

        return (gain_p * input) + (gain_i * integral) + (gain_d * derivative);
    }

    public void Reset()
    {
        integral = lastInput = 0;
    }
}

public class ProximitySensor
{
    public IMyCameraBlock lidar;

    public float yaw;
    public float pitch;
    public bool failunsafe;
    public bool failasgroup;
    public double distance;
    public double dmsrange;
    public string failunsafeTriggerCommands;
    public string proximityTriggerCommands;

    public bool dmsActive;
    public double dmsPrevDist;

    public ProximitySensor(IMyCameraBlock inputLidar)
    {
        lidar = inputLidar;
    }
}

public class CustomConfiguration
{
    public IMyTerminalBlock configBlock;
    public Dictionary<string, string> config;

    public CustomConfiguration(IMyTerminalBlock block)
    {
        configBlock = block;
        config = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
    }

    public void Load() { ParseCustomData(configBlock, config); }

    public void Save()
    {
        WriteCustomData(configBlock, config);
    }

    public string Get(string key, string defVal = null)
    {
        return config.GetValueOrDefault(key.Trim(), defVal);
    }

    public void Get(string key, ref string res)
    {
        string val;
        if (config.TryGetValue(key.Trim(), out val)) res = val;
    }

    public void Get(string key, ref int res)
    {
        int val;
        if (int.TryParse(Get(key), out val)) res = val;
    }

    public void Get(string key, ref float res)
    {
        float val;
        if (float.TryParse(Get(key), out val)) res = val;
    }

    public void Get(string key, ref double res)
    {
        double val;
        if (double.TryParse(Get(key), out val)) res = val;
    }

    public void Get(string key, ref bool res)
    {
        bool val;
        if (bool.TryParse(Get(key), out val)) res = val;
    }
    public void Get(string key, ref bool? res)
    {
        bool val;
        if (bool.TryParse(Get(key), out val)) res = val;
    }

    public void Set(string key, string value)
    {
        config[key.Trim()] = value;
    }

    public static void ParseCustomData(IMyTerminalBlock block, Dictionary<string, string> cfg, bool clr = true)
    {
        if (clr)
        {
            cfg.Clear();
        }

        string[] arr = block.CustomData.Split(new char[] {'\r','\n'}, StringSplitOptions.RemoveEmptyEntries);
        for (int i = 0; i < arr.Length; i++)
        {
            string ln = arr[i];
            string va;

            int p = ln.IndexOf('=');
            if (p > -1)
            {
                va = ln.Substring(p + 1);
                ln = ln.Substring(0, p);
            }
            else
            {
                va = "";
            }
            cfg[ln.Trim()] = va.Trim();
        }
    }

    public static void WriteCustomData(IMyTerminalBlock block, Dictionary<string, string> cfg)
    {
        StringBuilder sb = new StringBuilder(cfg.Count * 100);
        foreach (KeyValuePair<string, string> va in cfg)
        {
            sb.Append(va.Key).Append('=').Append(va.Value).Append('\n');
        }
        block.CustomData = sb.ToString();
    }
}