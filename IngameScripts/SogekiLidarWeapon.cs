//============================================================
// Script powering my Stalker Skyfighter and Spacefighter.
// Uses iterative solver for rockets to allow extremely
// accurate intercept despite rocket speed limit.
//============================================================

//------------------------------------------------------------
// ADN - Sogeki Lidar Weapon Script v3.4
//------------------------------------------------------------

const string LIDAR_BLOCK_TAG = "Lidar";                 //Tag of Cameras to be used as Lidars
const string AIM_POINT_BLOCK_TAG = "Ship Cockpit";      //Tag of Block to use as forward facing reference. If not found it will auto detect one
const string GYROSCOPE_NAME_TAG = "Ship Gyroscope";     //Tag of Gyroscope used for aiming. It will use all gyroscopes on grid if not found
const string STATUS_DISPLAY_PANEL_TAG = "ST_STATUS";    //Tag of the Status Display Text Panel (Optional)

const string REMOTE_CONTROL_NAME = "Ship Remote";       //Name of Remote Control Block for ship information

const string TARGET_GPS_PANEL = "R_TARGET";             //Name of Text Panel to output target information

const string LOCKED_TRIGGER_BLOCK = "ST_ALERT";         //Name of Block to trigger when lock is stabilized (Alert selected)
const string LOCKED_TRIGGER_ACTION = "PlaySound";       //Name of Action to trigger when lock is stabilized (Alert selected)

const string STATE_DISPLAY_PB_TAG = "Projector Alert Computer"; //Name of Programmable Block to handle Targeting State Display
const string STATE_DISPLAY_PB_ARG_PREFIX = "SIGNAL:";   //Arguments prefix to be passed to the State Display PB together with the state number

const string ROCKET_LOCKED_FIRE_TAG = "Rocket Launcher";//Name of Block to trigger rockets when lock is stabilized (Fire selected)
const string ROCKET_LOCKED_FIRE_ACTION = "ShootOnce";   //Name of Action to trigger rockets when lock is stabilized (Fire selected)

const string GATLING_LOCKED_FIRE_TAG = "Gatling Gun";   //Name of Block to trigger gatlings when lock is stabilized (Fire selected)
const string GATLING_LOCKED_FIRE_ACTION = "ShootOnce";  //Name of Action to trigger gatlings when lock is stabilized (Fire selected)

//------------------------------ Projectile Parameters Configuration ------------------------------

double SHOOT_AHEAD_TIME = 0;                            //Time in seconds to shoot ahead
double SHOOT_AHEAD_DISTANCE = 0;                        //Distance in meters to shoot ahead

double ROCKET_PROJECTILE_FORWARD_OFFSET = 4;            //If rocket launcher's tip is not flushed with the AIM_POINT_BLOCK_TAG, use this to offset the distance. By default, rockets are spawn 4 meters in front of the rocket launcher's tip
double ROCKET_PROJECTILE_INITIAL_SPEED = 100;           //Initial speed of the projectile at launch in m/s (Rocket)
double ROCKET_PROJECTILE_ACCELERATION = 600;            //Acceleration of the projectile in m/s^2 (Rocket)
double ROCKET_PROJECTILE_MAX_SPEED = 200;               //Maximum speed of the projectile in m/s (Rocket)
double ROCKET_PROJECTILE_MAX_RANGE = 800;               //Maximum range of the projectile in meters (Rocket)

double GATLING_PROJECTILE_FORWARD_OFFSET = 0;           //If gatling gun's tip is not flushed with the AIM_POINT_BLOCK_TAG, use this to offset the distance
double GATLING_PROJECTILE_INITIAL_SPEED = 400;          //Initial speed of the projectile at launch in m/s (Gatling)
double GATLING_PROJECTILE_ACCELERATION = 0;             //Acceleration of the projectile in m/s^2 (Gatling)
double GATLING_PROJECTILE_MAX_SPEED = 400;              //Maximum speed of the projectile in m/s (Gatling)
double GATLING_PROJECTILE_MAX_RANGE = 800;              //Maximum range of the projectile in meters (Gatling)

int AIM_STABILIZE_TICKS = 1;                            //Number of 1/60 seconds of constant stabilizeAngle before firing weapon
double AIM_STABILIZE_ANGLE = 0.2;                       //Angle in degrees to consider aiming stabilized

//------------------------------ Lock On Configuration ------------------------------

float LIDAR_MIN_LOCK_DISTANCE = 50;                     //Minimum distance allowed for Lidar to lock-on
float LIDAR_MAX_LOCK_DISTANCE = 3000;                   //Maximum distance allowed for Lidar to lock-on

int LIDAR_REFRESH_INTERVAL = 0;                         //Number of ticks before next Lidar lock-on attempt is triggered. Zero for autodetect
int LIDAR_LOCK_LOST_TICKS = 15000000;                   //Number of ticks to declare target is lost

float LIDAR_REFRESH_CALC_FACTOR = 0.95f;                //Estimated ratio of lidars to utilize

bool excludeFriendly = false;                           //Whether to exclude locking on to friendlies

float fivePointInitialLockDist = 0f;                    //The amount of spread for 5 point raycast for easier initial lock-on. Zero to use normal single beam lock-on

int raycastAheadCount = 0;                              //Raycast ahead of of your initial aiming, to mitigate high ping and gyro sync lag. Zero to use normal single beam lock-on
float raycastAheadSeconds = 0f;

bool checkSelfOcclusion = false;                        //Perform additional checks to ensure Raycast does not hit your own ship due to camera view being blocked

//------------------------------ Above Is User Configuration Section. This Section Is For PID Tuning ------------------------------

const double DEF_SMALL_GRID_P = 40;                     //The default proportional gain of small grid gyroscopes
const double DEF_SMALL_GRID_I = 0;                      //The default integral gain of small grid gyroscopes
const double DEF_SMALL_GRID_D = 13;                     //The default derivative gain of small grid gyroscopes

const double DEF_BIG_GRID_P = 15;                       //The default proportional gain of large grid gyroscopes
const double DEF_BIG_GRID_I = 0;                        //The default integral gain of large grid gyroscopes
const double DEF_BIG_GRID_D = 7;                        //The default derivative gain of large grid gyroscopes

bool useDefaultPIDValues = true;                        //Whether to use predefined PID values based on detected grid size

double AIM_P = 0;                                       //The proportional gain of the gyroscope turning (Set useDefaultPIDValues to true to use default values)
double AIM_I = 0;                                       //The integral gain of the gyroscope turning (Set useDefaultPIDValues to true to use default values)
double AIM_D = 0;                                       //The derivative gain of the gyroscope turning (Set useDefaultPIDValues to true to use default values)
double AIM_LIMIT = 60;                                  //Limit value of both yaw and pitch combined

double INTEGRAL_WINDUP_LIMIT = 0;                       //Integral value limit to minimize integral windup. Zero means no limit

//------------------------------ Below Is Main Script Body ------------------------------

IMyRemoteControl remoteControl;

List<IMyCameraBlock> aimLidars;
IMyTerminalBlock aimPointBlock;

List<IMyShipController> cockpits;
IMyShipController controlledCockpit;

GyroControl gyroControl;

PIDController yawController;
PIDController pitchController;
PIDController rollController;

IMyTextPanel targetPanel;

IMyTerminalBlock alertBlock;
bool alertTriggered;

IMyProgrammableBlock stateDisplayPB;
int currentDisplayState = -1;

List<IMyTerminalBlock> fireBlockRockets;
List<IMyTerminalBlock> fireBlockGatlings;

bool isFire;
bool isAim;

int weaponType;
bool useOffsetTargeting;

IMyTextPanel statusPanel;
IMyTextPanel statusPanelExt;
bool needStatusUpdate;
string statusMessage = "";

MatrixD refWorldMatrix;
MatrixD refLookAtMatrix;

MyDetectedEntityInfo lidarTargetInfo;

Vector3D lidarLastForward;
Vector3D lidarAimChangeVector;
long lidarLastForwardClock;

Vector3D relativeHitPosition;

Vector3D lidarTargetAcceleration;
double lidarTargetDistance;
double lidarTargetSpeed;
double lidarTargetRadius;
long lastLidarTargetClock;

bool lastTargetFound;

double angleToTarget;

int subCounter = 0;
int subMode = 0;
int mode = 0;
int prevMode = 0;
int clock = 0;
bool init = false;

Random rnd = new Random();

float coneLimit = 0;
double sideScale = 0;
double ticksRatio = 0;
double ticksFactor = 0;

int lidarStaggerIndex = 0;

long nextLidarTriggerTicks = 0;
long nextLidarRecountTicks = 0;

CubeTree occlusionChecker;
IEnumerator<int> iterOcclusionCreator;

const double DIST_FACTOR = Math.PI / 2;
const double RPM_FACTOR = 1800 / Math.PI;
const double ACOS_FACTOR = 180 / Math.PI;
const float GYRO_FACTOR = (float)(Math.PI / 30);
const double RADIAN_FACTOR = Math.PI / 180;

const int MIN_RECOUNT_TICKS = 15;
const float SECOND = 60f;

Program()
{
    Runtime.UpdateFrequency = UpdateFrequency.Update1;
}

void Main(string arguments, UpdateType updateSource)
{
    if (!init)
    {
        if (subMode == 0)
        {
            if (Me.CustomData.Length > 0)
            {
                ProcessCustomConfiguration();
            }

            InitScript(arguments);

            if (raycastAheadCount > 0 && raycastAheadSeconds == 0f)
            {
                raycastAheadSeconds = 0.5f;
            }

            if (checkSelfOcclusion)
            {
                occlusionChecker = new CubeTree();
                iterOcclusionCreator = occlusionChecker.InitTree(Me.CubeGrid, Me.Position, 1000);

                subMode = 1;
            }
        }

        if (subMode == 1)
        {
            if (iterOcclusionCreator.MoveNext())
            {
                Echo("--- Creating Occlusion Checker ---\nBlocks Processed:" + iterOcclusionCreator.Current);
                return;
            }
            else
            {
                iterOcclusionCreator.Dispose();
                iterOcclusionCreator = null;
            }
        }

        mode = subMode = subCounter = 0;
        clock = 0;

        prevMode = -1;

        init = true;
        return;
    }

    if (arguments.Length > 0)
    {
        ProcessCommand(arguments);
    }

    if ((updateSource & UpdateType.Update1) == 0 || Runtime.TimeSinceLastRun.Ticks == 0)
    {
        return;
    }

    clock++;

    ProcessModeChange();

    bool isUnreachable = true;

    if (mode == 1)
    {
        if (nextLidarTriggerTicks <= clock)
        {
            refWorldMatrix = aimPointBlock.WorldMatrix;
            Vector3D forwardVector = refWorldMatrix.Forward;
            Vector3D targetPosition = refWorldMatrix.Translation + (forwardVector * LIDAR_MAX_LOCK_DISTANCE);

            int lidarCount = raycastAheadCount > 0 ? raycastAheadCount + 1 : (fivePointInitialLockDist > 0 ? 5 : 1);
            List<IMyCameraBlock> lidars = GetLidarsAndRecountTicks(aimLidars, ref targetPosition, 0, lidarStaggerIndex++, lidarCount, ref refWorldMatrix);
            if (lidars.Count >= lidarCount)
            {
                Vector3D?[] refPoints = SpreadRaycastPoint(ref targetPosition, ref forwardVector, LIDAR_MAX_LOCK_DISTANCE, lidarCount);
                Vector3D[] refVectors = null;
                if (lidars.Count > 1)
                {
                    refVectors = new Vector3D[] { targetPosition, refWorldMatrix.Up, refWorldMatrix.Left };
                }

                for (int i = 0; i < lidars.Count; i++)
                {
                    MyDetectedEntityInfo entityInfo = lidars[i].Raycast(refPoints[i].Value);
                    if (!entityInfo.IsEmpty())
                    {
                        double targetDistance = Vector3D.Distance(entityInfo.Position, aimPointBlock.GetPosition());
                        if (targetDistance > LIDAR_MIN_LOCK_DISTANCE)
                        {
                            if (IsValidLidarTarget(ref entityInfo))
                            {
                                lidarTargetDistance = targetDistance;
                                lidarTargetSpeed = entityInfo.Velocity.Length();
                                lidarTargetRadius = Vector3D.Distance(entityInfo.BoundingBox.Min, entityInfo.BoundingBox.Max);
                                lidarTargetAcceleration = new Vector3D();

                                if (entityInfo.HitPosition != null)
                                {
                                    relativeHitPosition = Vector3D.Transform(entityInfo.HitPosition.Value - entityInfo.Position, MatrixD.Invert(entityInfo.Orientation));
                                }
                                else
                                {
                                    relativeHitPosition = Vector3D.Zero;
                                }

                                lidarTargetInfo = entityInfo;
                                lastLidarTargetClock = clock;

                                subCounter = 0;

                                statusMessage = "";
                                mode = 2;

                                ProcessModeChange();

                                lastTargetFound = true;

                                break;
                            }
                        }
                    }
                }
            }
        }
    }
    else if (mode == 2)
    {
        refWorldMatrix = aimPointBlock.WorldMatrix;

        Vector3D targetPosition = lidarTargetInfo.Position + (lidarTargetInfo.Velocity / SECOND * (clock - lastLidarTargetClock));

        if (useOffsetTargeting)
        {
            targetPosition += Vector3D.Transform(relativeHitPosition, lidarTargetInfo.Orientation);
        }

        if (nextLidarTriggerTicks <= clock)
        {
            bool targetFound = false;

            double overshootDistance = lidarTargetRadius / 2;

            IMyCameraBlock aimLidar = GetLidarAndRecountTicks(aimLidars, ref targetPosition, overshootDistance, lidarStaggerIndex++, ref refWorldMatrix);
            if (aimLidar != null)
            {
                Vector3D testTargetPosition = targetPosition + (Vector3D.Normalize(targetPosition - aimLidar.GetPosition()) * overshootDistance);

                MyDetectedEntityInfo entityInfo = aimLidar.Raycast(testTargetPosition);
                if (!entityInfo.IsEmpty())
                {
                    if (entityInfo.EntityId == lidarTargetInfo.EntityId)
                    {
                        lidarTargetDistance = Vector3D.Distance(entityInfo.Position, aimPointBlock.GetPosition());
                        lidarTargetSpeed = entityInfo.Velocity.Length();
                        lidarTargetRadius = Vector3D.Distance(entityInfo.BoundingBox.Min, entityInfo.BoundingBox.Max);
                        lidarTargetAcceleration = (entityInfo.Velocity - lidarTargetInfo.Velocity) * SECOND / (clock - lastLidarTargetClock);

                        lidarTargetInfo = entityInfo;
                        lastLidarTargetClock = clock;

                        targetPosition = entityInfo.Position;

                        if (useOffsetTargeting)
                        {
                            targetPosition += Vector3D.Transform(relativeHitPosition, lidarTargetInfo.Orientation);
                        }

                        targetFound = true;
                    }
                }
            }

            lastTargetFound = targetFound;
        }

        if (targetPanel != null)
        {
            targetPanel.WritePublicTitle("GPS:Sogeki:" + Math.Round(targetPosition.X, 2) + ":" + Math.Round(targetPosition.Y, 2) + ":" + Math.Round(targetPosition.Z, 2) + ":");
        }

        Vector3D aimDirection;
        double travelDistance;

        if (subMode == 0)
        {
            if (SHOOT_AHEAD_TIME > 0 || SHOOT_AHEAD_DISTANCE > 0)
            {
                targetPosition += (new Vector3D(lidarTargetInfo.Velocity) * SHOOT_AHEAD_TIME) + (SHOOT_AHEAD_DISTANCE == 0 ? Vector3D.Zero : Vector3D.Normalize(lidarTargetInfo.Velocity) * SHOOT_AHEAD_DISTANCE);
            }

            switch (weaponType)
            {
                case 0:
                    aimDirection = ComputeInterceptPointWithInheritSpeed(targetPosition, lidarTargetInfo.Velocity, (ROCKET_PROJECTILE_FORWARD_OFFSET == 0 ? refWorldMatrix.Translation : refWorldMatrix.Translation + (refWorldMatrix.Forward * ROCKET_PROJECTILE_FORWARD_OFFSET)), remoteControl.GetShipVelocities().LinearVelocity, ROCKET_PROJECTILE_INITIAL_SPEED, ROCKET_PROJECTILE_ACCELERATION, ROCKET_PROJECTILE_MAX_SPEED, ROCKET_PROJECTILE_MAX_RANGE);
                    break;
                default:
                    aimDirection = ComputeInterceptPoint(targetPosition, lidarTargetInfo.Velocity - remoteControl.GetShipVelocities().LinearVelocity, (GATLING_PROJECTILE_FORWARD_OFFSET == 0 ? refWorldMatrix.Translation : refWorldMatrix.Translation + (refWorldMatrix.Forward * GATLING_PROJECTILE_FORWARD_OFFSET)), GATLING_PROJECTILE_INITIAL_SPEED, GATLING_PROJECTILE_ACCELERATION, GATLING_PROJECTILE_MAX_SPEED);
                    break;
            }
            aimDirection = aimDirection - refWorldMatrix.Translation;

            travelDistance = aimDirection.Length();
        }
        else
        {
            aimDirection = targetPosition - refWorldMatrix.Translation;
            travelDistance = lidarTargetDistance;
        }

        if (Double.IsNaN(aimDirection.Sum))
        {
            isUnreachable = true;

            aimDirection = targetPosition - refWorldMatrix.Translation;

            subCounter = 0;
        }
        else
        {
            isUnreachable = false;

            aimDirection = Vector3D.Normalize(aimDirection);
            angleToTarget = Math.Acos(Math.Min(refWorldMatrix.Forward.Dot(aimDirection), 1));

            double maxWeaponRange;
            switch (weaponType)
            {
                case 0:
                    maxWeaponRange = ROCKET_PROJECTILE_MAX_RANGE;
                    break;
                default:
                    maxWeaponRange = GATLING_PROJECTILE_MAX_RANGE;
                    break;
            }

            if ((angleToTarget * ACOS_FACTOR) <= AIM_STABILIZE_ANGLE && (maxWeaponRange == 0 || travelDistance <= maxWeaponRange))
            {
                subCounter += 1;
            }
            else
            {
                subCounter = 0;
            }
        }

        refLookAtMatrix = MatrixD.CreateLookAt(Vector3D.Zero, refWorldMatrix.Forward, refWorldMatrix.Up);
        aimDirection = Vector3D.Normalize(Vector3D.TransformNormal(aimDirection, refLookAtMatrix));

        AimAtTarget(aimDirection);

        if (!controlledCockpit.IsUnderControl)
        {
            for (int i = 0; i < cockpits.Count; i++)
            {
                if (cockpits[i].IsUnderControl)
                {
                    controlledCockpit = cockpits[i];
                    break;
                }
            }
        }
        gyroControl.SetGyroRoll(controlledCockpit.RollIndicator * -30);

        if (lastLidarTargetClock + LIDAR_LOCK_LOST_TICKS <= clock)
        {
            statusMessage = "Target Lost";
            subCounter = 0;
            mode = 0;
        }
    }

    if (mode == 2)
    {
        if (isFire)
        {
            if (fireBlockRockets != null)
            {
                if (subCounter >= AIM_STABILIZE_TICKS)
                {
                    switch (weaponType)
                    {
                        case 0:
                            TriggerLockFireRockets();
                            break;
                        default:
                            TriggerLockFireGatlings();
                            break;
                    }
                }
            }
        }
        else
        {
            if (alertBlock != null)
            {
                if (subCounter >= AIM_STABILIZE_TICKS)
                {
                    if (!alertTriggered)
                    {
                        TriggerLockAlert();
                        alertTriggered = true;
                    }
                }
                else
                {
                    alertTriggered = false;
                }
            }
        }
    }

    if (stateDisplayPB != null)
    {
        int displayState = (mode == 2 ? (subCounter >= AIM_STABILIZE_TICKS ? 3 : 2) : mode);
        if (currentDisplayState != displayState)
        {
            currentDisplayState = displayState;
            stateDisplayPB.TryRun(STATE_DISPLAY_PB_ARG_PREFIX + displayState);
        }
    }

    if (statusPanel != null && (needStatusUpdate || clock % 15 == 0))
    {
        needStatusUpdate = false;

        StringBuilder sb = new StringBuilder();

        if (mode == 0)
        {
            sb.Append(" Status: Idle").AppendLine(statusMessage.Length > 0 ? " - " + statusMessage : "");
        }
        else if (mode == 1)
        {
            sb.AppendLine(" Status: Finding");
        }
        else if (mode == 2)
        {
            sb.Append(" Status: ").AppendLine(subCounter >= AIM_STABILIZE_TICKS ? "Aim Locked" : "Aiming");
        }

        string leadType;
        switch (weaponType)
        {
            case 0:
                leadType = "Rocket";
                break;
            default:
                leadType = "Gatling";
                break;
        }

        if (subMode == 0)
        {
            sb.Append(" Type: ").Append(useOffsetTargeting ? "Offset " : "").Append("Lead ").AppendLine(leadType);
        }
        else if (subMode == 1)
        {
            sb.AppendLine(" Type: Track Target");
        }

        sb.Append(" Auto Aim: ").AppendLine(isAim ? "ENGAGED" : "-");
        sb.Append(" Auto Fire: ").AppendLine(isFire ? "ENGAGED" : "-");

        if (mode == 2)
        {
            if (statusPanelExt != null)
            {
                statusPanel.WritePublicText(sb);
                sb.Clear();
            }
            else
            {
                sb.AppendLine();
            }

            sb.Append(" Error: ").AppendLine(isUnreachable ? "[X]" : Math.Round(Math.Tan(angleToTarget) * lidarTargetDistance, 2).ToString());
            sb.Append(" Lock: ").AppendLine(lastTargetFound ? "Stable" : "Unstable");
            sb.Append("\n Distance: ").AppendLine(Math.Round(lidarTargetDistance, 2).ToString());
            sb.Append(" Speed: ").AppendLine(Math.Round(lidarTargetSpeed, 2).ToString());
            sb.Append(" Radius: ").AppendLine(Math.Round(lidarTargetRadius, 2).ToString());

            if (statusPanelExt != null)
            {
                statusPanelExt.WritePublicText(sb);
            }
            else
            {
                statusPanel.WritePublicText(sb);
            }
        }
        else
        {
            statusPanel.WritePublicText(sb);
            if (statusPanelExt != null)
            {
                statusPanelExt.WritePublicText("");
            }
        }
    }
}

void ProcessModeChange()
{
    if (prevMode != mode)
    {
        prevMode = mode;

        if (mode == 0)
        {
            gyroControl.SetGyroOverride(false);

            if (targetPanel != null)
            {
                targetPanel.WritePublicTitle("");
            }
        }
        else if (mode == 1)
        {
            gyroControl.SetGyroOverride(false);
        }
        else if (mode == 2)
        {
            if (isAim)
            {
                gyroControl.ResetGyro();
                gyroControl.SetGyroOverride(true);
            }
            else
            {
                gyroControl.SetGyroOverride(false);
            }

            alertTriggered = false;
        }

        needStatusUpdate = true;
    }
}

void ProcessCommand(string command)
{
    if (command.Equals("TOGGLE"))
    {
        if (mode == 0)
        {
            mode = 1;
        }
        else
        {
            mode = 0;
        }
    }
    else if (command.Equals("LOCK"))
    {
        if (mode == 0)
        {
            mode = 1;
        }
    }
    else if (command.Equals("LEAD"))
    {
        if (mode == 0)
        {
            mode = 1;
        }
        subMode = 0;
    }
    else if (command.Equals("TRACK"))
    {
        if (mode == 0)
        {
            mode = 1;
        }
        subMode = 1;
    }
    else if (command.Equals("UNLOCK"))
    {
        mode = 0;
    }
    else if (command.Equals("FIRE"))
    {
        isFire = !isFire;
    }
    else if (command.Equals("AIM"))
    {
        isAim = !isAim;

        if (mode == 2)
        {
            if (isAim)
            {
                gyroControl.ResetGyro();
                gyroControl.SetGyroOverride(true);
            }
            else
            {
                gyroControl.SetGyroOverride(false);
            }
        }
    }
    else if (command.Equals("ROCKET"))
    {
        weaponType = 0;
    }
    else if (command.Equals("GATLING"))
    {
        weaponType = 1;
    }
    else if (command.Equals("WEAPONSWITCH"))
    {
        weaponType = (weaponType == 0 ? 1 : 0);
    }
    else if (command.Equals("OFFSET"))
    {
        useOffsetTargeting = !useOffsetTargeting;
    }

    statusMessage = "";
    needStatusUpdate = true;
}

void TriggerLockAlert()
{
    if (alertBlock.HasAction(LOCKED_TRIGGER_ACTION))
    {
        alertBlock.ApplyAction(LOCKED_TRIGGER_ACTION);
    }
}

void TriggerLockFireRockets()
{
    for (int i = 0; i < fireBlockRockets.Count; i++)
    {
        if (fireBlockRockets[i].HasAction(ROCKET_LOCKED_FIRE_ACTION))
        {
            fireBlockRockets[i].ApplyAction(ROCKET_LOCKED_FIRE_ACTION);
        }
    }
}

void TriggerLockFireGatlings()
{
    for (int i = 0; i < fireBlockGatlings.Count; i++)
    {
        if (fireBlockGatlings[i].HasAction(GATLING_LOCKED_FIRE_ACTION))
        {
            fireBlockGatlings[i].ApplyAction(GATLING_LOCKED_FIRE_ACTION);
        }
    }
}

Vector3D?[] SpreadRaycastPoint(ref Vector3D targetPoint, ref Vector3D forwardVector, float distance, int lidarCount)
{
    Vector3D?[] refPoints;

    if (lidarCount > 1)
    {
        if (raycastAheadCount > 0)
        {
            lidarAimChangeVector = (forwardVector * distance) - lidarLastForward;
            lidarAimChangeVector = lidarAimChangeVector / (clock - lidarLastForwardClock) * SECOND * raycastAheadSeconds / raycastAheadCount;
            lidarLastForward = (forwardVector * distance);
            lidarLastForwardClock = clock;

            refPoints = new Vector3D?[lidarCount];
            for (int i = 0; i < lidarCount; i++)
            {
                refPoints[i] = targetPoint + (lidarAimChangeVector * i);
            }
        }
        else
        {
            MatrixD matrix = MatrixD.CreateFromDir(forwardVector);
            refPoints = new Vector3D?[5] { targetPoint,
                                            targetPoint + (matrix.Up * fivePointInitialLockDist),
                                            targetPoint + (matrix.Up * -fivePointInitialLockDist),
                                            targetPoint + (matrix.Left * fivePointInitialLockDist),
                                            targetPoint + (matrix.Left * -fivePointInitialLockDist) };
        }
    }
    else
    {
        refPoints = new Vector3D?[1] { targetPoint };
    }

    return refPoints;
}

List<IMyCameraBlock> GetAvailableLidars(List<IMyCameraBlock> lidars, ref Vector3D aimPoint, double overshootDistance, int indexOffset, int lidarCount)
{
    List<IMyCameraBlock> result = new List<IMyCameraBlock>(lidarCount);

    for (int i = 0; i < lidars.Count; i++)
    {
        IMyCameraBlock lidar = lidars[(i + indexOffset) % lidars.Count];

        MatrixD lidarWorldMatrix = lidar.WorldMatrix;
        Vector3D aimVector = aimPoint - lidarWorldMatrix.Translation;
        double distance = aimVector.Length();

        if (lidar.IsWorking && lidar.CanScan(distance + overshootDistance))
        {
            Vector3D scaleLeft = sideScale * lidarWorldMatrix.Left;
            Vector3D scaleUp = sideScale * lidarWorldMatrix.Up;

            if (sideScale >= 0)
            {
                if (aimVector.Dot(lidarWorldMatrix.Forward + scaleLeft) >= 0 &&
                    aimVector.Dot(lidarWorldMatrix.Forward - scaleLeft) >= 0 &&
                    aimVector.Dot(lidarWorldMatrix.Forward + scaleUp) >= 0 &&
                    aimVector.Dot(lidarWorldMatrix.Forward - scaleUp) >= 0)
                {
                    if (occlusionChecker == null || IsNotOccluded(lidar, ref aimVector))
                    {
                        result.Add(lidar);

                        if (result.Count >= lidarCount) break;
                    }
                }
            }
            else
            {
                if (aimVector.Dot(lidarWorldMatrix.Forward + scaleLeft) >= 0 ||
                    aimVector.Dot(lidarWorldMatrix.Forward - scaleLeft) >= 0 ||
                    aimVector.Dot(lidarWorldMatrix.Forward + scaleUp) >= 0 ||
                    aimVector.Dot(lidarWorldMatrix.Forward - scaleUp) >= 0)
                {
                    if (occlusionChecker == null || IsNotOccluded(lidar, ref aimPoint))
                    {
                        result.Add(lidar);

                        if (result.Count >= lidarCount) break;
                    }
                }
            }
        }
    }

    return result;
}

bool IsNotOccluded(IMyTerminalBlock block, ref Vector3D worldAimVector)
{
    Matrix lidarLookAtMatrix = Matrix.CreateLookAt(Vector3D.Zero, block.CubeGrid.WorldMatrix.Forward, block.CubeGrid.WorldMatrix.Up);
    Ray ray = new Ray(block.Position, Vector3.Transform(worldAimVector, lidarLookAtMatrix));
    return !occlusionChecker.TestRayHit(ray);
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

        ticksFactor = ticksRatio / (Math.Max((float)result.Count / count, 1) * LIDAR_REFRESH_CALC_FACTOR);
        nextLidarRecountTicks = clock + MIN_RECOUNT_TICKS;

        if (result.Count > count)
        {
            List<IMyCameraBlock> filtered = new List<IMyCameraBlock>(count);
            for (int i = 0; i < count; i++)
            {
                filtered.Add(result[i]);
            }
            result = filtered;
        }
    }
    else
    {
        result = GetAvailableLidars(lidars, ref aimPoint, overshootDistance, indexOffset, count);
    }

    if (result.Count >= count)
    {
        RecountLidarTicks(Vector3D.Distance(aimPoint, recountRefMatrix.Translation) + overshootDistance, ticksFactor);
    }

    return result;
}

void RecountLidarTicks(double distance, double factor)
{
    if (LIDAR_REFRESH_INTERVAL == 0)
    {
        nextLidarTriggerTicks = clock + (long)Math.Ceiling(distance * factor);
    }
    else
    {
        nextLidarTriggerTicks = clock + LIDAR_REFRESH_INTERVAL;
    }
}

bool IsValidLidarTarget(ref MyDetectedEntityInfo entityInfo)
{
    if (entityInfo.Type != MyDetectedEntityType.Asteroid && entityInfo.Type != MyDetectedEntityType.Planet)
    {
        if (!excludeFriendly || IsNotFriendly(entityInfo.Relationship))
        {
            return true;
        }
    }
    return false;
}

bool IsNotFriendly(VRage.Game.MyRelationsBetweenPlayerAndBlock relationship)
{
    return (relationship != VRage.Game.MyRelationsBetweenPlayerAndBlock.FactionShare && relationship != VRage.Game.MyRelationsBetweenPlayerAndBlock.Owner);
}

Vector3D ComputeInterceptPoint(Vector3D targetLocation, Vector3D targetVelocity, Vector3D projectileLocation, double projectileInitialSpeed, double projectileAcceleration, double projectileMaxSpeed)
{
    //---------- Calculate Impact Point ----------

    //targetDirection Must Be Normalized
    Vector3D z = targetLocation - projectileLocation;
    double k = (projectileAcceleration == 0 ? 0 : (projectileMaxSpeed - projectileInitialSpeed) / projectileAcceleration);
    double p = (0.5 * projectileAcceleration * k * k) + (projectileInitialSpeed * k) - (projectileMaxSpeed * k);

    double a = (projectileMaxSpeed * projectileMaxSpeed) - targetVelocity.LengthSquared();
    double b = 2 * ((p * projectileMaxSpeed) - targetVelocity.Dot(z));
    double c = (p * p) - z.LengthSquared();

    double t = SolveQuadratic(a, b, c);

    if (Double.IsNaN(t) || t < 0)
    {
        return new Vector3D(Double.NaN, Double.NaN, Double.NaN);
    }
    else
    {
        return targetLocation + (targetVelocity * t);
    }
}

Vector3D ComputeInterceptPointWithInheritSpeed(Vector3D targetLocation, Vector3D targetVelocity, Vector3D projectileLocation, Vector3D shipDirection, double projectileInitialSpeed, double projectileAcceleration, double projectileMaxSpeed, double projectileMaxRange)
{
    //---------- Calculate Impact Point ----------

    //targetDirection Must Be Normalized
    Vector3D z = targetLocation - projectileLocation;
    double k = (projectileAcceleration == 0 ? 0 : (projectileMaxSpeed - projectileInitialSpeed) / projectileAcceleration);
    double p = (0.5 * projectileAcceleration * k * k) + (projectileInitialSpeed * k) - (projectileMaxSpeed * k);

    double a = (projectileMaxSpeed * projectileMaxSpeed) - targetVelocity.LengthSquared();
    double b = 2 * ((p * projectileMaxSpeed) - targetVelocity.Dot(z));
    double c = (p * p) - z.LengthSquared();

    double t = SolveQuadratic(a, b, c);

    if (Double.IsNaN(t) || t < 0)
    {
        return new Vector3D(Double.NaN, Double.NaN, Double.NaN);
    }

    int u = (int)Math.Ceiling(t * 60);

    Vector3D targetPoint = targetLocation + (targetVelocity * t);

    Vector3D aimDirection;
    Vector3D stepAcceleration;
    Vector3D currentPosition;
    Vector3D currentDirection;

    aimDirection = Vector3D.Normalize(targetPoint - projectileLocation);
    stepAcceleration = (aimDirection * projectileAcceleration) / 60;

    currentPosition = projectileLocation;
    currentDirection = shipDirection + (aimDirection * projectileInitialSpeed);

    for (int i = 0; i < u; i++)
    {
        currentDirection += stepAcceleration;

        double speed = currentDirection.Length();
        if (speed > projectileMaxSpeed)
        {
            currentDirection = currentDirection / speed * projectileMaxSpeed;
        }

        currentPosition += (currentDirection / 60);

        if ((i + 1) % 60 == 0)
        {
            if (Vector3D.Distance(projectileLocation, currentPosition) > projectileMaxRange)
            {
                return targetPoint;
            }
        }
    }

    return targetPoint + targetPoint - currentPosition;
}

double SolveQuadratic(double a, double b, double c)
{
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

int GetMultiplierSign(double value)
{
    return (value < 0 ? -1 : 1);
}

void AimAtTarget(Vector3D targetVector)
{
    //---------- Activate Gyroscopes To Turn Towards Target ----------

    Vector3D yawVector = new Vector3D(targetVector.GetDim(0), 0, targetVector.GetDim(2));
    Vector3D pitchVector = new Vector3D(0, targetVector.GetDim(1), targetVector.GetDim(2));
    yawVector.Normalize();
    pitchVector.Normalize();

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

//------------------------------ Initialization Methods ------------------------------

void InitScript(string arguments)
{
    aimLidars = GetLidars();
    if (aimLidars == null) throw new Exception("--- Initialization Failed ---");

    aimPointBlock = GetAimPointBlock();
    if (aimPointBlock == null) throw new Exception("--- Initialization Failed ---");

    List<IMyTerminalBlock> gyros = GetGyroscopes();
    if (gyros == null) throw new Exception("--- Initialization Failed ---");

    for (int i = 0; i < aimLidars.Count; i++)
    {
        aimLidars[i].ApplyAction("OnOff_On");
        aimLidars[i].EnableRaycast = true;
    }

    if (aimLidars.Count > 0)
    {
        coneLimit = aimLidars[0].RaycastConeLimit;
        sideScale = Math.Tan((90 - coneLimit) * RADIAN_FACTOR);
        ticksRatio = aimLidars[0].TimeUntilScan(aimLidars[0].AvailableScanRange + 1000) * 0.00006;
        ticksFactor = ticksRatio / Math.Max((int)Math.Floor(aimLidars.Count * LIDAR_REFRESH_CALC_FACTOR), 1);
    }
    else
    {
        coneLimit = 45;
        sideScale = 1;
        ticksRatio = 0.03;
        ticksFactor = 0.03;
    }

    remoteControl = GetSingleBlockWithName(REMOTE_CONTROL_NAME) as IMyRemoteControl;
    if (remoteControl == null) throw new Exception("--- Initialization Failed ---");

    cockpits = GetBlocksOfTypeCasted<IMyShipController>();
    controlledCockpit = remoteControl;

    targetPanel = GetSingleBlockWithName(TARGET_GPS_PANEL) as IMyTextPanel;

    refWorldMatrix = aimPointBlock.WorldMatrix;
    refLookAtMatrix = MatrixD.CreateLookAt(Vector3D.Zero, refWorldMatrix.Forward, refWorldMatrix.Up);

    gyroControl = new GyroControl(gyros, ref refWorldMatrix);

    InitPIDControllers();

    List<IMyTerminalBlock> blocks = GetBlocksWithName<IMyTextPanel>(STATUS_DISPLAY_PANEL_TAG);
    if (blocks.Count > 0)
    {
        if (blocks.Count > 1)
        {
            if (blocks[0].CustomName.IndexOf("EXT", StringComparison.OrdinalIgnoreCase) > -1)
            {
                statusPanelExt = blocks[0] as IMyTextPanel;
                statusPanel = blocks[1] as IMyTextPanel;
            }
            else
            {
                statusPanel = blocks[0] as IMyTextPanel;
                statusPanelExt = blocks[1] as IMyTextPanel;
            }
        }
        else
        {
            statusPanel = blocks[0] as IMyTextPanel;
        }
    }

    alertBlock = GetSingleBlockWithName(LOCKED_TRIGGER_BLOCK);

    stateDisplayPB = GetSingleBlockWithName(STATE_DISPLAY_PB_TAG) as IMyProgrammableBlock;

    fireBlockRockets = GetBlocksWithName<IMyTerminalBlock>(ROCKET_LOCKED_FIRE_TAG);
    fireBlockGatlings = GetBlocksWithName<IMyTerminalBlock>(GATLING_LOCKED_FIRE_TAG);
}

List<IMyCameraBlock> GetLidars()
{
    List<IMyTerminalBlock> blocks = GetBlocksWithName<IMyCameraBlock>(LIDAR_BLOCK_TAG);
    if (blocks.Count == 0)
    {
        return null;
    }
    else
    {
        List<IMyCameraBlock> cameraBlocks = new List<IMyCameraBlock>();
        for (int i = 0; i < blocks.Count; i++)
        {
            cameraBlocks.Add(blocks[i] as IMyCameraBlock);
        }

        return cameraBlocks;
    }
}

IMyTerminalBlock GetAimPointBlock()
{
    List<IMyTerminalBlock> blocks = GetBlocksWithName<IMyTerminalBlock>(AIM_POINT_BLOCK_TAG);
    if (blocks.Count > 1)
    {
        Echo("Error: More than one Aim Point Reference Block with tag " + AIM_POINT_BLOCK_TAG + ". Please rename away one of them.");
        return null;
    }
    if (blocks.Count == 1)
    {
        return (IMyTerminalBlock)blocks[0];
    }

    return null;
}

List<IMyTerminalBlock> GetGyroscopes()
{
    List<IMyTerminalBlock> blocks = GetBlocksWithName<IMyGyro>(GYROSCOPE_NAME_TAG);
    if (blocks.Count > 0)
    {
        return blocks;
    }

    GridTerminalSystem.GetBlocksOfType<IMyGyro>(blocks);
    if (blocks.Count > 0)
    {
        return blocks;
    }
    else
    {
        Echo("Error: No Gyroscopes detected. Please add at least one.");
        return null;
    }
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

    yawController = new PIDController(AIM_P, AIM_I, AIM_D, INTEGRAL_WINDUP_LIMIT, -INTEGRAL_WINDUP_LIMIT, SECOND);
    pitchController = new PIDController(AIM_P, AIM_I, AIM_D, INTEGRAL_WINDUP_LIMIT, -INTEGRAL_WINDUP_LIMIT, SECOND);
    rollController = new PIDController(AIM_P, AIM_I, AIM_D, INTEGRAL_WINDUP_LIMIT, -INTEGRAL_WINDUP_LIMIT, SECOND);
}

//------------------------------ Command Processing Methods ------------------------------

void ProcessCustomConfiguration()
{
    CustomConfiguration cfg = new CustomConfiguration(Me);
    cfg.Load();

    cfg.Get("LIDAR_MIN_LOCK_DISTANCE", ref LIDAR_MIN_LOCK_DISTANCE);
    cfg.Get("LIDAR_MAX_LOCK_DISTANCE", ref LIDAR_MAX_LOCK_DISTANCE);
    cfg.Get("LIDAR_REFRESH_INTERVAL", ref LIDAR_REFRESH_INTERVAL);
    cfg.Get("LIDAR_LOCK_LOST_TICKS", ref LIDAR_LOCK_LOST_TICKS);
    cfg.Get("LIDAR_REFRESH_CALC_FACTOR", ref LIDAR_REFRESH_CALC_FACTOR);
    cfg.Get("excludeFriendly", ref excludeFriendly);
    cfg.Get("fivePointInitialLockDist", ref fivePointInitialLockDist);
    cfg.Get("raycastAheadCount", ref raycastAheadCount);
    cfg.Get("raycastAheadSeconds", ref raycastAheadSeconds);
    cfg.Get("checkSelfOcclusion", ref checkSelfOcclusion);
    cfg.Get("useDefaultPIDValues", ref useDefaultPIDValues);
    cfg.Get("AIM_P", ref AIM_P);
    cfg.Get("AIM_I", ref AIM_I);
    cfg.Get("AIM_D", ref AIM_D);
    cfg.Get("AIM_LIMIT", ref AIM_LIMIT);
    cfg.Get("INTEGRAL_WINDUP_LIMIT", ref INTEGRAL_WINDUP_LIMIT);
}

//------------------------------ Gyro Control API ------------------------------

IMyTerminalBlock GetSingleBlockWithName(string name)
{
    List<IMyTerminalBlock> blocks = new List<IMyTerminalBlock>();
    GridTerminalSystem.SearchBlocksOfName(name, blocks);

    return (blocks.Count > 0 ? blocks[0] : null);
}

List<T> GetBlocksOfTypeCasted<T>() where T: class, IMyTerminalBlock
{
    List<T> blocks = new List<T>();
    GridTerminalSystem.GetBlocksOfType<T>(blocks);

    return blocks;
}

List<IMyTerminalBlock> GetBlocksWithName<T>(string name) where T: class, IMyTerminalBlock
{
    List<IMyTerminalBlock> blocks = new List<IMyTerminalBlock>();
    GridTerminalSystem.SearchBlocksOfName(name, blocks);

    List<IMyTerminalBlock> filteredBlocks = new List<IMyTerminalBlock>();
    for (int i = 0; i < blocks.Count; i++)
    {
        IMyTerminalBlock block = blocks[i] as T;
        if (block != null)
        {
            filteredBlocks.Add(block);
        }
    }

    return filteredBlocks;
}

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

    public void ApplyAction(string actionName)
    {
        foreach (IMyGyro gyro in gyros)
        {
            gyro.ApplyAction(actionName);
        }
    }

    public void SetGyroOverride(bool bOverride)
    {
        foreach (IMyGyro gyro in gyros)
        {
            if (gyro.GyroOverride != bOverride)
            {
                gyro.ApplyAction("Override");
            }
        }
    }

    public void SetGyroYaw(float yawRate)
    {
        for (int i = 0; i < gyros.Count; i++)
        {
            byte index = gyroYaw[i];
            gyros[i].SetValue(profiles[index], (index % 2 == 0 ? yawRate : -yawRate) * MathHelper.RadiansPerSecondToRPM);
        }
    }

    public void SetGyroPitch(float pitchRate)
    {
        for (int i = 0; i < gyros.Count; i++)
        {
            byte index = gyroPitch[i];
            gyros[i].SetValue(profiles[index], (index % 2 == 0 ? pitchRate : -pitchRate) * MathHelper.RadiansPerSecondToRPM);
        }
    }

    public void SetGyroRoll(float rollRate)
    {
        for (int i = 0; i < gyros.Count; i++)
        {
            byte index = gyroRoll[i];
            gyros[i].SetValue(profiles[index], (index % 2 == 0 ? rollRate : -rollRate) * MathHelper.RadiansPerSecondToRPM);
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

public class CustomConfiguration
{
    public IMyTerminalBlock configBlock;
    public Dictionary<string, string> config;

    public CustomConfiguration(IMyTerminalBlock block)
    {
        configBlock = block;
        config = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
    }

    public void Load()
    {
        ParseCustomData(configBlock, config);
    }

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
        if (config.TryGetValue(key.Trim(), out val))
        {
            res = val;
        }
    }

    public void Get(string key, ref int res)
    {
        int val;
        if (int.TryParse(Get(key), out val))
        {
            res = val;
        }
    }

    public void Get(string key, ref float res)
    {
        float val;
        if (float.TryParse(Get(key), out val))
        {
            res = val;
        }
    }

    public void Get(string key, ref double res)
    {
        double val;
        if (double.TryParse(Get(key), out val))
        {
            res = val;
        }
    }

    public void Get(string key, ref bool res)
    {
        bool val;
        if (bool.TryParse(Get(key), out val))
        {
            res = val;
        }
    }
    public void Get(string key, ref bool? res)
    {
        bool val;
        if (bool.TryParse(Get(key), out val))
        {
            res = val;
        }
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

public class CubeTree
{
    public Vector3I[] ADJACENT_VECTORS = { new Vector3I(-1, 0, 0), new Vector3I(1, 0, 0), new Vector3I(0, -1, 0), new Vector3I(0, 1, 0), new Vector3I(0, 0, -1), new Vector3I(0, 0, 1) };

    public TreeNode Root;
    public IMyCubeGrid Grid;

    public CubeTree()
    {

    }

    public IEnumerator<int> InitTree(IMyCubeGrid grid, Vector3I start, int limit)
    {
        int counter = 0;

        Grid = grid;
        Root = new TreeNode(Grid.Min, Grid.Max);

        Stack<Vector3I> remaining = new Stack<Vector3I>();
        HashSet<Vector3I> tested = new HashSet<Vector3I>();

        remaining.Push(start);
        tested.Add(start);
        Root.Add(start);

        bool exists;
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
                    exists = Grid.CubeExists(next);
                    if (exists)
                    {
                        remaining.Push(next);
                        Root.Add(next);
                    }
                }
            }

            counter++;
            if (counter % limit == 0)
            {
                yield return counter;
            }
        }
    }

    public bool TestRayHit(Ray ray)
    {
        Vector3I rayCenter = new Vector3I(ray.Position);

        Stack<TreeNode> remaining = new Stack<TreeNode>();

        remaining.Push(Root);

        while (remaining.Count > 0)
        {
            TreeNode current = remaining.Pop();

            if (!current.IsLeaf)
            {
                for (int i = 0; i < 8; i++)
                {
                    TreeNode child = current.Children[i];
                    if (child != null)
                    {
                        if (child.HitBox.Intersects(ray) != null)
                        {
                            if (child.IsLeaf)
                            {
                                if (child.Center != rayCenter)
                                {
                                    return true;
                                }
                            }
                            else
                            {
                                remaining.Push(child);
                            }
                        }
                    }
                }
            }
        }

        return false;
    }
}

public class TreeNode
{
    public Vector3I? Min;
    public Vector3I? Max;

    public Vector3I Center;
    public BoundingBox HitBox;

    public bool IsLeaf;
    public TreeNode[] Children;

    public TreeNode(Vector3I min, Vector3I max)
    {
        if (min == max)
        {
            IsLeaf = true;
            Children = null;

            Center = min;
        }
        else
        {
            IsLeaf = false;
            Children = new TreeNode[8];

            Min = min;
            Max = max;

            Center = min + ((max - min) / 2);
        }

        HitBox = new BoundingBox(min - new Vector3(0.5f, 0.5f, 0.5f), max + new Vector3(0.5f, 0.5f, 0.5f));
    }

    public void Add(Vector3I cube)
    {
        if (IsLeaf)
        {
            return;
        }
        else
        {
            int index = GetChildIndex(cube);
            if (Children[index] == null)
            {
                Vector3I newMin = new Vector3I();
                Vector3I newMax = new Vector3I();
                newMin.X = (cube.X <= Center.X ? Min.Value.X : Center.X + 1);
                newMin.Y = (cube.Y <= Center.Y ? Min.Value.Y : Center.Y + 1);
                newMin.Z = (cube.Z <= Center.Z ? Min.Value.Z : Center.Z + 1);
                newMax.X = (cube.X <= Center.X ? Center.X : Max.Value.X);
                newMax.Y = (cube.Y <= Center.Y ? Center.Y : Max.Value.Y);
                newMax.Z = (cube.Z <= Center.Z ? Center.Z : Max.Value.Z);
                Children[index] = new TreeNode(newMin, newMax);
            }
            Children[index].Add(cube);
        }
    }

    public int GetChildIndex(Vector3I cube)
    {
        return (cube.X <= Center.X ? 0 : 1) + (cube.Y <= Center.Y ? 0 : 2) + (cube.Z <= Center.Z ? 0 : 4);
    }
}