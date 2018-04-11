//============================================================
// Script powering my <Whatever>storm Missiles. Supposed to
// be further updated and then published. Inter-grid
// communication is to be added in the future.
//============================================================

//------------------------------------------------------------
// ADN - Trajectory Missile Script v3.4
//------------------------------------------------------------

//Type of block to disconnect missile from launching ship: 0 = Merge Block, 1 = Rotor, 2 = Connector, 3 = Merge Block And Any Locked Connectors, 4 = Rotor And Any Locked Connectors, 99 = No detach required
int missileDetachPortType = 0;

//Type of missile trajectory: 0 = Frefall To Target With Aiming (For Cluster Bomb Deployments), 1 = Frefall To Target Without Aiming (For Cluster Bomb Deployments), 2 = Thrust And Home In To Target
int missileTrajectoryType = 0;

//Script will only extract missile blocks on same grid as this PB
bool missileBlockSameGridOnly = true;

//------------------------------ Reference Block Name Configuration ------------------------------

string strShipRefTargetPanel = "R_TARGET";              //For changing target location or issue command in-flight

//By default all gyroscopes, thrusters and merge blocks will be considered for use. Setting a value here limits the script to use specific set of blocks
string strGyroscopesTag = "";
string strThrustersTag = "";
string strDetachPortTag = "";
string strDirectionRefBlockTag = "";

//For debugging purposes
string strStatusDisplayPrefix = "<D>";

//------------------------------ Missile Handling Configuration ------------------------------

double driftVectorReduction = 1.5;
double launchSeconds = 5;
double missileTravelHeight = 3000;                      //The height in metres the missile will try to reach, measuring from launch point

int missileDeployDistanceCorrection = 0;                //Deploy missile this distance earlier in metres
string missileDeployCommand = "";

bool? boolDrift = null;
bool? boolLeadTarget = null;
bool? boolNaturalDampener = null;

//------------------------------ This Section Is Missile Turning Parameters ------------------------------

double MAX_FALL_SPEED = 104.37;                         //Maximum falling speed. If speed mod is used, this is the maximum speed of the speed mod
int HEIGHT_DEAD_ZONE = 500;                             //Height range above the cruise height where missile will not perform any tangent vector adjustment to prevent oscillation

bool readjustMaxFallSpeed = true;                       //If speed mods are used, user may forget to configure MAX_FALL_SPEED. This flag will attempt to readjust it based on current missile speed

//------------------------------ Above Is User Configuration Section. This Section Is For PID Tuning ------------------------------

double AIM_P = 0;
double AIM_I = 0;
double AIM_D = 0;
double AIM_LIMIT = 6.3;

double INTEGRAL_WINDUP_UPPER_LIMIT = 0;
double INTEGRAL_WINDUP_LOWER_LIMIT = 0;

//------------------------------ Script Parameters Configuration ------------------------------

int MERGE_SEPARATE_WAIT_THRESHOLD = 60;

bool outputMissileStatus = false;

//------------------------------ Important Constants ------------------------------

const double DEF_SMALL_GRID_P = 31.42;
const double DEF_SMALL_GRID_I = 0;
const double DEF_SMALL_GRID_D = 10.48;

const double DEF_BIG_GRID_P = 15.71;
const double DEF_BIG_GRID_I = 0;
const double DEF_BIG_GRID_D = 7.05;

const float SECOND = 60f;

//------------------------------ Below Is Main Script Body ------------------------------

IMyTextPanel shipRefTargetPanel;

IMyShipController remoteControl;
IMyTerminalBlock refForwardBlock;
IMyTerminalBlock refDownwardBlock;
IMyTerminalBlock statusDisplay;

GyroControl gyroControl;

List<IMyTerminalBlock> thrusters;
float[] thrustValues;

MatrixD refWorldMatrix;
MatrixD refViewMatrix;
bool refForwardReverse;

Vector3D naturalGravity;
double naturalGravityLength;

Vector3D referencePoint;               //Coordinates of the planet center
double cruiseHeight;                   //The height from planet center the missile should cruise at
double cruiseHeightPeak;               //The cut off height from planet center the missile will start to reduce height

Vector3D driftVector;
double speed;
double rpm;

double lastSpeed;
Vector3D lastPosition;

Vector3D targetPosition;
Vector3D lastTargetPosition;

bool targetPositionSet;

Vector3D targetVector;
double distToTarget;

Vector3D impactPoint;
double timeToImpact;

PIDController yawController;
PIDController pitchController;
PIDController rollController;

bool spinActive = false;

int subCounter = 0;
int subMode = 0;
int mode = 0;
int clock = 0;
bool init = false;

IMyTerminalBlock detachBlock;
int detachBlockType = -1;

List<KeyValuePair<double, string[]>> rpmTriggerList;
List<KeyValuePair<double, string[]>> distTriggerList;
List<KeyValuePair<int, string[]>> timeTriggerList;
Dictionary<string, List<IMyTerminalBlock>> savedBlockList;

Random rnd = new Random();

void Main(string arguments, UpdateType updateSource)
{
    //---------- Initialization And General Controls ----------

    if (!init)
    {
        if (subMode == 0)       //Check for configuration command
        {
            subMode = 1;

            if (Me.CustomData.Length > 0)
            {
                ProcessCustomConfiguration();
            }

            if (arguments.Length > 0)
            {
                ProcessConfigurationCommand(arguments);

                if (!targetPositionSet)
                {
                    return;
                }
            }
        }

        if (subMode == 1)       //Missile still on launching ship's grid
        {
            InitLaunchingShipRefBlocks();

            if (shipRefTargetPanel == null)
            {
                boolLeadTarget = false;
            }

            if (!targetPositionSet && !PopulateTargetLocation(arguments))
            {
                if (shipRefTargetPanel != null)
                {
                    if (!PopulateTargetLocation(shipRefTargetPanel.GetPublicTitle()))
                    {
                        return;
                    }
                }
                else
                {
                    return;
                }
            }

            Runtime.UpdateFrequency = UpdateFrequency.Update1;

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

            if (detachBlockType == 0)
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

            if (!InitMissileBlocks())
            {
                throw new Exception("--- Initialization Failed ---");
            }
        }

        gyroControl.Enabled(true);

        lastPosition = refWorldMatrix.Translation;

        subCounter = (int)(launchSeconds * SECOND);
        FireThrusters();

        subMode = 0;
        mode = 1;
        clock = 0;

        init = true;
        return;
    }

    //---------- Modes And Controls ----------

    bool allowDisplayUpdate = true;

    if (shipRefTargetPanel != null)
    {
        targetPositionSet = PopulateTargetLocation(shipRefTargetPanel.GetPublicTitle());
        ExecuteTargetCommand(shipRefTargetPanel.CustomData);
    }

    if ((updateSource & UpdateType.Update1) == 0 || Runtime.TimeSinceLastRun.Ticks == 0)
    {
        return;
    }

    clock++;

    CalculateParameters();

    if (naturalGravityLength < 0.01)
    {
        return;
    }

    if (mode == 1)
    {
        FireThrusters();

        if (subCounter > 0)
        {
            subCounter -= 1;
        }
        else
        {
            gyroControl.SetGyroOverride(true);

            subCounter = 0;
            mode = 2;
        }
    }
    else if (mode == 2)
    {
        if (clock % SECOND == 0)
        {
            FireThrusters();
        }

        targetVector = -naturalGravity;
        distToTarget = naturalGravityLength;

        targetVector = Vector3D.TransformNormal(targetVector, refViewMatrix);
        targetVector.Normalize();

        AimAtTarget();

        if (boolNaturalDampener == true)
        {
            Vector3D alignTarget = Vector3D.Normalize(targetPosition - refWorldMatrix.Translation);
            AimDampenerAtVector(ref alignTarget);
        }

        if ((refWorldMatrix.Translation - lastPosition).Length() > missileTravelHeight - (speed * speed / naturalGravityLength / 2))
        {
            remoteControl.TryGetPlanetPosition(out referencePoint);

            cruiseHeight = (lastPosition - referencePoint).Length() + missileTravelHeight;
            cruiseHeightPeak = cruiseHeight + HEIGHT_DEAD_ZONE;

            subCounter = 0;
            mode = 4;
        }
    }
    else if (mode == 3)
    {
        if (clock % SECOND == 0)
        {
            FireThrusters();
        }

        Vector3D vectorN = targetPosition - refWorldMatrix.Translation;
        vectorN.Normalize();

        targetVector = Vector3D.Reject(vectorN, naturalGravity);
        targetVector.Normalize();

        distToTarget = targetVector.Length();
        targetVector = targetVector / distToTarget;

        targetVector = Vector3D.TransformNormal(targetVector, refViewMatrix);
        targetVector.Normalize();

        AimAtTarget();

        if (boolNaturalDampener == true)
        {
            AimDampenerAtVector(ref naturalGravity);
        }

        subCounter = 0;
        mode = 4;
    }
    else if (mode == 4)
    {
        if (clock % SECOND == 0)
        {
            FireThrusters();
        }

        Vector3D vectorN = targetPosition - refWorldMatrix.Translation;
        vectorN.Normalize();

        targetVector = Vector3D.Reject(vectorN, naturalGravity);
        targetVector.Normalize();

        double currentHeight = (referencePoint - refWorldMatrix.Translation).Length();
        if (currentHeight > cruiseHeightPeak)
        {
            targetVector += (naturalGravity / Math.Tan(Math.Asin(cruiseHeightPeak / currentHeight)));
        }
        else if (currentHeight < cruiseHeight)
        {
            targetVector -= (naturalGravity / Math.Tan(Math.Asin(currentHeight / cruiseHeight)));
        }

        distToTarget = targetVector.Length();
        targetVector = targetVector / distToTarget;

        if (boolDrift == true && speed >= 5)
        {
            if (driftVector.Dot(naturalGravity) < 0)
            {
                targetVector = (targetVector * speed) - (Vector3D.Reject(driftVector, naturalGravity) / driftVectorReduction);
            }
            else
            {
                targetVector = (targetVector * speed) - (driftVector / driftVectorReduction);
            }
            targetVector.Normalize();
        }

        targetVector = Vector3D.TransformNormal(targetVector, refViewMatrix);
        targetVector.Normalize();

        AimAtTarget();

        if (boolNaturalDampener == true)
        {
            AimDampenerAtVector(ref naturalGravity);
        }

        if (readjustMaxFallSpeed && speed > MAX_FALL_SPEED + 0.1)
        {
            MAX_FALL_SPEED = speed;
        }

        Vector3D targetHeightVector = (targetPosition - referencePoint);
        double targetHeightLength = targetHeightVector.Length();
        targetHeightVector = targetHeightVector / targetHeightLength;
        Vector3D missileSurfacePoint = referencePoint - (naturalGravity * targetHeightLength);

        double landDistance = (missileSurfacePoint - targetPosition).Length();
        double dropDistance = currentHeight - targetHeightLength;
        double relativeImpactSpeed = Math.Sqrt(2 * dropDistance * naturalGravityLength);
        double dropTime = relativeImpactSpeed / naturalGravityLength;
        double largestPossibleDistance = speed * dropTime * 2;

        if (landDistance - missileDeployDistanceCorrection <= largestPossibleDistance)
        {
            SimulateImpactPoint(referencePoint, naturalGravityLength, targetPosition, refWorldMatrix.Translation, driftVector, MAX_FALL_SPEED, 0.01, 3000, out impactPoint, out timeToImpact);
            if ((impactPoint - missileSurfacePoint).Length() >= landDistance - missileDeployDistanceCorrection)
            {
                subCounter = 0;
                mode = 5;
            }
        }
    }
    else if (mode == 5)
    {
        if (missileTrajectoryType <= 1)
        {
            DisableAllThrusters();
        }

        ExecuteTargetCommand(missileDeployCommand);

        lastTargetPosition = targetPosition;

        if (missileTrajectoryType == 1)
        {
            gyroControl.ZeroTurnGyro();
        }

        subCounter = 0;
        mode = 6;
    }
    else if (mode == 6)
    {
        targetVector = targetPosition - refWorldMatrix.Translation;
        distToTarget = targetVector.Length();
        targetVector = targetVector / distToTarget;

        if (missileTrajectoryType == 2)
        {
            if (boolDrift == true && speed >= 5)
            {
                targetVector = (targetVector * speed) - (driftVector / driftVectorReduction);
                targetVector.Normalize();
            }
        }

        targetVector = Vector3D.TransformNormal(targetVector, refViewMatrix);
        targetVector.Normalize();

        if (missileTrajectoryType != 1)
        {
            AimAtTarget();
        }

        if (missileTrajectoryType == 2 && !spinActive)
        {
            if (boolNaturalDampener == true)
            {
                AimDampenerAtVector(ref naturalGravity);
            }
        }

        distToTarget = (targetPosition - refWorldMatrix.Translation).Length();

        if (rpmTriggerList != null && rpmTriggerList.Count > 0)
        {
            int i = 0;
            while (i < rpmTriggerList.Count)
            {
                if (rpmTriggerList[i].Key <= rpm)
                {
                    ProcessSingleCommand(rpmTriggerList[i].Value);
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
                    ProcessSingleCommand(distTriggerList[i].Value);
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
                    ProcessSingleCommand(timeTriggerList[i].Value);
                    timeTriggerList.RemoveAt(i);
                }
                else
                {
                    i++;
                }
            }
        }
    }

    if (statusDisplay != null && allowDisplayUpdate)
    {
        if (mode == 0)
        {
            DisplayStatus("Idle");
        }
        else if (mode == 1)
        {
            DisplayStatus("Launching");
        }
        else if (mode == 2)
        {
            DisplayStatus("Gaining Altitude");
        }
        else if (mode == 3)
        {
            DisplayStatus("Flight Calibration");
        }
        else if (mode == 4)
        {
            DisplayStatus("Enroute To Target");
        }
        else if (mode == 5)
        {
            DisplayStatus("Decceleration");
        }
        else if (mode == 6)
        {
            DisplayStatus("Approach Alignment");
        }
        else if (mode == 7 || mode == 8 || mode == 9)
        {
            DisplayStatus("Deploy");
        }
        else if (mode == 17 || mode == 18)
        {
            DisplayStatus("Homing Descend");
        }
        else
        {
            DisplayStatus("-");
        }
    }

    if (outputMissileStatus)
    {
        Echo("ST:" + mode + ":" + subMode + ":" + subCounter + ":" + clock + ":-:" +
            Math.Round(targetPosition.GetDim(0), 5) + ":" + Math.Round(targetPosition.GetDim(1), 5) + ":" + Math.Round(targetPosition.GetDim(2), 5) + ":" +
            0 + ":");
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

//------------------------------ Missile And Target Information Methods ------------------------------

void SimulateImpactPoint(Vector3D planetCenter, double planetGravity, Vector3D targetPoint, Vector3D currentPoint, Vector3D currentDirection, double maxSpeedLimit, double angleLimit, int loopLimit, out Vector3D vImpactPoint, out double nTimeToImpact)
{
    double targetHeight = (planetCenter - targetPoint).Length();
    double acosAngleLimit = Math.Cos(angleLimit);
    bool hitMaxSpeed = false;

    for (int i = 0; i < loopLimit; i++)
    {
        Vector3D centerToCurrent = currentPoint - planetCenter;
        double currentImpactHeight = centerToCurrent.Length();
        if (currentImpactHeight <= targetHeight)
        {
            vImpactPoint = currentPoint;
            nTimeToImpact = (i / 60.0);

            return;
        }

        Vector3D gravityVector = Vector3D.Normalize(planetCenter - currentPoint) * planetGravity;

        if (hitMaxSpeed && angleLimit > 0)
        {
            double acosCurrentAngle = currentDirection.Dot(gravityVector) / planetGravity / currentDirection.Length();
            if (acosCurrentAngle >= acosAngleLimit)
            {
                vImpactPoint = (centerToCurrent / currentImpactHeight * targetHeight) + planetCenter;
                nTimeToImpact = (i / 60.0) + (Math.Abs(currentImpactHeight - targetHeight) / maxSpeedLimit);

                return;
            }
        }

        currentDirection += (gravityVector / 60);
        if (maxSpeedLimit > 0)
        {
            double speed = currentDirection.Length();
            if (speed > maxSpeedLimit)
            {
                currentDirection = currentDirection / speed * maxSpeedLimit;
                hitMaxSpeed = true;
            }

            currentPoint += (currentDirection / 60);
        }
    }

    Vector3D centerToFinalPoint = currentPoint - planetCenter;
    double finalImpactHeight = centerToFinalPoint.Length();

    vImpactPoint = (centerToFinalPoint / finalImpactHeight * targetHeight) + planetCenter;
    nTimeToImpact = (loopLimit / 60.0) + (Math.Abs(finalImpactHeight - targetHeight) / maxSpeedLimit);
}

void CalculateParameters()
{
    //---------- Calculate Missile Related Variables ----------

    refWorldMatrix = refForwardBlock.WorldMatrix;
    if (refForwardReverse)
    {
        refWorldMatrix.Forward = refWorldMatrix.Backward;
        refWorldMatrix.Left = refWorldMatrix.Right;
    }
    refViewMatrix = MatrixD.Transpose(refWorldMatrix);

    lastSpeed = speed;
    driftVector = remoteControl.GetShipVelocities().LinearVelocity;
    speed = driftVector.Length();

    rpm = Math.Abs(remoteControl.GetShipVelocities().AngularVelocity.Dot(refWorldMatrix.Forward)) * MathHelper.RadiansPerSecondToRPM;

    naturalGravity = remoteControl.GetNaturalGravity();
    naturalGravityLength = naturalGravity.Length();
    naturalGravity = (naturalGravityLength > 0 ? naturalGravity / naturalGravityLength : Vector3D.Zero);
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

void AimDampenerAtVector(ref Vector3D aimVector)
{
    //---------- Activate Gyroscopes To Aim Dampener At Natural Gravity ----------

    if (refDownwardBlock == null || naturalGravityLength < 0.01)
    {
        return;
    }

    MatrixD dampenerLookAtMatrix = MatrixD.CreateLookAt(Vector3D.Zero, refDownwardBlock.WorldMatrix.Forward, refWorldMatrix.Forward);

    Vector3D gravityVector = Vector3D.TransformNormal(aimVector, dampenerLookAtMatrix);
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

//------------------------------ Missile Separation Methods ------------------------------

bool DetachFromGrid(bool testOnly = false)
{
    List<IMyTerminalBlock> blocks;

    switch (missileDetachPortType)
    {
        case 0:
        case 3:
            blocks = (strDetachPortTag != null && strDetachPortTag.Length > 0 ? GetBlocksWithName<IMyShipMergeBlock>(strDetachPortTag) : GetBlocksOfType<IMyShipMergeBlock>());
            detachBlock = GetClosestBlockFromReference(blocks, Me, true);

            if (!testOnly)
            {
                if (detachBlock == null)
                {
                    Echo("Error: Missing Merge Block " + (strDetachPortTag != null && strDetachPortTag.Length > 0 ? "with tag " + strDetachPortTag + " to detach" : "to detach."));
                    return false;
                }
                detachBlockType = 0;

                detachBlock.ApplyAction("OnOff_Off");
            }
            return true;
        case 1:
        case 4:
            blocks = (strDetachPortTag != null && strDetachPortTag.Length > 0 ? GetBlocksWithName<IMyMotorBase>(strDetachPortTag) : GetBlocksOfType<IMyMotorBase>());
            for (int i = 0; i < blocks.Count; i++)
            {
                IMyCubeGrid grid = ((IMyMotorBase)blocks[i]).TopGrid;
                if (grid != null && grid == Me.CubeGrid)
                {
                    detachBlock = blocks[i];
                    break;
                }
            }

            if (detachBlock == null)
            {
                for (int i = 0; i < blocks.Count; i++)
                {
                    if (blocks[i].CubeGrid == Me.CubeGrid)
                    {
                        detachBlock = blocks[i];
                        break;
                    }
                }
            }

            if (!testOnly)
            {
                if (detachBlock == null)
                {
                    Echo("Error: Missing Rotor " + (strDetachPortTag != null && strDetachPortTag.Length > 0 ? "with tag " + strDetachPortTag + " to detach" : "to detach."));
                    return false;
                }
                detachBlockType = 1;

                detachBlock.ApplyAction("Detach");
            }
            return true;
        case 2:
            blocks = (strDetachPortTag != null && strDetachPortTag.Length > 0 ? GetBlocksWithName<IMyShipConnector>(strDetachPortTag) : GetBlocksOfType<IMyShipConnector>());
            detachBlock = GetClosestBlockFromReference(blocks, Me, true);

            if (!testOnly)
            {
                if (detachBlock == null)
                {
                    Echo("Error: Missing Connector " + (strDetachPortTag != null && strDetachPortTag.Length > 0 ? "with tag " + strDetachPortTag + " to detach" : "to detach."));
                    return false;
                }
                detachBlockType = 2;

                detachBlock.ApplyAction("Unlock");
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

IMyTerminalBlock GetConnectedMergeBlock(IMyCubeGrid grid, IMyTerminalBlock mergeBlock)
{
    IMySlimBlock slimBlock = grid.GetCubeBlock(mergeBlock.Position - new Vector3I(Base6Directions.GetVector(mergeBlock.Orientation.Left)));
    return (slimBlock == null ? null : slimBlock.FatBlock as IMyTerminalBlock);
}

void DetachLockedConnectors()
{
    List<IMyTerminalBlock> blocks = GetBlocksOfType<IMyShipConnector>();
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

//------------------------------ Command Processing Methods ------------------------------

bool PopulateTargetLocation(string command)
{
    if (command == null || command.Length == 0)
    {
        return false;
    }

    string[] tokens = command.Trim().Split(':');
    if (tokens[0].Trim().Equals("GPS") && tokens.Length > 4)
    {
        Vector3D parsedVector = new Vector3D();
        double result;

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

        targetPosition = parsedVector;
        return true;
    }
    else
    {
        return false;
    }
}

void ExecuteTargetCommand(string commandLine)
{
    string[] keyValues = commandLine.Split(',');

    for (int i = 0; i < keyValues.Length; i++)
    {
        string[] tokens = keyValues[i].Trim().Split(':');
        if (tokens.Length > 0)
        {
            ProcessSingleCommand(tokens);
        }
    }
}

void ProcessSingleCommand(string[] tokens)
{
    string cmdToken = tokens[0].Trim();
    if (cmdToken.StartsWith("ACT") && tokens.Length >= 3)
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
        case 'C':
            triggerBlocks = (savedBlockList != null ? savedBlockList[tokens[1]] : null);
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
        case 'C':
            triggerBlocks = (savedBlockList != null ? savedBlockList[tokens[1]] : null);
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
                        triggerBlocks[i].SetValueFloat(tokens[2], (float)(timeToImpact <= 0 ? (distToTarget / speed) : timeToImpact) / float.Parse(tokens[3]));
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
    else if (cmdToken.Equals("TGR") && tokens.Length >= 3)
    {
        if (rpmTriggerList == null)
        {
            rpmTriggerList = new List<KeyValuePair<double, string[]>>();
        }

        string[] items = new string[tokens.Length - 2];
        Array.Copy(tokens, 2, items, 0, items.Length);
        rpmTriggerList.Add(new KeyValuePair<double, string[]>(Double.Parse(tokens[1]), items));
    }
    else if (cmdToken.Equals("TGD") && tokens.Length >= 3)
    {
        if (distTriggerList == null)
        {
            distTriggerList = new List<KeyValuePair<double, string[]>>();
        }

        string[] items = new string[tokens.Length - 2];
        Array.Copy(tokens, 2, items, 0, items.Length);
        distTriggerList.Add(new KeyValuePair<double, string[]>(Double.Parse(tokens[1]), items));
    }
    else if (cmdToken.Equals("TGE") && tokens.Length >= 3)
    {
        if (distTriggerList == null)
        {
            distTriggerList = new List<KeyValuePair<double, string[]>>();
        }

        string[] items = new string[tokens.Length - 2];
        Array.Copy(tokens, 2, items, 0, items.Length);
        distTriggerList.Add(new KeyValuePair<double, string[]>(distToTarget - Double.Parse(tokens[1]), items));
    }
    else if (cmdToken.Equals("TGT") && tokens.Length >= 3)
    {
        if (timeTriggerList == null)
        {
            timeTriggerList = new List<KeyValuePair<int, string[]>>();
        }

        string[] items = new string[tokens.Length - 2];
        Array.Copy(tokens, 2, items, 0, items.Length);
        int ticks = (int)(Double.Parse(tokens[1]) * SECOND) + clock;
        timeTriggerList.Add(new KeyValuePair<int, string[]>(ticks, items));
    }
    else if (cmdToken.StartsWith("SAV") && tokens.Length >= 3)
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
            if (savedBlockList == null)
            {
                savedBlockList = new Dictionary<string, List<IMyTerminalBlock>>();
            }
            savedBlockList[tokens[2]] = triggerBlocks;
        }
    }
    else if (cmdToken.Equals("SPIN") && tokens.Length >= 1)
    {
        gyroControl.SetGyroRoll(tokens.Length >= 2 ? Int32.Parse(tokens[1]) : 30);
        spinActive = true;
    }
    else if (cmdToken.Equals("ZTH"))
    {
        DisableAllThrusters();
    }
    else if (cmdToken.Equals("ZGY"))
    {
        gyroControl.ZeroTurnGyro();
    }
    else if (cmdToken.Equals("DISCONNECT"))
    {
        shipRefTargetPanel = null;
    }
    else if (cmdToken.Equals("ABORT"))
    {
        mode = 99;
    }
}

void ProcessCustomConfiguration()
{
    CustomConfiguration cfg = new CustomConfiguration(Me);
    cfg.Load();

    cfg.Get("missileDetachPortType", ref missileDetachPortType);
    cfg.Get("missileTrajectoryType", ref missileTrajectoryType);
    cfg.Get("missileBlockSameGridOnly", ref missileBlockSameGridOnly);
    cfg.Get("strShipRefTargetPanel", ref strShipRefTargetPanel);
    cfg.Get("strGyroscopesTag", ref strGyroscopesTag);
    cfg.Get("strThrustersTag", ref strThrustersTag);
    cfg.Get("strDetachPortTag", ref strDetachPortTag);
    cfg.Get("strDirectionRefBlockTag", ref strDirectionRefBlockTag);
    cfg.Get("strStatusDisplayPrefix", ref strStatusDisplayPrefix);
    cfg.Get("missileTravelHeight", ref missileTravelHeight);
    cfg.Get("missileDeployDistanceCorrection", ref missileDeployDistanceCorrection);
    cfg.Get("missileDeployCommand", ref missileDeployCommand);
    cfg.Get("MAX_FALL_SPEED", ref MAX_FALL_SPEED);
    cfg.Get("HEIGHT_DEAD_ZONE", ref HEIGHT_DEAD_ZONE);
    cfg.Get("readjustMaxFallSpeed", ref readjustMaxFallSpeed);
    cfg.Get("driftVectorReduction", ref driftVectorReduction);
    cfg.Get("launchSeconds", ref launchSeconds);
    cfg.Get("boolDrift", ref boolDrift);
    cfg.Get("boolLeadTarget", ref boolLeadTarget);
    cfg.Get("boolNaturalDampener", ref boolNaturalDampener);
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
    string cmdToken = tokens[0].Trim();
    if (cmdToken.Equals("R_TAR") && tokens.Length >= 2)
    {
        strShipRefTargetPanel = tokens[1];
    }
    else if (cmdToken.Equals("V_DVR") && tokens.Length >= 2)
    {
        double dvrValue;
        if (Double.TryParse(tokens[1], out dvrValue))
        {
            driftVectorReduction = dvrValue;
        }
    }
    else if (cmdToken.Equals("V_LS") && tokens.Length >= 2)
    {
        double lsValue;
        if (Double.TryParse(tokens[1], out lsValue))
        {
            launchSeconds = lsValue;
        }
    }
    else if (cmdToken.Equals("V_MTH") && tokens.Length >= 2)
    {
        double mthValue;
        if (Double.TryParse(tokens[1], out mthValue))
        {
            missileTravelHeight = mthValue;
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
    else if (cmdToken.Equals("GPS") && tokens.Length > 4)
    {
        Vector3D parsedVector = new Vector3D();
        double result;

        if (Double.TryParse(tokens[2], out result))
        {
            parsedVector.SetDim(0, result);

            if (Double.TryParse(tokens[3], out result))
            {
                parsedVector.SetDim(1, result);

                if (Double.TryParse(tokens[4], out result))
                {
                    parsedVector.SetDim(2, result);

                    targetPosition = parsedVector;
                    targetPositionSet = true;
                }
            }
        }
    }
}

//------------------------------ Initialization Methods ------------------------------

void InitLaunchingShipRefBlocks()
{
    List<IMyTerminalBlock> blocks;

    blocks = GetBlocksWithName<IMyTextPanel>(strShipRefTargetPanel);

    if (blocks.Count > 0)
    {
        if (blocks.Count > 1)
        {
            Echo("Warning: More than one Text Panel with name " + strShipRefTargetPanel + " found. Using first one detected.");
        }

        shipRefTargetPanel = blocks[0] as IMyTextPanel;
    }
}

bool InitMissileBlocks()
{
    List<IMyTerminalBlock> gyros = GetGyroscopes();
    if (gyros == null || gyros.Count == 0) return false;

    thrusters = GetThrusters();
    if (thrusters == null || thrusters.Count == 0) return false;

    remoteControl = GetRemoteControl();
    if (remoteControl == null) return false;

    if (missileBlockSameGridOnly)
    {
        FilterSameGrid(Me.CubeGrid, ref gyros);
        FilterSameGrid(Me.CubeGrid, ref thrusters);
    }

    bool isFixedDirection = false;

    if (strDirectionRefBlockTag != null && strDirectionRefBlockTag.Length > 0)
    {
        refForwardBlock = GetSingleBlockWithName(strDirectionRefBlockTag, missileBlockSameGridOnly);
        isFixedDirection = (refForwardBlock != null);
    }

    if (refForwardBlock == null || boolNaturalDampener == null || boolDrift == null)
    {
        thrustValues = ComputeMaxThrustValues(thrusters);
    }

    if (refForwardBlock == null)
    {
        refForwardBlock = ComputeHighestThrustReference(thrusters, thrustValues);
        refForwardReverse = true;
    }

    refWorldMatrix = refForwardBlock.WorldMatrix;
    if (refForwardReverse)
    {
        refWorldMatrix.Forward = refWorldMatrix.Backward;
        refWorldMatrix.Left = refWorldMatrix.Right;
    }

    gyroControl = new GyroControl(gyros, ref refWorldMatrix);

    InitThrusters(isFixedDirection);
    thrustValues = null;

    InitPIDControllers();

    if (boolLeadTarget == null)
    {
        boolLeadTarget = true;
    }

    if (strStatusDisplayPrefix != null && strStatusDisplayPrefix.Length > 0)
    {
        List<IMyTerminalBlock> blocks = GetBlocksWithName<IMyTerminalBlock>(strStatusDisplayPrefix, 1);
        if (blocks.Count > 0)
        {
            statusDisplay = blocks[0];

            if (statusDisplay.HasAction("OnOff_On"))
            {
                statusDisplay.ApplyAction("OnOff_On");

                IMyRadioAntenna radioAntenna = statusDisplay as IMyRadioAntenna;
                if (radioAntenna != null && !radioAntenna.IsBroadcasting)
                {
                    radioAntenna.ApplyAction("EnableBroadCast");
                }
            }
        }
    }

    return true;
}

List<IMyTerminalBlock> GetGyroscopes()
{
    List<IMyTerminalBlock> blocks = GetBlocksWithName<IMyGyro>(strGyroscopesTag);
    if (blocks.Count > 0)
    {
        return blocks;
    }

    GridTerminalSystem.GetBlocksOfType<IMyGyro>(blocks);
    if (blocks.Count == 0)
    {
        Echo("Error: Missing Gyroscopes.");
    }
    return blocks;
}

List<IMyTerminalBlock> GetThrusters()
{
    List<IMyTerminalBlock> blocks = GetBlocksWithName<IMyThrust>(strThrustersTag);
    if (blocks.Count > 0)
    {
        return blocks;
    }

    GridTerminalSystem.GetBlocksOfType<IMyThrust>(blocks);
    if (blocks.Count == 0)
    {
        Echo("Warning: Missing Thrusters.");
    }
    return blocks;
}

IMyShipController GetRemoteControl()
{
    List<IMyTerminalBlock> blocks = GetBlocksOfType<IMyShipController>();
    if (missileBlockSameGridOnly)
    {
        FilterSameGrid(Me.CubeGrid, ref blocks);
    }

    IMyShipController remoteBlock = (blocks.Count > 0 ? blocks[0] as IMyShipController : null);
    if (remoteBlock == null)
    {
        Echo("Error: Missing Remote Control.");
    }
    return remoteBlock;
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

void InitThrusters(bool isFixedDirection)
{
    //---------- Find Forward Thrusters ----------

    List<IMyTerminalBlock> checkThrusters = thrusters;
    thrusters = new List<IMyTerminalBlock>();

    if (!isFixedDirection || boolNaturalDampener == null || boolDrift == null)
    {
        IMyTerminalBlock leftThruster = null;
        IMyTerminalBlock rightThruster = null;
        IMyTerminalBlock upThruster = null;
        IMyTerminalBlock downThruster = null;

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
                        refDownwardBlock = upThruster;
                    }
                    break;
                case Base6Directions.Direction.Down:
                    downThruster = checkThrusters[i];
                    downThrustTotal += thrustValues[i];
                    break;
            }

            checkThrusters[i].ApplyAction("OnOff_On");
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
                    refDownwardBlock = leftThruster;
                }
                else if (rightThrustTotal == highestThrust)
                {
                    refDownwardBlock = rightThruster;
                }
                else if (upThrustTotal == highestThrust)
                {
                    refDownwardBlock = upThruster;
                }
                else
                {
                    refDownwardBlock = downThruster;
                }
            }
            boolNaturalDampener = (refDownwardBlock != null);

            if (boolDrift == null)
            {
                float lowestThrust = Math.Min(Math.Min(Math.Min(leftThrustTotal, rightThrustTotal), upThrustTotal), downThrustTotal);
                boolDrift = (highestThrust > lowestThrust * 2);
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
                refDownwardBlock = checkThrusters[i];
            }

            checkThrusters[i].ApplyAction("OnOff_On");
        }

        if (boolNaturalDampener == true && refDownwardBlock == null)
        {
            Echo("Warning: Natural Gravity Dampener feature not possible as there are no Downward Thrusters found.");
            boolNaturalDampener = false;
        }
    }
}

float[] ComputeMaxThrustValues(List<IMyTerminalBlock> checkThrusters)
{
    float[] thrustValues = new float[checkThrusters.Count];

    for (int i = 0; i < checkThrusters.Count; i++)
    {
        thrustValues[i] = Math.Max(((IMyThrust)checkThrusters[i]).MaxEffectiveThrust, 0.00001f);
    }

    return thrustValues;
}

IMyTerminalBlock ComputeHighestThrustReference(List<IMyTerminalBlock> checkThrusters, float[] thrustValues)
{
    if (checkThrusters.Count == 0)
    {
        return null;
    }

    IMyTerminalBlock fwdThruster = null;
    IMyTerminalBlock bwdThruster = null;
    IMyTerminalBlock leftThruster = null;
    IMyTerminalBlock rightThruster = null;
    IMyTerminalBlock upThruster = null;
    IMyTerminalBlock downThruster = null;

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

//------------------------------ Thruster Control Methods ------------------------------

void FireThrusters()
{
    if (thrusters != null)
    {
        for (int i = 0; i < thrusters.Count; i++)
        {
            thrusters[i].SetValue<float>("Override", thrusters[i].GetMaximum<float>("Override"));
        }
    }
}

void DisableAllThrusters()
{
    List<IMyTerminalBlock> blocks = new List<IMyTerminalBlock>();
    GridTerminalSystem.GetBlocksOfType<IMyThrust>(blocks);

    for (int i = 0; i < blocks.Count; i++)
    {
        blocks[i].ApplyAction("OnOff_Off");
    }
}

//------------------------------ Name Finder API ------------------------------

IMyTerminalBlock GetSingleBlockWithName(string name, bool sameGridOnly = false)
{
    List<IMyTerminalBlock> blocks = new List<IMyTerminalBlock>();
    GridTerminalSystem.SearchBlocksOfName(name, blocks);
    if (sameGridOnly)
    {
        FilterSameGrid(Me.CubeGrid, ref blocks);
    }

    return (blocks.Count > 0 ? blocks[0] : null);
}

List<IMyTerminalBlock> GetBlocksOfType<T>() where T: class, IMyTerminalBlock
{
    List<IMyTerminalBlock> blocks = new List<IMyTerminalBlock>();
    GridTerminalSystem.GetBlocksOfType<T>(blocks);

    return blocks;
}

List<T> GetBlocksOfTypeCasted<T>() where T: class, IMyTerminalBlock
{
    List<T> blocks = new List<T>();
    GridTerminalSystem.GetBlocksOfType<T>(blocks);

    return blocks;
}

List<IMyTerminalBlock> GetBlocksWithName(string name)
{
    List<IMyTerminalBlock> blocks = new List<IMyTerminalBlock>();
    GridTerminalSystem.SearchBlocksOfName(name, blocks);

    return blocks;
}

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

void FilterSameGrid<T>(IMyCubeGrid grid, ref List<T> blocks) where T: class, IMyTerminalBlock
{
    List<T> filtered = new List<T>();
    for (int i = 0; i < blocks.Count; i++)
    {
        if (blocks[i].CubeGrid == grid)
        {
            filtered.Add(blocks[i]);
        }
    }
    blocks = filtered;
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