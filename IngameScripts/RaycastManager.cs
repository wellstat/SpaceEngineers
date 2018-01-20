public class RaycastManager
{
    //Number of ticks before refreshing nextRaycastReadyRefresh
    public const int RAYCAST_READY_REFRESH_TICKS = 15;

    //Maximum Number of ticks allowed for a slipped target before declaring lock is lost
    public const int SLIP_LOST_THRESHOLD_TICKS = 120;

    //Threshold number of relock-on offset raycast hits to switch to use offset raycasting permanently
    public const int USE_OFFSET_THRESHOLD_HIT_COUNT = 5;

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

    //Percentage of the estimated available raycast to utilize. The remaining are reserved for emergency relock-on operations
    public float RaycastUsagePercent { get; set; } = 0.9f;

    //Percentage of the estimated reserved raycast to utilize for relock-on operations
    public float RelockUsagePercent { get; set; } = 0.5f;

    //Maximum frames to relock-on (1 frame is 8 points. Example 3 frames = Max 24 raycast to spam for relock-on)
    public int RelockMaxFrameDepth { get; set; } = 3;

    //Whether to allow lock-on to a close by target if initial raycast or relock failed to aquire same target
    public RelockCloseByType RelockCloseByEntityType { get; set; } = RelockCloseByType.AFTER_RELOCK;

    //Latest raycast info returned from the camera raycast method
    public MyDetectedEntityInfo LatestRaycastEntityInfo { get; protected set; }

    //Clock value of the last triggering of a raycast operation
    public int LatestRaycastTriggeredClock { get; protected set; } = -1;

    //Clock value of the first lock on via start tracking methods
    public int FirstRaycastLockedClock { get; protected set; } = -1;

    //Clock value of when the next raycast will be ready
    public int NextRaycastReadyClock { get; protected set; }

    //Current state of the raycast tracking
    public RaycastState CurrentRaycastState { get; protected set; }

    //The offset vector relative to the target's orientation. Used for offset targeting
    public Vector3D OffsetTargetVector { get; protected set; }

    //Tag object for external custom use
    public object Tag { get; set; }

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

            NextRaycastReadyClock = 0;
        }
    }

    int[,] relockPattern = new int[,] {{1,0},{-1,0},{0,1},{0,-1},{-1,-1},{1,-1},{-1,1},{1,1}};

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

    //Internal index for staggering cameras
    int staggerIndex;

    //Number of ticks to charge 1 meter of raycast per camera
    float ticksRatio;

    //Internal clock
    int clock;

    //Instantiate new class with the predefined set of cameras
    public RaycastManager(List<IMyCameraBlock> cameras)
    {
        Cameras = cameras;

        if (cameras.Count > 0)
        {
            ticksRatio = cameras[0].TimeUntilScan(cameras[0].AvailableScanRange + 1000) * 0.00006f;
        }
        else
        {
            //Default value of 60 ticks per 2km (1 sec = 60 ticks)
            ticksRatio = 0.03f;
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

            RefreshRaycastReadyClock(ref targetPosition, 0, (raycastStrategy == RaycastStrategy.DEFAULT ? 1 : 2));

            IMyCameraBlock camera;
            if (GetRaycastable(ref targetPosition, 0, out camera, out targetPosition))
            {
                MyDetectedEntityInfo entityInfo = camera.Raycast(targetPosition);
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
                            double dot = aimForwardVector.Dot(t_AimVector);
                            if (dot > -1 && dot < 1)
                            {
                                if (raycastModValue <= 0) raycastModValue = 0.5;

                                Vector3D aimSideVector = (dot * aimForwardVector) - t_AimVector;
                                double angle = Math.Acos(dot);
                                aimSideVector /= Math.Sin(angle);
                                angle = angle / ((clock - t_AimClock) * extraRaycastCount) * 60f * raycastModValue;
                                for (int i = 1; i <= extraRaycastCount; i++)
                                {
                                    testPositions.Add(aimStartPosition + (((aimForwardVector * Math.Cos(angle * i)) + (aimSideVector * Math.Sin(angle * i))) * distance));
                                }
                            }
                        }
                        else //RaycastStrategy.GHOST_CIRCULAR
                        {
                            if (raycastModValue <= 0) raycastModValue = 0.2;

                            MatrixD aimViewMatrix = MatrixD.CreateFromDir(aimForwardVector);
                            double changeDistance = distance * Math.Sin(MathHelperD.ToRadians(raycastModValue));
                            double changeAngle = Math.PI * 2 / extraRaycastCount;
                            for (int i = 1; i <= extraRaycastCount; i++)
                            {
                                Vector3D changeVector = new Vector3D(Math.Sin(changeAngle * i), Math.Cos(changeAngle * i), 0);
                                testPositions.Add(aimStartPosition + (aimForwardVector * distance) + (Vector3D.TransformNormal(changeVector, aimViewMatrix) * changeDistance));
                            }
                        }

                        for (int i = 0; i < testPositions.Count; i++)
                        {
                            targetPosition = testPositions[i];
                            if (GetRaycastable(ref targetPosition, 0, out camera, out targetPosition))
                            {
                                entityInfo = camera.Raycast(targetPosition);
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
            RefreshRaycastReadyClock(ref targetPosition, 0);

            IMyCameraBlock camera;
            if (GetRaycastable(ref targetPosition, 0, out camera, out targetPosition))
            {
                MyDetectedEntityInfo entityInfo = camera.Raycast(targetPosition);
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

        //Check if target center is raycastable
        Vector3D targetPosition = entityInfo.Position;
        IMyCameraBlock camera;
        if (reservedRaycastCharges > 0 && GetRaycastable(ref targetPosition, 0, out camera, out targetPosition))
        {
            MyDetectedEntityInfo syncEntityInfo = camera.Raycast(targetPosition);
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
                double targetRadius = LatestRaycastEntityInfo.BoundingBox.HalfExtents.Length();
                Vector3D travelVector = LatestRaycastEntityInfo.Velocity / 60f * (clock - LatestRaycastTriggeredClock);
                Vector3D targetPosition = LatestRaycastEntityInfo.Position + travelVector;
                if (useOffsetRaycasting)
                {
                    targetPosition += Vector3D.TransformNormal(offsetRaycastVector, LatestRaycastEntityInfo.Orientation);
                }

                RefreshRaycastReadyClock(ref targetPosition, targetRadius);

                IMyCameraBlock camera;
                if (GetRaycastable(ref targetPosition, targetRadius, out camera, out targetPosition))
                {
                    MyDetectedEntityInfo entityInfo = camera.Raycast(targetPosition);
                    if (!entityInfo.IsEmpty() && entityInfo.EntityId == LatestRaycastEntityInfo.EntityId)
                    {
                        //Raycast successfully maintained on target
                        SetRaycastTargetInfo(ref entityInfo, useOffsetRaycasting);
                    }
                    else //Target not found. Performing aggressive relock-on
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
                            relockStatus = AttemptRelock(origin, targetRadius, ref entityInfo); //Try without offset
                        }
                        else //Not using offset previously
                        {
                            relockStatus = AttemptRelock(origin + Vector3D.TransformNormal(offsetRaycastVector, LatestRaycastEntityInfo.Orientation), targetRadius, ref entityInfo); //Try with offset
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
                            relockStatus = AttemptRelock(LatestRaycastEntityInfo.Position, targetRadius, ref entityInfo); //Try previous position
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
                                    relockStatus = AttemptRelock(origin + (travelVector * i * relockPattern[j,0]) + (verticalVector * i * relockPattern[j,1]), targetRadius, ref entityInfo);
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
                }
                else
                {
                    CurrentRaycastState = RaycastState.TARGET_SLIPPED;
                }
            }
        }
    }

    RaycastResult AttemptRelock(Vector3D targetPosition, double targetRadius, ref MyDetectedEntityInfo closestEntityInfo)
    {
        if (reservedRaycastCharges > 0)
        {
            IMyCameraBlock camera;
            if (GetRaycastable(ref targetPosition, targetRadius, out camera, out targetPosition))
            {
                MyDetectedEntityInfo entityInfo = camera.Raycast(targetPosition);
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
    }

    void RefreshRaycastReadyClock(ref Vector3D targetPosition, double overshootDistance, int scaleDelay = 1)
    {
        if (clock >= nextRaycastReadyRefresh)
        {
            nextRaycastReadyRefresh = clock + RAYCAST_READY_REFRESH_TICKS;

            int count = 0;
            int charges = 0;
            double maxDist = 0;
            for (int i = 0; i < m_cameras.Count; i++)
            {
                IMyCameraBlock camera = m_cameras[i];
                if (camera.IsWorking)
                {
                    Vector3D direction = targetPosition - camera.GetPosition();
                    double distance = direction.Length();
                    direction /= distance;

                    if (camera.CanScan(targetPosition + (direction * overshootDistance)))
                    {
                        distance += overshootDistance;
                        maxDist = Math.Max(maxDist, distance);
                        count++;
                        charges += (int)Math.Floor(camera.AvailableScanRange / distance);
                    }
                }
            }
            if (maxDist == 0 && m_cameras.Count > 0)
            {
                maxDist = (targetPosition - m_cameras[0].GetPosition()).Length() + overshootDistance;
            }

            float available = (count > 1 ? count * RaycastUsagePercent: RaycastUsagePercent);

            float factor = ticksRatio / available;
            nextRaycastDelay = (int)Math.Ceiling(maxDist * factor);

            reservedRaycastCharges = (int)Math.Floor((charges - available) * RelockUsagePercent);
        }

        NextRaycastReadyClock = clock + (nextRaycastDelay * scaleDelay);
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

    bool GetRaycastable(ref Vector3D targetPosition, double overshootDistance, out IMyCameraBlock readyCamera, out Vector3D newTargetPosition)
    {
        for (int i = 0; i < m_cameras.Count; i++)
        {
            if (staggerIndex >= m_cameras.Count)
            {
                staggerIndex = 0;
            }
            IMyCameraBlock camera = m_cameras[staggerIndex];
            if (camera.IsWorking)
            {
                Vector3D aimPoint;
                if (overshootDistance == 0)
                {
                    aimPoint = targetPosition;
                }
                else
                {
                    Vector3D aimVector = targetPosition - camera.GetPosition();
                    double distance = aimVector.Length();
                    aimPoint = targetPosition + (aimVector / distance * overshootDistance);
                }

                if (camera.CanScan(aimPoint))
                {
                    newTargetPosition = aimPoint;
                    readyCamera = camera;
                    return true;
                }
            }
            staggerIndex++;
        }
        newTargetPosition = targetPosition;
        readyCamera = null;
        return false;
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
            outTargetPosition = LatestRaycastEntityInfo.Position + (outTargetVelocity / 60f * (clock - LatestRaycastTriggeredClock));
            if (UseOffsetTargeting && OffsetTargetVector.Sum > 0)
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

    public enum RaycastStrategy
    {
        DEFAULT,        //Shoot only one single raycast
        GHOST_AHEAD,    //Shoot additional raycast ahead to compensate for lag
        GHOST_CIRCULAR  //Shoot additional raycast around aim point to compensate for jitter
    }

    public enum RelockCloseByType
    {
        DISABLED,       //No relocking to close by target
        IMMEDIATE,      //Relock to close by target upon detecting suitable
        AFTER_RELOCK    //Relock to close by only when all relock attempts failed
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