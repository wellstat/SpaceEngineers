# Helper Codes

This folder contains code snippets and helper classes to be included into other ingame scripts.


## RaycastManager.cs

Helper class for managing raycast lock-on. Camera selection, raycast frequency, cooldown based on distance, and lock lost re-lock attempts are all automatically managed.

### Instantiation

Class is to be instantiated with the constructor. Example below:
```
RaycastManager raycastManager = new RaycastManager(cameras);
```
***cameras** is a list extracted from GridTerminalSystem as **List\<IMyCameraBlock\>***

### Update Every Script Run

The RaycastManager **MUST** have its **Update()** method called every script loop at **Update1 Frequency**. No more, no less.
```
raycastManager.Update();
```
*Preferably called near the start of the script.*

If there are no targets, you can still keep calling for convienence. The class will auto filter and do nothing if no targets are locked on.

### Initial Lock Aquisition

For initial aiming, the following 2 methods can be used:

**StartTrackingByPosition()** - Start tracking by attempting to lock on to a specific coordinate. Example below:
```
raycastManager.StartTrackingByPosition(targetPosition);
raycastManager.StartTrackingByPosition(targetPosition, detectedEntityInfo.EntityId);
```
*detectedEntityInfo.EntityId is an additional parameter to ensure it only locks on to a particular grid.*

**StartTrackingByVector()** - Start tracking by attempting to lock on to a target in the specified forward direction with a max distance. Example below:
```
raycastManager.StartTrackingByVector(aimingCameraBlock, 5000);
raycastManager.StartTrackingByVector(aimingTurretBlock, 5000);
raycastManager.StartTrackingByVector(remoteControl.GetPosition(), remoteControl.WorldMatrix.Forward, 5000);
```
*aimingCameraBlock is a reference block for aiming. If this block is a turret, the turret's aimed point (azimuth and elevation) is used instead of its absolute grid forward.*

**StartTrackingByVector()** also support 2 additional lock-on patterns, Ghost Ahead and Ghost Circular.

Ghost Ahead will spend additional raycasts to trace ahead of your aim point. Used primary for lag compensation (including gyro lag and rubberband):
```
raycastManager.StartTrackingByVector(aimingCameraBlock, 5000, RaycastManager.RaycastStrategy.GHOST_AHEAD, 4, 0.5);
```
*Example uses Ghost Ahead to trace 4 additional raycasts to compensate an estimated 0.5 seconds of user input movements*

Ghost Circular will spend additional raycasts to trace a circle around your aim point. Used primary for lag compensation (including gyro lag and rubberband):
```
raycastManager.StartTrackingByVector(aimingCameraBlock, 5000, RaycastManager.RaycastStrategy.GHOST_CIRCULAR, 6, 0.2);
```
*Example uses Ghost Circular to trace 6 additional raycasts around the aim point with 0.2 degree deviations*

**RaycastResult.RAYCAST_SUCCESS** is returned when lock-on is successful. The lock is automatically maintained by calling the **Update()** method every script run as described above.

### Get Locked-on Target Information

To get the locked-on target information, use the following method **AFTER** the **Update()** method is called:

```
raycastManager.GetRaycastTarget(out outTargetPosition, out outTargetVelocity, out outEntityId);
```


## PIDController.cs

Helper class for implementing a PID controller.


## CustomConfiguration.cs

Helper class for reading simple configuration from Custom Data of Terminal Blocks.
The values are **case insensitive** and whitespaces are automatically trimmed.

Values are specified as key=value pairs:
Example:
```
raycastCount=5
launchDelay=4
maxRange=5000
```


## GyroControl.cs

Helper class for controlling gyroscopes for aiming. Gyroscopes can be on subgrids, but MUST not change orientation (rotor moving etc) after this class is instantiated.


## GyroControlFixedGrid.cs

Helper class for controlling gyroscopes for aiming. Gyroscopes MUST be on the same grid as the Programmable Block.


## VectorAverageFilter.cs

Helper class to average out Vector3D values.

