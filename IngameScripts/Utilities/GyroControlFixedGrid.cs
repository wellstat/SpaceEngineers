public class GyroControlFixedGrid
{
    string[] profiles = {"Yaw","Yaw","Pitch","Pitch","Roll","Roll"};

    List<IMyGyro> gyros;

    private byte[] gyroYaw;
    private byte[] gyroPitch;
    private byte[] gyroRoll;

    public GyroControlFixedGrid(List<IMyTerminalBlock> newGyros, MatrixD refWorldMatrix)
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