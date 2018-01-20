public class GyroControl
{
    List<IMyGyro> gyros;
    private MatrixD[] gyroMatrix;

    public GyroControl(List<IMyTerminalBlock> newGyros, MatrixD refWorldMatrix)
    {
        gyros = new List<IMyGyro>(newGyros.Count);
        gyroMatrix = new MatrixD[newGyros.Count];

        Quaternion rotQ = Quaternion.CreateFromRotationMatrix(MatrixD.Transpose(refWorldMatrix));

        int index = 0;
        foreach (IMyTerminalBlock block in newGyros)
        {
            IMyGyro gyro = block as IMyGyro;
            if (gyro != null)
            {
                gyroMatrix[index] = MatrixD.Transpose(MatrixD.Transform(gyro.WorldMatrix, rotQ));
                gyros.Add(gyro);

                index++;
            }
        }

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

    public void SetGyroValues(float yaw, float pitch, float roll)
    {
        for (int i = 0; i < gyros.Count; i++)
        {
            Vector3 vec = Vector3.TransformNormal(new Vector3(-pitch, yaw, roll), gyroMatrix[i]);
            IMyGyro gyro = gyros[i];
            gyro.Yaw = vec.Y;
            gyro.Pitch = vec.X;
            gyro.Roll = vec.Z;
        }
    }

    public void ResetGyro()
    {
        foreach (IMyGyro gyro in gyros)
        {
            gyro.Yaw = 0f;
            gyro.Pitch = 0f;
            gyro.Roll = 0f;
        }
    }
}