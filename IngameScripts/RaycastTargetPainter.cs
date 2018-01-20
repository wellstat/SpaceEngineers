//============================================================
// Script powering my Raycast Target Painter Ship.
//============================================================

//------------------------------------------------------------
// ADN - Raycast Target Painter Script v1.0
//------------------------------------------------------------
IMyCameraBlock lidar = null;
IMyTextPanel targetPanel = null;
IMyTextPanel statusPanel = null;

double distance = 10000;

bool init = false;

Program()
{
    Runtime.UpdateFrequency = UpdateFrequency.Update100;
}

void Main(string arguments, UpdateType updateSource)
{
    if (!init)
    {
        lidar = GridTerminalSystem.GetBlockWithName("Ship Camera") as IMyCameraBlock;
        targetPanel = GridTerminalSystem.GetBlockWithName("Target Panel") as IMyTextPanel;
        statusPanel = GridTerminalSystem.GetBlockWithName("Status Panel") as IMyTextPanel;

        if (lidar != null)
        {
            lidar.EnableRaycast = true;
            lidar.ApplyAction("OnOff_On");
        }

        init = (lidar != null && targetPanel != null && statusPanel != null);
        if (!init) return;
    }

    if (arguments.Length > 0)
    {
        string[] tokens = arguments.Trim().Split(':');
        string cmdToken = (tokens.Length > 0 ? tokens[0].Trim().ToUpper() : "");

        float fval;

        switch (cmdToken)
        {
            case "DISTANCE":
                if (tokens.Length > 1 && tokens[1].Length > 0)
                {
                    char sign = tokens[1][0];
                    if (sign == '+' || sign == '-')
                    {
                        tokens[1] = tokens[1].Substring(1);
                    }
                    if (float.TryParse(tokens[1], out fval))
                    {
                        distance = Math.Max((sign == '+' ? distance + fval : (sign == '-' ? distance - fval : fval)), 0);
                    }
                }
                break;
            case "SCAN":
                MyDetectedEntityInfo entityInfo = lidar.Raycast(Math.Min(distance, lidar.AvailableScanRange));

                StringBuilder sbTarget = new StringBuilder();
                StringBuilder sbGPS = new StringBuilder();

                if (entityInfo.IsEmpty())
                {
                    sbTarget.Append("-- No Targets Found --");
                }
                else
                {
                    sbTarget.Append("EnityId: ").Append(entityInfo.EntityId).Append('\n');
                    sbTarget.Append("Distance: ").Append(Math.Round(Vector3D.Distance(lidar.GetPosition(), (entityInfo.HitPosition == null ? entityInfo.Position : entityInfo.HitPosition.Value)), 2)).Append("m\n");
                    sbTarget.Append("Radius: ").Append(Math.Round(Vector3D.Distance(entityInfo.BoundingBox.Min, entityInfo.BoundingBox.Max), 2)).Append("m\n");
                    sbTarget.Append("\nHit Position X: ").Append(Math.Round((entityInfo.HitPosition == null ? entityInfo.Position : entityInfo.HitPosition.Value).X, 2)).Append('\n');
                    sbTarget.Append("Hit Position Y: ").Append(Math.Round((entityInfo.HitPosition == null ? entityInfo.Position : entityInfo.HitPosition.Value).Y, 2)).Append('\n');
                    sbTarget.Append("Hit Position Z: ").Append(Math.Round((entityInfo.HitPosition == null ? entityInfo.Position : entityInfo.HitPosition.Value).Z, 2)).Append('\n');
                    sbTarget.Append("\nCenter X: ").Append(Math.Round(entityInfo.Position.X, 2)).Append('\n');
                    sbTarget.Append("Center Y: ").Append(Math.Round(entityInfo.Position.Y, 2)).Append('\n');
                    sbTarget.Append("Center Z: ").Append(Math.Round(entityInfo.Position.Z, 2)).Append('\n');

                    sbGPS.Append("GPS:Hit Position:").Append(VectorToString((entityInfo.HitPosition == null ? entityInfo.Position : entityInfo.HitPosition.Value), 2)).Append(":\n");
                    sbGPS.Append("GPS:Center:").Append(VectorToString(entityInfo.Position, 2)).Append(":\n");
                }

                targetPanel.WritePublicText(sbTarget);
                targetPanel.CustomData = sbGPS.ToString();

                break;
        }
    }

    StringBuilder sb = new StringBuilder();

    sb.Append("Total Charges: ").Append(Math.Round(lidar.AvailableScanRange, 0)).Append("m\n");
    sb.Append("Scan Distance: ").Append(Math.Round(distance, 0)).Append("m\n");

    float coveragePercent = (float)Math.Min(lidar.AvailableScanRange / distance * 100, 100);
    sb.Append("\nCoverage: ").Append(Math.Round(coveragePercent, 0)).Append("%\n");
    DrawProgressBar(sb, statusPanel.GetValueFloat("FontSize"), coveragePercent).Append('\n');
    sb.Append("Available Scans Count: ").Append(Math.Floor(lidar.AvailableScanRange / distance)).Append('\n');

    statusPanel.WritePublicText(sb);
}

StringBuilder DrawProgressBar(StringBuilder sb, float fontSize, float percent)
{
    int total = (int)((1f / fontSize) * 72) - 2;
    int filled = (int)Math.Round(percent / 100 * total);
    sb.Append('[').Append('I', filled).Append('`', total - filled).Append(']');
    return sb;
}

string VectorToString(Vector3D vector, int decimals)
{
    return Math.Round(vector.GetDim(0), decimals) + ":" + Math.Round(vector.GetDim(1), decimals) + ":" + Math.Round(vector.GetDim(2), decimals);
}