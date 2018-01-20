//============================================================
// Script to allow mouse aiming of a Rotor Turret. Used on
// almost all my rotor turret builds. Correlated means
// filtering to take the nearest Azimuth and Elevation rotor
// sets, so that you can have multiple Rotor Turrets with
// same Azimuth and Elevation rotor names and it will select
// the closest ones properly based on grid (elevation rotor
// must be on Azimuth rotor's TopGrid etc).
//============================================================

//------------------------------------------------------------
// ADN - Rotor Turret Correlated Script v1.0
//------------------------------------------------------------
string COCKPIT_TAG = "Cockpit";
string ROTOR_X_TAG = "Azimuth";
string ROTOR_Y_TAG = "Elevation";
string REVERSE_TAG = "Reverse";

float INPUT_SENSITIVITY = 1f;
float ROTOR_SPEED_LIMIT = 30f;

bool REQUIRE_HANDBRAKE_TO_TURN = false;

IMyCockpit cockpit = null;
List<TurretRotor> rotorX = null;
List<TurretRotor> rotorY = null;

bool init = false;

Program()
{
    Runtime.UpdateFrequency = UpdateFrequency.Update1;
}

void Main(string arguments, UpdateType updateSource)
{
    if (!init)
    {
        if (Me.CustomData.Length > 0)
        {
            ProcessCustomConfiguration();
        }

        List<IMyCockpit> rawCockpits = new List<IMyCockpit>();
        List<IMyMotorStator> rawRotorsX = new List<IMyMotorStator>();
        List<IMyMotorStator> rawRotorsY = new List<IMyMotorStator>();

        List<IMyTerminalBlock> blocks = new List<IMyTerminalBlock>();
        GridTerminalSystem.SearchBlocksOfName(COCKPIT_TAG, blocks);
        for (int i = 0; i < blocks.Count; i++)
        {
            IMyCockpit cockpit = blocks[i] as IMyCockpit;
            if (cockpit != null)
            {
                rawCockpits.Add(cockpit);
            }
        }

        blocks.Clear();
        GridTerminalSystem.SearchBlocksOfName(ROTOR_X_TAG, blocks);
        for (int i = 0; i < blocks.Count; i++)
        {
            IMyMotorStator rotor = blocks[i] as IMyMotorStator;
            if (rotor != null)
            {
                rawRotorsX.Add(rotor);
            }
        }

        blocks.Clear();
        GridTerminalSystem.SearchBlocksOfName(ROTOR_Y_TAG, blocks);
        for (int i = 0; i < blocks.Count; i++)
        {
            IMyMotorStator rotor = blocks[i] as IMyMotorStator;
            if (rotor != null)
            {
                rawRotorsY.Add(rotor);
            }
        }

        FilterByCorrelation(ref rawCockpits, ref rawRotorsX, ref rawRotorsY);

        if (rawCockpits.Count > 0 && rawRotorsX.Count > 0 && rawRotorsY.Count > 0)
        {
            cockpit = rawCockpits[0];

            rotorX = new List<TurretRotor>();
            for (int i = 0; i < rawRotorsX.Count; i++)
            {
                TurretRotor turretRotor = new TurretRotor();
                turretRotor.Rotor = rawRotorsX[i];
                turretRotor.Reverse = (rawRotorsX[i].CustomName.IndexOf(REVERSE_TAG, StringComparison.OrdinalIgnoreCase) > -1);
                rotorX.Add(turretRotor);
            }

            rotorY = new List<TurretRotor>();
            for (int i = 0; i < rawRotorsY.Count; i++)
            {
                TurretRotor turretRotor = new TurretRotor();
                turretRotor.Rotor = rawRotorsY[i];
                turretRotor.Reverse = (rawRotorsY[i].CustomName.IndexOf(REVERSE_TAG, StringComparison.OrdinalIgnoreCase) > -1);
                rotorY.Add(turretRotor);
            }

            init = true;
            return;
        }
        else
        {
            throw new Exception("Correlation Failed. Unable To Find\nUsable Cockpit And X Y Rotors");
        }
    }

    if (arguments.Length > 0)
    {
        string[] keyValues = arguments.Split(',');

        for (int i = 0; i < keyValues.Length; i++)
        {
            string[] tokens = keyValues[i].Trim().Split(':');
            if (tokens.Length > 0)
            {
                string configKey = tokens[0];
                float value;

                switch (configKey)
                {
                    case "INPUT_SENSITIVITY":
                        if (tokens.Length > 1 && tokens[1].Length > 0)
                        {
                            char sign = tokens[1][0];
                            if (sign == '+' || sign == '-')
                            {
                                tokens[1] = tokens[1].Substring(1);
                            }
                            if (float.TryParse(tokens[1], out value))
                            {
                                INPUT_SENSITIVITY = Math.Max((sign == '+' ? INPUT_SENSITIVITY + value : (sign == '-' ? INPUT_SENSITIVITY - value : value)), 0);
                            }
                        }
                        break;
                    case "ROTOR_SPEED_LIMIT":
                        if (tokens.Length > 1)
                        {
                            if (float.TryParse(tokens[1], out value))
                            {
                                if (value > 0)
                                {
                                    ROTOR_SPEED_LIMIT = value;
                                }
                            }
                        }
                        break;
                    case "SIT_ENABLE":
                        foreach (TurretRotor tRotor in rotorX)
                        {
                            tRotor.Rotor.SetValueBool("ShareInertiaTensor", true);
                        }
                        foreach (TurretRotor tRotor in rotorY)
                        {
                            tRotor.Rotor.SetValueBool("ShareInertiaTensor", true);
                        }
                        break;
                    case "SIT_DISABLE":
                        foreach (TurretRotor tRotor in rotorX)
                        {
                            tRotor.Rotor.SetValueBool("ShareInertiaTensor", false);
                        }
                        foreach (TurretRotor tRotor in rotorY)
                        {
                            tRotor.Rotor.SetValueBool("ShareInertiaTensor", false);
                        }
                        break;
                    case "SIT_AZ_ENABLE":
                        foreach (TurretRotor tRotor in rotorX)
                        {
                            tRotor.Rotor.SetValueBool("ShareInertiaTensor", true);
                        }
                        break;
                    case "SIT_AZ_DISABLE":
                        foreach (TurretRotor tRotor in rotorX)
                        {
                            tRotor.Rotor.SetValueBool("ShareInertiaTensor", false);
                        }
                        break;
                    case "SIT_EV_ENABLE":
                        foreach (TurretRotor tRotor in rotorY)
                        {
                            tRotor.Rotor.SetValueBool("ShareInertiaTensor", true);
                        }
                        break;
                    case "SIT_EV_DISABLE":
                        foreach (TurretRotor tRotor in rotorY)
                        {
                            tRotor.Rotor.SetValueBool("ShareInertiaTensor", false);
                        }
                        break;
                    default:
                        break;
                }
            }
        }
    }

    if (!REQUIRE_HANDBRAKE_TO_TURN || cockpit.HandBrake)
    {
        float rotorXVelocity = (cockpit.IsUnderControl ? cockpit.RotationIndicator[1] * INPUT_SENSITIVITY : 0f);
        float rotorYVelocity = (cockpit.IsUnderControl ? cockpit.RotationIndicator[0] * INPUT_SENSITIVITY : 0f);

        rotorXVelocity = Math.Min(Math.Max(rotorXVelocity, -ROTOR_SPEED_LIMIT), ROTOR_SPEED_LIMIT);
        rotorYVelocity = Math.Min(Math.Max(rotorYVelocity, -ROTOR_SPEED_LIMIT), ROTOR_SPEED_LIMIT);

        for (int i = 0; i < rotorX.Count; i++)
        {
            rotorX[i].Rotor.SetValue("Velocity", (rotorX[i].Reverse ? -rotorXVelocity : rotorXVelocity));
        }

        for (int i = 0; i < rotorY.Count; i++)
        {
            rotorY[i].Rotor.SetValue("Velocity", (rotorY[i].Reverse ? rotorYVelocity : -rotorYVelocity));
        }
    }
}

void FilterByCorrelation(ref List<IMyCockpit> rawCockpits, ref List<IMyMotorStator> rawRotorsX, ref List<IMyMotorStator> rawRotorsY)
{
    IMyCockpit nearestCockpit = null;
    foreach (IMyCockpit cockpit in rawCockpits)
    {
        if (nearestCockpit == null || (Me.GetPosition() - cockpit.GetPosition()).Length() < (Me.GetPosition() - nearestCockpit.GetPosition()).Length())
        {
            nearestCockpit = cockpit;
        }
    }

    rawCockpits.Clear();
    if (nearestCockpit != null)
    {
        rawCockpits.Add(nearestCockpit);
    }
    else    //No Cockpits Found. Abort Correlation And Fail Initialization
    {
        return;
    }

    IMyMotorStator nearestRotorX = null;
    foreach (IMyMotorStator rotor in rawRotorsX)
    {
        if (nearestRotorX == null || (Me.GetPosition() - rotor.GetPosition()).Length() < (Me.GetPosition() - nearestRotorX.GetPosition()).Length())
        {
            nearestRotorX = rotor;
        }
    }

    rawRotorsX.Clear();
    if (nearestRotorX != null)
    {
        rawRotorsX.Add(nearestRotorX);
    }
    else    //No Cockpits Found. Abort Correlation And Fail Initialization
    {
        return;
    }

    if (nearestRotorX.TopGrid != null)
    {
        List<IMyMotorStator> filteredRotorY = new List<IMyMotorStator>();
        foreach (IMyMotorStator rotor in rawRotorsY)
        {
            if (rotor.CubeGrid == nearestRotorX.TopGrid)
            {
                filteredRotorY.Add(rotor);
            }
        }
        rawRotorsY = filteredRotorY;
    }
    else
    {
        rawRotorsY.Clear();
    }
}

void ProcessCustomConfiguration()
{
    CustomConfiguration cfg = new CustomConfiguration(Me);
    cfg.Load();

    cfg.Get("COCKPIT_TAG", ref COCKPIT_TAG);
    cfg.Get("ROTOR_X_TAG", ref ROTOR_X_TAG);
    cfg.Get("ROTOR_Y_TAG", ref ROTOR_Y_TAG);
    cfg.Get("REVERSE_TAG", ref REVERSE_TAG);
    cfg.Get("INPUT_SENSITIVITY", ref INPUT_SENSITIVITY);
    cfg.Get("ROTOR_SPEED_LIMIT", ref ROTOR_SPEED_LIMIT);
    cfg.Get("REQUIRE_HANDBRAKE_TO_TURN", ref REQUIRE_HANDBRAKE_TO_TURN);
}

public struct TurretRotor
{
    public IMyMotorStator Rotor;
    public bool Reverse;
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