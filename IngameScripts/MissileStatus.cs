//============================================================
// Raw version of the published script on my workshop.
//============================================================

//------------------------------------------------------------
// ADN - Missile Status Script v3.2
//------------------------------------------------------------

static string strComputerTag = "MS_COM";                        //Prefix name of the Programmable Blocks to monitor
static string strStatusPanelMainTag = "MS_STATUS";              //Name of the Text Panel to output main missile status
static string strStatusPanelExtendedTag = "MS_STATUS_EXT";      //Name of the Text Panel to output extended missile status
static string strMissileLockAlertTag = "MS_ALERT";              //Name of the Block to when alert when missile is locked
static string strMissileLockAlertAction = "PlaySound";          //Action to trigger for the specified strMissileLockAlertTag

static string strMissileLockAlarmBlocksTag = "MS_ALARM_BLOCK";  //Tag name for Blocks to turn on when missile is locked
static float missileLockAlarmBlocksDuration = 1f;               //Duration in seconds for the blocks to be turned on before turning them off

static int refreshInterval = 10;                                //How many ticks to wait for each status refresh

//------------------------------ Below Is Main Script Body ------------------------------

List<IMyTerminalBlock> missileComputers = null;
bool[] isLockedState = null;

IMyTextPanel statusPanelMain = null;
IMyTextPanel statusPanelExtended = null;

IMyTerminalBlock missileLockAlert = null;
IMyProgrammableBlock missileLockAlertPB = null;

List<IMyTerminalBlock> alarmBlocks = null;
int alarmOnRemaining = 0;
bool alarmOnFlag = false;

int clock = 0;
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

        missileComputers = GetBlocksWithName<IMyProgrammableBlock>(strComputerTag);

        List<IMyTerminalBlock> blocks = GetBlocksWithName<IMyTextPanel>(strStatusPanelMainTag);
        if (blocks == null || blocks.Count == 0)
        {
            throw new Exception("Error: Text Panel with name " + strStatusPanelMainTag + " is not found");
        }
        statusPanelMain = blocks[0] as IMyTextPanel;

        blocks = GetBlocksWithName<IMyTextPanel>(strStatusPanelExtendedTag);
        if (blocks != null && blocks.Count > 0)
        {
            statusPanelExtended = blocks[0] as IMyTextPanel;
        }

        blocks = GetBlocksWithName<IMyTerminalBlock>(strMissileLockAlertTag);
        if (blocks != null && blocks.Count > 0)
        {
            missileLockAlert = blocks[0] as IMyTerminalBlock;
            missileLockAlertPB = blocks[0] as IMyProgrammableBlock;
        }

        if (strMissileLockAlarmBlocksTag != null && strMissileLockAlarmBlocksTag.Length > 0)
        {
            blocks = GetBlocksWithName<IMyTerminalBlock>(strMissileLockAlarmBlocksTag);

            alarmBlocks = new List<IMyTerminalBlock>();
            for (int i = 0; i < blocks.Count; i++)
            {
                if (blocks[i].HasAction("OnOff_On"))
                {
                    alarmBlocks.Add(blocks[i]);
                }
            }

            if (alarmBlocks.Count == 0)
            {
                alarmBlocks = null;
            }
        }

        if (missileLockAlert != null || alarmBlocks != null)
        {
            isLockedState = new bool[missileComputers.Count];
        }

        clock = 0;

        init = true;
        return;
    }

    if (arguments.Length > 0)
    {
        if (arguments.Equals("RESET"))
        {
            missileComputers = GetBlocksWithName<IMyProgrammableBlock>(strComputerTag);

            if (missileLockAlert != null || alarmBlocks != null)
            {
                isLockedState = new bool[missileComputers.Count];
            }
        }

        return;
    }

    if (missileComputers.Count == 0 && clock % 60 == 0)
    {
        missileComputers = GetBlocksWithName<IMyProgrammableBlock>(strComputerTag);

        if (missileLockAlert != null || alarmBlocks != null)
        {
            isLockedState = new bool[missileComputers.Count];
        }
    }

    if (alarmOnFlag)
    {
        alarmOnRemaining--;

        if (alarmOnRemaining <= 0)
        {
            alarmOnRemaining = 0;
            alarmOnFlag = false;

            if (alarmBlocks != null)
            {
                for (int i = 0; i < alarmBlocks.Count; i++)
                {
                    alarmBlocks[i].ApplyAction("OnOff_Off");
                }
            }
        }
    }

    if (clock % refreshInterval == 0)
    {
        statusPanelMain.WritePublicText("");
        if (statusPanelExtended != null)
        {
            statusPanelExtended.WritePublicText("");
        }

        for (int i = 0; i < missileComputers.Count; i++)
        {
            if (missileComputers[i].IsWorking)
            {
                string statusOutput = missileComputers[i].DetailedInfo;

                if (statusOutput.StartsWith("ST:"))
                {
                    string[] statusTokens = statusOutput.Split(':');
                    string statusMessage;
                    string extendedMessage;

                    switch (statusTokens[5])
                    {
                        case "W":
                            statusMessage = "Awaiting Command";
                            extendedMessage = "-";
                            break;
                        case "F":
                            statusMessage = "Launching";
                            extendedMessage = "-";
                            break;
                        case "K":
                            statusMessage = "Seeking";
                            extendedMessage = "-";
                            break;
                        case "L":
                            statusMessage = "Locked";
                            extendedMessage = FormatVectorString(statusTokens[6], statusTokens[7], statusTokens[8], 0);
                            break;
                        case "T":
                            statusMessage = "Tracing";
                            extendedMessage = FormatVectorString(statusTokens[6], statusTokens[7], statusTokens[8], 0);
                            break;
                        case "C":
                            statusMessage = "Camera";
                            extendedMessage = FormatVectorString(statusTokens[6], statusTokens[7], statusTokens[8], 0);
                            break;
                        case "D":
                            statusMessage = "Cruise";
                            extendedMessage = FormatVectorString(statusTokens[6], statusTokens[7], statusTokens[8], 0);
                            break;
                        case "G":
                            statusMessage = "Glide";
                            extendedMessage = FormatVectorString(statusTokens[6], statusTokens[7], statusTokens[8], 0);
                            break;
                        case "U":
                            statusMessage = "Turret Locked";
                            extendedMessage = FormatVectorString(statusTokens[6], statusTokens[7], statusTokens[8], 0);
                            break;
                        case "S":
                            statusMessage = "Sensor Locked";
                            extendedMessage = FormatVectorString(statusTokens[6], statusTokens[7], statusTokens[8], 0);
                            break;
                        default:
                            statusMessage = "Unknown";
                            extendedMessage = "-";
                            break;
                    }

                    statusPanelMain.WritePublicText("#" + (i + 1) + ": " + statusMessage + "\n", true);
                    if (statusPanelExtended != null)
                    {
                        statusPanelExtended.WritePublicText("#" + (i + 1) + ": " + extendedMessage + "\n", true);
                    }

                    if (missileLockAlert != null || alarmBlocks != null)
                    {
                        if (!isLockedState[i] && statusTokens[5].Equals("L"))
                        {
                            isLockedState[i] = true;

                            if (missileLockAlertPB != null)
                            {
                                missileLockAlertPB.TryRun(strMissileLockAlertAction);
                            }
                            else if (missileLockAlert != null)
                            {
                                if (missileLockAlert.HasAction(strMissileLockAlertAction))
                                {
                                    missileLockAlert.ApplyAction(strMissileLockAlertAction);
                                }
                            }

                            if (alarmBlocks != null)
                            {
                                for (int j = 0; j < alarmBlocks.Count; j++)
                                {
                                    alarmBlocks[j].ApplyAction("OnOff_On");
                                }

                                alarmOnRemaining = (int)(missileLockAlarmBlocksDuration * 60);
                                alarmOnFlag = true;
                            }
                        }
                    }
                }
                else
                {
                    statusPanelMain.WritePublicText("#" + (i + 1) + ": Idle\n", true);
                    if (statusPanelExtended != null)
                    {
                        statusPanelExtended.WritePublicText("#" + (i + 1) + ": -\n", true);
                    }
                }
            }
            else
            {
                statusPanelMain.WritePublicText("#" + (i + 1) + ": Destroyed\n", true);
                if (statusPanelExtended != null)
                {
                    statusPanelExtended.WritePublicText("#" + (i + 1) + ": -\n", true);
                }
            }
        }
    }
}

void ProcessCustomConfiguration()
{
    CustomConfiguration cfg = new CustomConfiguration(Me);
    cfg.Load();

    cfg.Get("strComputerTag", ref strComputerTag);
    cfg.Get("strStatusPanelMainTag", ref strStatusPanelMainTag);
    cfg.Get("strStatusPanelExtendedTag", ref strStatusPanelExtendedTag);
    cfg.Get("strMissileLockAlertTag", ref strMissileLockAlertTag);
    cfg.Get("strMissileLockAlertAction", ref strMissileLockAlertAction);
    cfg.Get("refreshInterval", ref refreshInterval);
    cfg.Get("strMissileLockAlarmBlocksTag", ref strMissileLockAlarmBlocksTag);
    cfg.Get("missileLockAlarmBlocksDuration", ref missileLockAlarmBlocksDuration);
}

//------------------------------ Misc API ------------------------------

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

string FormatVectorString(string posX, string posY, string posZ, int decimals)
{
    double value;

    if (Double.TryParse(posX, out value))
    {
        posX = Math.Round(value, decimals).ToString();
    }
    else
    {
        posX = "NaN";
    }

    if (Double.TryParse(posY, out value))
    {
        posY = Math.Round(value, decimals).ToString();
    }
    else
    {
        posY = "NaN";
    }

    if (Double.TryParse(posZ, out value))
    {
        posZ = Math.Round(value, decimals).ToString();
    }
    else
    {
        posZ = "NaN";
    }

    return "X: " + posX + ", Y: " + posY + ", Z: " + posZ;
}