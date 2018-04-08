//============================================================
// Raw version of the published script on my workshop.
//============================================================

//------------------------------------------------------------
// ADN - Launch Control Script v4.0
//------------------------------------------------------------

string strComputerTag = "Missile Computer";         //Name tag of the Programmable Block loaded with the Easy Lidar Homing Script.
string strDetachConnectorTag = "";                  //Name tag of the Connector closest to the Programmable Block to Unlock. Set blank to disable.
string strAdditionalCustomData = "";                //Additional Custom Data to be added to the Missile Programmable Block.

string listenerId = null;
string listenerGroup = null;
string allowedSenderId = null;

int launchSelectionType = 2;                        //0 = Any, 1 = Closest, 2 = Furthest

//------------------------------ Below Is Main Script Body ------------------------------

void Main(string arguments, UpdateType updateSource)
{
    bool initInterGridComms = ((updateSource & UpdateType.Antenna) > 0);

    if (initInterGridComms)
    {
        listenerId = Me.GetId().ToString();
        listenerGroup = null;
    }

    if (Me.CustomData.Length > 0)
    {
        ProcessCustomConfiguration(initInterGridComms);
    }

    if (initInterGridComms)
    {
        if (!ProcessCommunicationMessage(ref arguments))
        {
            return;
        }
    }

    List<IMyTerminalBlock> blocks = GetBlocksWithName<IMyProgrammableBlock>(strComputerTag);

    IMyProgrammableBlock missileComputer = null;

    switch (launchSelectionType)
    {
        case 1:
            missileComputer = ReturnClosest(blocks) as IMyProgrammableBlock;
            break;
        case 2:
            missileComputer = ReturnFurthest(blocks) as IMyProgrammableBlock;
            break;
        default:
            missileComputer = ReturnAny(blocks) as IMyProgrammableBlock;
            break;
    }

    if (missileComputer != null)
    {
        if (strAdditionalCustomData.Length > 0)
        {
            missileComputer.CustomData += "\n" + strAdditionalCustomData.Replace("\\n","\n");
        }

        List<TerminalActionParameter> parameters = new List<TerminalActionParameter>();
        parameters.Add(TerminalActionParameter.Get(arguments));
        missileComputer.ApplyAction("Run", parameters);

        if (strDetachConnectorTag != null && strDetachConnectorTag.Length > 0)
        {
            blocks = GetBlocksWithName<IMyShipConnector>(strDetachConnectorTag);
            IMyTerminalBlock closestBlock = GetClosestBlockFromReference(blocks, missileComputer);

            if (closestBlock != null)
            {
                closestBlock.ApplyAction("Unlock");

                IMyShipConnector otherConnector = ((IMyShipConnector)closestBlock).OtherConnector;
                if (otherConnector != null)
                {
                    otherConnector.ApplyAction("Unlock");
                }
            }
        }
    }
}

bool ProcessCommunicationMessage(ref string message)
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
                    message = msg.Substring(start);

                    return true;
                }
            }
        }
    }

    return false;
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
            return listenerId.Equals(recipient, StringComparison.OrdinalIgnoreCase) ||
                    (listenerGroup != null && listenerGroup.Equals(recipient, StringComparison.OrdinalIgnoreCase));
        case 1:
            return listenerId.EndsWith(recipient.Substring(1), StringComparison.OrdinalIgnoreCase) ||
                    (listenerGroup != null && listenerGroup.EndsWith(recipient.Substring(1), StringComparison.OrdinalIgnoreCase));
        case 2:
            return listenerId.StartsWith(recipient.Substring(0, recipient.Length - 1), StringComparison.OrdinalIgnoreCase) ||
                    (listenerGroup != null && listenerGroup.StartsWith(recipient.Substring(0, recipient.Length - 1), StringComparison.OrdinalIgnoreCase));
        default:
            return (recipient.Length == 1) || (listenerId.IndexOf(recipient.Substring(1, recipient.Length - 2), StringComparison.OrdinalIgnoreCase) > -1) ||
                    (listenerGroup != null && (listenerGroup.IndexOf(recipient.Substring(1, recipient.Length - 2), StringComparison.OrdinalIgnoreCase) > -1));
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

void ProcessCustomConfiguration(bool initInterGridComms = false)
{
    CustomConfiguration cfg = new CustomConfiguration(Me);
    cfg.Load();

    cfg.Get("strComputerTag", ref strComputerTag);
    cfg.Get("strDetachConnectorTag", ref strDetachConnectorTag);
    cfg.Get("strAdditionalCustomData", ref strAdditionalCustomData);
    cfg.Get("launchSelectionType", ref launchSelectionType);

    if (initInterGridComms)
    {
        cfg.Get("listenerId", ref listenerId);
        cfg.Get("listenerGroup", ref listenerGroup);
        cfg.Get("allowedSenderId", ref allowedSenderId);
    }
}

IMyTerminalBlock ReturnAny(List<IMyTerminalBlock> blocks)
{
    if (blocks.Count > 0)
    {
        Random rnd = new Random();
        return blocks[rnd.Next(0, blocks.Count)];
    }
    return null;
}

IMyTerminalBlock ReturnClosest(List<IMyTerminalBlock> blocks)
{
    double currDist = 0;
    double closestDist = Double.MaxValue;
    IMyTerminalBlock closestBlock = null;

    for (int i = 0; i < blocks.Count; i++)
    {
        currDist = (blocks[i].GetPosition() - Me.GetPosition()).Length();
        if (currDist < closestDist)
        {
            closestDist = currDist;
            closestBlock = blocks[i];
        }
    }

    return closestBlock;
}

IMyTerminalBlock ReturnFurthest(List<IMyTerminalBlock> blocks)
{
    double currDist = 0;
    double furthestDist = 0;
    IMyTerminalBlock furthestBlock = null;

    for (int i = 0; i < blocks.Count; i++)
    {
        currDist = (blocks[i].GetPosition() - Me.GetPosition()).Length();
        if (currDist > furthestDist)
        {
            furthestDist = currDist;
            furthestBlock = blocks[i];
        }
    }

    return furthestBlock;
}

IMyTerminalBlock GetClosestBlockFromReference(List<IMyTerminalBlock> checkBlocks, IMyTerminalBlock referenceBlock)
{
    IMyTerminalBlock checkBlock = null;
    double prevCheckDistance = Double.MaxValue;

    for (int i = 0; i < checkBlocks.Count; i++)
    {
        double currCheckDistance = (checkBlocks[i].GetPosition() - referenceBlock.GetPosition()).Length();
        if (currCheckDistance < prevCheckDistance)
        {
            prevCheckDistance = currCheckDistance;
            checkBlock = checkBlocks[i];
        }
    }

    return checkBlock;
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