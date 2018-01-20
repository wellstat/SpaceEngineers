//============================================================
// Used on most of my new Raycast Lock-on Homing Missile
// Launch Platforms. This script manipulates the Projector
// by switching between saved projector offset values to
// simulate an animation.
//============================================================

//------------------------------------------------------------
// ADN - Projector Alert Script v1.1
//------------------------------------------------------------
string BLINKING_ALERT_PROJECTOR_TAG = "Missile Alert Projector";
string SIGNAL_ALERT_PROJECTOR_TAG = "Sogeki Alert Projector";

int[][] blinkingProjectorSettings = new int[][] { new int[] {0,1,50,2,0,0}, new int[] {0,1,50,-2,2,0} };
int[][] signalProjectorSettings = new int[][] { new int[] {0,1,50,2,0,2}, new int[] {0,1,50,-2,1,2}, new int[] {0,1,50,2,-1,-2}, new int[] {0,1,50,0,0,0} };

int blinkIntervalTicks = 15;
int blinkDurationTicks = 120;

IMyProjector blinkingProjector = null;
IMyProjector signalProjector = null;

int blinkSwitchTicks = 0;
int blinkStopTicks = 0;
bool blinkOn = false;

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
        blinkingProjector = GetBlockOfTypeWithTag<IMyProjector>(BLINKING_ALERT_PROJECTOR_TAG);
        signalProjector = GetBlockOfTypeWithTag<IMyProjector>(SIGNAL_ALERT_PROJECTOR_TAG);

        clock = 0;

        init = true;
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
                int value;

                switch (configKey)
                {
                    case "SIGNAL":
                        if (tokens.Length > 1)
                        {
                            if (int.TryParse(tokens[1], out value))
                            {
                                if (value >= 0 && value < signalProjectorSettings.Length)
                                {
                                    signalProjector.ProjectionOffset = new Vector3I(signalProjectorSettings[value][0], signalProjectorSettings[value][1], signalProjectorSettings[value][2]);
                                    signalProjector.ProjectionRotation = new Vector3I(signalProjectorSettings[value][3], signalProjectorSettings[value][4], signalProjectorSettings[value][5]);
                                    signalProjector.UpdateOffsetAndRotation();
                                }
                            }
                        }
                        break;
                    case "BLINK":
                        if (blinkStopTicks <= clock)
                        {
                            blinkSwitchTicks = clock;
                            blinkOn = false;
                        }
                        blinkStopTicks = clock + blinkDurationTicks;
                        break;
                    default:
                        break;
                }
            }
        }
    }

    if ((updateSource & UpdateType.Update1) == 0)
    {
        return;
    }

    clock++;

    if (blinkStopTicks > clock)
    {
        if (blinkSwitchTicks <= clock)
        {
            blinkOn = !blinkOn;
            int index = (blinkOn ? 1 : 0);
            blinkingProjector.ProjectionOffset = new Vector3I(blinkingProjectorSettings[index][0], blinkingProjectorSettings[index][1], blinkingProjectorSettings[index][2]);
            blinkingProjector.ProjectionRotation = new Vector3I(blinkingProjectorSettings[index][3], blinkingProjectorSettings[index][4], blinkingProjectorSettings[index][5]);
            blinkingProjector.UpdateOffsetAndRotation();
            blinkSwitchTicks = clock + blinkIntervalTicks;
        }
    }
    else if (blinkOn)
    {
        blinkingProjector.ProjectionOffset = new Vector3I(blinkingProjectorSettings[0][0], blinkingProjectorSettings[0][1], blinkingProjectorSettings[0][2]);
        blinkingProjector.ProjectionRotation = new Vector3I(blinkingProjectorSettings[0][3], blinkingProjectorSettings[0][4], blinkingProjectorSettings[0][5]);
        blinkingProjector.UpdateOffsetAndRotation();
        blinkOn = false;
    }
}

T GetBlockOfTypeWithTag<T>(string name) where T: class, IMyTerminalBlock
{
    List<IMyTerminalBlock> blocks = new List<IMyTerminalBlock>();
    GridTerminalSystem.SearchBlocksOfName(name, blocks);
    for (int i = 0; i < blocks.Count; i++)
    {
        T block = blocks[i] as T;
        if (block != null)
        {
            return block;
        }
    }
    return null;
}