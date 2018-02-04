//============================================================
// Raw version of the published script on my workshop.
//============================================================

//------------------------------------------------------------
// ADN - Floor Plan Script v8.3
//------------------------------------------------------------

string strDisplayPanelTag = "L_PANEL";
string strReportPanelTag = "L_REPORT";
string strStatusPanelTag = "L_STATUS";
string strMenuPanelTag = "L_MENU";
string strAlarmTag = "L_ALARM";

bool bUseAlarmSystem = true;

static bool bOptimizedRender = true;

bool bExcludeHiddenBlocks = false;

string strExcludeAlertKeyword = "[NOALERT]";

string strAlertBlockTypes = "AirVent,SensorBlock,Door,AirtightHangarDoor,AirtightSlideDoor,LargeGatlingTurret,LargeMissileTurret,InteriorTurret";

//---------- Configurable Constants ----------

int HORIZONTAL_COUNT = 1;
int VERTICAL_COUNT = 1;

int REFRESH_INTERVAL = 60;

float INSTRUCTION_PERCENT_LIMIT = 0.2f;

const int DEF_PIXEL_SIZE = 1;

const bool DEF_PAINT_DENSITY = true;
const bool DEF_PAINT_TERMINAL_BLOCKS = true;
const bool DEF_PAINT_ALERT_BLOCKS = true;

const bool DEF_BLINK_FOR_ALERT = false;
const bool DEF_USE_RELATIVE_DENSITY = false;
const bool DEF_INCLUDE_TERMINAL_DENSITY = true;
const bool DEF_SMOOTH_EDGES = false;

const bool DEF_PAINT_CROSS_FOR_ALERT = false;

const bool DEF_PAINT_LOCATION_MARKER = false;

const byte DEF_ALERT_MASK = AL_VENT | AL_SENSOR | AL_TURRET | AL_DOOR | AL_HACKED;

const float HIGH_DENSITY_THRESHOLD = 1.0f;
const float MID_DENSITY_THRESHOLD = 0.5f;
const float LOW_DENSITY_THRESHOLD = 0.25f;

const int TURRET_TRIP_TICKS = 120;

const int TEXT_MAX_RESOLUTION_X = 178;
const int TEXT_MAX_RESOLUTION_Y = 178;

const int WIDE_LCD_MAX_RESOLUTION_X = 356;
const int WIDE_LCD_MAX_RESOLUTION_Y = 178;

const int BOX_AREA_DRAW_DENSITY_LIMIT = 4900;

//---------- Color Configuration ----------

static char chrBackground = Rgb(0, 0, 0);
static char chrStructure = Rgb(4, 4, 4);
static char chrDensity = Rgb(3, 3, 3);
static char chrLowDensity = Rgb(2, 2, 2);

static char chrDetectedEntity = Rgb(7, 0, 0);
static char chrHacked = Rgb(7, 0, 0);

static char chrMissingBlock = Rgb(7, 0, 0);
static char chrAlertAirVents = Rgb(5, 5, 0);
static char chrAlertTurrets = Rgb(7, 3, 0);
static char chrAlertDoors = Rgb(1, 1, 7);

static char chrTerminalBlockFunctional = Rgb(1, 6, 1);
static char chrTerminalBlockDamaged = Rgb(5, 5, 0);
static char chrTerminalBlockMissing = Rgb(7, 0, 0);

static char chrLocationMarkerEven = Rgb(1, 1, 7);
static char chrLocationMarkerOdd = Rgb(1, 6, 1);

const int TEXT_COLOR_R = 255;
const int TEXT_COLOR_G = 255;
const int TEXT_COLOR_B = 255;

//---------- Color Code Generator ----------

//Credits to Krypt for this color generator
//http://steamcommunity.com/sharedfiles/filedetails/?id=787881521
static char Rgb(byte r, byte g, byte b)
{
    return (char)(0xe100 + (r << 6) + (g << 3) + b);
}

//---------- Script Compound Parameters ----------

const byte SUB_X = 32, ADD_X = 16, SUB_Y = 8, ADD_Y = 4, SUB_Z = 2, ADD_Z = 1;

const byte AL_VENT = 1, AL_SENSOR = 2, AL_TURRET = 4, AL_DOOR = 8, AL_HACKED = 16;

Vector3I[] ADJACENT_VECTORS = { new Vector3I(-1, 0, 0), new Vector3I(1, 0, 0), new Vector3I(0, -1, 0), new Vector3I(0, 1, 0), new Vector3I(0, 0, -1), new Vector3I(0, 0, 1) };
static byte[] ADJACENT_MASKS = new byte[6];

HashSet<string> WATCH_BLK_TYPE = null;

static Dictionary<string, int[]> FLAG_CMD = null;
HashSet<string> GRID_CMD = null;

Dictionary<string, int> MENU_CMD = null;

int progressIconIndex = 0;
char[] progressIcons = new char[8];

char[] cleanChar = " _-:;&".ToCharArray();
string[] triggerActions = new string[] { "TriggerNow", "PlaySound", "StartCountdown", "ShootOnce", "Close", "OnOff_On" };

const long fontMonospace = 1147350002L;

int instLimit = 45000;

//---------- Below Is Main Script Body ----------

List<LayoutPanel> lyts = new List<LayoutPanel>();

List<IMyTextPanel> rptPnls = new List<IMyTextPanel>();
List<IMyTextPanel> statPnls = new List<IMyTextPanel>();
List<IMyTextPanel> menuPnls = new List<IMyTextPanel>();

int pnStart = 0;
int pnEnd = 0;
int pnIndex = 0;

Dictionary<Vector3I, byte> structBlocks = null;
Stack<Vector3I> schRemain = null;
HashSet<Vector3I> schTested = null;

List<KeyValuePair<Vector3I, byte>> structDataList = null;

List<IMyTerminalBlock> termBlocks = null;
List<IMyTerminalBlock> tempBlocks = null;

List<WatchField> watchBlocks = null;

List<IMyTerminalBlock>[] priorityBlocks = null;
int priorityCount = 0;

bool needSensorDetected = false;
List<Vector3D> sensorDetected = new List<Vector3D>();

List<IMyTerminalBlock> alarmBlocks = new List<IMyTerminalBlock>();
Dictionary<string, int[]> groupStats = new Dictionary<string, int[]>();

Vector3I gridMin = new Vector3I();
Vector3I gridMax = new Vector3I();

Vector3I totalMin = new Vector3I();
Vector3I totalMax = new Vector3I();

int dmgTermCnt = 0;
int msTermCnt = 0;
int msStructCnt = 0;
int ventALCnt = 0;
int sensALCnt = 0;
int hackCnt = 0;
int shootTurCnt = 0;
int curMsStructCnt = 0;
int curVentALCnt = 0;
int curSensALCnt = 0;

int lastRenderTicks = 0;
int currentRenderTicks = 0;
string errorMessage = "";
int prevSubMode = -2;

int curMnId = 0;
int actMnCnt = 0;

Menu selMnu;
Menu rtMnu;
bool refreshMenu;
int maxMnCntShown = 7;

int subCounter = 0;
int subMode = -1;
int mode = 0;
int clock = 0;
bool init = false;

Program()
{
    Runtime.UpdateFrequency = UpdateFrequency.Update1;
}

void Main(string args, UpdateType updateSource)
{
    clock++;

    bool haveArgs;
    if (args.Length > 0)
    {
        args = args.Trim().ToUpper();
        haveArgs = true;
    }
    else
    {
        haveArgs = false;
    }

    if (haveArgs && menuPnls.Count > 0)
    {
        if (args.Length > 0)
        {
            haveArgs = ExecuteMenuFlow(ref args);
        }
    }

    if (!init)
    {
        DisplayStatus();

        if (subMode == -1)
        {
            StaticInit();

            lastRenderTicks = clock;

            subMode = 0;
            return;
        }

        if (subMode == 0)
        {
            if (Me.CustomData.Length > 0)
            {
                ProcessCustomConfiguration();
            }

            instLimit = (int)(INSTRUCTION_PERCENT_LIMIT * Runtime.MaxInstructionCount);

            InitDisplayPanels();

            InitMiscPanels(rptPnls, strReportPanelTag);
            InitMiscPanels(statPnls, strStatusPanelTag);
            InitMiscPanels(menuPnls, strMenuPanelTag);

            errorMessage = "";

            if (lyts.Count == 0)
            {
                errorMessage += "[NO DISPLAY PANELS DETECTED]\nEnsure Panels With " + strDisplayPanelTag + " Tag Exists\n";
            }

            if (errorMessage.Length > 0)
            {
                DisplayStatus();
            }

            if (menuPnls.Count > 0)
            {
                selMnu = rtMnu;

                curMnId = 0;
                actMnCnt = 0;

                ComputeActualMenuCount();
                DisplayMenu();
            }

            structBlocks = new Dictionary<Vector3I, byte>();
            schRemain = new Stack<Vector3I>();
            schTested = new HashSet<Vector3I>();

            structBlocks.Add(Me.Position, 0);
            schRemain.Push(Me.Position);
            schTested.Add(Me.Position);

            subMode = 1;
            return;
        }
        else if (subMode == 1)
        {
            if (InitStructureData(Me.CubeGrid, schRemain, structBlocks, schTested, instLimit))
            {
                schRemain = null;
                schTested = null;

                tempBlocks = null;

                gridMin = Me.CubeGrid.Min;
                gridMax = Me.CubeGrid.Max;

                subMode = 2;
            }
            return;
        }
        else if (subMode == 2)
        {
            if (tempBlocks == null)
            {
                tempBlocks = new List<IMyTerminalBlock>();
                GridTerminalSystem.GetBlocks(tempBlocks);

                termBlocks = new List<IMyTerminalBlock>();
            }

            subCounter = FilterTerminalBlocks(Me.CubeGrid, tempBlocks, termBlocks, subCounter, instLimit);
            if (subCounter >= tempBlocks.Count)
            {
                tempBlocks = null;

                subCounter = 0;
                subMode = 3;
            }
            return;
        }
        else if (subMode == 3)
        {
            pnStart = 0;
            pnEnd = lyts.Count - 1;

            for (int i = pnStart; i <= pnEnd; i++)
            {
                lyts[i].InitDisplayParameters();
                lyts[i].InitScreenParameters();
                lyts[i].ResetStructureBase();
            }

            CompileTotalGridSize();

            structDataList = new List<KeyValuePair<Vector3I, byte>>(structBlocks);

            subCounter = 0;
            subMode = 4;

            watchBlocks = new List<WatchField>();

            return;
        }
        else if (subMode == 4)
        {
            subCounter = FilterWatchedBlocks(Me.CubeGrid, termBlocks, watchBlocks, subCounter, instLimit);
            if (subCounter >= termBlocks.Count)
            {
                bool bPaintDensity = false;
                for (int i = pnStart; i <= pnEnd; i++)
                {
                    if (lyts[i].bPaintDensity)
                    {
                        bPaintDensity = true;
                        break;
                    }
                }

                subCounter = 0;
                subMode = (bPaintDensity ? 5 : 7);
            }
            return;
        }
        else if (subMode == 5)
        {
            subCounter = ComputeStructureDensity(structDataList, subCounter, instLimit);
            if (subCounter >= structDataList.Count)
            {
                pnIndex = pnStart;
                subCounter = 0;
                subMode = 6;
            }
            return;
        }
        else if (subMode == 6)
        {
            bool completeDraw;
            if (lyts[pnIndex].bPaintDensity && lyts[pnIndex].scArea <= BOX_AREA_DRAW_DENSITY_LIMIT)
            {
                subCounter = lyts[pnIndex].PaintStructureDensity(subCounter, instLimit);
                completeDraw = (subCounter >= lyts[pnIndex].axWhY * lyts[pnIndex].axWhX);
            }
            else
            {
                completeDraw = true;
            }

            if (completeDraw)
            {
                if (pnIndex >= pnEnd)
                {
                    subCounter = 0;
                    subMode = 7;
                }
                else
                {
                    pnIndex++;
                    subCounter = 0;
                }
            }
            return;
        }
        else if (subMode == 7)
        {
            subCounter = PaintStructureBase(structDataList, subCounter, instLimit);
            if (subCounter >= structDataList.Count)
            {
                subMode = 8;
            }
            return;
        }
        else if (subMode == 8)
        {
            for (int i = 0; i < lyts.Count; i++)
            {
                if (lyts[i].bPaintLocationMarker && lyts[i].scArea <= BOX_AREA_DRAW_DENSITY_LIMIT)
                {
                    lyts[i].PaintLocationMarker(chrLocationMarkerEven, chrLocationMarkerOdd);
                }
            }

            subMode = 9;
            return;
        }

        if (bUseAlarmSystem)
        {
            alarmBlocks.Clear();
            GridTerminalSystem.SearchBlocksOfName(strAlarmTag, alarmBlocks);
        }

        currentRenderTicks = clock;

        subCounter = 0;
        subMode = -1;
        mode = 0;

        init = true;
        return;
    }

    if (haveArgs)
    {
        bool exitFlag = ProcessRunCommands(args);

        if (refreshMenu)
        {
            refreshMenu = false;
            DisplayMenu();
        }

        if (exitFlag)
        {
            return;
        }
    }

    if (mode == 0)
    {
        if (clock % REFRESH_INTERVAL == 0)
        {
            pnIndex = 0;
            subMode = 0;
            mode = 1;
        }
    }
    else if (mode == 1)
    {
        if (subMode == 0)
        {
            pnIndex = ResetRenderData(pnIndex, lyts.Count - 1, instLimit);
            if (pnIndex >= lyts.Count)
            {
                for (int i = 0; i < lyts.Count; i++)
                {
                    if (lyts[i].bPaintTerminalBlocks)
                    {
                        lyts[i].priorityDrawn = new Dictionary<Vector4I, int>();
                    }

                    lyts[i].drawnPositions = new HashSet<Vector2I>();
                }

                for (int i = 0; i < priorityBlocks.Length; i++)
                {
                    priorityBlocks[i].Clear();
                }

                shootTurCnt = 0;

                subCounter = 0;
                subMode = 1;
            }
        }
        else if (subMode == 1)
        {
            subCounter = PaintGoodTerminalBlocks(termBlocks, priorityBlocks, subCounter, instLimit);
            if (subCounter >= termBlocks.Count)
            {
                dmgTermCnt = priorityBlocks[0].Count;
                msTermCnt = priorityBlocks[1].Count;
                hackCnt = priorityBlocks[2].Count;

                priorityCount = dmgTermCnt + msTermCnt + hackCnt;

                subCounter = 0;
                subMode = 2;
            }
        }
        else if (subMode == 2)
        {
            subCounter = PaintBadTerminalBlocks(priorityBlocks, subCounter, instLimit);
            if (subCounter >= priorityCount)
            {
                for (int i = 0; i < lyts.Count; i++)
                {
                    lyts[i].priorityDrawn = null;
                }

                msStructCnt = 0;

                subCounter = 0;
                subMode = 3;
            }
        }
        else if (subMode == 3)
        {
            subCounter = PaintMissingStructure(structDataList, subCounter, instLimit);
            if (subCounter >= structDataList.Count)
            {
                for (int i = 0; i < lyts.Count; i++)
                {
                    if (lyts[i].bPaintAlertBlocks)
                    {
                        lyts[i].isBlinkFrame = (lyts[i].bBlinkForAlert ? !lyts[i].isBlinkFrame : true);
                    }

                    lyts[i].drawnPositions = null;
                }

                sensorDetected.Clear();

                curMsStructCnt = msStructCnt;

                ventALCnt = 0;
                sensALCnt = 0;

                subCounter = 0;
                subMode = 4;
            }
        }
        else if (subMode == 4)
        {
            subCounter = PaintWatchedBlocks(watchBlocks, subCounter, instLimit);
            if (subCounter >= watchBlocks.Count)
            {
                curVentALCnt = ventALCnt;
                curSensALCnt = sensALCnt;

                DisplayStatus();
                pnIndex = 0;
                subCounter = 0;
                subMode = 5;
            }
        }
        else if (subMode == 5)
        {
            pnIndex = Render(pnIndex, instLimit);
            if (pnIndex >= lyts.Count)
            {
                subCounter = 0;
                subMode = 6;
            }
        }
        else
        {
            if (bUseAlarmSystem)
            {
                ProcessAlarms();
            }

            DisplayReport();

            lastRenderTicks = currentRenderTicks;
            currentRenderTicks = clock;
            subMode = -1;
            mode = 0;
        }
    }
    else if (mode == 3)
    {
        if (subMode == 0)
        {
            for (int i = 0; i < lyts.Count; i++)
            {
                lyts[i].ShowLargeNumber(i + 1);
            }
        }
    }
    else if (mode == 4)
    {
        for (int i = 0; i < lyts.Count; i++)
        {
            lyts[i].HideLargeNumber();
        }
        mode = 0;
    }

    DisplayStatus();
}

void DisplayStatus()
{
    if (clock % 15 == 0 || prevSubMode != subMode)
    {
        prevSubMode = subMode;

        StringBuilder sb = new StringBuilder();

        if (clock % 15 == 0)
        {
            progressIconIndex = (progressIconIndex + 1) % 8;
        }

        if (init)
        {
            sb.Append("Status: Operational  ").Append(progressIcons[progressIconIndex]).Append("\n");
            sb.Append("Refresh Rate: ").Append(Math.Round((currentRenderTicks - lastRenderTicks) * 16.66667f)).Append("ms\n");
            sb.Append("Total Display Panels: ").Append(lyts.Count).Append("\n\n");
            sb.Append("Total Armor Blocks: ").Append(structBlocks.Count).Append('\n');
            sb.Append("Total Terminal Blocks: ").Append(termBlocks.Count).Append('\n');
            sb.Append("Total Alert Blocks: ").Append(watchBlocks.Count).Append("\n\n");
            sb.Append("Damaged Terminal Blocks: ").Append(dmgTermCnt).Append('\n');
            sb.Append("Missing Terminal Blocks: ").Append(msTermCnt).Append('\n');
            sb.Append("Missing Armor Blocks: ").Append(curMsStructCnt).Append('\n');
            sb.Append("Hull Integrity: ").Append(Math.Round((structBlocks.Count - curMsStructCnt) * 100f / structBlocks.Count, 0) + "%").Append('\n');
            sb.Append("Terminal Integrity: ").Append(Math.Round((termBlocks.Count - dmgTermCnt - msTermCnt) * 100f / termBlocks.Count, 0) + "%").Append('\n');
            sb.Append("Air Vent Alerts: ").Append(curVentALCnt).Append('\n');
            sb.Append("Sensor Alerts: ").Append(curSensALCnt).Append('\n');
            sb.Append('\n').Append(errorMessage).Append('\n');
        }
        else
        {
            sb.Append("Status: Initialization  ").Append(progressIcons[progressIconIndex]).Append("\nStage: ");
            switch (subMode)
            {
                case -1:
                    sb.Append("Initializing Static Values\n");
                    break;
                case 0:
                    sb.Append("Searching For Display Panels\n");
                    break;
                case 1:
                    sb.Append("Searching For Armor Blocks\n");
                    break;
                case 2:
                    sb.Append("Searching For Terminal Blocks\n");
                    break;
                case 3:
                    sb.Append("Initializing Display Panels\n");
                    break;
                case 4:
                    sb.Append("Filtering Alert Blocks\n");
                    break;
                case 5:
                    sb.Append("Computing Structure Density\n");
                    break;
                case 6:
                    sb.Append("Painting Structure Density\n");
                    break;
                case 7:
                    sb.Append("Painting Main Structure\n");
                    break;
                case 8:
                    sb.Append("Painting Location Markers\n");
                    break;
                case 9:
                    sb.Append("Initialization Completed\n");
                    break;
            }
            if (subMode > 0)
            {
                sb.Append("Total Display Panels: ").Append(lyts.Count).Append("\n\n");
            }
            if (subMode > 1)
            {
                sb.Append("Total Armor Blocks: ").Append(structBlocks.Count).Append('\n');
            }
            if (subMode > 2)
            {
                sb.Append("Total Terminal Blocks: ").Append(termBlocks.Count).Append('\n');
            }
            if (subMode > 4)
            {
                sb.Append("Total Alert Blocks: ").Append(watchBlocks.Count).Append('\n');
            }
            sb.Append('\n').Append(errorMessage).Append('\n');
        }

        for (int i = 0; i < statPnls.Count; i++)
        {
            statPnls[i].WritePublicText(sb);
        }
        Echo(sb.ToString());
    }
}

void ProcessAlarms()
{
    for (int i = 0; i < alarmBlocks.Count; i++)
    {
        IMyTerminalBlock blk = alarmBlocks[i];
        if (blk.IsFunctional)
        {
            string param = blk.CustomName;
            int startIndex = param.IndexOf(strAlarmTag, StringComparison.OrdinalIgnoreCase);
            if (startIndex > -1)
            {
                param = param.Substring(startIndex + strAlarmTag.Length).Trim();
            }

            bool andCond = (param.Length > 0 && param[0] == '&');

            int count = 0;
            int value = 0;
            string action = null;
            string[] tokens = param.Split(cleanChar, StringSplitOptions.RemoveEmptyEntries);
            switch (tokens.Length)
            {
                case 0:
                    count = 0;
                    break;
                case 1:
                    count = 1;
                    break;
                case 2:
                    count = 1;
                    Int32.TryParse(tokens[1], out value);
                    action = (value == 0 ? tokens[1] : null);
                    break;
                default:
                    count = (tokens.Length / 2);
                    action = (tokens.Length % 2 == 0 ? null : tokens[tokens.Length - 1]);
                    break;
            }

            bool trigger = andCond;

            for (int s = 0; s < count; s++)
            {
                int index = s * 2;
                string cmd = tokens[index].ToUpper();

                index++;
                if (index >= tokens.Length || !Int32.TryParse(tokens[index], out value))
                {
                    value = 0;
                }

                switch (cmd)
                {
                    case "HULL":
                        trigger = (((structBlocks.Count - curMsStructCnt) * 100f / structBlocks.Count) <= (value <= 0 ? 50 : value));
                        break;
                    case "TERM":
                        trigger = (((termBlocks.Count - dmgTermCnt - msTermCnt) * 100f / termBlocks.Count) <= (value <= 0 ? 50 : value));
                        break;
                    case "VENT":
                        trigger = (curVentALCnt >= (value <= 0 ? 1 : value));
                        break;
                    case "SENSOR":
                        trigger = (curSensALCnt >= (value <= 0 ? 1 : value));
                        break;
                    case "HACK":
                        trigger = (hackCnt >= (value <= 0 ? 1 : value));
                        break;
                    case "TURRET":
                        trigger = (shootTurCnt >= (value <= 0 ? 1 : value));
                        break;
                }

                if (andCond != trigger)
                {
                    break;
                }
            }

            if (trigger)
            {
                if (action != null && blk.HasAction(action = action.Replace('+', '_')))
                {
                    blk.ApplyAction(action);
                }
                else
                {
                    for (int x = 0; x < triggerActions.Length; x++)
                    {
                        if (blk.HasAction(triggerActions[x]))
                        {
                            blk.ApplyAction(triggerActions[x]);
                            break;
                        }
                    }
                }
            }
        }
    }
}

void DisplayReport()
{
    needSensorDetected = false;

    for (int i = 0; i < rptPnls.Count; i++)
    {
        IMyTextPanel panel = rptPnls[i];

        string param = panel.GetPublicTitle().Trim();
        if (param.Length == 0 || param.ToUpper().Equals("PUBLIC TITLE"))
        {
            param = "HULL,,TERM,,VENT,,SENSOR,,HACK,,TURRET";
            panel.WritePublicTitle(param);
        }

        float value;
        float fontSize = panel.GetValueFloat("FontSize");

        StringBuilder sb = new StringBuilder();

        string[] tokens = param.Split(',');
        for (int x = 0; x < tokens.Length; x++)
        {
            string cmd = tokens[x].Trim();

            int[] stats;
            string grp;
            string label;
            int dash = cmd.IndexOf("/");
            if (dash > -1)
            {
                grp = cmd.Substring(dash + 1).Trim();
                cmd = cmd.Substring(0, dash).Trim().ToUpper();

                dash = grp.IndexOf("/");
                if (dash > -1)
                {
                    label = grp.Substring(dash + 1).Trim();
                    grp = grp.Substring(0, dash).Trim();
                }
                else
                {
                    label = grp;
                }
            }
            else
            {
                cmd = cmd.ToUpper();
                label = grp = "";
            }

            switch (cmd)
            {
                case "HULL":
                    value = (structBlocks.Count - curMsStructCnt) * 100f / structBlocks.Count;
                    sb.Append("Hull Integrity: ").Append(curMsStructCnt > -1 ? Math.Round(value, 0) + "%" : "-").Append('\n');
                    DrawProgressBar(sb, fontSize, value).Append('\n');
                    break;
                case "TERM":
                    value = (termBlocks.Count - (dmgTermCnt * 0.5f) - msTermCnt) * 100f / termBlocks.Count;
                    sb.Append("Terminal Integrity: ").Append(Math.Round(value, 0) + "%").Append('\n');
                    DrawProgressBar(sb, fontSize, value).Append('\n');
                    break;
                case "VENT":
                    sb.Append("Leaked Vents: ").Append(curVentALCnt).Append('\n');
                    break;
                case "SENSOR":
                    sb.Append("Sensor Detection: ").Append(curSensALCnt).Append('\n');
                    break;
                case "HACK":
                    sb.Append("Blocks Hacked: ").Append(hackCnt).Append('\n');
                    break;
                case "TURRET":
                    sb.Append("Turrets Shooting: ").Append(shootTurCnt).Append('\n');
                    break;
                case "SENSORGPS":
                    for (int s = 0; s < sensorDetected.Count; s++)
                    {
                        sb.Append("GPS:S").Append(s + 1).Append(":").Append(VectorToString(sensorDetected[s], 2)).Append(":\n");
                    }
                    needSensorDetected = true;
                    break;
                case "FUNC":
                    stats = RetrieveGroupStatistics(groupStats, grp);
                    sb.Append(label).Append(": ").Append(stats[1]).Append(" / ").Append(stats[0]).Append('\n');
                    break;
                case "FUNCP":
                    stats = RetrieveGroupStatistics(groupStats, grp);
                    value = (stats[0] == 0 ? 0f : 100f * stats[1] / stats[0]);
                    sb.Append(label).Append(": ").Append(stats[1]).Append(" / ").Append(stats[0]).Append('\n');
                    DrawProgressBar(sb, fontSize, value).Append('\n');
                    break;
                case "WORK":
                    stats = RetrieveGroupStatistics(groupStats, grp);
                    sb.Append(label).Append(": ").Append(stats[2]).Append(" / ").Append(stats[0]).Append('\n');
                    break;
                case "WORKP":
                    stats = RetrieveGroupStatistics(groupStats, grp);
                    value = (stats[0] == 0 ? 0f : 100f * stats[2] / stats[0]);
                    sb.Append(label).Append(": ").Append(stats[2]).Append(" / ").Append(stats[0]).Append('\n');
                    DrawProgressBar(sb, fontSize, value).Append('\n');
                    break;
                default:
                    sb.Append('\n');
                    break;
            }
        }

        panel.WritePublicText(sb);
    }
}

int[] RetrieveGroupStatistics(Dictionary<string, int[]> cacheStats, string grp)
{
    if (cacheStats.ContainsKey(grp))
    {
        return cacheStats[grp];
    }
    else
    {
        int[] stats = new int[3];

        List<IMyTerminalBlock> blks = new List<IMyTerminalBlock>();
        GridTerminalSystem.SearchBlocksOfName(grp, blks);

        for (int i = 0; i < blks.Count; i++)
        {
            if (blks[i].IsFunctional)
            {
                stats[1] += 1;
            }
            if (blks[i].IsWorking)
            {
                stats[2] += 1;
            }
        }
        stats[0] = blks.Count;

        cacheStats[grp] = stats;
        return stats;
    }
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

int Render(int index, int instLimit)
{
    while (index < lyts.Count && Runtime.CurrentInstructionCount < instLimit)
    {
        lyts[index].Render();

        index++;
    }

    return index;
}

int ResetRenderData(int index, int endIndex, int instLimit)
{
    while (index <= endIndex && Runtime.CurrentInstructionCount < instLimit)
    {
        lyts[index].ResetRenderData();

        index++;
    }

    return index;
}

void RedrawStructure()
{
    if (mode == 3 || mode == 4)
    {
        for (int i = 0; i < lyts.Count; i++)
        {
            lyts[i].HideLargeNumber();
        }
    }

    for (int i = pnStart; i <= pnEnd; i++)
    {
        lyts[i].ResetGridWidth();
        lyts[i].InitScreenParameters();
    }

    CompileTotalGridSize();

    structDataList = new List<KeyValuePair<Vector3I, byte>>(structBlocks);

    subCounter = 0;
    subMode = 4;

    watchBlocks = new List<WatchField>();

    mode = 0;

    init = false;
}

bool AdjustGridSize(ref int gridStart, ref int gridEnd, int gridMinimum, int gridMaximum, bool isStart, bool isPositive, bool isPan)
{
    int tmpGridStart = gridStart;
    int tmpGridEnd = gridEnd;

    if (isStart)
    {
        tmpGridStart += (isPositive ? 1 : -1);
        if (isPan)
        {
            tmpGridEnd += (isPositive ? 1 : -1);
        }
    }
    else
    {
        tmpGridEnd += (isPositive ? 1 : -1);
        if (isPan)
        {
            tmpGridStart += (isPositive ? 1 : -1);
        }
    }

    if (tmpGridStart > tmpGridEnd || tmpGridStart < gridMinimum || tmpGridEnd > gridMaximum)
    {
        return false;
    }

    gridStart = tmpGridStart;
    gridEnd = tmpGridEnd;

    return true;
}

/*----- Menu Processing Methods -----*/

bool ExecuteMenuFlow(ref string arguments)
{
    if (MENU_CMD.ContainsKey(arguments))
    {
        string menuAction = null;

        ComputeActualMenuCount();

        int operation = MENU_CMD[arguments];
        switch (operation)
        {
            case 1:
                curMnId--;
                break;
            case 2:
                curMnId++;
                break;
            case 3:
                curMnId -= maxMnCntShown;
                break;
            case 4:
                curMnId += maxMnCntShown;
                break;
            case 5:
                if (selMnu.Parent != null)
                {
                    selMnu = selMnu.Parent;
                    curMnId = 0;
                    ComputeActualMenuCount();
                }
                break;
            case 6:
                if (selMnu.ChildMenus != null && selMnu.ChildMenus.Count > 0)
                {
                    List<Menu> menuItems = selMnu.ChildMenus;

                    int currentMarker = -1;
                    for (int i = 0; i < menuItems.Count; i++)
                    {
                        int actualCount = (menuItems[i].RepeatList == null ? 1 : menuItems[i].RepeatList.Count);

                        if (currentMarker + actualCount >= curMnId)
                        {
                            if (actualCount > 1)
                            {
                                menuItems[i].CurrentSelection = curMnId - currentMarker - 1;
                            }
                            if (menuItems[i].Action != null)
                            {
                                menuAction = menuItems[i].GetAction();
                            }
                            if (menuItems[i].ChildMenus != null && menuItems[i].ChildMenus.Count > 0)
                            {
                                selMnu = menuItems[i];
                                curMnId = 0;
                                ComputeActualMenuCount();
                            }
                            break;
                        }
                        else
                        {
                            currentMarker += actualCount;
                        }
                    }
                }

                break;
        }

        if (curMnId < 0)
        {
            curMnId = 0;
        }
        else if (curMnId >= actMnCnt)
        {
            curMnId = actMnCnt - 1;
        }

        DisplayMenu();

        if (menuAction == null)
        {
            refreshMenu = false;
            return false;
        }
        else
        {
            arguments = menuAction.Trim().ToUpper();
            refreshMenu = true;
            return true;
        }
    }
    else
    {
        return true;
    }
}

void ComputeActualMenuCount()
{
    actMnCnt = 0;

    if (selMnu.ChildMenus != null && selMnu.ChildMenus.Count > 0)
    {
        List<Menu> menuItems = selMnu.ChildMenus;
        for (int i = 0; i < menuItems.Count; i++)
        {
            actMnCnt += (menuItems[i].RepeatList == null ? 1 : menuItems[i].RepeatList.Count);
        }
    }
}

void DisplayMenu()
{
    StringBuilder sb = new StringBuilder();

    sb.Append("------------------------------------------------------------\n");
    sb.Append("Floor Plan Script Menu\n");
    sb.Append("------------------------------------------------------------\n\n");

    string breadCrumb = selMnu.GetTitle();
    Menu currentMenu = selMnu.Parent;
    while (currentMenu != null)
    {
        breadCrumb = currentMenu.GetTitle() + " > " + breadCrumb;
        currentMenu = currentMenu.Parent;
    }
    sb.Append(breadCrumb).Append("\n\n");

    if (selMnu.ChildMenus != null && selMnu.ChildMenus.Count > 0)
    {
        List<Menu> menuItems = selMnu.ChildMenus;

        int menuStartIndex = curMnId - (maxMnCntShown / 2);
        int menuEndIndex = curMnId + (maxMnCntShown / 2);

        if (menuStartIndex < 0)
        {
            menuStartIndex = 0;
            menuEndIndex = Math.Min(maxMnCntShown - 1, actMnCnt);
        }
        else if (menuEndIndex >= actMnCnt)
        {
            menuEndIndex = actMnCnt - 1;
            menuStartIndex = Math.Max(menuEndIndex - maxMnCntShown + 1, 0);
        }

        int currentMarker = -1;
        for (int i = 0; i < menuItems.Count; i++)
        {
            int actualCount = (menuItems[i].RepeatList == null ? 1 : menuItems[i].RepeatList.Count);

            if (currentMarker + actualCount >= menuStartIndex)
            {
                if (actualCount == 1)
                {
                    bool isCurrent = (curMnId == (currentMarker + 1));
                    sb.Append(isCurrent ? "> [> " : "> ").Append(menuItems[i].GetName()).Append(isCurrent ? " <]\n" : "\n");

                    currentMarker++;
                }
                else
                {
                    int markerStart = menuStartIndex - currentMarker - 1;
                    if (markerStart < 0)
                    {
                        markerStart = 0;
                    }
                    int markerEnd = markerStart + (menuEndIndex - menuStartIndex);
                    if (markerEnd >= actualCount)
                    {
                        markerEnd = actualCount - 1;
                    }
                    int tempSelection = menuItems[i].CurrentSelection;
                    for (int j = markerStart; j <= markerEnd; j++)
                    {
                        bool isCurrent = (curMnId == (currentMarker + j + 1));
                        menuItems[i].CurrentSelection = j;
                        sb.Append(isCurrent ? "> [> " : "> ").Append(menuItems[i].GetName()).Append(isCurrent ? " <]\n" : "\n");
                    }
                    menuItems[i].CurrentSelection = tempSelection;

                    currentMarker += markerEnd - markerStart + 1;
                }
            }
            else
            {
                currentMarker += actualCount;
            }

            if (currentMarker >= menuEndIndex)
            {
                break;
            }
        }
    }

    for (int i = 0; i < menuPnls.Count; i++)
    {
        menuPnls[i].WritePublicText(sb);
    }
}

/*----- Parameter Processing Methods -----*/

void ProcessCustomConfiguration()
{
    CustomConfiguration cfg = new CustomConfiguration(Me);
    cfg.Load();

    cfg.Get("strDisplayPanelTag", ref strDisplayPanelTag);
    cfg.Get("strReportPanelTag", ref strReportPanelTag);
    cfg.Get("strStatusPanelTag", ref strStatusPanelTag);
    cfg.Get("strMenuPanelTag", ref strMenuPanelTag);
    cfg.Get("strAlarmTag", ref strAlarmTag);
    cfg.Get("bUseAlarmSystem", ref bUseAlarmSystem);
    cfg.Get("bOptimizedRender", ref bOptimizedRender);
    cfg.Get("bExcludeHiddenBlocks", ref bExcludeHiddenBlocks);
    cfg.Get("strExcludeAlertKeyword", ref strExcludeAlertKeyword);
    cfg.Get("strAlertBlockTypes", ref strAlertBlockTypes);
    cfg.Get("HORIZONTAL_COUNT", ref HORIZONTAL_COUNT);
    cfg.Get("VERTICAL_COUNT", ref VERTICAL_COUNT);
    cfg.Get("REFRESH_INTERVAL", ref REFRESH_INTERVAL);

    cfg.Get("INSTRUCTION_PERCENT_LIMIT", ref INSTRUCTION_PERCENT_LIMIT);
    INSTRUCTION_PERCENT_LIMIT = Math.Max(Math.Min(INSTRUCTION_PERCENT_LIMIT, 1f), 0.01f);

}

bool ProcessRunCommands(string cmd)
{
    string[] tokens = cmd.Split(':');
    if (tokens.Length > 1)
    {
        cmd = tokens[0].Trim();

        int value;
        if (Int32.TryParse(tokens[1], out value) && value >= 0 && value <= lyts.Count)
        {
            pnStart = pnEnd = Math.Max(value - 1, 0);
        }
        else
        {
            pnStart = pnEnd = 0;
        }
    }
    else
    {
        pnStart = 0;
        pnEnd = lyts.Count - 1;
    }

    if (cmd.Equals("REDRAW"))
    {
        for (int i = pnStart; i <= pnEnd; i++)
        {
            if (lyts[i].DisplayParametersChanged() && lyts[i].ProcessDisplayParameters())
            {
                lyts[i].ResetStructureBase();
            }
        }
        RedrawStructure();
        return true;
    }
    else if (cmd.Equals("RELOAD"))
    {
        InitDisplayPanels();

        InitMiscPanels(rptPnls, strReportPanelTag);
        InitMiscPanels(statPnls, strStatusPanelTag);
        InitMiscPanels(menuPnls, strMenuPanelTag);

        if (bUseAlarmSystem)
        {
            alarmBlocks.Clear();
            GridTerminalSystem.SearchBlocksOfName(strAlarmTag, alarmBlocks);
        }

        groupStats.Clear();

        subCounter = 0;
        subMode = 3;
        mode = 0;

        init = false;
        return true;
    }
    else if (cmd.Equals("RESET"))
    {
        subCounter = 0;
        subMode = 0;
        mode = 0;

        init = false;
        return true;
    }
    else if (FLAG_CMD.ContainsKey(cmd))
    {
        int[] opt = FLAG_CMD[cmd];
        bool needRedraw = (opt[0] <= 3);

        for (int i = pnStart; i <= pnEnd; i++)
        {
            LayoutPanel p = lyts[i];

            switch (opt[0])
            {
                case 0:
                    p.bPaintDensity = (opt[1] == 2 ? !p.bPaintDensity : (opt[1] == 0));
                    break;
                case 1:
                    p.bUseRelativeDensity = (opt[1] == 2 ? !p.bUseRelativeDensity : (opt[1] == 0));
                    break;
                case 2:
                    p.bIncludeTerminalDensity = (opt[1] == 2 ? !p.bIncludeTerminalDensity : (opt[1] == 0));
                    break;
                case 3:
                    p.bSmoothEdges = (opt[1] == 2 ? !p.bSmoothEdges : (opt[1] == 0));
                    break;
                case 4:
                    p.bPaintLocationMarker = (opt[1] == 2 ? !p.bPaintLocationMarker : (opt[1] == 0));
                    break;
                case 5:
                    p.bPaintTerminalBlocks = (opt[1] == 2 ? !p.bPaintTerminalBlocks : (opt[1] == 0));
                    break;
                case 6:
                    p.bPaintAlertBlocks = (opt[1] == 2 ? !p.bPaintAlertBlocks : (opt[1] == 0));
                    break;
                case 7:
                    p.bBlinkForAlert = (opt[1] == 2 ? !p.bBlinkForAlert : (opt[1] == 0));
                    break;
                case 8:
                    p.bPaintCrossForAlert = (opt[1] == 2 ? !p.bPaintCrossForAlert : (opt[1] == 0));
                    break;
            }

            if (needRedraw)
            {
                p.WriteAndReset();
            }
            else
            {
                p.WriteDisplayParameters();
            }
        }

        if (needRedraw)
        {
            RedrawStructure();
            return true;
        }
        else
        {
            mode = (mode == 3 ? 4 : 0);
        }
    }
    else if (cmd.Equals("DEFAULT_VIEW"))
    {
        for (int i = pnStart; i <= pnEnd; i++)
        {
            CustomConfiguration.ParseCustomData(lyts[i].Panel, lyts[i].config);
            lyts[i].ProcessDisplayParameters(lyts[i].config.GetValueOrDefault("DEFAULTVIEW", ""));

            lyts[i].WriteAndReset();
        }
        RedrawStructure();
        return true;
    }
    else if (cmd.Equals("REMEMBER_VIEW"))
    {
        for (int i = pnStart; i <= pnEnd; i++)
        {
            lyts[i].config["DEFAULTVIEW"] = lyts[i].Panel.GetPublicTitle();
            CustomConfiguration.WriteCustomData(lyts[i].Panel, lyts[i].config);
        }
    }
    else if (cmd.Equals("ZOOM_IN") || cmd.Equals("ZOOM_OUT") || cmd.Equals("ZOOM_IN_FAST") || cmd.Equals("ZOOM_OUT_FAST"))
    {
        int startX;
        int endX;
        int minX;
        int maxX;
        int startY;
        int endY;
        int minY;
        int maxY;

        bool haveChanges = false;

        for (int i = pnStart; i <= pnEnd; i++)
        {
            GetLayoutStartEndMinMax(lyts[i], lyts[i].grAxXDir, out startX, out endX, out minX, out maxX);
            GetLayoutStartEndMinMax(lyts[i], lyts[i].grAxYDir, out startY, out endY, out minY, out maxY);

            bool needRedraw;

            if (cmd.EndsWith("FAST"))
            {
                int newStartX;
                int newStartY;
                int newEndX;
                int newEndY;

                if (cmd.Equals("ZOOM_IN_FAST"))
                {
                    newStartX = startX + ((endX - startX + 1) / 4);
                    newStartY = startY + ((endY - startY + 1) / 4);
                    newEndX = endX - ((endX - startX + 1) / 4);
                    newEndY = endY - ((endY - startY + 1) / 4);
                }
                else
                {
                    newStartX = Math.Max(startX - Math.Max((endX - startX + 1) / 2, 1), minX);
                    newStartY = Math.Max(startY - Math.Max((endY - startY + 1) / 2, 1), minY);
                    newEndX = Math.Min(endX + Math.Max((endX - startX + 1) / 2, 1), maxX);
                    newEndY = Math.Min(endY + Math.Max((endY - startY + 1) / 2, 1), maxY);
                }

                if (newStartX <= newEndX && newStartY <= newEndY && (newStartX != startX || newStartY != startY || newEndX != endX || newEndY != endY))
                {
                    needRedraw = true;

                    startX = newStartX;
                    endX = newEndX;
                    startY = newStartY;
                    endY = newEndY;
                }
                else
                {
                    needRedraw = false;
                }
            }
            else if (cmd.Equals("ZOOM_IN"))
            {
                startX++;
                endX--;
                startY++;
                endY--;

                needRedraw = (startX <= endX && startY <= endY);
            }
            else
            {
                startX--;
                endX++;
                startY--;
                endY++;

                needRedraw = (startX >= minX && endX <= maxX && startY >= minY && endY <= maxY);
            }

            if (needRedraw)
            {
                haveChanges = true;

                SetLayoutStartEnd(lyts[i], lyts[i].grAxXDir, startX, endX);
                SetLayoutStartEnd(lyts[i], lyts[i].grAxYDir, startY, endY);

                lyts[i].WriteAndReset();
            }
        }

        if (haveChanges)
        {
            RedrawStructure();
            return true;
        }
    }
    else if (cmd.Equals("ZOOM_RESET"))
    {
        for (int i = pnStart; i <= pnEnd; i++)
        {
            lyts[i].grStX = gridMin.X;
            lyts[i].grEndX = gridMax.X;
            lyts[i].grStY = gridMin.Y;
            lyts[i].grEndY = gridMax.Y;
            lyts[i].grStZ = gridMin.Z;
            lyts[i].grEndZ = gridMax.Z;

            lyts[i].WriteAndReset();
        }
        RedrawStructure();
        return true;
    }
    else if (cmd.Equals("FLIP_LEFT"))
    {
        for (int i = pnStart; i <= pnEnd; i++)
        {
            do
            {
                lyts[i].grAxX += 2;
                if (lyts[i].grAxX > 5)
                {
                    lyts[i].grAxX = (lyts[i].grAxX == 6 ? 1 : 0);
                }
            }
            while ((lyts[i].grAxY / 2) == (lyts[i].grAxX / 2));

            lyts[i].WriteAndReset();
        }
        RedrawStructure();
        return true;
    }
    else if (cmd.Equals("FLIP_RIGHT"))
    {
        for (int i = pnStart; i <= pnEnd; i++)
        {
            do
            {
                lyts[i].grAxX -= 2;
                if (lyts[i].grAxX < 0)
                {
                    lyts[i].grAxX = (lyts[i].grAxX == -2 ? 5 : 4);
                }
            }
            while ((lyts[i].grAxY / 2) == (lyts[i].grAxX / 2));

            lyts[i].WriteAndReset();
        }
        RedrawStructure();
        return true;
    }
    else if (cmd.Equals("FLIP_UP"))
    {
        for (int i = pnStart; i <= pnEnd; i++)
        {
            do
            {
                lyts[i].grAxY += 2;
                if (lyts[i].grAxY > 5)
                {
                    lyts[i].grAxY = (lyts[i].grAxY == 6 ? 1 : 0);
                }
            }
            while ((lyts[i].grAxY / 2) == (lyts[i].grAxX / 2));

            lyts[i].WriteAndReset();
        }
        RedrawStructure();
        return true;
    }
    else if (cmd.Equals("FLIP_DOWN"))
    {
        for (int i = pnStart; i <= pnEnd; i++)
        {
            do
            {
                lyts[i].grAxY -= 2;
                if (lyts[i].grAxY < 0)
                {
                    lyts[i].grAxY = (lyts[i].grAxY == -2 ? 5 : 4);
                }
            }
            while ((lyts[i].grAxY / 2) == (lyts[i].grAxX / 2));

            lyts[i].WriteAndReset();
        }
        RedrawStructure();
        return true;
    }
    else if (cmd.Equals("SPIN_CW"))
    {
        for (int i = pnStart; i <= pnEnd; i++)
        {
            int tmpGridAxisY = lyts[i].grAxY;
            lyts[i].grAxY = lyts[i].grAxX;
            lyts[i].grAxX = (tmpGridAxisY % 2 == 0 ? tmpGridAxisY + 1 : tmpGridAxisY - 1);

            lyts[i].WriteAndReset();
        }
        RedrawStructure();
        return true;
    }
    else if (cmd.Equals("SPIN_CCW"))
    {
        for (int i = pnStart; i <= pnEnd; i++)
        {
            int tmpGridAxisX = lyts[i].grAxX;
            lyts[i].grAxX = lyts[i].grAxY;
            lyts[i].grAxY = (tmpGridAxisX % 2 == 0 ? tmpGridAxisX + 1 : tmpGridAxisX - 1);

            lyts[i].WriteAndReset();
        }
        RedrawStructure();
        return true;
    }
    else if (cmd.Equals("SHRINK_SLICE") || cmd.Equals("EXPAND_SLICE"))
    {
        bool shrink = cmd.Equals("SHRINK_SLICE");

        bool haveChanges = false;

        for (int i = pnStart; i <= pnEnd; i++)
        {
            int startZ;
            int endZ;
            int minZ;
            int maxZ;

            GetLayoutStartEndMinMax(lyts[i], lyts[i].grAxZDir, out startZ, out endZ, out minZ, out maxZ);

            startZ += (shrink ? 1 : -1);
            endZ += (shrink ? -1 : 1);

            if (startZ >= minZ && endZ <= maxZ && startZ <= endZ)
            {
                haveChanges = true;

                SetLayoutStartEnd(lyts[i], lyts[i].grAxZDir, startZ, endZ);

                lyts[i].WriteAndReset();
            }
        }

        if (haveChanges)
        {
            RedrawStructure();
            return true;
        }
    }
    else if (cmd.Equals("INC_HDENSITY") || cmd.Equals("DEC_HDENSITY") || cmd.Equals("INC_MDENSITY") || cmd.Equals("DEC_MDENSITY") || cmd.Equals("INC_LDENSITY") || cmd.Equals("DEC_LDENSITY"))
    {
        string[] cmdTokens = cmd.Split('_');

        bool isInc = cmdTokens[0].Equals("INC");
        int denseType = (cmdTokens[1].Equals("HDENSITY") ? 0 : (cmdTokens[1].Equals("MDENSITY") ? 1 : 2));

        bool haveChanges = false;

        for (int i = pnStart; i <= pnEnd; i++)
        {
            float newValue = (denseType == 0 ? lyts[i].highDensityThreshold : (denseType == 1 ? lyts[i].midDensityThreshold : lyts[i].lowDensityThreshold)) + (isInc ? -0.05f : 0.05f);
            newValue = Math.Min(1f, Math.Max(0f, newValue));

            bool needRedraw = false;

            switch (denseType)
            {
                case 0:
                    if (lyts[i].highDensityThreshold != newValue)
                    {
                        lyts[i].highDensityThreshold = newValue;
                        needRedraw = true;
                    }
                    break;
                case 1:
                    if (lyts[i].midDensityThreshold != newValue)
                    {
                        lyts[i].midDensityThreshold = newValue;
                        needRedraw = true;
                    }
                    break;
                case 2:
                    if (lyts[i].lowDensityThreshold != newValue)
                    {
                        lyts[i].lowDensityThreshold = newValue;
                        needRedraw = true;
                    }
                    break;
            }

            if (needRedraw)
            {
                haveChanges = true;
                lyts[i].WriteAndReset();
            }
        }

        if (haveChanges)
        {
            RedrawStructure();
            return true;
        }
    }
    else if (GRID_CMD.Contains(cmd))
    {
        bool isStart;
        bool isPositive;
        bool isPan;

        bool haveChanges = false;

        string[] token = cmd.Split('_');
        Base6Directions.Axis axisDirection;

        isPan = token[0].Equals("PAN");

        if (isPan)
        {
            isPositive = ("RIGHT,DOWN,BACKWARD".Contains(token[1]));
        }
        else
        {
            isPositive = token[0].Equals("PLUS");
        }

        for (int i = pnStart; i <= pnEnd; i++)
        {
            switch (token[1])
            {
                case "LEFT":
                    isStart = !lyts[i].grFlipX;
                    isPositive = (lyts[i].grFlipX ? !isPositive : isPositive);
                    axisDirection = lyts[i].grAxXDir;
                    break;
                case "RIGHT":
                    isStart = lyts[i].grFlipX;
                    isPositive = (lyts[i].grFlipX ? !isPositive : isPositive);
                    axisDirection = lyts[i].grAxXDir;
                    break;
                case "UP":
                    isStart = !lyts[i].grFlipY;
                    isPositive = (lyts[i].grFlipY ? !isPositive : isPositive);
                    axisDirection = lyts[i].grAxYDir;
                    break;
                case "DOWN":
                    isStart = lyts[i].grFlipY;
                    isPositive = (lyts[i].grFlipY ? !isPositive : isPositive);
                    axisDirection = lyts[i].grAxYDir;
                    break;
                case "FORWARD":
                    isStart = !lyts[i].grFlipZ;
                    isPositive = (lyts[i].grFlipZ ? !isPositive : isPositive);
                    axisDirection = lyts[i].grAxZDir;
                    break;
                case "BACKWARD":
                    isStart = lyts[i].grFlipZ;
                    isPositive = (lyts[i].grFlipZ ? !isPositive : isPositive);
                    axisDirection = lyts[i].grAxZDir;
                    break;
                default:
                    isStart = isPositive = false;
                    axisDirection = 0;
                    break;
            }

            bool needRedraw = false;

            switch (axisDirection)
            {
                case Base6Directions.Axis.LeftRight:
                    needRedraw = AdjustGridSize(ref lyts[i].grStX, ref lyts[i].grEndX, gridMin.X, gridMax.X, isStart, isPositive, isPan);
                    break;
                case Base6Directions.Axis.UpDown:
                    needRedraw = AdjustGridSize(ref lyts[i].grStY, ref lyts[i].grEndY, gridMin.Y, gridMax.Y, isStart, isPositive, isPan);
                    break;
                case Base6Directions.Axis.ForwardBackward:
                    needRedraw = AdjustGridSize(ref lyts[i].grStZ, ref lyts[i].grEndZ, gridMin.Z, gridMax.Z, isStart, isPositive, isPan);
                    break;
            }

            if (needRedraw)
            {
                haveChanges = true;
                lyts[i].WriteAndReset();
            }
        }

        if (haveChanges)
        {
            RedrawStructure();
            return true;
        }
    }
    else if (cmd.Equals("LIGHTEN"))
    {
        for (int i = pnStart; i <= pnEnd; i++)
        {
            lyts[i].LightenDisplay();
        }
    }
    else if (cmd.Equals("DARKEN"))
    {
        for (int i = pnStart; i <= pnEnd; i++)
        {
            lyts[i].DarkenDisplay();
        }
    }
    else if (cmd.Equals("TOGGLE_IMAGE"))
    {
        for (int i = pnStart; i <= pnEnd; i++)
        {
            lyts[i].ToggleImageDisplay();
        }
    }
    else if (cmd.Equals("SHOW_HIDE_NUMBER"))
    {
        subMode = 0;
        mode = (mode == 3 ? 4 : 3);
    }
    else if (cmd.StartsWith("TOGGLE_AL_"))
    {
        for (int i = pnStart; i <= pnEnd; i++)
        {
            byte mask = 0;
            switch (cmd.Substring(10))
            {
                case "VENT":
                    mask = AL_VENT;
                    break;
                case "SENSOR":
                    mask = AL_SENSOR;
                    break;
                case "TURRET":
                    mask = AL_TURRET;
                    break;
                case "DOOR":
                    mask = AL_DOOR;
                    break;
                case "HACKED":
                    mask = AL_HACKED;
                    break;
            }

            if (mask > 0)
            {
                if ((lyts[i].alertMask & mask) > 0)
                {
                    lyts[i].alertMask &= (byte)~mask;
                }
                else
                {
                    lyts[i].alertMask |= mask;
                }
                lyts[i].WriteDisplayParameters();
            }
        }
        mode = (mode == 3 ? 4 : 0);
    }

    return false;
}

void GetLayoutStartEndMinMax(LayoutPanel layout, Base6Directions.Axis axis, out int start, out int end, out int min, out int max)
{
    if (axis == Base6Directions.Axis.LeftRight)
    {
        start = layout.grStX;
        end = layout.grEndX;
        min = gridMin.X;
        max = gridMax.X;
    }
    else if (axis == Base6Directions.Axis.UpDown)
    {
        start = layout.grStY;
        end = layout.grEndY;
        min = gridMin.Y;
        max = gridMax.Y;
    }
    else
    {
        start = layout.grStZ;
        end = layout.grEndZ;
        min = gridMin.Z;
        max = gridMax.Z;
    }
}

void SetLayoutStartEnd(LayoutPanel layout, Base6Directions.Axis axis, int start, int end)
{
    if (axis == Base6Directions.Axis.LeftRight)
    {
        layout.grStX = start;
        layout.grEndX = end;
    }
    else if (axis == Base6Directions.Axis.UpDown)
    {
        layout.grStY = start;
        layout.grEndY = end;
    }
    else
    {
        layout.grStZ = start;
        layout.grEndZ = end;
    }
}

string GetLayoutParameterValue(string key)
{
    string[] tokens = key.Split(':');
    if (tokens.Length > 1)
    {
        key = tokens[0].Trim();

        int value;
        if (Int32.TryParse(tokens[1], out value) && value > 0 && value <= lyts.Count)
        {
            return lyts[value - 1].GetParameterDescription(key);
        }
    }
    return "";
}

/*----- Initialization Methods -----*/

void StaticInit()
{
    if (strExcludeAlertKeyword != null)
    {
        strExcludeAlertKeyword = (strExcludeAlertKeyword.Length == 0 ? null : strExcludeAlertKeyword.ToUpper());
    }

    progressIcons[0] = '\u2014';
    progressIcons[1] = '\\';
    progressIcons[2] = '|';
    progressIcons[3] = '/';
    progressIcons[4] = '\u2014';
    progressIcons[5] = '\\';
    progressIcons[6] = '|';
    progressIcons[7] = '/';

    ADJACENT_MASKS[0] = SUB_X;
    ADJACENT_MASKS[1] = ADD_X;
    ADJACENT_MASKS[2] = SUB_Y;
    ADJACENT_MASKS[3] = ADD_Y;
    ADJACENT_MASKS[4] = SUB_Z;
    ADJACENT_MASKS[5] = ADD_Z;

    WATCH_BLK_TYPE = new HashSet<string>(strAlertBlockTypes.Split(','));

    string[] flags = new string[] { "DENSE", "RELDENSE", "TERMDENSE", "SMOOTH", "MARKER", "TERM", "ALERT", "BLINK", "CROSS" };
    FLAG_CMD = new Dictionary<string, int[]>();
    for (int i = 0; i < flags.Length; i++)
    {
        FLAG_CMD.Add(flags[i], new int[] { i, 0 });
        FLAG_CMD.Add("NO_" + flags[i], new int[] { i, 1 });
        FLAG_CMD.Add("TOGGLE_" + flags[i], new int[] { i, 2 });
    }

    GRID_CMD = new HashSet<string>(new string[] { "PLUS_LEFT", "MINUS_LEFT", "PLUS_RIGHT", "MINUS_RIGHT", "PLUS_UP", "MINUS_UP", "PLUS_DOWN", "MINUS_DOWN", "PLUS_FORWARD", "MINUS_FORWARD", "PLUS_BACKWARD", "MINUS_BACKWARD", "PAN_LEFT", "PAN_RIGHT", "PAN_UP", "PAN_DOWN", "PAN_FORWARD", "PAN_BACKWARD" });

    MENU_CMD = new Dictionary<string, int>();
    MENU_CMD.Add("UP", 1);
    MENU_CMD.Add("U", 1);
    MENU_CMD.Add("DOWN", 2);
    MENU_CMD.Add("D", 2);
    MENU_CMD.Add("LEFT", 3);
    MENU_CMD.Add("L", 3);
    MENU_CMD.Add("RIGHT", 4);
    MENU_CMD.Add("R", 4);
    MENU_CMD.Add("BACK", 5);
    MENU_CMD.Add("B", 5);
    MENU_CMD.Add("ENTER", 6);
    MENU_CMD.Add("E", 6);

    priorityBlocks = new List<IMyTerminalBlock>[3];
    for (int i = 0; i < priorityBlocks.Length; i++)
    {
        priorityBlocks[i] = new List<IMyTerminalBlock>();
    }

    InitMenu();
}

void InitMenu()
{
    Func<string, string> funcVK = GetLayoutParameterValue;

    Menu mnCL = new Menu(funcVK);
    mnCL.Name = "Lighten Display";
    mnCL.Action = "LIGHTEN:{pindex}";
    mnCL.ValueKey = "BRIGHTNESS:{pindex}";
    mnCL.NameReplaceLevel = 4;
    mnCL.ActionReplaceLevel = 4;

    Menu mnCD = new Menu(funcVK);
    mnCD.Name = "Darken Display";
    mnCD.Action = "DARKEN:{pindex}";
    mnCD.ValueKey = "BRIGHTNESS:{pindex}";
    mnCD.NameReplaceLevel = 4;
    mnCD.ActionReplaceLevel = 4;

    Menu mnCTI = new Menu(funcVK);
    mnCTI.Name = "Toggle Image";
    mnCTI.Action = "TOGGLE_IMAGE:{pindex}";
    mnCTI.ValueKey = "IMAGE:{pindex}";
    mnCTI.NameReplaceLevel = 4;
    mnCTI.ActionReplaceLevel = 4;

    Menu mnCTAC = new Menu(funcVK);
    mnCTAC.Name = "Toggle Cross For Alert";
    mnCTAC.Action = "TOGGLE_CROSS:{pindex}";
    mnCTAC.ValueKey = "CROSS:{pindex}";
    mnCTAC.NameReplaceLevel = 4;
    mnCTAC.ActionReplaceLevel = 4;

    Menu mnCTLM = new Menu(funcVK);
    mnCTLM.Name = "Toggle Location Marker";
    mnCTLM.Action = "TOGGLE_MARKER:{pindex}";
    mnCTLM.ValueKey = "MARKER:{pindex}";
    mnCTLM.NameReplaceLevel = 4;
    mnCTLM.ActionReplaceLevel = 4;

    Menu mnCA = new Menu();
    mnCA.Title = "Panel {index}";
    mnCA.Name = "Panel {index} - {name}";
    mnCA.RepeatList = lyts;
    mnCA.NameReplaceLevel = 3;
    mnCA.AddMenu(mnCL);
    mnCA.AddMenu(mnCD);
    mnCA.AddMenu(mnCTI);
    mnCA.AddMenu(mnCTAC);
    mnCA.AddMenu(mnCTLM);

    Menu mnCPA = new Menu();
    mnCPA.Name = "All Panels";
    mnCPA.CurrentSelection = -99;
    Menu mnCLC = mnCL.CopyMenu();
    Menu mnCDC = mnCD.CopyMenu();
    Menu mnCTIC = mnCTI.CopyMenu();
    Menu mnCTACC = mnCTAC.CopyMenu();
    Menu mnCTLMC = mnCTLM.CopyMenu();
    mnCLC.Action = mnCLC.Action.Replace(":{pindex}", "");
    mnCLC.ValueKey = "";
    mnCDC.Action = mnCDC.Action.Replace(":{pindex}", "");
    mnCDC.ValueKey = "";
    mnCTIC.Action = mnCTIC.Action.Replace(":{pindex}", "");
    mnCTIC.ValueKey = "";
    mnCTACC.Action = mnCTACC.Action.Replace(":{pindex}", "");
    mnCTACC.ValueKey = "";
    mnCTLMC.Action = mnCTLMC.Action.Replace(":{pindex}", "");
    mnCTLMC.ValueKey = "";
    mnCPA.AddMenu(mnCLC);
    mnCPA.AddMenu(mnCDC);
    mnCPA.AddMenu(mnCTIC);
    mnCPA.AddMenu(mnCTACC);
    mnCPA.AddMenu(mnCTLMC);

    Menu mnCosmetics = new Menu();
    mnCosmetics.Name = "Panel Cosmetics";
    mnCosmetics.AddMenu(mnCPA);
    mnCosmetics.AddMenu(mnCA);

    string[] menuNamesArray = new string[] { "Toggle Density", "Toggle Relative Density", "Toggle Smooth Edges", "Toggle Terminal Blocks", "Toggle Alert Blocks", "Toggle Vent Alerts", "Toggle Sensor Alerts", "Toggle Turret Alerts", "Toggle Door Alerts", "Toggle Hacked Alerts", "Toggle Alert Blinking", "Default View", "Remember View", "Zoom In", "Zoom Out", "Zoom In Fast", "Zoom Out Fast", "Zoom Reset", "Flip Left", "Flip Right", "Flip Up", "Flip Down", "Spin Clockwise", "Spin Counter Clockwise", "Shrink Slice", "Expand Slice", "Increase High Density", "Decrease High Density", "Increase Medium Density", "Decrease Medium Density", "Increase Low Density", "Decrease Low Density", "Pan Left", "Pan Right", "Pan Up", "Pan Down", "Pan Forward", "Pan Backward", "Plus Left", "Minus Left", "Plus Right", "Minus Right", "Plus Up", "Minus Up", "Plus Down", "Minus Down", "Plus Forward", "Minus Forward", "Plus Backward", "Minus Backward" };
    string[] menuActionsArray = new string[] { "TOGGLE_DENSE", "TOGGLE_RELDENSE", "TOGGLE_SMOOTH", "TOGGLE_TERM", "TOGGLE_ALERT", "TOGGLE_AL_VENT", "TOGGLE_AL_SENSOR", "TOGGLE_AL_TURRET", "TOGGLE_AL_DOOR", "TOGGLE_AL_HACKED", "TOGGLE_BLINK", "DEFAULT_VIEW", "REMEMBER_VIEW", "ZOOM_IN", "ZOOM_OUT", "ZOOM_IN_FAST", "ZOOM_OUT_FAST", "ZOOM_RESET", "FLIP_LEFT", "FLIP_RIGHT", "FLIP_UP", "FLIP_DOWN", "SPIN_CW", "SPIN_CCW", "SHRINK_SLICE", "EXPAND_SLICE", "INC_HDENSITY", "DEC_HDENSITY", "INC_MDENSITY", "DEC_MDENSITY", "INC_LDENSITY", "DEC_LDENSITY", "PAN_LEFT", "PAN_RIGHT", "PAN_UP", "PAN_DOWN", "PAN_FORWARD", "PAN_BACKWARD", "PLUS_LEFT", "MINUS_LEFT", "PLUS_RIGHT", "MINUS_RIGHT", "PLUS_UP", "MINUS_UP", "PLUS_DOWN", "MINUS_DOWN", "PLUS_FORWARD", "MINUS_FORWARD", "PLUS_BACKWARD", "MINUS_BACKWARD" };
    string[] menuActionsValueKey = new string[] { "DENSE", "RELDENSE", "SMOOTH", "TERM", "ALERT", "AL_VENT", "AL_SENSOR", "AL_TURRET", "AL_DOOR", "AL_HACKED", "BLINK", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "" };

    Menu mnAA = new Menu();
    mnAA.Title = "Panel {index}";
    mnAA.Name = "Panel {index} - {name}";
    mnAA.RepeatList = lyts;
    mnAA.NameReplaceLevel = 3;

    Menu mnAPA = new Menu();
    mnAPA.Name = "All Panels";
    mnAPA.CurrentSelection = -99;

    for (int i = 0; i < menuNamesArray.Length; i++)
    {
        Menu mnNext = new Menu(funcVK);
        mnNext.Name = menuNamesArray[i];
        mnNext.Action = menuActionsArray[i] + ":{pindex}";
        mnNext.ValueKey = menuActionsValueKey[i] + (menuActionsValueKey[i].Length > 0 ? ":{pindex}" : "");
        mnNext.NameReplaceLevel = 4;
        mnNext.ActionReplaceLevel = 4;
        mnAA.AddMenu(mnNext);
        Menu mnNextCopy = mnNext.CopyMenu();
        mnNextCopy.Action = mnNextCopy.Action.Replace(":{pindex}", "");
        mnNextCopy.ValueKey = "";
        mnAPA.AddMenu(mnNextCopy);
    }

    Menu mnActions = new Menu();
    mnActions.Name = "Panel Actions";
    mnActions.AddMenu(mnAPA);
    mnActions.AddMenu(mnAA);

    Menu mnTogNum = new Menu();
    mnTogNum.Name = "Show/Hide Panel Number";
    mnTogNum.Action = "SHOW_HIDE_NUMBER";

    Menu mnRedraw = new Menu();
    mnRedraw.Name = "Redraw All Panels";
    mnRedraw.Action = "REDRAW";

    Menu mnReload = new Menu();
    mnReload.Name = "Reload Panels";
    mnReload.Action = "RELOAD";

    Menu mnAreYouSure = new Menu();
    mnAreYouSure.Name = "Are You Sure?";

    Menu mnBlank = new Menu();
    mnBlank.Name = "-";

    Menu mnResetConfirm = new Menu();
    mnResetConfirm.Name = "Yes";
    mnResetConfirm.Action = "RESET";

    Menu mnReset = new Menu();
    mnReset.Name = "Reset System";
    mnReset.AddMenu(mnAreYouSure);
    mnReset.AddMenu(mnBlank);
    mnReset.AddMenu(mnResetConfirm);

    rtMnu = new Menu();
    rtMnu.Name = "Home";
    rtMnu.AddMenu(mnCosmetics);
    rtMnu.AddMenu(mnActions);
    rtMnu.AddMenu(mnTogNum);
    rtMnu.AddMenu(mnRedraw);
    rtMnu.AddMenu(mnReload);
    rtMnu.AddMenu(mnReset);

    selMnu = rtMnu;

    curMnId = 0;
    actMnCnt = 0;
}

void InitMiscPanels(List<IMyTextPanel> panels, string tagName)
{
    List<IMyTerminalBlock> blks = new List<IMyTerminalBlock>();
    GridTerminalSystem.SearchBlocksOfName(tagName, blks);
    panels.Clear();
    for (int i = 0; i < blks.Count; i++)
    {
        IMyTextPanel panel = blks[i] as IMyTextPanel;
        if (panel != null)
        {
            panel.ShowPublicTextOnScreen();
            panels.Add(panel);
        }
    }
}

void InitDisplayPanels()
{
    List<IMyTerminalBlock> displayPanels = new List<IMyTerminalBlock>();
    GridTerminalSystem.SearchBlocksOfName(strDisplayPanelTag, displayPanels);

    lyts.Clear();
    for (int i = 0; i < displayPanels.Count; i++)
    {
        IMyTextPanel panel = displayPanels[i] as IMyTextPanel;
        if (panel != null)
        {
            LayoutPanel layout = new LayoutPanel(GridTerminalSystem, Runtime, Me);

            layout.Panel = panel;
            layout.PanelHorizontalCount = HORIZONTAL_COUNT;
            layout.PanelVerticalCount = VERTICAL_COUNT;

            layout.InitDisplayPanels();

            lyts.Add(layout);
        }
    }
}

bool InitStructureData(IMyCubeGrid cubeGrid, Stack<Vector3I> remainingCubes, Dictionary<Vector3I, byte> foundCubes, HashSet<Vector3I> testedCubes, int instLimit)
{
    bool exists;
    byte adjacent;
    Vector3I nextCube;

    while (remainingCubes.Count > 0 && Runtime.CurrentInstructionCount < instLimit)
    {
        Vector3I currentCube = remainingCubes.Pop();
        adjacent = 0;

        for (int i = 0; i < 6; i++)
        {
            nextCube = currentCube + ADJACENT_VECTORS[i];
            if (testedCubes.Contains(nextCube))
            {
                if (foundCubes.ContainsKey(nextCube))
                {
                    adjacent |= ADJACENT_MASKS[i];
                }
            }
            else
            {
                testedCubes.Add(nextCube);
                exists = cubeGrid.CubeExists(nextCube);
                if (exists)
                {
                    remainingCubes.Push(nextCube);
                    foundCubes.Add(nextCube, 0);
                    adjacent |= ADJACENT_MASKS[i];
                }
            }
        }

        foundCubes[currentCube] = adjacent;
    }

    return (remainingCubes.Count == 0);
}

/*----- Grid Computation Methods -----*/

int ComputeStructureDensity(List<KeyValuePair<Vector3I, byte>> blkList, int index, int instLimit)
{
    while (index < blkList.Count && Runtime.CurrentInstructionCount < instLimit)
    {
        KeyValuePair<Vector3I, byte> blk = blkList[index];

        IMySlimBlock slimBlock = Me.CubeGrid.GetCubeBlock(blk.Key);
        bool isArmorBlock = (slimBlock == null || slimBlock.FatBlock == null);

        for (int i = pnStart; i <= pnEnd; i++)
        {
            if (lyts[i].bPaintDensity)
            {
                lyts[i].ComputeStructureDensity(blk, isArmorBlock);
            }
        }

        index++;
    }

    return index;
}

void CompileTotalGridSize()
{
    totalMin = gridMax;
    totalMax = gridMin;

    for (int i = 0; i < lyts.Count; i++)
    {
        totalMin.X = Math.Min(totalMin.X, lyts[i].grMin.X);
        totalMin.Y = Math.Min(totalMin.Y, lyts[i].grMin.Y);
        totalMin.Z = Math.Min(totalMin.Z, lyts[i].grMin.Z);
        totalMax.X = Math.Max(totalMax.X, lyts[i].grMax.X);
        totalMax.Y = Math.Max(totalMax.Y, lyts[i].grMax.Y);
        totalMax.Z = Math.Max(totalMax.Z, lyts[i].grMax.Z);
    }
}

int FilterTerminalBlocks(IMyCubeGrid cubeGrid, List<IMyTerminalBlock> blkList, List<IMyTerminalBlock> filteredList, int index, int instLimit)
{
    while (index < blkList.Count && Runtime.CurrentInstructionCount < instLimit)
    {
        IMyTerminalBlock blk = blkList[index];
        if (blk.CubeGrid == cubeGrid)
        {
            if (!bExcludeHiddenBlocks || blk.ShowInTerminal)
            {
                filteredList.Add(blk);
            }
        }

        index++;
    }

    return index;
}

int FilterWatchedBlocks(IMyCubeGrid grid, List<IMyTerminalBlock> blkList, List<WatchField> watchedList, int index, int instLimit)
{
    while (index < blkList.Count && Runtime.CurrentInstructionCount < instLimit)
    {
        IMyTerminalBlock blk = blkList[index];

        if (Vector3I.BoxContains(totalMin, totalMax, blk.Position) && blk.CubeGrid == grid)
        {
            if (WATCH_BLK_TYPE.Contains(blk.BlockDefinition.TypeIdString.Replace("MyObjectBuilder_", "")))
            {
                if (strExcludeAlertKeyword == null || !blk.CustomName.ToUpper().Contains(strExcludeAlertKeyword))
                {
                    watchedList.Add(new WatchField(blk));
                }
            }
        }

        index++;
    }

    return index;
}

/*----- Paint Methods -----*/

int PaintStructureBase(List<KeyValuePair<Vector3I, byte>> blkList, int index, int instLimit)
{
    while (index < blkList.Count && Runtime.CurrentInstructionCount < instLimit)
    {
        KeyValuePair<Vector3I, byte> blk = blkList[index];

        for (int i = pnStart; i <= pnEnd; i++)
        {
            lyts[i].PaintStructureBase(blk);
        }

        index++;
    }

    return index;
}

int PaintMissingStructure(List<KeyValuePair<Vector3I, byte>> blkList, int index, int instLimit)
{
    while (index < blkList.Count && Runtime.CurrentInstructionCount < instLimit)
    {
        KeyValuePair<Vector3I, byte> blk = blkList[index];

        if (!Me.CubeGrid.CubeExists(blk.Key))
        {
            for (int i = 0; i < lyts.Count; i++)
            {
                lyts[i].PaintMissingStructure(blk);
            }

            msStructCnt++;
        }

        index++;
    }

    return index;
}

int PaintGoodTerminalBlocks(List<IMyTerminalBlock> blkList, List<IMyTerminalBlock>[] badBlocks, int index, int instLimit)
{
    while (index < blkList.Count && Runtime.CurrentInstructionCount < instLimit)
    {
        IMyTerminalBlock blk = blkList[index];

        int drawLevel;

        if (blk.CubeGrid == Me.CubeGrid)
        {
            if (blk.IsFunctional)
            {
                drawLevel = 1;
            }
            else if (Me.CubeGrid.CubeExists(blk.Position))
            {
                IMySlimBlock slimBlock = Me.CubeGrid.GetCubeBlock(blk.Position);
                if (slimBlock != null && slimBlock.FatBlock != blk)
                {
                    IMyTerminalBlock replaceBlock = slimBlock.FatBlock as IMyTerminalBlock;
                    if (replaceBlock != null && replaceBlock.Min == blk.Min && replaceBlock.Max == blk.Max)
                    {
                        blkList[index] = replaceBlock;
                        blk = replaceBlock;
                    }

                    drawLevel = 3;
                }
                else
                {
                    drawLevel = 2;
                }
            }
            else
            {
                drawLevel = 3;
            }
        }
        else
        {
            IMyTerminalBlock replaceBlock = null;
            if (Me.CubeGrid.CubeExists(blk.Position))
            {
                IMySlimBlock slimBlock = Me.CubeGrid.GetCubeBlock(blk.Position);
                if (slimBlock != null)
                {
                    replaceBlock = slimBlock.FatBlock as IMyTerminalBlock;
                    if (replaceBlock != null && replaceBlock.Min == blk.Min && replaceBlock.Max == blk.Max)
                    {
                        blkList[index] = replaceBlock;
                        blk = replaceBlock;
                    }
                }
            }

            if (replaceBlock == null)
            {
                drawLevel = 3;
            }
            else if (replaceBlock.IsFunctional)
            {
                drawLevel = 1;
            }
            else
            {
                drawLevel = 2;
            }
        }

        if (drawLevel < 3 && blk.IsBeingHacked)
        {
            drawLevel = 4;
        }

        if (drawLevel == 1)
        {
            for (int i = 0; i < lyts.Count; i++)
            {
                if (lyts[i].bPaintTerminalBlocks)
                {
                    lyts[i].PaintTerminalBlocks(blk, drawLevel, chrTerminalBlockFunctional, false);
                }
            }
        }
        else
        {
            badBlocks[drawLevel - 2].Add(blk);
        }

        index++;
    }

    return index;
}

int PaintBadTerminalBlocks(List<IMyTerminalBlock>[] badBlocks, int index, int instLimit)
{
    while (index < priorityCount && Runtime.CurrentInstructionCount < instLimit)
    {
        IMyTerminalBlock blk;
        char chrPaint;
        int drawLevel;

        if (index < dmgTermCnt)
        {
            blk = priorityBlocks[0][index];
            chrPaint = chrTerminalBlockDamaged;
            drawLevel = 2;
        }
        else if (index < msTermCnt + dmgTermCnt)
        {
            blk = priorityBlocks[1][index - dmgTermCnt];
            chrPaint = chrTerminalBlockMissing;
            drawLevel = 3;
        }
        else
        {
            blk = priorityBlocks[2][index - msStructCnt - dmgTermCnt];
            chrPaint = chrHacked;
            drawLevel = 4;
        }

        for (int i = 0; i < lyts.Count; i++)
        {
            if (lyts[i].bPaintTerminalBlocks)
            {
                if (drawLevel != 4 || (lyts[i].alertMask & AL_HACKED) > 0)
                {
                    lyts[i].PaintTerminalBlocks(blk, drawLevel, chrPaint, (drawLevel == 4));
                }
            }
        }

        index++;
    }

    return index;
}

int PaintWatchedBlocks(List<WatchField> blkList, int index, int instLimit)
{
    IMyAirVent airVent;
    IMySensorBlock sensor;
    IMyLargeTurretBase turret;
    IMyDoor door;

    while (index < blkList.Count && Runtime.CurrentInstructionCount < instLimit)
    {
        IMyTerminalBlock blk = blkList[index].Block;

        if (blk.CubeGrid != Me.CubeGrid || !blk.IsFunctional)
        {
            if (Me.CubeGrid.CubeExists(blk.Position))
            {
                IMySlimBlock slimBlock = Me.CubeGrid.GetCubeBlock(blk.Position);
                if (slimBlock != null && slimBlock.FatBlock != blk)
                {
                    IMyTerminalBlock replaceBlock = slimBlock.FatBlock as IMyTerminalBlock;
                    if (replaceBlock != null && replaceBlock.Min == blk.Min && replaceBlock.Max == blk.Max)
                    {
                        blkList[index] = new WatchField(replaceBlock);
                        blk = replaceBlock;
                    }
                }
            }
        }

        byte mask = 0;
        char chrPaint = chrStructure;

        if ((airVent = blk as IMyAirVent) != null)
        {
            if (airVent.IsFunctional && (airVent.Status != VentStatus.Pressurized))
            {
                mask = AL_VENT;
                chrPaint = chrAlertAirVents;

                ventALCnt++;
            }
        }
        else if ((sensor = blk as IMySensorBlock) != null)
        {
            if (sensor.LastDetectedEntity.EntityId != 0)
            {
                Vector3D pos = sensor.LastDetectedEntity.Position;
                Vector3I detectedPosition = Me.CubeGrid.WorldToGridInteger(pos);

                if (needSensorDetected)
                {
                    sensorDetected.Add(pos);
                }

                for (int i = 0; i < lyts.Count; i++)
                {
                    if (lyts[i].bPaintAlertBlocks && lyts[i].isBlinkFrame && lyts[i].scArea <= BOX_AREA_DRAW_DENSITY_LIMIT && ((lyts[i].alertMask & AL_SENSOR) > 0))
                    {
                        lyts[i].PaintWatchedBlocks(blk, detectedPosition, chrDetectedEntity);
                    }
                }

                sensALCnt++;
            }
        }
        else if ((turret = blk as IMyLargeTurretBase) != null)
        {
            if (turret.IsFunctional)
            {
                if (turret.HasTarget)
                {
                    mask = AL_TURRET;
                    chrPaint = chrAlertTurrets;

                    shootTurCnt++;
                }
            }
        }
        else if ((door = blk as IMyDoor) != null)
        {
            if (door.IsFunctional && (door.Status != DoorStatus.Closed))
            {
                mask = AL_DOOR;
                chrPaint = chrAlertDoors;
            }
        }

        if (mask > 0)
        {
            for (int i = 0; i < lyts.Count; i++)
            {
                if (lyts[i].bPaintAlertBlocks && lyts[i].isBlinkFrame && lyts[i].scArea <= BOX_AREA_DRAW_DENSITY_LIMIT && ((lyts[i].alertMask & mask) > 0))
                {
                    lyts[i].PaintWatchedBlocks(blk, null, chrPaint);
                }
            }
        }

        index++;
    }

    return index;
}

/*----- Definitions -----*/

public class WatchField
{
    public WatchField(IMyTerminalBlock blk)
    {
        Block = blk;
    }
    public IMyTerminalBlock Block;
    public float[] Value;
}

public class Menu
{
    public string Title { get; set; }
    public string Name { get; set; }
    public string Action { get; set; }

    public Func<string, string> FuncValueLookup { get; set; }
    public string ValueKey { get; set; }

    public Menu Parent { get; set; }
    public List<Menu> ChildMenus { get; set; }

    public System.Collections.IList RepeatList { get; set; }
    public int NameReplaceLevel { get; set; }
    public int ActionReplaceLevel { get; set; }

    public int CurrentSelection { get; set; }

    public Menu() { }
    public Menu(Func<string, string> funcValueKeyLookup)
    {
        FuncValueLookup = funcValueKeyLookup;
    }

    public Menu CopyMenu()
    {
        Menu newMenu = new Menu();

        newMenu.Title = Title;
        newMenu.Name = Name;
        newMenu.Action = Action;
        newMenu.Parent = Parent;
        newMenu.ChildMenus = ChildMenus;
        newMenu.RepeatList = RepeatList;
        newMenu.NameReplaceLevel = NameReplaceLevel;
        newMenu.ActionReplaceLevel = ActionReplaceLevel;
        newMenu.CurrentSelection = CurrentSelection;
        newMenu.FuncValueLookup = FuncValueLookup;

        return newMenu;
    }

    public void AddMenu(Menu childMenu)
    {
        if (ChildMenus == null)
        {
            ChildMenus = new List<Menu>();
        }

        childMenu.Parent = this;
        ChildMenus.Add(childMenu);
    }

    public string GetName()
    {
        if (Name == null)
        {
            return "";
        }
        else if (NameReplaceLevel <= 0)
        {
            if (FuncValueLookup != null && ValueKey != null && ValueKey.Length > 0)
            {
                return Name + "   ( " + FuncValueLookup(ValueKey) + " )";
            }
            else
            {
                return Name;
            }
        }
        else
        {
            string mnName = Name;

            string mnValue = null;
            if (FuncValueLookup != null && ValueKey != null && ValueKey.Length > 0)
            {
                mnValue = ValueKey;
            }

            if ((NameReplaceLevel & 1) > 0)
            {
                mnName = mnName.Replace("{index}", (CurrentSelection + 1).ToString());
                if (mnValue != null)
                {
                    mnValue = mnValue.Replace("{index}", (CurrentSelection + 1).ToString());
                }
            }
            if ((NameReplaceLevel & 2) > 0)
            {
                if (RepeatList == null || RepeatList.Count == 0)
                {
                    mnName = mnName.Replace("{name}", "");
                    if (mnValue != null)
                    {
                        mnValue = mnValue.Replace("{name}", "");
                    }
                }
                else
                {
                    int index = Math.Max(Math.Min(CurrentSelection, RepeatList.Count - 1), 0);
                    mnName = mnName.Replace("{name}", RepeatList[index].ToString());
                    if (mnValue != null)
                    {
                        mnValue = mnValue.Replace("{name}", RepeatList[index].ToString());
                    }
                }
            }
            if ((NameReplaceLevel & 4) > 0)
            {
                mnName = mnName.Replace("{pindex}", (Parent == null ? "" : (Parent.CurrentSelection + 1).ToString()));
                if (mnValue != null)
                {
                    mnValue = mnValue.Replace("{pindex}", (Parent == null ? "" : (Parent.CurrentSelection + 1).ToString()));
                }
            }

            if (mnValue != null)
            {
                mnName += "   ( " + FuncValueLookup(mnValue) + " )";
            }

            return mnName;
        }
    }

    public string GetAction()
    {
        if (Action == null)
        {
            return "";
        }
        else if (ActionReplaceLevel <= 0)
        {
            return Action;
        }
        else
        {
            string mnAction = Action;
            if (mnAction == null)
            {
                return null;
            }

            if ((ActionReplaceLevel & 1) > 0)
            {
                mnAction = mnAction.Replace("{index}", (CurrentSelection + 1).ToString());
            }
            if ((ActionReplaceLevel & 2) > 0)
            {
                if (RepeatList == null || RepeatList.Count == 0)
                {
                    mnAction = mnAction.Replace("{name}", "");
                }
                else
                {
                    int index = Math.Max(Math.Min(CurrentSelection, RepeatList.Count - 1), 0);
                    mnAction = mnAction.Replace("{name}", RepeatList[index].ToString());
                }
            }
            if ((ActionReplaceLevel & 4) > 0)
            {
                mnAction = mnAction.Replace("{pindex}", (Parent == null ? "" : (Parent.CurrentSelection + 1).ToString()));
            }

            return mnAction;
        }
    }

    public string GetTitle()
    {
        if (Title == null)
        {
            return GetName();
        }
        else if (NameReplaceLevel <= 0)
        {
            return Title;
        }
        else
        {
            string mnTitle = Title;

            if ((NameReplaceLevel & 1) > 0)
            {
                mnTitle = mnTitle.Replace("{index}", (CurrentSelection + 1).ToString());
            }
            if ((NameReplaceLevel & 2) > 0)
            {
                if (RepeatList == null || RepeatList.Count == 0)
                {
                    mnTitle = mnTitle.Replace("{name}", "");
                }
                else
                {
                    int index = Math.Max(Math.Min(CurrentSelection, RepeatList.Count - 1), 0);
                    mnTitle = mnTitle.Replace("{name}", RepeatList[index].ToString());
                }
            }
            if ((NameReplaceLevel & 4) > 0)
            {
                mnTitle = mnTitle.Replace("{pindex}", (Parent == null ? "" : (Parent.CurrentSelection + 1).ToString()));
            }

            return mnTitle;
        }
    }
}

public class LayoutPanel
{
    public IMyGridTerminalSystem GridTerminalSystem { get; set; }
    public IMyGridProgramRuntimeInfo Runtime { get; set; }
    public IMyTerminalBlock ReferenceBlock { get; set; }
    public IMyCubeGrid CubeGrid { get; set; }

    public int PanelHorizontalCount { get; set; }
    public int PanelVerticalCount { get; set; }

    public bool bPaintDensity = DEF_PAINT_DENSITY;
    public bool bPaintTerminalBlocks = DEF_PAINT_TERMINAL_BLOCKS;
    public bool bPaintAlertBlocks = DEF_PAINT_ALERT_BLOCKS;

    public bool bBlinkForAlert = DEF_BLINK_FOR_ALERT;
    public bool bUseRelativeDensity = DEF_USE_RELATIVE_DENSITY;
    public bool bIncludeTerminalDensity = DEF_INCLUDE_TERMINAL_DENSITY;
    public bool bSmoothEdges = DEF_SMOOTH_EDGES;

    public bool bPaintCrossForAlert = DEF_PAINT_CROSS_FOR_ALERT;

    public bool bPaintLocationMarker = DEF_PAINT_LOCATION_MARKER;

    public byte alertMask = DEF_ALERT_MASK;

    public int pixelSize = DEF_PIXEL_SIZE;

    public float highDensityThreshold = HIGH_DENSITY_THRESHOLD;
    public float midDensityThreshold = MID_DENSITY_THRESHOLD;
    public float lowDensityThreshold = LOW_DENSITY_THRESHOLD;

    public IMyTextPanel Panel { get; set; }
    public IMyTextPanel[][] Panels { get; set; }
    public bool multiplePanels;

    public int grAxX, grAxY, grStX, grEndX, grStY, grEndY, grStZ, grEndZ, grWhX, grWhY, grWhZ;

    public int axWhX, axWhY, axWhZ;

    public byte drawLeftMask, drawRightMask, drawTopMask, drawBottomMask;

    public Base6Directions.Axis grAxXDir, grAxYDir, grAxZDir;
    public bool grFlipX, grFlipY, grFlipZ;
    public Vector3I grMin, grMax;
    public int grOffsetX, grOffsetY;

    public char[][] structBase;
    public Vector4I[][] structDensity;
    public int scWhX, scWhY, scOffX, scOffY, scSzX, scSzY, scArea;
    public float boxAspect;

    public HashSet<Vector2I> drawnPositions;
    public Dictionary<Vector4I, int> priorityDrawn;

    public char[][] renderData;

    public bool isBlinkFrame;

    public string prevDisplayParam;

    public Dictionary<string, string> config;

    public LayoutPanel(IMyGridTerminalSystem gridTerminalSystem, IMyGridProgramRuntimeInfo runtime, IMyTerminalBlock referenceBlock)
    {
        GridTerminalSystem = gridTerminalSystem;
        Runtime = runtime;
        ReferenceBlock = referenceBlock;
        CubeGrid = referenceBlock.CubeGrid;

        prevDisplayParam = "";

        config = new Dictionary<string, string>();
    }

    public override string ToString()
    {
        return (Panel == null ? "" : Panel.CustomName);
    }

    /*----- Rendering Methods -----*/

    public void Render()
    {
        if (multiplePanels)
        {
            int singleWidthX = renderData[0].Length / PanelHorizontalCount;
            int singleWidthY = renderData.Length / PanelVerticalCount;

            for (int y = 0; y < PanelVerticalCount; y++)
            {
                for (int x = 0; x < PanelHorizontalCount; x++)
                {
                    int startY = y * singleWidthY;
                    int endY = startY + singleWidthY;
                    int startX = x * singleWidthX;

                    StringBuilder sb = new StringBuilder((singleWidthX * singleWidthY) + 1);

                    if (bOptimizedRender)
                    {
                        if (startY < scOffY)
                        {
                            for (int i = startY; i < scOffY; i++)
                            {
                                sb.Append('\n');
                            }

                            startY = scOffY;
                        }

                        if (endY > renderData.Length - scOffY)
                        {
                            endY = renderData.Length - scOffY;
                        }

                        int drawWidthX;
                        if (startX + singleWidthX > renderData[0].Length - scOffX)
                        {
                            drawWidthX = Math.Max(renderData[0].Length - scOffX - startX, 0);
                        }
                        else
                        {
                            drawWidthX = singleWidthX;
                        }

                        for (int i = startY; i < endY; i++)
                        {
                            sb.Append(renderData[i], startX, drawWidthX).Append('\n');
                        }
                    }
                    else
                    {
                        for (int i = startY; i < endY; i++)
                        {
                            sb.Append(renderData[i], startX, singleWidthX).Append('\n');
                        }
                    }

                    Panels[y][x].WritePublicText(sb);
                }
            }
        }
        else
        {
            StringBuilder sb = new StringBuilder((renderData[0].Length * renderData.Length) + 1);

            if (bOptimizedRender)
            {
                for (int i = 0; i < scOffY; i++)
                {
                    sb.Append('\n');
                }

                for (int i = scOffY; i < renderData.Length - scOffY; i++)
                {
                    sb.Append(renderData[i], 0, renderData[0].Length - scOffX).Append('\n');
                }
            }
            else
            {
                for (int i = 0; i < renderData.Length; i++)
                {
                    sb.Append(renderData[i]).Append('\n');
                }
            }

            Panel.WritePublicText(sb);
        }
    }

    /*----- Initialization Methods -----*/

    public void InitDisplayPanel()
    {
        Panel.SetValue("FontSize", 0.1f * pixelSize);
        Panel.SetValue("FontColor", new Color(TEXT_COLOR_R, TEXT_COLOR_G, TEXT_COLOR_B));
        Panel.SetValue("Font", fontMonospace);

        Panel.ShowTextureOnScreen();
        Panel.ShowPublicTextOnScreen();

        Vector3I left, down;
        GetTileVectors(out left, out down);
        if (left.Length() != down.Length())
        {
            scWhX = WIDE_LCD_MAX_RESOLUTION_X / pixelSize;
            scWhY = WIDE_LCD_MAX_RESOLUTION_Y / pixelSize;
        }
        else
        {
            scWhX = TEXT_MAX_RESOLUTION_X / pixelSize;
            scWhY = TEXT_MAX_RESOLUTION_Y / pixelSize;
        }

        multiplePanels = false;
    }

    public void InitDisplayPanels()
    {
        if (PanelHorizontalCount == 1 && PanelVerticalCount == 1)
        {
            Panels = null;
            InitDisplayPanel();

            return;
        }

        Vector3I pnLeft, pnDown;
        GetTileVectors(out pnLeft, out pnDown);

        bool truncatePanels = false;
        int maxArea = 0;
        int mkX = 0;
        int mkY = 0;

        Panels = new IMyTextPanel[PanelVerticalCount][];
        for (int y = 0; y < PanelVerticalCount; y++)
        {
            Panels[y] = new IMyTextPanel[PanelHorizontalCount];
            for (int x = 0; x < PanelHorizontalCount; x++)
            {
                IMySlimBlock slimBlock = CubeGrid.GetCubeBlock(Panel.Position + (x * pnLeft) + (y * pnDown));

                IMyTextPanel panel = (slimBlock == null ? null : slimBlock.FatBlock as IMyTextPanel);
                if (panel == null)
                {
                    if (x == 0)
                    {
                        PanelVerticalCount = y;
                    }
                    truncatePanels = true;
                    break;
                }

                int area = (x + 1) * (y + 1);
                if (area > maxArea)
                {
                    maxArea = area;
                    mkX = x;
                    mkY = y;
                }

                Panels[y][x] = panel;

                Panels[y][x].SetValue("FontSize", 0.1f * pixelSize);
                Panels[y][x].SetValue("FontColor", new Color(TEXT_COLOR_R, TEXT_COLOR_G, TEXT_COLOR_B));
                Panels[y][x].SetValue("Font", fontMonospace);

                Panels[y][x].ShowTextureOnScreen();
                Panels[y][x].ShowPublicTextOnScreen();
            }
        }

        if (truncatePanels)
        {
            PanelHorizontalCount = mkX + 1;
            PanelVerticalCount = mkY + 1;

            if (PanelHorizontalCount == 1 && PanelVerticalCount == 1)
            {
                Panels = null;
                InitDisplayPanel();

                return;
            }

            IMyTextPanel[][] remainingPanels = new IMyTextPanel[PanelVerticalCount][];
            for (int y = 0; y < PanelVerticalCount; y++)
            {
                remainingPanels[y] = new IMyTextPanel[PanelHorizontalCount];
                for (int x = 0; x < PanelHorizontalCount; x++)
                {
                    remainingPanels[y][x] = Panels[y][x];
                }
            }
            Panels = remainingPanels;
        }

        if (pnLeft.Length() != pnDown.Length())
        {
            scWhX = WIDE_LCD_MAX_RESOLUTION_X / pixelSize * PanelHorizontalCount;
            scWhY = WIDE_LCD_MAX_RESOLUTION_Y / pixelSize * PanelVerticalCount;
        }
        else
        {
            scWhX = TEXT_MAX_RESOLUTION_X / pixelSize * PanelHorizontalCount;
            scWhY = TEXT_MAX_RESOLUTION_Y / pixelSize * PanelVerticalCount;
        }

        multiplePanels = true;
    }

    public void GetTileVectors(out Vector3I pnLeft, out Vector3I pnDown)
    {
        pnLeft = -Base6Directions.GetIntVector(Panel.Orientation.Left);
        pnDown = -Base6Directions.GetIntVector(Panel.Orientation.Up);

        Vector3I pnSize = Panel.Max - Panel.Min + new Vector3I(1, 1, 1);
        pnLeft = new Vector3I(pnLeft.X * pnSize.X, pnLeft.Y * pnSize.Y, pnLeft.Z * pnSize.Z);
        pnDown = new Vector3I(pnDown.X * pnSize.X, pnDown.Y * pnSize.Y, pnDown.Z * pnSize.Z);
    }

    public void UpdatePixelSize()
    {
        if (Panel.Min != Panel.Max)
        {
            scWhX = WIDE_LCD_MAX_RESOLUTION_X / pixelSize * PanelHorizontalCount;
            scWhY = WIDE_LCD_MAX_RESOLUTION_Y / pixelSize * PanelVerticalCount;
        }
        else
        {
            scWhX = TEXT_MAX_RESOLUTION_X / pixelSize * PanelHorizontalCount;
            scWhY = TEXT_MAX_RESOLUTION_Y / pixelSize * PanelVerticalCount;
        }

        if (PanelHorizontalCount == 1 && PanelVerticalCount == 1)
        {
            Panel.SetValue("FontSize", 0.1f * pixelSize);
        }
        else
        {
            for (int y = 0; y < PanelVerticalCount; y++)
            {
                for (int x = 0; x < PanelHorizontalCount; x++)
                {
                    Panels[y][x].SetValue("FontSize", 0.1f * pixelSize);
                }
            }
        }
    }

    public bool DisplayParametersChanged()
    {
        return !Panel.GetPublicTitle().Equals(prevDisplayParam);
    }

    public void InitDisplayParameters()
    {
        grAxX = -1;
        grAxY = -3;
        grStX = CubeGrid.Min.X;
        grEndX = CubeGrid.Max.X;
        grStY = CubeGrid.Min.Y;
        grEndY = CubeGrid.Max.Y;
        grStZ = CubeGrid.Min.Z;
        grEndZ = CubeGrid.Max.Z;
        grWhX = grEndX - grStX + 1;
        grWhY = grEndY - grStY + 1;
        grWhZ = grEndZ - grStZ + 1;

        ProcessDisplayParameters();

        if (grAxX == -1 || grAxY == -3)
        {
            InitDefaultAxisView();
        }

        if (prevDisplayParam.Length == 0 || prevDisplayParam.Trim().ToUpper().Equals("PUBLIC TITLE"))
        {
            WriteDisplayParameters();
        }
    }

    public void InitScreenParameters()
    {
        GetAxisVariables(grAxX, out axWhX, out grAxXDir);
        GetAxisVariables(grAxY, out axWhY, out grAxYDir);

        grFlipX = (grAxX % 2 != 0);
        grFlipY = (grAxY % 2 != 0);

        int axisZ = (3 - (grAxX / 2) - (grAxY / 2)) * 2;
        GetAxisVariables(axisZ, out axWhZ, out grAxZDir);

        Base6Directions.Direction directionZ = Base6Directions.GetCross(GetAxisDirection(grAxXDir, grFlipX), GetAxisDirection(grAxYDir, grFlipY));
        grFlipZ = (directionZ == Base6Directions.Direction.Right || directionZ == Base6Directions.Direction.Down || directionZ == Base6Directions.Direction.Backward);

        if (scWhX - axWhX - 1 < 0)
        {
            TruncateAxisX(axWhX - scWhX + 1);
        }

        if (scWhY - axWhY - 1 < 0)
        {
            TruncateAxisY(axWhY - scWhY + 1);
        }

        scSzX = (scWhX - axWhX - 1) / axWhX;
        scSzY = (scWhY - axWhY - 1) / axWhY;

        int maxScreenBlockSize = Math.Min(scSzX, scSzY);
        scSzX = maxScreenBlockSize;
        scSzY = maxScreenBlockSize;

        boxAspect = (scSzX == scSzY ? 1f : (float)(scSzY + 1) / (scSzX + 1));

        scArea = scSzX * scSzY;

        scOffX = (scWhX - (scSzX * axWhX) - axWhX - 1) / 2;
        scOffY = (scWhY - (scSzY * axWhY) - axWhY - 1) / 2;

        structDensity = new Vector4I[axWhY][];
        for (int i = 0; i < axWhY; i++)
        {
            structDensity[i] = new Vector4I[axWhX];
        }

        grMin = new Vector3I(grStX, grStY, grStZ);
        grMax = new Vector3I(grEndX, grEndY, grEndZ);

        grOffsetX = (grFlipX ? grMax.AxisValue(grAxXDir) : grMin.AxisValue(grAxXDir));
        grOffsetY = (grFlipY ? grMax.AxisValue(grAxYDir) : grMin.AxisValue(grAxYDir));

        drawLeftMask = ADJACENT_MASKS[grAxX];
        drawRightMask = ADJACENT_MASKS[((grAxX / 2) * 2) + 1 - (grAxX % 2)];
        drawTopMask = ADJACENT_MASKS[grAxY];
        drawBottomMask = ADJACENT_MASKS[((grAxY / 2) * 2) + 1 - (grAxY % 2)];
    }

    public void InitDefaultAxisView()
    {
        IMyTerminalBlock cockpit = null;
        IMyTerminalBlock mainCockpit = null;
        IMyTerminalBlock remote = null;

        List<IMyTerminalBlock> blks = new List<IMyTerminalBlock>();
        GridTerminalSystem.GetBlocksOfType<IMyCockpit>(blks);
        for (int i = 0; i < blks.Count; i++)
        {
            if (IsValidCockpit(blks[i]))
            {
                cockpit = blks[i];
                if (cockpit.GetValueBool("MainCockpit"))
                {
                    mainCockpit = cockpit;
                    break;
                }
            }
        }
        if (mainCockpit == null)
        {
            blks = new List<IMyTerminalBlock>();
            GridTerminalSystem.GetBlocksOfType<IMyRemoteControl>(blks);
            for (int i = 0; i < blks.Count; i++)
            {
                if (blks[i].CubeGrid == CubeGrid)
                {
                    remote = blks[i];
                    if (remote.GetValueBool("MainCockpit"))
                    {
                        mainCockpit = remote;
                        break;
                    }
                }
            }
        }
        if (mainCockpit == null)
        {
            mainCockpit = (cockpit == null ? remote : cockpit);
        }

        string panelName = Panel.CustomName.ToUpper();
        MatrixD matrix = (mainCockpit == null ? CubeGrid.WorldMatrix : mainCockpit.WorldMatrix);
        Vector3D vecX, vecY;
        Base6Directions.Direction dirX, dirY;
        if (panelName.Contains("SIDE"))
        {
            vecX = matrix.Backward;
            vecY = matrix.Down;
        }
        else if (panelName.Contains("FRONT"))
        {
            vecX = matrix.Left;
            vecY = matrix.Down;
        }
        else
        {
            vecX = matrix.Backward;
            vecY = matrix.Left;
        }
        dirX = CubeGrid.WorldMatrix.GetClosestDirection(vecX);
        dirY = CubeGrid.WorldMatrix.GetClosestDirection(vecY);

        grAxX = GetGridAxis(dirX);
        grAxY = GetGridAxis(dirY);
    }

    public bool IsValidCockpit(IMyTerminalBlock blk)
    {
        if (blk.CubeGrid == CubeGrid)
        {
            string subTypeId = blk.BlockDefinition.SubtypeId;
            return (subTypeId == null || !subTypeId.Contains("Passenger"));
        }
        return false;
    }

    public int GetGridAxis(Base6Directions.Direction dir)
    {
        switch (dir)
        {
            case Base6Directions.Direction.Forward:
                return 5;
            case Base6Directions.Direction.Backward:
                return 4;
            case Base6Directions.Direction.Left:
                return 1;
            case Base6Directions.Direction.Right:
                return 0;
            case Base6Directions.Direction.Up:
                return 3;
            default:
                return 2;
        }
    }

    public void GetAxisVariables(int axis, out int width, out Base6Directions.Axis dir)
    {
        switch (axis)
        {
            case 0:
            case 1:
                width = grWhX;
                dir = Base6Directions.Axis.LeftRight;
                return;
            case 2:
            case 3:
                width = grWhY;
                dir = Base6Directions.Axis.UpDown;
                return;
            case 4:
            case 5:
                width = grWhZ;
                dir = Base6Directions.Axis.ForwardBackward;
                return;
        }

        width = grWhX;
        dir = Base6Directions.Axis.LeftRight;
    }

    Base6Directions.Direction GetAxisDirection(Base6Directions.Axis axis, bool isReverse)
    {
        switch (axis)
        {
            case Base6Directions.Axis.LeftRight:
                return (isReverse ? Base6Directions.Direction.Left : Base6Directions.Direction.Right);
            case Base6Directions.Axis.UpDown:
                return (isReverse ? Base6Directions.Direction.Up : Base6Directions.Direction.Down);
            case Base6Directions.Axis.ForwardBackward:
                return (isReverse ? Base6Directions.Direction.Forward : Base6Directions.Direction.Backward);
            default:
                return (isReverse ? Base6Directions.Direction.Left : Base6Directions.Direction.Right);
        }
    }

    public void TruncateAxisX(int amount)
    {
        axWhX -= amount;
        TruncateAxis(grAxX, axWhX, amount);
    }

    public void TruncateAxisY(int amount)
    {
        axWhY -= amount;
        TruncateAxis(grAxY, axWhY, amount);
    }

    public void TruncateAxis(int axis, int width, int amount)
    {
        bool reverse = (axis % 2 != 0);
        switch (axis)
        {
            case 0:
            case 1:
                grWhX = width;
                if (reverse)
                {
                    grStX += amount;
                }
                else
                {
                    grEndX -= amount;
                }
                break;
            case 2:
            case 3:
                grWhY = width;
                if (reverse)
                {
                    grStY += amount;
                }
                else
                {
                    grEndY -= amount;
                }
                break;
            case 4:
            case 5:
                grWhZ = width;
                if (reverse)
                {
                    grStZ += amount;
                }
                else
                {
                    grEndZ -= amount;
                }
                break;
        }
    }

    /*----- Reset Methods -----*/

    public void ResetGridWidth()
    {
        grWhX = grEndX - grStX + 1;
        grWhY = grEndY - grStY + 1;
        grWhZ = grEndZ - grStZ + 1;
    }

    public void ResetStructureBase()
    {
        structBase = new char[scWhY][];
        for (int i = 0; i < scWhY; i++)
        {
            structBase[i] = new char[scWhX];
            structBase[i] = new string(chrBackground, scWhX).ToCharArray();
        }
    }

    public void ResetRenderData()
    {
        renderData = new char[structBase.Length][];
        for (int i = 0; i < scWhY; i++)
        {
            renderData[i] = new char[structBase[i].Length];
            Array.Copy(structBase[i], renderData[i], structBase[i].Length);
        }
    }

    /*----- Parameter Processing Methods -----*/

    public bool ProcessDisplayParameters(string paramLine = null)
    {
        paramLine = (paramLine == null ? Panel.GetPublicTitle().Trim().ToUpper() : paramLine);
        prevDisplayParam = paramLine;

        string[] keyValues = paramLine.Split(',');

        int tmGrAxX = grAxX;
        int tmGrAxY = grAxY;
        int tmGrStX = grStX;
        int tmGrEndX = grEndX;
        int tmGrStY = grStY;
        int tmGrEndY = grEndY;
        int tmGrStZ = grStZ;
        int tmGrEndZ = grEndZ;

        bool pixelSizeChanged = false;

        for (int i = 0; i < keyValues.Length; i++)
        {
            string[] tk = keyValues[i].Trim().Split(':');
            if (tk.Length > 0)
            {
                int val;
                string cmd = tk[0].Trim();
                bool two = (tk.Length >= 2);
                if (cmd.Equals("A_X") && two)
                {
                    val = ParseAxisToken(tk[1]);
                    if (val > -1)
                    {
                        tmGrAxX = Math.Min(5, Math.Max(0, val));
                    }
                }
                else if (cmd.Equals("A_Y") && two)
                {
                    val = ParseAxisToken(tk[1]);
                    if (val > -1)
                    {
                        tmGrAxY = Math.Min(5, Math.Max(0, val));
                    }
                }
                else if (cmd.Equals("X") && two)
                {
                    if (Int32.TryParse(tk[1], out val))
                    {
                        tmGrStX = tmGrEndX = val;
                    }
                }
                else if (cmd.Equals("X1") && two)
                {
                    if (Int32.TryParse(tk[1], out val))
                    {
                        tmGrStX = val;
                    }
                }
                else if (cmd.Equals("X2") && two)
                {
                    if (Int32.TryParse(tk[1], out val))
                    {
                        tmGrEndX = val;
                    }
                }
                else if (cmd.Equals("Y") && two)
                {
                    if (Int32.TryParse(tk[1], out val))
                    {
                        tmGrStY = tmGrEndY = val;
                    }
                }
                else if (cmd.Equals("Y1") && two)
                {
                    if (Int32.TryParse(tk[1], out val))
                    {
                        tmGrStY = val;
                    }
                }
                else if (cmd.Equals("Y2") && two)
                {
                    if (Int32.TryParse(tk[1], out val))
                    {
                        tmGrEndY = val;
                    }
                }
                else if (cmd.Equals("Z") && two)
                {
                    if (Int32.TryParse(tk[1], out val))
                    {
                        tmGrStZ = tmGrEndZ = val;
                    }
                }
                else if (cmd.Equals("Z1") && two)
                {
                    if (Int32.TryParse(tk[1], out val))
                    {
                        tmGrStZ = val;
                    }
                }
                else if (cmd.Equals("Z2") && two)
                {
                    if (Int32.TryParse(tk[1], out val))
                    {
                        tmGrEndZ = val;
                    }
                }
                else if (FLAG_CMD.ContainsKey(cmd))
                {
                    int[] opt = FLAG_CMD[cmd];
                    bool yes = (opt[1] == 0);
                    switch (opt[0])
                    {
                        case 0:
                            bPaintDensity = yes;
                            break;
                        case 1:
                            bUseRelativeDensity = yes;
                            break;
                        case 2:
                            bIncludeTerminalDensity = yes;
                            break;
                        case 3:
                            bSmoothEdges = yes;
                            break;
                        case 4:
                            bPaintLocationMarker = yes;
                            break;
                        case 5:
                            bPaintTerminalBlocks = yes;
                            break;
                        case 6:
                            bPaintAlertBlocks = yes;
                            break;
                        case 7:
                            bBlinkForAlert = yes;
                            break;
                        case 8:
                            bPaintCrossForAlert = yes;
                            break;
                    }
                }
                else if (cmd.Equals("AL_MASK"))
                {
                    byte bvalue;
                    if (byte.TryParse(tk[1], out bvalue))
                    {
                        alertMask = bvalue;
                    }
                }
                else if (cmd.Equals("THR_HDENSE"))
                {
                    float fvalue;
                    if (float.TryParse(tk[1], out fvalue))
                    {
                        highDensityThreshold = Math.Min(1f, Math.Max(0f, (float)Math.Round(fvalue, 2)));
                    }
                }
                else if (cmd.Equals("THR_MDENSE"))
                {
                    float fvalue;
                    if (float.TryParse(tk[1], out fvalue))
                    {
                        midDensityThreshold = Math.Min(1f, Math.Max(0f, (float)Math.Round(fvalue, 2)));
                    }
                }
                else if (cmd.Equals("THR_LDENSE"))
                {
                    float fvalue;
                    if (float.TryParse(tk[1], out fvalue))
                    {
                        lowDensityThreshold = Math.Min(1f, Math.Max(0f, (float)Math.Round(fvalue, 2)));
                    }
                }
                else if (cmd.Equals("PIXEL"))
                {
                    int prevPixelSize = pixelSize;
                    if (Int32.TryParse(tk[1], out val))
                    {
                        pixelSize = Math.Min(10, Math.Max(1, val));
                    }
                    pixelSizeChanged = (prevPixelSize != pixelSize);
                }
            }
        }

        if ((tmGrAxX / 2) == (tmGrAxY / 2))
        {
            return false;
        }

        if (tmGrEndX < tmGrStX)
        {
            int tmp = tmGrStX;
            tmGrStX = tmGrEndX;
            tmGrEndX = tmp;
        }

        if (tmGrEndY < tmGrStY)
        {
            int tmp = tmGrStY;
            tmGrStY = tmGrEndY;
            tmGrEndY = tmp;
        }

        if (tmGrEndZ < tmGrStZ)
        {
            int tmp = tmGrStZ;
            tmGrStZ = tmGrEndZ;
            tmGrEndZ = tmp;
        }

        tmGrStX = Math.Min(Math.Max(tmGrStX, CubeGrid.Min.X), CubeGrid.Max.X);
        tmGrEndX = Math.Min(Math.Max(tmGrEndX, CubeGrid.Min.X), CubeGrid.Max.X);

        tmGrStY = Math.Min(Math.Max(tmGrStY, CubeGrid.Min.Y), CubeGrid.Max.Y);
        tmGrEndY = Math.Min(Math.Max(tmGrEndY, CubeGrid.Min.Y), CubeGrid.Max.Y);

        tmGrStZ = Math.Min(Math.Max(tmGrStZ, CubeGrid.Min.Z), CubeGrid.Max.Z);
        tmGrEndZ = Math.Min(Math.Max(tmGrEndZ, CubeGrid.Min.Z), CubeGrid.Max.Z);

        if (pixelSizeChanged)
        {
            UpdatePixelSize();
        }

        if (tmGrAxX != grAxX || tmGrAxY != grAxY || tmGrStX != grStX || tmGrEndX != grEndX || tmGrStY != grStY || tmGrEndY != grEndY || tmGrStZ != grStZ || tmGrEndZ != grEndZ || pixelSizeChanged)
        {
            grAxX = tmGrAxX;
            grAxY = tmGrAxY;
            grStX = tmGrStX;
            grEndX = tmGrEndX;
            grStY = tmGrStY;
            grEndY = tmGrEndY;
            grStZ = tmGrStZ;
            grEndZ = tmGrEndZ;
            grWhX = grEndX - grStX + 1;
            grWhY = grEndY - grStY + 1;
            grWhZ = grEndZ - grStZ + 1;

            return true;
        }
        else
        {
            return false;
        }
    }

    public int ParseAxisToken(string token)
    {
        switch (token)
        {
            case "X":
            case "+X":
                return 0;
            case "-X":
                return 1;
            case "Y":
            case "+Y":
                return 2;
            case "-Y":
                return 3;
            case "Z":
            case "+Z":
                return 4;
            case "-Z":
                return 5;
            default:
                return -1;
        }
    }

    public string RetrieveAxisParameter(int gridAxis)
    {
        switch (gridAxis)
        {
            case 0:
                return "X";
            case 1:
                return "-X";
            case 2:
                return "Y";
            case 3:
                return "-Y";
            case 4:
                return "Z";
            case 5:
                return "-Z";
            default:
                return "";
        }
    }

    public void WriteDisplayParameters()
    {
        StringBuilder sb = new StringBuilder();
        sb.Append("A_X:").Append(RetrieveAxisParameter(grAxX)).Append(',');
        sb.Append("A_Y:").Append(RetrieveAxisParameter(grAxY)).Append(',');
        sb.Append("X1:").Append(grStX).Append(',');
        sb.Append("X2:").Append(grEndX).Append(',');
        sb.Append("Y1:").Append(grStY).Append(',');
        sb.Append("Y2:").Append(grEndY).Append(',');
        sb.Append("Z1:").Append(grStZ).Append(',');
        sb.Append("Z2:").Append(grEndZ).Append(',');
        sb.Append(bSmoothEdges ? "SMOOTH" : "NO_SMOOTH").Append(',');
        sb.Append(bUseRelativeDensity ? "RELDENSE" : "NO_RELDENSE").Append(',');
        sb.Append(bIncludeTerminalDensity ? "TERMDENSE" : "NO_TERMDENSE").Append(',');
        sb.Append(bPaintDensity ? "DENSE" : "NO_DENSE").Append(',');
        sb.Append(bPaintTerminalBlocks ? "TERM" : "NO_TERM").Append(',');
        sb.Append(bBlinkForAlert ? "BLINK" : "NO_BLINK").Append(',');
        sb.Append(bPaintAlertBlocks ? "ALERT" : "NO_ALERT").Append(',');
        sb.Append(bPaintCrossForAlert ? "CROSS" : "NO_CROSS").Append(',');
        sb.Append(bPaintLocationMarker ? "MARKER" : "NO_MARKER").Append(',');
        sb.Append("AL_MASK:").Append(alertMask).Append(',');
        sb.Append("THR_HDENSE:").Append(Math.Round(highDensityThreshold, 2)).Append(',');
        sb.Append("THR_MDENSE:").Append(Math.Round(midDensityThreshold, 2)).Append(',');
        sb.Append("THR_LDENSE:").Append(Math.Round(lowDensityThreshold, 2)).Append(',');
        sb.Append("PIXEL:").Append(pixelSize);

        Panel.WritePublicTitle(sb.ToString());
        prevDisplayParam = Panel.GetPublicTitle();
    }

    /*----- Misc Methods -----*/

    public string GetParameterDescription(string key)
    {
        switch (key)
        {
            case "DENSE":
                return (bPaintDensity ? "On" : "Off");
            case "RELDENSE":
                return (bUseRelativeDensity ? "On" : "Off");
            case "SMOOTH":
                return (bSmoothEdges ? "On" : "Off");
            case "TERM":
                return (bPaintTerminalBlocks ? "On" : "Off");
            case "ALERT":
                return (bPaintAlertBlocks ? "On" : "Off");
            case "AL_VENT":
                return ((alertMask & AL_VENT) > 0 ? "On" : "Off");
            case "AL_SENSOR":
                return ((alertMask & AL_SENSOR) > 0 ? "On" : "Off");
            case "AL_TURRET":
                return ((alertMask & AL_TURRET) > 0 ? "On" : "Off");
            case "AL_DOOR":
                return ((alertMask & AL_DOOR) > 0 ? "On" : "Off");
            case "AL_HACKED":
                return ((alertMask & AL_HACKED) > 0 ? "On" : "Off");
            case "BLINK":
                return (bBlinkForAlert ? "On" : "Off");
            case "BRIGHTNESS":
                return Math.Round(Panel.GetValueColor("FontColor").R / 2.55f, 0) + "%";
            case "IMAGE":
                return (!Panel.ShowText ? "On" : "Off");
            case "CROSS":
                return (bPaintCrossForAlert ? "On" : "Off");
            case "MARKER":
                return (bPaintLocationMarker ? "On" : "Off");
            default:
                return "";
        }
    }

    public void WriteAndReset()
    {
        WriteDisplayParameters();
        ResetStructureBase();
    }

    public void RefreshReferenceGrid()
    {
        CubeGrid = ReferenceBlock.CubeGrid;
    }

    public void LightenDisplay()
    {
        Color color = Panel.GetValueColor("FontColor");
        int value = Math.Min(color.R + 10, 255);

        if (multiplePanels)
        {
            for (int y = 0; y < PanelVerticalCount; y++)
            {
                for (int x = 0; x < PanelHorizontalCount; x++)
                {
                    Panels[y][x].SetValue("FontColor", new Color(value, value, value));
                }
            }
        }
        else
        {
            Panel.SetValue("FontColor", new Color(value, value, value));
        }
    }

    public void DarkenDisplay()
    {
        Color color = Panel.GetValueColor("FontColor");
        int value = Math.Max(color.R - 10, 0);

        if (multiplePanels)
        {
            for (int y = 0; y < PanelVerticalCount; y++)
            {
                for (int x = 0; x < PanelHorizontalCount; x++)
                {
                    Panels[y][x].SetValue("FontColor", new Color(value, value, value));
                }
            }
        }
        else
        {
            Panel.SetValue("FontColor", new Color(value, value, value));
        }
    }

    public void ToggleImageDisplay()
    {
        if (multiplePanels)
        {
            for (int y = 0; y < PanelVerticalCount; y++)
            {
                for (int x = 0; x < PanelHorizontalCount; x++)
                {
                    if (Panels[y][x].ShowText)
                    {
                        Panels[y][x].ShowTextureOnScreen();
                    }
                    else
                    {
                        Panels[y][x].ShowPublicTextOnScreen();
                    }
                }
            }
        }
        else
        {
            if (Panel.ShowText)
            {
                Panel.ShowTextureOnScreen();
            }
            else
            {
                Panel.ShowPublicTextOnScreen();
            }
        }
    }

    public void ShowLargeNumber(int number)
    {
        if (multiplePanels)
        {
            for (int y = 0; y < PanelVerticalCount; y++)
            {
                for (int x = 0; x < PanelHorizontalCount; x++)
                {
                    Panels[y][x].SetValue("FontSize", 10f);
                    Panels[y][x].WritePublicText(number.ToString());
                }
            }
        }
        else
        {
            Panel.SetValue("FontSize", 10f);
            Panel.WritePublicText(number.ToString());
        }
    }

    public void HideLargeNumber()
    {
        if (multiplePanels)
        {
            for (int y = 0; y < PanelVerticalCount; y++)
            {
                for (int x = 0; x < PanelHorizontalCount; x++)
                {
                    Panels[y][x].SetValue("FontSize", 0.1f * pixelSize);
                    Panels[y][x].WritePublicText("");
                }
            }
        }
        else
        {
            Panel.SetValue("FontSize", 0.1f * pixelSize);
            Panel.WritePublicText("");
        }
    }

    /*----- Computation Methods -----*/

    public void ComputeStructureDensity(KeyValuePair<Vector3I, byte> blk, bool isArmorBlock)
    {
        Vector3I pos = blk.Key;

        if (Vector3I.BoxContains(grMin, grMax, pos) && (bIncludeTerminalDensity || isArmorBlock))
        {
            int scPX, scPY;
            GetScreenPositions(ref pos, out scPX, out scPY);

            int depthPosZ = pos.AxisValue(grAxZDir);
            Vector4I density = structDensity[scPY][scPX];
            if (density.Z == 0)
            {
                structDensity[scPY][scPX] = new Vector4I(depthPosZ, depthPosZ, 1, 0);
            }
            else
            {
                structDensity[scPY][scPX] = new Vector4I(Math.Min(density.X, depthPosZ), Math.Max(density.Y, depthPosZ), density.Z + 1, density.W);
            }

            if (bSmoothEdges)
            {
                density = structDensity[scPY][scPX];
                structDensity[scPY][scPX] = new Vector4I(density.X, density.Y, density.Z, (byte)density.W | blk.Value);
            }
        }
    }

    /*----- Paint Methods -----*/

    public void PaintStructureBase(KeyValuePair<Vector3I, byte> blk)
    {
        Vector3I pos = blk.Key;

        if (Vector3I.BoxContains(grMin, grMax, pos))
        {
            int scPX, scPY;
            GetScreenPositions(ref pos, out scPX, out scPY);

            bool dwL = ((blk.Value & drawLeftMask) == 0);
            bool dwR = ((blk.Value & drawRightMask) == 0);
            bool dwT = ((blk.Value & drawTopMask) == 0);
            bool dwB = ((blk.Value & drawBottomMask) == 0);

            if (bSmoothEdges)
            {
                if (dwL && dwT && !dwR && !dwB)
                {
                    dwL = dwT = false;
                    PaintBackSlash(structBase, scPX, scPY, chrStructure);
                }
                else if (!dwL && dwT && dwR && !dwB)
                {
                    dwT = dwR = false;
                    PaintForwardSlash(structBase, scPX, scPY, chrStructure);
                }
                else if (!dwL && !dwT && dwR && dwB)
                {
                    dwR = dwB = false;
                    PaintBackSlash(structBase, scPX, scPY, chrStructure);
                }
                else if (dwL && !dwT && !dwR && dwB)
                {
                    dwB = dwL = false;
                    PaintForwardSlash(structBase, scPX, scPY, chrStructure);
                }
            }

            int mkX = 0;
            int mkY = 0;
            int mkX1 = 0;
            int mkY1 = 0;

            if (dwL || dwR || dwT)
            {
                mkY = scOffY + (scPY * (scSzY + 1));
            }
            if (dwB)
            {
                mkY1 = scOffY + ((scPY + 1) * (scSzY + 1));
            }
            if (dwL || dwT || dwB)
            {
                mkX = scOffX + (scPX * (scSzX + 1));
            }
            if (dwR)
            {
                mkX1 = scOffX + ((scPX + 1) * (scSzX + 1));
            }

            if (dwL)
            {
                for (int y = 0; y <= scSzY + 1; y++)
                {
                    structBase[mkY + y][mkX] = chrStructure;
                }
            }
            if (dwR)
            {
                for (int y = 0; y <= scSzY + 1; y++)
                {
                    structBase[mkY + y][mkX1] = chrStructure;
                }
            }
            if (dwT)
            {
                for (int x = 0; x <= scSzX + 1; x++)
                {
                    structBase[mkY][mkX + x] = chrStructure;
                }
            }
            if (dwB)
            {
                for (int x = 0; x <= scSzX + 1; x++)
                {
                    structBase[mkY1][mkX + x] = chrStructure;
                }
            }
        }
    }

    public int PaintStructureDensity(int index, int instLimit)
    {
        int totalIndex = axWhY * axWhX;

        while (index < totalIndex && Runtime.CurrentInstructionCount < instLimit)
        {
            int scPX = index % axWhX;
            int scPY = index / axWhX;

            Vector4I density = structDensity[scPY][scPX];
            if (density.Z > 0)
            {
                char chrPaint;
                int densityMax = (bUseRelativeDensity ? density.Y - density.X + 1 : axWhZ);
                if (density.Z >= Math.Ceiling(densityMax * highDensityThreshold))
                {
                    chrPaint = chrStructure;
                }
                else if (density.Z >= Math.Ceiling(densityMax * midDensityThreshold))
                {
                    chrPaint = chrDensity;
                }
                else if (density.Z >= Math.Ceiling(densityMax * lowDensityThreshold))
                {
                    chrPaint = chrLowDensity;
                }
                else
                {
                    index++;
                    continue;
                }

                if (bSmoothEdges)
                {
                    bool dwL = ((density.W & drawLeftMask) == 0);
                    bool dwR = ((density.W & drawRightMask) == 0);
                    bool dwT = ((density.W & drawTopMask) == 0);
                    bool dwB = ((density.W & drawBottomMask) == 0);

                    if (dwL && dwT && !dwR && !dwB)
                    {
                        PaintSolidBackBottomTriangle(structBase, scPX, scPY, chrPaint);
                    }
                    else if (!dwL && dwT && dwR && !dwB)
                    {
                        PaintSolidForwardBottomTriangle(structBase, scPX, scPY, chrPaint);
                    }
                    else if (!dwL && !dwT && dwR && dwB)
                    {
                        PaintSolidBackTopTriangle(structBase, scPX, scPY, chrPaint);
                    }
                    else if (dwL && !dwT && !dwR && dwB)
                    {
                        PaintSolidForwardTopTriangle(structBase, scPX, scPY, chrPaint);
                    }
                    else
                    {
                        PaintSolidBox(structBase, scPX, scPY, chrPaint);
                    }
                }
                else
                {
                    PaintSolidBox(structBase, scPX, scPY, chrPaint);
                }
            }

            index++;
        }

        return index;
    }

    public void PaintLocationMarker(char chrPaintEven, char chrPaintOdd)
    {
        int scPX = (grFlipX ? grOffsetX - Panel.Position.AxisValue(grAxXDir) : Panel.Position.AxisValue(grAxXDir) - grOffsetX);
        int scPY = (grFlipY ? grOffsetY - Panel.Position.AxisValue(grAxYDir) : Panel.Position.AxisValue(grAxYDir) - grOffsetY);

        if (scPX >= 0 && scPX < axWhX && scPY >= 0 && scPY < axWhY)
        {
            PaintLocationMarker(structBase, scPX, scPY, chrPaintEven, chrPaintOdd);
        }
    }

    public void PaintMissingStructure(KeyValuePair<Vector3I, byte> blk)
    {
        Vector3I pos = blk.Key;

        if (Vector3I.BoxContains(grMin, grMax, pos))
        {
            int scPX, scPY;
            GetScreenPositions(ref pos, out scPX, out scPY);

            if (bSmoothEdges)
            {
                bool dwL = ((blk.Value & drawLeftMask) == 0);
                bool dwR = ((blk.Value & drawRightMask) == 0);
                bool dwT = ((blk.Value & drawTopMask) == 0);
                bool dwB = ((blk.Value & drawBottomMask) == 0);

                int mkY = scOffY + (scPY * (scSzY + 1));
                int mkY1 = scOffY + ((scPY + 1) * (scSzY + 1));
                int mkX = scOffX + (scPX * (scSzX + 1));
                int mkX1 = scOffX + ((scPX + 1) * (scSzX + 1));

                if (dwL && dwT && !dwR && !dwB)
                {
                    PaintBackSlash(renderData, scPX, scPY, chrMissingBlock);
                    for (int y = 0; y <= scSzY + 1; y++)
                    {
                        renderData[mkY + y][mkX1] = chrMissingBlock;
                    }
                    for (int x = 0; x <= scSzX + 1; x++)
                    {
                        renderData[mkY1][mkX + x] = chrMissingBlock;
                    }
                }
                else if (!dwL && dwT && dwR && !dwB)
                {
                    PaintForwardSlash(renderData, scPX, scPY, chrMissingBlock);
                    for (int x = 0; x <= scSzX + 1; x++)
                    {
                        renderData[mkY1][mkX + x] = chrMissingBlock;
                    }
                    for (int y = 0; y <= scSzY + 1; y++)
                    {
                        renderData[mkY + y][mkX] = chrMissingBlock;
                    }
                }
                else if (!dwL && !dwT && dwR && dwB)
                {
                    PaintBackSlash(renderData, scPX, scPY, chrMissingBlock);
                    for (int y = 0; y <= scSzY + 1; y++)
                    {
                        renderData[mkY + y][mkX] = chrMissingBlock;
                    }
                    for (int x = 0; x <= scSzX + 1; x++)
                    {
                        renderData[mkY][mkX + x] = chrMissingBlock;
                    }
                }
                else if (dwL && !dwT && !dwR && dwB)
                {
                    PaintForwardSlash(renderData, scPX, scPY, chrMissingBlock);
                    for (int x = 0; x <= scSzX + 1; x++)
                    {
                        renderData[mkY][mkX + x] = chrMissingBlock;
                    }
                    for (int y = 0; y <= scSzY + 1; y++)
                    {
                        renderData[mkY + y][mkX1] = chrMissingBlock;
                    }
                }
                else
                {
                    Vector2I screenPosition = new Vector2I(scPX, scPY);
                    if (!drawnPositions.Contains(screenPosition))
                    {
                        PaintHollowBox(renderData, scPX, scPY, chrMissingBlock);
                        drawnPositions.Add(screenPosition);
                    }
                }
            }
            else
            {
                Vector2I screenPosition = new Vector2I(scPX, scPY);
                if (!drawnPositions.Contains(screenPosition))
                {
                    PaintHollowBox(renderData, scPX, scPY, chrMissingBlock);
                    drawnPositions.Add(screenPosition);
                }
            }
        }
    }

    public void PaintTerminalBlocks(IMyTerminalBlock blk, int drawLevel, char chrPaint, bool cross)
    {
        Vector3I pos = blk.Position;

        if (Vector3I.BoxContains(grMin, grMax, pos))
        {
            Vector3I min = blk.Min;
            Vector3I max = blk.Max;

            if (min == max)
            {
                int scPX, scPY;
                GetScreenPositions(ref pos, out scPX, out scPY);

                Vector4I boundary = new Vector4I(scPX, scPY, scPX, scPY);

                int takenPriority = priorityDrawn.GetValueOrDefault<Vector4I, int>(boundary, 0);

                if (drawLevel > takenPriority)
                {
                    if (cross)
                    {
                        PaintCrossBox(renderData, scPX, scPY, scPX, scPY, chrPaint);
                    }
                    else
                    {
                        PaintHollowBox(renderData, scPX, scPY, chrPaint);
                    }
                    priorityDrawn[boundary] = drawLevel;
                }
            }
            else
            {
                int scPX1, scPY1, scPX2, scPY2;
                GetScreenPositions(ref min, ref max, out scPX1, out scPY1, out scPX2, out scPY2);

                Vector4I boundary = new Vector4I(scPX1, scPY1, scPX2, scPY2);

                int takenPriority = priorityDrawn.GetValueOrDefault<Vector4I, int>(boundary, 0);

                if (drawLevel > takenPriority)
                {
                    if (cross)
                    {
                        PaintCrossBox(renderData, scPX1, scPY1, scPX2, scPY2, chrPaint);
                    }
                    else
                    {
                        PaintHollowBox(renderData, scPX1, scPY1, scPX2, scPY2, chrPaint);
                    }
                    priorityDrawn[boundary] = drawLevel;
                }
            }
        }
    }

    public void PaintWatchedBlocks(IMyTerminalBlock blk, Vector3I? detectedPosition, char chrPaint)
    {
        Vector3I pos = blk.Position;

        if (Vector3I.BoxContains(grMin, grMax, pos))
        {
            if (detectedPosition != null)
            {
                pos = detectedPosition.Value;

                int scPX, scPY;
                GetScreenPositions(ref pos, out scPX, out scPY);

                if (scPX >= 0 && scPX < axWhX && scPY >= 0 && scPY < axWhY)
                {
                    if (bPaintCrossForAlert)
                    {
                        PaintCrossBox(renderData, scPX, scPY, scPX, scPY, chrPaint);
                    }
                    else
                    {
                        PaintSolidBox(renderData, scPX, scPY, chrPaint);
                    }
                }
            }
            else
            {
                Vector3I min = blk.Min;
                Vector3I max = blk.Max;

                if (min == max)
                {
                    int scPX, scPY;
                    GetScreenPositions(ref pos, out scPX, out scPY);

                    if (bPaintCrossForAlert)
                    {
                        PaintCrossBox(renderData, scPX, scPY, scPX, scPY, chrPaint);
                    }
                    else
                    {
                        PaintSolidBox(renderData, scPX, scPY, chrPaint);
                    }
                }
                else
                {
                    int scPX1, scPY1, scPX2, scPY2;
                    GetScreenPositions(ref min, ref max, out scPX1, out scPY1, out scPX2, out scPY2);

                    if (bPaintCrossForAlert)
                    {
                        PaintCrossBox(renderData, scPX1, scPY1, scPX2, scPY2, chrPaint);
                    }
                    else
                    {
                        PaintSolidBox(renderData, scPX1, scPY1, scPX2, scPY2, chrPaint);
                    }
                }
            }
        }
    }

    public void GetScreenPositions(ref Vector3I position, out int scPX, out int scPY)
    {
        scPX = (grFlipX ? grOffsetX - position.AxisValue(grAxXDir) : position.AxisValue(grAxXDir) - grOffsetX);
        scPY = (grFlipY ? grOffsetY - position.AxisValue(grAxYDir) : position.AxisValue(grAxYDir) - grOffsetY);
    }

    public void GetScreenPositions(ref Vector3I min, ref Vector3I max, out int scPX1, out int scPY1, out int scPX2, out int scPY2)
    {
        scPX1 = (grFlipX ? grOffsetX - min.AxisValue(grAxXDir) : min.AxisValue(grAxXDir) - grOffsetX);
        scPY1 = (grFlipY ? grOffsetY - min.AxisValue(grAxYDir) : min.AxisValue(grAxYDir) - grOffsetY);
        scPX2 = (grFlipX ? grOffsetX - max.AxisValue(grAxXDir) : max.AxisValue(grAxXDir) - grOffsetX);
        scPY2 = (grFlipY ? grOffsetY - max.AxisValue(grAxYDir) : max.AxisValue(grAxYDir) - grOffsetY);

        if (scPX1 > scPX2)
        {
            int tmPX1 = scPX1;
            scPX1 = scPX2;
            scPX2 = tmPX1;
        }

        if (scPY1 > scPY2)
        {
            int tmPY1 = scPY1;
            scPY1 = scPY2;
            scPY2 = tmPY1;
        }

        scPX1 = Math.Max(scPX1, 0);
        scPY1 = Math.Max(scPY1, 0);
        scPX2 = Math.Min(scPX2, axWhX - 1);
        scPY2 = Math.Min(scPY2, axWhY - 1);
    }

    /*----- Shape Drawing Methods -----*/

    public void PaintForwardSlash(char[][] map, int pX, int pY, char code)
    {
        int mkX = scOffX + (pX * (scSzX + 1));
        int mkY = scOffY + (pY * (scSzY + 1));

        for (int x = 0; x <= scSzX + 1; x++)
        {
            int y = (scSzX == scSzY ? x : (int)Math.Round(x * boxAspect));
            map[mkY + y][mkX + x] = code;
        }
    }

    public void PaintBackSlash(char[][] map, int pX, int pY, char code)
    {
        int mkX = scOffX + (pX * (scSzX + 1));
        int mkY = scOffY + (pY * (scSzY + 1));

        for (int x = 0; x <= scSzX + 1; x++)
        {
            int y = scSzY + 1 - (scSzX == scSzY ? x : (int)Math.Round(x * boxAspect));
            map[mkY + y][mkX + x] = code;
        }
    }

    public void PaintSolidForwardTopTriangle(char[][] map, int pX, int pY, char code)
    {
        int mkX = scOffX + (pX * (scSzX + 1));
        int mkY = scOffY + (pY * (scSzY + 1));

        for (int x = 0; x <= scSzX + 1; x++)
        {
            int k = (scSzX == scSzY ? x : (int)Math.Round(x * boxAspect));
            for (int y = 0; y <= k; y++)
            {
                map[mkY + y][mkX + x] = code;
            }
        }
    }

    public void PaintSolidForwardBottomTriangle(char[][] map, int pX, int pY, char code)
    {
        int mkX = scOffX + (pX * (scSzX + 1));
        int mkY = scOffY + (pY * (scSzY + 1));

        for (int x = 0; x <= scSzX + 1; x++)
        {
            int k = (scSzX == scSzY ? x : (int)Math.Round(x * boxAspect));
            for (int y = k; y <= scSzY + 1; y++)
            {
                map[mkY + y][mkX + x] = code;
            }
        }
    }

    public void PaintSolidBackTopTriangle(char[][] map, int pX, int pY, char code)
    {
        int mkX = scOffX + (pX * (scSzX + 1));
        int mkY = scOffY + (pY * (scSzY + 1));

        for (int x = 0; x <= scSzX + 1; x++)
        {
            int k = scSzY + 1 - (scSzX == scSzY ? x : (int)Math.Round(x * boxAspect));
            for (int y = 0; y <= k; y++)
            {
                map[mkY + y][mkX + x] = code;
            }
        }
    }

    public void PaintSolidBackBottomTriangle(char[][] map, int pX, int pY, char code)
    {
        int mkX = scOffX + (pX * (scSzX + 1));
        int mkY = scOffY + (pY * (scSzY + 1));

        for (int x = 0; x <= scSzX + 1; x++)
        {
            int k = scSzY + 1 - (scSzX == scSzY ? x : (int)Math.Round(x * boxAspect));
            for (int y = k; y <= scSzY + 1; y++)
            {
                map[mkY + y][mkX + x] = code;
            }
        }
    }

    public void PaintSolidBox(char[][] map, int pX, int pY, char code)
    {
        int mkX = scOffX + (pX * (scSzX + 1));
        int mkY = scOffY + (pY * (scSzY + 1));

        for (int x = 0; x <= scSzX + 1; x++)
        {
            for (int y = 0; y <= scSzY + 1; y++)
            {
                map[mkY + y][mkX + x] = code;
            }
        }
    }

    public void PaintSolidBox(char[][] map, int pX1, int pY1, int pX2, int pY2, char code)
    {
        int mkX = scOffX + (pX1 * (scSzX + 1));
        int mkY = scOffY + (pY1 * (scSzY + 1));

        int maxX = (pX2 - pX1 + 1) * (scSzX + 1);
        int maxY = (pY2 - pY1 + 1) * (scSzY + 1);

        for (int x = 0; x <= maxX; x++)
        {
            for (int y = 0; y <= maxY; y++)
            {
                map[mkY + y][mkX + x] = code;
            }
        }
    }

    public void PaintCrossBox(char[][] map, int pX1, int pY1, int pX2, int pY2, char code)
    {
        int mkX = scOffX + (pX1 * (scSzX + 1));
        int mkY = scOffY + (pY1 * (scSzY + 1));

        int maxX = (pX2 - pX1 + 1) * (scSzX + 1);
        int maxY = (pY2 - pY1 + 1) * (scSzY + 1);

        for (int x = 0; x <= maxX; x++)
        {
            int y = (scSzX == scSzY ? x : (int)Math.Round(x * boxAspect));
            map[mkY + y][mkX + x] = code;
            map[mkY + maxY - y][mkX + x] = code;
            map[mkY][mkX + x] = code;
            map[mkY + maxY][mkX + x] = code;
        }

        for (int y = 0; y <= maxY; y++)
        {
            map[mkY + y][mkX] = code;
            map[mkY + y][mkX + maxX] = code;
        }
    }

    public void PaintHollowBox(char[][] map, int pX, int pY, char code)
    {
        int mkX = scOffX + (pX * (scSzX + 1));
        int mkY = scOffY + (pY * (scSzY + 1));

        for (int x = 0; x <= scSzX + 1; x++)
        {
            map[mkY][mkX + x] = code;
            map[mkY + scSzY + 1][mkX + x] = code;
        }

        for (int y = 0; y <= scSzY + 1; y++)
        {
            map[mkY + y][mkX] = code;
            map[mkY + y][mkX + scSzX + 1] = code;
        }
    }

    public void PaintHollowBox(char[][] map, int pX1, int pY1, int pX2, int pY2, char code)
    {
        int mkX = scOffX + (pX1 * (scSzX + 1));
        int mkY = scOffY + (pY1 * (scSzY + 1));

        int maxX = (pX2 - pX1 + 1) * (scSzX + 1);
        int maxY = (pY2 - pY1 + 1) * (scSzY + 1);

        for (int x = 0; x <= maxX; x++)
        {
            map[mkY][mkX + x] = code;
            map[mkY + maxY][mkX + x] = code;
        }

        for (int y = 0; y <= maxY; y++)
        {
            map[mkY + y][mkX] = code;
            map[mkY + y][mkX + maxX] = code;
        }
    }

    public void PaintLocationMarker(char[][] map, int pX, int pY, char even, char odd)
    {
        int mkX = scOffX + (pX * (scSzX + 1));
        int mkY = scOffY + (pY * (scSzY + 1));

        for (int x = 0; x <= scSzX + 1; x++)
        {
            for (int y = 0; y <= scSzY + 1; y++)
            {
                map[mkY + y][mkX + x] = ((x + y) % 2 == 0 ? even : odd);
            }
        }
    }
}

public class CustomConfiguration
{
    public IMyTerminalBlock configBlock;
    public Dictionary<string, string> config;

    public CustomConfiguration(IMyTerminalBlock blk)
    {
        configBlock = blk;
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

    public static void ParseCustomData(IMyTerminalBlock blk, Dictionary<string, string> cfg, bool clr = true)
    {
        if (clr)
        {
            cfg.Clear();
        }

        string[] arr = blk.CustomData.Split(new char[] { '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries);
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

    public static void WriteCustomData(IMyTerminalBlock blk, Dictionary<string, string> cfg)
    {
        StringBuilder sb = new StringBuilder(cfg.Count * 100);
        foreach (KeyValuePair<string, string> va in cfg)
        {
            sb.Append(va.Key).Append('=').Append(va.Value).Append('\n');
        }
        blk.CustomData = sb.ToString();
    }
}