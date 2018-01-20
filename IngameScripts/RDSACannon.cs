//============================================================
// Script powering my RDSA Railgun Rotor Cannon.
//============================================================

//------------------------------------------------------------
// ADN - RDSA Cannon Script v1.0
//------------------------------------------------------------
string rotorTag = "RDSA";
string mergeBlockTag = "RDSA";

string projectileWarheadName = "Projectile Warhead";

string rotorTurretPBTag = "Rotor Turret Computer";
string argRotorsSITEnable = "SIT_ENABLE";
string argRotorsSITDisable = "SIT_DISABLE";

List<IMyMotorStator> rdsaRotors = null;
IMyShipMergeBlock mergeBlock = null;
IMyProgrammableBlock rotorTurretPB = null;

int state = 0;

bool init = false;

Program()
{
    Runtime.UpdateFrequency = UpdateFrequency.Once;
}

void Init()
{
    rdsaRotors = new List<IMyMotorStator>();
    GridTerminalSystem.GetBlocksOfType(rdsaRotors, block => { return block.CustomName.IndexOf(rotorTag, StringComparison.OrdinalIgnoreCase) >= 0; });

    if (rdsaRotors.Count > 0)
    {
        List<IMyShipMergeBlock> mergeBlocks = new List<IMyShipMergeBlock>();
        GridTerminalSystem.GetBlocksOfType(mergeBlocks, block => { return block.CustomName.IndexOf(mergeBlockTag, StringComparison.OrdinalIgnoreCase) >= 0; });
        if (mergeBlocks.Count > 0)
        {
            List<IMyProgrammableBlock> rotorTurretPBs = new List<IMyProgrammableBlock>();
            GridTerminalSystem.GetBlocksOfType(rotorTurretPBs, block => { return block.CustomName.IndexOf(rotorTurretPBTag, StringComparison.OrdinalIgnoreCase) >= 0; });

            FilterByCorrelation(ref rdsaRotors, ref mergeBlocks, ref rotorTurretPBs);

            if (rdsaRotors.Count > 0 && mergeBlocks.Count == 1)
            {
                mergeBlock = mergeBlocks[0];

                rotorTurretPB = (rotorTurretPBs.Count > 0 ? rotorTurretPBs[0] : null);

                state = 0;
                init = true;
            }
        }
    }

    if (init)
    {
        Echo("----- System Online -----");

        Echo("\n--[ RDSA Rotors ]--");
        foreach (IMyMotorStator rotor in rdsaRotors)
        {
            Echo(rotor.CustomName);
        }

        Echo("\n--[ Projectile Holder Merge Block ]--");
        Echo(mergeBlock.CustomName);
    }
    else
    {
        Echo("Correlation Failure. Unable To \nFind RDSA Rotors Or\nProjectile Holder\nMerge Block In Proper\nConfiguration.");
    }
}

void FilterByCorrelation(ref List<IMyMotorStator> rdsaRotors, ref List<IMyShipMergeBlock> mergeBlocks, ref List<IMyProgrammableBlock> rotorTurretPBs)
{
    Dictionary<long, IMyTerminalBlock> gridLookup = new Dictionary<long, IMyTerminalBlock>();
    Dictionary<long, IMyTerminalBlock> topGridLookup = new Dictionary<long, IMyTerminalBlock>();
    foreach (IMyMotorStator rotor in rdsaRotors)
    {
        if (rotor.TopGrid != null)
        {
            gridLookup[rotor.CubeGrid.EntityId] = rotor;
            topGridLookup[rotor.TopGrid.EntityId] = rotor;
        }
    }
    foreach (IMyShipMergeBlock merge in mergeBlocks)
    {
        gridLookup[merge.CubeGrid.EntityId] = merge;
    }

    IMyTerminalBlock baseBlock = null;
    foreach (IMyMotorStator rotor in rdsaRotors)
    {
        if (!topGridLookup.ContainsKey(rotor.CubeGrid.EntityId))
        {
            if (baseBlock == null || (Me.GetPosition() - rotor.GetPosition()).Length() < (Me.GetPosition() - baseBlock.GetPosition()).Length())
            {
                baseBlock = rotor;
            }
        }
    }

    rdsaRotors.Clear();
    mergeBlocks.Clear();

    while (baseBlock != null)
    {
        if ((baseBlock as IMyShipMergeBlock) != null)
        {
            mergeBlocks.Add(baseBlock as IMyShipMergeBlock);
            break;
        }
        else    //Can Only Been A Rotor At This Point
        {
            rdsaRotors.Add(baseBlock as IMyMotorStator);

            IMyCubeGrid topGrid = (baseBlock as IMyMotorStator).TopGrid;
            long gridId = topGrid.EntityId;

            if (gridLookup.TryGetValue(gridId, out baseBlock))
            {
                gridLookup.Remove(gridId);  //Prevent Cyclic Lookup
            }
            else
            {
                baseBlock = null;
            }
        }
    }

    if (mergeBlocks.Count == 0) //Cannot Find Merge Block As Ending Block
    {
        rdsaRotors.Clear();
    }

    IMyProgrammableBlock nearestPB = null;
    foreach (IMyProgrammableBlock pb in rotorTurretPBs)
    {
        if (nearestPB == null || (Me.GetPosition() - pb.GetPosition()).Length() < (Me.GetPosition() - nearestPB.GetPosition()).Length())
        {
            nearestPB = pb;
        }
    }

    rotorTurretPBs.Clear();
    if (nearestPB != null)
    {
        rotorTurretPBs.Add(nearestPB);
    }
}

void Main(string arguments, UpdateType updateSource)
{
    if (arguments.Length > 0 && arguments.Trim().Equals("RESET", StringComparison.OrdinalIgnoreCase))
    {
        init = false;
        Init();

        return;
    }

    if (!init)
    {
        Init();

        if (!init)
        {
            return;
        }

        if ((updateSource & UpdateType.Once) > 0)
        {
            return;
        }
    }

    if (state > 0 && ((updateSource & UpdateType.Update1) == 0))
    {
        return;
    }

    if (arguments.Length > 0 && arguments.Trim().StartsWith("TIME:", StringComparison.OrdinalIgnoreCase))
    {
        string countdownStr = arguments.Trim().Substring(5).Trim();
        float countdown;
        if (float.TryParse(countdownStr, out countdown))
        {
            IMyWarhead warhead = GridTerminalSystem.GetBlockWithName(projectileWarheadName) as IMyWarhead;
            if (warhead != null && warhead.CubeGrid == mergeBlock.CubeGrid)
            {
                warhead.IsArmed = true;
                warhead.DetonationTime = countdown;
                warhead.StartCountdown();
            }
        }
    }

    switch (state)
    {
        case 0:
            Runtime.UpdateFrequency = UpdateFrequency.Update1;

            foreach (IMyMotorStator rotor in rdsaRotors)
            {
                rotor.SetValueBool("ShareInertiaTensor", true);
                rotorTurretPB?.TryRun(argRotorsSITEnable);

                rotor.SetValueFloat("Displacement", 0.2f);
            }
            break;
        case 1:
            foreach (IMyMotorStator rotor in rdsaRotors)
            {
                rotor.SetValueFloat("Displacement", -0.4f);
            }
            break;
        case 2:
            foreach (IMyMotorStator rotor in rdsaRotors)
            {
                rotor.SetValueFloat("Displacement", 0.2f);
            }
            break;
        case 3:
            foreach (IMyMotorStator rotor in rdsaRotors)
            {
                rotor.SetValueBool("ShareInertiaTensor", false);
                rotorTurretPB?.TryRun(argRotorsSITDisable);

                rotor.SetValueFloat("Displacement", -0.4f);
            }
            mergeBlock.ApplyAction("OnOff_Off");
            break;
        case 4:
            Runtime.UpdateFrequency = UpdateFrequency.None;
            state = -1;

            mergeBlock.ApplyAction("OnOff_On");
            break;
    }
    state++;
}