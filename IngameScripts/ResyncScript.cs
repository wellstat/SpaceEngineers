//============================================================
// Script to immediately enable Raycast on all cameras on
// the grid to ensure it charges up on server startup.
//============================================================

//------------------------------------------------------------
// ADN - Resync Script v1.0
//------------------------------------------------------------
bool resyncedCameraRaycast = false;

Program()
{
    Runtime.UpdateFrequency = UpdateFrequency.Update100;
}

void Main(string arguments, UpdateType updateSource)
{
    if (!resyncedCameraRaycast)
    {
        List<IMyCameraBlock> blocks = new List<IMyCameraBlock>();
        GridTerminalSystem.GetBlocksOfType<IMyCameraBlock>(blocks);

        for (int i = 0; i < blocks.Count; i++)
        {
            blocks[i].ApplyAction("OnOff_On");
            blocks[i].EnableRaycast = true;
        }

        if (blocks.Count > 0)
        {
            resyncedCameraRaycast = true;
            Runtime.UpdateFrequency = UpdateFrequency.None;
        }
    }
}