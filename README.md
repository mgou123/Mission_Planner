# Mission_Planner
Mission Planner Software Stack for Bumblebee AUV 3.5, ASV 2.0 and above

## Details
This branch should be able to bring AUV4 to the first buoy it detects (need sensor fusion to work first. Check rqt to verify that vision pipeline does indeed work). Run the robosub21_auv4_controls.launch file to do this. Some things to work on:

1. Sonar: For sensor fusion to work, need sonar detector
2. Identity of Buoy:
  - Right now identity of the detected buoys are based on ML. Need to integrate into the FindBuoy node so that it can output position of the needed buoy
  - Buoy identities as returned from ML is either "Tommy Gun" or "Badge"
3. Navigate_To_Pose: Need to change NavigateControls to NavigateToPose because NavigateControls is dumb
