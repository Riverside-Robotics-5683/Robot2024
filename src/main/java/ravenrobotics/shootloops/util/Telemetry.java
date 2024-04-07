package ravenrobotics.shootloops.util;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Telemetry
{
    //The teleop tab as a Shuffleboard tab object.
    public static ShuffleboardTab teleopTab = Shuffleboard.getTab("TeleOp");
    //The teleop tab name.
    public static String teleopTabString = "TeleOp";

    /**
     * Switches the active Shuffleboard tab to the tab used for TeleOp.
     */
    public static void switchToTeleopTab()
    {
        Shuffleboard.selectTab(teleopTabString);
    }
}
