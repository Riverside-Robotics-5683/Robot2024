package ravenrobotics.shootloops.util;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Telemetry
{
    public static ShuffleboardTab teleopTab = Shuffleboard.getTab("TeleOp");
    public static String teleopTabString = "TeleOp";

    /**
     * Switches the active Shuffleboard tab to the tab used for TeleOp.
     */
    public static void switchToTeleopTab()
    {
        Shuffleboard.selectTab(teleopTabString);
    }
}
