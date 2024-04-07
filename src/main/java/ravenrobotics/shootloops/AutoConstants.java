package ravenrobotics.shootloops;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.util.Units;

public class AutoConstants 
{
    //PID Constants for PathPlanner.
    public static final PIDConstants kAutoTranslationPIDConstants = new PIDConstants(1.0463E-06, 0.0, 0.0);
    public static final PIDConstants kAutoRotationPIDConstants = new PIDConstants(1.0463E-06, 0.0, 0.0);
    //Max speed for autonomous and the radius of the robot (more PathPlanner).
    public static final double kMaxAutoSpeed = 5;
    public static final double kRobotRadius = Units.inchesToMeters(14);
}
