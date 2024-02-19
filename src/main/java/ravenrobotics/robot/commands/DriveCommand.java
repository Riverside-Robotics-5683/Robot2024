package ravenrobotics.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.Command;
import ravenrobotics.robot.Constants.DrivetrainConstants;
import ravenrobotics.robot.subsystems.DriveSubsystem;
import ravenrobotics.robot.subsystems.IMUSubsystem;
import ravenrobotics.robot.util.Telemetry;

public class DriveCommand extends Command
{
    //The drive subsystem so the drivetrain can be driven ;).
    private final DriveSubsystem driveSubsystem;

    private final IMUSubsystem imuSubsystem;
    
    //Suppliers for joystick values and whether to drive field relative.
    private final DoubleSupplier xSpeed, ySpeed, tSpeed, maxSpeed;
    private final BooleanSupplier isFieldRelative;

    //Limiters so we don't break the chassis by instantly applying power.
    private final SlewRateLimiter xLimiter, yLimiter, tLimiter;

    //Max speed entry.
    private final GenericEntry maxSpeedEntry = Telemetry.teleopTab.add("Max Speed", DrivetrainConstants.kDriveMaxSpeedMPS * 0.1).getEntry();
    //Axis entries.
    private final GenericEntry xAxisEntry = Telemetry.teleopTab.add("X Axis", 0).getEntry();
    private final GenericEntry yAxisEntry = Telemetry.teleopTab.add("Y Axis", 0).getEntry();
    private final GenericEntry zAxisEntry = Telemetry.teleopTab.add("Z Axis", 0).getEntry();
    //Axis filter entries.
    private final GenericEntry xAxisFilterEntry = Telemetry.teleopTab.add("X Axis Filter", 0).getEntry();
    private final GenericEntry yAxisFilterEntry = Telemetry.teleopTab.add("Y Axis Filter", 0).getEntry();
    private final GenericEntry zAxisFilterEntry = Telemetry.teleopTab.add("Z Axis Filter", 0).getEntry();

    /**
     * Command to drive the robot using joystick axes.
     * 
     * @param strafeSpeed The axis for moving left/right.
     * @param forwardSpeed The axis for moving forward/backward.
     * @param rotationSpeed The axis for rotating.
     * @param maxSpeed The axis for controlling the max speed of the robot.
     * @param isFieldRelative The boolean for whether to drive field relative.
     */
    public DriveCommand(DoubleSupplier strafeSpeed, DoubleSupplier forwardSpeed, DoubleSupplier rotationSpeed, DoubleSupplier maxSpeed, BooleanSupplier isFieldRelative)
    {
        //Initialize subsystem instances.
        this.driveSubsystem = DriveSubsystem.getInstance();
        this.imuSubsystem = IMUSubsystem.getInstance();

        //Initialize double suppliers (getting joystick values)
        this.xSpeed = strafeSpeed;
        this.ySpeed = forwardSpeed;
        this.tSpeed = rotationSpeed;
        this.maxSpeed = maxSpeed;

        //Initialize boolean supplier (whether to drive field or robot relative)
        this.isFieldRelative = isFieldRelative;

        //Initialize SlewRateLimiters so we don't acclerate too quickly.
        xLimiter = new SlewRateLimiter(DrivetrainConstants.kTranslationSlewRate);
        yLimiter = new SlewRateLimiter(DrivetrainConstants.kTranslationSlewRate);
        tLimiter = new SlewRateLimiter(DrivetrainConstants.kRotationSlewRate);

        //Add the subsystem as a requirement for the command, so the subsystem isn't being controlled by two different commands at once.
        addRequirements(driveSubsystem, imuSubsystem);
    }

    @Override
    public void initialize()
    {
        Telemetry.switchToTeleopTab();
    }

    @Override
    public void execute()
    {
        //Temporary variables for the speeds.
        double xSpeedMPS, ySpeedMPS, tSpeedMPS, mSpeed;

        //Update the axis data on Shuffleboard.
        xAxisEntry.setDouble(xSpeed.getAsDouble());
        yAxisEntry.setDouble(ySpeed.getAsDouble());
        zAxisEntry.setDouble(tSpeed.getAsDouble());

        //Convert the maxSpeed axis to a range of 0-1.
        mSpeed = (maxSpeed.getAsDouble() + 1) * 0.5;
        if (mSpeed < 0.1)
        {
            mSpeed = 0.1;
        }


        //Update the max speed on Shuffleboard as a percentage.
        maxSpeedEntry.setDouble(mSpeed * 100);

        //Conver the max speed to a speed in meters per second.
        mSpeed *= DrivetrainConstants.kDriveMaxSpeedMPS;

        //Get the target strafe, forward/backward, and rotation speeds.
        xSpeedMPS = xLimiter.calculate(xSpeed.getAsDouble()) * mSpeed;
        ySpeedMPS = yLimiter.calculate(ySpeed.getAsDouble()) * mSpeed;
        tSpeedMPS = tLimiter.calculate(tSpeed.getAsDouble()) * mSpeed;

        if (Math.abs(xSpeed.getAsDouble()) < 0.03)
        {
            xLimiter.reset(0);
            xSpeedMPS = 0;
        }
        
        if (Math.abs(ySpeed.getAsDouble()) < 0.03)
        {
            yLimiter.reset(0);
            ySpeedMPS = 0;
        }

        if (Math.abs(tSpeed.getAsDouble()) < 0.01)
        {
            tLimiter.reset(0);
            tSpeedMPS = 0;
        }

        //Update the filter data on Shuffleboard.
        xAxisFilterEntry.setDouble(xLimiter.lastValue());
        yAxisFilterEntry.setDouble(yLimiter.lastValue());
        zAxisFilterEntry.setDouble(tLimiter.lastValue());

        //Convert the target speeds to a chassis speed.
        ChassisSpeeds targetSpeeds = new ChassisSpeeds(xSpeedMPS, ySpeedMPS, tSpeedMPS);

        //Convert the chassis speeds if driving field-oriented.
        if (isFieldRelative.getAsBoolean())
        {
            targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(targetSpeeds, imuSubsystem.getYaw());
        }
        //Drive the subsystem.
        driveSubsystem.drive(targetSpeeds, mSpeed);
    }
}
