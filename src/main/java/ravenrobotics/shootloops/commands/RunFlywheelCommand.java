package ravenrobotics.shootloops.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import ravenrobotics.shootloops.subsystems.DriveSubsystem;
import ravenrobotics.shootloops.subsystems.FlywheelSubsystem;
import ravenrobotics.shootloops.subsystems.IntakeSubsystem;

public class RunFlywheelCommand extends Command 
{
    //Declare the intake, flywheel, and drive subsystems.
    private final IntakeSubsystem intakeSubsystem;
    private final FlywheelSubsystem flywheelSubsystem;
    private final DriveSubsystem driveSubsystem;
    
    /**
     * Command to run the flywheel to shoot a note into the speaker.
     */
    public RunFlywheelCommand()
    {
        //Get the instances of the intake, flywheel, and drive subsystems.
        this.intakeSubsystem = IntakeSubsystem.getInstance();
        this.flywheelSubsystem = FlywheelSubsystem.getInstance();
        this.driveSubsystem = DriveSubsystem.getInstance();
        //Add all of the subsystems being used as requirements for the command.
        addRequirements(intakeSubsystem, flywheelSubsystem, driveSubsystem);
    }

    @Override
    public void initialize()
    {
        //Set the motors to brake mode so that the robot stops instaed of coasting.
        driveSubsystem.motorsToBrake();
    }

    @Override
    public void execute()
    {
        //Set the drive subsystem to not be driving, so the shot can be stabilized.
        driveSubsystem.drive(new ChassisSpeeds(0, 0, 0));
        //Start the flywheels so that the note can be shot.
        flywheelSubsystem.shootOn();
        //Wait for the RPMs of the flywheels to reach the target velocity.
        while (flywheelSubsystem.getBottomVelocity() < 8000 && flywheelSubsystem.getTopVelocity() < 8000)
        {
            //System.out.println("Velocity:" + flywheelSubsystem.getVelocity());
            continue;
        }
        //Wait 0.1 seconds to let the flywheels fully stabilize.
        Timer.delay(0.1);
        //Start the intake wheels so that the note is launched.
        intakeSubsystem.runRollersFlywheel();
    }

    @Override
    public void end(boolean interrupted)
    {
        //Stop the flywheels, stop the rollers, and set the motors back to coast mode.
        flywheelSubsystem.stopFly();
        intakeSubsystem.stopRollers();
        driveSubsystem.motorsToCoast();
    }
}
