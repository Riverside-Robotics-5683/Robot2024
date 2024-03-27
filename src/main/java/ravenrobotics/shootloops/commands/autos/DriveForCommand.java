package ravenrobotics.shootloops.commands.autos;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import ravenrobotics.shootloops.subsystems.DriveSubsystem;

public class DriveForCommand extends Command
{
    //Declares the drive subsystem object.
    private final DriveSubsystem driveSubsystem;
    //Delcares the variables for the forward speed, rotation speed, and the amount of time to drive.
    private final double forwardSpeed, rotationSpeed, time;

    private boolean isDone = false;

    /**
     * Command for driving forward/backward, and rotating left/right for a specific amount of time.
     * 
     * @param forwardSpeed The forward/backward speed in m/s.
     * @param rotationSpeed The rotation speed (left/right) in m/s.
     * @param time The amount of time to drive.
     */
    public DriveForCommand(double forwardSpeed, double rotationSpeed, double time)
    {
        //Get the drive subsystem instance.
        driveSubsystem = DriveSubsystem.getInstance();
        //Set the target speeds and time.
        this.forwardSpeed = forwardSpeed;
        this.rotationSpeed = rotationSpeed;
        this.time = time;
        //Add the drive subsystem as a requirement.
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize()
    {
        //Make sure the motors are in brake mode.
        driveSubsystem.motorsToBrake();
    }

    @Override
    public void execute()
    {
        //Start driving the specified speeds.
        driveSubsystem.drive(new ChassisSpeeds(forwardSpeed, 0, rotationSpeed));
        //Wait the specified amount of time, then stop.
        Timer.delay(time);
        isDone = true;
    }

    @Override
    public void end(boolean isInterrupted)
    {
        driveSubsystem.drive(new ChassisSpeeds(0, 0, 0));
    }

    @Override
    public boolean isFinished()
    {
        return isDone;
    }
}
