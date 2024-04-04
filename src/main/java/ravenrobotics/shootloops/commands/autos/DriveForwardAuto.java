package ravenrobotics.shootloops.commands.autos;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import ravenrobotics.shootloops.subsystems.DriveSubsystem;

public class DriveForwardAuto extends Command 
{
    //Declare drive subsystem for driving.
    private final DriveSubsystem driveSubsystem;

    private boolean isDone = false;

    private double timeStamp;
    
    /**
     * Command for only driving forward during auto.
     */
    public DriveForwardAuto()
    {
        //Get the drive subsystem instance.
        this.driveSubsystem = DriveSubsystem.getInstance();
        //Add the subsystem as a requirement so that other commands aren't using it while this command is schedules.
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize()
    {
        //Get the starting timestamp for when the command started.
        timeStamp = Timer.getFPGATimestamp();
    }

    @Override
    public void execute()
    {
        //Drive forward at 1.25 m/s.
        driveSubsystem.drive(new ChassisSpeeds(1.25, 0, 0));
    }

    @Override
    public void end(boolean isInterrupted)
    {
        //Stop driving since we are done.
        driveSubsystem.drive(new ChassisSpeeds(0, 0, 0));
    }

    @Override
    public boolean isFinished()
    {
        //Get the current timestamp.
        double currentTimestamp = Timer.getFPGATimestamp();
        //If 2 seconds have elapsed, finish the command.
        if(currentTimestamp - timeStamp >= 2.0)
        {
            isDone = true;
        }
        return isDone;
    }
}
