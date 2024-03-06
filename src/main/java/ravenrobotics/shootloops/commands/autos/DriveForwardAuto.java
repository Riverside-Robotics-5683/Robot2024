package ravenrobotics.shootloops.commands.autos;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import ravenrobotics.shootloops.subsystems.DriveSubsystem;

public class DriveForwardAuto extends Command 
{
    private final DriveSubsystem driveSubsystem;

    private boolean isDone = false;

    private double timeStamp;
    
    public DriveForwardAuto()
    {
        this.driveSubsystem = DriveSubsystem.getInstance();

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize()
    {
        timeStamp = Timer.getFPGATimestamp();
    }

    @Override
    public void execute()
    {
        driveSubsystem.drive(new ChassisSpeeds(1.25, 0, 0));
    }

    @Override
    public void end(boolean isInterrupted)
    {
        driveSubsystem.drive(new ChassisSpeeds(0, 0, 0));
    }

    @Override
    public boolean isFinished()
    {
        double currentTimestamp = Timer.getFPGATimestamp();
        if(currentTimestamp - timeStamp >= 2.0)
        {
            isDone = true;
        }
        return isDone;
    }
}
