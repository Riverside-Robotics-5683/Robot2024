package ravenrobotics.shootloops.commands.autos.ppcommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import ravenrobotics.shootloops.subsystems.DriveSubsystem;
import ravenrobotics.shootloops.subsystems.IMUSubsystem;

public class LineUpWithSpeakerCommand extends Command
{
    //Declare the drive and IMU subsystems.
    private final DriveSubsystem driveSubsystem;
    private final IMUSubsystem imuSubsystem;

    private boolean isDone = false;

    /**
     * Command for manually lining up with the speaker during autonomous.
     */
    public LineUpWithSpeakerCommand()
    {
        this.driveSubsystem = DriveSubsystem.getInstance();
        this.imuSubsystem = IMUSubsystem.getInstance();

        addRequirements(driveSubsystem, imuSubsystem);
    }

    @Override
    public void initialize()
    {
        driveSubsystem.drive(new ChassisSpeeds(-0.25, 0, 0));
    }

    @Override
    public void execute()
    {
        driveSubsystem.drive(new ChassisSpeeds(-0.25 , 0, 0));
        Timer.delay(0.125);
        if(Math.abs(imuSubsystem.getXSpeed()) < 0.01)
        {
            isDone = true;
        }
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
