package ravenrobotics.shootloops.commands.autos;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import ravenrobotics.shootloops.subsystems.DriveSubsystem;

public class DriveForCommand extends Command
{
    private final DriveSubsystem driveSubsystem;

    private final double forwardSpeed, rotationSpeed, time;

    private boolean isDone = false;

    public DriveForCommand(double forwardSpeed, double rotationSpeed, double time)
    {
        driveSubsystem = DriveSubsystem.getInstance();

        this.forwardSpeed = forwardSpeed;
        this.rotationSpeed = rotationSpeed;
        this.time = time;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize()
    {
        driveSubsystem.motorsToBrake();
    }

    @Override
    public void execute()
    {
        driveSubsystem.drive(new ChassisSpeeds(forwardSpeed, 0, rotationSpeed));
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
