package ravenrobotics.robot.commands.autos.ppcommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import ravenrobotics.robot.subsystems.DriveSubsystem;
import ravenrobotics.robot.subsystems.FlywheelSubsystem;
import ravenrobotics.robot.subsystems.IntakeSubsystem;

public class AutoShootCommand extends Command
{
    private final DriveSubsystem driveSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final FlywheelSubsystem flywheelSubsystem;

    private boolean isDone = false;

    public AutoShootCommand()
    {
        this.driveSubsystem = DriveSubsystem.getInstance();
        this.intakeSubsystem = IntakeSubsystem.getInstance();
        this.flywheelSubsystem = FlywheelSubsystem.getInstance();

        addRequirements(driveSubsystem, intakeSubsystem, flywheelSubsystem);
    }

    @Override
    public void initialize()
    {
        flywheelSubsystem.shootOn();
    }

    @Override
    public void execute()
    {
        Timer.delay(0.9);
        intakeSubsystem.runRollers();
        Timer.delay(1.5);
        isDone = true;
    }

    @Override
    public void end(boolean isInterrupted)
    {
        flywheelSubsystem.stopFly();
        intakeSubsystem.stopRollers();
    }

    @Override
    public boolean isFinished()
    {
        return isDone;
    }
}
