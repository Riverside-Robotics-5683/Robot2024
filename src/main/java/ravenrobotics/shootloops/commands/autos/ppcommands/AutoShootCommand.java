package ravenrobotics.shootloops.commands.autos.ppcommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import ravenrobotics.shootloops.subsystems.FlywheelSubsystem;
import ravenrobotics.shootloops.subsystems.IntakeSubsystem;

public class AutoShootCommand extends Command
{
    private final IntakeSubsystem intakeSubsystem;
    private final FlywheelSubsystem flywheelSubsystem;

    private boolean isDone = false;

    public AutoShootCommand()
    {
        this.intakeSubsystem = IntakeSubsystem.getInstance();
        this.flywheelSubsystem = FlywheelSubsystem.getInstance();

        addRequirements(intakeSubsystem, flywheelSubsystem);
    }

    @Override
    public void initialize()
    {
        flywheelSubsystem.shootOn();
    }

    @Override
    public void execute()
    {
        // while (!intakeSubsystem.waitForIntake())
        // {
        //     continue;
        // }
        while (flywheelSubsystem.getVelocity() < 7000)
        {
            continue;
        }
        intakeSubsystem.runRollersFlywheel();
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
