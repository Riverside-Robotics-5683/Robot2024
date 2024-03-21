package ravenrobotics.shootloops.commands.autos.ppcommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import ravenrobotics.shootloops.subsystems.IntakeSubsystem;
import ravenrobotics.shootloops.subsystems.IntakeSubsystem.IntakeArmPosition;

public class SetIntakeCommand extends Command
{
    private final IntakeSubsystem intakeSubsystem;
    private final IntakeArmPosition position;

    private boolean isFinished = false;

    public SetIntakeCommand(IntakeArmPosition position)
    {
        this.intakeSubsystem = IntakeSubsystem.getInstance();
        this.position = position;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize()
    {
        intakeSubsystem.setIntakePosition(position);
        Timer.delay(0.5);
    }

    @Override
    public void execute()
    {
        isFinished = intakeSubsystem.waitForIntake();
    }

    @Override
    public boolean isFinished()
    {
        return isFinished;
    }
}
