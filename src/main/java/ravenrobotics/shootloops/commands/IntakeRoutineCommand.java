package ravenrobotics.shootloops.commands;

import edu.wpi.first.wpilibj2.command.Command;
import ravenrobotics.shootloops.subsystems.IntakeSubsystem;
import ravenrobotics.shootloops.subsystems.IntakeSubsystem.IntakeArmPosition;

public class IntakeRoutineCommand extends Command
{
    private final IntakeSubsystem intakeSubsystem;

    public IntakeRoutineCommand()
    {
        this.intakeSubsystem = IntakeSubsystem.getInstance();

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize()
    {
        intakeSubsystem.setIntakePosition(IntakeArmPosition.kIntake);
    }

    @Override
    public void execute()
    {
        intakeSubsystem.runRollersIntake();
    }

    @Override
    public void end(boolean isInterrupted)
    {
        intakeSubsystem.stopRollers();
        intakeSubsystem.setIntakePosition(IntakeArmPosition.kRetracted);
    }

    @Override
    public boolean isFinished()
    {
        return intakeSubsystem.getDistanceSensor();
    }
}
