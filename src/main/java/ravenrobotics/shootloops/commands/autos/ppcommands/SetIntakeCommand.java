package ravenrobotics.shootloops.commands.autos.ppcommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import ravenrobotics.shootloops.subsystems.IntakeSubsystem;
import ravenrobotics.shootloops.subsystems.IntakeSubsystem.IntakeArmPosition;

public class SetIntakeCommand extends Command
{
    //Declare the intake subsystem and the variable to hold the target position.
    private final IntakeSubsystem intakeSubsystem;
    private final IntakeArmPosition position;

    private boolean isFinished = false;

    /**
     * Command for changing the intake's position during autonomous.
     * 
     * @param position The target position for the intake.
     */
    public SetIntakeCommand(IntakeArmPosition position)
    {
        //Get the intake subsystem and set the target position to the position provided.
        this.intakeSubsystem = IntakeSubsystem.getInstance();
        this.position = position;
        //Add the intake subsystem as a requirement.
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize()
    {
        //Set the intake position target to the target position, then wait 0.5 seconds.
        intakeSubsystem.setIntakePosition(position);
        Timer.delay(0.5);
    }

    @Override
    public void execute()
    {
        //Check if the intake is moving to check if finished.
        isFinished = intakeSubsystem.waitForIntake();
    }

    @Override
    public boolean isFinished()
    {
        return isFinished;
    }
}
