package ravenrobotics.shootloops.commands;

import edu.wpi.first.wpilibj2.command.Command;
import ravenrobotics.shootloops.subsystems.IntakeSubsystem;
import ravenrobotics.shootloops.subsystems.IntakeSubsystem.IntakeArmPosition;

public class IntakeRoutineCommand extends Command
{
    //Declare the intake subsystem.
    private final IntakeSubsystem intakeSubsystem;

    /**
     * Command to automatically run the intake and retract when done.
     */
    public IntakeRoutineCommand()
    {
        //Gets the active intake subsystem instance.
        this.intakeSubsystem = IntakeSubsystem.getInstance();
        //Add the subsystem as a requirement so that it isn't being used by other commands while this command is running.
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize()
    {
        //Set the intake to deployed for intaking.
        intakeSubsystem.setIntakePosition(IntakeArmPosition.kIntake);
    }

    @Override
    public void execute()
    {
        //Run the rollers to intake the note.
        intakeSubsystem.runRollersIntake();
    }

    @Override
    public void end(boolean isInterrupted)
    {
        //Stop the rollers, then retract the intake.
        intakeSubsystem.stopRollers();
        intakeSubsystem.setIntakePosition(IntakeArmPosition.kRetracted);
    }

    @Override
    public boolean isFinished()
    {
        //Check if there is a note in the intake, and finish the command if there is.
        return intakeSubsystem.getDistanceSensor();
    }
}
