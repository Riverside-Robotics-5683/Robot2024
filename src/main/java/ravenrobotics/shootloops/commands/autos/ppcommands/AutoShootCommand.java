package ravenrobotics.shootloops.commands.autos.ppcommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import ravenrobotics.shootloops.subsystems.FlywheelSubsystem;
import ravenrobotics.shootloops.subsystems.IntakeSubsystem;

public class AutoShootCommand extends Command
{
    //Declare the Intake and Flywheel subsystems.
    private final IntakeSubsystem intakeSubsystem;
    private final FlywheelSubsystem flywheelSubsystem;

    private boolean isDone = false;

    /**
     * Command for shooting a note into the speaker during autonomous.
     */
    public AutoShootCommand()
    {
        //Get the instances for the subsystems.
        this.intakeSubsystem = IntakeSubsystem.getInstance();
        this.flywheelSubsystem = FlywheelSubsystem.getInstance();
        //Add the subsystems as requirements so none of them are being used while the command is running.
        addRequirements(intakeSubsystem, flywheelSubsystem);
    }

    @Override
    public void initialize()
    {
        //Turn on the flywheel for shooting.
        flywheelSubsystem.shootOn();
    }

    @Override
    public void execute()
    {
        //Wait for both flywheel arrays to spin up to 8000 RPM.
        while (flywheelSubsystem.getBottomVelocity() < 8000 && flywheelSubsystem.getTopVelocity() < 8000)
        {
            continue;
        }
        //Wait 0.1 seconds.
        Timer.delay(0.1);
        //Turn on the intake to shoot out the note.
        intakeSubsystem.runRollersFlywheel();
        //Wait 1.5 seconds to be done.
        Timer.delay(1.5);
        isDone = true;
    }

    @Override
    public void end(boolean isInterrupted)
    {
        //Stop the flywheels and the rollers.
        flywheelSubsystem.stopFly();
        intakeSubsystem.stopRollers();
    }

    @Override
    public boolean isFinished()
    {
        return isDone;
    }
}
