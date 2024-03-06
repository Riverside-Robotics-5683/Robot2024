package ravenrobotics.shootloops.commands.autos.ppcommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import ravenrobotics.shootloops.subsystems.DriveSubsystem;
import ravenrobotics.shootloops.subsystems.IntakeSubsystem;
import ravenrobotics.shootloops.subsystems.IntakeSubsystem.IntakeArmPosition;

public class IntakeNoteCommand extends Command 
{
    private final DriveSubsystem driveSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    
    private boolean isDone = false;

    private int mode = 0;

    public IntakeNoteCommand()
    {
        this.driveSubsystem = DriveSubsystem.getInstance();
        this.intakeSubsystem = IntakeSubsystem.getInstance();

        addRequirements(driveSubsystem, intakeSubsystem);
    }

    @Override
    public void initialize()
    {
        intakeSubsystem.setIntakePosition(IntakeArmPosition.kDeployed);
    }

    @Override
    public void execute()
    {
        if (!intakeSubsystem.waitForIntake() && mode == 0)
        {
            return;
        }

        mode = 1;

        intakeSubsystem.intakeRunRollers();

        driveSubsystem.drive(new ChassisSpeeds(1, 0, 0));

        Timer.delay(3);

        isDone = true;
    }

    @Override
    public void end(boolean isInterrupted)
    {
        driveSubsystem.drive(new ChassisSpeeds(0, 0, 0));
        intakeSubsystem.stopRollers();
        intakeSubsystem.setIntakePosition(IntakeArmPosition.kRetracted);
    }

    @Override
    public boolean isFinished()
    {
        return isDone;
    }
}
