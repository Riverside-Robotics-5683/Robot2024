package ravenrobotics.shootloops.commands.autos.ppcommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import ravenrobotics.shootloops.subsystems.DriveSubsystem;
import ravenrobotics.shootloops.subsystems.IntakeSubsystem;
import ravenrobotics.shootloops.subsystems.IntakeSubsystem.IntakeArmPosition;

public class IntakeNoteCommand extends Command 
{
    private final DriveSubsystem driveSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    public IntakeNoteCommand()
    {
        this.driveSubsystem = DriveSubsystem.getInstance();
        this.intakeSubsystem = IntakeSubsystem.getInstance();

        addRequirements(driveSubsystem, intakeSubsystem);
    }

    @Override
    public void execute()
    {
        intakeSubsystem.runRollersIntake();
        driveSubsystem.drive(new ChassisSpeeds(0.125, 0, 0));
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
        return intakeSubsystem.getDistanceSensor();
    }
}
