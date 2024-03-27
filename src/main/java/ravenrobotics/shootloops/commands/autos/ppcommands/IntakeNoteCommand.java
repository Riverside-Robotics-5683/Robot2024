package ravenrobotics.shootloops.commands.autos.ppcommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import ravenrobotics.shootloops.subsystems.DriveSubsystem;
import ravenrobotics.shootloops.subsystems.IntakeSubsystem;

public class IntakeNoteCommand extends Command 
{
    //Declare the drive and intake subsystems.
    private final DriveSubsystem driveSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    /**
     * Command for intaking a note during autonomous. CONFIGURE WITH A TIMEOUT.
     */
    public IntakeNoteCommand()
    {
        //Get the instances of the subsystems.
        this.driveSubsystem = DriveSubsystem.getInstance();
        this.intakeSubsystem = IntakeSubsystem.getInstance();
        //Add the subsystems as requirements.
        addRequirements(driveSubsystem, intakeSubsystem);
    }

    @Override
    public void execute()
    {
        //Turn on the intake rollers.
        intakeSubsystem.runRollersIntake();
        //Drive at a slow-ish speed that is fine for just picking up a note.
        driveSubsystem.drive(new ChassisSpeeds(0.125, 0, 0));
    }

    @Override
    public void end(boolean isInterrupted)
    {
        //Stop driving.
        driveSubsystem.drive(new ChassisSpeeds(0, 0, 0));
        //Stop the rollers.
        intakeSubsystem.stopRollers();
    }

    @Override
    public boolean isFinished()
    {
        //Check if it's finished by checking the note sensor (there's a 3 second timeout for if we don't pick up a note).
        return intakeSubsystem.getDistanceSensor();
    }
}
