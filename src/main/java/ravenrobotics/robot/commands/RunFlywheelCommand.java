package ravenrobotics.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import ravenrobotics.robot.subsystems.DriveSubsystem;
import ravenrobotics.robot.subsystems.FlywheelSubsystem;
import ravenrobotics.robot.subsystems.IntakeSubsystem;

public class RunFlywheelCommand extends Command 
{
    private final IntakeSubsystem intakeSubsystem;
    private final FlywheelSubsystem flywheelSubsystem;
    private final DriveSubsystem driveSubsystem;
    
    public RunFlywheelCommand()
    {
        this.intakeSubsystem = IntakeSubsystem.getInstance();
        this.flywheelSubsystem = FlywheelSubsystem.getInstance();
        this.driveSubsystem = DriveSubsystem.getInstance();

        addRequirements(intakeSubsystem, flywheelSubsystem, driveSubsystem);
    }

    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {
        driveSubsystem.drive(new ChassisSpeeds(0, 0, 0));
        flywheelSubsystem.shootOn();
        while (flywheelSubsystem.getVelocity() < 5500)
        {
            //System.out.println("Velocity:" + flywheelSubsystem.getVelocity());
            continue;
        }
        Timer.delay(0.05);
        intakeSubsystem.runRollers();
    }

    @Override
    public void end(boolean interrupted)
    {
        flywheelSubsystem.stopFly();
        intakeSubsystem.stopRollers();
    }
}
