package ravenrobotics.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import ravenrobotics.robot.Constants.ClimberConstants;
import ravenrobotics.robot.util.Telemetry;

public class ClimberSubsystem extends SubsystemBase
{
    private final CANSparkMax leftClimber = new CANSparkMax(ClimberConstants.kLeftClimber, MotorType.kBrushless);
    private final RelativeEncoder leftClimberEncoder = leftClimber.getEncoder();
    private final SparkPIDController leftClimberController = leftClimber.getPIDController();

    private final CANSparkMax rightClimber = new CANSparkMax(ClimberConstants.kRightClimber, MotorType.kBrushless);
    private final RelativeEncoder rightClimberEncoder = rightClimber.getEncoder();
    private final SparkPIDController rightClimberController = rightClimber.getPIDController();

    private GenericEntry leftClimberEntry = Telemetry.teleopTab.add("Left Climber", 0).getEntry();
    private GenericEntry rightClimberEntry = Telemetry.teleopTab.add("Right Climber", 0).getEntry();

    private static ClimberSubsystem instance;

    public static ClimberSubsystem getInstance()
    {
        if (instance == null)
        {
            instance = new ClimberSubsystem();
        }

        return instance;
    }

    private ClimberSubsystem()
    {
        configMotors();
    }

    @Override
    public void periodic()
    {
        leftClimberEntry.setDouble(leftClimberEncoder.getPosition());
        rightClimberEntry.setDouble(rightClimberEncoder.getPosition());
    }

    private void configMotors()
    {
        leftClimber.restoreFactoryDefaults();
        rightClimber.restoreFactoryDefaults();

        leftClimber.setIdleMode(IdleMode.kCoast);
        rightClimber.setIdleMode(IdleMode.kCoast);

        leftClimber.setInverted(ClimberConstants.kInvertLeftSide);
        rightClimber.setInverted(ClimberConstants.kInvertRightSide);

        leftClimberEncoder.setPosition(0.0);
        rightClimberEncoder.setPosition(0.0);

        leftClimber.burnFlash();
        rightClimber.burnFlash();
    }
}
