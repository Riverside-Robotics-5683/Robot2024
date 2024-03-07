package ravenrobotics.shootloops.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import ravenrobotics.shootloops.Constants.ClimberConstants;
import ravenrobotics.shootloops.Constants.MotorConstants;
import ravenrobotics.shootloops.util.Telemetry;

public class ClimberSubsystem extends SubsystemBase
{
    private final CANSparkMax leftClimber = new CANSparkMax(ClimberConstants.kLeftClimber, MotorType.kBrushless);
    private final RelativeEncoder leftClimberEncoder = leftClimber.getEncoder();
    private final SparkPIDController leftClimberController = leftClimber.getPIDController();

    private final CANSparkMax rightClimber = new CANSparkMax(ClimberConstants.kRightClimber, MotorType.kBrushless);
    private final RelativeEncoder rightClimberEncoder = rightClimber.getEncoder();
    private final SparkPIDController rightClimberController = rightClimber.getPIDController();

    private static ClimberSubsystem instance;

    private final GenericEntry leftEntry = Telemetry.teleopTab.add("Left Climber", 45).getEntry();
    private final GenericEntry rightEntry = Telemetry.teleopTab.add("Right Climber", 45).getEntry();

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
        System.out.println("Creating ClimberSubsystem instance.");
        configMotors();
        leftEntry.setDouble(leftClimberEncoder.getPosition());
        rightEntry.setDouble(rightClimberEncoder.getPosition());
    }

    public void leftUp()
    {
        leftClimberController.setReference(-ClimberConstants.kClimberSetpoint, ControlType.kPosition);
    }

    public void rightUp()
    {
        rightClimberController.setReference(ClimberConstants.kClimberSetpoint, ControlType.kPosition);
    }

    public void leftDown()
    {
        leftClimberController.setReference(0, ControlType.kPosition);
    }

    public void rightDown()
    {
        rightClimberController.setReference(0, ControlType.kPosition);
    }

    public void leftIncrement(double increment)
    {
        double newReference = rightClimberEncoder.getPosition() + increment;
        newReference = MathUtil.clamp(newReference, ClimberConstants.kClimberSetpoint, 0);
        
        leftClimberController.setReference(newReference, ControlType.kPosition);
    }

    public void rightIncrement(double increment)
    {
        double newReference = rightClimberEncoder.getPosition() + increment;
        newReference = MathUtil.clamp(newReference, 0, ClimberConstants.kClimberSetpoint);

        rightClimberController.setReference(newReference, ControlType.kPosition);
    }

    public void bothUp()
    {
        leftUp();
        rightUp();
    }

    public void bothDown()
    {
        leftDown();
        rightDown();
    }

    @Override
    public void periodic()
    {
        leftEntry.setDouble(leftClimberEncoder.getPosition());
        rightEntry.setDouble(rightClimberEncoder.getPosition());

        // System.out.println("Left Climber:" + leftClimberEncoder.getPosition());
        // System.out.println("Right Climber:" + rightClimberEncoder.getPosition());
    }

    public void test()
    {
        leftClimber.set(-.75);
    }

    private void configMotors()
    {
        leftClimber.restoreFactoryDefaults();
        rightClimber.restoreFactoryDefaults();

        leftClimber.setIdleMode(IdleMode.kCoast);
        rightClimber.setIdleMode(IdleMode.kCoast);

        leftClimber.setInverted(ClimberConstants.kInvertLeftSide);
        rightClimber.setInverted(ClimberConstants.kInvertRightSide);

        leftClimber.setSmartCurrentLimit(MotorConstants.kAmpLimit);
        rightClimber.setSmartCurrentLimit(MotorConstants.kAmpLimit);

        leftClimberEncoder.setPosition(0);
        rightClimberEncoder.setPosition(0);

        leftClimberEncoder.setPositionConversionFactor(10);
        rightClimberEncoder.setPositionConversionFactor(10);

        leftClimberController.setOutputRange(-1, 1);
        rightClimberController.setOutputRange(1, 1);

        leftClimberController.setP(1.0);
        leftClimberController.setI(0.0);
        leftClimberController.setD(0.5);

        rightClimberController.setP(1.0);
        rightClimberController.setI(0.0);
        rightClimberController.setD(0.5);

        leftClimber.burnFlash();
        rightClimber.burnFlash();
    }
}
