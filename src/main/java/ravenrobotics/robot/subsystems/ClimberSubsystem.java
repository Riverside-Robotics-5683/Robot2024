package ravenrobotics.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import ravenrobotics.robot.Constants.ClimberConstants;
import ravenrobotics.robot.Constants.MotorConstants;

public class ClimberSubsystem extends SubsystemBase
{
    private final CANSparkMax leftClimber = new CANSparkMax(ClimberConstants.kLeftClimber, MotorType.kBrushless);
    private final RelativeEncoder leftClimberEncoder = leftClimber.getEncoder();
    private final SparkPIDController leftClimberController = leftClimber.getPIDController();

    private final CANSparkMax rightClimber = new CANSparkMax(ClimberConstants.kRightClimber, MotorType.kBrushless);
    private final RelativeEncoder rightClimberEncoder = rightClimber.getEncoder();
    private final SparkPIDController rightClimberController = rightClimber.getPIDController();

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

    public void leftUp()
    {
        leftClimberController.setReference(0, ControlType.kPosition);
    }

    public void rightUp()
    {
        rightClimberController.setReference(0, ControlType.kPosition);
    }

    public void leftDown()
    {
        leftClimberController.setReference(-30, ControlType.kPosition);
    }

    public void rightDown()
    {
        rightClimberController.setReference(-30, ControlType.kPosition);
    }

    public void leftIncrement(double increment)
    {
        if (increment < -30 || increment > 0)
        {
            return;
        }

        double newReference = MathUtil.clamp(leftClimberEncoder.getPosition() + increment, ClimberConstants.kClimberSetpoint, 0);
        
        leftClimberController.setReference(newReference, ControlType.kPosition);
    }

    public void rightIncrement(double increment)
    {
        if (increment < -30 || increment > 0)
        {
            return;
        }

        double newReference = MathUtil.clamp(rightClimberEncoder.getPosition() + increment, ClimberConstants.kClimberSetpoint, 0);

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

    public void test()
    {
        leftClimber.set(-.75);
    }

    private void configMotors()
    {
        leftClimber.restoreFactoryDefaults();
        rightClimber.restoreFactoryDefaults();

        leftClimber.setIdleMode(IdleMode.kBrake);
        rightClimber.setIdleMode(IdleMode.kBrake);

        leftClimber.setInverted(ClimberConstants.kInvertLeftSide);
        rightClimber.setInverted(ClimberConstants.kInvertRightSide);

        leftClimber.setSmartCurrentLimit(MotorConstants.kAmpLimit);
        rightClimber.setSmartCurrentLimit(MotorConstants.kAmpLimit);

        leftClimberEncoder.setPosition(0.0);
        rightClimberEncoder.setPosition(0.0);

        leftClimberController.setOutputRange(-.75, .75);
        rightClimberController.setOutputRange(-.75, .75);

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
