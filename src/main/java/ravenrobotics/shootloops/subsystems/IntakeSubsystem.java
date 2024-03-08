package ravenrobotics.shootloops.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import ravenrobotics.shootloops.Constants.IntakeConstants;
import ravenrobotics.shootloops.Constants.MotorConstants;
import ravenrobotics.shootloops.subsystems.RGBSubsystem.RGBValues;
import ravenrobotics.shootloops.util.Telemetry;

public class IntakeSubsystem extends SubsystemBase
{
    //Roller motor and encoder.
    private final CANSparkMax rollerMotor = new CANSparkMax(IntakeConstants.kRollerMotor, MotorType.kBrushless);
    private final RelativeEncoder rollerMotorEncoder = rollerMotor.getEncoder();
    //Arm motor and encoder.
    private final CANSparkMax armMotor = new CANSparkMax(IntakeConstants.kArmMotor, MotorType.kBrushless);
    private final RelativeEncoder armMotorEncoder = armMotor.getEncoder();

    //PID Controller for the arm.
    private final SparkPIDController armPIDController = armMotor.getPIDController();

    private final DigitalInput distanceSensor = new DigitalInput(0);
    private final Debouncer distanceFilter = new Debouncer(0.1, DebounceType.kBoth);

    //Shuffleboard
    private final GenericEntry armPositionEntry = Telemetry.teleopTab.add("Arm Position", 0).getEntry();

    private static IntakeSubsystem instance;

    public enum IntakeArmPosition
    {
        kDeployed,
        kRetracted
    }

    /**
     * Returns the active instance of the IntakeSubsystem.
     * 
     * @return The IntakeSubsystem instance.
     */
    public static IntakeSubsystem getInstance()
    {
        //If the instance hasn't been created it yet, create it.
        if (instance == null)
        {
            System.out.println("Creating IntakeSubsystem instance");
            instance = new IntakeSubsystem();
        }
        
        //Return the instance.
        return instance;
    }

    /**
     * Subsystem for controlling the intake on the robot.
     */
    private IntakeSubsystem()
    {
        //Configure the motors and encoders for use.
        configMotors();
        configEncoders();
    }

    /**
     * Sets the intake position.
     * 
     * @param position The desired position of the intake.
     */
    public void setIntakePosition(IntakeArmPosition position)
    {
        if (position == IntakeArmPosition.kDeployed)
        {
            RGBSubsystem.getInstance().setPattern(RGBValues.kIntakeOut);
            armPIDController.setReference(IntakeConstants.kArmDeployedSetpoint, ControlType.kPosition);
        }
        else if (position == IntakeArmPosition.kRetracted)
        {
            RGBSubsystem.getInstance().setPattern(RGBValues.kDefault);
            armPIDController.setReference(0, ControlType.kPosition);
        }
    }

    public boolean waitForIntake()
    {
        boolean isReference = Math.abs(armMotorEncoder.getVelocity()) < 3;

        return isReference;
    }

    public void runRollers()
    {
        //changed because rollers were super speedy
        rollerMotor.set(1);
    }

    public void intakeRunRollers()
    {
        rollerMotor.set(-0.75);
    }

    public void runRollersSlow()
    {
        rollerMotor.set(.5);
    }

    public void stopRollers()
    {
        rollerMotor.stopMotor();
    }

    public boolean getDistanceSensor()
    {
        return distanceFilter.calculate(distanceSensor.get());
    }

    @Override
    public void periodic()
    {
        if(DriverStation.isEnabled())
        {
            //System.out.println("Arm Position: " + armMotorEncoder.getPosition());
        }
        //Update arm motor's position on Shuffleboard.
        armPositionEntry.setDouble(armMotorEncoder.getPosition());
    }

    /**
     * Configures the subsystem's motors for use.
     */
    private void configMotors()
    {
        //Reset to factory defaults.
        rollerMotor.restoreFactoryDefaults();

        //Set idle mode.
        rollerMotor.setIdleMode(IdleMode.kCoast);
        armMotor.setIdleMode(IdleMode.kBrake);

        rollerMotor.setSmartCurrentLimit(MotorConstants.kAmpLimit);
        armMotor.setSmartCurrentLimit(MotorConstants.kAmpLimit);

        armMotor.setClosedLoopRampRate(0.01);
        armPIDController.setFeedbackDevice(armMotorEncoder);
        armPIDController.setOutputRange(-0.3, 0.5);

        //Set the PID constants for the PID controller.
        armPIDController.setP(IntakeConstants.kArmP);
        armPIDController.setI(IntakeConstants.kArmI);
        armPIDController.setD(IntakeConstants.kArmD);

        //Save configuration.
        rollerMotor.burnFlash();
        armMotor.burnFlash();
    }

    /**
     * Configures the subsystem's encoders for use.
     */
    private void configEncoders()
    {
        rollerMotorEncoder.setPosition(0.0);
        armMotorEncoder.setPosition(0.0);
    }
}
