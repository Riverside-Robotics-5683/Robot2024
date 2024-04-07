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
    private final GenericEntry sensorEntry = Telemetry.teleopTab.add("Note Sensor", false).getEntry();

    //Instance object for simplifying access to the subsystem.
    private static IntakeSubsystem instance;

    /**
     * The different positions the intake can go to.
     */
    public enum IntakeArmPosition
    {
        /**
         * Set the intake position to intaking.
         */
        kIntake,
        /**
         * Set the intake position to amp scoring.
         */
        kAmp,
        /**
         * Set the intake position to retracted for shooting.
         */
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
        if (position == IntakeArmPosition.kIntake)
        {
            //Set the RGB to the intake out pattern.
            RGBSubsystem.getInstance().setPattern(RGBValues.kIntakeOut);
            //Set the PID controller to go to the setpoint for intaking.
            armPIDController.setReference(IntakeConstants.kArmDeployedSetpoint, ControlType.kPosition);
        }
        else if (position == IntakeArmPosition.kAmp)
        {
            //Set the PID controller to go to the setpoint for amp scoring.
            armPIDController.setReference(IntakeConstants.kArmAmpSetpoint, ControlType.kPosition);
        }
        else if (position == IntakeArmPosition.kRetracted)
        {
            //Set the RGB to the default pattern.
            RGBSubsystem.getInstance().setPattern(RGBValues.kDefault);
            //Set the PID controller to go to the setpoint for being retracted.
            armPIDController.setReference(0, ControlType.kPosition);
        }
    }

    /**
     * Checks if the intake is moving.
     * 
     * @return Whether the intake is moving or not.
     */
    public boolean waitForIntake()
    {
        boolean isReference = Math.abs(armMotorEncoder.getVelocity()) < 3;

        return isReference;
    }

    /**
     * Runs the intake rollers to launch the note into the flywheel.
     */
    public void runRollersFlywheel()
    {
        rollerMotor.set(1);
    }

    /**
     * Runs the intake rollers to intake a note.
     */
    public void runRollersIntake()
    {
        rollerMotor.set(-0.75);
    }
    
    /**
     * Runs the rollers to score a note into the amp.
     */
    public void runRollersAmp()
    {
        rollerMotor.set(0.6);
    }

    /**
     * Runs the intake rollers slower.
     */
    public void runRollersSlow()
    {
        rollerMotor.set(.5);
    }

    /**
     * Stops the intake rollers.
     */
    public void stopRollers()
    {
        rollerMotor.stopMotor();
    }

    /**
     * Gets the current reading from the distance sensor.
     * 
     * @return Whether the distance sensor is detecting anything.
     */
    public boolean getDistanceSensor()
    {
        boolean isDetected = distanceFilter.calculate(distanceSensor.get());

        //Since the sensor returns true if it doesn't detect anything, return the opposite value (false).
        if(isDetected) return false;

        //If the sensor does detect something, return true.
        return true;
    }

    @Override
    public void periodic()
    {
        if(DriverStation.isEnabled())
        {
            //System.out.println("Arm Position: " + armMotorEncoder.getPosition());
        }
        //Update the arm motor's position on Shuffleboard.
        armPositionEntry.setDouble(armMotorEncoder.getPosition());
        //Update the distance sensor 
        sensorEntry.setBoolean(getDistanceSensor());
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

        //Set the current limits.
        rollerMotor.setSmartCurrentLimit(MotorConstants.kAmpFreeLimit);
        armMotor.setSmartCurrentLimit(MotorConstants.kAmpFreeLimit);

        //Set the ramp rate.
        armMotor.setClosedLoopRampRate(0.01);
        //Set the feedback device for the PID controller.
        armPIDController.setFeedbackDevice(armMotorEncoder);
        //Set the output range for the PID controller.
        armPIDController.setOutputRange(-0.5, 0.5);

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
