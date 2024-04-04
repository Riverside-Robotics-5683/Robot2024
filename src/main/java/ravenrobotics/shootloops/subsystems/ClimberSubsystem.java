package ravenrobotics.shootloops.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import ravenrobotics.shootloops.Constants.ClimberConstants;
import ravenrobotics.shootloops.util.Telemetry;

public class ClimberSubsystem extends SubsystemBase 
{
    //Declare the climber motor controllers.
    private final TalonSRX leftClimber = new TalonSRX(ClimberConstants.kLeftClimber);
    private final TalonSRX rightClimber = new TalonSRX(ClimberConstants.kRightClimber);

    //Declare the limit switches.
    private final DigitalInput leftLimitSwitch = new DigitalInput(1);
    private final DigitalInput rightLimitSwitch = new DigitalInput(2);

    //Declare the Shuffleboard entries for the limit switches.
    private final GenericEntry leftClimberLimit = Telemetry.teleopTab.add("Left Limit Switch", false).getEntry();
    private final GenericEntry rightClimberLimit = Telemetry.teleopTab.add("Right Limit Switch", false).getEntry();

    /**
     * The different directions the climber can go.
     */
    public enum ClimberDirection
    {
        /**
         * Move the climber up.
         */
        kUp,
        /**
         * Move the climber down.
         */
        kDown
    }

    //The static instance for other classes to access.
    private static ClimberSubsystem instance;

    /**
     * Returns the active instance of the ClimberSubsystem.
     * 
     * @return The active ClimberSubsystem instance.
     */
    public static ClimberSubsystem getInstance()
    {
        //If the instance doesn't exist yet, create a new one.
        if (instance == null)
        {
            instance = new ClimberSubsystem();
        }
        //Return the instance.
        return instance;
    }

    private ClimberSubsystem()
    {
        configMotors();
    }

    /**
     * Moves the left climber.
     * 
     * @param direction The direction to move the climber.
     */
    public void moveLeft(ClimberDirection direction)
    {
        switch (direction)
        {
            //If commanded to move up, set the climber output to move up.
            case kUp -> leftClimber.set(ControlMode.PercentOutput, -.75);
            //If commanded to move down, set the climber to move down if the limit switch isn't engaged.
            case kDown -> {if (!leftLimitSwitch.get()) {leftClimber.set(ControlMode.PercentOutput, 0); return;} leftClimber.set(ControlMode.PercentOutput, .75);}
        }
    }

    /**
     * Moves the right climber.
     * 
     * @param direction The direction to move the climber.
     */
    public void moveRight(ClimberDirection direction)
    {
        switch (direction)
        {
            //If commanded to move up, set the climber output to move up.
            case kUp -> rightClimber.set(ControlMode.PercentOutput, .75);
            //If commanded to move down, set the climber to move down if the limit switch isn't engaged.
            case kDown -> {if (!rightLimitSwitch.get()) {rightClimber.set(ControlMode.PercentOutput, 0); return;} rightClimber.set(ControlMode.PercentOutput, -.75);}
        }
    }

    /**
     * Moves both climbers up.
     */
    public void bothUp()
    {
        moveLeft(ClimberDirection.kUp);
        moveRight(ClimberDirection.kUp);
    }

    /**
     * Moves both climbers down.
     */
    public void bothDown()
    {
        moveLeft(ClimberDirection.kDown);
        moveRight(ClimberDirection.kDown);
    }

    /**
     * Immediately stops both motors.
     */
    public void stopMotors()
    {
        leftClimber.set(ControlMode.PercentOutput, 0);
        rightClimber.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Sets the climber motors to brake mode.
     */
    public void toBrake()
    {
        leftClimber.setNeutralMode(NeutralMode.Brake);
        rightClimber.setNeutralMode(NeutralMode.Brake);
    }

    /**
     * Sets the climber motors to coast mode.
     */
    public void toCoast()
    {
        leftClimber.setNeutralMode(NeutralMode.Coast);
        rightClimber.setNeutralMode(NeutralMode.Coast);
    }

    @Override
    public void periodic()
    {
        leftClimberLimit.setBoolean(!leftLimitSwitch.get());
        rightClimberLimit.setBoolean(!rightLimitSwitch.get());
    }

    private void configMotors()
    {
        //Create a new motor config.
        TalonSRXConfiguration motorConfig = new TalonSRXConfiguration();

        //Set peak current to 30 amps for 1/2 a second (500 milliseconds).
        motorConfig.peakCurrentLimit = 30;
        motorConfig.peakCurrentDuration = 500;
        //Set the normal current limit to 25 amps.
        motorConfig.continuousCurrentLimit = 25;

        //Apply the new settings.
        leftClimber.configAllSettings(motorConfig);
        rightClimber.configAllSettings(motorConfig);

        //Set the neutural mode to coast mode.
        leftClimber.setNeutralMode(NeutralMode.Coast);
        rightClimber.setNeutralMode(NeutralMode.Coast);
    }
}
