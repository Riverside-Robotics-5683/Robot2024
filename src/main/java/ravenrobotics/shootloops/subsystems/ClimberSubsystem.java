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
    private final TalonSRX leftClimber = new TalonSRX(ClimberConstants.kLeftClimber);
    private final TalonSRX rightClimber = new TalonSRX(ClimberConstants.kRightClimber);

    private final DigitalInput leftLimitSwitch = new DigitalInput(1);
    private final DigitalInput rightLimitSwitch = new DigitalInput(2);

    private final GenericEntry leftClimberLimit = Telemetry.teleopTab.add("Left Limit Switch", false).getEntry();
    private final GenericEntry rightClimberLimit = Telemetry.teleopTab.add("Right Limit Switch", false).getEntry();

    public enum ClimberDirection
    {
        kUp,
        kDown
    }

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

    public void moveLeft(ClimberDirection direction)
    {
        switch (direction)
        {
            case kUp -> leftClimber.set(ControlMode.PercentOutput, -.75);
            case kDown -> {if (!leftLimitSwitch.get()) return; leftClimber.set(ControlMode.PercentOutput, .75);}
        }
    }

    public void moveRight(ClimberDirection direction)
    {
        switch (direction)
        {
            case kUp -> rightClimber.set(ControlMode.PercentOutput, .75);
            case kDown -> {if (!rightLimitSwitch.get()) return; rightClimber.set(ControlMode.PercentOutput, -.75);}
        }
    }

    public void bothUp()
    {
        moveLeft(ClimberDirection.kUp);
        moveRight(ClimberDirection.kUp);
    }

    public void bothDown()
    {
        moveLeft(ClimberDirection.kDown);
        moveRight(ClimberDirection.kDown);
    }

    public void stopMotors()
    {
        leftClimber.set(ControlMode.PercentOutput, 0);
        rightClimber.set(ControlMode.PercentOutput, 0);
    }

    public void toBrake()
    {
        leftClimber.setNeutralMode(NeutralMode.Brake);
        rightClimber.setNeutralMode(NeutralMode.Brake);
    }

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
        TalonSRXConfiguration motorConfig = new TalonSRXConfiguration();

        motorConfig.peakCurrentLimit = 30;
        motorConfig.peakCurrentDuration = 500;
        motorConfig.continuousCurrentLimit = 25;

        leftClimber.configAllSettings(motorConfig);
        rightClimber.configAllSettings(motorConfig);

        leftClimber.setNeutralMode(NeutralMode.Coast);
        rightClimber.setNeutralMode(NeutralMode.Coast);
    }
}
