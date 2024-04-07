package ravenrobotics.shootloops.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import ravenrobotics.shootloops.Constants.FlywheelConstants;
import ravenrobotics.shootloops.Constants.MotorConstants;

public class FlywheelSubsystem extends SubsystemBase 
{
    //The flywheel motors.
    private final CANSparkMax topMotor = new CANSparkMax(FlywheelConstants.kTopFlyWheel, MotorType.kBrushless);
    private final CANSparkMax bottomMotor = new CANSparkMax(FlywheelConstants.kBottomFlyWheel, MotorType.kBrushless);
    
    //The encoders 
    private final RelativeEncoder topMotorEncoder = topMotor.getEncoder();
    private final RelativeEncoder bottomMotorEncoder = bottomMotor.getEncoder();

    //Allows you to use FlywheelSubsystem in other classes
    private static FlywheelSubsystem instance;

    /**
     * Returns the active instance of the FlyWheelSubSystem.
     * 
     * @return The FlyWheelSubsystem instance.
     */
    public static FlywheelSubsystem getInstance()
    {
        //If the instance hasn't been created it yet, create it.
        if (instance == null)
        {
            System.out.println("Creating FlywheelSubsystem object.");
            instance = new FlywheelSubsystem();
        }
            
        //Return the instance
        return instance;
    }

    /**
     * Turn on the flywheels for shooting.
     */
    public void shootOn() 
    {
        topMotor.set(-1);
        bottomMotor.set(-1);
    }

    /**
     * Stop the flywheels.
     */
    public void stopFly()
    {
     topMotor.set(0);
     bottomMotor.set(0);
    }

    /**
     * Get the total velocity of the flywheels.
     * 
     * @return The total velocity as a double.
     */
    public double getTotalVelocity()
    {
        return -(topMotorEncoder.getVelocity() + bottomMotorEncoder.getVelocity() / (double)2);
    }

    /**
     * Get the velocity of the top flywheel.
     * 
     * @return The velocity as a double.
     */
    public double getTopVelocity()
    {
        return -topMotorEncoder.getVelocity();
    }

    /**
     * Get the velocity of the bottom flywheel.
     * 
     * @return The velocity as a double.
     */
    public double getBottomVelocity()
    {
        return -bottomMotorEncoder.getVelocity();
    }

    public void configMotors()
    {
        //Restore the factory settings to the motors.
        topMotor.restoreFactoryDefaults();
        bottomMotor.restoreFactoryDefaults();

        //Invert the motors.
        topMotor.setInverted(true);
        bottomMotor.setInverted(true);

        //Set the idle mode to coast mode.
        topMotor.setIdleMode(IdleMode.kCoast);
        bottomMotor.setIdleMode(IdleMode.kCoast);

        //Set the max current limit so that things don't break.
        topMotor.setSmartCurrentLimit(MotorConstants.kAmpFreeLimit);
        bottomMotor.setSmartCurrentLimit(MotorConstants.kAmpFreeLimit);

        //Reset the encoder position.
        topMotorEncoder.setPosition(0.0);
        bottomMotorEncoder.setPosition(0.0);

        //Set the velocity conversion factor so that velocity readings are accurate.
        topMotorEncoder.setVelocityConversionFactor(3);
        bottomMotorEncoder.setVelocityConversionFactor(3);

        topMotor.burnFlash();
        bottomMotor.burnFlash();
    }
}
