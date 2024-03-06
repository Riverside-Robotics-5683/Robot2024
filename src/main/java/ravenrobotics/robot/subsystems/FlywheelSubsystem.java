package ravenrobotics.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import ravenrobotics.robot.Constants.FlywheelConstants;
import ravenrobotics.robot.Constants.MotorConstants;

public class FlywheelSubsystem extends SubsystemBase 
{
    //Motors
    private final CANSparkMax topMotor = new CANSparkMax(FlywheelConstants.kTopFlyWheel, MotorType.kBrushless);
    private final CANSparkMax bottomMotor = new CANSparkMax(FlywheelConstants.kBottomFlyWheel, MotorType.kBrushless);
    
    //Motor Encoders
    private final RelativeEncoder topMotorEncoder = topMotor.getEncoder();
    private final RelativeEncoder bottomMotorEncoder = bottomMotor.getEncoder();

    //Ramps the speeds FlyWheel to a set velocity when enabled
    private final BangBangController controller = new BangBangController();

    //Allows you to use FlywheelSubsystem in other classes
    private static FlywheelSubsystem instance;

    private final SlewRateLimiter rampLimiter = new SlewRateLimiter(0.5);

    //Tell the flywheel is enabled
    private boolean isEnabled = false;
    private boolean isOverride = false;

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

    public void shootOn() 
    {
        topMotor.set(-1);
        bottomMotor.set(-1);
    }

    public void stopFly()
    {
     topMotor.set(0);
     bottomMotor.set(0);
    }

    public double getVelocity()
    {
        return -(topMotorEncoder.getVelocity() + bottomMotorEncoder.getVelocity() / (double)2);
    }

    @Override
    public void periodic(){
    }

    public void configMotors()
    {
        topMotor.restoreFactoryDefaults();
        bottomMotor.restoreFactoryDefaults();

        topMotor.setInverted(true);
        bottomMotor.setInverted(true);

        topMotor.setIdleMode(IdleMode.kCoast);
        bottomMotor.setIdleMode(IdleMode.kCoast);

        topMotor.setSmartCurrentLimit(MotorConstants.kAmpLimit);
        bottomMotor.setSmartCurrentLimit(MotorConstants.kAmpLimit);

        topMotorEncoder.setPosition(0.0);
        bottomMotorEncoder.setPosition(0.0);

        topMotorEncoder.setVelocityConversionFactor(3);
        bottomMotorEncoder.setVelocityConversionFactor(3);

        topMotor.burnFlash();
        bottomMotor.burnFlash();
    }
}
