// package ravenrobotics.shootloops.subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkPIDController;
// import com.revrobotics.CANSparkBase.ControlType;
// import com.revrobotics.CANSparkBase.IdleMode;
// import com.revrobotics.CANSparkLowLevel.MotorType;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import ravenrobotics.shootloops.Constants.ClimberConstants;
// import ravenrobotics.shootloops.Constants.MotorConstants;

// public class ClimberSubsystem extends SubsystemBase 
// {
//     private final CANSparkMax leftClimber = new CANSparkMax(ClimberConstants.kLeftClimber, MotorType.kBrushless);
//     private final RelativeEncoder leftClimberEncoder = leftClimber.getEncoder();
//     private final SparkPIDController leftClimberController = leftClimber.getPIDController();

//     private final CANSparkMax rightClimber = new CANSparkMax(ClimberConstants.kRightClimber, MotorType.kBrushless);
//     private final RelativeEncoder rightClimberEncoder = rightClimber.getEncoder();
//     private final SparkPIDController rightClimberController = rightClimber.getPIDController();

//     public enum ClimberPosition
//     {
//         kDown,
//         kUp
//     }

//     private static ClimberSubsystem instance;

//     public static ClimberSubsystem getInstance()
//     {
//         if (instance == null)
//         {
//             instance = new ClimberSubsystem();
//         }

//         return instance;
//     }

//     private ClimberSubsystem()
//     {
//         configMotors();
//     }

//     public void leftToPosition(ClimberPosition position)
//     {
//         switch (position)
//         {
//             case kUp -> leftClimberController.setReference(ClimberConstants.kClimberSetpoint, ControlType.kPosition);
//             case kDown -> leftClimberController.setReference(0, ControlType.kPosition);
//         }
//     }

//     public void rightToPosition(ClimberPosition position)
//     {
//         switch (position)
//         {
//             case kUp -> rightClimberController.setReference(ClimberConstants.kClimberSetpoint, ControlType.kPosition);
//             case kDown -> rightClimberController.setReference(0, ControlType.kPosition);
//         }
//     }

//     public void leftIncrement(double increment)
//     {
//         if (increment < 0 || increment > ClimberConstants.kClimberSetpoint)
//         {
//             return;
//         }
        
//         double newSetpoint = leftClimberEncoder.getPosition() + increment;
//         newSetpoint = MathUtil.clamp(newSetpoint, 0, ClimberConstants.kClimberSetpoint);

//         leftClimberController.setReference(newSetpoint, ControlType.kPosition);
//     }

//     public void rightIncrement(double increment)
//     {
//         if (increment < 0 || increment > ClimberConstants.kClimberSetpoint)
//         {
//             return;
//         }

//         double newSetpoint = rightClimberEncoder.getPosition() + increment;
//         newSetpoint = MathUtil.clamp(newSetpoint, 0, ClimberConstants.kClimberSetpoint);

//         rightClimberController.setReference(newSetpoint, ControlType.kPosition);
//     }

//     public void bothDown()
//     {
//         leftToPosition(ClimberPosition.kDown);
//         rightToPosition(ClimberPosition.kDown);
//     }

//     public void bothUp()
//     {
//         leftToPosition(ClimberPosition.kUp);
//         rightToPosition(ClimberPosition.kUp);
//     }

//     public void leftDown()
//     {
//         leftClimber.set(-.6);
//     }

//     public void leftUp()
//     {
//         leftClimber.set(.6);
//     }

//     public void rightDown()
//     {
//         rightClimber.set(-.6);
//     }

//     public void rightUp()
//     {
//         rightClimber.set(.6);
//     }

//     public void stopMotors()
//     {
//         leftClimber.stopMotor();
//         rightClimber.stopMotor();
//     }

//     private void configMotors()
//     {
//         leftClimber.restoreFactoryDefaults();
//         rightClimber.restoreFactoryDefaults();

//         leftClimber.setInverted(ClimberConstants.kInvertLeftSide);
//         rightClimber.setInverted(ClimberConstants.kInvertRightSide);

//         leftClimber.setSmartCurrentLimit(MotorConstants.kAmpStallLimit, MotorConstants.kAmpFreeLimit);
//         rightClimber.setSmartCurrentLimit(MotorConstants.kAmpStallLimit, MotorConstants.kAmpFreeLimit);

//         leftClimber.setIdleMode(IdleMode.kCoast);
//         rightClimber.setIdleMode(IdleMode.kCoast);

//         leftClimberEncoder.setPosition(0);
//         rightClimberEncoder.setPosition(0);

//         leftClimber.burnFlash();
//         rightClimber.burnFlash();
//     }
// }
