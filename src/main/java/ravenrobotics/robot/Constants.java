package ravenrobotics.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants 
{
    //Constants for controller IDs, etc.
    public static class DriverStationConstants
    {
        public static final int kDriverPort = 0;
        public static final int kSystemsPort = 1;
    }
    //Constants for the drivetrain, like motor IDs and whether they should be inverted.
    public static class DrivetrainConstants
    {
        /////////////////////
        //////Motor IDs//////
        /////////////////////
        public static final int kFrontLeftMotor = 2;
        public static final int kFrontRightMotor = 3;
        public static final int kBackLeftMotor = 4;
        public static final int kBackRightMotor = 5;
        /////////////////////
        ////Invert Motors////
        ////////////////////
        public static final boolean kInvertFrontLeftSide = true;
        public static final boolean kInvertFrontRightSide = false;
        public static final boolean kInvertBackLeftSide = true;
        public static final boolean kInvertBackRightSide = false;
        /////////////////////
        /////Max Voltage/////
        /////////////////////
        public static final int kDriveMaxVoltage = 12;
        public static final double kDriveSysIDVoltageRampRate = 0.5;
        /////////////////////
        ///Other Constants///
        /////////////////////
        public static final double kDriveMaxSpeedMPS = Units.feetToMeters(13.87);
        //Slew rates.
        public static final double kTranslationSlewRate = 1.5;
        public static final double kRotationSlewRate = 1.0;
        /////////////////////
        //Encoder Constants//
        /////////////////////
        public static final double kWheelDiameter = Units.inchesToMeters(6);
        public static final double kEncoderDistanceConversionFactor = Math.PI * kWheelDiameter * (12 / 72);
        public static final double kEncoderVelocityConversionFactor = 12 / 72;
    }
    //Constants for the Pigeon2 IMU, such as the ID and various configuration settings.
    public static class IMUConstants
    {
        //ID of the Pigeon2.
        public static final int kPigeon2ID = 1;
        public static final String kPigeon2CANBus = "ctre";
        //Whether the IMU should default to future (potentially unsupported) configs.
        public static final boolean kFutureProofConfigs = false;
        //IMU trim for readings.
        public static final double kTrimX = 0.0;
        public static final double kTrimY = 0.0;
        public static final double kTrimZ = 0.0;
        //Mounting position in degrees.
        public static final double kMountPitch = 0.0;
        public static final double kMountRoll = 0.0;
        public static final double kMountYaw = 0.0;
        //Whether specific features should be enabled. (keep them to false)
        public static final boolean kDisableNoMotionCalibration = false;
        public static final boolean kDisableTemperatureCompensation = false;
    }
    //Constants for the intake.
    public static class IntakeConstants
    {
        //Motor constants.
        public static final int kRollerMotor = 6;
        public static final int kArmMotor = 7;
        //PID Constants for the arm.
        public static final double kArmP = 1.0;
        public static final double kArmI = 0.0;
        public static final double kArmD = 0.5;
        
        public static final double kArmDeployedSetpoint = 57.4;

        public static final double kRollerFF = 1.0;
        public static final double kRollerSetpoint = 4000;
        public static final double kNoteInDistance = 6;
    }

    //Constants for the flywheel.
    public static class FlywheelConstants
    {
        //Fly Wheel Motor Id's
        public static final int kTopFlyWheel = 8;
        public static final int kBottomFlyWheel = 9;
        //Inverting Fly Wheel
        public static final boolean kInvertTopFlyWheel = false;
        public static final boolean kInvertBottomFlyWheel = true;
        //Setpoint
        public static final double kSetPoint = 3000;
    }

    public static class ClimberConstants
    {
        public static final int kLeftClimber = 10;
        public static final int kRightClimber = 11;

        public static final boolean kInvertLeftSide = true;
        public static final boolean kInvertRightSide = false;
    }

    //Constants for kinematics.
    public static class KinematicsConstants
    {
        //Offset from the center of the robot to a wheel.
        public static final double kFrontOffset = Units.inchesToMeters(14.838) / 2;
        public static final double kBackOffset = Units.inchesToMeters(14.547) / 2;
        //Translation2d offsets for each wheel.
        public static final Translation2d kFrontLeftOffset = new Translation2d(kFrontOffset, kFrontOffset);
        public static final Translation2d kFrontRightOffset = new Translation2d(kFrontOffset, -kFrontOffset);
        public static final Translation2d kBackLeftOffset = new Translation2d(-kBackOffset, kBackOffset);
        public static final Translation2d kBackRightOffset = new Translation2d(-kBackOffset, -kBackOffset);
        //Actual kinematics object for performing calculations.
        public static final MecanumDriveKinematics kDriveKinematics = new MecanumDriveKinematics(
            kFrontLeftOffset,
            kFrontRightOffset, 
            kBackLeftOffset,
            kBackRightOffset);
    }
}
