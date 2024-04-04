package ravenrobotics.shootloops.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import ravenrobotics.shootloops.Constants.DrivetrainConstants;
import ravenrobotics.shootloops.Constants.KinematicsConstants;
import ravenrobotics.shootloops.Constants.MotorConstants;
import ravenrobotics.shootloops.util.Telemetry;

public class DriveSubsystem extends SubsystemBase
{
    //Drivetrain motors. Configured for use with a NEO motor, which is brushless.
    private final CANSparkMax frontLeft = new CANSparkMax(DrivetrainConstants.kFrontLeftMotor, MotorType.kBrushless);
    private final CANSparkMax frontRight = new CANSparkMax(DrivetrainConstants.kFrontRightMotor, MotorType.kBrushless);
    private final CANSparkMax backLeft = new CANSparkMax(DrivetrainConstants.kBackLeftMotor, MotorType.kBrushless);
    private final CANSparkMax backRight = new CANSparkMax(DrivetrainConstants.kBackRightMotor, MotorType.kBrushless);

    //Drivetrain encoders.
    private final RelativeEncoder frontLeftEncoder = frontLeft.getEncoder();
    private final RelativeEncoder frontRightEncoder = frontRight.getEncoder();
    private final RelativeEncoder backLeftEncoder = backLeft.getEncoder();
    private final RelativeEncoder backRightEncoder = backRight.getEncoder();
    
    //Battery voltage
    private GenericEntry batteryVoltage = Telemetry.teleopTab.add("Battery Voltage", 12).getEntry();

    //SysID variables for measuring the voltage, distance, and velocity.
    private final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Angle> drivenDistance = mutable(Rotations.of(0));
    private final MutableMeasure<Velocity<Angle>> distanceVelocity = mutable(RotationsPerSecond.of(0));

    //SysID mechanism for applying the tests.
    private final SysIdRoutine.Mechanism sysIDMechanism = new SysIdRoutine.Mechanism(
        (Measure<Voltage> voltage) ->
        {
            //Set the motor power to the new voltages.
            frontLeft.set(voltage.in(Volts) / RobotController.getBatteryVoltage());
            frontRight.set(voltage.in(Volts) / RobotController.getBatteryVoltage());
            backLeft.set(voltage.in(Volts) / RobotController.getBatteryVoltage());
            backRight.set(voltage.in(Volts) / RobotController.getBatteryVoltage());
        },
        log -> 
        {
            //Log front left motor voltage, position, and velocity.
            log.motor("frontLeft")
                .voltage(appliedVoltage.mut_replace(frontLeft.get() * RobotController.getBatteryVoltage(), Volts))
                .angularPosition(drivenDistance.mut_replace(frontLeftEncoder.getPosition(), Rotations))
                .angularVelocity(distanceVelocity.mut_replace(frontLeftEncoder.getVelocity(), RotationsPerSecond));
            //Log front right motor voltage, position, and velocity.
            log.motor("frontRight")
                .voltage(appliedVoltage.mut_replace(frontLeft.get() * RobotController.getBatteryVoltage(), Volts))
                .angularPosition(drivenDistance.mut_replace(frontRightEncoder.getPosition(), Rotations))
                .angularVelocity(distanceVelocity.mut_replace(frontRightEncoder.getVelocity(), RotationsPerSecond));
            //Log back left motor voltage, position, and velocity.
            log.motor("backLeft")
                .voltage(appliedVoltage.mut_replace(backLeft.get() * RobotController.getBatteryVoltage(), Volts))
                .angularPosition(drivenDistance.mut_replace(backLeftEncoder.getPosition(), Rotations))
                .angularVelocity(distanceVelocity.mut_replace(backRightEncoder.getVelocity(), RotationsPerSecond));
            //Log back right motor voltage, position, and velocity.
            log.motor("backRight")
                .voltage(appliedVoltage.mut_replace(backRight.get() * RobotController.getBatteryVoltage(), Volts))
                .angularPosition(drivenDistance.mut_replace(backRightEncoder.getPosition(), Rotations))
                .angularVelocity(distanceVelocity.mut_replace(backRightEncoder.getVelocity(), RotationsPerSecond));
        }, this);

    //SysID Routine for creating the commands to run the routines.
    private final SysIdRoutine sysIDRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        sysIDMechanism);

    //Odometry-related things.
    private final MecanumDriveOdometry driveOdometry;
    private Pose2d drivetrainPose;
    private Field2d fieldData = new Field2d();

    //Variable for whether the robot is climbing, so the drivebase is disabled.
    private boolean isClimbing = false;
    //The center of rotation (for orbiting on a specific wheel to avoid another bot).
    private Translation2d centerOfRotation = new Translation2d();

    //Whether the motors should be in brake mode.
    private boolean isBrakeMode = false;
    //Manual braking commanded from the driver.
    private boolean manualBrakeMode = false;

    //Brake mode entry in Shuffleboard.
    private final GenericEntry brakeModeEntry = Telemetry.teleopTab.add("Brake Mode", false).getEntry();

    //The main data log.
    private final DataLog log;

    //Log entries for the individual wheel speeds.
    private final DoubleLogEntry frontLeftSpeed;
    private final DoubleLogEntry frontRightSpeed;
    private final DoubleLogEntry backLeftSpeed;
    private final DoubleLogEntry backRightSpeed;

    //Log entries for the actual ChassisSpeeds and the target ChassisSpeeds.
    private final StringLogEntry chassisSpeedsLog;
    private final StringLogEntry chassisSpeedInputLog;
    //Log entry for the values measured from the IMU's accelerometer (testing).
    private final DoubleArrayLogEntry imuSpeedsLog;

    //Instance object for simplifying getting the subsystem for commands.
    private static DriveSubsystem instance;

    /**
     * Get the active instance of DriveSubsystem.
     * 
     * @return The DriveSubsystem instance.
     */
    public static DriveSubsystem getInstance()
    {
        //If the instance doesn't exist yet, create it.
        if (instance == null)
        {
            System.out.println("Creating DriveSubsystem instance.");
            instance = new DriveSubsystem();
        }
        //Return the instance.
        return instance;
    }

    /**
     * Subsystem for controlling the drivetrain of the robot.
     */
    private DriveSubsystem()
    {
        //Configure the motors and encoders for use.
        configMotors();

        //Initialize odometry for use.
        driveOdometry = new MecanumDriveOdometry(KinematicsConstants.kDriveKinematics,
        IMUSubsystem.getInstance().getYaw(),
        getWheelPositions());

        //Add the Field2d widget to Shuffleboard so we can see the robot's position.
        Telemetry.teleopTab.add("Robot Position", fieldData);

        //Get the active data log instance.
        log = DataLogManager.getLog();

        //Instantiate the entries for the wheel speeds.
        frontLeftSpeed = new DoubleLogEntry(log, "/drive/fLSpeed");
        frontRightSpeed = new DoubleLogEntry(log, "/drive/fRSpeed");
        backLeftSpeed = new DoubleLogEntry(log, "/drive/bLSpeed");
        backRightSpeed = new DoubleLogEntry(log, "/drive/bRSpeed");

        //Instantiate the entries for the actual ChassisSpeeds entries and the IMU speeds.
        chassisSpeedsLog = new StringLogEntry(log, "/drive/chassisSpeed");
        imuSpeedsLog = new DoubleArrayLogEntry(log, "/imu/imuSpeeds");

        //Instantiate the entries for the target ChassisSpeeds provided by driver input.
        chassisSpeedInputLog = new StringLogEntry(log, "/drive/chassisInput");
    }

    /**
     * Configures PathPlanner to use the drivetrain.
     */
    public void configPathPlanner()
    {
        //Configure PathPlanner's auto builder so autos can be automatically built.
        AutoBuilder.configureRamsete(
            //Gives the robot pose as a Pose2d.
            this::getRobotPose,
            //Resets the robot pose to a Pose2d (the beginning of the path).
            this::resetRobotPose,
            //Gets the robot speed so it can drive it per limitations.
            this::getRobotSpeed,
            //Feeds a ChassisSpeed object to the drivetrain for actually driving the path.
            this::drive,
            new ReplanningConfig(false, false),
            //Check if the path should get flipped if we are on Red Alliance.
            () ->
            {
                //Get the current alliance.
                var alliance = DriverStation.getAlliance();
                //Check if there is an alliance present, then check if it is Red.
                if (alliance.isPresent())
                {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                //If the alliance is blue alliance, don't mirror the path.
                return false;
            },

            //Add the subsystem as a requirement for the AutoBuilder commands.
            instance
        );
    }

    /**
     * Drive the drivetrain.
     * 
     * @param speeds The target speed of the drivetrain as a ChassisSpeeds object.
     */
    public void drive(ChassisSpeeds speeds)
    {
        //If the robot is climbing, don't set wheel speeds.
        if (isClimbing) return;

        //Append the target ChassisSpeeds to the log.
        chassisSpeedInputLog.append(speeds.toString());

        //If we are making a major turn, set the motors to brake mode.
        if (Math.abs(speeds.omegaRadiansPerSecond) > 0.1)
        {
            motorsToBrake();
        }
        else
        {
            //If we are not making a major turn, set the motors back to coast mode.
            motorsToCoast();
        }

        //If we are manually braking, set the motors to brake mode.
        if (manualBrakeMode)
        {
            motorsToBrake();
        }

        //Convert the ChassisSpeeds to individual wheel speeds.
        MecanumDriveWheelSpeeds wheelSpeeds = KinematicsConstants.kDriveKinematics.toWheelSpeeds(speeds, centerOfRotation);
        wheelSpeeds.desaturate(DrivetrainConstants.kDriveMaxSpeedMPS);
        
        //Convert the speeds into the range for the motors, then set them.
        frontLeft.set(wheelSpeeds.frontLeftMetersPerSecond / DrivetrainConstants.kDriveMaxSpeedMPS);
        frontRight.set(wheelSpeeds.frontRightMetersPerSecond / DrivetrainConstants.kDriveMaxSpeedMPS);
        backLeft.set(wheelSpeeds.rearLeftMetersPerSecond / DrivetrainConstants.kDriveMaxSpeedMPS);
        backRight.set(wheelSpeeds.rearRightMetersPerSecond / DrivetrainConstants.kDriveMaxSpeedMPS);

        //Update the odometry since we are changing position.
        drivetrainPose = driveOdometry.update(
            IMUSubsystem.getInstance().getYaw(),
            getWheelPositions()
        );

        //Log the individual wheel speeds to the log.
        frontLeftSpeed.append(wheelSpeeds.frontLeftMetersPerSecond);
        frontRightSpeed.append(wheelSpeeds.frontRightMetersPerSecond);
        backLeftSpeed.append(wheelSpeeds.rearLeftMetersPerSecond);
        backRightSpeed.append(wheelSpeeds.rearRightMetersPerSecond);
    }

    /**
     * Sets the center of rotation for the drivetrian kinematics.
     * 
     * @param centerOfRotation The new center of rotation as a Translation2d.
     */
    public void setCenterOfRotation(Translation2d centerOfRotation)
    {
        this.centerOfRotation = centerOfRotation;
    }

    /**
     * Resets the center of rotation to the center.
     */
    public void resetCenterofRotation()
    {
        this.centerOfRotation = new Translation2d();
    }

    /**
     * Sets the drivetrain to the climbing mode (no moving).
     */
    public void isClimbing()
    {
        isClimbing = true;
    }

    /**
     * Sets the drivetrain to not be in climbing mode (moving allowed).
     */
    public void isNotClimbing()
    {
        isClimbing = false;
    }

    /**
     * Returns the robot's speed as a ChassisSpeeds.
     * 
     * @return The robot's speed as a ChassisSpeeds.
     */
    public ChassisSpeeds getRobotSpeed()
    {
        //Create a MecanumDriveWheelSpeeds object from the encoder speeds.
        var speeds = new MecanumDriveWheelSpeeds(
            frontLeftEncoder.getVelocity(),
            frontRightEncoder.getVelocity(),
            backLeftEncoder.getVelocity(),
            backRightEncoder.getVelocity()
        );

        //Construct a ChassisSpeeds of the robot's overall speed from the wheel speeds, then return it.
        ChassisSpeeds chassisSpeeds = KinematicsConstants.kDriveKinematics.toChassisSpeeds(speeds);
        return chassisSpeeds;
    }

    /**
     * Returns the positions of the drivetrain wheels.
     * 
     * @return The positions of the wheels as a MecanumDriveWhelPositions.
     */
    public MecanumDriveWheelPositions getWheelPositions()
    {
        return new MecanumDriveWheelPositions(
            frontLeftEncoder.getPosition(),
            frontRightEncoder.getPosition(),
            backLeftEncoder.getPosition(),
            backRightEncoder.getPosition()
        );
    }

    /**
     * Returns the drivetrain's pose as a Pose2d.
     * 
     * @return The drivetrain's pose as a Pose2d.
     */
    public Pose2d getRobotPose()
    {
        return drivetrainPose;
    }

    /**
     * Resets the robot's odometry to a new position.
     * 
     * @param newPose The robot's new position as a Pose2d.
     */
    public void resetRobotPose(Pose2d newPose)
    {
        //Reset all the drive motor encoders.
        frontLeftEncoder.setPosition(0);
        frontRightEncoder.setPosition(0);
        backLeftEncoder.setPosition(0);
        backRightEncoder.setPosition(0);

        //Get the alliance from the driver station.
        var alliance = DriverStation.getAlliance();

        if (alliance.isPresent())
        {
            //If the alliance is red alliance, set the IMU to the preset pose but unflipped.
            if (alliance.get() == Alliance.Red) IMUSubsystem.getInstance().resetYaw(newPose.getRotation().getDegrees() - 180);
            //If the alliance is blue alliance, set the IMU to the preset pose.
            else IMUSubsystem.getInstance().resetYaw(newPose.getRotation().getDegrees());
        }
        else
        {
            //If the alliance isn't "present", set the IMU to the pose.
            IMUSubsystem.getInstance().resetYaw(newPose.getRotation().getDegrees());
        }

        //Set the drivetrainPose to the new pose.
        drivetrainPose = newPose;
        //Reset the odometry to the new pose.
        driveOdometry.resetPosition(IMUSubsystem.getInstance().getYaw(), getWheelPositions(), newPose);
    }

    /**
     * Immediately stops all of the drive motors.
     */
    public void stopMotors()
    {
        frontLeft.stopMotor();
        frontRight.stopMotor();
        backLeft.stopMotor();
        backRight.stopMotor();
    }

    /**
     * Sets the drivetrain motors to brake mode.
     */
    public void motorsToBrake()
    {
        isBrakeMode = true;

        frontLeft.setIdleMode(IdleMode.kBrake);
        frontRight.setIdleMode(IdleMode.kBrake);
        backLeft.setIdleMode(IdleMode.kBrake);
        backRight.setIdleMode(IdleMode.kBrake);
    }

    /**
     * Sets the drivetrain motors to coast mode.
     */
    public void motorsToCoast()
    {
        isBrakeMode = false;

        frontLeft.setIdleMode(IdleMode.kCoast);
        frontRight.setIdleMode(IdleMode.kCoast);
        backLeft.setIdleMode(IdleMode.kCoast);
        backRight.setIdleMode(IdleMode.kCoast);
    }

    /**
     * Turns on the manual brake.
     */
    public void manualBrakeOn()
    {
        manualBrakeMode = true;
    }

    /**
     * Turns off the manual brake.
     */
    public void manualBrakeOff()
    {
        manualBrakeMode = false;
    }

    /**
     * Get the quasistatic SysID command.
     * 
     * @param direction The direction to run the motors.
     * @return The command to schedule.
     */
    public Command getSysIDQuasistatic(SysIdRoutine.Direction direction)
    {
        return sysIDRoutine.quasistatic(direction);
    }

    /**
     * Get the dynamic SysID command.
     * 
     * @param direction The direction to run the motors.
     * @return The command to schedule.
     */
    public Command getSysIDDynamic(SysIdRoutine.Direction direction)
    {
        return sysIDRoutine.dynamic(direction);
    }

    @Override
    public void periodic()
    {
        //Update the battery voltage on telemetry.
        batteryVoltage.setDouble(RobotController.getBatteryVoltage());

        //Get the IMU accelerometer speeds.
        double[] imuSpeeds = {IMUSubsystem.getInstance().getXSpeed(), IMUSubsystem.getInstance().getYSpeed(), IMUSubsystem.getInstance().getZSpeed()};

        //Append the acclerometer speeds to the log.
        imuSpeedsLog.append(imuSpeeds);
        //Set the Shufflebo
        brakeModeEntry.setBoolean(isBrakeMode);

        //Update the odometry.
        drivetrainPose = driveOdometry.update(
            IMUSubsystem.getInstance().getYaw(),
            getWheelPositions()
        );

        //Append the actual robot speed to the log.
        chassisSpeedsLog.append(getRobotSpeed().toString());

        //Update the robot pose on the field widget.
        fieldData.setRobotPose(drivetrainPose);
    }

    /**
     * Configure the drivetrain motors for use.
     */
    private void configMotors()
    {
        //Restore all the motors to factory defaults, so that we can start fresh and nothing interferes.
        frontLeft.restoreFactoryDefaults();
        frontRight.restoreFactoryDefaults();
        backLeft.restoreFactoryDefaults();
        backRight.restoreFactoryDefaults();

        //Reverse the default direction of the back side so everything drives normally.
        frontLeft.setInverted(DrivetrainConstants.kInvertFrontLeftSide);
        backLeft.setInverted(DrivetrainConstants.kInvertBackLeftSide);
        frontRight.setInverted(DrivetrainConstants.kInvertFrontRightSide);
        backRight.setInverted(DrivetrainConstants.kInvertBackRightSide);

        //Set the idle mode to brake so that the robot does a better job of staying in place.
        frontLeft.setIdleMode(IdleMode.kCoast);
        frontRight.setIdleMode(IdleMode.kCoast);
        backLeft.setIdleMode(IdleMode.kCoast);
        backRight.setIdleMode(IdleMode.kCoast);

        //Set the current limits so that stuff DOESN'T break.
        frontLeft.setSmartCurrentLimit(MotorConstants.kAmpFreeLimit);
        frontRight.setSmartCurrentLimit(MotorConstants.kAmpFreeLimit);
        backLeft.setSmartCurrentLimit(MotorConstants.kAmpFreeLimit);
        backRight.setSmartCurrentLimit(MotorConstants.kAmpFreeLimit);

        //Reset the encoder positions to 0.
        frontLeftEncoder.setPosition(0);
        frontRightEncoder.setPosition(0);
        backLeftEncoder.setPosition(0);
        backRightEncoder.setPosition(0);

        //Set the position conversion factor so that the motors report an accurate distance.
        frontLeftEncoder.setPositionConversionFactor(DrivetrainConstants.kEncoderConversionFactor);
        frontRightEncoder.setPositionConversionFactor(DrivetrainConstants.kEncoderConversionFactor);
        backLeftEncoder.setPositionConversionFactor(DrivetrainConstants.kEncoderConversionFactor);
        backRightEncoder.setPositionConversionFactor(DrivetrainConstants.kEncoderConversionFactor);

        //Set the velocity conversion factor so that the motors report an accurate velocity.
        frontLeftEncoder.setVelocityConversionFactor(DrivetrainConstants.kEncoderConversionFactor);
        frontRightEncoder.setVelocityConversionFactor(DrivetrainConstants.kEncoderConversionFactor);
        backLeftEncoder.setVelocityConversionFactor(DrivetrainConstants.kEncoderConversionFactor);
        backRightEncoder.setVelocityConversionFactor(DrivetrainConstants.kEncoderConversionFactor);

        //Save the configuration to the motors.
        frontLeft.burnFlash();
        backLeft.burnFlash();
        frontRight.burnFlash();
        backRight.burnFlash();
    }
}
