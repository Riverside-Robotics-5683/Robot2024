package ravenrobotics.shootloops.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import ravenrobotics.shootloops.Constants.FlywheelConstants;
import ravenrobotics.shootloops.Constants.MotorConstants;

public class FlywheelSubsystem extends SubsystemBase {

    //The flywheel motors.
    private final CANSparkMax topMotor = new CANSparkMax(
        FlywheelConstants.kTopFlyWheel,
        MotorType.kBrushless
    );
    private final CANSparkMax bottomMotor = new CANSparkMax(
        FlywheelConstants.kBottomFlyWheel,
        MotorType.kBrushless
    );

    //The encoders
    private final RelativeEncoder topMotorEncoder = topMotor.getEncoder();
    private final RelativeEncoder bottomMotorEncoder = bottomMotor.getEncoder();

    private final BangBangController topIdleController =
        new BangBangController();
    private final BangBangController bottomIdleController =
        new BangBangController();
    private boolean isIdle = true;

    private final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Angle> flywheelDistance = mutable(
        Rotations.of(0)
    );
    private final MutableMeasure<Velocity<Angle>> flywheelVelocity = mutable(
        RotationsPerSecond.of(0)
    );

    private final SysIdRoutine.Mechanism sysIdMechanism =
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> voltage) -> {
                topMotor.set(
                    voltage.in(Volts) / RobotController.getBatteryVoltage()
                );
                bottomMotor.set(
                    voltage.in(Volts) / RobotController.getBatteryVoltage()
                );
            },
            log -> {
                log
                    .motor("topFlywheel")
                    .voltage(
                        appliedVoltage.mut_replace(
                            topMotor.get() *
                            RobotController.getBatteryVoltage(),
                            Volts
                        )
                    )
                    .angularPosition(
                        flywheelDistance.mut_replace(
                            topMotorEncoder.getPosition(),
                            Rotations
                        )
                    )
                    .angularVelocity(
                        flywheelVelocity.mut_replace(
                            topMotorEncoder.getVelocity() / 60,
                            RotationsPerSecond
                        )
                    );
                log
                    .motor("bottomFlywheel")
                    .voltage(
                        appliedVoltage.mut_replace(
                            bottomMotor.get() *
                            RobotController.getBatteryVoltage(),
                            Volts
                        )
                    )
                    .angularPosition(
                        flywheelDistance.mut_replace(
                            bottomMotorEncoder.getPosition(),
                            Rotations
                        )
                    )
                    .angularVelocity(
                        flywheelVelocity.mut_replace(
                            bottomMotorEncoder.getVelocity() / 60,
                            RotationsPerSecond
                        )
                    );
            },
            this
        );

    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        sysIdMechanism
    );

    private final DataLog log;

    private final DoubleLogEntry topSpeedLog;
    private final DoubleLogEntry bottomSpeedLog;

    //Allows you to use FlywheelSubsystem in other classes
    private static FlywheelSubsystem instance;

    /**
     * Returns the active instance of the FlyWheelSubSystem.
     *
     * @return The FlyWheelSubsystem instance.
     */
    public static FlywheelSubsystem getInstance() {
        //If the instance hasn't been created it yet, create it.
        if (instance == null) {
            System.out.println("Creating FlywheelSubsystem object.");
            instance = new FlywheelSubsystem();
        }

        //Return the instance
        return instance;
    }

    private FlywheelSubsystem() {
        topIdleController.setSetpoint(FlywheelConstants.kSetPoint);
        bottomIdleController.setSetpoint(FlywheelConstants.kSetPoint);

        log = DataLogManager.getLog();

        topIdleController.setTolerance(500);
        bottomIdleController.setTolerance(500);

        topSpeedLog = new DoubleLogEntry(log, "/flywheel/topSpeed");
        bottomSpeedLog = new DoubleLogEntry(log, "/flywheel/bottomSpeed");
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    /**
     * Turn on the flywheels for shooting.
     */
    public void shootOn() {
        isIdle = false;
        topMotor.set(1);
        bottomMotor.set(1);
    }

    /**
     * Stop the flywheels.
     */
    public void stopFly() {
        isIdle = true;
    }

    public void disableIdle() {
        isIdle = false;
    }

    /**
     * Get the total velocity of the flywheels.
     *
     * @return The total velocity as a double.
     */
    public double getTotalVelocity() {
        return -(
            topMotorEncoder.getVelocity() +
            bottomMotorEncoder.getVelocity() / (double) 2
        );
    }

    /**
     * Get the velocity of the top flywheel.
     *
     * @return The velocity as a double.
     */
    public double getTopVelocity() {
        return Math.abs(topMotorEncoder.getVelocity());
    }

    /**
     * Get the velocity of the bottom flywheel.
     *
     * @return The velocity as a double.
     */
    public double getBottomVelocity() {
        return Math.abs(bottomMotorEncoder.getVelocity());
    }

    @Override
    public void periodic() {
        if (isIdle) {
            topMotor.set(topIdleController.calculate(getTopVelocity()));
            bottomMotor.set(
                bottomIdleController.calculate(getBottomVelocity())
            );
        }
        topSpeedLog.append(getTopVelocity());
        bottomSpeedLog.append(getBottomVelocity());
    }

    public void configMotors() {
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
        topMotorEncoder.setVelocityConversionFactor(1);
        bottomMotorEncoder.setVelocityConversionFactor(1);

        topMotor.burnFlash();
        bottomMotor.burnFlash();
    }
}
