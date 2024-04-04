package ravenrobotics.shootloops.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import ravenrobotics.shootloops.Constants.IMUConstants;
import ravenrobotics.shootloops.util.Telemetry;

public class IMUSubsystem extends SubsystemBase 
{
    //Pigeon2 object for actually interfacing with the IMU.
    private final Pigeon2 imu = new Pigeon2(IMUConstants.kPigeon2ID, IMUConstants.kPigeon2CANBus);

    //Instance object for simplifying getting the subsystem in commands.
    private static IMUSubsystem instance;

    //Shuffleboard (telemetry)
    private GenericEntry imuHeading = Telemetry.teleopTab.add("IMU Heading", 0)
        .withWidget(BuiltInWidgets.kGyro).getEntry();

    //The simulated IMU.
    private Pigeon2SimState simIMU = imu.getSimState();

    /**
     * Get the active instance of the IMUSubsystem.
     * @return The active IMUSubsystem instance.
     */
    public static IMUSubsystem getInstance()
    {
        //Create the instance if it doesn't exist yet.
        if (instance == null)
        {
            instance = new IMUSubsystem();
        }
        //Return the instance.
        return instance;
    }

    /**
     * Private initializer for the subsystem; configures the IMU settings then zeros the yaw.
     */
    private IMUSubsystem()
    {
        //Configure IMU.
        configIMU();
        //Zero the heading.
        zeroYaw();
    }

    /**
     * Get the current heading of the IMU.
     * @return The heading as a Rotation2d.
     */
    public Rotation2d getYaw()
    {
        //Get the heading, then convert it to a Rotation2d and return it.
        var heading = imu.getYaw().refresh().getValueAsDouble();
        return Rotation2d.fromDegrees(heading);
    }

    /**
     * Get the current acceleration on the X-axis from the IMU.
     * 
     * @return The acceleration in Gs as a double.
     */
    public double getXSpeed()
    {
        //Get the speed on the x-axis and return it.
        var speed = imu.getAccelerationX().refresh().getValueAsDouble();
        return speed;
    }

    /**
     * Get the current acceleration on the Y-axis from the IMU.
     * 
     * @return The acceleration in Gs as a double.
     */
    public double getYSpeed()
    {
        //Get the speed from the y-axis and return it.
        var speed = imu.getAccelerationY().refresh().getValueAsDouble();
        return speed;
    }
    
    /**
     * Get the current acceleration on the Z-axis from the IMU.
     * 
     * @return The acceleration in Gs as a double.
     */
    public double getZSpeed()
    {
        //Get the speed from the z-axis and return it.
        var speed = imu.getAccelerationZ().refresh().getValueAsDouble();
        return speed;
    }

    /**
     * Set the heading of the IMU to zero.
     */
    public void zeroYaw()
    {
        imu.setYaw(0.0);
    }

    /**
     * Reset the heading of the IMU to a specific angle.
     * 
     * @param angle The angle as a double.
     */
    public void resetYaw(double angle)
    {
        imu.setYaw(angle);
    }

    /**
     * Sets the heading of the IMU to the orientation of the left subwoofer.
     */
    public void setYawToLeftSubwoofer()
    {
        imu.setYaw(-300);
    }

    /**
     * Sets the heading of the IMU to the orientation of the right subwoofer.
     */
    public void setYawToRightSubwoofer()
    {
        imu.setYaw(60.0);
    }

    @Override
    public void periodic()
    {
        //If the yaw is greater than 360 degrees, reset it to the coterminal angle within 0-360 degrees.
        if(getYaw().getDegrees() > 360.0)
        {
            imu.setYaw(getYaw().getDegrees() - 360.0);
        }
        //If the yaw is less than -360 degrees, reset it to the coterminal angle within -360-0 degrees.
        else if(getYaw().getDegrees() < -360.0)
        {
            imu.setYaw(getYaw().getDegrees() + 360.0);
        }

        //Update IMU heading on Shuffleboard
        imuHeading.setDouble(-getYaw().getDegrees());
    }

    /**
     * Configures the IMU for use.
     */
    private void configIMU()
    {
        //Factory reset the Pigeon2.
        imu.getConfigurator().apply(new Pigeon2Configuration());
        var config = new Pigeon2Configuration();

        //Disable using newer (potentially unsupported) configs by default.
        config.FutureProofConfigs = false;
        //IMU trim.
        config.GyroTrim.GyroScalarX = IMUConstants.kTrimX;
        config.GyroTrim.GyroScalarY = IMUConstants.kTrimY;
        config.GyroTrim.GyroScalarZ = IMUConstants.kTrimZ;
        //IMU pose.
        config.MountPose.MountPosePitch = IMUConstants.kMountPitch;
        config.MountPose.MountPoseRoll = IMUConstants.kMountRoll;
        config.MountPose.MountPoseYaw = IMUConstants.kMountYaw;
        //IMU features on/off.
        config.Pigeon2Features.DisableNoMotionCalibration = IMUConstants.kDisableNoMotionCalibration;
        config.Pigeon2Features.DisableTemperatureCompensation = IMUConstants.kDisableTemperatureCompensation;
        //Keep this to true, disables the whole purpose of the IMU otherwise :D.
        config.Pigeon2Features.EnableCompass = true;

        //Apply the config.
        imu.getConfigurator().apply(config);
    }

    @Override
    public void simulationPeriodic()
    {
        //Set the supply voltage to the sim supply voltage.
        simIMU.setSupplyVoltage(RobotController.getBatteryVoltage());
    }
}
