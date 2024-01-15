package ravenrobotics.robot.util;

import edu.wpi.first.math.MathUtil;
import ravenrobotics.robot.Constants.DrivetrainConstants;

public class Conversions
{
    /**
     * Convert a speed (in meters per second) into a motor voltage.
     * @param metersPerSecond The speed (in meters per second).
     * @param maxVoltage The maximum voltage to use.
     * @return The converted voltage.
     */
    public static double MPSToVoltage(double metersPerSecond, double maxVoltage)
    {
        //Divide the speed by the max speed (from the DrivetrainConstants class) to get that value, then multiply by the max voltage to get the voltage.
        var convertedSpeed = metersPerSecond / DrivetrainConstants.kDriveMaxSpeedMPS * maxVoltage;
        //Clamp the value so that it doesn't overflow/underflow from the max voltage, then return it.
        return MathUtil.clamp(convertedSpeed, -maxVoltage, maxVoltage);
    }

    /**
     * Convert a speed (in meters per second) into a motor voltage.
     * @param metersPerSecond The speed (in meters per second).
     * @param maxVoltage The maximum voltage to use.
     * @param maxSpeedMPS The maximum speed of the mechanism you are converting (in meters per second).
     * @return The converted voltage.
     */
    public static double MPSToVoltage(double metersPerSecond, double maxVoltage, double maxSpeedMPS)
    {
        //Divide the speed by the provided max speed to get that value, then multiply that by the max voltage to get the voltage.
        var convertedSpeed = metersPerSecond / maxSpeedMPS * maxVoltage;
        //Clamp the value so that it doesn't overflow/underflow from the max voltage, then return it.
        return MathUtil.clamp(convertedSpeed, -maxVoltage, maxVoltage);
    }
}
