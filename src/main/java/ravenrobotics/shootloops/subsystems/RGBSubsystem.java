package ravenrobotics.shootloops.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RGBSubsystem extends SubsystemBase 
{
    //The Spark PWM object for controlling the Blinkin.
    private final Spark rgbController = new Spark(0);

    //The instance object for simplifying access to the subsystem.
    private static RGBSubsystem instance;

    /**
     * Different patterns for the robot status.
     */
    public enum RGBValues
    {
        /**
         * The default pattern for normal drivign.
         */
        kDefault(0.81),
        /**
         * The default pattern while a note is in the intake.
         */
        kNoteInDefault(0.65),
        /**
         * The pattern for when the robot is shooting a note.
         */
        kShooting(0.77),
        /**
         * The pattern for when the intake is out.
         */
        kIntakeOut(0.61),
        /**
         * The test pattern.
         */
        kTest(-0.55);

        //Attach values to the enums.
        private final double id;
        RGBValues(double id) { this.id = id; }
        /**
         * Gets the value of the specified enum.
         * 
         * @return The value.
         */
        public double getValue() { return id; }
    }

    /**
     * Gets the active instance of the RGBSubsystem.
     * 
     * @return The active instance.
     */
    public static RGBSubsystem getInstance()
    {
        //If the instance doesn't exist yet, create it.
        if (instance == null)
        {
            //Logging.
            System.out.println("Creating RGBSubsystem instance.");
            instance = new RGBSubsystem();
        }

        //Returns the instance.
        return instance;
    }

    /**
     * Sets the pattern for the RGB to display.
     * 
     * @param color The pattern.
     */
    public void setPattern(RGBValues color)
    {
        //Logging.
        System.out.println("Color:" + color.getValue());
        //Set the PWM value to the value from the color enum.
        rgbController.set(color.getValue());
    }
}
