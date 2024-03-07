package ravenrobotics.shootloops.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RGBSubsystem extends SubsystemBase 
{
    private final Spark rgbController = new Spark(0);

    private static RGBSubsystem instance;

    public enum RGBValues
    {
        kDefault(0.81),
        kNoteIn(0.65),
        kTest(-0.55);

        private final double id;
        RGBValues(double id) { this.id = id; }
        public double getValue() { return id; }
    }

    public static RGBSubsystem getInstance()
    {
        if (instance == null)
        {
            System.out.println("Creating RGBSubsystem instance.");
            instance = new RGBSubsystem();
        }

        return instance;
    }

    public void setPattern(RGBValues color)
    {
        System.out.println("Color:" + color.getValue());
        rgbController.set(color.getValue());
    }
}
