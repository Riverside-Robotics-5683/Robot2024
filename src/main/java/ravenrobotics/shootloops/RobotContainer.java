// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package ravenrobotics.shootloops;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import ravenrobotics.shootloops.Constants.DriverStationConstants;
import ravenrobotics.shootloops.Constants.KinematicsConstants;
import ravenrobotics.shootloops.commands.DriveCommand;
import ravenrobotics.shootloops.commands.IntakeRoutineCommand;
import ravenrobotics.shootloops.commands.RunFlywheelCommand;
import ravenrobotics.shootloops.commands.autos.DriveForCommand;
import ravenrobotics.shootloops.commands.autos.DriveForwardAuto;
import ravenrobotics.shootloops.commands.autos.ppcommands.*;
import ravenrobotics.shootloops.subsystems.ClimberSubsystem;
import ravenrobotics.shootloops.subsystems.ClimberSubsystem.ClimberDirection;
import ravenrobotics.shootloops.subsystems.DriveSubsystem;
import ravenrobotics.shootloops.subsystems.FlywheelSubsystem;
import ravenrobotics.shootloops.subsystems.IMUSubsystem;
import ravenrobotics.shootloops.subsystems.IntakeSubsystem;
import ravenrobotics.shootloops.subsystems.IntakeSubsystem.IntakeArmPosition;
import ravenrobotics.shootloops.subsystems.RGBSubsystem;
import ravenrobotics.shootloops.subsystems.RGBSubsystem.RGBValues;
import ravenrobotics.shootloops.util.Telemetry;

public class RobotContainer {

    //Driver controller (drives the robot around).
    private final CommandXboxController driverController =
        new CommandXboxController(DriverStationConstants.kDriverPort);
    private final CommandXboxController systemsController =
        new CommandXboxController(DriverStationConstants.kSystemsPort);

    //Whether to drive field relative or not.
    public boolean isFieldRelative = true;
    private final GenericEntry isFieldRelativeEntry = Telemetry.teleopTab
        .add("Field Relative", true)
        .getEntry();

    //The commands for running the flywheel and automatically intaking.
    private final RunFlywheelCommand flywheelCommand = new RunFlywheelCommand();
    private final IntakeRoutineCommand intakeCommand =
        new IntakeRoutineCommand();

    //Chooser on the dashboard for autos.
    private final SendableChooser<Command> autoChooser;

    private final SendableChooser<Command> sysIdChooser;

    /**
     * The container for setting everything driver- and auto-related.
     */
    public RobotContainer() {
        //Configure PathPlanner.
        DriveSubsystem.getInstance().configPathPlanner();
        //Initialize the climber subsystem.
        ClimberSubsystem.getInstance();

        //Register the speaker/note related auto commands.
        NamedCommands.registerCommand(
            "lineUpSpeaker",
            new LineUpWithSpeakerCommand()
        );
        NamedCommands.registerCommand("shootNote", new AutoShootCommand());
        NamedCommands.registerCommand(
            "intakeNote",
            new IntakeNoteCommand().withTimeout(3)
        );

        //Register the IMU-related auto commands.
        NamedCommands.registerCommand(
            "imuLeft",
            new InstantCommand(() ->
                IMUSubsystem.getInstance().setYawToLeftSubwoofer()
            )
        );
        NamedCommands.registerCommand(
            "imuRight",
            new InstantCommand(() ->
                IMUSubsystem.getInstance().setYawToRightSubwoofer()
            )
        );

        //Register the note-related auto commands.
        NamedCommands.registerCommand(
            "intakeOut",
            new SetIntakeCommand(IntakeArmPosition.kIntake)
        );
        NamedCommands.registerCommand(
            "intakeIn",
            new SetIntakeCommand(IntakeArmPosition.kRetracted)
        );

        //Set the RGB pattern to the default pattern.
        RGBSubsystem.getInstance().setPattern(RGBValues.kDefault);

        //Initialize the auto chooser with all of the PathPlanner autos.
        autoChooser = AutoBuilder.buildAutoChooser();

        sysIdChooser = new SendableChooser<Command>();

        sysIdChooser.setDefaultOption(
            "Quasi Forward",
            FlywheelSubsystem.getInstance().sysIdQuasistatic(Direction.kForward)
        );
        sysIdChooser.addOption(
            "Quasi Backward",
            FlywheelSubsystem.getInstance().sysIdQuasistatic(Direction.kReverse)
        );
        sysIdChooser.addOption(
            "Dynamic Forward",
            FlywheelSubsystem.getInstance().sysIdDynamic(Direction.kForward)
        );
        sysIdChooser.addOption(
            "Dynamic Backward",
            FlywheelSubsystem.getInstance().sysIdDynamic(Direction.kReverse)
        );

        //Send the chooser to the dashboard.
        Telemetry.teleopTab.add("Auto Chooser", autoChooser);
        Telemetry.teleopTab.add("SysID", sysIdChooser);

        //Configure configured controller bindings.
        configureBindings();
        //Set the default command for the drive subsystem to the drive command.
        //Main drive command.
        DriveCommand driveCommand = new DriveCommand(
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            () -> isFieldRelative
        );
        DriveSubsystem.getInstance().setDefaultCommand(driveCommand);
    }

    private void configureBindings() {
        //Set the buttons on the driver controller for field-relative and zeroing the heading.
        driverController
            .back()
            .onTrue(new InstantCommand(this::toggleFieldRelative));
        driverController
            .start()
            .onTrue(
                new InstantCommand(() -> IMUSubsystem.getInstance().zeroYaw())
            );

        //Set the ABXY array on the driver controller for switching the center of rotation.
        driverController
            .y()
            .onTrue(
                new InstantCommand(() ->
                    DriveSubsystem.getInstance()
                        .setCenterOfRotation(
                            KinematicsConstants.kFrontLeftOffset
                        )
                )
            );
        driverController
            .b()
            .onTrue(
                new InstantCommand(() ->
                    DriveSubsystem.getInstance()
                        .setCenterOfRotation(
                            KinematicsConstants.kFrontRightOffset
                        )
                )
            );
        driverController
            .x()
            .onTrue(
                new InstantCommand(() ->
                    DriveSubsystem.getInstance()
                        .setCenterOfRotation(
                            KinematicsConstants.kBackLeftOffset
                        )
                )
            );
        driverController
            .a()
            .onTrue(
                new InstantCommand(() ->
                    DriveSubsystem.getInstance()
                        .setCenterOfRotation(
                            KinematicsConstants.kBackRightOffset
                        )
                )
            );

        //Set the right stick on the driver controller for resetting the center of rotation.
        driverController
            .rightStick()
            .onTrue(
                new InstantCommand(() ->
                    DriveSubsystem.getInstance().resetCenterofRotation()
                )
            );
        //Set the right trigger on the driver controller to turn on the brake.
        driverController
            .rightTrigger()
            .whileTrue(
                new StartEndCommand(
                    () -> DriveSubsystem.getInstance().manualBrakeOn(),
                    () -> DriveSubsystem.getInstance().manualBrakeOff()
                )
            );

        //Set the X and B buttons on the systems controller to automatically run the intake and then manually retract it.
        systemsController.x().onTrue(intakeCommand);
        systemsController.b().onTrue(new InstantCommand(intakeCommand::cancel));

        //Set the left trigger on the systemst controller to manually run the rollers for intaking.
        systemsController
            .leftTrigger()
            .onTrue(
                new InstantCommand(() ->
                    IntakeSubsystem.getInstance().runRollersIntake()
                )
            );
        systemsController
            .leftTrigger()
            .onFalse(
                new InstantCommand(() ->
                    IntakeSubsystem.getInstance().stopRollers()
                )
            );

        //Set the left trigger+y/a on the systems controllerto move the left climber up or down.
        systemsController
            .leftBumper()
            .and(systemsController.y())
            .whileTrue(
                new StartEndCommand(
                    () ->
                        ClimberSubsystem.getInstance()
                            .moveLeft(ClimberDirection.kUp),
                    () -> ClimberSubsystem.getInstance().stopMotors()
                )
            );
        systemsController
            .leftBumper()
            .and(systemsController.a())
            .whileTrue(
                new StartEndCommand(
                    () ->
                        ClimberSubsystem.getInstance()
                            .moveLeft(ClimberDirection.kDown),
                    () -> ClimberSubsystem.getInstance().stopMotors()
                )
            );

        //Set the right trigger+y/a on the systems controller to move the right climber up or down.
        systemsController
            .rightBumper()
            .and(systemsController.y())
            .whileTrue(
                new StartEndCommand(
                    () ->
                        ClimberSubsystem.getInstance()
                            .moveRight(ClimberDirection.kUp),
                    () -> ClimberSubsystem.getInstance().stopMotors()
                )
            );
        systemsController
            .rightBumper()
            .and(systemsController.a())
            .whileTrue(
                new StartEndCommand(
                    () ->
                        ClimberSubsystem.getInstance()
                            .moveRight(ClimberDirection.kDown),
                    () -> ClimberSubsystem.getInstance().stopMotors()
                )
            );

        //Set y/a on the systems controller to move both climbers up or down.
        systemsController
            .rightBumper()
            .negate()
            .and(systemsController.leftBumper().negate())
            .and(systemsController.y())
            .whileTrue(
                new StartEndCommand(
                    () -> ClimberSubsystem.getInstance().bothUp(),
                    () -> ClimberSubsystem.getInstance().stopMotors()
                )
            );
        systemsController
            .rightBumper()
            .negate()
            .and(systemsController.leftBumper().negate())
            .and(systemsController.a())
            .whileTrue(
                new StartEndCommand(
                    () -> ClimberSubsystem.getInstance().bothDown(),
                    () -> ClimberSubsystem.getInstance().stopMotors()
                )
            );

        //Set the right trigger on the systems controller to run the flywheel for shooting a note.
        systemsController.rightTrigger().onTrue(flywheelCommand);
        systemsController
            .rightTrigger()
            .onFalse(new InstantCommand(flywheelCommand::cancel));

        //Set the POV hat on the systems controller to set the intake to the amp position when pressed in the up direction, and to move back to the retracted position when pressed down.
        systemsController
            .pov(0)
            .onTrue(
                new InstantCommand(() ->
                    IntakeSubsystem.getInstance()
                        .setIntakePosition(IntakeArmPosition.kAmp)
                )
            );
        systemsController
            .pov(180)
            .onTrue(
                new InstantCommand(() ->
                    IntakeSubsystem.getInstance()
                        .setIntakePosition(IntakeArmPosition.kRetracted)
                )
            );

        //Set the POV hat on the systems controller to push out the note when pushed right.
        systemsController
            .pov(90)
            .whileTrue(
                new StartEndCommand(
                    () -> IntakeSubsystem.getInstance().runRollersAmp(),
                    () -> IntakeSubsystem.getInstance().stopRollers()
                )
            );
    }

    /**
     * Sets up the robot for teleop.
     */
    public void setupTeleop() {
        //Set the motors to coast mode and turns off the manual brake.
        DriveSubsystem.getInstance().motorsToCoast();
        DriveSubsystem.getInstance().manualBrakeOff();
        // ClimberSubsystem.getInstance().bothDown();
    }

    private void toggleFieldRelative() {
        //Toggle field relative (if true set false, if false, set true)
        if (isFieldRelative) {
            isFieldRelative = false;
            isFieldRelativeEntry.setBoolean(isFieldRelative);
        } else {
            isFieldRelative = true;
            isFieldRelativeEntry.setBoolean(isFieldRelative);
        }
    }

    /**
     * Get the command to run during autonomous.
     *
     * @return The chosen command.
     */
    public Command getAutonomousCommand() {
        //Set the motors to brake mode and turn the manual brake on.
        // DriveSubsystem.getInstance().motorsToBrake();
        // DriveSubsystem.getInstance().manualBrakeOn();
        // //Zero the IMU.
        // IMUSubsystem.getInstance().zeroYaw();
        // //ClimberSubsystem.getInstance().bothDown();
        // //Return the selected command.
        // return autoChooser.getSelected();
        FlywheelSubsystem.getInstance().disableIdle();
        return sysIdChooser.getSelected();
    }
}
