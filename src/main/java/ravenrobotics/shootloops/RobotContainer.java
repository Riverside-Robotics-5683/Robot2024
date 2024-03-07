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
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import ravenrobotics.shootloops.Constants.DriverStationConstants;
import ravenrobotics.shootloops.commands.DriveCommand;
import ravenrobotics.shootloops.commands.RunFlywheelCommand;
import ravenrobotics.shootloops.commands.autos.DriveForwardAuto;
import ravenrobotics.shootloops.commands.autos.ppcommands.*;
import ravenrobotics.shootloops.subsystems.ClimberSubsystem;
import ravenrobotics.shootloops.subsystems.DriveSubsystem;
import ravenrobotics.shootloops.subsystems.IMUSubsystem;
import ravenrobotics.shootloops.subsystems.IntakeSubsystem;
import ravenrobotics.shootloops.subsystems.RGBSubsystem;
import ravenrobotics.shootloops.subsystems.IntakeSubsystem.IntakeArmPosition;
import ravenrobotics.shootloops.subsystems.RGBSubsystem.RGBValues;
import ravenrobotics.shootloops.util.Telemetry;

public class RobotContainer 
{
  //Driver controller (drives the robot around).
  //private final CommandXboxController driverController = new CommandXboxController(DriverStationConstants.kDriverPort);
  private final CommandJoystick driverJoystick = new CommandJoystick(DriverStationConstants.kDriverPort);
  private final CommandXboxController systemsController = new CommandXboxController(DriverStationConstants.kSystemsPort);

  //Whether to drive field relative or not.
  public boolean isFieldRelative = false;
  private GenericEntry isFieldRelativeEntry = Telemetry.teleopTab.add("Field Relative", false).getEntry();

  private final RunFlywheelCommand flywheelCommand = new RunFlywheelCommand();

  private final SendableChooser<Command> autoChooser;

  //Main drive command.
  private final DriveCommand driveCommand = new DriveCommand(
    () -> -driverJoystick.getY(),
    () -> -driverJoystick.getX(),
    () -> -driverJoystick.getZ(),
    () -> isFieldRelative);

  public RobotContainer()
  {
    DriveSubsystem.getInstance().configPathPlanner();

    NamedCommands.registerCommand("lineUpSpeaker", new LineUpWithSpeakerCommand());
    NamedCommands.registerCommand("shootNote", new AutoShootCommand());
    NamedCommands.registerCommand("intakeNote", new IntakeNoteCommand());

    RGBSubsystem.getInstance().setPattern(RGBValues.kDefault);

    autoChooser = AutoBuilder.buildAutoChooser();

    autoChooser.addOption("Drive Forward", new DriveForwardAuto());

    Telemetry.teleopTab.add("Auto Chooser", autoChooser);

    ClimberSubsystem.getInstance();
    
    //Configure configured controller bindings.
    configureBindings();
    DriveSubsystem.getInstance().setDefaultCommand(driveCommand);
  }

  private void configureBindings()
  {
    //Set the buttons on the joystick for field-relative and zeroing the heading.
    driverJoystick.button(2).onTrue(new InstantCommand(() -> toggleFieldRelative()));
    driverJoystick.button(12).onTrue(new InstantCommand(() -> IMUSubsystem.getInstance().zeroYaw()));

    driverJoystick.button(5).onTrue(new InstantCommand(() -> ClimberSubsystem.getInstance().leftIncrement(2)));
    driverJoystick.button(3).onTrue(new InstantCommand(() -> ClimberSubsystem.getInstance().leftIncrement(-2)));

    driverJoystick.button(6).onTrue(new InstantCommand(() -> ClimberSubsystem.getInstance().rightIncrement(2)));
    driverJoystick.button(4).onTrue(new InstantCommand(() -> ClimberSubsystem.getInstance().rightIncrement(-2)));

    driverJoystick.pov(0).onTrue(new InstantCommand(() -> ClimberSubsystem.getInstance().bothUp()));
    driverJoystick.pov(180).onTrue(new InstantCommand(() -> ClimberSubsystem.getInstance().bothDown()));

    driverJoystick.button(11).onTrue(new InstantCommand(() -> RGBSubsystem.getInstance().setPattern(RGBValues.kTest)));

    // driverJoystick.button(7).onTrue(DriveSubsystem.getInstance().getSysIDDynamic(Direction.kForward));
    // driverJoystick.button(8).onTrue(DriveSubsystem.getInstance().getSysIDDynamic(Direction.kReverse));
    // driverJoystick.button(9).onTrue(DriveSubsystem.getInstance().getSysIDQuasistatic(Direction.kForward));
    // driverJoystick.button(10).onTrue(DriveSubsystem.getInstance().getSysIDQuasistatic(Direction.kReverse));
    
    systemsController.x().onTrue(new InstantCommand(() -> IntakeSubsystem.getInstance().setIntakePosition(IntakeArmPosition.kDeployed)));
    systemsController.b().onTrue(new InstantCommand(() -> IntakeSubsystem.getInstance().setIntakePosition(IntakeArmPosition.kRetracted)));

    systemsController.leftTrigger().onTrue(new InstantCommand(() -> IntakeSubsystem.getInstance().intakeRunRollers()));
    systemsController.leftTrigger().onFalse(new InstantCommand(() -> IntakeSubsystem.getInstance().stopRollers()));

    systemsController.rightTrigger().onTrue(flywheelCommand);
    systemsController.rightTrigger().onFalse(new InstantCommand(() -> flywheelCommand.cancel()));
  }

  public void setupTeleop()
  {
    // ClimberSubsystem.getInstance().leftDown();
    // ClimberSubsystem.getInstance().rightDown();
  }

  private void toggleFieldRelative()
  {
    //Toggle field relative (if true set false, if false set true)
    if (isFieldRelative)
    {
      isFieldRelative = false;
      isFieldRelativeEntry.setBoolean(isFieldRelative);
    }
    else
    {
      isFieldRelative = true;
      isFieldRelativeEntry.setBoolean(isFieldRelative);
    }
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
