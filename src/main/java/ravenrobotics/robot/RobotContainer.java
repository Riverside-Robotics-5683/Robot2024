// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package ravenrobotics.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import ravenrobotics.robot.Constants.DriverStationConstants;
import ravenrobotics.robot.commands.DriveCommand;
import ravenrobotics.robot.commands.RunFlywheelCommand;
import ravenrobotics.robot.subsystems.DriveSubsystem;
import ravenrobotics.robot.subsystems.FlywheelSubsystem;
import ravenrobotics.robot.subsystems.IMUSubsystem;
import ravenrobotics.robot.subsystems.IntakeSubsystem;
import ravenrobotics.robot.subsystems.IntakeSubsystem.IntakeArmPosition;
import ravenrobotics.robot.util.Telemetry;

public class RobotContainer 
{
  //Driver controller (drives the robot around).
  //private final CommandXboxController driverController = new CommandXboxController(DriverStationConstants.kDriverPort);
  private final CommandJoystick driverJoystick = new CommandJoystick(DriverStationConstants.kDriverPort);
  private final CommandXboxController systemsController = new CommandXboxController(DriverStationConstants.kSystemsPort);

  //Whether to drive field relative or not.
  public boolean isFieldRelative = false;
  private GenericEntry isFieldRelativeEntry = Telemetry.teleopTab.add("Field Relative", false).getEntry();

  private final SendableChooser<Command> teleopModeChooser = new SendableChooser<Command>();

  //Main drive command.
  private final DriveCommand driveCommand = new DriveCommand(
    () -> -driverJoystick.getY(),
    () -> -driverJoystick.getX(),
    () -> -driverJoystick.getZ(),
    () -> -driverJoystick.getThrottle(),
    () -> isFieldRelative);

  public RobotContainer()
  {
    //Add drive command to TeleOp mode chooser.
    teleopModeChooser.addOption("Drive", driveCommand);
    //Put the TeleOp mode chooser on the dashboard.
    Telemetry.teleopTab.add("TeleOp Mode", teleopModeChooser);
    //Configure configured controller bindings.
    configureBindings();
    DriveSubsystem.getInstance().setDefaultCommand(driveCommand);
  }

  private void configureBindings()
  {
    // Shooting: parallel command to run intake rollers and flywheel while trigger held 
    driverJoystick.button(1).whileTrue(new RunFlywheelCommand());


    //Set the buttons on the joystick for field-relative and zeroing the heading.
    driverJoystick.button(2).onTrue(new InstantCommand(() -> toggleFieldRelative()));
    driverJoystick.button(3).onTrue(new InstantCommand(() -> IMUSubsystem.getInstance().zeroYaw()));

    driverJoystick.button(7).onTrue(DriveSubsystem.getInstance().getSysIDDynamic(Direction.kForward));
    driverJoystick.button(8).onTrue(DriveSubsystem.getInstance().getSysIDDynamic(Direction.kReverse));
    driverJoystick.button(9).onTrue(DriveSubsystem.getInstance().getSysIDQuasistatic(Direction.kForward));
    driverJoystick.button(10).onTrue(DriveSubsystem.getInstance().getSysIDQuasistatic(Direction.kReverse));

    driverJoystick.button(5).onTrue(new InstantCommand(() -> IntakeSubsystem.getInstance().setIntakePosition(IntakeArmPosition.kDeployed)));
    driverJoystick.button(4).onTrue(new InstantCommand(() -> IntakeSubsystem.getInstance().setIntakePosition(IntakeArmPosition.kRetracted)));

    driverJoystick.button(6).toggleOnTrue(new InstantCommand(() -> IntakeSubsystem.getInstance().intakeRunRollers()));
    driverJoystick.button(6).toggleOnFalse(new InstantCommand(() -> IntakeSubsystem.getInstance().stopRollers()));
    //systemsController.y().toggleOnTrue(new InstantCommand(() -> IntakeSubsystem.getInstance().runRollers()));
    //systemsController.y().toggleOnFalse(new InstantCommand(() -> IntakeSubsystem.getInstance().stopRollers()));

    //systemsController.back().toggleOnTrue(new InstantCommand(() -> FlywheelSubsystem.getInstance().override()));
   // systemsController.back().toggleOnFalse(new InstantCommand(() -> FlywheelSubsystem.getInstance().unOverride()));
  }

  public void setupTeleopCommand()
  {
    Command selectedCommand = teleopModeChooser.getSelected();
    if (selectedCommand == null)
    {
      DriveSubsystem.getInstance().setDefaultCommand(driveCommand);
      return;
    }
    DriveSubsystem.getInstance().setDefaultCommand(selectedCommand);
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
    return Commands.print("No autonomous command configured");
  }
}
