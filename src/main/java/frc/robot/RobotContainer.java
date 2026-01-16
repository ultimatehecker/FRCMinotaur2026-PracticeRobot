// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DrivetrainCommands;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.GlobalConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.GyroIO;
import frc.robot.subsystems.drivetrain.GyroIOHardware;
import frc.robot.subsystems.drivetrain.ModuleIO;
import frc.robot.subsystems.drivetrain.ModuleIOHardware;

public class RobotContainer {
  private final Drivetrain drivetrain;
  private final CommandXboxController primaryController = new CommandXboxController(0);
  private final LoggedDashboardChooser<Command> autonomousChooser;

  public RobotContainer() {
    switch (GlobalConstants.kCurrentMode) {
      case REAL:
        drivetrain = new Drivetrain(
          new GyroIOHardware(), 
          new ModuleIOHardware(0, DrivetrainConstants.kFrontLeftModuleConstants), 
          new ModuleIOHardware(1, DrivetrainConstants.kFrontRightModuleConstants), 
          new ModuleIOHardware(2, DrivetrainConstants.kBackLeftModuleConstants),
          new ModuleIOHardware(3, DrivetrainConstants.kBackRightModuleConstants)
        );

        break;
      case SIM:
        drivetrain = new Drivetrain(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
        break;
      default:
        drivetrain = new Drivetrain(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
        break;
    }

    autonomousChooser = new LoggedDashboardChooser<>("Auton Choices", AutoBuilder.buildAutoChooser());
    autonomousChooser.addOption("Drivetrain Wheel Radius Characterization", DrivetrainCommands.wheelRadiusCharacterization(drivetrain));
    autonomousChooser.addOption("Drivetrain Simple FF Characterization", DrivetrainCommands.feedforwardCharacterization(drivetrain));
    autonomousChooser.addOption("Drivetrain SysId (Quasistatic Forward)", drivetrain.sysIdQuasistatic(Direction.kForward));
    autonomousChooser.addOption("Drivetrain SysId (Quasistatic Reverse)", drivetrain.sysIdQuasistatic(Direction.kReverse));
    autonomousChooser.addOption("Drivetrain SysId (Dynamic Forward)", drivetrain.sysIdDynamic(Direction.kForward));
    autonomousChooser.addOption("Drivetrain SysId (Dynamic Forward)", drivetrain.sysIdDynamic(Direction.kReverse));

    configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(DrivetrainCommands.joystickDrive(
      drivetrain, 
      () -> -primaryController.getLeftY(), 
      () -> -primaryController.getLeftX(), 
      () -> -primaryController.getRightX()
    ));

    primaryController.a().whileTrue(DrivetrainCommands.joystickDriveAtAngle(
      drivetrain,
      () -> -primaryController.getLeftY(),
      () -> -primaryController.getLeftX(),
      () -> Rotation2d.kZero
    ));

    primaryController.x().onTrue(Commands.runOnce(drivetrain::stopWithX, drivetrain));
    primaryController.b().onTrue(Commands.runOnce(() ->
      drivetrain.setPose(new Pose2d(drivetrain.getPose().getTranslation(), Rotation2d.kZero)), drivetrain).ignoringDisable(true));
  }

  public Command getAutonomousCommand() {
    return autonomousChooser.get();
  }
}
