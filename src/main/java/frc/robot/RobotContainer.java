// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import frc.robot.command_factories.DrivetrainFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.minolib.controller.CommandSimulatedXboxController;
import frc.minolib.controller.SimulatedXboxController;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.GlobalConstants.Mode;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.GyroIO;
import frc.robot.subsystems.drivetrain.GyroIOHardware;
import frc.robot.subsystems.drivetrain.GyroIOSimulation;
import frc.robot.subsystems.drivetrain.ModuleIO;
import frc.robot.subsystems.drivetrain.ModuleIOHardware;
import frc.robot.subsystems.drivetrain.ModuleIOSimulation;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;

public class RobotContainer {
  private final Drivetrain drivetrain;
  private final Vision vision;

  private final CommandSimulatedXboxController primaryController = new CommandSimulatedXboxController(0);
  private final LoggedDashboardChooser<Command> autonomousChooser;

  private SwerveDriveSimulation driveSimulation = null;

  private Drivetrain buildDrivetrain() {
    switch (GlobalConstants.kCurrentMode) {
      case REAL -> {
        return new Drivetrain(
          new GyroIOHardware(), 
          new ModuleIOHardware(0, DrivetrainConstants.kFrontLeftModuleConstants), 
          new ModuleIOHardware(1, DrivetrainConstants.kFrontRightModuleConstants), 
          new ModuleIOHardware(2, DrivetrainConstants.kBackLeftModuleConstants),
          new ModuleIOHardware(3, DrivetrainConstants.kBackRightModuleConstants),
          (pose) -> {}
        );
      }

      case SIM -> {
        this.driveSimulation = new SwerveDriveSimulation(DrivetrainConstants.kMapleSimConfiguration, new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

        return new Drivetrain(
          new GyroIOSimulation(driveSimulation.getGyroSimulation()), 
          new ModuleIOSimulation(driveSimulation.getModules()[0]), 
          new ModuleIOSimulation(driveSimulation.getModules()[1]), 
          new ModuleIOSimulation(driveSimulation.getModules()[2]), 
          new ModuleIOSimulation(driveSimulation.getModules()[3]),
          driveSimulation::setSimulationWorldPose
        );
      }

      default -> {
        return new Drivetrain(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, (pose) -> {});
      }
    }
  }

  public Vision buildAprilTagVision() {
    switch (GlobalConstants.kCurrentMode) {
      case REAL -> {
        return new Vision(
          drivetrain::addVisionMeasurement, 
          new VisionIOPhotonVision(VisionConstants.camera0Name, VisionConstants.robotToCamera0), 
          new VisionIOPhotonVision(VisionConstants.camera1Name, VisionConstants.robotToCamera1)
        );
      }

      case SIM -> {
        return new Vision(drivetrain::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
      }

      default -> {
        return new Vision(drivetrain::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
      }
    }
  }

  public Drivetrain getDrivetrain() {
    return drivetrain;
  }

  public Vision getAprilTagVision() {
    return vision;
  }

  public RobotContainer() {
    drivetrain = buildDrivetrain();
    vision = buildAprilTagVision();

    autonomousChooser = new LoggedDashboardChooser<Command>("Auton Choices", AutoBuilder.buildAutoChooser());
    autonomousChooser.addOption("Drivetrain Wheel Radius Characterization", DrivetrainFactory.wheelRadiusCharacterization(drivetrain));
    autonomousChooser.addOption("Drivetrain Simple FF Characterization", DrivetrainFactory.feedforwardCharacterization(drivetrain));
    autonomousChooser.addOption("Drivetrain SysId (Quasistatic Forward)", drivetrain.sysIdQuasistatic(Direction.kForward));
    autonomousChooser.addOption("Drivetrain SysId (Quasistatic Reverse)", drivetrain.sysIdQuasistatic(Direction.kReverse));
    autonomousChooser.addOption("Drivetrain SysId (Dynamic Forward)", drivetrain.sysIdDynamic(Direction.kForward));
    autonomousChooser.addOption("Drivetrain SysId (Dynamic Reverse)", drivetrain.sysIdDynamic(Direction.kReverse));

    configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(DrivetrainFactory.joystickDrive(
      drivetrain, 
      () -> -primaryController.getLeftY(), 
      () -> -primaryController.getLeftX(), 
      () -> -primaryController.getRightX()
    ));

    primaryController.a().whileTrue(DrivetrainFactory.joystickDriveAtAngle(
      drivetrain,
      () -> -primaryController.getLeftY(),
      () -> -primaryController.getLeftX(),
      () -> drivetrain.computeAngleFromTarget(drivetrain.getPose(), DrivetrainConstants.t)
    ));

    primaryController.x().onTrue(Commands.runOnce(drivetrain::stopWithX, drivetrain));
    primaryController.y().onTrue(DrivetrainFactory.driveToPoint(drivetrain, 5, 5, new Pose2d(3, 4, new Rotation2d(0))));
    primaryController.b().onTrue(Commands.runOnce(() -> drivetrain.setPose(new Pose2d(drivetrain.getPose().getTranslation(), Rotation2d.kZero)), drivetrain).ignoringDisable(true));
  }

  public Command getAutonomousCommand() {
    return autonomousChooser.get();
  }

  public void resetSimulationField() {
    if (GlobalConstants.kCurrentMode != Mode.SIM) return;

    drivetrain.setPose(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
    if (GlobalConstants.kCurrentMode != Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput("FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
  }
}