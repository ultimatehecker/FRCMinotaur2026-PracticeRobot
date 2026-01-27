package frc.robot.constants;

import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

import frc.robot.utilities.SwerveModuleType;

public class DrivetrainConstants {
    public static final double kOdometryFrequency = 50.0;
    public static final SwerveModuleType kSwerveModuleType = SwerveModuleType.MK3_FAST;

    public static final Pose2d kBlueLeftStartingPose = new Pose2d(3.500, 7.400, new Rotation2d(Math.toRadians(-45)));
    public static final Pose2d kBlueCenterStartingPose = new Pose2d(3.550, 4.000, new Rotation2d(Math.toRadians(0)));
    public static final Pose2d kBlueRightStartingPose = new Pose2d(3.500, 0.650, new Rotation2d(Math.toRadians(45)));
    public static final Pose2d t = new Pose2d(4.616, 4.039, new Rotation2d(0));

    public static final SwerveModuleConstants kFrontLeftModuleConstants = new SwerveModuleConstants(5, 6, 11, new Rotation2d(0.09 - 1.04));
    public static final SwerveModuleConstants kFrontRightModuleConstants = new SwerveModuleConstants(1, 2, 9, new Rotation2d(1.773));
    public static final SwerveModuleConstants kBackLeftModuleConstants = new SwerveModuleConstants(7, 8, 12, new Rotation2d(-3.203));
    public static final SwerveModuleConstants kBackRightModuleConstants = new SwerveModuleConstants(3, 4, 10, new Rotation2d(-1.703));

    public static final double kMaximumLinearVelocityMetersPerSecond = 4.5;
    public static final double kMaximumLinearAccelerationMetersPerSecondSquared = 9.0;
    public static final double kMaximumRotationalVelocityRadiansPerSecond = 3 * Math.PI;
    public static final double kMaximumRotationalAccelerationRadiansPerSecondSquared = 6 * Math.PI; 

    // Drive Motor PID Constants
    public static final double driveKp = 0.0;
    public static final double driveKi = 0.0;
    public static final double driveKd = 0.0;
    public static final double driveKs = 0.0;
    public static final double driveKv = 0.1;
    public static final double driveKa = 0.0;
    public static final double driveSimKp = 0.05;
    public static final double driveSimKi = 0.0;
    public static final double driveSimKd = 0.0;
    public static final double driveSimKs = 0.0;
    public static final double driveSimKv = 0.0789;

    // Drive Motor Configuration
    public static final boolean kDriveMotorInverted = kSwerveModuleType.isDriveInverted();
    public static final int kDriveMotorCurrentLimit = 40;
    public static final double kDriveMotorReduction = kSwerveModuleType.getDriveReduction();
    public static final DCMotor kDriveSimulatedGearbox = DCMotor.getNEO(1);

    // Drive Motor Encoder Configuration
    public static final double kDriveEncoderPositionFactor = (2 * Math.PI) / kDriveMotorReduction; 
    public static final double kDriveEncoderVelocityFactor = (2 * Math.PI) / 60.0 / kDriveMotorReduction; 

    // Steer Motor PID Constants
    public static final double steerKp = 3.0;
    public static final double steerKi = 0.0;
    public static final double steerKd = 0.0;
    public static final double steerSimKp = 8.0;
    public static final double steerSimKi = 0.0;
    public static final double steerSimKd = 0.0;
    public static final double kSteerPIDMinInput = -Math.PI; // Radians
    public static final double kSteerPIDMaxInput = Math.PI; // Radians

    // Steer Motor Configuration
    public static final boolean kSteerMotorInverted = kSwerveModuleType.isSteerInverted();
    public static final int kSteerMotorCurrentLimit = 30;
    public static final double kSteerMotorReduction = kSwerveModuleType.getSteerReduction();
    public static final DCMotor kSteerSimulatedGearbox = DCMotor.getNEO(1);

    // Steer Motor Encoder Configuration
    public static final boolean kSteerEncoderInverted = false;
    public static final boolean kSwerveEncoderInverted = false;
    public static final double kSteerEncoderPositionFactor = 2 * Math.PI / kSteerMotorReduction; // Rotations -> Radians
    public static final double kSteerEncoderVelocityFactor = 2 * Math.PI / kSteerMotorReduction / 60; // RPM -> Rad/Sec

    public static final double kWheelRadiusMeters = Units.inchesToMeters(2.0);
    public static final double kTrackWidth = Units.inchesToMeters(15);
    public static final double kWheelBase = Units.inchesToMeters(15);
    public static final double kDriveBaseRadius = Math.hypot(kTrackWidth / 2.0, kWheelBase / 2.0);
    public static final Translation2d[] kModuleTranslations = new Translation2d[] {
        new Translation2d(kTrackWidth / 2.0, kWheelBase / 2.0),
        new Translation2d(kTrackWidth / 2.0, -kWheelBase / 2.0),
        new Translation2d(-kTrackWidth / 2.0, kWheelBase / 2.0),
        new Translation2d(-kTrackWidth / 2.0, -kWheelBase / 2.0)
    };

    public static final double kRobotMassKilograms = 25.00;
    public static final double kRobotMOI = 6.883;
    public static final double kWheelCOF = 1.0;
    public static final RobotConfig kPathPlannerConfiguration = new RobotConfig(
        kRobotMassKilograms,
        kRobotMOI,
        new ModuleConfig(
            kWheelRadiusMeters, 
            kMaximumLinearVelocityMetersPerSecond, 
            kWheelCOF, 
            kDriveSimulatedGearbox, 
            kDriveMotorCurrentLimit, 
            1
        ),
        kModuleTranslations
    );

    public static final DriveTrainSimulationConfig kMapleSimConfiguration = DriveTrainSimulationConfig.Default()
        .withCustomModuleTranslations(kModuleTranslations)
        .withRobotMass(Kilogram.of(kRobotMassKilograms))
        .withGyro(COTS.ofNav2X())
        .withSwerveModule(
            new SwerveModuleSimulationConfig(
                kDriveSimulatedGearbox,
                kSteerSimulatedGearbox,
                kDriveMotorReduction,
                kSteerMotorReduction,
                Volts.of(0.1),
                Volts.of(0.1),
                Meters.of(kWheelRadiusMeters),
                KilogramSquareMeters.of(0.02),
                kWheelCOF
            )
        );

    public record SwerveModuleConstants (
        int driveMotorID, 
        int steerMotorID, 
        int swerveEncoderID, 
        Rotation2d swerveEncoderOffset
    ) {};
}
