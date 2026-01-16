package frc.robot.constants;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.utilities.SwerveModuleType;

public class DrivetrainConstants {
    public static final double kOdometryFrequency = 50.0;
    public static final SwerveModuleType kSwerveModuleType = SwerveModuleType.MK3_FAST;

    public static final SwerveModuleConstants kFrontLeftModuleConstants = new SwerveModuleConstants(1, 2, 9, Rotation2d.fromRotations(-0.033447)); // replace with actual values
    public static final SwerveModuleConstants kFrontRightModuleConstants = new SwerveModuleConstants(3, 4, 10, Rotation2d.fromRotations(-0.478271)); // replace with actual values
    public static final SwerveModuleConstants kBackLeftModuleConstants = new SwerveModuleConstants(5, 6, 11, Rotation2d.fromRotations(0.0)); // replace with actual values
    public static final SwerveModuleConstants kBackRightModuleConstants = new SwerveModuleConstants(7, 8, 12, Rotation2d.fromRotations(-0.227051)); // replace with actual values

    public static final double kDriveMaximumSpeedMetersPerSecond = 4.5;
    public static final double kDriveMaximumAccelerationMetersPerSecondSquared = 9.0;

    // Drive Motor PID Constants
    public static final double driveKp = 0.0;
    public static final double driveKd = 0.0;
    public static final double driveKs = 0.0;
    public static final double driveKv = 0.1;
    public static final double driveSimP = 0.05;
    public static final double driveSimD = 0.0;
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
    public static final double steerKp = 2.0;
    public static final double steerKd = 0.0;
    public static final double steerSimP = 8.0;
    public static final double steerSimD = 0.0;
    public static final double kSteerPIDMinInput = -Math.PI; // Radians
    public static final double kSteerPIDMaxInput = Math.PI; // Radians

    // Steer Motor Configuration
    public static final boolean kSteerMotorInverted = kSwerveModuleType.isSteerInverted();
    public static final int kSteerMotorCurrentLimit = 20;
    public static final double kSteerMotorReduction = kSwerveModuleType.getSteerReduction();
    public static final DCMotor kSteerSimulatedGearbox = DCMotor.getNeo550(1);

    // Steer Motor Encoder Configuration
    public static final boolean kSteerEncoderInverted = false;
    public static final boolean kSwerveEncoderInverted = false;
    public static final double kSteerEncoderPositionFactor = 2 * Math.PI / kSteerMotorReduction; // Rotations -> Radians
    public static final double kSteerEncoderVelocityFactor = 2 * Math.PI / kSteerMotorReduction / 60; // RPM -> Rad/Sec

    public static final double kWheelRadiusMeters = Units.inchesToMeters(2.0);
    public static final double kTrackWidth = Units.inchesToMeters(21.5);
    public static final double kWheelBase = Units.inchesToMeters(21.5);
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
            kDriveMaximumSpeedMetersPerSecond, 
            kWheelCOF, 
            kDriveSimulatedGearbox, 
            kDriveMotorCurrentLimit, 
            1
        ),
        kModuleTranslations
    );

    public record SwerveModuleConstants (
        int driveMotorID, 
        int steerMotorID, 
        int swerveEncoderID, 
        Rotation2d swerveEncoderOffset
    ) {};
}
