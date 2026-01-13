package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.signals.MagnetHealthValue;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ModuleIO {
    @AutoLog
    public class ModuleIOInputs {
        public boolean isDriveMotorConnected = false; // Will depricate when using MinoSparkMax
        public double drivePositionRadians = 0.0;
        public double driveVelocityRadiansPerSecond = 0.0;
        public double driveAppliedVoltage = 0.0;
        public double driveCurrentAmperes = 0.0;
        public double driveTempuratureCelsius = 0.0; // Will depricate when using MinoSparkMax

        public boolean isSteerMotorConnected = false; // Will depricate when using MinoSparkMax
        public Rotation2d steerPositionRadians = Rotation2d.kZero;
        public double steerVelocityRadiansPerSecond = 0.0;
        public double steerAppliedVoltage = 0.0;
        public double steerCurrentAmperes = 0.0;
        public double steerTempuratureCelsius = 0.0; // Will depricate when using MinoSparkMax

        public boolean isSwerveEncoderConnected = false;
        public Rotation2d swerveEncoderPositionRadians = Rotation2d.kZero; // figure out units for this as we code
        public double swerveEncoderVelocityRadiansPerSecond = 0.0;
        public MagnetHealthValue swerveEncoderMagnetHealth = MagnetHealthValue.Magnet_Invalid;
        public double swerveEncoderSupplyVoltage = 0.0;
        
        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsRadians = new double[] {};
        public Rotation2d[] odometrySteerPositionsRadians = new Rotation2d[] {};
    }

    public default void updateInputs(ModuleIOInputs inputs) {};

    public default void setDriveOpenLoop(double output) {};

    public default void setSteerOpenLoop(double output) {};

    public default void setDriveVelocity(double velocityRadiansPerSecond) {};

    public default void setSteerPosition(Rotation2d rotation) {};
}
