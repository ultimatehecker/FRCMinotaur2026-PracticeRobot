package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.constants.DrivetrainConstants;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Module {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int index;

    private final Alert driveMotorDisconnected;
    private final Alert steerMotorDisconnected;
    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

    public Module(ModuleIO io, int index) {
        this.io = io;
        this.index = index;

        driveMotorDisconnected = new Alert("Drive Motor on module " + Integer.toString(index) + " is currently disconnected.", AlertType.kError);
        steerMotorDisconnected = new Alert("Steer Motor on module " + Integer.toString(index) + " is currently disconnected.", AlertType.kError);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drivetrain/Module " + Integer.toString(index), inputs);

        int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
        odometryPositions = new SwerveModulePosition[sampleCount];

        for (int i = 0; i < sampleCount; i++) {
            double positionMeters = inputs.odometryDrivePositionsRadians[i] * DrivetrainConstants.kWheelRadiusMeters;
            Rotation2d angle = inputs.odometrySteerPositionsRadians[i];
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }

        driveMotorDisconnected.set(!inputs.isDriveMotorConnected);
        steerMotorDisconnected.set(!inputs.isSteerMotorConnected);
    }

    public Rotation2d getAngle() {
        return inputs.steerPositionRadians;
    }

    public double getPositionMeters() {
        return inputs.drivePositionRadians * DrivetrainConstants.kWheelRadiusMeters;
    }

    public double getVelocityMetersPerSecond() {
        return inputs.driveVelocityRadiansPerSecond * DrivetrainConstants.kWheelRadiusMeters;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSecond(), getAngle());
    }

    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }

    public double[] getOdometryTimestamps() {
        return inputs.odometryTimestamps;
    }

    public double getWheelRadiusCharacterizationPosition() {
        return inputs.drivePositionRadians;
    }

    public double getFFCharacterizationVelocity() {
        return inputs.driveVelocityRadiansPerSecond;
    }

    public void runSetpoint(SwerveModuleState state) {
        state.optimize(getAngle());
        state.cosineScale(inputs.steerPositionRadians);

        io.setDriveVelocity(state.speedMetersPerSecond / DrivetrainConstants.kWheelRadiusMeters);

        if(Math.abs(state.angle.getRadians() - getAngle().getRadians()) < 0.1) return;
        io.setSteerPosition(state.angle);
    }

    public void runCharacterization(double output) {
        io.setDriveOpenLoop(output);
        io.setSteerPosition(Rotation2d.kZero);
    }

    public void stop() {
        io.setDriveOpenLoop(0.0);
        io.setSteerOpenLoop(0.0);
    }
}
