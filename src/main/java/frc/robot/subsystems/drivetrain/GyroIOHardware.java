package frc.robot.subsystems.drivetrain;

import java.util.Queue;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.GyroIO.GyroIOInputs;

public class GyroIOHardware implements GyroIO {
    private final AHRS gyro;
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;

    public GyroIOHardware() {
        gyro = new AHRS(NavXComType.kMXP_SPI, (byte) DrivetrainConstants.kOdometryFrequency);

        yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(gyro::getAngle);
        yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = gyro.isConnected();
        inputs.yawPosition = Rotation2d.fromDegrees(-gyro.getAngle());
        inputs.yawVelocityRadiansPerSecond = Units.degreesToRadians(-gyro.getRawGyroZ());

        inputs.odometryYawTimestamps = yawTimestampQueue.stream().mapToDouble((Double v) -> v).toArray();
        inputs.odometryYawPositions = yawPositionQueue.stream().map((Double v) -> Rotation2d.fromDegrees(v)).toArray(Rotation2d[]::new);

        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }
}
