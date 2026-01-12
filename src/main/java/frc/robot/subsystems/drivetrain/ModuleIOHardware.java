package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ModuleIOHardware {
    private final SparkMax driveMotor;
    private final SparkMax steerMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder steerEncoder;
    private final CANcoder swerveEncoder;

    private final SparkMaxConfig driveConfiguration;
    private final SparkMaxConfig steerConfiguration;
}
