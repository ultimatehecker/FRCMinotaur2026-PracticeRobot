package frc.robot.subsystems.drivetrain;

import java.util.Queue;

import com.ctre.phoenix6.hardware.CANcoder;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.DrivetrainConstants.SwerveModuleConstants;
import frc.robot.utilities.REVUtility;

public class ModuleIOHardware {
    private final Rotation2d steerRotationOffet;

    private final SparkMax driveMotor;
    private final SparkMax steerMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder steerEncoder;
    private final CANcoder swerveEncoder;

    private final SparkBaseConfig driveConfiguration;
    private final SparkBaseConfig steerConfiguration;

    private final SparkClosedLoopController driveController;
    private final SparkClosedLoopController steerController;

    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> steerPositionQueue;

    private final Debouncer driveConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
    private final Debouncer steerConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

    public ModuleIOHardware(int moduleNumber, SwerveModuleConstants swerveModuleConstants) {
        driveMotor = new SparkMax(swerveModuleConstants.driveMotorID(), MotorType.kBrushless);
        steerMotor = new SparkMax(swerveModuleConstants.steerMotorID(), MotorType.kBrushless);
        swerveEncoder = new CANcoder(swerveModuleConstants.swerveEncoderID());

        driveEncoder = driveMotor.getEncoder();
        steerEncoder = steerMotor.getEncoder();
        steerRotationOffet = swerveModuleConstants.swerveEncoderOffset();

        driveController = driveMotor.getClosedLoopController();
        steerController = steerMotor.getClosedLoopController();

        driveConfiguration = new SparkMaxConfig()
            .idleMode(IdleMode.kBrake)
            .inverted(DrivetrainConstants.kDriveMotorInverted)
            .smartCurrentLimit(DrivetrainConstants.kDriveMotorCurrentLimit)
            .voltageCompensation(12.0);

        driveConfiguration.encoder
            .positionConversionFactor(DrivetrainConstants.kDriveEncoderPositionFactor)
            .velocityConversionFactor(DrivetrainConstants.kDriveEncoderVelocityFactor)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);
        
        driveConfiguration.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(DrivetrainConstants.driveKp, 0.0, DrivetrainConstants.driveKd);

        driveConfiguration.signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs((int) (1000.0 / DrivetrainConstants.kOdometryFrequency))
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);

        REVUtility.tryUntilOk(driveMotor, 5, () -> driveMotor.configure(driveConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        REVUtility.tryUntilOk(driveMotor, 6, () -> driveEncoder.setPosition(0.0));

        steerConfiguration = new SparkMaxConfig()
            .idleMode(IdleMode.kBrake)
            .inverted(DrivetrainConstants.kSteerEncoderInverted)
            .smartCurrentLimit(DrivetrainConstants.kSteerMotorCurrentLimit)
            .voltageCompensation(12.0);

        steerConfiguration.encoder
            .inverted(DrivetrainConstants.kSteerEncoderInverted)
            .positionConversionFactor(DrivetrainConstants.kDriveEncoderPositionFactor)
            .velocityConversionFactor(DrivetrainConstants.kDriveEncoderVelocityFactor)
            .uvwAverageDepth(2);

        steerConfiguration.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(DrivetrainConstants.kSteerPIDMinInput, DrivetrainConstants.kSteerPIDMaxInput)
            .pid(DrivetrainConstants.steerKp, 0.0, DrivetrainConstants.steerKd);

        steerConfiguration.signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs((int) (1000.0 / DrivetrainConstants.kOdometryFrequency))
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);

        REVUtility.tryUntilOk(steerMotor, 5, () -> steerMotor.configure(driveConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        REVUtility.tryUntilOk(steerMotor, 6, () -> steerEncoder.setPosition(0.0)); // IMPORTANT TODO: Need to set the position to the current position of the absolute encoder;

        timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
        drivePositionQueue = SparkOdometryThread.getInstance().registerSignal(driveMotor, driveEncoder::getPosition);
        steerPositionQueue = SparkOdometryThread.getInstance().registerSignal(steerMotor, steerEncoder::getPosition);
    }
}
