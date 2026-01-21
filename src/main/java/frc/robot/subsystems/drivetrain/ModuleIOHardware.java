package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.Queue;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.DrivetrainConstants.SwerveModuleConstants;
import frc.robot.utilities.PhoenixUtility;
import frc.robot.utilities.REVUtility;

public class ModuleIOHardware implements ModuleIO {
    private final SparkMax driveMotor;
    private final SparkMax steerMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder steerEncoder;
    private final CANcoder swerveEncoder;

    private final SparkBaseConfig driveConfiguration;
    private final SparkBaseConfig steerConfiguration;
    private final CANcoderConfiguration swerveEncoderConfiguration;

    private final SparkClosedLoopController driveController;
    private final SparkClosedLoopController steerController;

    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> steerPositionQueue;

    private final Debouncer driveConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
    private final Debouncer steerConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
    private final Debouncer swerveEncoderConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

    private StatusSignal<Angle> swerveEncoderPosition;
    private StatusSignal<AngularVelocity> swerveEncoderVelocity;
    private StatusSignal<MagnetHealthValue> swerveEncoderMagnetHealth;
    private StatusSignal<Voltage> swerveEncoderSupplyVoltage;

    private Rotation2d swerveEncoderOffset;

    public ModuleIOHardware(int moduleNumber, SwerveModuleConstants swerveModuleConstants) {
        driveMotor = new SparkMax(swerveModuleConstants.driveMotorID(), MotorType.kBrushless);
        steerMotor = new SparkMax(swerveModuleConstants.steerMotorID(), MotorType.kBrushless);
        swerveEncoder = new CANcoder(swerveModuleConstants.swerveEncoderID());

        driveEncoder = driveMotor.getEncoder();
        steerEncoder = steerMotor.getEncoder();

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
        REVUtility.tryUntilOk(driveMotor, 5, () -> driveEncoder.setPosition(0.0));

        swerveEncoderConfiguration = new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                .withAbsoluteSensorDiscontinuityPoint(0.5)
                .withMagnetOffset(swerveModuleConstants.swerveEncoderOffset().getRotations())
                .withSensorDirection(DrivetrainConstants.kSwerveEncoderInverted ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive)
            );

        PhoenixUtility.tryUntilOk(5, () -> swerveEncoder.getConfigurator().apply(swerveEncoderConfiguration));

        swerveEncoderPosition = swerveEncoder.getAbsolutePosition();
        swerveEncoderVelocity = swerveEncoder.getVelocity();
        swerveEncoderMagnetHealth = swerveEncoder.getMagnetHealth();
        swerveEncoderSupplyVoltage = swerveEncoder.getSupplyVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50, 
            swerveEncoderPosition,
            swerveEncoderVelocity,
            swerveEncoderMagnetHealth,
            swerveEncoderSupplyVoltage
        );

        steerConfiguration = new SparkMaxConfig()
            .idleMode(IdleMode.kBrake)
            .inverted(DrivetrainConstants.kSteerEncoderInverted)
            .smartCurrentLimit(DrivetrainConstants.kSteerMotorCurrentLimit)
            .voltageCompensation(12.0);

        steerConfiguration.encoder
            .positionConversionFactor(DrivetrainConstants.kSteerEncoderPositionFactor)
            .velocityConversionFactor(DrivetrainConstants.kSteerEncoderVelocityFactor)
            .uvwMeasurementPeriod(10)
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

        REVUtility.tryUntilOk(steerMotor, 5, () -> steerMotor.configure(steerConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        REVUtility.tryUntilOk(steerMotor, 5, () -> steerEncoder.setPosition(swerveEncoderPosition.getValue().in(Radians)));

        timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
        drivePositionQueue = SparkOdometryThread.getInstance().registerSignal(driveMotor, driveEncoder::getPosition);
        steerPositionQueue = SparkOdometryThread.getInstance().registerSignal(steerMotor, steerEncoder::getPosition);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        REVUtility.sparkStickyFault = false;
        inputs.steerPositionRadians = Rotation2d.fromRotations(swerveEncoderPosition.getValue().in(Rotations)); //REVUtility.ifOk(driveMotor, driveEncoder::getPosition, (v) -> inputs.drivePositionRadians = v);
        REVUtility.ifOk(driveMotor, driveEncoder::getVelocity, (v) -> inputs.driveVelocityRadiansPerSecond = v);
        REVUtility.ifOk(driveMotor, new DoubleSupplier[] {driveMotor::getAppliedOutput, driveMotor::getBusVoltage}, (v) -> inputs.driveAppliedVoltage = v[0] * v[1]);
        REVUtility.ifOk(driveMotor, driveMotor::getOutputCurrent, (v) -> inputs.driveCurrentAmperes = v);
        REVUtility.ifOk(driveMotor, driveMotor::getMotorTemperature, (v) -> inputs.driveTempuratureCelsius = v);
        inputs.isDriveMotorConnected = driveConnectedDebouncer.calculate(!REVUtility.sparkStickyFault);

        REVUtility.sparkStickyFault = false;
        REVUtility.ifOk(steerMotor, steerEncoder::getPosition, (v) -> inputs.steerPositionRadians = new Rotation2d(v));
        REVUtility.ifOk(steerMotor, steerEncoder::getVelocity, (v) -> inputs.steerVelocityRadiansPerSecond = v);
        REVUtility.ifOk(steerMotor, new DoubleSupplier[] {steerMotor::getAppliedOutput, steerMotor::getBusVoltage}, (v) -> inputs.steerAppliedVoltage = v[0] * v[1]);
        REVUtility.ifOk(steerMotor, steerMotor::getOutputCurrent, (v) -> inputs.steerCurrentAmperes = v);
        REVUtility.ifOk(steerMotor, steerMotor::getMotorTemperature, (v) -> inputs.driveTempuratureCelsius = v);
        inputs.isSteerMotorConnected = steerConnectedDebouncer.calculate(!REVUtility.sparkStickyFault);

        StatusCode swerveEncoderStatus = BaseStatusSignal.refreshAll(swerveEncoderPosition, swerveEncoderVelocity, swerveEncoderMagnetHealth, swerveEncoderSupplyVoltage);
        inputs.isSwerveEncoderConnected = swerveEncoderConnectedDebouncer.calculate(swerveEncoderStatus.isOK());
        inputs.swerveEncoderPositionRadians = Rotation2d.fromRotations(swerveEncoderPosition.getValue().in(Rotations));
        inputs.swerveEncoderVelocityRadiansPerSecond = swerveEncoderVelocity.getValue().in(RadiansPerSecond);
        inputs.swerveEncoderMagnetHealth = swerveEncoderMagnetHealth.getValue();
        inputs.swerveEncoderSupplyVoltage = swerveEncoderSupplyVoltage.getValue().in(Volts);

        inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double v) -> v).toArray();
        inputs.odometryDrivePositionsRadians = drivePositionQueue.stream().mapToDouble((Double v) -> v).toArray();
        inputs.odometrySteerPositionsRadians = steerPositionQueue.stream().map((Double v) -> Rotation2d.fromRadians(v)).toArray(Rotation2d[]::new);

        timestampQueue.clear();
        drivePositionQueue.clear();
        steerPositionQueue.clear();
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveMotor.setVoltage(output);
    }

    @Override
    public void setSteerOpenLoop(double output) {
        steerMotor.setVoltage(output);
    }

    @Override
    public void setDriveVelocity(double velocityRadiansPerSecond) {
        double ffVoltage = DrivetrainConstants.driveKs * Math.signum(velocityRadiansPerSecond) + DrivetrainConstants.driveKv * velocityRadiansPerSecond;
        driveController.setSetpoint(velocityRadiansPerSecond, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ffVoltage, ArbFFUnits.kVoltage);
    }

    @Override
    public void setSteerPosition(Rotation2d rotation) {
        double setpoint = MathUtil.inputModulus(rotation.getRadians(), DrivetrainConstants.kSteerPIDMinInput, DrivetrainConstants.kSteerPIDMaxInput);
        steerController.setSetpoint(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
}
