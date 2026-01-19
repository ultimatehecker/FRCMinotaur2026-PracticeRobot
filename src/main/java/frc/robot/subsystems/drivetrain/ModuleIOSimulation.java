package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Arrays;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import com.ctre.phoenix6.signals.MagnetHealthValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.minolib.advantagekit.LoggedTunableNumber;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.utilities.REVUtility;

public class ModuleIOSimulation implements ModuleIO {
    private final SwerveModuleSimulation moduleSimulation;
    private final SimulatedMotorController.GenericMotorController driverMotor;
    private final SimulatedMotorController.GenericMotorController steerMotor;

    private final LoggedTunableNumber driveKp = new LoggedTunableNumber("Drivetrain/DriveKp", DrivetrainConstants.driveSimKp);
    private final LoggedTunableNumber driveKi = new LoggedTunableNumber("Drivetrain/DriveKi", DrivetrainConstants.driveSimKi);
    private final LoggedTunableNumber driveKd = new LoggedTunableNumber("Drivetrain/DriveKd", DrivetrainConstants.driveSimKd);
    private final LoggedTunableNumber driveKs = new LoggedTunableNumber("Drivetrain/DriveKs", DrivetrainConstants.driveSimKs);
    private final LoggedTunableNumber driveKv = new LoggedTunableNumber("Drivetrain/DriveKv", DrivetrainConstants.driveSimKv);
    private final LoggedTunableNumber steerKp = new LoggedTunableNumber("Drivetrain/SteerKp", DrivetrainConstants.steerSimKp);
    private final LoggedTunableNumber steerKi = new LoggedTunableNumber("Drivetrain/SteerKi", DrivetrainConstants.steerSimKi);
    private final LoggedTunableNumber steerKd = new LoggedTunableNumber("Drivetrain/SteerKd", DrivetrainConstants.steerSimKd);

    private boolean driveClosedLoop = false;
    private boolean steerClosedLoop = false;

    private PIDController driveController = new PIDController(driveKp.get(), driveKi.get(), driveKd.get());
    private PIDController steerController = new PIDController(steerKp.get(), steerKi.get(), steerKd.get());

    private double driveFFVoltage = 0.0;
    private double driveAppliedVoltage = 0.0;
    private double steerAppliedVoltage = 0.0;

    public ModuleIOSimulation(SwerveModuleSimulation moduleSimulation) {
        this.moduleSimulation = moduleSimulation;
        driverMotor = moduleSimulation.useGenericMotorControllerForDrive().withCurrentLimit(Amps.of(DrivetrainConstants.kDriveMotorCurrentLimit));
        steerMotor = moduleSimulation.useGenericControllerForSteer().withCurrentLimit(Amps.of(DrivetrainConstants.kSteerMotorCurrentLimit));

        steerController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        if(driveClosedLoop) {
            driveAppliedVoltage = driveFFVoltage + driveController.calculate(moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond));
        } else {
            driveController.reset();
        }

        if(steerClosedLoop) {
            steerAppliedVoltage = steerController.calculate(moduleSimulation.getSteerAbsoluteFacing().getRadians());
        } else {
            steerController.reset();
        }

        driverMotor.requestVoltage(Volts.of(driveAppliedVoltage));
        steerMotor.requestVoltage(Volts.of(steerAppliedVoltage));

        inputs.isDriveMotorConnected = true;
        inputs.drivePositionRadians = moduleSimulation.getDriveWheelFinalPosition().in(Radians);
        inputs.driveVelocityRadiansPerSecond = moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond);
        inputs.driveCurrentAmperes = Math.abs(moduleSimulation.getDriveMotorSupplyCurrent().in(Amps));
        inputs.driveAppliedVoltage = driveAppliedVoltage;
        inputs.driveTempuratureCelsius = 0.0;

        inputs.isSteerMotorConnected = true;
        inputs.steerPositionRadians = moduleSimulation.getSteerAbsoluteFacing(); // figure out why cannot use getSteerRelativeEncoderPosition()
        inputs.steerVelocityRadiansPerSecond = moduleSimulation.getSteerRelativeEncoderVelocity().in(RadiansPerSecond);
        inputs.steerCurrentAmperes = Math.abs(moduleSimulation.getSteerMotorSupplyCurrent().in(Amps));
        inputs.steerAppliedVoltage = steerAppliedVoltage;
        inputs.steerTempuratureCelsius = 0.0;

        inputs.isSwerveEncoderConnected = true;
        inputs.swerveEncoderMagnetHealth = MagnetHealthValue.Magnet_Green;
        inputs.swerveEncoderPositionRadians = moduleSimulation.getSteerAbsoluteFacing();
        inputs.swerveEncoderVelocityRadiansPerSecond = moduleSimulation.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
        inputs.swerveEncoderSupplyVoltage = 12.0;

        inputs.odometryTimestamps = REVUtility.getSimulationOdometryTimeStamps();
        inputs.odometryDrivePositionsRadians = Arrays.stream(moduleSimulation.getCachedDriveWheelFinalPositions()).mapToDouble(angle -> angle.in(Radians)).toArray();
        inputs.odometrySteerPositionsRadians = moduleSimulation.getCachedSteerAbsolutePositions();

        if (
            driveKp.hasChanged(hashCode()) ||
            driveKi.hasChanged(hashCode()) ||
            driveKd.hasChanged(hashCode()) ||
            steerKp.hasChanged(hashCode()) ||
            steerKi.hasChanged(hashCode()) ||
            steerKd.hasChanged(hashCode())
        ) {
            driveController.setPID(driveKp.get(), driveKi.get(), driveKd.get());
            steerController.setPID(steerKp.get(), steerKi.get(), steerKd.get());
        }
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveClosedLoop = false;
        driveAppliedVoltage = output;
    }

    @Override
    public void setSteerOpenLoop(double output) {
        steerClosedLoop = false;
        steerAppliedVoltage = output;
    }

    @Override
    public void setDriveVelocity(double velocityRadiansPerSecond) {
        driveClosedLoop = true;
        driveFFVoltage = driveKs.get() * Math.signum(velocityRadiansPerSecond) + driveKv.get() * velocityRadiansPerSecond;
        driveController.setSetpoint(velocityRadiansPerSecond);
    }

    @Override
    public void setSteerPosition(Rotation2d rotation) {
        steerClosedLoop = true;
        steerController.setSetpoint(rotation.getRadians());
    }
}
