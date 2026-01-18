package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Arrays;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.utilities.REVUtility;

public class ModuleIOSimulation implements ModuleIO {
    private final SwerveModuleSimulation moduleSimulation;
    private final SimulatedMotorController.GenericMotorController driverMotor;
    private final SimulatedMotorController.GenericMotorController steerMotor;

    private boolean driveClosedLoop = false;
    private boolean steerClosedLoop = false;

    private final PIDController driveController = new PIDController(DrivetrainConstants.driveSimP, 0.0, DrivetrainConstants.driveSimD);
    private final PIDController steerController = new PIDController(DrivetrainConstants.steerSimP, 0.0, DrivetrainConstants.steerSimD);

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
        inputs.isDriveMotorConnected = true;
        inputs.drivePositionRadians = moduleSimulation.getDriveWheelFinalPosition().in(Radians);
        inputs.driveVelocityRadiansPerSecond = moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond);
        inputs.driveCurrentAmperes = Math.abs(moduleSimulation.getDriveMotorSupplyCurrent().in(Amps));
        inputs.driveAppliedVoltage = driveAppliedVoltage;
        inputs.driveTempuratureCelsius = 0.0;

        inputs.isSteerMotorConnected = true;
        inputs.steerPositionRadians = moduleSimulation.getSteerAbsoluteFacing(); // idk what to do here (probably will just get relative position and convert to rotation2d)
        inputs.steerVelocityRadiansPerSecond = moduleSimulation.getSteerRelativeEncoderVelocity().in(RadiansPerSecond);
        inputs.steerCurrentAmperes = Math.abs(moduleSimulation.getSteerMotorSupplyCurrent().in(Amps));
        inputs.steerAppliedVoltage = steerAppliedVoltage;
        inputs.steerTempuratureCelsius = 0.0;

        // Figure out what to do here for the absolute steering encoder

        inputs.odometryTimestamps = REVUtility.getSimulationOdometryTimeStamps();
        inputs.odometryDrivePositionsRadians = Arrays.stream(moduleSimulation.getCachedDriveWheelFinalPositions()).mapToDouble(angle -> angle.in(Radians)).toArray();
        inputs.odometrySteerPositionsRadians = moduleSimulation.getCachedSteerAbsolutePositions();
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
        driveFFVoltage = DrivetrainConstants.driveSimKs * Math.signum(velocityRadiansPerSecond) + DrivetrainConstants.driveSimKv * velocityRadiansPerSecond;
        driveController.setSetpoint(velocityRadiansPerSecond);
    }

    @Override
    public void setSteerPosition(Rotation2d rotation) {
        steerClosedLoop = true;
        steerController.setSetpoint(rotation.getRadians());
    }
}
