package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.GlobalConstants.Mode;
import frc.robot.utilities.LocalADStarAK;

public class Drivetrain extends SubsystemBase {
    public static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4];
    private final SysIdRoutine sysID;
    private final Alert gyroDisconnectedAlert;

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(DrivetrainConstants.kModuleTranslations);
    private Rotation2d rawGyroRotation = Rotation2d.kZero;
    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(), 
        new SwerveModulePosition(),
        new SwerveModulePosition()
    };

    private final PIDController choreoXController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController choreoYController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController choreoThetaController = new PIDController(7.5, 0.0, 0.0);

    private final PIDController autoDriveToPointController = new PIDController(6, 0, 0.1);
    private final PIDController teleopDriveToPointController = new PIDController(3.6, 0, 0.3);
    private final PIDController combinedRotationController = new PIDController(1, 0, 0.1);

    private boolean isDriveToPointPoseApplied = false;
    private Pose2d desiredDriveToPointPose;
    private final Timer driveToPoseTimer = new Timer();
    private Optional<Pose2d> driveToPointPoseToBeApplied;

    private boolean isChoreoTracjectoryApplied = false;
    private Trajectory<SwerveSample> desiredChoreoTrajectory;
    private final Timer choreoTimer = new Timer();
    private Optional<SwerveSample> choreoSampleToBeApplied;
    
    private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Pose2d.kZero);
    private final Consumer<Pose2d> resetSimulationPoseCallBack;
    private Field2d field;

    public Drivetrain(GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO, Consumer<Pose2d> resetSimulationPoseCallBack) {
        this.gyroIO = gyroIO;
        this.resetSimulationPoseCallBack = resetSimulationPoseCallBack;

        modules[0] = new Module(flModuleIO, 0);
        modules[1] = new Module(frModuleIO, 1);
        modules[2] = new Module(blModuleIO, 2);
        modules[3] = new Module(brModuleIO, 3);

        gyroDisconnectedAlert = new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kWarning);
        SparkOdometryThread.getInstance().start();

        AutoBuilder.configure(
            this::getPose,
            this::setPose,
            this::getChassisSpeeds,
            this::runVelocity,
            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0.0, 0.0), 
                new PIDConstants(5.0, 0.0, 0.0)
            ),
            DrivetrainConstants.kPathPlannerConfiguration,
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
            this
        );

        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback((p) -> {
            Logger.recordOutput("Odometry/Trajectory", p.toArray(new Pose2d[0]));
        });

        PathPlannerLogging.setLogTargetPoseCallback((p) -> {
            Logger.recordOutput("Odometry/TrajectorySetpoint", p);
        });

        sysID = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (s) -> Logger.recordOutput("Drivetrain/SysIdState", s.toString())
            ), 
            new SysIdRoutine.Mechanism((v) -> runCharacterization(v.in(Volts)), null, this)
        );

        combinedRotationController.enableContinuousInput(-Math.PI, Math.PI);
        field = new Field2d();
    }

    @Override
    public void periodic() {
        odometryLock.lock();
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drivetrain/Gyro", gyroInputs);

        for(Module module : modules) {
            module.periodic();
        }

        odometryLock.unlock();

        if(DriverStation.isDisabled()) {
            for(Module module : modules) {
                module.stop();
            }
        }

        if(DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveModuleStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveModuleStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        double[] sampleTimestamps = modules[0].getOdometryTimestamps();
        int sampleCount = sampleTimestamps.length;

        for (int i = 0; i < sampleCount; i++) {
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                    modulePositions[moduleIndex].distanceMeters - lastModulePositions[moduleIndex].distanceMeters, modulePositions[moduleIndex].angle
                );

                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            if (gyroInputs.connected) {
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
        }

        gyroDisconnectedAlert.set(!gyroInputs.connected && GlobalConstants.kCurrentMode != Mode.SIM);

        field.setRobotPose(getPose());
        SmartDashboard.putData("Field", field);
    }

    public void runVelocity(ChassisSpeeds speeds) {
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DrivetrainConstants.kMaximumLinearVelocityMetersPerSecond);

        Logger.recordOutput("SwerveModuleStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

        for(int i = 0; i < modules.length; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }

        Logger.recordOutput("SwerveModuleStates/SetpointsOptimized", setpointStates);
    }

    public void followTrajectory(Trajectory<SwerveSample> trajectory) {
        desiredChoreoTrajectory = trajectory;

        if(!isChoreoTracjectoryApplied) {
            choreoTimer.restart();
            choreoSampleToBeApplied = trajectory.sampleAt(choreoTimer.get(), false);
            isChoreoTracjectoryApplied = true;
        } else {
            choreoSampleToBeApplied = desiredChoreoTrajectory.sampleAt(choreoTimer.get(), false);
        }

        if (choreoSampleToBeApplied.isPresent()) {
            var sample = choreoSampleToBeApplied.get();
            
            Logger.recordOutput("Drivetrain/Choreo/Timer Value", choreoTimer.get());
            Logger.recordOutput("Drivetrain/Choreo/Trajectory Name", desiredChoreoTrajectory.name());
            Logger.recordOutput("Drivetrain/Choreo/Total Time", desiredChoreoTrajectory.getTotalTime());
            Logger.recordOutput("Drivetrain/Choreo/Sample/Desired Pose", sample.getPose());
            Logger.recordOutput("Drivetrain/Choreo/Sample/Desired Chassis Speeds", sample.getChassisSpeeds());
            Logger.recordOutput("Drivetrain/Choreo/Sample/Module Forces X", sample.moduleForcesX());
            Logger.recordOutput("Drivetrain/Choreo/Sample/Module Forces Y", sample.moduleForcesY());

            Pose2d pose = getPose();
            ChassisSpeeds targetSpeeds = sample.getChassisSpeeds();

            targetSpeeds.vxMetersPerSecond += choreoXController.calculate(pose.getX(), sample.x);
            targetSpeeds.vyMetersPerSecond += choreoYController.calculate(pose.getY(), sample.y);
            targetSpeeds.omegaRadiansPerSecond += choreoThetaController.calculate(pose.getRotation().getRadians(), sample.heading);

            runVelocity(targetSpeeds);
        }
    }

    public void runDriveToPoint(double constraintedMaximumLinearVelocity, double constraintedMaximumAngularVelocity, Pose2d targetPose) {
        desiredDriveToPointPose = targetPose;

        if(!isDriveToPointPoseApplied) {
            driveToPoseTimer.restart();
            driveToPointPoseToBeApplied = Optional.of(desiredDriveToPointPose);
            isDriveToPointPoseApplied = true;
        } else {
            driveToPointPoseToBeApplied = Optional.of(desiredDriveToPointPose);
        }

        if(driveToPointPoseToBeApplied.isPresent()) {
            Translation2d translationToDesiredPoint = desiredDriveToPointPose.getTranslation().minus(getPose().getTranslation());
            double linearDistance = translationToDesiredPoint.getNorm();
            double frictionConstant = 0.0;

            if (linearDistance >= Units.inchesToMeters(0.5)) {
                frictionConstant = 0.02 * DrivetrainConstants.kMaximumLinearVelocityMetersPerSecond;
            }

            Rotation2d directionOfTravel = translationToDesiredPoint.getAngle();
            double velocityOutput = 0.0;

            double currentHeading = getRotation().getRadians();
            double targetHeading = desiredDriveToPointPose.getRotation().getRadians();

            double angularVelocity = combinedRotationController.calculate(currentHeading, targetHeading);

            if (DriverStation.isAutonomous()) {
                velocityOutput = Math.min(
                    Math.abs(autoDriveToPointController.calculate(linearDistance, 0)) + frictionConstant,
                    constraintedMaximumLinearVelocity
                );
            } else {
                velocityOutput = Math.min(
                    Math.abs(teleopDriveToPointController.calculate(linearDistance, 0)) + frictionConstant,
                    constraintedMaximumLinearVelocity
                );
            }

            double xComponent = velocityOutput * directionOfTravel.getCos();
            double yComponent = velocityOutput * directionOfTravel.getSin();

            Logger.recordOutput("Drivetrain/DriveToPoint/VelocitySetpointX", xComponent);
            Logger.recordOutput("Drivetrain/DriveToPoint/VelocitySetpointY", yComponent);
            Logger.recordOutput("Drivetrain/DriveToPoint/VelocityOutput", velocityOutput);
            Logger.recordOutput("Drivetrain/DriveToPoint/LinearDistance", linearDistance);
            Logger.recordOutput("Drivetrain/DriveToPoint/DirectionOfTravel", directionOfTravel);
            Logger.recordOutput("Drivetrain/DriveToPoint/DesiredPoint", desiredDriveToPointPose);
            Logger.recordOutput("Drivetrain/DriveToPoint/DesiredHeading", targetHeading);
            Logger.recordOutput("Drivetrain/DriveToPoint/CurrentHeading", currentHeading);

            if (Double.isNaN(constraintedMaximumAngularVelocity)) {
                runVelocity(new ChassisSpeeds(xComponent, yComponent, angularVelocity));
            } else {
                angularVelocity = MathUtil.clamp(angularVelocity, -constraintedMaximumAngularVelocity, constraintedMaximumAngularVelocity);
                runVelocity(new ChassisSpeeds(xComponent, yComponent, angularVelocity));
            }
        }
    }

    public Rotation2d computeAngleFromTarget(Pose2d robotPose, Pose2d targetPose) {
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double robotHeading = robotPose.getRotation().getRadians();

        double turretX = robotX + 0 * Math.cos(robotHeading) - 0 * Math.sin(robotHeading);
        double turretY = robotY + 0 * Math.sin(robotHeading) + 0 * Math.cos(robotHeading);

        double dx = targetPose.getX() - turretX;
        double dy = targetPose.getY() - turretY;

        double targetAngleGlobal = Math.atan2(dy, dx);
        double angleToGoal = targetAngleGlobal - robotHeading;
        
        return new Rotation2d(angleToGoal);
    }

    public boolean isAtDriveToPointSetpoint() {
        if (
            MathUtil.isNear(desiredDriveToPointPose.getX(), getPose().getX(), Units.inchesToMeters(0.5)) && 
            MathUtil.isNear(desiredDriveToPointPose.getY(), getPose().getY(), Units.inchesToMeters(0.5)) &&
            isDriveToPointPoseApplied
        ) {
            isDriveToPointPoseApplied = false;
            return true;
        } else {
            return false;
        }
    }

    public boolean isAtChoreoSetpoint() {
        if (
            MathUtil.isNear(desiredChoreoTrajectory.getFinalPose(false).get().getX(), getPose().getX(), Units.inchesToMeters(0.5)) && 
            MathUtil.isNear(desiredChoreoTrajectory.getFinalPose(false).get().getY(), getPose().getY(), Units.inchesToMeters(0.5)) &&
            isChoreoTracjectoryApplied
        ) {
            isChoreoTracjectoryApplied = false;
            return true;
        } else {
            return false;
        }
    }

    public void runCharacterization(double output) {
        for(int i = 0; i < modules.length; i++) {
            modules[i].runCharacterization(output);
        }
    }

    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for(int i = 0; i < modules.length; i++) {
            headings[i] = DrivetrainConstants.kModuleTranslations[i].getAngle();
        }

        kinematics.resetHeadings(headings);
        stop();
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0))
            .withTimeout(1.0)
            .andThen(sysID.quasistatic(direction));
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0))
            .withTimeout(1.0)
            .andThen(sysID.dynamic(direction));
    }

    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }

        return states;
    }

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getPosition();
        }

        return positions;
    }

    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    private ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for(int i = 0; i < modules.length; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }

        return values;
    }

    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < modules.length; i++) {
            output += modules[i].getFFCharacterizationVelocity() / 4.0;
        }

        return output;
    }

    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    @AutoLogOutput(key = "Odometry/Rotation")
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    public void setPose(Pose2d pose) {
        resetSimulationPoseCallBack.accept(pose);
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    public double getMaxLinearSpeedMetersPerSecond() {
        return DrivetrainConstants.kMaximumLinearVelocityMetersPerSecond;
    }

    public double getMaxAngularSpeedRadiansPerSecond() {
        return DrivetrainConstants.kMaximumLinearAccelerationMetersPerSecondSquared / DrivetrainConstants.kDriveBaseRadius;
    }
}
