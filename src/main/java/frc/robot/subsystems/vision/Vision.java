package frc.robot.subsystems.vision;

import java.util.LinkedList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;

public class Vision extends SubsystemBase {
    private final VisionConsumer consumer;
    private final VisionIO[] io;
    private final VisionIOInputsAutoLogged[] inputs;
    private final Alert[] disconnectedAlerts;

    public Vision(VisionConsumer consumer, VisionIO... io) {
        this.consumer = consumer;
        this.io = io;

        this.inputs = new VisionIOInputsAutoLogged[io.length];
        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
        }

        this.disconnectedAlerts = new Alert[io.length];
        for (int i = 0; i < inputs.length; i++) {
            disconnectedAlerts[i] = new Alert("Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
        }
    }

    @Override
    public void periodic() {
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
        }

        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();

        for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

            for (int tagId : inputs[cameraIndex].tagIds) {
                var tagPose = VisionConstants.aprilTagLayout.getTagPose(tagId);
                if (tagPose.isPresent()) {
                    tagPoses.add(tagPose.get());
                }
            }

            for (var observation : inputs[cameraIndex].poseObservations) {
                boolean rejectPose = observation.numTags() == 0 || (observation.numTags() == 1 && observation.averageAmbiguity() > VisionConstants.maxAmbiguity) //|| Math.abs(observation.cameraPose().getZ()) > VisionConstants.maxZError 
                    || observation.cameraPose().getX() < 0.0
                    || observation.cameraPose().getX() > VisionConstants.aprilTagLayout.getFieldLength()
                    || observation.cameraPose().getY() < 0.0
                    || observation.cameraPose().getY() > VisionConstants.aprilTagLayout.getFieldWidth();

                // Add pose to log
                robotPoses.add(observation.cameraPose());

                if (rejectPose) {
                    robotPosesRejected.add(observation.cameraPose());
                } else {
                    robotPosesAccepted.add(observation.cameraPose());
                }

                // Skip if rejected
                if (rejectPose) {
                    continue;
                }

                double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.numTags();
                double linearStdDev = VisionConstants.linearStdDevBaseline * stdDevFactor;
                double angularStdDev = VisionConstants.angularStdDevBaseline * stdDevFactor;

                if (cameraIndex < VisionConstants.cameraStdDevFactors.length) {
                    linearStdDev *= VisionConstants.cameraStdDevFactors[cameraIndex];
                    angularStdDev *= VisionConstants.cameraStdDevFactors[cameraIndex];
                }

                //Send vision observation
                consumer.accept(
                    observation.cameraPose().toPose2d(),
                    observation.timestamp(),
                    VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)
                );
            }

            Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
            Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses", robotPoses.toArray(new Pose3d[0]));
            Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted", robotPosesAccepted.toArray(new Pose3d[0]));
            Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected", robotPosesRejected.toArray(new Pose3d[0]));

            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);
        }

        // Log summary data
        Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
    }

    @FunctionalInterface
    public static interface VisionConsumer {
        public void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);
    }
}