package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        boolean connected;
        public int[] tagIds = new int[0];
        PoseObservation[] poseObservations = new PoseObservation[0];
    }

    public static record PoseObservation(
        double timestamp,
        Pose3d cameraPose,
        double latencySecs,
        double averageAmbiguity,
        double reprojectionError,
        long tagsSeenBitMap,
        int numTags,
        double averageTagDistance,
        PoseObservationType type
    ) {}

    public enum PoseObservationType {
        SINGLE_TAG,
        MULTI_TAG
    }

    default void updateInputs(VisionIOInputs inputs) {}
}