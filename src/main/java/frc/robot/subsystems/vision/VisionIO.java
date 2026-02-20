package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.minolib.vision.PhotonFiducialResult;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        boolean connected;
        public int[] tagIds = new int[0];
        PhotonFiducialResult[] poseObservations = new PhotonFiducialResult[0];
    }

    public enum PoseObservationType {
        SINGLE_TAG,
        MULTI_TAG
    }

    default void updateInputs(VisionIOInputs inputs) {}
}