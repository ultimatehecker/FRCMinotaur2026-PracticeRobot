package frc.robot.subsystems.vision;

import java.util.function.Supplier;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.VisionConstants;

public class VisionIOSimulation extends VisionIOPhotonVision {
  private static final double DIAGONAL_FOV = 96.0; // FOV in degrees
  private static final int kImgWidth = 1600; // image width in px
  private static final int kImgHeight = 1200; // image heigh in px

  private Supplier<Pose2d> poseSupplier;
  private VisionSystemSim visionSim;
  private PhotonCameraSim cameraSim;

  /**
   * Creates a new VisionIOSim object.
   *
   * @param layout the AprilTag field layout
   * @param poseSupplier a Pose2d supplier that returns the robot's pose based on its odometry
   * @param robotToCamera the transform from the robot's center to the simulated camera
   */
  public VisionIOSimulation(String cameraName, AprilTagFieldLayout layout, Supplier<Pose2d> poseSupplier, Transform3d robotToCamera) {
    super(cameraName, layout, robotToCamera);

    this.poseSupplier = poseSupplier;

    this.visionSim = new VisionSystemSim(cameraName);
    this.visionSim.addAprilTags(layout);
    
    SimCameraProperties cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(kImgWidth, kImgHeight, Rotation2d.fromDegrees(DIAGONAL_FOV));
    cameraProp.setCalibError(VisionConstants.kSimAverageErrorPixels, VisionConstants.kSimErrorStdDevPixels);
    cameraProp.setFPS(30);
    cameraProp.setAvgLatencyMs(100);
    cameraProp.setLatencyStdDevMs(30);

    this.cameraSim = new PhotonCameraSim(camera, cameraProp, layout);

    visionSim.addCamera(cameraSim, robotToCamera);
  }

  /**
   * Updates the specified VisionIOInputs object with the latest data from the camera.
   *
   * @param inputs the VisionIOInputs object to update with the latest data from the camera
   */
  @Override
  public void updateInputs(VisionIOInputs inputs) {
    this.visionSim.update(poseSupplier.get());
    super.updateInputs(inputs);
  }
}