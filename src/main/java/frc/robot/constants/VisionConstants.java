package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
      public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

      public static String camera0Name = "namehere";
      public static String camera1Name = "namehere";

      public static Transform3d robotToCamera0 = new Transform3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0));
      public static Transform3d robotToCamera1 = new Transform3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0));

      public static double maxAmbiguity = 0.3;
      public static double maxZError = 0.75;

      public static double linearStdDevBaseline = 0.02; // Meters
      public static double angularStdDevBaseline = 0.06; // Radians

      public static double[] cameraStdDevFactors = new double[] {
            1.0, // Camera 0
            1.0 // Camera 1
      };

      public static double linearStdDevMegatag2Factor = 0.5; 
      public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; 
}
