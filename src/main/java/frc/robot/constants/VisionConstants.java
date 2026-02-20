package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
      public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

      public static String camera0Name = "frontLeft";
      public static String camera1Name = "frontRight";

      public static Transform3d robotToCamera0 = new Transform3d(Units.inchesToMeters(9.2837556), Units.inchesToMeters(1.6423085), Units.inchesToMeters(6.9584678), new Rotation3d(0.0, Units.degreesToRadians(340), Units.degreesToRadians(324)));
      public static Transform3d robotToCamera1 = new Transform3d(Units.inchesToMeters(9.2859579), Units.inchesToMeters(-2.1461229), Units.inchesToMeters(6.9584678), new Rotation3d(0.0, Units.degreesToRadians(340), Units.degreesToRadians(36)));

      public static double maxAmbiguity = 0.5;
      public static double maxZError = 0.75;

      public static double linearStdDevBaseline = 0.02; // Meters
      public static double angularStdDevBaseline = Double.POSITIVE_INFINITY; // Radians

      public static double[] cameraStdDevFactors = new double[] {
            1.0, // Camera 0
            1.0 // Camera 1
      };

      public static double linearStdDevMegatag2Factor = 0.5; 
      public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; 

      public static final double kSimAverageErrorPixels = 0.1;
      public static final double kSimErrorStdDevPixels = 0.05;
}
