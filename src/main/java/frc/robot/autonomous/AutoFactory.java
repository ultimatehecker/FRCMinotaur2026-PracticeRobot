package frc.robot.autonomous;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class AutoFactory {
    private final DriverStation.Alliance alliance;
    private final RobotContainer robotContainer;
    private final Choreo.TrajectoryCache trajectoryCache;
    
    public AutoFactory(final DriverStation.Alliance alliance, final RobotContainer robotContainer) {
        this.alliance = alliance;
        this.robotContainer = robotContainer;

        trajectoryCache = new Choreo.TrajectoryCache();
    }

    public Pair<Pose2d, Command> initializeIdleCommand(Pose2d startingPose) {
        return Pair.of(startingPose, Commands.idle());
    }

    private String trajectoryName(final Location start, final Location end) {
        var name = "%S_TO_%S".formatted(start, end);
        var x = "%S_%S".formatted(alliance, name);

        System.out.println("%S_%S".formatted(alliance, name));
        return x;
    }


    public Command followChoreoTrajectory(Trajectory<SwerveSample> trajectory) {
        return new InstantCommand(() -> robotContainer.getDrivetrain().followTrajectory(trajectory));
    }
}
