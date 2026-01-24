package frc.robot.autonomous;

import java.util.function.Function;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoRoutine {
    public final Auto auto;
    private final String name;

    private final Function<AutoFactory, Pair<Pose2d, Command>> commandFactory;

    public AutoRoutine(final Auto auto, final String name, final Function<AutoFactory, Pair<Pose2d, Command>> commandFactory) {
        this.auto = auto;
        this.name = name;
        this.commandFactory = commandFactory;
    }

    public Auto getAuto() {
        return auto;
    }

    public String getName() {
        return name;
    }

    public Pose2d getStartingPose(final AutoFactory autoFactory) {
        return commandFactory.apply(autoFactory).getFirst();
    }

    public Command getCommand(final AutoFactory autoFactory) {
        return commandFactory.apply(autoFactory).getSecond();
    }
}
