package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class Module {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int index;

    private final Alert driveMotorDisconnected;
    private final Alert steerMotorDisconnected;
    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

    public Module(ModuleIO io, int index) {
        this.io = io;
        this.index = index;

        driveMotorDisconnected = new Alert("Drive Motor on module " + Integer.toString(index) + " is currently disconnected.", AlertType.kError);
        steerMotorDisconnected = new Alert("Steer Motor on module " + Integer.toString(index) + " is currently disconnected.", AlertType.kError);
    }
}
