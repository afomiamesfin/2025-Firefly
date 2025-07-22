package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class DrivetrainState {
    private final SwerveModulePosition[] modulePositions;
    private final Rotation2d rotation;

    public DrivetrainState(SwerveModulePosition[] modulePositions, Rotation2d rotation) {
        this.modulePositions = modulePositions;
        this.rotation = rotation;
    }

    public SwerveModulePosition[] getModulePositions() {
        return modulePositions;
    }

    public Rotation2d getRotation() {
        return rotation;
    }
}
