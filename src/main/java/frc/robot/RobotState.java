package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class RobotState {

    private static RobotState INSTANCE;
    public static RobotState getInstance(){
        if (INSTANCE == null) {
            INSTANCE = new RobotState();
        }
        return INSTANCE;
    }

    private Pose2d initialPose;
    private Pose2d robotPose;
    private Rotation2d navxOffset;
    private Rotation2d robotRotation2d;
    private SwerveModulePosition[] swerveModulePositions;

    // neutralize all values
    private RobotState(){
        initialPose = new Pose2d();
        robotPose = new Pose2d();
        navxOffset = new Rotation2d(0);
        robotRotation2d = new Rotation2d(0);
    }

    public boolean hasActualState(){
        return swerveModulePositions != null;
    }

    public void addDrivetrainState(DrivetrainState state){
        this.swerveModulePositions = state.getModulePositions();
        this.robotRotation2d = state.getRotation();
    }

    public void updateRobotPose(Pose2d pose){
        this.robotPose = pose;
    }

    // reset pose after reseting offset based on initial rotation value
    public void reset(Pose2d init){
        navxOffset = new Rotation2d();
        navxOffset = init.getRotation();
        initialPose = init;
    }

    // accounts for offset when initialized or reset
    public Rotation2d getRotation(){
        return robotRotation2d.rotateBy(navxOffset);
    }

    // general getters
        public Pose2d getInitialPose(){
            return initialPose;
        }

        public Pose2d getRobotPose(){
            return robotPose;
        }

        public SwerveModulePosition[] getModulePositions(){
            return swerveModulePositions;
        }
}
