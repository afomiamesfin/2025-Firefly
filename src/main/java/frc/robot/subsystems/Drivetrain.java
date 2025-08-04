// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.studica.frc.AHRS;
import com.team2052.swervemodule.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;

public class Drivetrain extends SubsystemBase {

  private static Drivetrain INSTANCE;
  public static Drivetrain getInstance(){
    if (INSTANCE == null) {
        INSTANCE = new Drivetrain();
    }
    return INSTANCE;
  }

  private final SwerveDriveOdometry odometry;
  private final AHRS navx;

  private RobotState robotState = RobotState.getInstance();
  private ChassisSpeeds currentChassisSpeeds = new ChassisSpeeds(); // general drivetrain velocity

  private final SwerveModule frontLeftWheel;
  private final SwerveModule frontRightWheel;
  private final SwerveModule backLeftWheel;
  private final SwerveModule backRightWheel;

  public double xMPS;
  public double yMPS;
  public double radiansPS;

  private static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    // front left
      new Translation2d(Constants.Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
    // front right
      new Translation2d(Constants.Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
    // back left
      new Translation2d(-Constants.Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
    // back right
      new Translation2d(-Constants.Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );
  
  public Drivetrain() {

    /*
     * Configure Modules and Navx
     */
    navx = new AHRS(AHRS.NavXComType.kMXP_SPI);

    frontLeftWheel = new SwerveModule(
      "frontLeftModule",
      Constants.Drivetrain.FRONT_LEFT_MODULE_DRIVE_MOTOR, 
      Constants.Drivetrain.FRONT_LEFT_MODULE_ANGLE_MOTOR,
      Constants.Drivetrain.FRONT_LEFT_MODULE_ENCODER,
      new Rotation2d(Constants.Drivetrain.FRONT_LEFT_MODULE_ANGLE_OFFSET_RADIANS)
    );
    frontRightWheel = new SwerveModule(
      "frontRightModule",
      Constants.Drivetrain.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
      Constants.Drivetrain.FRONT_RIGHT_MODULE_ANGLE_MOTOR,
      Constants.Drivetrain.FRONT_RIGHT_MODULE_ENCODER,
      new Rotation2d(Constants.Drivetrain.FRONT_RIGHT_MODULE_ANGLE_OFFSET_RADIANS)
    );
    backLeftWheel = new SwerveModule(
      "backLeftModule",
      Constants.Drivetrain.BACK_LEFT_MODULE_DRIVE_MOTOR, 
      Constants.Drivetrain.BACK_LEFT_MODULE_ANGLE_MOTOR,
      Constants.Drivetrain.BACK_LEFT_MODULE_ENCODER,
      new Rotation2d(Constants.Drivetrain.BACK_LEFT_MODULE_ANGLE_OFFSET_RADIANS)

    );
    backRightWheel = new SwerveModule(
      "backRightModule",
      Constants.Drivetrain.BACK_RIGHT_MODULE_DRIVE_MOTOR, 
      Constants.Drivetrain.BACK_RIGHT_MODULE_ANGLE_MOTOR,
      Constants.Drivetrain.BACK_RIGHT_MODULE_ENCODER,
      new Rotation2d(Constants.Drivetrain.BACK_RIGHT_MODULE_ANGLE_OFFSET_RADIANS)
    );

    odometry = new SwerveDriveOdometry(kinematics, navx.getRotation2d(), getModulePositions());
  }

/*
  * Drive methods
  */

  // robot relative
  public void driveChassisSpeeds(ChassisSpeeds speeds){
    // TODO: add logger functionality
    currentChassisSpeeds = speeds;
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
    setModuleStates(moduleStates);
  }

  // Manually entered speeds turn into ChassisSpeeds
  // all parmeters must be normalized [-1.0, 1.0]
  public void driveNorm(
    double normalizedXVelo,
    double normalizedYVelo,
    double normalizedRotationVelo,
    boolean fieldCentric
  ){
    // ensure normalized values while maintaining the sign
    normalizedXVelo = Math.copySign(
      Math.min(Math.abs(normalizedXVelo), 1.0), 
      normalizedXVelo);

    normalizedYVelo = Math.copySign(
      Math.min(Math.abs(normalizedYVelo), 1.0), 
      normalizedYVelo);

    normalizedRotationVelo = Math.copySign(
      Math.min(Math.abs(normalizedRotationVelo), 1.0), 
      normalizedRotationVelo);

    ChassisSpeeds speeds = new ChassisSpeeds(
      normalizedXVelo * SwerveModule.getMaxVelocityMetersPerSecond(), // turns # into fractional value of max speed
      normalizedYVelo * SwerveModule.getMaxVelocityMetersPerSecond(),
      normalizedRotationVelo * SwerveModule.getMaxAngularVelocityRadiansPerSecond()
    );
    
    int invert = -1; // TODO: do we want to invert?

    driveChassisSpeeds(
      fieldCentric ? 
        ChassisSpeeds.fromRobotRelativeSpeeds(
          speeds.vxMetersPerSecond * invert,
          speeds.vyMetersPerSecond * invert,
          speeds.omegaRadiansPerSecond,
          new Rotation2d(-navx.getRotation2d().getRadians())
        )
      : speeds
    ); 
  }

  /*
   * Key Setters and Getters
   */

  // TODO: validate setModuleStates) method from 2025Scarab rewrite
  public void setModuleStates(SwerveModuleState[] states) {
    // if wheels don't have a drive velocity, maintain the current orientation 
    boolean robotMoving = 
      states[0].speedMetersPerSecond != 0
      || states[1].speedMetersPerSecond != 0
      || states[2].speedMetersPerSecond != 0
      || states[3].speedMetersPerSecond != 0;

    frontLeftWheel.setState(
      states[0].speedMetersPerSecond,
      robotMoving ? states[0].angle : frontLeftWheel.getState().angle);
    frontRightWheel.setState(
      states[1].speedMetersPerSecond,
      robotMoving ? states[1].angle : frontRightWheel.getState().angle);
    backLeftWheel.setState(
      states[2].speedMetersPerSecond,
      robotMoving ? states[2].angle : backLeftWheel.getState().angle);
    backRightWheel.setState(
      states[3].speedMetersPerSecond,
      robotMoving ? states[3].angle : backRightWheel.getState().angle);
  }

   private SwerveModulePosition[] getModulePositions(){
    return new SwerveModulePosition[]{
          frontLeftWheel.getPosition(),
          frontRightWheel.getPosition(),
          backLeftWheel.getPosition(),
          backRightWheel.getPosition()
        };
  }

  public ChassisSpeeds getSpeeds(){
    return currentChassisSpeeds;
  }

  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  @Override
  public void periodic() {
      debugAll();
      odometry.update(navx.getRotation2d(), getModulePositions());
      // System.out.println("Current Gyro Reading Degrees: " + navx.getRotation2d().getDegrees());
  }

  /*
   * Maintenance methods
   */
  public void stop(){
    driveNorm(0, 0, 0, false);
  }

  public void zeroGyro(){
    navx.zeroYaw();
  }

  public void resetPose(Pose2d initPose){
    odometry.resetPose(initPose);
    odometry.resetPosition(navx.getRotation2d(), getModulePositions(), initPose);
  }

  public void debugAll(){
    frontLeftWheel.debug("Front Left Wheel");
    frontRightWheel.debug("Front Right Wheel");
    backLeftWheel.debug("Back Left Wheel");
    backRightWheel.debug("Back Right Wheel");
    Logger.recordOutput("Robot Pose2d", odometry.getPoseMeters());
  }
}
