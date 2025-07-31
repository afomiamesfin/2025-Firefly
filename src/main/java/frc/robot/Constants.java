// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class MotorIDs { // TODO: adjust to appropriate values
    public static final int TOP_SHOOTER_MOTOR = 15;
    public static final int BOTTOM_SHOOTER_MOTOR = 16;
    public static final int HOPPER_MOTOR = 17; // "hz blue compliant wheels"
    public static final int PRELOAD_INDEXER_MOTOR = 18; // "green compliant wheel "
    public static final int FEEDER_INDEXER_MOTOR = 19; // "small blue wheel under shooter"
    public static final int INTAKE_MOTOR = 20;
  }

  public static class SolenoidIDs {
    public static final int COMPRESSOR_MODULE_ID = 13;

    public static final int INTAKE_OUT = 5;
    public static final int INTAKE_IN = 4;

    public static final int SHOOTER_OUT = 7;
    public static final int SHOOTER_IN = 6;
  }

  public static class LimitSwitchIDs { 
    public static final int CLAW_A_SWITCH = 7;
    public static final int CLAW_B_SWITCH = 2;
    public static final int INDEXER_PRELOAD = 9;
    public static final int INDEXER_FEEDER = 8;
  }

  public static final class ShooterIDs { // TODO: adjust speeds
    public static final double PRELOAD_WHEEL_SPEED = 1;
    public static final double TOP_WHEEL_SPEED = 5;
    public static final double BOTTOM_WHEEL_SPEED = 5;
    public static final double INDEXER_WHEEL_SPEED = 0.75;

    public static final double SHOOTER_TOLERANCE = 0.01;
    public static final double SHOOTER_TOP_PULLDOWN_PCT = 0.97;
    public static final double SHOOTER_BOTTOM_PULLDOWN_PCT = 0.97;

    public static final double ANGLE_CHANGE_THRESHOLD_INCHES = 66;
    public static final double ANGLE_CHANGE_TOLERANCE_INCHES = 0.6;
  }

  public static final class Drivetrain{ // TODO: validate all values
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(22.5);
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(23.25);

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 4;
    public static final int FRONT_LEFT_MODULE_ANGLE_MOTOR = 6;
    public static final int FRONT_LEFT_MODULE_ENCODER = 5;
    public static final double FRONT_LEFT_MODULE_ANGLE_OFFSET_RADIANS = Math.toRadians(217.6);//-12.62054443359375

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 9;
    public static final int FRONT_RIGHT_MODULE_ANGLE_MOTOR = 7;
    public static final int FRONT_RIGHT_MODULE_ENCODER = 8;
    public static final double FRONT_RIGHT_MODULE_ANGLE_OFFSET_RADIANS = Math.toRadians(175.7);//1.79901123046875

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 1;
    public static final int BACK_LEFT_MODULE_ANGLE_MOTOR = 3;
    public static final int BACK_LEFT_MODULE_ENCODER = 2;
    public static final double BACK_LEFT_MODULE_ANGLE_OFFSET_RADIANS = Math.toRadians(39.9);//-360.99700927734375

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 12;
    public static final int BACK_RIGHT_MODULE_ANGLE_MOTOR = 10;
    public static final int BACK_RIGHT_MODULE_ENCODER = 11;
    public static final double BACK_RIGHT_MODULE_ANGLE_OFFSET_RADIANS = Math.toRadians(61.1);//-360.14556884765625
  }

  public static final class Intake {  // set intake & hopper speeds
    public static final double INTAKE_SPEED_PCT = .90;
    public static final double HOPPER_SPEED_PCT = .75;
  }

  public static final class LimitSwitch {
    public static final int CLAW_A_LIMIT_SWITCH = 7;
    public static final int CLAW_B_LIMIT_SWITCH = 2;
    public static final int INDEXER_PRELOAD = 9;
    public static final int INDEXER_FEEDER = 8;
  }

public static final class Shooter { // TODO: validate
  public static final double PRELOAD_WHEEL_SPEED = 1;
  public static final double TOP_WHEEL_SPEED = 5;
  public static final double BOTTOM_WHEEL_SPEED = 5;
  public static final double INDEXER_WHEEL_SPEED = 0.75;

  public static final double SHOOTER_TOLERANCE = 0.01;
  public static final double SHOOTER_TOP_PULLDOWN_PCT = 0.97;
  public static final double SHOOTER_BOTTOM_PULLDOWN_PCT = 0.97;

  // public static final double ANGLE_CHANGE_THRESHOLD_TY = -5.0;
  // public static final double ANGLE_CHANGE_TOLERANCE_DEGREES = 1; //

  public static final double ANGLE_CHANGE_THRESHOLD_DISTANCE_INCHES = 66;
  public static final double ANGLE_CHANGE_TOLERANCE_DISTANCE_INCHES = 6;

  public static final double FIRING_ANGLE_1_DEGREES = 72.0;
  public static final double FIRING_ANGLE_2_DEGREES = 60.0;
  public static final double FLYWHEEL_RADIUS_METERS = Units.inchesToMeters(2);

  public static final double SHOOTER_MOUNT_HEIGHT_METERS = Units.inchesToMeters(20);
  public static final int DEFAULT_ASSUMED_SHOOTER_CONFIG = 0;
}
}
