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

  public static final class Drivetrain{ // TODO: validate measurements
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(22.5);
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(23.25);

  }
}
