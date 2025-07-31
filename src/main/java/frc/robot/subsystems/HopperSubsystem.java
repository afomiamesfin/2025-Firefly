// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorIDs;

public class HopperSubsystem extends SubsystemBase {

  private static HopperSubsystem INSTANCE;
  public static HopperSubsystem getInstance(){
    if (INSTANCE == null) {
        INSTANCE = new HopperSubsystem();
    }
    return INSTANCE;
  }

    private TalonSRX hopperMotor;

    public HopperSubsystem() {
        hopperMotor = new TalonSRX(MotorIDs.HOPPER_MOTOR);
    }

    public void run() {
        hopperMotor.set(ControlMode.PercentOutput, Constants.Intake.HOPPER_SPEED_PCT);
    }

    public void stop() {
        hopperMotor.set(ControlMode.PercentOutput, 0);
    }

    public void reverse() {
        hopperMotor.set(ControlMode.PercentOutput, -Constants.Intake.HOPPER_SPEED_PCT);
    }

  @Override
  public void periodic() {

  }
}
