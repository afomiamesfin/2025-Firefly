// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {

  Joystick joystick = new Joystick(0);
  Joystick rotation = new Joystick(1);

  Drivetrain drivetrain = Drivetrain.getInstance();
  RobotState robotPose = RobotState.getInstance();
 

  public RobotContainer() {
    configureBindings();

    // Drivetrain.getInstance().setDefaultCommand(
    //   Drivetrain.getInstance()
    // );
  }

 
  private void configureBindings() {
    
  }

 
  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}
