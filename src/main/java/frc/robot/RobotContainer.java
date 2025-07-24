// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {

  Joystick joystick = new Joystick(0);
  Joystick rotation = new Joystick(1);

  Drivetrain drivetrain = Drivetrain.getInstance();
  RobotState robotPose = RobotState.getInstance();
 

  public RobotContainer() {
    Drivetrain.getInstance().setDefaultCommand(
      new DriveCommand(
        joystick::getX, 
        joystick::getY, 
        rotation::getX, 
        () -> true)
    );

    configureBindings();
  }

 
  private void configureBindings() {
    // Intake

    // Shooter
  }

 
  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}
