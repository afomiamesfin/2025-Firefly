// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {

  Joystick joystick = new Joystick(0);
  Joystick rotation = new Joystick(1);

  Drivetrain drivetrain = Drivetrain.getInstance();
  RobotState robotPose = RobotState.getInstance();
  HopperSubsystem hoppper = HopperSubsystem.getInstance();
  IndexerSubsystem indexer = IndexerSubsystem.getInstance();
  IntakeSubsystem intake = IntakeSubsystem.getInstance();
  ShooterSubsystem shooter = ShooterSubsystem.getInstance();
 

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

  JoystickButton intakeButton;
  JoystickButton outtakeOneButton;
  JoystickButton outtakeAllButton;

  JoystickButton shoot3ftButton;
  JoystickButton shoot7ftButton;

  JoystickButton adjustableShootButton;
  JoystickButton angle1Button;
  JoystickButton angle2Button;

  JoystickButton zeroGyroButton;
 
  private void configureBindings() {
    // Intake

    // Shooter
    
  }

 
  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}
