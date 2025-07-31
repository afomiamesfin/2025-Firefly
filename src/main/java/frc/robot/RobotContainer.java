// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommandFactory;
import frc.robot.commands.IntakeCommandFactory.OuttakeMode;
import frc.robot.commands.ShootCommandFactory;
import frc.robot.commands.ShootCommandFactory.ShootMode;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.FiringAngle;

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
  // ------------------------- BUTTON CREATION ------------------------- //
    // intake buttons
    intakeButton = new JoystickButton(joystick, 1);
    outtakeOneButton = new JoystickButton(joystick, 7);
    outtakeAllButton = new JoystickButton(joystick,118);
  
    // shooter buttons
    shoot3ftButton = new JoystickButton(joystick, 5);
    shoot7ftButton = new JoystickButton(joystick, 3);
    adjustableShootButton = new JoystickButton(joystick, 4);
  
    angle1Button = new JoystickButton(joystick, 9);
    angle2Button = new JoystickButton(joystick, 10);
  
    zeroGyroButton = new JoystickButton(joystick, 8);

  // ------------------------- BUTTON'S COMMANDS ------------------------- //
    // intake commands
    Command teleopIntakeOnCommand = 
      new ConditionalCommand(
        // insure intake doesn't stopthe shooter (hopper requirement)
        // shoot pressed? only intake
        IntakeCommandFactory.teleopOnlyIntakeCommand(), 
        // shoot not pressed? intake all
        IntakeCommandFactory.teleopIntakeCommand(), 
        () -> (shoot3ftButton.getAsBoolean() || shoot7ftButton.getAsBoolean()));
    
    Command outtake1Command = IntakeCommandFactory.outtake(OuttakeMode.ONE_BALL);
    
    Command outtakeAllCommand = IntakeCommandFactory.outtake(OuttakeMode.ALL_BALLS);

    // shooter commands
    // TODO: confirm that these are the correct speeds? not using vision anymore
      // math may be contained within the 
    Command threeFootShootCommand = ShootCommandFactory.shootCommand1(
      ShootMode.SHOOT_ALL, 
      Constants.Shooter.SHOOTER_TOP_PULLDOWN_PCT, 
      Constants.Shooter.SHOOTER_BOTTOM_PULLDOWN_PCT, 
      FiringAngle.ANGLE_1);
    
    Command sevenFootShootCommand = ShootCommandFactory.shootCommand1(
      ShootMode.SHOOT_ALL, 
      Constants.Shooter.SHOOTER_TOP_PULLDOWN_PCT, 
      Constants.Shooter.SHOOTER_BOTTOM_PULLDOWN_PCT, 
      FiringAngle.ANGLE_2);

    Command adjustableShootCommand = 
      ShootCommandFactory.throttleShootCommand(
        ShootMode.SHOOT_ALL, 
        joystick::getThrottle);

  // ------------------------- BUTTON BINDINGS ------------------------- //
    // intake button
    intakeButton.whileTrue(teleopIntakeOnCommand);
    outtakeOneButton.whileTrue(outtake1Command);
    outtakeAllButton.whileTrue(outtakeAllCommand);

    // shooter button - need to adust throttle speeds
    shoot3ftButton.whileTrue(threeFootShootCommand);
    shoot7ftButton.whileTrue(sevenFootShootCommand);
    adjustableShootButton.whileTrue(adjustableShootCommand);

    angle1Button.onTrue(new InstantCommand(() -> shooter.setShootAngle1()));
    angle2Button.onTrue(new InstantCommand(() -> shooter.setShootAngle2()));

    zeroGyroButton.onTrue(new InstantCommand(() -> drivetrain.zeroGyro()));
  }

 
  public Command getAutonomousCommand() {
    System.out.println("RAN AUTONOMOUS COMMAND - INSTANT COMMAND");
    return new InstantCommand();
  }
}
