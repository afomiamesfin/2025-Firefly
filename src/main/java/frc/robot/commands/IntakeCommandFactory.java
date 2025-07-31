// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommandFactory {
    // ALL COMMANDS FROM OLD INTAKE COMMAND FOLDER
    private static final IntakeSubsystem intake = IntakeSubsystem.getInstance();
    private static final IndexerSubsystem indexer = IndexerSubsystem.getInstance();
    private static final HopperSubsystem hopper = HopperSubsystem.getInstance();

    // HopperBaseCommand - base command to run and control the hopper, being the indexer blue and green wheels and hopper compliant wheels 
    public static Command hopperBaseCommand(){
        return Commands.run(
            () -> {
                if (!indexer.getCargoStagedDetected()) { 
                    // run wheels until all balls staged
                    indexer.runPreload();
                    indexer.runFeeder();
                    hopper.run();  
                } else if (indexer.getCargoStagedDetected() && !indexer.getCargoPreStagedDetected()) {
                    // ball ready to be fired, 2nd one not staged
                    indexer.stopFeeder();
                    indexer.runPreload();
                    hopper.run();
                } else {
                    indexer.stopFeeder();
                    indexer.stopPreload();
                    hopper.stop();
                }
            }, 
            new Subsystem(){})  // no requirements (mad @ null value), need to be able to run intake while shooting
            // isFinished() >>>
            .until(() -> (indexer.getCargoPreStagedDetected() && indexer.getCargoPreStagedDetected()))
            // end() >>>
            .finallyDo((interrupted) -> {
                indexer.stopFeeder();
                indexer.stopPreload();
                hopper.stop();
            });
    }

    // Intake Arm Toggle Command - toggle intake up or down
    public static Command intakeArmToggleCommand(){
        return new ConditionalCommand(
        new InstantCommand(() -> intake.armIn()), 
        new InstantCommand(() -> intake.armOut()), 
        () -> intake.isArmOut());
    }

    // Intake Hopper Run Command - don't allow more balls to be picked up if both stage / prestage are full - check first
    public static Command intakeHopperRunCommand(){
        return hopperBaseCommand()
            .alongWith(
                Commands.run(
                // just run hopper base command + intake IF both stages empty
                    () -> {
                        if (!(indexer.getCargoStagedDetected() && indexer.getCargoPreStagedDetected())) {
                            intake.run();
                        }
                    },
                    new Subsystem() {} // no requirements
                )
            )
            .finallyDo((interrupted) -> { // uses until from hopperBaseCommandMethod as isFinished() method
                intake.stop(); // hopperBaseCommand’s finallyDo should already handle stopping indexer/hopper
            });
    }

    // Intake Stop Command - halt intake + indexer
    public static Command intakeStopCommand(){
        return Commands.run(
            () -> {
                intake.stop();
                hopper.stop();
            }, 
            intake, hopper);
    }

    // Only Intake Command - only control intake in motor & NOTHING else
    public static Command onlyIntakeCommand(){
        return Commands.runEnd(() -> intake.run(), () -> intake.stop(), intake);
    }

    // Outtake - reverse intake, hopper, & indexer for 1/2 balls - depends on OuttakeMode
    public enum OuttakeMode{
        ONE_BALL,
        ALL_BALLS
    }
    public static Command outtake(OuttakeMode mode){
        return Commands.runEnd(
            () -> {
                intake.reverse();
                hopper.reverse();
                indexer.runPreloadReverse();
                if(mode == OuttakeMode.ALL_BALLS){indexer.runFeederReverse();}
            }, 
            () -> {
                intake.stop();
                hopper.stop();
                indexer.stopPreload();
                indexer.stopFeeder();
            }, 
            intake, hopper, indexer);
    }

    // Teleop Intake - command used in teleop to lower + run intake & put it up when finished
    public static Command teleopIntakeCommand(){
        return intakeHopperRunCommand()
        .beforeStarting(() -> intake.armOut())
        .finallyDo(() -> intake.armIn());
    }

    // Only Intake Command - run intake, lower before and put up when finished
    public static Command teleopOnlyIntakeCommand(){
        return onlyIntakeCommand()
        .beforeStarting(() -> intake.armOut())
        .finallyDo(() -> intake.armIn());
    }
}
