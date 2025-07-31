// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ShootCommandFactory.ShootMode;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.FiringAngle;

public class ShootCommandFactory { // ALL COMMANDS FROM OLD SHOOTER COMMAND FOLDER
    private static final IntakeSubsystem intake = IntakeSubsystem.getInstance();
    private static final IndexerSubsystem indexer = IndexerSubsystem.getInstance();
    private static final HopperSubsystem hopper = HopperSubsystem.getInstance(); 
    private  static final ShooterSubsystem shooter = ShooterSubsystem.getInstance();

// STOP ALL - safety method for all commands
    public static Command STOPALL(){
        return Commands.run(
            () -> {
                intake.stop();
                indexer.stopFeeder();
                indexer.stopPreload();
                hopper.stop();
                shooter.stop();
            }
        );
    }

// Shooter Indexing Command - command to index balls for the shooter (used in other shoot commands)
    public enum ShootMode{
        SHOOT_SINGLE,
        SHOOT_ALL
    }
    public boolean baseIsReadyToShoot(){ // replace for other shoot commands for custom control
        return shooter.isAtSpeed();
    }
    public static Command shooterIndexingCommand(ShootMode mode){
        boolean stagedCargoDetected = indexer.getCargoStagedDetected();
        boolean preStagedCargoDetected = indexer.getCargoPreStagedDetected();

        return Commands.run(
            () -> {
                if (mode == ShootMode.SHOOT_ALL && !stagedCargoDetected && preStagedCargoDetected) { 
                // ball ready to fire, 2nd not detected
                    indexer.runFeeder();
                    indexer.runPreload();
                    hopper.run();
                } else if (shooter.isAtSpeed()) {
                // ready to fire - 1 or 2?
                    indexer.runFeeder();
                    hopper.run();
                    if (mode == ShootMode.SHOOT_ALL){
                        indexer.runPreload();
                    }
                } else {
                // not tryna shoot yet - are we loading 2nd ball?
                    indexer.stopFeeder();
                    if (preStagedCargoDetected) {
                        indexer.stopPreload();
                        hopper.stop();
                    } else {
                        indexer.runPreload();
                        hopper.run();
                    }
                }
            }, 
            indexer, hopper)
        .finallyDo(() -> {
            hopper.stop();
            indexer.stopFeeder();
            indexer.stopPreload();
        });
    }

// SHOOTER COMMAND - shoot balls manually w/o vision, manual wheel speeds or ShooterTrajConfig
    public static Command shootCommand1(ShootMode shootmode, double topVelo, double bottomVelo, FiringAngle angleMode){
        // initialize() - set shoot angle
        if(null == angleMode){}
        else {
            switch (angleMode) {
                case ANGLE_1:
                    shooter.setShootAngle1();
                    break;
                default:
                    shooter.setShootAngle2();
                    break;
            }
        }

        // execute() + super.execute() -> super is shooterIndexingCommand, try to combine w/ scheduler
        return Commands.parallel(
            Commands.run(() -> shooter.runAtSpeed(topVelo, bottomVelo), shooter, indexer), // or run pct output
            shooterIndexingCommand(shootmode)
        )
        .finallyDo(
            () -> { // stop everyone
                hopper.stop();
                indexer.stopFeeder();
                indexer.stopPreload();
                shooter.stop();
            }
        );
    }

// Throttle Shoot Command - https://github.com/frc2052/2024FireflyAutos/blob/2025Swerve/src/main/java/frc/robot/commands/shooter/ThrottleShootCommand.java
// based on %ct of throttle
    public static int convertThrottleToPower(double throttle){
        double throttlePercent = Math.abs(throttle - 1) / 2;
        return (int)Math.round(throttlePercent * 17000 + 2000);
    }
    public static Command throttleShootCommand(ShootMode mode, DoubleSupplier throttle){
        return Commands.parallel(
            // dynamic shooter speed (from throttle)
            Commands.run(
                () -> {
                    double speed = convertThrottleToPower(throttle.getAsDouble());
                    System.out.println("===== THROTTLE SHOOT COMMAND SPEED (TPS): " + speed);
                    shooter.runAtSpeed(speed, speed);
                }, 
                shooter),

            // shooter indexing logic
            shooterIndexingCommand(mode)
        );
    }



}
