// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.util.struct.StructGenerator;
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
        System.out.println("SHOOTER INDEXING COMMAND");
        boolean stagedCargoDetected = indexer.getCargoStagedDetected();
        boolean preStagedCargoDetected = indexer.getCargoPreStagedDetected();

        System.out.println("STAGED?:" + stagedCargoDetected + " PRESTAGED?: " + preStagedCargoDetected);

        return Commands.run(
            () -> {
                if (mode == ShootMode.SHOOT_ALL && !stagedCargoDetected && preStagedCargoDetected) { 
                    System.out.println("only prestaged, run feeder & preload");
                // ball ready to fire, 2nd not detected
                    indexer.runFeeder();
                    indexer.runPreload();
                    hopper.run();
                } else if (shooter.isAtSpeed()) {
                    System.out.println("shooter at speed");
                // ready to fire - 1 or 2?
                    indexer.runFeeder();
                    hopper.run();
                    if (mode == ShootMode.SHOOT_ALL){
                        indexer.runPreload();
                    }
                } else {
                // not tryna shoot yet - are we loading 2nd ball?
                System.out.println("loading? not trying to shoot");
                    indexer.stopFeeder();
                    if (preStagedCargoDetected) {
                        indexer.stopPreload();
                        hopper.stop();
                    } else {
                        indexer.runPreload();
                        hopper.run();
                    }
                }
            }
            )
        .finallyDo(() -> {
            System.out.println("===== STOPPING SHOOTER INDEXING COMMAND");
            hopper.stop();
            indexer.stopFeeder();
            indexer.stopPreload();
        });
    }

// SHOOTER COMMAND - shoot balls manually w/o vision, manual wheel speeds or ShooterTrajConfig
    public static Command shootPercentage(ShootMode shootmode, double topPercent, double bottomPercent, FiringAngle angleMode){
        System.out.println("===== RUNNING SHOOT PERCENTAGE");
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
        return Commands.run(
            () -> {
                if(angleMode == FiringAngle.ANGLE_1){
                    shooter.shootAtPercentage(75, 75);
                } else {
                    shooter.shootAtPercentage(50, 50);
                }
                System.out.println("==== RUNNING SHOOT PERCENTAGE COMMAND");
            }
            ) // or run pct output
            // shooterIndexingCommand(shootmode)
        .finallyDo(
            () -> { // stop everyone
                System.out.println("===== END SHOOT PERCENTAGE COMMAND");
                shooter.stop();
            }
        );
    }

    public static Command manualRunIndexer(){
        return Commands.run(
            indexer::runFeeder, 
            indexer)
        .finallyDo(indexer::stopFeeder);
    }
    
    public static Command manualIndexOut(){
        return Commands.run(
            indexer::runFeederReverse,
            indexer)
        .finallyDo(indexer::stopFeeder);
    }

// Throttle Shoot Command - https://github.com/frc2052/2024FireflyAutos/blob/2025Swerve/src/main/java/frc/robot/commands/shooter/ThrottleShootCommand.java
// based on %ct of throttle
    public static double convertThrottleToPercent(double throttle){
        double throttlePercent = throttle * 100;
        // System.out.println(throttlePercent);
        return throttlePercent;
        // return (int)Math.round(throttlePercent * 17000 + 2000);
    }
    public static Command basicThrottleShootCommand(DoubleSupplier throttle){
        return Commands.run(
            () -> {
                double percentRun = 
                convertThrottleToPercent(throttle.getAsDouble()) < 25?
                50 : convertThrottleToPercent(throttle.getAsDouble());

                System.out.println("==== THROTTLE AS PERCENT: " + percentRun);

                shooter.shootAtPercentage(percentRun, percentRun);
            },
            shooter)
            .finallyDo(
                () -> shooter.stop()
            );
    }
    public static Command throttleShootCommand(ShootMode mode, DoubleSupplier throttle){
        return Commands.parallel(
            // dynamic shooter speed (from throttle)
            Commands.run(
                () -> {
                    double speed = convertThrottleToPercent(throttle.getAsDouble());
                    System.out.println("===== THROTTLE SHOOT COMMAND SPEED (TPS): " + speed);
                    shooter.runAtSpeed(speed, speed);
                }, 
                shooter),

            // shooter indexing logic
            shooterIndexingCommand(mode)
        );
    }



}
