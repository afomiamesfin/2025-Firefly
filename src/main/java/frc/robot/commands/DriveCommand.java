// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class DriveCommand extends Command {

  Drivetrain drivetrain = Drivetrain.getInstance();

  // suppliers - will be joystick inputs (or defaults for manual autos)
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier omegaSupplier;
  private final BooleanSupplier fieldCentricSupplier;

  // SlewRateLimiter limits the ROC of a value (usually voltage or velocity)
  private final SlewRateLimiter xLim;
  private final SlewRateLimiter yLim;
  private final SlewRateLimiter omegaLim;


  public DriveCommand(
    DoubleSupplier xSupplier,
    DoubleSupplier ySupplier,
    DoubleSupplier omegaSupplier,
    BooleanSupplier fieldCentricSupplier
  ) {
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.omegaSupplier = omegaSupplier;
    this.fieldCentricSupplier = fieldCentricSupplier;

    xLim = new SlewRateLimiter(2); // rate in units / second
    yLim = new SlewRateLimiter(2);
    omegaLim = new SlewRateLimiter(3);

    addRequirements(Drivetrain.getInstance());
  }

  /*
   * Getter Methods
   */

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  /*
   * Maintenance Methods
   */
  protected double slewAxis(SlewRateLimiter lim, double val){
    return lim.calculate(Math.copySign(Math.pow(val, 2), val));
  }

// protected double deadBand(double value) {
//     if (Math.abs(value) <= 0.15) {
//         return 0.0;
//     }
//     // Limit the value to always be in the range of [-1.0, 1.0]
//     return Math.copySign(Math.min(1.0, Math.abs(value)), value);
// }
}
