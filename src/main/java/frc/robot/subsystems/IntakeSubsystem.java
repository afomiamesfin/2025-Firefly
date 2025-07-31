package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants.SolenoidIDs;

public class IntakeSubsystem extends SubsystemBase {

    private static IntakeSubsystem INSTANCE;
    public static IntakeSubsystem getInstance(){
        if (INSTANCE == null) {
            INSTANCE = new IntakeSubsystem();
        }
        return INSTANCE;
    }

    private final DoubleSolenoid armSolenoid;
    private TalonSRX intakeMotor;
    private boolean isArmOut;
    private boolean intakeRunning;

    public IntakeSubsystem() {
        armSolenoid = new DoubleSolenoid(
            Constants.SolenoidIDs.COMPRESSOR_MODULE_ID, 
            PneumaticsModuleType.REVPH,
            SolenoidIDs.INTAKE_IN, 
            SolenoidIDs.INTAKE_OUT
        );

        intakeMotor = new TalonSRX(Constants.MotorIDs.INTAKE_MOTOR);
        isArmOut = armSolenoid.get() == Value.kReverse;
    }

    public void armIn(){
        armSolenoid.set(Value.kForward);
        isArmOut = false;
    }
  
    public void armOut(){
        armSolenoid.set(Value.kReverse);
        isArmOut = true;
    }

    public boolean isArmOut() {
        return isArmOut;
    }

    public void run(){
        intakeMotor.set(ControlMode.PercentOutput, Constants.Intake.INTAKE_SPEED_PCT);
        intakeRunning = true;
    }

    public void reverse(){
        intakeMotor.set(ControlMode.PercentOutput, -Constants.Intake.INTAKE_SPEED_PCT);
        intakeRunning = true;
    }
   
    public void stop(){
        intakeMotor.set(ControlMode.PercentOutput, 0);
        intakeRunning = false;
    }

    @Override
    public void periodic() { // TODO: ADV SCOPE? Shuffleboard?
        SmartDashboard.putBoolean("Intake Arm Out", isArmOut);
        SmartDashboard.putBoolean("Intake Running", intakeRunning);
    }
}
