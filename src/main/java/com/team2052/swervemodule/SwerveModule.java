package com.team2052.swervemodule;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveModule {
    protected static final int CAN_TIMEOUT_MS = 250;

    protected final String debugName;
    private final TalonFX steerMotor;
    private final TalonFXConfiguration steerMotorConfig;
    private final TalonFXConfiguration driveMotorConfig;
    private final TalonFX driveMotor;

    private final double anglePosConversionFactor;
    private final double drivePosConversionFactor;
    private final double driveVeloConversionFactor;
    private final double maxVeloMPS;

    // rotary magnetic encoder over can-bus
    protected final CANcoder canCoder;
  
    public SwerveModule(
        String debugName,
        int driveChannel,
        int steerChannel,
        int canCoderchannel,
        Rotation2d angleOffsetRadians
    ){

    /*
    * CANCoder Initialization + Configuration 
    */
        MagnetSensorConfigs magsense = new MagnetSensorConfigs()
            .withMagnetOffset(-angleOffsetRadians.getDegrees())
            .withAbsoluteSensorDiscontinuityPoint(1.0); // [0, 360]
            
        // TODO: how to replace following line of code?
            // canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;

        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration().withMagnetSensor(magsense);

        canCoder = new CANcoder(canCoderchannel);
        checkStatusCodeError("Failed to configre CANCoder", 
            canCoder.getConfigurator().apply(canCoderConfig));
        checkStatusCodeError("Failed to set CANCoder status frame period", 
            canCoder.getAbsolutePosition().setUpdateFrequency(4.0)); // Hz = cycles per second: 4z = 250 ms 

    /*
    * Drive motor Initalization
    */
        driveMotor = new TalonFX(driveChannel);
        driveMotorConfig = new TalonFXConfiguration();
        driveMotor.setNeutralMode(NeutralModeValue.Brake);
        // inverted is clockwise positive, inverted false is counterclockwise positive
        // OLD VERSION" driveMotor.setInverted(SwerveConstants.SwerveModule.DRIVE_INVERTED);
        driveMotorConfig.MotorOutput.Inverted = 
            SwerveConstants.SwerveModule.DRIVE_INVERTED ?
                InvertedValue.Clockwise_Positive
                :InvertedValue.CounterClockwise_Positive;

        // set up conversion factor
        drivePosConversionFactor = // meters per encoder tick
            (Math.PI * SwerveConstants.SwerveModule.WHEEL_DIAMETER_METERS 
            / SwerveConstants.SwerveModule.TICKS_PER_ROTATION) * SwerveConstants.SwerveModule.DRIVE_REDUCTION;

        driveVeloConversionFactor = drivePosConversionFactor / 60.0; 
        // ticks per minute / 60 = ticks per second * (meters per tick) = meters per second

        // VoltageConfigs driveVoltageConfig = new VoltageConfigs();
        // driveVoltageConfig.withPeakForwardVoltage(SwerveConstants.MAX_VOLTAGE_VOLTS);
        // driveVoltageConfig.withPeakReverseVoltage(SwerveConstants.MAX_VOLTAGE_VOLTS);

       checkStatusCodeError(
            "Failed to set drive motor position signal update frequency", 
            driveMotor.getPosition().setUpdateFrequency(4.0));
        checkStatusCodeError(
            "Failed to set drive motor velocity signal update frequency", 
            driveMotor.getVelocity().setUpdateFrequency(4.0));
        checkStatusCodeError(
            "Failed to apply drive motor configs", 
            driveMotor.getConfigurator().apply(driveMotorConfig));


    /*
    * Steer motor Initalization
    */
        steerMotor = new TalonFX(driveChannel);
        steerMotorConfig = new TalonFXConfiguration();
        steerMotor.setNeutralMode(NeutralModeValue.Brake);

        // pid config
        TalonFXConfiguration steerMotorConfig = new TalonFXConfiguration();
        steerMotorConfig.Slot0.kP = SwerveConstants.SwerveModule.ANGLE_MOTOR_P;
        steerMotorConfig.Slot0.kI = SwerveConstants.SwerveModule.ANGLE_MOTOR_I;
        steerMotorConfig.Slot0.kD = SwerveConstants.SwerveModule.ANGLE_MOTOR_D;
    
        // set up conversion factors
        anglePosConversionFactor = 
            (2.0 * Math.PI / SwerveConstants.SwerveModule.TICKS_PER_ROTATION)
            * SwerveConstants.SwerveModule.ANGLE_REDUCTION; 

        // feedback sensor is integrated (TalonFX internal encoder)
        steerMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        checkStatusCodeError(
            "Failed to apply drive motor configs", 
            steerMotor.getConfigurator().apply(steerMotorConfig));

        // System.out.println("CANCoder Absolute Position: " + canCoder.getAbsolutePosition());
    
        // TODO: validate new method of setting encoder pos / status frame period
        // internal encoder pos matches CANCoder
        double absoluteAngleRad = Math.toRadians(canCoder.getAbsolutePosition().refresh().getValueAsDouble());
        double motorPosUnits = absoluteAngleRad / anglePosConversionFactor;
        checkStatusCodeError(
            "Failed to set steer motor encoderposition", 
            steerMotor.setPosition(motorPosUnits));

        checkStatusCodeError(
            "Failed to set steer motor position signal update frequency", 
            steerMotor.getPosition().setUpdateFrequency(4.0));
        checkStatusCodeError(
            "Failed to set steer motor velocity signal update frequency", 
            steerMotor.getVelocity().setUpdateFrequency(4.0));

        maxVeloMPS = SwerveConstants.SwerveModule.FALCON500_ROUNDS_PER_MINUTE/ 60
            * SwerveConstants.SwerveModule.WHEEL_DIAMETER_METERS * Math.PI
            * SwerveConstants.SwerveModule.DRIVE_REDUCTION;

        this.debugName = debugName;
    }

    public void setDriveMotorNeutralModeBrake(){
        driveMotor.setNeutralMode(NeutralModeValue.Brake);
        // System.out.println("Drive motor set to BREAK.");
    }

    public void setDriveMotorNeutralModeCoast(){
        driveMotor.setNeutralMode(NeutralModeValue.Coast);
        // System.out.println("Drive motor set to COAST.");
    }

    // TODO: validate need of refresh() method or not
    public SwerveModuleState getState(){
        return new SwerveModuleState(
            // ticks to MPS using predefined conversion rate
            driveMotor.getVelocity().getValueAsDouble() * driveVeloConversionFactor,
            new Rotation2d(
                // ticks to Radians using predefined conversion rate
                steerMotor.getPosition().getValueAsDouble() * anglePosConversionFactor
            )
        );
    }

    public void setState(double veloMPS, Rotation2d steerAngle){
        SwerveModuleState desiredState = new SwerveModuleState(veloMPS, steerAngle);
        // change angle val to 0-2pi, simplify to nearest angle
        desiredState.optimize(steerAngle);
        // set to desired velo as percentage of max
        double driveOutput = desiredState.speedMetersPerSecond / maxVeloMPS;
        driveMotor.setControl(new DutyCycleOut(driveOutput));
        // angle to rotations for pos control
        double targetAngleRotations = desiredState.angle.getRadians() / anglePosConversionFactor;
        steerMotor.setControl(new PositionDutyCycle(targetAngleRotations));
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            driveMotor.getPosition().getValueAsDouble() * drivePosConversionFactor,
            new Rotation2d(
                // ticks to Radians using predefined conversion rate
                steerMotor.getPosition().getValueAsDouble() * anglePosConversionFactor
            )
        );
    }

    public CANcoder getCANCoder(){
        return canCoder;
    }

    /*
     * Math-y methods from 2025 Swerve rewrite
     */
    public static double getMaxVelocityMetersPerSecond() {
        /*
         * Theoretical max velocity (highest speed in a straight line) = 
         * [Motor free speed (RPM)] / 60 * [Drive reduction] * [Wheel diameter (m)] * pi
         */
        return SwerveConstants.SwerveModule.FALCON500_ROUNDS_PER_MINUTE / 60 * SwerveConstants.SwerveModule.DRIVE_REDUCTION * 
            SwerveConstants.SwerveModule.WHEEL_DIAMETER_METERS * Math.PI;
    }

    public static double getMaxAngularVelocityRadiansPerSecond() {
        /*
         * Theoretical max angular velocity (in-place) in Radians / second
         */
        return getMaxVelocityMetersPerSecond() / Math.hypot(
            Constants.Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
            Constants.Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0
        );
    }

    public void debug(String debugName) { // TODO: ADV SCOPE? Shuffleboard?
        SmartDashboard.putNumber(debugName + " Offset Degrees", canCoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber(debugName + " Degrees", Math.toDegrees(steerMotor.getPosition().getValueAsDouble() * anglePosConversionFactor));
    }

    /*
     * Error checks from 2025Scarab
     */
    @SuppressWarnings("unchecked")
    protected <E> void checkStatusCodeError(String message, StatusCode error) {
      if (error != StatusCode.OK) {
        DriverStation.reportError(
            message + " on [front right] module: " + error.toString(),
            true
        );
      }
    }

    /*
     * From 2025 Scarab rewrite
     */
    public static class SwerveModuleConstants {
		public final int driveMotorID;
		public final int steerMotorID;
		public final double steerOffset;

		public SwerveModuleConstants(int driveMotorID, int steerMotorID, double steerOffset) {
			this.driveMotorID = driveMotorID;
			this.steerMotorID = steerMotorID;
			this.steerOffset = steerOffset;
		}
	}
        
}
