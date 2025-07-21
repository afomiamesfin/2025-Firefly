package com.team2052.swervemodule;

import com.ctre.phoenix6.StatusCode;

// shared elements - not dependent on motor type

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

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
    private final double maxVeloConversionFactor;

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

        // set up conversion factos
        drivePosConversionFactor = 
            (Math.PI * SwerveConstants.SwerveModule.WHEEL_DIAMETER_METERS 
            / SwerveConstants.SwerveModule.TICKS_PER_ROTATION) * SwerveConstants.SwerveModule.DRIVE_REDUCTION;

        driveVeloConversionFactor = drivePosConversionFactor / 60.0;

        // VoltageConfigs driveVoltageConfig = new VoltageConfigs();
        // driveVoltageConfig.withPeakForwardVoltage(SwerveConstants.MAX_VOLTAGE_VOLTS);
        // driveVoltageConfig.withPeakReverseVoltage(SwerveConstants.MAX_VOLTAGE_VOLTS);

        StatusCode status1 = driveMotor.getPosition().setUpdateFrequency(4.0);
        StatusCode status2 = driveMotor.getVelocity().setUpdateFrequency(4.0);
        checkStatusCodeError("Failed to set drive motor position signal update frequency", status1);
        checkStatusCodeError("Failed to set drive motor velocity signal update frequency", status2);

    /*
    * Steer motor Initalization
    */
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
