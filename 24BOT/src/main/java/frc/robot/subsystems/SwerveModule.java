package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveTrainConstants;

public class SwerveModule {

    // motors
    private final TalonFX driveMotor;
    private final TalonSRX turnMotor;
    private final boolean turnInversed;

    // pid for motors
    private final PIDController turningPIDController;

    // offset if needed
    private final double absoluteEncoderOffset;
    private final boolean absoluteEncoderReversed;


    public SwerveModule(int driveMotorId, int turnMotorId, boolean driveMotorReversed, boolean turnMotorReversed, double absoluteEncoderOffset, boolean absoluteEncoderReversed){
        
        // offsets if needed
        this.absoluteEncoderOffset = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        
        // initialize motors with provided ids
        this.driveMotor = new TalonFX(driveMotorId);
        this.turnMotor = new TalonSRX(turnMotorId);

        // config drive motor
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        driveConfig.MotorOutput.Inverted = driveMotorReversed ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        this.driveMotor.getConfigurator().apply(driveConfig);

        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = 0.05; // Add 0.05 V output to overcome static friction
        slot0Configs.kV = 0.13; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = 0.10; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative

        driveMotor.getConfigurator().apply(slot0Configs);

        // config turn motor (v5)
        this.turnMotor.configFactoryDefault();
        this.turnMotor.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition, 0, 10);
        turnInversed = turnMotorReversed;

        // pid controller
        turningPIDController = new PIDController(DriveTrainConstants.kPTurning, DriveTrainConstants.kITurning, DriveTrainConstants.kDTurning);
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    // getters
    public double getTurningPositionRadians() {
        double position = turnMotor.getSelectedSensorPosition() - absoluteEncoderOffset;
        if (absoluteEncoderReversed){
            position = -position;
        }
        while (position < 0){
            position += DriveTrainConstants.kTurnEncoderResolution;
        }
        position %= DriveTrainConstants.kTurnEncoderResolution;
        return (position - DriveTrainConstants.kTurnEncoderResolution/2)/DriveTrainConstants.kTurnEncoderResolution * 2 * Math.PI;
    }

    public double getDrivePosition() {
        return driveMotor.getPosition().getValueAsDouble() * DriveTrainConstants.kDriveEncoderToMeters;
    }

    public double getDriveVelocity() {
        return driveMotor.getVelocity().getValueAsDouble() * DriveTrainConstants.kDriveEncoderToMeters;
    }

    // stop

    public void stop() {
        driveMotor.set(0);
        turnMotor.set(ControlMode.PercentOutput,0);
    }

    // swervemodulestate format
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPositionRadians()));
    }

    public SwerveModulePosition getPositionOdometry() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPositionRadians()));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(),  new Rotation2d(getTurningPositionRadians()));
    }

    public void setDesiredState(SwerveModuleState state){
        if (Math.abs(state.speedMetersPerSecond) < 0.01) {
            stop();
            return;
        }
        
        state = SwerveModuleState.optimize(state, getState().angle);

        //SmartDashboard.putNumber("Desired Velocity[" +  driveMotor.getDeviceID() + "]", state.speedMetersPerSecond);
        //SmartDashboard.putNumber("Actual Velocity [" +  driveMotor.getDeviceID() + "]", getDriveVelocity());

        // create a velocity closed-loop request, voltage output, slot 0 configs
        final VelocityVoltage m_request = new VelocityVoltage(state.speedMetersPerSecond / DriveTrainConstants.kDriveEncoderToMeters).withSlot(0);

        // set velocity
        driveMotor.setControl(m_request);
        double calculatedTurnSpeed = turningPIDController.calculate(getTurningPositionRadians(), state.angle.getRadians());
        
        // if (driveMotor.getDeviceID() == 1){
        //     SmartDashboard.putNumber("Desired Wheel Angle [" +  driveMotor.getDeviceID() + "]", state.angle.getRadians());
        //     SmartDashboard.putNumber("Desired Wheel Speed [" +  driveMotor.getDeviceID() + "]", state.speedMetersPerSecond);
        // }
        //SmartDashboard.putNumber("Turn Motor Setting [" +  driveMotor.getDeviceID() + "]", calculatedTurnSpeed);

        if (turnInversed){
            calculatedTurnSpeed *= -1;
        }
        turnMotor.set(TalonSRXControlMode.PercentOutput, calculatedTurnSpeed);
        
    }

    public void showDebugInfo(){
        
        // if (driveMotor.getDeviceID() == 1){
        //     SmartDashboard.putNumber("Actual Wheel Angle [" +  driveMotor.getDeviceID() + "]", getTurningPositionRadians());
        //     SmartDashboard.putNumber("Actual Wheel Speed [" +  driveMotor.getDeviceID() + "]", getDriveVelocity());
        // }

    }
}
