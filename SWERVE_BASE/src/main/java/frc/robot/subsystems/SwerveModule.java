package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveTrainConstants;

public class SwerveModule {

    // motors
    private final TalonFX driveMotor;
    private final TalonSRX turnMotor;

    // pid for motors
    private final PIDController turningPIDController;
    private final PIDController speedPIDController;

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

        // config turn motor (v5)
        this.turnMotor.configFactoryDefault();
        this.turnMotor.setInverted(turnMotorReversed);
        this.turnMotor.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition, 0, 10);

        // pid controller
        turningPIDController = new PIDController(DriveTrainConstants.kPTurning, DriveTrainConstants.kITurning, DriveTrainConstants.kDTurning);
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
        speedPIDController = new PIDController(DriveTrainConstants.kPDrive, DriveTrainConstants.kIDrive, DriveTrainConstants.kDDrive);
    }

    // getters
    public double getTurningPositionRadians() {
        double encoder_reading = (turnMotor.getSelectedSensorPosition() % DriveTrainConstants.kEncoderResolution) - absoluteEncoderOffset;
        if (absoluteEncoderReversed){
            encoder_reading = -encoder_reading;
        }
        while (encoder_reading < 0){
            encoder_reading += DriveTrainConstants.kEncoderResolution;
        }
        return (encoder_reading * 2 * Math.PI /DriveTrainConstants.kEncoderResolution - Math.PI);
    }

    public double getDrivePosition() {
        return driveMotor.getPosition().getValueAsDouble() * DriveTrainConstants.kDriveEncoderToMeters;
    }

    public double getDriveVelocity() {
        return driveMotor.getVelocity().getValueAsDouble() * DriveTrainConstants.kDriveEncoderToMeters;
    }

    public double getTurnVelocity() {
        return turnMotor.getSelectedSensorVelocity() * DriveTrainConstants.kTurnEncoderToRad;
    }

    // stop

    public void stop() {
        driveMotor.set(0);
        turnMotor.set(ControlMode.Velocity,0);
    }

    // swervemodulestate format
    public SwerveModulePosition getPosition() {
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

        double calculatedDriveSpeed = speedPIDController.calculate(getDriveVelocity(), state.speedMetersPerSecond);
        double calculatedTurnSpeed = turningPIDController.calculate(getTurningPositionRadians(), state.angle.getRadians());
        

        SmartDashboard.putNumber("Swerve[" + driveMotor.getDeviceID() + "] calculated drive speed:", calculatedDriveSpeed);
        SmartDashboard.putNumber("Swerve[" + driveMotor.getDeviceID() + "] calculated turn speed:", calculatedTurnSpeed);


        driveMotor.set(calculatedDriveSpeed);
        turnMotor.set(TalonSRXControlMode.Velocity, calculatedTurnSpeed);
        
        SmartDashboard.putString("Swerve[" + driveMotor.getDeviceID() + "] desired state:", "Speed: " + Double.toString(state.speedMetersPerSecond) + ", Angle: " + Double.toString(state.angle.getDegrees()));
    }

    public void showDebugInfo(){
        SmartDashboard.putString("Swerve[" + driveMotor.getDeviceID() + "] sensor outputs:",  "Speed: " + Double.toString(driveMotor.getPosition().getValueAsDouble()) + ", Angle: " + Double.toString(turnMotor.getSelectedSensorPosition()));
        SmartDashboard.putString("Swerve[" + driveMotor.getDeviceID() + "] actual state:",  "Speed: " + Double.toString(getState().speedMetersPerSecond) + ", Angle: " + Double.toString(getState().angle.getDegrees()));
    }
}
