// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// we want the robot to recognize targets and turn towards them 
// change angle the arm is raised to based on how far it is
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class LimeLight extends SubsystemBase {
  /** Creates a new LimeLight. */
  private static LimeLight instance = null; //ensures only one instance of this class is created and interacts with the NetworkTable
	private static NetworkTable table; //stores a reference to the NetworkTable associated with the Limelight
	private static NetworkTableEntry tx;
	private static NetworkTableEntry ty;
	private static NetworkTableEntry tv;
	private static NetworkTableEntry ta;
	private static NetworkTableEntry camMode;
	private static NetworkTableEntry ledMode;

  private enum LEDMode {

		PIPELINE(0),
		OFF(1),
		BLINK(2),
		ON(3);

		private int modeValue;
		private LEDMode(int modeVal) {

			this.modeValue = modeVal;
		}
	}
		
	private enum CamMode {
	
		VISION(0),
		DRIVER(1);

		private int modeValue;
		private CamMode(int modeVal) {
			this.modeValue = modeVal;
		}
	}

  public LimeLight getInstance() {
    if (instance == null)
      instance = new LimeLight();
    return instance;

  }

  public LimeLight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    tv = table.getEntry("tv");
    ta = table.getEntry("ta");
    ledMode = table.getEntry("ledMode");

  }

  public double getTargetOffsetX() {
    return tx.getDouble(0.0);
  }

  public double getTargetOffsetY() {
    return ty.getDouble(0.0);
  }

  public boolean isTargetAvailable() {
    return tv.getNumber(0).intValue() == 1 ? true : false;
  }

  public double getTargetArea() {
    return ta.getDouble(0.0);
  }

  private void setLEDMode(LEDMode mode) {
    ledMode.setNumber(mode.modeValue);

  }

  public double getDistance(){
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry ty = table.getEntry("ty");
    double targetOffsetAngle_Vertical = ty.getDouble(0.0);


    double angleToGoalDegrees = LimelightConstants.limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    //calculate distance
    return (LimelightConstants.goalHeightInches - LimelightConstants.limelightLensHeightInches) / Math.tan(angleToGoalRadians);
  }

  public void turnOffLED() {

    this.setLEDMode(LEDMode.OFF);
  }

  public void turnOnLED() {
    this.setLEDMode(LEDMode.ON);
  }

  public void blinkLED() {
    this.setLEDMode(LEDMode.BLINK);
  }

  private void setCamMode(CamMode mode) {
    camMode.setNumber(mode.modeValue);
  }

  public void setModeDriver() {

		this.setLEDMode(LEDMode.OFF);
		this.setCamMode(CamMode.DRIVER);
	}

	public void setModeVision() {
	
		this.setLEDMode(LEDMode.ON);
		this.setCamMode(CamMode.VISION);
	}

  private boolean isModeDriver() {

		return ledMode.getDouble(0.0) == LEDMode.OFF.modeValue && camMode.getDouble(0.0) == CamMode.DRIVER.modeValue;
	}
	
	private boolean isModeVision() {
	
		return ledMode.getDouble(0.0) == LEDMode.ON.modeValue && camMode.getDouble(0.0) == CamMode.VISION.modeValue;
	}

  public double aim_proportional()
  {    
    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = getTargetOffsetX() * Math.PI/180;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  public double range_proportional()
  {    
    double targetingForwardSpeed = getTargetOffsetY();
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }


	//now we will use them to create a method that toggles between modes
	
	public void toggleMode() {
		if (this.isModeDriver()) {
			this.setModeVision();
		}
		else if (this.isModeVision()) {
			this.setModeDriver();
		}
		else {
			this.blinkLED();
		}
	}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tv = table.getEntry("tv");
    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
  }
}
