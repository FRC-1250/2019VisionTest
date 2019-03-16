/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.Cmd_ManualDrive;

import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;


public class Sub_DriveTrain extends Subsystem {
  // Motor Controllers
	WPI_TalonSRX fLeftMotor = new WPI_TalonSRX(RobotMap.DRV_LEFT_FRONT);
	WPI_TalonSRX bLeftMotor = new WPI_TalonSRX(RobotMap.DRV_LEFT_BACK);
	WPI_TalonSRX mLeftMotor = new WPI_TalonSRX(RobotMap.DRV_LEFT_MID);
	WPI_TalonSRX bRightMotor = new WPI_TalonSRX(RobotMap.DRV_RIGHT_BACK);
	WPI_TalonSRX fRightMotor = new WPI_TalonSRX(RobotMap.DRV_RIGHT_FRONT);
	WPI_TalonSRX mRightMotor = new WPI_TalonSRX(RobotMap.DRV_RIGHT_MID);

	// Motor Control groups
	private SpeedController gLeftMotor = new SpeedControllerGroup(fLeftMotor, bLeftMotor, mLeftMotor);
	private SpeedController gRightMotor = new SpeedControllerGroup(fRightMotor, bRightMotor, mRightMotor);
	private DifferentialDrive diffDriveGroup = new DifferentialDrive(gLeftMotor, gRightMotor);
	public static double accumError = 0;
	private final double AUTO_TURN_RATE = 0.3;
	private final double KP_SIMPLE_STRAIT = 0.01;
	private final double KP_SIMPLE = 0.05;
	private final double KI_SIMPLE = 0.03;

	AnalogGyro gyro = new AnalogGyro(RobotMap.GYRO);
	
	public int driveSetpoint = 10;
	private final double DRIVE_TICKS = 365.8;
	
	private double CUBE_AREA_SETPOINT = 12;
	
	private static double areaDiff;
	
	public Sub_DriveTrain() {
		fLeftMotor.setInverted(false);
		fRightMotor.setInverted(false);
		fLeftMotor.setNeutralMode(NeutralMode.Brake);
		mLeftMotor.setNeutralMode(NeutralMode.Brake);
		bLeftMotor.setNeutralMode(NeutralMode.Brake);
		fRightMotor.setNeutralMode(NeutralMode.Brake);
		mRightMotor.setNeutralMode(NeutralMode.Brake);
		bRightMotor.setNeutralMode(NeutralMode.Brake);
		fLeftMotor.configOpenloopRamp(0.1, 10);
		mLeftMotor.configOpenloopRamp(0.1, 10);
		bLeftMotor.configOpenloopRamp(0.1, 10);
		fRightMotor.configOpenloopRamp(0.1, 10);
		mRightMotor.configOpenloopRamp(0.1, 10);
		bRightMotor.configOpenloopRamp(0.1, 10);
		
		bRightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		fLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		
	}

	public void initDefaultCommand() {
		setDefaultCommand(new Cmd_ManualDrive());
	}

	public void drive(double left, double right) {
		diffDriveGroup.tankDrive(left, right);
	}

	public void drive(Joystick joy) {
		drive(-joy.getY(), -joy.getThrottle());
	}
	
	public void driveStraight() {
		double diff = 0 - gyro.getAngle();
		double corrected = .05 * diff;
		diffDriveGroup.arcadeDrive(0.8, 0 + corrected);
		
	}
	public void driveArcade(Joystick joy){
		diffDriveGroup.arcadeDrive(-joy.getY(), -joy.getX());
	}

	
	 public void setpointCube(double areaSetpoint) {
	    	CUBE_AREA_SETPOINT = areaSetpoint;
	    }
	
	
	public void trackCube(double xOffset, double aJoy) {
		
		double xDiff = xOffset;
		// areaDiff = CUBE_AREA_SETPOINT - areaOffset;
		double xCorrect = xDiff;
		// double aCorrect = 0.35 * areaDiff;
		SmartDashboard.putNumber("steering_adjust", xOffset);
		SmartDashboard.putNumber("joystickY", aJoy);

		diffDriveGroup.arcadeDrive(aJoy, xCorrect);
		
		SmartDashboard.putNumber("Area Diff", areaDiff);
	}
	
	public boolean isAtCubeSP(double cubeArea) {
		return (cubeArea > CUBE_AREA_SETPOINT);
	}
	
	public void driveStop() {
		diffDriveGroup.arcadeDrive(0, 0);
	}
    
    public void pause() {
    	drive(0,0);
    }
    
    public double getGyroAngle() {
    	return gyro.getAngle();
    }
    
    
    public void resetGyro() {
    	gyro.reset();
    }
    
    public int getLeftTicks() {
    	return bRightMotor.getSelectedSensorPosition(0);
    }
    
    
    public void setSetpointPos(int distance) {
    	driveSetpoint = (int)DRIVE_TICKS * distance;
    }
    
    public boolean isDoneDriving() {
    	
		int currVal = this.getLeftTicks() * 1;
		int distToPos = currVal - driveSetpoint;
		SmartDashboard.putNumber("DistToPos", distToPos);
    	return (distToPos >= 0);
	}
	
	public boolean isDoneDrivingBack() {
    	
		int currVal = this.getLeftTicks() * 1;
		int distToPos = currVal - driveSetpoint;
		SmartDashboard.putNumber("DistToPos", distToPos);
    	return (distToPos <= 0);
    }
 
 public void driveToPos( double upperSpeed, double lowerSpeed) {
    	
		double diff = 0 - gyro.getAngle();
		double corrected = .05 * diff;

    	double sign = Math.signum(driveSetpoint);
    	
    	diffDriveGroup.arcadeDrive(linearRamp(upperSpeed,lowerSpeed) * sign, 0 + corrected);
    	
    }
    
    private double linearRamp( double upperSpeed, double lowerSpeed) {
    	double diff = (driveSetpoint - (double)Math.abs(getLeftTicks()));
    	double corrected = .05 * diff;
    	double upperBound = Math.min(upperSpeed , corrected);
    	double lowerBound = Math.max(lowerSpeed , upperBound);
    	
    	SmartDashboard.putNumber("correctedoutput", corrected);
    	return lowerBound;
    	
    }
    
    public void turn (double angle, double upperSpeed, double lowerSpeed) {
//    	double rotation = AUTO_TURN_RATE * getGainPI(angle,this.getGyroAngle(),KP_SIMPLE, KI_SIMPLE);
    	//rotation  = rotation * (int)Math.signum(angle);
    	double corrected;
		//double upperBound; 
		// double lowerBound;
    	
		double rotation = angle - getGyroAngle();
//    	int sign = (int)Math.signum(getGyroAngle());
    	double sign = Math.signum(rotation);
    	
		corrected = 0.05 * rotation;
    	
    	if (sign > 0){
    		 corrected = Math.min(upperSpeed * sign, corrected);
    		 corrected = Math.max(lowerSpeed * sign, corrected);
    	}
    		
    	else {
    		 corrected = Math.max(upperSpeed * sign, corrected);
			 corrected = Math.min(lowerSpeed * sign, corrected);    	    		
    	}

    	diffDriveGroup.arcadeDrive(0, corrected);
    }
  private double getGainPI(double setpoint, double current,double kP, double kI) {
    	
    	double error = setpoint - current; 
    	double p = KP_SIMPLE * error;
    	accumError = accumError + error;
    	double i = KI_SIMPLE * error;
    	return p + i;
    }
  public boolean isDoneTurning(double angle) {
  	return (Math.abs(angle - this.getGyroAngle()) < 2);
  }
  public void resetSensorPos() {
  	fRightMotor.setSelectedSensorPosition(0, 0, 10);
  	fLeftMotor.setSelectedSensorPosition(0, 0, 10);
  }
    
}

