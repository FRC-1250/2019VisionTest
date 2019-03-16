/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	public static final int DRV_LEFT_FRONT = 10;
	public static final int DRV_LEFT_MID = 11;
	public static final int DRV_LEFT_BACK = 12;
	public static final int DRV_RIGHT_FRONT = 23;
	public static final int DRV_RIGHT_MID = 24;
	public static final int DRV_RIGHT_BACK = 25;

	public static final int GYRO = 0;

	// Solenoid PCM Channel
	public static final int COL_SOL_COL_0 = 0;
	public static final int COL_SOL_COL_1 = 1;
	// Collector Sensor IDs
	public static final int COL_SENSE_HATCH = 1;
	public static final int COL_SENSE_BALL = 0;
	// Drop Motor Victors IDs
	public static final int COL_DROPMOTOR_0 = 18;
	public static final int COL_DROPMOTOR_1 = 19;
	// Arm Collector ID

	public static final int COL_ARM_0 = 20;
	public static final int COL_ARM_1 = 21;
	public static final int COL_SOL_COL_3 = 3;
}
