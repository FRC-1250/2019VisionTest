/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class Sub_Collector extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  Solenoid solenoidHatch = new Solenoid(RobotMap.COL_SOL_COL_0);
  Solenoid solenoidCollect = new Solenoid(RobotMap.COL_SOL_COL_1);
  Solenoid solenoidPincers = new Solenoid(RobotMap.COL_SOL_COL_3);
  DigitalInput sensorHatch = new DigitalInput(RobotMap.COL_SENSE_HATCH);
  DigitalInput sensorBall = new DigitalInput(RobotMap.COL_SENSE_BALL);
  WPI_VictorSPX dropMotor0 = new WPI_VictorSPX(RobotMap.COL_DROPMOTOR_0);
  WPI_VictorSPX dropMotor1 = new WPI_VictorSPX(RobotMap.COL_DROPMOTOR_1);
  WPI_VictorSPX armCollector0 = new WPI_VictorSPX(RobotMap.COL_ARM_0);
  WPI_VictorSPX armCollector1 = new WPI_VictorSPX(RobotMap.COL_ARM_1);

  // Motor groups for collector

  private SpeedController gDropMotors = new SpeedControllerGroup(dropMotor0, dropMotor1);

  private SpeedController gCollectorMotors = new SpeedControllerGroup(armCollector0, armCollector1);

  public Sub_Collector() {
    armCollector1.setInverted(true);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  // Methods for collector, collection speed, and throw speed
  public void collectorIntake() {
    gCollectorMotors.set(1);
  }

  public void collectorThrow() {
    gCollectorMotors.set(-1);
  }

  // Stop collector
  public void collectorStop() {
    gCollectorMotors.set(0);
  }

  // Half the speed of the cargo throw
  public void collectorThrowSlow() {
    gCollectorMotors.set(-0.5);
  }

  // Sets speed of collector
  public void collectorSetSpeed(double speed) {
    gCollectorMotors.set(speed);

  }

  // Extends hatch cylinders
  public void solenoidExtendHatch() {
    solenoidHatch.set(true);
  }

  // Retracts hatch cylinders
  public void solenoidRetractHatch() {
    solenoidHatch.set(false);
  }

  // Extends collector cylinders
  public void solenoidExtendCollector() {
    solenoidCollect.set(true);
  }

  // Retracts collector cylinders
  public void solenoidRetractCollector() {
    solenoidCollect.set(false);
  }

  // Extends pincer cylinders
  public void solenoidExtendPincers() {
    solenoidPincers.set(true);
  }

  // Retracts pincer cylinders
  public void solenoidRetractPincers() {
    solenoidPincers.set(false);
  }

  // Returns if hatch sensor is activated
  public boolean isBallSensorBackup() {
    return sensorHatch.get();
  }

  // Returns if ball sensor is activated
  public boolean isBallSensor() {
    return sensorBall.get();
  }

  // Sets speed of drop motors
  public void dropMotorSetSpeed(double speed) {
    gDropMotors.set(speed);
  }
}
