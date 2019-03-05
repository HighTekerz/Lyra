/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.tekerz.utilities.L;

/**
 * The hablifter implements the 
 */
public class HabLifter extends Subsystem {
  // we had 32 rotations per 90 degrees
  public static final double 
    FULL_HOLD_POWER = 0.2,
    ROTATIONS_PER_DEGREE = 32.0 / 90.0,
    MAX_SENSOR_READING = 10.0,
    MIN_SETSOR_READING = -48.0,
    FULL_SENSOR_RANGE = MAX_SENSOR_READING - MIN_SETSOR_READING,
    START_DEGREES_OFF_TDC = 0.0;

  TalonSRX
    habLifterRollingLead = RobotMap.Talons.habLifterWheelLead,
    habLifterRollingFollower = RobotMap.Talons.habLifterWheelFollower;

  CANSparkMax
    habLifterArmsLead = RobotMap.Sparks.habLifterArmsLead,
    habLifterArmsFollower = RobotMap.Sparks.habLifterArmsFollower;

  CANEncoder habLifterEnc = habLifterArmsLead.getEncoder();
  
  Solenoid
    habLifterLegs1 = RobotMap.Pneumatics.habLifterLegs1,
    habLifterLegs2 = RobotMap.Pneumatics.habLifterLegs2;

  double 
    p = 2.0 / FULL_SENSOR_RANGE,
    i = 0.0,
    d = 0.0,
    loopLengthInSeconds = .005;

  private final PIDOutput output = this::setArmPIDOutput;
  private final PIDSource input = new PIDSource() {
    
    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
    }
  
    @Override
    public double pidGet() {
      return getArmPosition();
    }
  
    @Override
    public PIDSourceType getPIDSourceType() {
      return PIDSourceType.kDisplacement;
    }
  };
  
  private final PIDController pIDLoop = new PIDController(p,i,d, input, output, loopLengthInSeconds) {
    @Override
    protected double calculateFeedForward() {
      // MULTIPLY
      double amountOfGravity = Math.sin(Math.toRadians(getArmPosition() / -ROTATIONS_PER_DEGREE));
      double fF = amountOfGravity * FULL_HOLD_POWER;
      L.ogSD("PID ARM FF", fF);
      return fF / 10.0;
    }
  };
    
  public HabLifter() {
    TalonSRXConfiguration config = new TalonSRXConfiguration();
    habLifterRollingLead.configAllSettings(config);
    habLifterRollingFollower.configAllSettings(config);
    
    habLifterRollingFollower.setInverted(true);
    habLifterRollingFollower.follow(habLifterRollingLead);;

    habLifterArmsLead.restoreFactoryDefaults();
    habLifterArmsFollower.restoreFactoryDefaults();

    habLifterArmsLead.setInverted(false);
    habLifterArmsFollower.follow(habLifterArmsLead, true);
  }

  @Override
  public void initDefaultCommand() {
  }

  public void driveWheels(double speed) {
    this.habLifterRollingLead.set(ControlMode.PercentOutput, speed);
  }

  public void feetOut() {
    this.habLifterLegs1.set(true);
    this.habLifterLegs2.set(true);
  }

  public void feetIn() {
    this.habLifterLegs1.set(false);
    this.habLifterLegs2.set(false);
  }

  private void setArmPIDOutput (double out) {
    habLifterArmsLead.set(out);
    L.ogSD("PID ARM ouput", out);
  }

  public double getArmPosition () {
    return habLifterEnc.getPosition();
  }
/**
 * 
 * @param setpointInDegrees the angle (in degrees) you want the arm to travel to.
 */
  public void setArmSetpoint(double setpointInDegrees) {
    double setpoint = setpointInDegrees * ROTATIONS_PER_DEGREE;
    this.pIDLoop.setSetpoint(setpoint);
  }

  public void enableArm() {
    this.pIDLoop.enable();
  }

  public void disableArm() {
    this.pIDLoop.disable();
  }

  public void clearEncoder() {
    habLifterEnc.setPosition(0.0);
  }

  public void log() {
    L.ogSD("PID ARM Sensor Degrees", getArmPosition() / HabLifter.ROTATIONS_PER_DEGREE);
  }
}
