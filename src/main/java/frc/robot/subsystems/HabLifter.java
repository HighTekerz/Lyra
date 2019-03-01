/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.tekerz.PIDF;
import frc.robot.tekerz.utilities.L;

/**
 * Add your docs here.
 */
public class HabLifter extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

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
    p = .01,
    i = 0.0,
    d = 0.001,
    f = 0.0;
  PIDF pid = new PIDF(p, i, d);

  public HabLifter() {
    TalonSRXConfiguration config = new TalonSRXConfiguration();
    habLifterRollingLead.configAllSettings(config);
    habLifterRollingFollower.configAllSettings(config);

    habLifterRollingFollower.follow(habLifterRollingLead);

    pid.setIMax(.5);

    habLifterArmsFollower.follow(habLifterArmsLead);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
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

  public void log() {
    
  }

  public void setArmSetpoint(double setpoint) {
    pid.setSetpoint(setpoint);
    pid.start();
  }

  public void executePID() {
    double speed = pid.loop(habLifterEnc.getPosition());
    L.ogSD("PID Sensor", habLifterEnc.getPosition());
    L.ogSD("PID ouput", speed);
    habLifterArmsLead.set(speed);
  }
}
