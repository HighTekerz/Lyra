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

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.tekerz.PIDF;
import frc.robot.tekerz.utilities.L;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public static double ticksPerInch = 360;

  TalonSRX 
    liftLead = RobotMap.Talons.liftLead,
    liftFollower = RobotMap.Talons.liftFollower;

  double 
    p = .01,
    i = 0.0,
    d = 0.001,
    f = 0.0;
  PIDF pid = new PIDF(p, i, d);

  public Elevator() {
    TalonSRXConfiguration config = new TalonSRXConfiguration();
    liftLead.configAllSettings(config);
    liftFollower.configAllSettings(config);

    liftFollower.follow(liftLead);

    pid.setIMax(.5);
  }

  @Override
  public void initDefaultCommand() {
  }

  public double getEncoder() {
    return liftLead.getSelectedSensorPosition();
  }

  public void setSetpoint(double setpoint) {
    pid.setSetpoint(setpoint);
    pid.start();
  }

  public void executePID() {
    double speed = pid.loop(this.getEncoder());
    L.ogSD("PID Sensor", this.getEncoder());
    L.ogSD("PID ouput", speed);
    liftLead.set(ControlMode.PercentOutput, speed);
  }

  public void log() {
    
  }
}
