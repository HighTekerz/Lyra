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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.multiarm.HatchFingerRelease;

/**
 * Add your docs here.
 */
public class MultiArm extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  Solenoid
    intakeFlap = RobotMap.Pneumatics.intakeFlap,
    hPFlap = RobotMap.Pneumatics.hPFlap,
    hPPusher = RobotMap.Pneumatics.hPPusher,
    hPFinger = RobotMap.Pneumatics.hPFinger;

  TalonSRX
    intake = RobotMap.Talons.intake;

  DigitalInput
    hPSensor0 = RobotMap.Switches.hPSensor0,
    hPSensor1 = RobotMap.Switches.hPSensor1,
    cargoSensor = RobotMap.Switches.cargoSensor;

  public MultiArm() {
    TalonSRXConfiguration config = new TalonSRXConfiguration();
    intake.configAllSettings(config);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new HatchFingerRelease());
  }

  public boolean hasCargo() {
    return cargoSensor.get();
  }

  public boolean hasHP() {
    return (this.hPSensor0.get() || this.hPSensor1.get());
  }

  public void runIntake(double speed) {
    this.intake.set(ControlMode.PercentOutput, speed);
  }

  public void intakeFlapUp() {
    this.intakeFlap.set(true);
  }

  public void intakeFlap() {
    this.intakeFlap.set(false);
  }

  public void hPFlapUp() {
    this.hPFlap.set(true);
  }

  public void hPFlapDown() {
    this.hPFlap.set(false);
  }

  public void hPPusherOut() {
    this.hPPusher.set(true);
  }

  public void hPPusherIn() {
    this.hPPusher.set(false);
  }

  public void setIntakeSpeed(double speed){
    intake.set(ControlMode.PercentOutput, speed);
  }

  public void setFingerUp(){
    hPFinger.set(true);
  }

  public void setFingerDown(){
    hPFinger.set(false);
  }

  /**
   * @return is hp flap up.
   */
  public boolean getHPFlapIsUp(){
    return hPFlap.get();
  }

  public void log() {
    
  }
}
