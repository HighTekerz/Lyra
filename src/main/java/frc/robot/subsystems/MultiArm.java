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
import frc.robot.commands.multiarm.FingerUpWhileSensorTripped;
import frc.robot.commands.multiarm.HatchFingerHold;
import frc.robot.commands.multiarm.HatchFingerRelease;
import frc.robot.tekerz.utilities.L;

/**
 * Add your docs here.
 */
public class MultiArm extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public enum MODE {
    CARGO,
    HATCH_PANEL,
    DEFENSE
  }

  private MODE mode = MODE.DEFENSE;

  Solenoid
    cargoArm = RobotMap.Pneumatics.intakeFlap,
    hPFlap = RobotMap.Pneumatics.hPFlap,
    hPPusher = RobotMap.Pneumatics.hPPusher,
    hPFinger = RobotMap.Pneumatics.hPFinger;

  TalonSRX
    cargoIntake = RobotMap.Talons.intake;

  DigitalInput
    hPSensor0 = RobotMap.Switches.hPSensor0,
    hPSensor1 = RobotMap.Switches.hPSensor1,
    cargoSensor = RobotMap.Switches.cargoSensor;

  public MultiArm() {
    TalonSRXConfiguration config = new TalonSRXConfiguration();
    cargoIntake.configAllSettings(config);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new HatchFingerHold());
  }

  public void setMode(MODE mode){
    this.mode = mode;
  }

  public MODE getMode() {
    return this.mode;
  }

  public boolean hasCargo() {
    return cargoSensor.get();
  }

  public boolean hasHP() {
    // We want these to return true when we have a hatch panel or when they come unplugged
    return (!this.hPSensor0.get() && !this.hPSensor1.get());
  }

  public void runIntake(double speed) {
    this.cargoIntake.set(ControlMode.PercentOutput, speed);
  }

  public void cargoArmUp() {
    this.cargoArm.set(true);
  }

  public void cargoArmDown() {
    this.cargoArm.set(false);
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

  public void setCargoIntakeSpeed(double speed){
    cargoIntake.set(ControlMode.PercentOutput, speed);
  }

  public void setFingerUp(){
    hPFinger.set(false);
  }

  public void setFingerDown(){
    hPFinger.set(true);
  }

  /**
   * @return is hp flap up.
   */
  public boolean isHPFlapUp(){
    return !hPFlap.get();
  }

  public void log() {
    L.ogSD("Hatch flap in system", hasHP());
    L.ogSD("Hatch flap is up", isHPFlapUp());
    L.ogSD("HP Sensor 0", hPSensor0.get());
    L.ogSD("HP Sensor 1", hPSensor1.get());
  }
}
