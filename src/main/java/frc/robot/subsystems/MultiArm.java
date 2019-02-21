/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

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
    hPStabilizer = RobotMap.Pneumatics.hPStabilizer;

  TalonSRX
    intake = RobotMap.Talons.intake;

  DigitalInput
    cargoSensor = RobotMap.Switches.cargoSensor;

  public MultiArm() {
    TalonSRXConfiguration config = new TalonSRXConfiguration();
    intake.configAllSettings(config);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void log() {
    
  }
}
