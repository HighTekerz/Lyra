/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.tekerz.L;

public class TestTalonMotors extends Command {
  TalonSRX talon = RobotMap.Talons.testTalonEncoder;
  public TestTalonMotors() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.Subsystems.sparkSub);
    requires(Robot.Subsystems.talonSub);
    requires(Robot.Subsystems.talonEncoderSub);
  }

  public TestTalonMotors(TalonSRX talon, String name) {
    this();
    this.setName(name);
    this.talon = talon;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    L.ogCmdInit(this);
    // these are to create the default and prevent null reference in the execute methods.
    // currentMotor = RobotMap.Talons.testTalonIMU;
    // lastMotor = RobotMap.Talons.testTalonIMU;

    // motorChooser.setDefaultOption("testTalon", RobotMap.Talons.testTalonIMU);
    // motorChooser.addOption("TestTalonEncoder", RobotMap.Talons.testTalonEncoder);

    // SmartDashboard.putData("Test Motor: B is slow Y is fast", motorChooser);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // if (motorChooser.getSelected() != currentMotor) {
    //   lastMotor.set(ControlMode.PercentOutput, 0.0);
    //   currentMotor = motorChooser.getSelected();
    //   lastMotor = currentMotor;
    // }

    if (Robot.oi.getButtonB()) {
      talon.set(ControlMode.PercentOutput, Robot.oi.getLeftY() / 10.0);
    } else if (Robot.oi.getButtonY()) {
      talon.set(ControlMode.PercentOutput, Robot.oi.getLeftY());      
    } else {
      talon.set(ControlMode.PercentOutput, 0.0);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    L.ogCmdEnd(this);
    this.cleanUp();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    L.ogCmdInterrupted(this);
    this.cleanUp();
  }

  private void cleanUp() {
    talon.set(ControlMode.PercentOutput, 0.0);
  }
}
