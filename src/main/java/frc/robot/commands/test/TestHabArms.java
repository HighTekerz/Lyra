/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.test;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.tekerz.utilities.L;

public class TestHabArms extends Command {
  CANSparkMax spark = RobotMap.Sparks.habLifterArmsLead;
  CANSparkMax sparkFollow = RobotMap.Sparks.habLifterArmsFollower;

  private TestHabArms() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.Subsystems.drivetrain);
    requires(Robot.Subsystems.elevator);
    requires(Robot.Subsystems.habLifter);
    requires(Robot.Subsystems.multiArm);
  }

  public TestHabArms(String name) {
    this();
    this.setName(name);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    L.ogCmdInit(this);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (Robot.oi.getButtonB()) {
      spark.set(Robot.oi.getLeftStickY() / 10.0);
      sparkFollow.set(-Robot.oi.getLeftStickY() / 10.0);
    } else if (Robot.oi.getButtonA()) {
      spark.set(Robot.oi.getLeftStickY() / 5.0);      
      sparkFollow.set(-Robot.oi.getLeftStickY() / 5.0);      
    } else if (Robot.oi.getButtonY()) {
      spark.set(Robot.oi.getLeftStickY());      
      sparkFollow.set(-Robot.oi.getLeftStickY());      
    } else {
      spark.set(0.0);
      sparkFollow.set(0.0);
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
    spark.set(0.0);
    sparkFollow.set(0.0);
  }
}
