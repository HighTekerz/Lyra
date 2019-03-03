/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.test;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.hablifter.ClearEncoder;

public class AddTestCommands extends Command {
  public AddTestCommands() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    SmartDashboard.putData(new TestSparkMotors(RobotMap.Sparks.habLifterArmsLead, "5,6 hablifter arms"));
    SmartDashboard.putData(new TestSparkMotors(RobotMap.Sparks.leftMotorLead, "1,3 left drive"));
    SmartDashboard.putData(new TestSparkMotors(RobotMap.Sparks.rightMotorLead, "2,4 right drive"));
    SmartDashboard.putData(new TestTalonMotors(RobotMap.Talons.habLifterWheelLead, "7,8 hablifter wheels"));
    SmartDashboard.putData(new TestTalonMotors(RobotMap.Talons.intake, "11 cargo intake"));
    SmartDashboard.putData(new TestTalonMotors(RobotMap.Talons.liftLead, "9, 10 elevator"));
    SmartDashboard.putData(new TestSolenoiods(RobotMap.Pneumatics.habLifterLegs1, "MainPCM-0"));
    SmartDashboard.putData(new TestSolenoiods(RobotMap.Pneumatics.habLifterLegs2, "MainPCM-1"));
    SmartDashboard.putData(new TestSolenoiods(RobotMap.Pneumatics.intakeFlap, "ArmPCM-2"));
    SmartDashboard.putData(new TestSolenoiods(RobotMap.Pneumatics.hPFlap, "ArmPCM-3"));
    SmartDashboard.putData(new TestSolenoiods(RobotMap.Pneumatics.hPPusher, "ArmPCM-4"));
    SmartDashboard.putData(new TestSolenoiods(RobotMap.Pneumatics.hPFinger, "ArmPCM-5"));
    SmartDashboard.putData("clear arm encs", new ClearEncoder());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
