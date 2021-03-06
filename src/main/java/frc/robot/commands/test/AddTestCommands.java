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
import frc.robot.commands.hablifter.DeployLegs;
import frc.robot.commands.hablifter.RunClimberWheels;
import frc.robot.commands.hablifter.RunClimberWheelsOnAngle;

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
    SmartDashboard.putData(new TestSolenoiods(RobotMap.Pneumatics.intakeFlap, "3 Cargo Flap"));
    SmartDashboard.putData(new TestSolenoiods(RobotMap.Pneumatics.hPFlap, "2 HP Flap"));
    SmartDashboard.putData(new TestSolenoiods(RobotMap.Pneumatics.hPPusher, "1 HP Pusher"));
    SmartDashboard.putData(new TestSolenoiods(RobotMap.Pneumatics.hPFinger, "0 HP Finger"));
    SmartDashboard.putData(new RunClimberWheelsOnAngle(.15, .75, -136));
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
