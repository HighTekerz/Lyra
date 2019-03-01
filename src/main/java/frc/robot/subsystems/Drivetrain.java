/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.drivetrain.DriveWithJoy;
import frc.robot.tekerz.utilities.L;

public class Drivetrain extends Subsystem {
  CANSparkMax 
    rightMotorLead = RobotMap.Sparks.rightMotorLead,
    rightMotorFollower = RobotMap.Sparks.rightMotorFollower,
    leftMotorLead = RobotMap.Sparks.leftMotorLead,
    leftMotorFollower = RobotMap.Sparks.leftMotorFollower;

  PigeonIMU imu = RobotMap.Pigeon.imu;

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new DriveWithJoy());
    L.ogSD("DriveWithJoy", this);
  }

  public Drivetrain() {
    rightMotorFollower.follow(rightMotorLead);
    leftMotorFollower.follow(leftMotorLead);
  }

  public void setWheelSpeed(double leftSpeed, double rightSpeed){
    rightMotorLead.set(rightSpeed);
    leftMotorLead.set(leftSpeed);
  }

  public void arcadeDrive(double straightSpeed, double turnSpeed){
    double leftSpeed = -straightSpeed - turnSpeed;
    double rightSpeed = -(-straightSpeed + turnSpeed);

    setWheelSpeed(leftSpeed, rightSpeed);
  }

  public void log() {
    
  }
}