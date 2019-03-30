/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.tekerz.utilities.L;


public class Sensors extends Subsystem {
  private SensorUpdater sensorThread = new SensorUpdater();
  private Thread updater = new Thread(sensorThread);

  public Sensors() {
    updater.start();
  }

  @Override
  public void initDefaultCommand() {
  }

  public void log() {
  }
}
