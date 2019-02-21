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

/**
 * Add your docs here.
 */
public class Sensors extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private Thread updater = new Thread(new SensorUpdater());
  private AnalogInput ai0 = new AnalogInput(0);
  private AnalogInput ai1 = new AnalogInput(1);

  public Sensors() {
    updater.start();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new SensorsCommand());
  }

  public void log() {
    SmartDashboard.putNumber("analog 0", ai0.getAverageVoltage());
    SmartDashboard.putNumber("analog 1", ai1.getAverageVoltage());
  }
}
