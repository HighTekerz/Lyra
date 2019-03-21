/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.RobotMap.Analogs;
import frc.robot.tekerz.utilities.L;

/**
 * Add your docs here.
 */
public class SensorUpdater implements Runnable {

    Timer t = new Timer();
    double lastTime = 0.0;
    double now = 0.0;


    public static volatile double
        distanceSinceBar,
        distanceFromBay;

    int i = 0;
    public void run() {
        t.start();
        try {
            while (true) {
                Thread.sleep(2);
                now = t.get();
                lastTime = now;
                distanceFromBay = RobotMap.Analogs.a0.getVoltage() - RobotMap.Analogs.a1.getVoltage();
                distanceSinceBar = RobotMap.Sparks.leftMotorLead.getEncoder().getPosition();
                L.ogSD("sensor class encoder value", distanceSinceBar);
            }
        } catch (Exception ex) {
            System.out.println(ex);

        }

    }

    public void log() {
    
    }
  }
