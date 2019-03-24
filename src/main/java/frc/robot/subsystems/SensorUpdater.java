/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.tekerz.utilities.L;

/**
 * Add your docs here.
 */
public class SensorUpdater implements Runnable {

    Timer t = new Timer();
    double lastTime = 0.0;
    double now = 0.0;

    public static volatile double 
        distanceFromCenter,
        goodEnoughTape = 1.75,
        voltageDifWhenCentered = 1.0;

    private List<AnalogInput> analogs = new ArrayList<AnalogInput>();

    private AnalogInput 
    first, 
    second, 
    third;

    public SensorUpdater() {
        for (int i = 0; i < 8; i++) {
            analogs.add(new AnalogInput(i));
        }
    }

    public double leftEncoderOverLinePosition;
    public double rightEncoderOverLinePosition;

    public void getLinePosition(){
        for(int i = 0; i < analogs.size(); i++){
            if(analogs.get(i).getVoltage() >= goodEnoughTape){
                leftEncoderOverLinePosition = Robot.Subsystems.drivetrain.getEnc(true);
                rightEncoderOverLinePosition = Robot.Subsystems.drivetrain.getEnc(false);
            }
        }
    }

    private void howFarOffCenter() {
        first = second = third = null;
        for (int i = 0; i < analogs.size(); i++) {
            if (analogs.get(i).getVoltage() > first.getVoltage()) {
                third = second;
                second = first;
                first = analogs.get(i);
            }
            else if (analogs.get(i).getVoltage() > second.getVoltage()) {
                third = second;
                second = analogs.get(i);
            }
            else if (analogs.get(i).getVoltage() > third.getVoltage()){
                third = analogs.get(i);
            }
        }

        distanceFromCenter = ((first.getChannel() - 3.5) * 2);
        distanceFromCenter += (first.getChannel() - second.getChannel());

        double direction = (first.getVoltage() - second.getVoltage())
                / Math.abs((first.getVoltage() - second.getVoltage()));

        if (Math.abs(first.getVoltage() - second.getVoltage()) > 1.0) {
            L.ogSD("Inches Off Center", ((first.getChannel() - 3.5) * 2));
        } else {
            L.ogSD("Inches Off Center", ((first.getChannel() - 3.5) * 2) + (.5 * direction));
        }
    }

    public void sort(){
        Collections.sort(analogs, new Comparator<AnalogInput>(){
            @Override
            public int compare(AnalogInput arg0, AnalogInput arg1) {
                return arg0.getVoltage() > arg1.getVoltage() ? -1 : (arg0.getVoltage() < arg1.getVoltage() ? 1 : 0);
            }
            
        });
    }

    public void run() {
        t.start();
        try {
            while (true) {
                Thread.sleep(2);
                howFarOffCenter();
                getLinePosition();
                now = t.get();
                lastTime = now;
                // distanceFromBay = RobotMap.Analogs.a0.getVoltage() -
                // RobotMap.Analogs.a1.getVoltage();
                // L.ogSD("sensor class encoder value", distanceSinceBar);
                // for (AnalogInput var : analogs) {
                //     SmartDashboard.putNumber("analog " + var.getChannel(), var.getVoltage());
                // }
            }
        } catch (Exception ex) {
            System.out.println(ex);

        }
    }
}
