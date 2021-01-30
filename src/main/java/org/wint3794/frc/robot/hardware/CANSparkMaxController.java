/*
 * File: CANSparkMaxController.java
 * Project: Robot Programming 2021
 * File Created: Tuesday, 26th January 2021 8:35 pm
 * Author: Manuel Diaz and Obed Garcia
 * 
 * Copyright (c) 2021 WinT 3794. Under MIT License.
 */

package org.wint3794.frc.robot.hardware;

import com.revrobotics.CANSparkMax;

import org.wint3794.frc.robot.Robot;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;

public class CANSparkMaxController extends CANSparkMax {

    private boolean isReal;
    private boolean inverted;

    public CANSparkMaxController(int deviceID, MotorType type) {
        super(deviceID, type);
        this.isReal = Robot.isReal();
    }

    @Override
    public void set(double speed) {
        if (isReal) {
            super.set(speed);
        } else {
            double finalSpeed = inverted ? -speed : speed;

            int dev = SimDeviceDataJNI.getSimDeviceHandle("SPARK MAX [" + this.getDeviceId() + "]");
            SimDouble output = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Applied Output"));

            output.set(finalSpeed);
        }
    }

    @Override
    public void setInverted(boolean isInverted) {
        if (isReal) {
            super.setInverted(isInverted);
            return;
        }

        this.inverted = isInverted;
    }

    @Override
    public boolean getInverted() {
        if (isReal){
            return super.getInverted();
        }

        return this.inverted;
    }    
}
