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
