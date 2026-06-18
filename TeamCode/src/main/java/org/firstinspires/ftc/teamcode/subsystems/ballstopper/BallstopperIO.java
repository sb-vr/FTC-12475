package org.firstinspires.ftc.teamcode.subsystems.ballstopper;

import com.qualcomm.robotcore.hardware.Servo;

public class BallstopperIO {
    private final Servo beunServo;

    public BallstopperIO(Servo beunServo) {
        this.beunServo = beunServo;
    }

    //set's position of servo between 0 and 1
    public void setPosition(double position) {
        double clamped = Math.max(-1.0, Math.min(1.0, position));
        beunServo.setPosition((clamped + 1.0) / 2.0);
    }

    public double getPosition() {
        return beunServo.getPosition();
    }
}
