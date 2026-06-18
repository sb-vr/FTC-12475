package org.firstinspires.ftc.teamcode.subsystems.ballstopper;

import com.qualcomm.robotcore.hardware.Servo;

public class BallstopperIO {
    private final Servo beunServo;

    // Hood angle limits (degrees)
    public static double MIN_HOOD_ANGLE = 0.0;
    public static double MAX_HOOD_ANGLE = 300.0;

    private static final double SERVO_RANGE_DEGREES = 300.0;

    public BallstopperIO(Servo beunServo) {
        this.beunServo = beunServo;
    }

    //set's position of hood between 15 en 40 degrees
    public void setAngle(double hoodAngleDegrees) {
        double servoAngleDegrees = Math.max(MIN_HOOD_ANGLE, Math.min(MAX_HOOD_ANGLE, hoodAngleDegrees));

        double position = (servoAngleDegrees / SERVO_RANGE_DEGREES) * 2.0 - 1.0;

        setPosition(position);
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
