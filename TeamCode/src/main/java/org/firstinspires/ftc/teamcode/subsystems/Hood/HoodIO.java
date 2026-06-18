package org.firstinspires.ftc.teamcode.subsystems.Hood;

import com.qualcomm.robotcore.hardware.Servo;

public class HoodIO {
    private final Servo h;

    private static final double GEAR_RATIO = 180.0 / 18.0; // = 10

    // Hood angle limits (degrees)
    public static double MIN_HOOD_ANGLE = 0.0;
    public static double MAX_HOOD_ANGLE = 60.0;

    private static final double SERVO_RANGE_DEGREES = 300.0;

    private static final double BASE_SERVO_OFFSET_DEGREES = MIN_HOOD_ANGLE * GEAR_RATIO;

    //ToDo get the mounting offset later before testing
    public static double MOUNTING_OFFSET_DEGREES = 0.0;

    public HoodIO(Servo h) {
        this.h =h;
    }

    //set's position of hood between 15 en 40 degrees
    public void setHoodAngle(double hoodAngleDegrees) {
        double clampedHoodAngle = Math.max(MIN_HOOD_ANGLE, Math.min(MAX_HOOD_ANGLE, hoodAngleDegrees));

        double servoAngleDegrees = clampedHoodAngle * GEAR_RATIO;
        double adjustedAngleDegrees = servoAngleDegrees - BASE_SERVO_OFFSET_DEGREES + MOUNTING_OFFSET_DEGREES;

        // Map to -1.0 – 1.0
        double position = (adjustedAngleDegrees / SERVO_RANGE_DEGREES) * 2.0 - 1.0;

        setPosition(position);
    }

    //set's position of servo between 0 and 1
    public void setPosition(double position) {
        double clamped = Math.max(-1.0, Math.min(1.0, position));
        h.setPosition((clamped + 1.0) / 2.0);
    }

    public double getPosition() {
        return h.getPosition();
    }
}
