package org.firstinspires.ftc.teamcode.subsystems.Hood;

import com.qualcomm.robotcore.hardware.Servo;

public class HoodIO {
    private final Servo h;

    private static final double GEAR_RATIO = 180.0 / 18.0; // = 10

    // Hood angle limits (degrees)
    public static double MIN_HOOD_ANGLE = 15.0;
    public static double MAX_HOOD_ANGLE = 40.0;

    private static final double SERVO_RANGE_DEGREES = 300.0;

    private static final double BASE_SERVO_OFFSET_DEGREES = MIN_HOOD_ANGLE * GEAR_RATIO; // = 150

    //ToDo get the mounting offset later before testing
    public static double MOUNTING_OFFSET_DEGREES = 0.0;

    public HoodIO(Servo h) {
        this.h =h;
    }

    //set's position of hood between 15 en 40 degrees
    public void setHoodAngle(double hoodAngleDegrees) {
        double clampedHoodAngle = Math.max(MIN_HOOD_ANGLE, Math.min(MAX_HOOD_ANGLE, hoodAngleDegrees));

        // Convert hood angle to required servo rotation
        double servoAngleDegrees = clampedHoodAngle * GEAR_RATIO;

        // Subtract base offset and apply mounting correction
        double adjustedAngleDegrees = servoAngleDegrees - BASE_SERVO_OFFSET_DEGREES + MOUNTING_OFFSET_DEGREES;

        // Convert to servo position (0.0 - 1.0)
        double position = adjustedAngleDegrees / SERVO_RANGE_DEGREES;

        setPosition(position);
    }

    //set's position of servo between 0 and 1
    public void setPosition(double position) {
        double mapped = (position + 1.0) / 2.0; // -1.0 → 0.0, 0.0 → 0.5, 1.0 → 1.0
        double clamped = Math.max(0.0, Math.min(1.0, mapped));
        h.setPosition(clamped);
    }

    public double getPosition() {
        return h.getPosition();
    }
}
