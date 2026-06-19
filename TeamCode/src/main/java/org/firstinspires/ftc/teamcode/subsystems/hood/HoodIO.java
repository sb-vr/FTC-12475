package org.firstinspires.ftc.teamcode.subsystems.hood;

import static org.firstinspires.ftc.teamcode.subsystems.constants.HoodConstants.BASE_SERVO_OFFSET_DEGREES;
import static org.firstinspires.ftc.teamcode.subsystems.constants.HoodConstants.GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.subsystems.constants.HoodConstants.MAX_HOOD_ANGLE;
import static org.firstinspires.ftc.teamcode.subsystems.constants.HoodConstants.MIN_HOOD_ANGLE;
import static org.firstinspires.ftc.teamcode.subsystems.constants.HoodConstants.SERVO_RANGE_DEGREES;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

//@Config
public class HoodIO {
    private final Servo h;

    //ToDo get the mounting offset later before testing
    public static double MOUNTING_OFFSET_DEGREES = 0.0;

    public HoodIO(Servo h) {
        this.h = h;
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
