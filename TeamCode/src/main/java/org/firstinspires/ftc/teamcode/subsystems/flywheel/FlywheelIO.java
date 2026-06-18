package org.firstinspires.ftc.teamcode.subsystems.flywheel;

import static org.firstinspires.ftc.teamcode.subsystems.constants.FlywheelConstants.D;
import static org.firstinspires.ftc.teamcode.subsystems.constants.FlywheelConstants.F;
import static org.firstinspires.ftc.teamcode.subsystems.constants.FlywheelConstants.I;
import static org.firstinspires.ftc.teamcode.subsystems.constants.FlywheelConstants.P;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class FlywheelIO {
    private final DcMotorEx s1;
    private final DcMotor s2;
    private boolean shooting = false;
    private double shootingVelocity = 0.0;

    public FlywheelIO(DcMotorEx s1, DcMotor s2) {
        this.s1 = s1;
        this.s2 = s2;
        s1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void updateShooter(Gamepad gamepad1) {
        s1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                P, I, D, F
        ));

        if (gamepad1.rightBumperWasPressed()) {
            shooting = true;
        } else if (gamepad1.right_trigger > 0.5) {
            shooting = false;
        }

        if (shooting) {
            s1.setVelocity(shootingVelocity);
        } else {
            s1.setVelocity(500);
        }

        s2.setPower(s1.getPower());
    }

    public void setShootingVelocity(double ticksPerSecond) {
        this.shootingVelocity = ticksPerSecond;
    }

    public void setAutonomousShoootingVelocity(double ticksPerSecond) {
        this.shootingVelocity = ticksPerSecond;
        s1.setVelocity(ticksPerSecond);
        s2.setPower(s1.getPower());
    }

    public boolean isShooting() {
        return shooting;
    }
}
