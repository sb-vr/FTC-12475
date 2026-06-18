package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class IntakeIO {
    private DcMotor i;

    private double targetPower = 0.0;

    private boolean isIntaking = false;


    public IntakeIO(DcMotor i) {
        this.i = i;
    }

    public void updateIntake(Gamepad gamepad1) {
        if (gamepad1.left_bumper) {
            isIntaking = true;
            targetPower = 1;
        } else if (gamepad1.left_trigger > 0.5) {
            isIntaking = false;
            targetPower = 0.0;
        } else if (gamepad1.y) {
            isIntaking = false;
            targetPower = -1;
        } else if (!isIntaking) {
            targetPower = 0.0;
        }
        i.setPower(targetPower);
    }

    public void setPower(double power) {
        i.setPower(power);
    }

    public boolean isIntaking() {
        return isIntaking;
    }
}
