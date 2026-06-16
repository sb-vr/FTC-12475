package org.firstinspires.ftc.teamcode.subsystems.storage;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class StorageIO {
    private DcMotor st;

    private double targetPower = 0.0;


    public StorageIO(DcMotor st) {
        this.st = st;
    }

    public void updateStorage(Gamepad gamepad1) {
        if (gamepad1.a) {
            targetPower = 0.8;
        } else if (gamepad1.y) {
            targetPower = -1;
        } else {
            targetPower = 0.0;
        }
        st.setPower(targetPower);
    }

    public void setPower(double power) {
        st.setPower(power);
    }
}
