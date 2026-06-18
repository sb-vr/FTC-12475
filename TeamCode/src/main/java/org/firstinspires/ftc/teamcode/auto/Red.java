package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDriveRR;
import org.firstinspires.ftc.teamcode.subsystems.flywheel.FlywheelIO;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeIO;
import org.firstinspires.ftc.teamcode.subsystems.storage.StorageIO;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;
@Config
@Autonomous(name = "FirstAutonoomRood", group = "Autonomous")
public class Red extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor i = null, st = null, s2 = null;
        DcMotorEx s1 = null;
        Servo beunServo = null;

        Pose2d beginPose = new Pose2d(133.54, 133.54, Math.toRadians(45));

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDriveRR.class)) {
            try { s1 = hardwareMap.get(DcMotorEx.class, "flywheel1");} catch (Exception e) {telemetry.addLine("Missing: flywheel1");}
            try {s2 = hardwareMap.dcMotor.get("flywheel2");} catch (Exception e) {telemetry.addLine("Mising: flywheel2");}
            try {st = hardwareMap.dcMotor.get("storage");} catch (Exception e) {telemetry.addLine("Missing: storage");}
            try {i = hardwareMap.dcMotor.get("intake");} catch (Exception e) {telemetry.addLine("Missing: intake");}
            try {beunServo = hardwareMap.get(Servo.class, "ballstopper");} catch (Exception e) {telemetry.addLine("Missing: ballstopper");}
            telemetry.update();

            if (s1 != null) {
                s1.setDirection(DcMotorSimple.Direction.FORWARD);
                s1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (s2 != null) s2.setDirection(DcMotorSimple.Direction.REVERSE);
            if (st != null) st.setDirection(DcMotorSimple.Direction.FORWARD);
            if (i != null) i.setDirection(DcMotorSimple.Direction.FORWARD);

            MecanumDriveRR drive = new MecanumDriveRR(hardwareMap, beginPose);

            FlywheelIO flywheel = (s1 != null && s2 != null) ? new FlywheelIO(s1, s2) : null;

            IntakeIO intake = (i != null) ? new IntakeIO(i) : null;

            StorageIO storage = (st != null) ? new StorageIO(st) : null;

            waitForStart();
            if (isStopRequested()) return;

            Servo BeunServo = beunServo;

            Actions.runBlocking(
                drive.actionBuilder(beginPose)

                    .afterTime(0, packet -> {
                        if (flywheel != null) flywheel.setAutonomousShoootingVelocity(1400);
                        if (BeunServo != null) BeunServo.setPosition(0.45);
                        return false;
                    })
                    .waitSeconds(2.5)
                    .afterTime(0, packet -> {
                        if (storage != null) storage.setPower(1.0);
                        if (intake != null) intake.setPower(1.0);
                        return false;
                    })
                    .waitSeconds(3)
                    .afterTime(0, packet -> {
                        if (flywheel != null) flywheel.setAutonomousShoootingVelocity(500);
                        if (storage != null) storage.setPower(0);
                        if (BeunServo != null) {BeunServo.setPosition(1);}
                        return false;
                    })
                    .turn(Math.toRadians(45))
                    .afterTime(0, packet -> {
                        if (intake != null) intake.setPower(1.0);
                        return false;
                    })
                    .lineToY(72)
                    .waitSeconds(1.0)
                    .afterTime(0, packet -> {
                        if (intake != null) intake.setPower(0.0);
                        if (BeunServo != null) {BeunServo.setPosition(0.45);}
                        return false;
                    })
                    .lineToY(133.54)
                    .turn(Math.toRadians(-45))
                    .afterTime(0, packet -> {
                        if (flywheel != null) flywheel.setAutonomousShoootingVelocity(1400);
                        return false;
                    })
                    .waitSeconds(2)
                    .afterTime(0, packet -> {
                        if (storage != null) storage.setPower(1.0);
                        if (intake != null) intake.setPower(1.0);
                        return false;
                    })
                    .waitSeconds(5)
                    .afterTime(0,packet -> {
                        if (flywheel != null) flywheel.setAutonomousShoootingVelocity(0.0);
                        if (storage != null) storage.setPower(0);
                        return false;
                    })
                    .build());
        }
    }
}