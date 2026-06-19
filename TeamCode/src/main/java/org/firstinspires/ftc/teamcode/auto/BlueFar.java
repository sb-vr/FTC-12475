package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ShootingLookupTable;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDriveRR;
import org.firstinspires.ftc.teamcode.subsystems.flywheel.FlywheelIO;
import org.firstinspires.ftc.teamcode.subsystems.hood.HoodIO;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeIO;
import org.firstinspires.ftc.teamcode.subsystems.storage.StorageIO;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionIO;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "BlueFar", group = "Autonomous")
public class BlueFar extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor i = null, st = null, s2 = null;
        DcMotorEx s1 = null;
        Servo beunServo = null, h = null;
        WebcamName camera = null;

        final Pose2d[] beginPose = {new Pose2d(0, 0, 0)};

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDriveRR.class)) {
            try {
                s1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
            } catch (Exception e) {
                telemetry.addLine("Missing: flywheel1");
            }
            try {
                s2 = hardwareMap.dcMotor.get("flywheel2");
            } catch (Exception e) {
                telemetry.addLine("Mising: flywheel2");
            }
            try {
                st = hardwareMap.dcMotor.get("storage");
            } catch (Exception e) {
                telemetry.addLine("Missing: storage");
            }
            try {
                i = hardwareMap.dcMotor.get("intake");
            } catch (Exception e) {
                telemetry.addLine("Missing: intake");
            }
            try {
                beunServo = hardwareMap.get(Servo.class, "ballstopper");
            } catch (Exception e) {
                telemetry.addLine("Missing: ballstopper");
            }
            try {
                camera = hardwareMap.get(WebcamName.class, "Webcam 1");
            } catch (Exception e) {
                telemetry.addLine("Missing: Webcam 1");
            }
            try {
                h = hardwareMap.get(Servo.class, "hood");
            } catch (Exception e) {
                telemetry.addLine("Missing: hood");
            }


            telemetry.update();

            if (s1 != null) {
                s1.setDirection(DcMotorSimple.Direction.FORWARD);
                s1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (s2 != null) s2.setDirection(DcMotorSimple.Direction.REVERSE);
            if (st != null) st.setDirection(DcMotorSimple.Direction.FORWARD);
            if (i != null) i.setDirection(DcMotorSimple.Direction.FORWARD);

            MecanumDriveRR drive = new MecanumDriveRR(hardwareMap, beginPose[0]);

            FlywheelIO flywheel = (s1 != null && s2 != null) ? new FlywheelIO(s1, s2) : null;

            StorageIO storage = (st != null) ? new StorageIO(st) : null;

            IntakeIO intake = (i != null) ? new IntakeIO(i) : null;

            VisionIO vision = (camera != null) ? new VisionIO(camera) : null;

            HoodIO hood = (h != null) ? new HoodIO(h) : null;

            double distance;
            if (vision != null) {
                AprilTagDetection detection = vision.getTagDistance(20);
                distance = (detection != null) ? detection.ftcPose.range : 0.0;
            } else {
                distance = 0;
            }

            waitForStart();
            if (isStopRequested()) return;

            //ToDo: chekc what the + 800 has to be its guesed for now


            Actions.runBlocking(
                    drive.actionBuilder(beginPose[0])
                            .afterTime(0, packet -> {
                                if (intake != null) intake.setPower(1);
                                return false;
                            })
                            .afterTime(0, packet -> {
                                if (flywheel != null) flywheel.setAutonomousShoootingVelocity(ShootingLookupTable.getFlywheelVelocity(distance)+800);
                                if (hood != null) hood.setHoodAngle(ShootingLookupTable.getHoodAngle(distance));
                                return false;
                            })
                            .waitSeconds(1.5).afterTime(0, packet -> {
                                if (storage != null) storage.setPower(0.7);
                                return false;
                            })
                            .waitSeconds(4).afterTime(0, packet -> {
                                if (flywheel != null) flywheel.setAutonomousShoootingVelocity(0);
                                if (hood != null) hood.setHoodAngle(0);
                                if (intake != null) intake.setPower(0);
                                return false;
                            })
                            .turn(Math.toRadians(80))
                            .lineToY(30)
                            .build()
            );
        }
    }
}
