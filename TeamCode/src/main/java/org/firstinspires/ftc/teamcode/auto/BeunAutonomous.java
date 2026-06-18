package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.ShootingLookupTable;
import org.firstinspires.ftc.teamcode.subsystems.hood.HoodIO;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDriveRR;
import org.firstinspires.ftc.teamcode.subsystems.flywheel.FlywheelIO;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionIO;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionLocalize;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "red15")
public class BeunAutonomous extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor i = null, st = null, s2 = null;
        DcMotorEx s1 = null;
        Servo beunServo = null, h = null;
        WebcamName camera = null;

        Pose2d beginPose = new Pose2d(10.46, 133.54, Math.toRadians(45));

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDriveRR.class)) {
            try {
                s1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
            } catch (Exception e) {
                telemetry.addLine("Missing: flywheel1");
            }
            try {
                h = hardwareMap.get(Servo.class, "hood");
            } catch (Exception e) {
                telemetry.addLine("Missing: hood");
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
                camera = hardwareMap.get(WebcamName.class, "Webcam 1");
            } catch (Exception e) {
                telemetry.addLine("Missing: Webcam 1");
            }


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

            HoodIO hood = (h != null) ? new HoodIO(h) : null;

            VisionIO vision = (camera != null) ? new VisionIO(camera) : null;

            VisionLocalize localize = (vision != null) ? new VisionLocalize(vision) : null;

            if (localize != null) localize.update();


            double distance = 0;
            if (vision != null) {
                AprilTagDetection detection = vision.getTagDistance(Constants.ALLIANCE_TAG_ID_RED);
                distance = (detection != null) ? detection.ftcPose.range : 0.0;
            }

            double targetVelocity = ShootingLookupTable.getFlywheelVelocity(distance);
            double targetAngle = ShootingLookupTable.getHoodAngle(distance);

            waitForStart();
            if (isStopRequested()) return;

            Actions.runBlocking(
                    drive.actionBuilder(beginPose).afterTime(0, packet -> {
                        if (flywheel != null) flywheel.setShootingVelocity(targetVelocity);
                        if (hood != null) hood.setHoodAngle(targetAngle);
                        return false;
                    }).waitSeconds(3).afterTime(0, packet -> {
                        if (flywheel != null) flywheel.setAutonomousShoootingVelocity(0);
                        if (hood != null) hood.setHoodAngle(0);
                        return false;
                    }).lineToY(48).build());

        }
    }
}
