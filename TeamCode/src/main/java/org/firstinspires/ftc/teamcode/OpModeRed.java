package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.RED_GOAL_X;
import static org.firstinspires.ftc.teamcode.Constants.RED_GOAL_Y;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Constants.*;
import org.firstinspires.ftc.teamcode.roadrunnerMeuk.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.Hood.HoodIO;
import org.firstinspires.ftc.teamcode.subsystems.ballstopper.BallstopperIO;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDriveRR;
import org.firstinspires.ftc.teamcode.subsystems.flywheel.FlywheelIO;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeIO;
import org.firstinspires.ftc.teamcode.subsystems.storage.StorageIO;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionIO;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionLocalize;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class OpModeRed extends LinearOpMode {

    private boolean isAligning = false;
    private final double Kp = 0.04; // Begin hiermee, verhoog naar 0.05 of 0.06 als hij te traag is
    private final double TOLERANCE_DEGREES = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor i = null, st = null, s2 = null;
        DcMotorEx s1 = null;
        Servo h = null, beunServo = null;
        WebcamName camera = null;
        PinpointLocalizer pinpointLocalizer = null;

        final ElapsedTime timer = new ElapsedTime();
        boolean wasShootingLastTick = false;

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
            h = hardwareMap.get(Servo.class, "hood");
        } catch (Exception e) {
            telemetry.addLine("Missing: hood");
        }
        try {
            beunServo = hardwareMap.get(Servo.class, "ballstopper");
        } catch (Exception e) {
            telemetry.addLine("Missing: ballstopper");
        }
        try {
            pinpointLocalizer = new PinpointLocalizer(hardwareMap, 0.001978956, new Pose2d(0, 0, 0));
        } catch (Exception e) {
            telemetry.addLine("Missing: PinpointLocalizer initialisatie mislukt!");
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

        MecanumDriveRR drive = new MecanumDriveRR(hardwareMap, new Pose2d(0, 0, 0));

        FlywheelIO flywheel = (s1 != null && s2 != null) ? new FlywheelIO(s1, s2) : null;

        BallstopperIO ballstopper = (beunServo != null) ? new BallstopperIO(beunServo) : null;

        IntakeIO intake = (i != null) ? new IntakeIO(i) : null;

        StorageIO storage = (st != null) ? new StorageIO(st) : null;

        HoodIO hood = (h != null) ? new HoodIO(h) : null;

        VisionIO vision = (camera != null) ? new VisionIO(camera) : null;

        VisionLocalize localize = (vision != null) ? new VisionLocalize(vision) : null;

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (localize != null) {
                localize.update();
            }

            double distance = 0;
            if (vision != null) {
                AprilTagDetection detection = vision.getTagDistance(Constants.ALLIANCE_TAG_ID_RED);
                distance = (detection != null) ? detection.ftcPose.range : 0.0;
            }

            Pose2d currentPinpointPose = null;
            if (pinpointLocalizer != null) {
                pinpointLocalizer.update();
                currentPinpointPose = pinpointLocalizer.getPose();
                telemetry.addLine("currentHeading: " + currentPinpointPose.heading.toDouble());
            }

            double headingInRad = 0;
            if (currentPinpointPose != null) {
                headingInRad = currentPinpointPose.heading.toDouble();
            }
            double headingInDeg = Math.toDegrees(headingInRad);

            if (gamepad1.dpadUpWasPressed()) {
                drive = new MecanumDriveRR(hardwareMap, new Pose2d(9, 6.3, 180));
            }

            Pose3D currentPosition = null;
            if (localize != null) {
                currentPosition = localize.getLastPose();
            }

            isAligning = gamepad1.x && (currentPosition != null);

            if (drive != null) {
                if (isAligning) {
//                    double currentX = localize.getX(DistanceUnit.INCH);
//                    double currentY = localize.getY(DistanceUnit.INCH);
                    double currentX = currentPinpointPose.position.x;
                    double currentY = currentPinpointPose.position.y;

                    double deltaX = RED_GOAL_X - currentX;
                    double deltaY = RED_GOAL_Y - currentY;

                    double targetHeading = Math.toDegrees(Math.atan2(deltaX, deltaY));

                    double normalizedHeadingInDeg = headingInDeg;
                    while (normalizedHeadingInDeg > 180) normalizedHeadingInDeg -= 360;
                    while (normalizedHeadingInDeg <= -180) normalizedHeadingInDeg += 360;

                    double error = targetHeading - normalizedHeadingInDeg;
                    while (error > 180) error -= 360;
                    while (error <= -180) error += 360;

                    if (Math.abs(error) < TOLERANCE_DEGREES) {
                        isAligning = false;
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0)); // Stop robot
                    } else {
                        double turnPower = error * Kp;

                        double maxTurnPower = 0.4;
                        turnPower = Math.max(-maxTurnPower, Math.min(maxTurnPower, turnPower));

                        if (Math.abs(turnPower) < 0.05) {
                            turnPower = Math.signum(turnPower) * 0.05;
                        }

                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), turnPower));
                    }
                } else {
                    if (Math.abs(gamepad1.left_stick_x) > 0.1 || Math.abs(gamepad1.right_stick_x) > 0.1) {
                        isAligning = false;
                    }

                    // Drive is now field-centric
                    drive.updatePoseEstimate();
                    Pose2d currentPose = drive.localizer.getPose();

                    //Joystick input
                    double inputY = -gamepad1.left_stick_y;
                    double inputX = gamepad1.left_stick_x;
                    double inputRx = gamepad1.right_stick_x;

                    inputY = -inputY;

                    double headingRadians = currentPose.heading.toDouble();
                    double rotatedX = inputX * Math.cos(headingRadians) - inputY * Math.sin(headingRadians);
                    double rotatedY = inputX * Math.sin(headingRadians) + inputY * Math.cos(headingRadians);

                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(rotatedY, rotatedX), inputRx
                    ));

                    telemetry.addLine("rotatedX: " + rotatedX);
                    telemetry.addLine("rotatedY: " + rotatedY);

                    // reset field centric drive
                    if (gamepad1.dpadDownWasPressed()) {
                        pinpointLocalizer.setPose(new Pose2d(0, 0, 0));
                    }
                }
            }

            if (intake != null) intake.updateIntake(gamepad1);
            if (flywheel != null) flywheel.updateShooter(gamepad1);
            if (storage != null) storage.updateStorage(gamepad1);
            if (localize != null) localize.update();

            if (distance > 0) {
                double targetVelocity = ShootingLookupTable.getFlywheelVelocity(distance);
                telemetry.addLine("Target velocity: " + targetVelocity);
                double targetHoodAngle = ShootingLookupTable.getHoodAngle(distance);

                if (flywheel != null) {
                    flywheel.setShootingVelocity(targetVelocity);
                }
                if (hood != null) {
                    hood.setHoodAngle(targetHoodAngle);
                }
            }

            if (flywheel != null && flywheel.isShooting() && !wasShootingLastTick) {
                timer.reset();
            }
            wasShootingLastTick = flywheel != null && flywheel.isShooting();

            //ToDo check which angle the servo has to go to!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            if (storage != null) {
                if (flywheel != null && flywheel.isShooting()) {
                    if (beunServo != null) {
                        beunServo.setPosition(0.45);
                    }
                    if (timer.milliseconds() >= 2000) {
                        storage.setPower(0.8);
                    } else {
                        storage.setPower(0);
                    }
                } else if (intake != null && intake.isIntaking()) {
                    if (beunServo != null) {
                        beunServo.setPosition(1);
                    }
                    storage.setPower(0.8);
                } else {
                    storage.setPower(0);
                }
            }

            telemetry.addLine(Double.toString(distance));
            if (ballstopper != null) {
                telemetry.addLine(Double.toString(ballstopper.getPosition()));
            }
            telemetry.addLine(Double.toString(ShootingLookupTable.getFlywheelVelocity(distance)));
            telemetry.addLine(Double.toString(ShootingLookupTable.getHoodAngle(distance)));

            telemetry.update();
        }
        if (vision != null) vision.stop();
    }
}
