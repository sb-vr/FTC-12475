package org.firstinspires.ftc.teamcode.subsystems.vision;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class VisionLocalize {
    private VisionIO vision;

    private Pose3D lastPose;

    public VisionLocalize(VisionIO vision) {
        this.vision = vision;
    }

    public void update() {
        List<AprilTagDetection> detections = vision.getDetections();

        if (detections.size() > 0) {
            for (AprilTagDetection tag : detections) {
                if (tag.metadata != null && !tag.metadata.name.contains("Obelisk")) {
                    lastPose = tag.robotPose;
                    break;
                }
            }
        }
    }

    public Pose3D getLastPose() {
        return lastPose;
    }

    public double getX(DistanceUnit unit) {
        return (lastPose != null) ? lastPose.getPosition().x : Double.NaN;
    }

    public double getY(DistanceUnit unit) {
        return (lastPose != null) ? lastPose.getPosition().y : Double.NaN;
    }

    public double getZ(DistanceUnit unit) {
        return (lastPose != null) ? lastPose.getPosition().z : Double.NaN;
    }

    public double getYaw(AngleUnit unit) {
        return (lastPose != null) ? lastPose.getOrientation().getYaw(unit) : Double.NaN;
    }

    public double getPitch(AngleUnit unit) {
        return (lastPose != null) ? lastPose.getOrientation().getPitch(unit) : Double.NaN;
    }

    public double getRoll(AngleUnit unit) {
        return (lastPose != null) ? lastPose.getOrientation().getRoll(unit) : Double.NaN;
    }
}
