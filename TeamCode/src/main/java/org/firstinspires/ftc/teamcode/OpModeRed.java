package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class OpModeRed extends MainOpMode {
    @Override
    protected double getGoalX() {
        return Constants.RED_GOAL_X;
    }

    @Override
    protected double getGoalY() {
        return Constants.RED_GOAL_Y;
    }

    @Override
    protected int getAllianceID() {
        return Constants.ALLIANCE_TAG_ID_RED;
    }


    @Override
    protected Pose2d getResetPoint() {
        return new Pose2d(-63, 63, Math.toRadians(180));
    }
}
