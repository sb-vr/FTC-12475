package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class OpModeBlue extends MainOpMode {
    @Override
    protected double getGoalX() {
        return Constants.BLUE_GOAL_X;
    }

    @Override
    protected double getGoalY() {
        return Constants.BLUE_GOAL_Y;
    }

    @Override
    protected int getAllianceID() {
        return Constants.ALLIANCE_TAG_ID_BLUE;
    }
}
