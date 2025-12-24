package org.firstinspires.ftc.team5898;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.util.Timer;
@TeleOp
public class PedroPathingTestAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        // START POSITION TO END POSITION
        // DRIVE - MOVEMENT
        // SHOOT - SCORE
    }

    @Override
    public void init() {

    }

    public void loop() {

    }
}
