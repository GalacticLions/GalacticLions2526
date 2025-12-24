package org.firstinspires.ftc.team5898;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team5898.pedroPathing.Constants;

@Autonomous(name = "Pedro Alpha Auto")
public class PedroPathing_Test_BlueSide extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opModeTimer;

    public enum PathState {
    // START POSITION - END POSITION
    // DRIVE > MOVEMENT STATE
    // SHOOT > SCORING
    DRIVE_STARTPOS_SHOOTPOS,
    SHOOT_PRELOAD,

    // DRIVE_SHOOTPOS_FIRSTROW,
    // DRIVE_FIRSTROW_GRABROW1,
    // DRIVE_GRABROW1_SHOOTPOS,
    // SHOOT_ROW1,

    // DRIVE_SHOOTPOS_SECONDROW,
    // DRIVE_SECONDROW_GRABROW2,
    // DRIVE_GRABROW2_SHOOTPOS,
    // SHOOT_ROW2,

    // DRIVE_SHOOTPOS_THIRDROW,
    // DRIVE_THIRDROW_GRABROW3,
    // DRIVE_GRABROW3_SHOOTPOS,
    // SHOOT_ROW2,

    DRIVE_SHOOTPOS_ENDPOS,
    }

    PathState pathState;

    private final Pose startPose = new Pose(21, 125, Math.toRadians(140));
    private final Pose shootPose = new Pose(46.5, 100, Math.toRadians(135));


    private final Pose endPose = new Pose(48, 126, Math.toRadians(180));

    private PathChain driveStartPosShootPos, driveShootPosEndPos;

    public void buildPaths() {
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        driveShootPosEndPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
    }


    public void statePathUpdate() {
        switch(pathState) {
            case DRIVE_STARTPOS_SHOOTPOS:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;

            case SHOOT_PRELOAD:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    follower.followPath(driveShootPosEndPos, true);
                    setPathState(PathState.DRIVE_SHOOTPOS_ENDPOS);
                    // TODO ADD FLYWHEEL LOGIC
                }
                break;

            case DRIVE_SHOOTPOS_ENDPOS:
                if (!follower.isBusy()) {
                    telemetry.addLine("Done All Paths");
                }
                break;
            default:
                telemetry.addLine("No State Commanded");
                break;

        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOS_SHOOTPOS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        // TODO ADD IN ANY OTHER INIT MECHANISMS

        buildPaths();
        follower.setPose(startPose);
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("Path State", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());

    }
}
