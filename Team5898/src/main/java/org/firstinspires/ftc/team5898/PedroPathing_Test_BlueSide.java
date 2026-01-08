package org.firstinspires.ftc.team5898;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.team5898.pedroPathing.Constants;

@Autonomous(name = "PedroPathing Blue")
public class PedroPathing_Test_BlueSide extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
    // START POSITION - END POSITION
    // DRIVE > MOVEMENT STATE
    // SHOOT > SCORING
    DRIVE_STARTPOS_SHOOTPOS,
    SHOOT_PRELOAD,

    DRIVE_SHOOTPOS_FIRSTROW,
    DRIVE_FIRSTROW_GRABROW1,
    DRIVE_GRABROW1_SHOOTPOS,
    SHOOT_ROW1,

    DRIVE_SHOOTPOS_SECONDROW,
    DRIVE_SECONDROW_GRABROW2,
    DRIVE_GRABROW2_SHOOTPOS,
    SHOOT_ROW2,

    DRIVE_SHOOTPOS_THIRDROW,
    DRIVE_THIRDROW_GRABROW3,
    DRIVE_GRABROW3_SHOOTPOS,
    SHOOT_ROW3,

    DRIVE_SHOOTPOS_ENDPOS,
    }

    PathState pathState;

    private final Pose startPose = new Pose(21, 125, Math.toRadians(140));
    private final Pose shootPose = new Pose(46.5, 100, Math.toRadians(135));
    private final Pose firstRowPose = new Pose(42, 84, Math.toRadians(180));
    private final Pose firstGrabPose = new Pose(15, 84, Math.toRadians(180));
    private final Pose secondRowPose = new Pose(42, 59, Math.toRadians(180));
    private final Pose secondGrabPose = new Pose(9, 59, Math.toRadians(180));
    private final Pose thirdRowPose = new Pose(42, 35, Math.toRadians(180));
    private final Pose thirdGrabPose = new Pose(9, 35, Math.toRadians(180));

    private final Pose endPose = new Pose(48, 126, Math.toRadians(180));

    private PathChain driveStartPosShootPos, driveShootPosEndPos;
    private PathChain driveShootPosRow1Pos, driveRow1PosGrab1Pos, driveGrab1PosShootPos;
    private PathChain driveShootPosRow2Pos, driveRow2PosGrab2Pos, driveGrab2PosShootPos;
    private PathChain driveShootPosRow3Pos, driveRow3PosGrab3Pos, driveGrab3PosShootPos;

    public void buildPaths() {
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        driveShootPosRow1Pos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, firstRowPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), firstRowPose.getHeading())
                .build();
        driveRow1PosGrab1Pos = follower.pathBuilder()
                .addPath(new BezierLine(firstRowPose, firstGrabPose))
                .setLinearHeadingInterpolation(firstRowPose.getHeading(), firstGrabPose.getHeading())
                .build();
        driveGrab1PosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(firstGrabPose, shootPose))
                .setLinearHeadingInterpolation(firstGrabPose.getHeading(), shootPose.getHeading())
                .build();

        driveShootPosRow2Pos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, secondRowPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), secondRowPose.getHeading())
                .build();
        driveRow2PosGrab2Pos = follower.pathBuilder()
                .addPath(new BezierLine(secondRowPose, secondGrabPose))
                .setLinearHeadingInterpolation(secondRowPose.getHeading(), secondGrabPose.getHeading())
                .build();
        driveGrab2PosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(secondGrabPose, shootPose))
                .setLinearHeadingInterpolation(secondGrabPose.getHeading(), shootPose.getHeading())
                .build();

        driveShootPosRow3Pos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, thirdRowPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), thirdRowPose.getHeading())
                .build();
        driveRow3PosGrab3Pos = follower.pathBuilder()
                .addPath(new BezierLine(thirdRowPose, thirdGrabPose))
                .setLinearHeadingInterpolation(thirdRowPose.getHeading(), thirdGrabPose.getHeading())
                .build();
        driveGrab3PosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(thirdGrabPose, shootPose))
                .setLinearHeadingInterpolation(thirdGrabPose.getHeading(), shootPose.getHeading())
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
                    // TODO ADD FLYWHEEL LOGIC
                    follower.followPath(driveShootPosRow1Pos, true);
                    setPathState(PathState.DRIVE_SHOOTPOS_FIRSTROW);
                }
                break;

            case DRIVE_SHOOTPOS_FIRSTROW:
                follower.followPath(driveShootPosRow1Pos, true);
                setPathState(PathState.DRIVE_FIRSTROW_GRABROW1);
                break;
            case DRIVE_FIRSTROW_GRABROW1:
                follower.followPath(driveRow1PosGrab1Pos, true);
                setPathState(PathState.DRIVE_GRABROW1_SHOOTPOS);
                break;
            case DRIVE_GRABROW1_SHOOTPOS:
                follower.followPath(driveGrab1PosShootPos, true);
                setPathState(PathState.SHOOT_ROW1);
                break;
            case SHOOT_ROW1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    // TODO ADD FLYWHEEL LOGIC
                    follower.followPath(driveShootPosRow2Pos, true);
                    setPathState(PathState.DRIVE_SHOOTPOS_SECONDROW);
                }
                break;

            case DRIVE_SHOOTPOS_SECONDROW:
                follower.followPath(driveShootPosRow2Pos, true);
                setPathState(PathState.DRIVE_SECONDROW_GRABROW2);
                break;
            case DRIVE_SECONDROW_GRABROW2:
                follower.followPath(driveRow2PosGrab2Pos, true);
                setPathState(PathState.DRIVE_GRABROW2_SHOOTPOS);
                break;
            case DRIVE_GRABROW2_SHOOTPOS:
                follower.followPath(driveGrab2PosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_ROW2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    // TODO ADD FLYWHEEL LOGIC
                    follower.followPath(driveShootPosRow3Pos, true);
                    setPathState(PathState.DRIVE_SHOOTPOS_THIRDROW);
                }
                break;

            case DRIVE_SHOOTPOS_THIRDROW:
                follower.followPath(driveShootPosRow3Pos, true);
                setPathState(PathState.DRIVE_THIRDROW_GRABROW3);
                break;
            case DRIVE_THIRDROW_GRABROW3:
                follower.followPath(driveRow3PosGrab3Pos, true);
                setPathState(PathState.DRIVE_GRABROW3_SHOOTPOS);
                break;
            case DRIVE_GRABROW3_SHOOTPOS:
                follower.followPath(driveGrab3PosShootPos, true);
                setPathState(PathState.SHOOT_ROW3);
                break;
            case SHOOT_ROW3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    // TODO ADD FLYWHEEL LOGIC
                    follower.followPath(driveShootPosEndPos, true);
                    setPathState(PathState.DRIVE_SHOOTPOS_ENDPOS);
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
