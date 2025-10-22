package org.firstinspires.ftc.team26248;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="StarterBotAuto_Mecanum", group="StarterBot")
// @Disabled
public class StarterBotAuto_Mecanum extends OpMode {

    // ============================
    // === LAUNCHER PARAMETERS ===
    // ============================
    final double FEED_TIME = 0.20;
    final double LAUNCHER_TARGET_VELOCITY = 1050;
    final double LAUNCHER_MIN_VELOCITY    = 950;
    final double TIME_BETWEEN_SHOTS       = 5;

    // ============================
    // === DRIVE PARAMETERS     ===
    // ============================
    final double DRIVE_SPEED      = 0.5;
    final double ROTATE_SPEED     = 0.2;
    final double WHEEL_DIAMETER_MM = 96;
    final double ENCODER_TICKS_PER_REV = 537.7;
    final double TICKS_PER_MM = (ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI));
    final double TRACK_WIDTH_MM = 404;

    // Strafing is less efficient (wheel geometry + scrub); tweak if needed (1.0–1.3 typical).
    final double STRAFE_EFFICIENCY = 1.10;

    int shotsToFire = 3;
    double robotRotationAngle = 45;

    // ============================
    // === TIMERS               ===
    // ============================
    private ElapsedTime shotTimer   = new ElapsedTime();
    private ElapsedTime feederTimer = new ElapsedTime();
    private ElapsedTime driveTimer  = new ElapsedTime();

    // ============================
    // === HARDWARE             ===
    // ============================
    // === CONFIG NAMES: replace these with your actual config names ===
    private static final String FL_NAME = "left_front_drive";
    private static final String FR_NAME = "right_front_drive";
    private static final String BL_NAME = "left_back_drive";
    private static final String BR_NAME = "right_back_drive";

    private DcMotor fl, fr, bl, br;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    // ============================
    // === STATE MACHINES       ===
    // ============================
    private enum LaunchState { IDLE, PREPARE, LAUNCH }
    private LaunchState launchState;

    private enum AutonomousState {
        DRIVING_BACK,
        LAUNCH,
        WAIT_FOR_LAUNCH,
        DRIVING_AWAY_FROM_GOAL,
        ROTATING,
        DRIVING_OFF_LINE,
        // Example: add a strafe step if desired
        // STRAFE_TO_PARK,
        COMPLETE
    }
    private AutonomousState autonomousState;

    private enum Alliance { RED, BLUE }
    private Alliance alliance = Alliance.RED;

    @Override
    public void init() {
        autonomousState = AutonomousState.DRIVING_BACK;
        launchState = LaunchState.IDLE;

        // === Map hardware ===
        fl = hardwareMap.get(DcMotor.class, FL_NAME);
        fr = hardwareMap.get(DcMotor.class, FR_NAME);
        bl = hardwareMap.get(DcMotor.class, BL_NAME);
        br = hardwareMap.get(DcMotor.class, BR_NAME);

        launcher = hardwareMap.get(DcMotorEx.class,"launcher");
        leftFeeder  = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        // === Motor directions ===
        // Typical mecanum: reverse left side or right side (pick one) so forward stick = forward.
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        // === Reset + brake ===
        resetDriveEncoders();
        setDriveZeroPowerBrake();

        launcher.setZeroPowerBehavior(BRAKE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300,0,0,10));

        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized (Mecanum)");
    }

    @Override
    public void init_loop() {
        rightFeeder.setPower(0);
        leftFeeder.setPower(0);

        if (gamepad1.b) {
            alliance = Alliance.RED;
        } else if (gamepad1.x) {
            alliance = Alliance.BLUE;
        }

        telemetry.addData("Press X", "for BLUE");
        telemetry.addData("Press B", "for RED");
        telemetry.addData("Selected Alliance", alliance);
    }

    @Override
    public void start() { }

    @Override
     public void loop() {
        switch(autonomousState){
            case DRIVING_AWAY_FROM_GOAL:
                if (driveLinear(DRIVE_SPEED, -4, DistanceUnit.INCH, 1)) {
                    resetDriveEncoders();
                    autonomousState = AutonomousState.ROTATING;
                }
                break;

            case ROTATING:
                robotRotationAngle = (alliance == Alliance.RED) ? 90 : -90;
                if (rotate(ROTATE_SPEED, robotRotationAngle, AngleUnit.DEGREES, 1)) {
                    resetDriveEncoders();
                    autonomousState = AutonomousState.DRIVING_OFF_LINE;
                }
                break;

            case DRIVING_OFF_LINE:
                if (driveLinear(DRIVE_SPEED, -36, DistanceUnit.INCH, 1)) {
                    // Example: you could strafe to the side to park:
                    // resetDriveEncoders();
                    // autonomousState = AutonomousState.STRAFE_TO_PARK;
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;

            /*
            case STRAFE_TO_PARK:
                // Positive distance = strafe right, negative = strafe left
                if (strafe(DRIVE_SPEED, 12, DistanceUnit.INCH, 1)) {
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;
            */

            case COMPLETE:
                // Do nothing
                break;
        }

        telemetry.addData("AutoState", autonomousState);
        telemetry.addData("LauncherState", launchState);
        telemetry.addData("Drive CurrPos",
                "FL(%d) FR(%d) BL(%d) BR(%d)",
                fl.getCurrentPosition(), fr.getCurrentPosition(), bl.getCurrentPosition(), br.getCurrentPosition());
        telemetry.addData("Drive TgtPos",
                "FL(%d) FR(%d) BL(%d) BR(%d)",
                fl.getTargetPosition(), fr.getTargetPosition(), bl.getTargetPosition(), br.getTargetPosition());
        telemetry.update();
    }

    @Override
    public void stop() { }

    // ============================
    // === LAUNCHER STATE MACH  ===
    // ============================
    boolean launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.PREPARE;
                    shotTimer.reset();
                }
                break;
            case PREPARE:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    launchState = LaunchState.LAUNCH;
                    leftFeeder.setPower(1);
                    rightFeeder.setPower(1);
                    feederTimer.reset();
                }
                break;
            case LAUNCH:
                if (feederTimer.seconds() > FEED_TIME) {
                    leftFeeder.setPower(0);
                    rightFeeder.setPower(0);

                    if (shotTimer.seconds() > TIME_BETWEEN_SHOTS) {
                        launchState = LaunchState.IDLE;
                        return true;
                    }
                }
                break;
        }
        return false;
    }

    // ============================
    // === DRIVE HELPERS        ===
    // ============================
    private void resetDriveEncoders() {
        for (DcMotor m : new DcMotor[]{fl, fr, bl, br}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setPower(0);
        }
    }

    private void setDriveZeroPowerBrake() {
        for (DcMotor m : new DcMotor[]{fl, fr, bl, br}) {
            m.setZeroPowerBehavior(BRAKE);
        }
    }

    /**
     * Linear drive forward/back using RUN_TO_POSITION on all four motors.
     * @return true once within tolerance for holdSeconds.
     */
    boolean driveLinear(double speed, double distance, DistanceUnit distanceUnit, double holdSeconds) {
        final double TOLERANCE_MM = 20;

        double targetTicks = distanceUnit.toMm(distance) * TICKS_PER_MM;

        int flTgt = (int) targetTicks;
        int frTgt = (int) targetTicks;
        int blTgt = (int) targetTicks;
        int brTgt = (int) targetTicks;

        fl.setTargetPosition(flTgt);
        fr.setTargetPosition(frTgt);
        bl.setTargetPosition(blTgt);
        br.setTargetPosition(brTgt);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fl.setPower(speed);
        fr.setPower(speed);
        bl.setPower(speed);
        br.setPower(speed);

        // If any wheel not within tolerance, reset timer
        double tolTicks = TOLERANCE_MM * TICKS_PER_MM;
        boolean allClose =
                Math.abs(flTgt - fl.getCurrentPosition()) <= tolTicks &&
                Math.abs(frTgt - fr.getCurrentPosition()) <= tolTicks &&
                Math.abs(blTgt - bl.getCurrentPosition()) <= tolTicks &&
                Math.abs(brTgt - br.getCurrentPosition()) <= tolTicks;

        if (!allClose) driveTimer.reset();
        return driveTimer.seconds() > holdSeconds;
    }

    /**
     * Mecanum strafe left/right with RUN_TO_POSITION.
     * Positive distance = right strafe, negative = left.
     */
    boolean strafe(double speed, double distance, DistanceUnit distanceUnit, double holdSeconds) {
        final double TOLERANCE_MM = 10;

        // Apply efficiency factor so requested mm → slightly larger ticks to account for scrub.
        double mm = distanceUnit.toMm(distance) * STRAFE_EFFICIENCY;
        int flTgt = (int) ( mm * TICKS_PER_MM);  // +
        int frTgt = (int) (-mm * TICKS_PER_MM);  // -
        int blTgt = (int) (-mm * TICKS_PER_MM);  // -
        int brTgt = (int) ( mm * TICKS_PER_MM);  // +

        fl.setTargetPosition(flTgt);
        fr.setTargetPosition(frTgt);
        bl.setTargetPosition(blTgt);
        br.setTargetPosition(brTgt);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fl.setPower(speed);
        fr.setPower(speed);
        bl.setPower(speed);
        br.setPower(speed);

        double tolTicks = TOLERANCE_MM * TICKS_PER_MM;
        boolean allClose =
                Math.abs(flTgt - fl.getCurrentPosition()) <= tolTicks &&
                Math.abs(frTgt - fr.getCurrentPosition()) <= tolTicks &&
                Math.abs(blTgt - bl.getCurrentPosition()) <= tolTicks &&
                Math.abs(brTgt - br.getCurrentPosition()) <= tolTicks;

        if (!allClose) driveTimer.reset();
        return driveTimer.seconds() > holdSeconds;
    }

    /**
     * In-place rotation using RUN_TO_POSITION on all four motors.
     * Positive angle rotates CCW (left side backward, right side forward).
     */
    boolean rotate(double speed, double angle, AngleUnit angleUnit, double holdSeconds) {
        final double TOLERANCE_MM = 10;

        double targetMm = angleUnit.toRadians(angle) * (TRACK_WIDTH_MM / 2.0);
        int leftTicks  = (int) (-(targetMm * TICKS_PER_MM));
        int rightTicks = (int) (  (targetMm * TICKS_PER_MM));

        int flTgt = leftTicks;
        int blTgt = leftTicks;
        int frTgt = rightTicks;
        int brTgt = rightTicks;

        fl.setTargetPosition(flTgt);
        fr.setTargetPosition(frTgt);
        bl.setTargetPosition(blTgt);
        br.setTargetPosition(brTgt);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fl.setPower(speed);
        fr.setPower(speed);
        bl.setPower(speed);
        br.setPower(speed);

        double tolTicks = TOLERANCE_MM * TICKS_PER_MM;
        boolean allClose =
                Math.abs(flTgt - fl.getCurrentPosition()) <= tolTicks &&
                Math.abs(frTgt - fr.getCurrentPosition()) <= tolTicks &&
                Math.abs(blTgt - bl.getCurrentPosition()) <= tolTicks &&
                Math.abs(brTgt - br.getCurrentPosition()) <= tolTicks;

        if (!allClose) driveTimer.reset();
        return driveTimer.seconds() > holdSeconds;
    }
}
