package org.firstinspires.ftc.team18443;

// ============================================================================
//  RobotHardware.java                                       GalacticLions2526
// ============================================================================
//
//   Description:
//      Hardware abstraction layer for the robot. This class defines and
//      initializes all hardware components (motors, servos, sensors, etc.),
//      provides an API for drive and mechanism control, and implements
//      motion/utility methods for both TeleOp and Autonomous modes
//
//   Usage:
//      - Instantiate RobotHardware with a LinearOpMode reference
//      - Call init() in runOpMode() before accessing any hardware
//
// ============================================================================
// This program is released under the BSD-3-Clause-Clear License
// See LICENSE file in root of this repository
// ============================================================================

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class RobotHardware {

// -------------------------------------------------------------------------------------------------
//    Hardware Device Definitions
// -------------------------------------------------------------------------------------------------
//
//    All physical devices used on the robot are declared here and initialized
//    in init(). Each is referenced throughout TeleOp and Autonomous modes

    // Drive motors for the mecanum drive base
    public DcMotorEx frontLeft, frontRight, backLeft, backRight;

    // Mechanism motors for game-specific mechanisms
    public DcMotorEx intakeWheels, intakeConveyor, flywheel;

    // Servos for game element manipulators
    public Servo flipper;

    // Odometry (future addition)
    // public GoBildaPinpointDriver odo;   // <— Odometry pod driver (future integration)
    // To use: instantiate with hardwareMap.get(GoBildaPinpointDriver.class, "odo");
    // Then read pose data via odo.getPosition() or odo.getHeadingRadians().

    // Inertial Measurement Unit (IMU) for orientation and field-centric control
    public IMU imu;

// -------------------------------------------------------------------------------------------------
//    Tunable Constants
// -------------------------------------------------------------------------------------------------
//
//    Encoder conversion factors, motion scaling values, and mechanism presets.
//    Adjust these for calibration, tuning, or improved accuracy

    // Encoder and motion parameters
    static final double COUNTS_PER_ROTATION   = 537.7; // GoBilda 312 RPM Yellow Jacket
    static final double WHEEL_DIAMETER_INCHES = 3.779; // GoBilda mecanum wheels
    static final double DRIVE_GEAR_REDUCTION  = 1.0;   // 1:1 gear ratio
    static final double COUNTS_PER_INCH       = (COUNTS_PER_ROTATION * DRIVE_GEAR_REDUCTION) /
                                                (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double JOYSTICK_DEADZONE     = 0.1;
    // Mechanism constants
    public final double FLYWHEEL_POWER_HIGH   = 0.85;
    public final double FLYWHEEL_POWER_MEDIUM = 0.55;
    public final double FLYWHEEL_POWER_OFF    = 0.0;
    public final double FLIPPER_UP            = 0.66;
    public final double FLIPPER_DOWN          = 0.33;
    public double strafeComp = 1.10; // Strafe compensation factor (empirical)

    // LinearOpMode Library Reference
    private final LinearOpMode opMode;

    // Constructor
    public RobotHardware(LinearOpMode opMode) {
        this.opMode = opMode;
    }

// -------------------------------------------------------------------------------------------------
//    Initialization Setup
// -------------------------------------------------------------------------------------------------

    /**
     * Initializes all hardware devices and configures motor directions,
     * zero power behaviors, and IMU orientation
     * <p>
     * Must be called once in {@code runOpMode()} before accessing hardware
     */
    public void init() {
        // Map motors by configuration names in the robot controller app
        frontLeft      = opMode.hardwareMap.get(DcMotorEx.class, "fl");
        frontRight     = opMode.hardwareMap.get(DcMotorEx.class, "fr");
        backLeft       = opMode.hardwareMap.get(DcMotorEx.class, "bl");
        backRight      = opMode.hardwareMap.get(DcMotorEx.class, "br");
        flywheel       = opMode.hardwareMap.get(DcMotorEx.class, "launcher");
        intakeConveyor = opMode.hardwareMap.get(DcMotorEx.class, "belt");
        intakeWheels   = opMode.hardwareMap.get(DcMotorEx.class, "intake");

        // Reverse left-side drive motors so positive power moves robot forward
        // Swap these if your robot's wiring or gearboxes are mirrored
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Apply BRAKE mode to mechanism motors for precise stopping
        // (Optional: enable BRAKE on drive motors for more controlled deceleration)
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeConveyor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeWheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Map servos by configuration names
        flipper = opMode.hardwareMap.get(Servo.class, "flipper");

        // Odometry setup (future)
        // odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        // odo.resetPosAndIMU();  // Resets odometry and IMU to starting pose

        // Configure IMU mounting orientation to match physical mounting;
        // incorrect orientation will affect field-centric driving
        imu = opMode.hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                // Default assumption: logo up, USB port facing forward
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);

        opMode.telemetry.addData(">", "Hardware Initialized");
        opMode.telemetry.update();
    }

// -------------------------------------------------------------------------------------------------
//    IMU and Orientation Helpers
// -------------------------------------------------------------------------------------------------

    /**
     * Resets the IMU's yaw (heading) angle to zero
     */
    public void resetYaw() {
        if (imu != null) imu.resetYaw();
    }

    /**
     * Gets the robot's heading (yaw) in radians (-π to π) from the IMU
     *
     * @return The robot's yaw angle in radians, range [-π, π]
     */
    public double getHeadingRad() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    /**
     * Gets the robot's heading (yaw) in degrees (-180° to 180°) from the IMU
     *
     * @return The robot's yaw angle in degrees, range [-180, 180]
     */
    public double getHeadingDeg() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

// -------------------------------------------------------------------------------------------------
//    TeleOp Drive Methods
// -------------------------------------------------------------------------------------------------

    /**
     * Sets raw drive motor powers with normalization so values remain within [-1, 1]
     *
     * @param fl Power for front-left motor
     * @param fr Power for front-right motor
     * @param bl Power for back-left motor
     * @param br Power for back-right motor
     */
    public void setDrivePowers(double fl, double fr, double bl, double br) {
        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);
    }

    /**
     * Applies a deadzone to joystick input to ignore small movements near zero
     *
     * @param input The raw joystick input value
     * @return The adjusted joystick value after applying the deadzone and scaling
     */
    public double applyJoystickDeadzone(double input) {
        if (Math.abs(input) < JOYSTICK_DEADZONE) {
            return 0.0;
        }
        else {
            if (input > 0.0) {
                return (input - JOYSTICK_DEADZONE) / (1.0 - JOYSTICK_DEADZONE);
            }
            else {
                return (input + JOYSTICK_DEADZONE) / (1.0 - JOYSTICK_DEADZONE);
            }
        }
    }

    /**
     * Robot-centric mecanum drive (controls relative to robot's orientation)
     *
     * @param x  Strafe (left/right)
     * @param y  Forward/backward
     * @param rx Rotation (clockwise/counterclockwise)
     */
    public void driveRobotCentric(double x, double y, double rx) {
        double fl = y + x + rx;
        double bl = y - x + rx;
        double fr = y - x - rx;
        double br = y + x - rx;
        setDrivePowers(fl, fr, bl, br);
    }

    /**
     * Field-centric mecanum drive (controls relative to field orientation).
     * Uses the IMU yaw angle to compensate for the robot's heading
     *
     * @param x  Strafe (left/right)
     * @param y  Forward/backward
     * @param rx Rotation (clockwise/counterclockwise)
     */
    public void driveFieldCentric(double x, double y, double rx) {
        x *= strafeComp;
        double heading = getHeadingRad();
        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
        driveRobotCentric(rotX, rotY, rx);
    }

// -------------------------------------------------------------------------------------------------
//     Autonomous Movement and Mechanism Control Methods
// -------------------------------------------------------------------------------------------------
//
//     Methods for Autonomous OpModes. Execute blocking motion sequences
//     until completion before returning

    /**
     * Moves the robot forward or backward a specified distance using encoder targets
     */
    public void moveToPosition(double inches, double speed){
        // Determine new target position and pass to motor controller
        int move = (int)(Math.round(inches * COUNTS_PER_INCH));
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + move);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + move);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + move);
        backRight.setTargetPosition(backRight.getCurrentPosition() + move);

        // Turn On RUN_TO_POSITION
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double kp = 0.05;

        // Loop until all motors have reached their targets
        while (opMode.opModeIsActive() && (frontLeft.isBusy() || frontRight.isBusy()
                || backLeft.isBusy() || backRight.isBusy())) {
            // Calculate average positions
            int leftPos  = (frontLeft.getCurrentPosition() + backLeft.getCurrentPosition()) / 2;
            int rightPos = (frontRight.getCurrentPosition() + backRight.getCurrentPosition()) / 2;
            int error = leftPos - rightPos;

            // Apply proportional correction to motor powers
            double leftPower = speed - (error * kp);
            double rightPower = speed + (error * kp);

            // Clamp powers to prevent stalling or exceeding max speed
            leftPower = Math.max(0.1, Math.abs(leftPower)) * Math.signum(leftPower);
            rightPower = Math.max(0.1, Math.abs(rightPower)) * Math.signum(rightPower);

            setDrivePowers(leftPower, rightPower, leftPower, rightPower);

            opMode.telemetry.addData("Drive", "Moving...");
            opMode.telemetry.update();

            opMode.sleep(20);
        }

        // Stop all motion
        setDrivePowers(0, 0, 0, 0);
    }

    /**
     * Rotates the robot by a specified angle using IMU-based yaw measurements
     * <p>
     * This method performs a two-phase rotation for improved accuracy:
     * <ul>
     *   <li>Phase 1: Coarse rotation to reach within ~10° of the target angle</li>
     *   <li>Phase 2: Fine rotation for higher precision (±5° tolerance)</li>
     * </ul>
     *
     * @param degrees -
     * @param speedDirection -
     */
    public void turnWithGyro(double degrees, double speedDirection) {
        // Create an object to receive the IMU angles
        YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
        double yaw = robotOrientation.getYaw(AngleUnit.DEGREES);

        // Determine first and second target angles
        double first, second;

        if (speedDirection > 0) { // turning right
            first = (degrees > 10) ? (degrees - 10) + devertify(yaw) : devertify(yaw);
            second = degrees + devertify(yaw);
        }
        else { // turning left
            first = (degrees > 10) ? -(degrees - 10) + devertify(yaw) : devertify(yaw);
            second = -degrees + devertify(yaw);
        }

        // Define angle tolerances
        double firstA = convertify(first - 5);
        double firstB = convertify(first + 5);
        double secondA = convertify(second - 5);
        double secondB = convertify(second + 5);

        turnWithEncoder(speedDirection);
        // Phase 1: Coarse turn
        while (opMode.opModeIsActive()) {
            yaw = convertify(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            boolean inRange = (Math.abs(firstA - firstB) < 11) ? (firstA < yaw && yaw < firstB)
                              : ((firstA < yaw && yaw <= 180) || (-180 <= yaw && yaw < firstB));
            if (inRange) break;
        }

        turnWithEncoder(speedDirection / 3);
        // Phase 2: Fine adjustment
        while (opMode.opModeIsActive()) {
            yaw = convertify(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            boolean inRange = (Math.abs(secondA - secondB) < 11) ? (secondA < yaw && yaw < secondB)
                              : ((secondA < yaw && yaw <= 180) || (-180 <= yaw && yaw < secondB));
            if (inRange) break;
        }

        // Stop all motion
        setDrivePowers(0, 0, 0, 0);

        // Turn off RUN_USING_ENCODER
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /*
     * Angle normalization utilities:
     *  - devertify(): converts negative angles into [0°, 360°]
     *  - convertify(): constrains angles to [-180°, 180°] for comparison
     */
    public double devertify(double degrees){
        return (degrees < 0) ? degrees + 360 : degrees;
    }
    public double convertify(double degrees){
        if(degrees > 360) {
            degrees -= 360;
        }
        else if(degrees < -180) {
            degrees += 360;
        }
        else if(degrees > 179) {
            degrees = -(360 - degrees);
        }
        return degrees;
    }
    public void turnWithEncoder(double input){
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //
        frontLeft.setPower(input);
        frontRight.setPower(-input);
        backLeft.setPower(input);
        backRight.setPower(-input);
    }

    /**
     * Strafes robot left or right using encoder targets.
     * Negative input for inches results in left strafing
     */
    public void strafeToPosition(double inches, double speed){
        // Determine new target position and pass to motor controller
        int move = (int)(Math.round(inches * COUNTS_PER_INCH * strafeComp));
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + move);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - move);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() - move);
        backRight.setTargetPosition(backRight.getCurrentPosition() + move);

        // Turn On RUN_TO_POSITION
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setDrivePowers(speed, speed, speed, speed);

        // Loop until all motors have reached their targets
        while (opMode.opModeIsActive() && (frontLeft.isBusy() || frontRight.isBusy()
                || backLeft.isBusy() || backRight.isBusy())) {
            opMode.telemetry.addData("Drive", "Strafing...");
            opMode.telemetry.update();

            opMode.sleep(20);
        }

        // Stop all motion
        setDrivePowers(0, 0, 0, 0);
    }

    /**
     * Moves the intake system by a specified number of encoder ticks
     */
    public void setIntake(int ticks) {
        // Set the ticks and motor mode
        intakeConveyor.setTargetPosition(intakeConveyor.getCurrentPosition() + ticks);
        intakeWheels.setTargetPosition(intakeWheels.getCurrentPosition() + ticks);
        intakeConveyor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeWheels.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intakeConveyor.setPower(1.0);
        intakeWheels.setPower(1.0);

        // Loop until all motors have reached their targets
        while (opMode.opModeIsActive() && (intakeConveyor.isBusy() ||
                intakeWheels.isBusy())) {
            opMode.telemetry.addData("Intake", "Moving...");
            opMode.telemetry.update();

            opMode.sleep(20);
        }

        // Stop all motion
        intakeConveyor.setPower(0.0);
        intakeWheels.setPower(0.0);
    }

    /**
     * Sets the flywheel to a predefined HIGH, MED, or OFF state
     */
    public void setFlywheelState(flywheelState state) {
        switch (state) {
            case HIGH:
                flywheel.setPower(FLYWHEEL_POWER_HIGH);
                break;

            case MEDIUM:
                flywheel.setPower(FLYWHEEL_POWER_MEDIUM);
                break;

            case OFF:
                flywheel.setPower(FLYWHEEL_POWER_OFF);
                break;
        }
    }
    public enum flywheelState {
        HIGH, MEDIUM, OFF
    }

    /**
     * Sets flipper position based on predefined UP/DOWN states
     */
    public void setFlipperState(flipperState state) {
        switch (state) {
            case UP:
                flipper.setPosition(FLIPPER_UP);
                break;

            case DOWN:
                flipper.setPosition(FLIPPER_DOWN);
                break;
        }
    }
    public enum flipperState {
        UP, DOWN
    }
}