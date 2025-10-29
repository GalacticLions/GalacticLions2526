package org.firstinspires.ftc.team18443;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

// ****************************************************************************
//  RobotHardware.java                                       GalacticLions2526
// ****************************************************************************
//   Description:
//      Hardware abstraction class for the robot. This class is responsible for
//      defining and initializing all hardware devices (motors, servos, etc.),
//      providing a small API for common robot drive modes, and holding utility
//      methods related to driving and orientation control.
//
//   Usage:
//      - Instantiate RobotHardware with a HardwareMap in your OpMode
//      - Call init() to initialize all hardware devices
//
// ****************************************************************************
// This program is released under the BSD-3-Clause-Clear License
// See LICENSE file in root of this repository
// ****************************************************************************

public class RobotHardware {

// -------------------------------------------------------------------------------------------------
//    Hardware Device & Variable Definitions
// -------------------------------------------------------------------------------------------------

    // Drive motors for drive base
    public DcMotorEx frontLeft, backLeft, frontRight, backRight;

    // Mechanism motors for game-specific mechanisms
    public DcMotorEx intake, flyWheel, conveyer;

    // Servos for game-specific manipulators
    public Servo flipper;

    // Odometry (future addition)
    // public GoBildaPinpointDriver odo;   // <— Odometry pod driver (future integration)
    // To use: instantiate with hardwareMap.get(GoBildaPinpointDriver.class, "odo");
    // Then read pose data via odo.getPosition() or odo.getHeadingRadians().

    // Inertial Measurement Unit (IMU) for orientation sensing
    public IMU imu;

    // Tunable Variables
    static final double COUNTS_PER_ROTATION   = 537.7; // GoBILDA 312 RPM Yellow Jacket
    static final double WHEEL_DIAMETER_INCHES = 3.779; // GoBilda mecanum wheels
    static final double DRIVE_GEAR_REDUCTION  = 1.0; // No External Gearing
    static final double COUNTS_PER_INCH       = (COUNTS_PER_ROTATION * DRIVE_GEAR_REDUCTION) /
                                                (WHEEL_DIAMETER_INCHES * Math.PI);
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
    
    public void init() {
        // Map motors by configuration names
        frontLeft  = opMode.hardwareMap.get(DcMotorEx.class, "fl");
        frontRight = opMode.hardwareMap.get(DcMotorEx.class, "fr");
        backLeft   = opMode.hardwareMap.get(DcMotorEx.class, "bl");
        backRight  = opMode.hardwareMap.get(DcMotorEx.class, "br");
        flyWheel   = opMode.hardwareMap.get(DcMotorEx.class, "launcher");
        conveyer   = opMode.hardwareMap.get(DcMotorEx.class, "belt");
        intake     = opMode.hardwareMap.get(DcMotorEx.class, "intake");

        // Reverse one side of the motors for mecanum drive to ensure consistent forward movement
        // If the robot drives backwards, reverse the other side instead
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Set zero power behavior for motors to BRAKE for holding position
        // Optionally apply BRAKE to the drivetrain for better control
        flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        conveyer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Map servos by configuration names
        flipper = opMode.hardwareMap.get(Servo.class, "flipper");

        // Odometry setup (future)
        // odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        // odo.resetPosAndIMU();  // Resets odometry and IMU to starting pose

        // IMU setup
        imu = opMode.hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // By default, IMU assumes REV Hub is mounted with logo up and USB port facing forward
        imu.initialize(parameters);
    }

// -------------------------------------------------------------------------------------------------
//    Methods for Driving & Orientation
// -------------------------------------------------------------------------------------------------

    // TeleOp

    /**
     * Resets the IMU's yaw angle to zero
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

    /**
     * Sets the drive motor powers, normalizing values to keep them within [-1, 1]
     *
     * @param fl The front left motor power
     * @param fr The front right motor power
     * @param bl The back left motor power
     * @param br The back right motor power
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

    // Autonomous

    /**
     * This function's purpose is simply to drive forward or backward.
     * To drive backward, simply make the inches input negative
     */
    public void moveToPosition(double inches, double speed){
        int move = (int)(Math.round(inches * COUNTS_PER_INCH));
        // Set the target position and motor mode
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + move);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + move);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + move);
        backRight.setTargetPosition(backRight.getCurrentPosition() + move);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        // Wait until the motors reach their target position
        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() &&
               backRight.isBusy()) {
            opMode.telemetry.addData("Busy...", "");
            opMode.telemetry.update();
        }

        // Stop the motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    /**
     * This function uses the Hub IMU Integrated Gyro to turn a precise number of degrees (+/- 5).
     * Degrees should always be positive, make speedDirection negative to turn left.
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

        // Convert to normalized angles
        double firstA = convertify(first - 5);
        double firstB = convertify(first + 5);
        double secondA = convertify(second - 5);
        double secondB = convertify(second + 5);

        turnWithEncoder(speedDirection);
        // Wait until yaw is in first range
        while (opMode.opModeIsActive()) {
            yaw = convertify(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            boolean inRange = (Math.abs(firstA - firstB) < 11) ? (firstA < yaw && yaw < firstB)
                              : ((firstA < yaw && yaw <= 180) || (-180 <= yaw && yaw < firstB));
            if (inRange) break;
        }

        turnWithEncoder(speedDirection / 3);
        // Wait until yaw is in second range
        while (opMode.opModeIsActive()) {
            yaw = convertify(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            boolean inRange = (Math.abs(secondA - secondB) < 11) ? (secondA < yaw && yaw < secondB)
                              : ((secondA < yaw && yaw <= 180) || (-180 <= yaw && yaw < secondB));
            if (inRange) break;
        }

        // Stop the motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

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
     * These functions are used in the turnWithGyro function to ensure inputs
     * are interpreted properly and set the encoder mode and turn.
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
     * This function uses the encoders to strafe left or right.
     * Negative input for inches results in left strafing
     */
    public void strafeToPosition(double inches, double speed){
        int move = (int)(Math.round(inches * COUNTS_PER_INCH * strafeComp));
        // Set the target position and motor mode
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + move);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - move);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() - move);
        backRight.setTargetPosition(backRight.getCurrentPosition() + move);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        // Wait until the motors reach their target position
        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() &&
               backRight.isBusy()) {
            opMode.telemetry.addData("Busy...", "");
            opMode.telemetry.update();
        }

        // Stop the motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
