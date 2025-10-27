package org.firstinspires.ftc.team18443;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

    // Hardware Map Reference
    private final HardwareMap hardwareMap;

    // Constructor
    public RobotHardware(HardwareMap hwMap) {
        this.hardwareMap = hwMap;
    }

// -------------------------------------------------------------------------------------------------
//    Initialization Setup
// -------------------------------------------------------------------------------------------------
    
    public void init() {
        // Map motors by configuration names
        frontLeft  = hardwareMap.get(DcMotorEx.class, "fl");
        frontRight = hardwareMap.get(DcMotorEx.class, "fr");
        backLeft   = hardwareMap.get(DcMotorEx.class, "bl");
        backRight  = hardwareMap.get(DcMotorEx.class, "br");
        flyWheel   = hardwareMap.get(DcMotorEx.class, "launcher");
        conveyer   = hardwareMap.get(DcMotorEx.class, "belt");
        intake     = hardwareMap.get(DcMotorEx.class, "intake");

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
        flipper = hardwareMap.get(Servo.class, "flipper");

        // Odometry setup (future)
        // odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        // odo.resetPosAndIMU();  // Resets odometry and IMU to starting pose

        // IMU setup
        imu = hardwareMap.get(IMU.class, "imu");
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
     * Use to make the robot move forward a specified distance at a given speed
     *
     * @param inches The distance to move forward, in inches
     * @param speed  The speed to travel (range: 0.0 to 1.0)
     */
    public void forward(double inches, double speed){
        moveToPosition(inches, speed);
    }

    /**
     * Use to make the robot move backward a specified distance at a given speed
     *
     * @param inches The distance to move backward, in inches
     * @param speed  The speed to travel (range: 0.0 to 1.0)
     */
    public void backward(double inches, double speed){
        moveToPosition(-inches, speed);
    }

    /**
     * Use to make the robot rotate left by the specified number of degrees at a given speed
     *
     * @param degrees The angle to rotate left, in degrees
     * @param speed   The speed of rotation (range: 0.0 to 1.0)
     */
    public void turnLeft(double degrees, double speed){
        turnWithGyro(degrees, -speed);
    }

    /**
     * Use to make the robot rotate right by the specified number of degrees at a given speed
     *
     * @param degrees The angle to rotate right, in degrees
     * @param speed   The speed of rotation (range: 0.0 to 1.0)
     */
    public void turnRight(double degrees, double speed){
        turnWithGyro(degrees, speed);
    }

    /**
     * Use to make the robot strafe left by the specified distance at a given speed
     *
     * @param inches The distance to strafe left, in inches
     * @param speed  The speed of strafing (range: 0.0 to 1.0)
     */
    public void strafeLeft(double inches, double speed){
        strafeToPosition(-inches, speed);
    }

    /**
     * Use to make the robot strafe right by the specified distance at a given speed
     *
     * @param inches The distance to strafe right, in inches
     * @param speed  The speed of strafing (range: 0.0 to 1.0)
     */
    public void strafeRight(double inches, double speed){
        strafeToPosition(inches, speed);
    }

    /*
     * This function's purpose is simply to drive forward or backward.
     * To drive backward, simply make the inches input negative
     */
    public void moveToPosition(double inches, double speed){
        int move = (int)(Math.round(inches * COUNTS_PER_INCH * strafeComp));
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
               backRight.isBusy()){

        }

        // Stop the motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void turnWithGyro(double degrees, double speedDirection){
        // Create an object to receive the IMU angles
        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();

        double yaw = robotOrientation.getYaw(AngleUnit.DEGREES);

    }

    /*
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
               backRight.isBusy()){

        }

        // Stop the motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
