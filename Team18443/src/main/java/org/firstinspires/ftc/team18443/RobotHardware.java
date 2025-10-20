package org.firstinspires.ftc.team18443;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

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
    public DcMotorEx launcher, belt;

    // Servos for game-specific manipulators
    // public Servo claw, grabber, leftOuttake, rightOuttake, wrist, leftIntake, rightIntake;

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
        launcher   = hardwareMap.get(DcMotorEx.class, "launcher");
        belt       = hardwareMap.get(DcMotorEx.class, "belt");

        // Reverse one side of the motors for mecanum drive to ensure consistent forward movement
        // If the robot drives backwards, reverse the other side instead
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Set zero power behavior for motors to BRAKE for holding position
        // Optionally apply BRAKE to the drivetrain for better control
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        belt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
}
