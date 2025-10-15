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
//      Hardware abstraction class for the robot. Initializes and manages
//      motors, sensors, and provides drive control methods
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

// ----------------------------------------------------------------------------
//    Hardware Device Definitions
// ----------------------------------------------------------------------------

    // Drive Motors
    public DcMotorEx leftFront, leftRear, rightFront, rightRear;

    // Mechanism Motors
    public DcMotorEx launcher, belt;

    // Servos (add as needed)
    // public Servo claw, grabber, leftOuttake, rightOuttake, wrist, leftIntake, rightIntake;

    // Odometry (future addition)
    // public GoBildaPinpointDriver odo;   // <— Odometry pod driver (future integration)
    // To use: instantiate with hardwareMap.get(GoBildaPinpointDriver.class, "odo");
    // Then read pose data via odo.getPosition() or odo.getHeadingRadians().

    // Sensors
    public IMU imu;

    // Hardware Map Reference
    private final HardwareMap hardwareMap;

    // Drive tunables
    public double strafeComp = 1.10; // Strafe compensation factor (empirical)

    // Constructor
    public RobotHardware(HardwareMap hwMap) {
        this.hardwareMap = hwMap;
    }

// ----------------------------------------------------------------------------
//    Initialization Setup
// ----------------------------------------------------------------------------
    
    public void init() {
        // Motors
        leftFront  = hardwareMap.get(DcMotorEx.class, "FL");
        leftRear   = hardwareMap.get(DcMotorEx.class, "BL");
        rightFront = hardwareMap.get(DcMotorEx.class, "FR");
        rightRear  = hardwareMap.get(DcMotorEx.class, "BR");
        launcher   = hardwareMap.get(DcMotorEx.class, "launcher");
        belt       = hardwareMap.get(DcMotorEx.class, "belt");

        // Reverse one side of the motors for mecanum drive to ensure consistent forward movement
        // If the robot drives backwards, reverse the other side instead
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        // Set zero power behavior
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        belt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Optionally apply to drivetrain for better control:
        // leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

// ----------------------------------------------------------------------------
//    Methods for Driving & Orientation
// ----------------------------------------------------------------------------

    /** Resets the IMU's yaw angle to zero */
    public void resetYaw() {
        if (imu != null) imu.resetYaw();
    }

    /** Gets the robot's heading (yaw) in radians (-π to π) from the IMU */
    public double getHeadingRad() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    /** Gets the robot's heading (yaw) in degrees (-180° to 180°) from the IMU */
    public double getHeadingDeg() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    /** Set normalized drive motor powers
     * Parameters:
     *   fl: front left power
     *   fr: front right power
     *   bl: back left power
     *   br: back right power
    */
    public void setDrivePowers(double fl, double fr, double bl, double br) {
        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        leftFront.setPower(fl / max);
        rightFront.setPower(fr / max);
        leftRear.setPower(bl / max);
        rightRear.setPower(br / max);
    }

    /** Robot-centric mecanum drive mode (controls relative to robot's orientation).
     * Parameters:
     *   x: strafe (left/right)
     *   y: forward/backward
     *   rx: rotation (clockwise/counterclockwise)
    */
    public void driveRobotCentric(double x, double y, double rx) {
        double fl = y + x + rx;
        double bl = y - x + rx;
        double fr = y - x - rx;
        double br = y + x - rx;
        setDrivePowers(fl, fr, bl, br);
    }

    /** Field-centric mecanum drive mode (controls relative to field orientation)
     * Uses IMU yaw angle for heading compensation
     * Parameters:
     *   x: strafe (left/right)
     *   y: forward/backward
     *   rx: rotation (clockwise/counterclockwise)
    */
    public void driveFieldCentric(double x, double y, double rx) {
        x *= strafeComp;
        double heading = getHeadingRad();
        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
        driveRobotCentric(rotX, rotY, rx);
    }
}
