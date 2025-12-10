package org.firstinspires.ftc.team5898;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.team5898.Constants.CannonConstants;
import org.firstinspires.ftc.team5898.Constants.SlideConstants;
import org.firstinspires.ftc.team5898.LimelightUtils.VisualServoing;

@TeleOp(name="Alpha TeleOP PID Test", group="TeleOP")
public class Alpha_TeleOP_PID extends OpMode {
    Limelight3A limelight;
    DcMotor frontLeft, frontRight, backLeft, backRight, leftSlide, rightSlide;
    DcMotor topLauncher, bottomLauncher;
    VisualServoing visualServoing;
    CRServo frontServo, backLeftServo, backRightServo;
    IMU imu;
    Integer Offset,  errorThreshold, slideLeftTarget, slideRightTarget,slideLeftPosition, slideRightPosition,leftError,rightError;
    Double slidePower, intakePower;

    Double LAUNCH_VELOCITY, LAUNCH_VELOCITY_ALT;


    @Override
    public void init() {
        //Constants Init
        //Cannon Constants
        intakePower = CannonConstants.IntakePower;
        Offset = SlideConstants.Offset;
        //Slide Constants
        slidePower = SlideConstants.movePower;
        errorThreshold = SlideConstants.ErrorThreshold;

        // Set target velocities in Ticks Per Second. These are example values and will need tuning.
        LAUNCH_VELOCITY = CannonConstants.LAUNCH_VELOCITY;
        LAUNCH_VELOCITY_ALT = CannonConstants.LAUNCH_VELOCITY_ALT;


        //Limelight Init
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        telemetry.setMsTransmissionInterval(11);
        limelight.setPollRateHz(90);
        limelight.start();

        //Motors Init
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");
        leftSlide = hardwareMap.get(DcMotor.class, "LS");
        rightSlide = hardwareMap.get(DcMotor.class, "RS");
        topLauncher = hardwareMap.get(DcMotor.class, "TLaunch");
        bottomLauncher = hardwareMap.get(DcMotor.class, "BLaunch");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        topLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
        bottomLauncher.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set motors to use encoders for velocity control. This is a crucial step.
        topLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Improved PIDF coefficients for more stable velocity control
        // P: Proportional gain - increased for faster response
        // I: Integral gain - added to eliminate steady-state error
        // D: Derivative gain - added to reduce overshoot and oscillation
        // F: Feed-forward gain - tuned for velocity control
        // Tune these values based on your motor's behavior:
        // - If oscillating: decrease P, increase D
        // - If slow to reach target: increase P, increase F
        // - If steady-state error: increase I (start small, like 0.1-0.5)

        //TODO: Tune PID for new Robot

        // Set zero power behavior to FLOAT for launchers (reduces resistance)
        topLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bottomLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideLeftTarget = 0;
        slideRightTarget = 0;
        frontServo = hardwareMap.get(CRServo.class, "IntServo");
        backLeftServo = hardwareMap.get(CRServo.class, "LServo");
        backRightServo = hardwareMap.get(CRServo.class, "RServo");

        //TODO: Directions could be flipped for code below
        frontServo.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftServo.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightServo.setDirection(DcMotorSimple.Direction.FORWARD);


        //IMU Init for Field-Centric
        //TODO: Double check for new robot
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);


        //Visual Servoing Init
        visualServoing = new VisualServoing(limelight, frontLeft, frontRight, backRight, backLeft, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        //Slide Control System
        slideLeftPosition = leftSlide.getCurrentPosition();
        slideRightPosition = rightSlide.getCurrentPosition();

        leftError = Math.abs(slideLeftTarget - slideLeftPosition);
        rightError = Math.abs(slideRightTarget - slideRightPosition) ;

        if (gamepad1.dpad_up) {
            slideRightTarget += 10;
            slideLeftTarget -= 10;
        }

        if (leftError >= 3) {
            leftSlide.setTargetPosition(slideLeftTarget);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setPower(slidePower);
        } else {
            leftSlide.setPower(0.0);
            leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (rightError >= 3) {
            rightSlide.setTargetPosition(slideRightTarget);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setPower(slidePower);
        } else {
            rightSlide.setPower(0.0);
            rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }



        //VisualServoing: gamepad1.a
        //Check if VisualServoing
        boolean isVisualServoing = gamepad1.a;
        if (isVisualServoing) {
            visualServoing.visualServo();
        }


        //Cannon Control - Now using setVelocity for closed-loop control
        if (gamepad2.left_stick_y > 0.3) {
            topLauncher.setPower(-0.6);
            bottomLauncher.setPower(0.6);
        } else if (gamepad2.left_stick_y < -0.3) {
            topLauncher.setPower(0.6);
            bottomLauncher.setPower(-0.6);
        } else {
            topLauncher.setPower(0);
            bottomLauncher.setPower(0);
        }

//        // Add telemetry to monitor launcher performance
//        telemetry.addData("Top Launcher Velocity", "Target: %.0f, Actual: %.0f",
//            gamepad2.left_stick_y > 0.3 ? -LAUNCH_VELOCITY_ALT : (gamepad2.left_stick_y < -0.3 ? LAUNCH_VELOCITY_ALT : 0.0),
//            topLauncher.getVelocity());
//        telemetry.addData("Bottom Launcher Velocity", "Target: %.0f, Actual: %.0f",
//            gamepad2.left_stick_y > 0.3 ? LAUNCH_VELOCITY : (gamepad2.left_stick_y < -0.3 ? -LAUNCH_VELOCITY : 0.0),
//            bottomLauncher.getVelocity());
//        telemetry.addData("Velocity Error", "Top: %.0f, Bottom: %.0f",
//            Math.abs(topLauncher.getVelocity() - (gamepad2.left_stick_y > 0.3 ? -LAUNCH_VELOCITY_ALT : (gamepad2.left_stick_y < -0.3 ? LAUNCH_VELOCITY_ALT : 0.0))),
//            Math.abs(bottomLauncher.getVelocity() - (gamepad2.left_stick_y > 0.3 ? LAUNCH_VELOCITY : (gamepad2.left_stick_y < -0.3 ? -LAUNCH_VELOCITY : 0.0))));
//        telemetry.update();
//

        //TODO: Need to be tested
        if (gamepad2.right_stick_y > 0.3) {
            backLeftServo.setPower(intakePower);
            backRightServo.setPower(intakePower);
            frontServo.setPower(intakePower);
        } else if (gamepad2.right_stick_y < -0.3) {
            backLeftServo.setPower(-intakePower);
            backRightServo.setPower(-intakePower);
            frontServo.setPower(-intakePower);
        }else {
            backLeftServo.setPower(0);
            backRightServo.setPower(0);
            frontServo.setPower(0);
        }



        //===========================================

        // Field-Centric Drive Code (skip if Visual Servoing is active)
        if (!isVisualServoing) {
            double y = -gamepad1.left_stick_y; // Forward/Backward (reversed)
            double x = gamepad1.left_stick_x * 1.1; // Strafe Left/Right (counteract imperfect strafing)
            double rx = gamepad1.right_stick_x; // Rotation

            // Reset IMU yaw with guide button
            if (gamepad1.guide) {
                imu.resetYaw();
            }

            // Get robot heading from IMU
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate movement direction based on robot's heading
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Calculate motor powers
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;
            if (!gamepad1.left_bumper) {
                // Set motor powers (adjust multiplier as needed)
                frontLeft.setPower(frontLeftPower);
                backLeft.setPower(backLeftPower);
                frontRight.setPower(frontRightPower);
                backRight.setPower(backRightPower);
            } else {
                frontLeft.setPower(frontLeftPower * 0.5);
                backLeft.setPower(backLeftPower * 0.5);
                frontRight.setPower(frontRightPower * 0.5);
                backRight.setPower(backRightPower * 0.5);
            }
        }

    }
    @Override
    public void stop(){
        limelight.stop();
        leftSlide.setPower(0);
        rightSlide.setPower(0);
    }
}