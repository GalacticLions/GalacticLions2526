package org.firstinspires.ftc.team5898;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.team5898.Constants.CannonConstants;
import org.firstinspires.ftc.team5898.Constants.SlideConstants;
import org.firstinspires.ftc.team5898.LimelightUtils.VisualServoing;

@TeleOp(name="Alpha TeleOP", group="TeleOP")
public class Alpha_TeleOP extends OpMode {
    Limelight3A limelight;
    DcMotor frontLeft, frontRight, backLeft, backRight, leftSlide, rightSlide, topLauncher, bottomLauncher;
    VisualServoing visualServoing;
    CRServo cannonLeft, cannonRight;
    IMU imu;
    Integer Offset,  errorThreshold, slideLeftTarget, slideRightTarget,slideLeftPosition, slideRightPosition,leftError,rightError;
    Double slidePower, LaunchPower,intakePower,LaunchPower_alt;

    @Override
    public void init() {
        //Constants Init
        //Cannon Constants
        LaunchPower = CannonConstants.LaunchPower;
        intakePower = CannonConstants.IntakePower;
        Offset = SlideConstants.Offset;
        //Slide Constants
        slidePower = SlideConstants.movePower;
        errorThreshold = SlideConstants.ErrorThreshold;
        LaunchPower_alt = CannonConstants.LaunchPower_Reduced;



        //Limelight Init
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        telemetry.setMsTransmissionInterval(11);
        limelight.setPollRateHz(90);
        limelight.stop();

        //Motors Init
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");
        leftSlide = hardwareMap.get(DcMotor.class, "LS");
        rightSlide = hardwareMap.get(DcMotor.class, "RS");
        topLauncher = hardwareMap.get(DcMotor.class, "TL");
        bottomLauncher = hardwareMap.get(DcMotor.class, "BLa");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        topLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomLauncher.setDirection(DcMotorSimple.Direction.FORWARD);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        slideLeftPosition = leftSlide.getCurrentPosition();
//        slideRightPosition = leftSlide.getCurrentPosition();
        slideLeftTarget = 0;
        slideRightTarget = 0;
        cannonLeft = hardwareMap.get(CRServo.class, "LC");
        cannonRight = hardwareMap.get(CRServo.class, "RC");
        //TODO: Directions could be flipped for code below
        cannonRight.setDirection(DcMotorSimple.Direction.REVERSE);
        cannonLeft.setDirection(DcMotorSimple.Direction.FORWARD);



        //Servo Init



        //IMU Init for Field-Centric
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

        } else if (gamepad1.dpad_down) {
            slideRightTarget -= 10;
            slideLeftTarget += 10;
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
        if (gamepad1.a) {
            visualServoing.visualServo();
        }


        //Cannon Control
        if (gamepad2.left_stick_y > 0.3) {
            topLauncher.setPower(-LaunchPower_alt);
            bottomLauncher.setPower(LaunchPower);
        } else if (gamepad2.left_stick_y < -0.3) {
            topLauncher.setPower(LaunchPower_alt);
            bottomLauncher.setPower(-LaunchPower);
        } else {
            topLauncher.setPower(0);
            bottomLauncher.setPower(0);
        }


        if (gamepad2.right_stick_y > 0.3) {
            cannonLeft.setPower(intakePower);
            cannonRight.setPower(intakePower);
        } else if (gamepad2.right_stick_y < -0.3) {
            cannonLeft.setPower(-intakePower);
            cannonRight.setPower(-intakePower);
        }else {
            cannonLeft.setPower(0);
            cannonRight.setPower(0);
        }



        //===========================================

        // Field-Centric Drive Code
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

        // Set motor powers (adjust multiplier as needed)
        frontLeft.setPower(frontLeftPower * 0.9);
        backLeft.setPower(backLeftPower * 0.9);
        frontRight.setPower(frontRightPower * 0.9);
        backRight.setPower(backRightPower * 0.9);

    }
    @Override
    public void stop(){
        limelight.stop();
        leftSlide.setPower(0);
        rightSlide.setPower(0);
    }
}