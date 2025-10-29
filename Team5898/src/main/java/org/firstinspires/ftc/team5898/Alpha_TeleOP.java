package org.firstinspires.ftc.team5898;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.team5898.LimelightUtils.VisualServoing;

@TeleOp(name="Alpha TeleOP", group="TeleOP")
public class Alpha_TeleOP extends OpMode {
    Limelight3A limelight;
    DcMotor frontLeft, frontRight, backLeft, backRight, leftSlide, rightSlide;
    Servo leftCanon, rightCanon;
    VisualServoing visualServoing;
    IMU imu;

    @Override
    public void init() {
        //Limelight Init
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
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

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        leftSlide.setTargetPosition(0);
        leftSlide.setTargetPosition(0);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //IMU Init for Field-Centric
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT, //TODO: Verify this
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        //Servo Init
//        leftCanon = hardwareMap.get(Servo.class, "LC");
//        rightCanon = hardwareMap.get(Servo.class, "RC");

        //Visual Servoing Init
        visualServoing = new VisualServoing(limelight, frontLeft, frontRight, backRight, backLeft, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        leftSlide.setPower(.7);
        leftSlide.setPower(.7);
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

        if (gamepad1.a){
            visualServoing.visualServo();
        }

        if(gamepad2.left_trigger >0.5 && gamepad2.right_trigger > 0.5){
            leftSlide.setTargetPosition(-3000);
            rightSlide.setTargetPosition(-3000);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
//        // Telemetry
//        telemetry.addData("Bot Heading", Math.toDegrees(botHeading));
//        telemetry.addData("Left Stick Y", y);
//        telemetry.addData("Left Stick X", x);
//        telemetry.addData("Right Stick X", rx);
//        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}