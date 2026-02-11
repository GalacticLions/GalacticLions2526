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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.team5898.Constants.CannonConstants;
import org.firstinspires.ftc.team5898.LimelightUtils.VisualServoing;

@TeleOp(name = "Beta TeleOP")
public class Beta_TeleOP_Refactor extends OpMode {
    Limelight3A limelight;
    DcMotor frontLeft, frontRight, backLeft, backRight, leftSlide, rightSlide;
    DcMotorEx topLauncher, bottomLauncher;
    VisualServoing visualServoing;
    CRServo backLeftServo, backRightServo;
    Servo stopperServo;
    DcMotor frontIntake;
    IMU imu;
    Double intakePower;
    Double LAUNCH_VELOCITY;
    int kP;
    Double kI;
    Double kD;
    Integer kF;

    enum States {IDLE, INTAKE, LAUNCH, SPOOL_UP, VISUAL_SERVOING};
    private States robotState = States.IDLE;
    boolean a1Pressed = gamepad1.a, b2Pressed = gamepad2.b, a2Pressed = gamepad2.a;
    boolean prevA1 = a1Pressed, prevB2 = b2Pressed, prevA2 = a2Pressed;
    boolean a1JustPressed = a1Pressed && !prevA1, b2JustPressed = b2Pressed && !prevB2, a2JustPressed = a2Pressed && !prevA2;



    @Override
    public void init() {
        //Constants Init
        //Cannon Constants
        intakePower = CannonConstants.IntakePower;
        kP = CannonConstants.kP;
        kI = CannonConstants.kI;
        kD = CannonConstants.kD;
        kF = CannonConstants.kF;

        // Set target velocities in Ticks Per Second. These are example values and will need tuning.
        LAUNCH_VELOCITY = CannonConstants.LAUNCH_VELOCITY;

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
        topLauncher = hardwareMap.get(DcMotorEx.class, "TLaunch");
        bottomLauncher = hardwareMap.get(DcMotorEx.class, "BLaunch");
        frontIntake = hardwareMap.get(DcMotor.class, "Intake");


        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);

        topLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
        bottomLauncher.setDirection(DcMotorSimple.Direction.FORWARD);

        topLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        topLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kP, kI, kD, kF));
        bottomLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kP, kI, kD, kF));

        topLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bottomLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        backLeftServo = hardwareMap.get(CRServo.class, "LServo");
        backRightServo = hardwareMap.get(CRServo.class, "RServo");
        stopperServo = hardwareMap.get(Servo.class, "STPR");

        frontIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftServo.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightServo.setDirection(DcMotorSimple.Direction.FORWARD);

        frontIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        kP = CannonConstants.kP;
        kI = CannonConstants.kI;
        kD = CannonConstants.kD;

        switch (robotState) {
            case IDLE:
                FieldCentricDrive(false);
                if (a1JustPressed) {
                    robotState = States.VISUAL_SERVOING;
                }
                backLeftServo.setPower(0);
                backRightServo.setPower(0);
                frontIntake.setPower(0);
                bottomLauncher.setPower(0);
                topLauncher.setPower(0);
                if(b2JustPressed){
                    robotState = States.INTAKE;
                }

                stopperServo.setPosition(1); //TODO: Can be changed

            case INTAKE:
                FieldCentricDrive(false);
                if(b2JustPressed){
                    robotState = States.IDLE;
                }

                if(gamepad2.right_trigger > 0.3){
                    robotState = States.SPOOL_UP;
                }

                backLeftServo.setPower(intakePower);
                backRightServo.setPower(intakePower);
                frontIntake.setPower(.7);
                stopperServo.setPosition(0.5); //TODO: Can be changed

            case LAUNCH:
                FieldCentricDrive(false);
                backLeftServo.setPower(.7);
                topLauncher.setVelocity(LAUNCH_VELOCITY);
                bottomLauncher.setVelocity(LAUNCH_VELOCITY);
                if () {

                }


            case SPOOL_UP:
                FieldCentricDrive(false);
                if(gamepad2.right_trigger > 0.3){
                robotState = States.SPOOL_UP;
            } else if (gamepad2.right_trigger < 0.3){
                robotState = States.IDLE;
            } if () {
                topLauncher.setVelocity(LAUNCH_VELOCITY);
                bottomLauncher.setVelocity(LAUNCH_VELOCITY);
                }


            case VISUAL_SERVOING:
                FieldCentricDrive(true);
                visualServoing.visualServo();
                if (a1JustPressed) {
                    robotState = States.IDLE;
                }

        }

        //VisualServoing: gamepad1.a
        //Check if VisualServoing
        boolean isVisualServoing = gamepad1.a;
        if (isVisualServoing) {
            visualServoing.visualServo();
        }

        //Cannon Control - Now using setVelocity for closed-loop control
        if (gamepad2.left_stick_y > 0.3) {
            topLauncher.setVelocity(-LAUNCH_VELOCITY);
            bottomLauncher.setVelocity(LAUNCH_VELOCITY);
        } else if (gamepad2.left_stick_y < -0.3) {
            topLauncher.setVelocity(LAUNCH_VELOCITY);
            bottomLauncher.setVelocity(-LAUNCH_VELOCITY);
        } else {
            topLauncher.setVelocity(0);
            bottomLauncher.setVelocity(0);
        }

        if (gamepad2.right_stick_y > 0.3) {
            backLeftServo.setPower(-intakePower);
            backRightServo.setPower(-intakePower);
            frontIntake.setPower(-intakePower);
        } else if (gamepad2.right_stick_y < -0.3) {
            backLeftServo.setPower(intakePower);
            backRightServo.setPower(intakePower);
            frontIntake.setPower(.7);
        }else {
            backLeftServo.setPower(0);
            backRightServo.setPower(0);
            frontIntake.setPower(0);
        }

        if (gamepad2.dpad_down){
            backLeftServo.setPower(intakePower-0.3);
            backRightServo.setPower(intakePower-0.3);
        }else if (gamepad2.dpad_up){
            backLeftServo.setPower(-intakePower+0.3);
            backRightServo.setPower(-intakePower+0.3);
        }




    }
    public void FieldCentricDrive(Boolean isVisualServoing){
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
    public void setIntakePower(double power) {
        frontIntake.setPower(0);
        backLeftServo.setPower(0);
        backRightServo.setPower(0);
        topLauncher.setVelocity(0);
        bottomLauncher.setVelocity(0);
    }
}
