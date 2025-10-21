package org.firstinspires.ftc.team5898;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous(name="Limelight Visual Servoing", group="Linear Opmode")
public class LimelightVisualServoing extends OpMode {
    Limelight3A limelight;
    DcMotor frontLeft, frontRight, backLeft, backRight;
    private static final double Kp = 0.03;
    private static final double MIN_COMMAND = 0.10;
    private static final double HEADING_THRESHOLD = 0.80;
    private static final double MAX_TURN_POWER = 0.7;

    @Override
    public void init(){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        limelight.pipelineSwitch(0);
        telemetry.setMsTransmissionInterval(11);
        limelight.start();
    }

    @Override
    public void loop() {
        VisualServoing();
    }

    @Override
    public void stop() {
        limelight.stop();
    }
    private void VisualServoing() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()){
            double tx = result.getTx();
            double ty = result.getTy();
            double ta = result.getTa();

            double headingError = tx; //Can be wrong
            double steeringAdjust = 0.0;
            if (Math.abs(headingError) > HEADING_THRESHOLD) {
                // Proportional control
                steeringAdjust = Kp * headingError;

                // Add minimum command to overcome friction
                if (headingError < 0) {
                    steeringAdjust += MIN_COMMAND;
                } else {
                    steeringAdjust -= MIN_COMMAND;
                }
                steeringAdjust = Math.max(-MAX_TURN_POWER, Math.min(MAX_TURN_POWER, steeringAdjust));
            }


            //Can be adjusted
            frontLeft.setPower(steeringAdjust);
            backLeft.setPower(steeringAdjust);
            frontRight.setPower(-steeringAdjust);
            backRight.setPower(-steeringAdjust);

            telemetry.addData("tx", tx);
            telemetry.addData("ty", ty);
            telemetry.addData("ta", ta);
            telemetry.addData("headingError", headingError);
            telemetry.addData("steeringAdjust", steeringAdjust);
            telemetry.update();
        }
        else {
            telemetry.addData("No valid result", "");
            telemetry.update();
        }
    }

}