package org.firstinspires.ftc.team5898.LimelightUtils;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;


@Configurable
public class VisualServoing {
    public static final double Kp = 0.03;
    public static final double MIN_COMMAND = 0.10;
    public static final double HEADING_THRESHOLD = 0.80;
    public static final double MAX_TURN_POWER = 0.7;

    private final Limelight3A limelight;
    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;
    private final Telemetry telemetry;

    public VisualServoing(Limelight3A limelight, DcMotor frontLeft, DcMotor frontRight, DcMotor backRight, DcMotor backLeft, Telemetry telemetry) {
        this.limelight = limelight;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.telemetry = telemetry;
    }

    public void visualServo() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx();
            double ty = result.getTy();
            double ta = result.getTa();

            double steeringAdjust = 0.0;
            if (Math.abs(tx) > HEADING_THRESHOLD) {
                // Proportional control
                steeringAdjust = Kp * tx;

                // Add minimum command to overcome friction
                if (tx < 0) {
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
            telemetry.addData("headingError", tx);
            telemetry.addData("steeringAdjust", steeringAdjust);
            telemetry.update();
        } else {
            telemetry.addData("No valid result", "");
            telemetry.update();
        }
    }
}
