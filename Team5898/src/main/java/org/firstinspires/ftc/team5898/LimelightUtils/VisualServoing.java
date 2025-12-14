package org.firstinspires.ftc.team5898.LimelightUtils;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Set;

@Configurable
public class VisualServoing {
    // Horizontal (tx) alignment - turning left/right
    private static final int[] ALLOWED_TAG_IDS = {20, 24};
    public static final double Kp_HEADING = 0.02; // Proportional gain for horizontal alignment
    public static final double MIN_COMMAND_HEADING = 0.10;
    public static final double HEADING_THRESHOLD = 0.80; // Horizontal offset threshold (degrees)
    public static final double MAX_TURN_POWER = 0.5;

    // Vertical (ty) alignment - moving forward/backward
    public static final double Kp_DISTANCE = 0.03; // Proportional gain for vertical alignment
    public static final double MIN_COMMAND_DISTANCE = 0.12;
    public static final double DISTANCE_THRESHOLD = 1.0; // Vertical offset threshold (degrees)
    public static final double MAX_DRIVE_POWER = 0.5;
    public static final double TARGET_TY = 0.0; // Target ty value (adjust based on desired distance)

    private final Limelight3A limelight;
    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;
    private final Telemetry telemetry;

    public VisualServoing(Limelight3A limelight, DcMotor frontLeft, DcMotor frontRight, DcMotor backRight,
            DcMotor backLeft, Telemetry telemetry) {
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
            double tx = result.getTx(); // Horizontal offset from center
            double ty = result.getTy(); // Vertical offset from center
            double ta = result.getTa();

            // Calculate steering adjustment based on horizontal offset (tx)
            // tx > 0 means target is to the right -> robot needs to turn right (positive steeringAdjust)
            // tx < 0 means target is to the left -> robot needs to turn left (negative steeringAdjust)
            double steeringAdjust = 0.0;
            if (Math.abs(tx) > HEADING_THRESHOLD) {
                // Proportional control for turning
                steeringAdjust = Kp_HEADING * tx;

                // Add minimum command to overcome friction (in the same direction as the error)
                if (tx > 0) {
                    steeringAdjust += MIN_COMMAND_HEADING;
                } else {
                    steeringAdjust -= MIN_COMMAND_HEADING;
                }
                steeringAdjust = Math.max(-MAX_TURN_POWER, Math.min(MAX_TURN_POWER, steeringAdjust));
            }

            // For vertical alignment (ty), we only move forward/backward if you want distance control.
            // If you ONLY want to align with the AprilTag's vertical axis (center horizontally),
            // set driveAdjust to 0 to disable forward/backward movement.
            // ty > TARGET_TY means target is above center -> robot is too far, needs to move forward
            // ty < TARGET_TY means target is below center -> robot is too close, needs to move backward
            double tyError = ty - TARGET_TY;
            double driveAdjust = 0.0;
            // DISABLED: Set to true if you want distance control, false for only horizontal alignment
            boolean enableDistanceControl = false;
            if (enableDistanceControl && Math.abs(tyError) > DISTANCE_THRESHOLD) {
                // Proportional control for forward/backward
                driveAdjust = Kp_DISTANCE * tyError;

                // Add minimum command to overcome friction (in the same direction as the error)
                if (tyError > 0) {
                    driveAdjust += MIN_COMMAND_DISTANCE;
                } else {
                    driveAdjust -= MIN_COMMAND_DISTANCE;
                }
                driveAdjust = Math.max(-MAX_DRIVE_POWER, Math.min(MAX_DRIVE_POWER, driveAdjust));
            }

            // Combine turning and driving: power = drive + turn (for left), drive - turn (for right)
            double frontLeftPower = driveAdjust + steeringAdjust;
            double backLeftPower = driveAdjust + steeringAdjust;
            double frontRightPower = driveAdjust - steeringAdjust;
            double backRightPower = driveAdjust - steeringAdjust;


            double maxPower = Math.max(Math.abs(frontLeftPower),
                    Math.max(Math.abs(backLeftPower),
                            Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))));
            if (maxPower > 1.0) {
                frontLeftPower /= maxPower;
                backLeftPower /= maxPower;
                frontRightPower /= maxPower;
                backRightPower /= maxPower;
            }

            // Apply motor powers
            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            telemetry.addData("tx", tx);
            telemetry.addData("ty", ty);
            telemetry.addData("ta", ta);
            telemetry.addData("steeringAdjust", steeringAdjust);
            telemetry.addData("driveAdjust", driveAdjust);
            telemetry.addData("Heading Aligned", Math.abs(tx) <= HEADING_THRESHOLD);
            telemetry.addData("Distance Aligned", Math.abs(tyError) <= DISTANCE_THRESHOLD);
            telemetry.update();
        } else {
            // Stop all motors when no valid target
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);

            telemetry.addData("No valid result", "");
            telemetry.update();
        }
    }
}
