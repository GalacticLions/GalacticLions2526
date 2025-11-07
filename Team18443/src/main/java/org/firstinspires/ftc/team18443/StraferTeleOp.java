package org.firstinspires.ftc.team18443;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// ****************************************************************************
//  StraferTeleOp.java                                       GalacticLions2526
// ****************************************************************************
//   Description:
//      This OpMode executes a field-centric Teleop for a mecanum-drive robot.
//      The code is structured as a LinearOpMode
//
//   Usage:
//      - Deploy this OpMode as a TeleOp on the FTC Driver Station
//
// ****************************************************************************
// This program is released under the BSD-3-Clause-Clear License
// See LICENSE file in root of this repository
// ****************************************************************************

@TeleOp(name="StraferTeleOp", group="TeleOp")
public class StraferTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {

// -------------------------------------------------------------------------------------------------
//    Hardware Setup and Initialization
// -------------------------------------------------------------------------------------------------
//    Note: Device names, IMU orientation, motor directions, and any device-specific
//          configurations are handled in RobotHardware.init()

        RobotHardware robot = new RobotHardware(this);
        robot.init();
        robot.resetYaw();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses START)
        waitForStart();

        if (isStopRequested()) return;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

// -------------------------------------------------------------------------------------------------
//    Primary Driver Controls (gamepad1)
// -------------------------------------------------------------------------------------------------
//
//    +------------------------+-----------------------------------+
//    | Control Input          | Action                            |
//    +------------------------+-----------------------------------+
//    | Left Stick Y           | Move forward/backward             |
//    | Left Stick X           | Strafe left/right                 |
//    | Right Stick X          | Rotate robot CW/CCW               |
//    | Guide Button           | Reset IMU yaw to zero             |
//    +------------------------+-----------------------------------+

            double y  = -gamepad1.left_stick_y;  // Forward/backward (negative because up is negative)
            double x  =  gamepad1.left_stick_x * robot.strafeComp; // Strafe left/right (scaled)
            double rx =  gamepad1.right_stick_x; // Rotation

            // Convert field-relative joystick inputs (x,y) to robot-relative using IMU heading
            double botHeading = robot.getHeadingRad();
            double X = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double Y = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Combine the joystick requests for each axis-motion
            double frontLeftPower  = (Y + X + rx);
            double frontRightPower = (Y - X - rx);
            double backLeftPower   = (Y - X + rx);
            double backRightPower  = (Y + X - rx);

            // Normalize motor power values
            robot.setDrivePowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);

            // Reset the IMU yaw angle to zero manually by pressing the 'guide' button
            // Note: The 'guide' button mapping may differ between controller types
            if (gamepad1.guide) {
                robot.resetYaw();
            }

// -------------------------------------------------------------------------------------------------
//    Secondary Driver Controls (gamepad2)
// -------------------------------------------------------------------------------------------------
//
//    +------------------------+-----------------------------------+
//    | Control Input          | Action                            |
//    +------------------------+-----------------------------------+
//    | D-Pad Up               | Run intake forward  (collect)     |
//    | D-Pad Down             | Run intake backward (eject)       |
//    | Right Trigger (> 0.8)  | Flywheel high power               |
//    | Right Trigger (> 0.3)  | Flywheel medium power             |
//    | Left Trigger  (> 0.5)  | Flywheel off                      |
//    | A Button               | Flipper up                        |
//    | B Button               | Flipper down                      |
//    +------------------------+-----------------------------------+

            // Control intake system (conveyor + wheels) using D-pad
            if (gamepad2.dpad_up) {
                // Run intake forward to collect artifacts
                robot.intakeConveyor.setPower(1.0);
                robot.intakeWheels.setPower(1.0);
            }
            else if (gamepad2.dpad_down) {
                // Run intake backward to eject artifacts
                robot.intakeConveyor.setPower(-1.0);
                robot.intakeWheels.setPower(-1.0);
            }
            else {
                // Stop intake when no D-Pad button is pressed
                robot.intakeConveyor.setPower(0.0);
                robot.intakeWheels.setPower(0.0);
            }

            // Control flywheel shooter using triggers
            if (gamepad2.right_trigger > 0.8) {
                // High power shooting mode (fully pressed trigger)
                robot.flywheel.setPower(robot.FLYWHEEL_POWER_HIGH);
            }
            else if (gamepad2.right_trigger > 0.3) {
                // Medium power shooting mode (partially pressed trigger)
                robot.flywheel.setPower(robot.FLYWHEEL_POWER_MEDIUM);

            }
            else if (gamepad2.left_trigger > 0.5) {
                // Turn off flywheel when left trigger is pressed
                robot.flywheel.setPower(robot.FLYWHEEL_POWER_OFF);
            }

            if (gamepad2.a) {
                robot.flipper.setPosition(robot.FLIPPER_UP);
            }
            else if (gamepad2.b) {
                robot.flipper.setPosition(robot.FLIPPER_DOWN);
            }

            telemetry.addData("Heading (rad)", "%.1f", robot.getHeadingRad());
            telemetry.update();
        }
    }
}
