package org.firstinspires.ftc.team18443;

// ============================================================================
//  StraferTeleOp.java                                       GalacticLions2526
// ============================================================================
//
//   Description:
//      Field-centric Teleop for a mecanum-drive robot. In this mode, movement
//      is relative to the field rather than its own orientation. Pushing
//      forward always moves the robot away from the driver, regardless of its
//      heading
//
//      Implemented as a LinearOpMode for straightforward, sequential execution
//
//   Usage:
//      - Deploy this OpMode as a TeleOp on the FTC Driver Station
//
// ============================================================================
// This program is released under the BSD-3-Clause-Clear License
// See LICENSE file in root of this repository
// ============================================================================

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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

            // Read joystick inputs and apply deadzone to prevent drift from minor stick movement
            double y  = robot.applyJoystickDeadzone(-gamepad1.left_stick_y); // Forward/backward
            double x  = robot.applyJoystickDeadzone(gamepad1.left_stick_x);  // Strafe left/right
            double rx = robot.applyJoystickDeadzone(gamepad1.right_stick_x); // Rotation

            // Drive the robot using field-centric control
            robot.driveFieldCentric(x, y, rx);

            // Reset the IMU yaw angle to zero manually by pressing the 'guide' button
            // Useful for correcting drift or re-aligning the robot to the field orientation
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
            double targetVelocity;

            if (gamepad2.right_trigger > 0.5) {
                // High power shooting mode
                targetVelocity = robot.FLYWHEEL_VEL_HIGH;
            } else if (gamepad2.left_trigger > 0.5) {
                // Medium power shooting mode
                targetVelocity = robot.FLYWHEEL_VEL_MEDIUM;
            } else {
                // Turn off flywheel when no trigger is pressed
                targetVelocity = robot.FLYWHEEL_VEL_OFF;
            }

            // Apply ramp to the current velocity
            double vel = robot.applyRampToVelocity(robot.flywheel.getVelocity(), targetVelocity);

            robot.flywheel.setVelocity(vel);

            if (gamepad2.a) {
                robot.flipper.setPosition(robot.FLIPPER_UP);   // Raise flipper
            }
            else if (gamepad2.b) {
                robot.flipper.setPosition(robot.FLIPPER_DOWN); // Lower flipper
            }

            telemetry.addData("Heading (rad)", "%.1f", robot.getHeadingRad());
            telemetry.update();
        }
    }
}
