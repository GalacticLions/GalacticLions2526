package org.firstinspires.ftc.team18443;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// ****************************************************************************
//  StraferTeleOp_Generic.java                              GalacticLions2526
// ****************************************************************************
//   Description:
//      This OpMode executes a field-centric Teleop for a mecanum drive robot
//      The code is structured as a LinearOpMode
//
//   Usage:
//      - Deploy as TeleOp via FTC Driver Station
//
//   Notes:
//      - This version uses a generic RobotHardware class for initialization
//        and basic helpers (motors + IMU). Update device names/orientation
//        inside RobotHardware to match your configuration.
//
// ****************************************************************************
// This program is released under the BSD-3-Clause-Clear License
// See LICENSE file in root of this repository
// ****************************************************************************

@TeleOp(name="Strafer Tele Op (Generic HW)", group="TeleOp")
public class StraferTeleOp_Generic extends LinearOpMode {

    private RobotHardware robot;

    @Override
    public void runOpMode() {

// ----------------------------------------------------------------------------
//    Define and Initialize the Motors
// ----------------------------------------------------------------------------
// Note: Make sure the ID's match in your configuration
//       (Device names and IMU orientation are set in RobotHardware.init())

        robot = new RobotHardware(hardwareMap);
        robot.init();       // sets up FL/BL/FR/BR, launcher, belt, IMU, directions, etc.
        robot.resetYaw();   // start with a zeroed heading for field-centric control

// ----------------------------------------------------------------------------
//    Define and Initialize the REV Hub's IMU (Inertial measurement unit)
// ----------------------------------------------------------------------------
// Note: Adjust orientation parameters in RobotHardware to match your mounting
//       By default, RobotHardware initializes and manages the IMU and yaw reset.

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses START)
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

// ----------------------------------------------------------------------------
//    Primary Driver Controls (gamepad1)
// ----------------------------------------------------------------------------

            double y  = -gamepad1.left_stick_y;  // Forward/backward (negative because up is negative)
            double x  =  gamepad1.left_stick_x * robot.strafeComp; // Strafe left/right (scaled to compensate for imperfect strafing)
            double rx =  gamepad1.right_stick_x; // Rotation

            // Convert field-relative joystick inputs to robot-relative using IMU heading
            double botHeading = robot.getHeadingRad();
            double X = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double Y = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Normalize motor powers to keep within [-1, 1] while maintaining power ratios
            double denominator = Math.max(Math.abs(Y) + Math.abs(X) + Math.abs(rx), 1.0);
            double frontLeftPower  = (Y + X + rx) / denominator;
            double backLeftPower   = (Y - X + rx) / denominator;
            double frontRightPower = (Y - X - rx) / denominator;
            double backRightPower  = (Y + X - rx) / denominator;

            // Apply powers via RobotHardware helper (handles safe normalization as well)
            robot.setDrivePowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);

            // Reset IMU yaw angle to zero manually by pressing the 'guide' button
            if (gamepad1.guide) {
                robot.resetYaw();
            }

// ----------------------------------------------------------------------------
//    Secondary Driver Controls (gamepad2)
// ----------------------------------------------------------------------------
            // (Add mechanism controls here as needed; launcher/belt are available in robot.*)

            telemetry.addData("Heading (deg)", "%.1f", robot.getHeadingDeg());
            telemetry.update();
        }
    }
}
