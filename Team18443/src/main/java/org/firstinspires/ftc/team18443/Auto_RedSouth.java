package org.firstinspires.ftc.team18443;

// ============================================================================
//  Auto_RedSouth.java                                       GalacticLions2526
// ============================================================================
//
//   Description:
//      Autonomous routine for the RED alliance starting on the SOUTH side
//      The code is structured as a LinearOpMode
//
//   Usage:
//      - Deploy this OpMode as an Autonomous on the FTC Driver Station
//
// ============================================================================
// This program is released under the BSD-3-Clause-Clear License
// See LICENSE file in root of this repository
// ============================================================================

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.team18443.RobotHardware.flywheelState.*;
import static org.firstinspires.ftc.team18443.RobotHardware.flipperState.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name="Auto_RedSouth", group="Autonomous", preselectTeleOp ="StraferTeleOp")
public class Auto_RedSouth extends LinearOpMode {

    private RobotHardware robot;

    @Override
    public void runOpMode() {

// -------------------------------------------------------------------------------------------------
//    Hardware Setup and Initialization
// -------------------------------------------------------------------------------------------------

        robot = new RobotHardware(this);
        robot.init();
        robot.resetYaw();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses START)
        waitForStart();

// -------------------------------------------------------------------------------------------------
//    Autonomous Function Calling Zone
// -------------------------------------------------------------------------------------------------
        /*
         * This is where you will define the sequence of movements and actions
         * that the robot performs during this Autonomous routine
         *
         * ===== HOW TO USE =====
         * - Call the desired function(s) in the order you want them to execute
         * - Use telemetry.addData(...) for debugging or progress tracking
         *
         * ===== EXAMPLE USAGE =====
         * forward(25, 0.5);
         * turnRight(90, 0.4);
         * strafeLeft(12, 0.5);
         */

        forward(20, 0.5);

        telemetry.addData("Auto Run", "Complete");
        telemetry.update();
        sleep(1000);  // Pause to display completion message
    }

    /**
     * Moves the robot forward a specified distance at a given speed
     *
     * @param inches The distance to move forward in inches
     * @param speed  The speed to travel (range: 0.0 to 1.0)
     * @see RobotHardware#moveToPosition(double, double)
     */
    public void forward(double inches, double speed) {
        robot.moveToPosition(inches, speed);
    }

    /**
     * Moves the robot backward a specified distance at a given speed
     *
     * @param inches The distance to move backward in inches
     * @param speed  The speed to travel (range: 0.0 to 1.0)
     * @see RobotHardware#moveToPosition(double, double)
     */
    public void backward(double inches, double speed) {
        robot.moveToPosition(-inches, speed);
    }

    /**
     * Rotates the robot left by a specified number of degrees at a given speed
     *
     * @param degrees The angle to rotate left in degrees
     * @param speed   The speed of rotation (range: 0.0 to 1.0)
     * @see RobotHardware#turnWithGyro(double, double)
     */
    public void turnLeft(double degrees, double speed) {
        robot.turnWithGyro(degrees, -speed);
    }

    /**
     * Rotates the robot right by a specified number of degrees at a given speed
     *
     * @param degrees The angle to rotate right in degrees
     * @param speed   The speed of rotation (range: 0.0 to 1.0)
     * @see RobotHardware#turnWithGyro(double, double)
     */
    public void turnRight(double degrees, double speed) {
        robot.turnWithGyro(degrees, speed);
    }

    /**
     * Strafes the robot left by a specified distance at a given speed
     *
     * @param inches The distance to strafe left in inches
     * @param speed  The speed of strafing (range: 0.0 to 1.0)
     * @see RobotHardware#strafeToPosition(double, double)
     */
    public void strafeLeft(double inches, double speed) {
        robot.strafeToPosition(-inches, speed);
    }

    /**
     * Strafes the robot right by a specified distance at a given speed
     *
     * @param inches The distance to strafe right in inches
     * @param speed  The speed of strafing (range: 0.0 to 1.0)
     * @see RobotHardware#strafeToPosition(double, double)
     */
    public void strafeRight(double inches, double speed) {
        robot.strafeToPosition(inches, speed);
    }

    /**
     * Moves the intake system by a specified number of ticks
     *
     * @param ticks Number of ticks to move
     */
    public void intake(int ticks) {
        robot.setIntake(ticks);
    }

    /**
     * Controls the flywheel by setting it to the specified state
     *
     * @param state The desired flywheel state (HIGH, MEDIUM, or OFF)
     */
    public void flywheel(RobotHardware.flywheelState state) {
        robot.setFlywheelState(state);
    }

    /**
     * Controls the flipper using the specified state
     *
     * @param state The desired flipper state (UP and DOWN)
     */
    public void flipper(RobotHardware.flipperState state) {
        robot.setFlipperState(state);
    }
}