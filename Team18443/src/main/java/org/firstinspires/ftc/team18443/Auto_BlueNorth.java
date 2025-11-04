package org.firstinspires.ftc.team18443;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.team18443.RobotHardware.flywheelState.*;
import static org.firstinspires.ftc.team18443.RobotHardware.flipperState.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// ****************************************************************************
//  Auto_BlueNorth.java                                      GalacticLions2526
// ****************************************************************************
//   Description:
//      Autonomous routine for the BLUE alliance starting on the NORTH side
//      The code is structured as a LinearOpMode
//
//   Usage:
//      - Deploy this OpMode as an Autonomous on the FTC Driver Station
//
// ****************************************************************************
// This program is released under the BSD-3-Clause-Clear License
// See LICENSE file in root of this repository
// ****************************************************************************

@Autonomous(name="Auto_BlueNorth", group="Autonomous", preselectTeleOp ="StraferTeleOp")
public class Auto_BlueNorth extends LinearOpMode {

    private RobotHardware robot;

    @Override
    public void runOpMode() {

// -------------------------------------------------------------------------------------------------
//    Hardware Setup and Initialization
// -------------------------------------------------------------------------------------------------

        RobotHardware robot = new RobotHardware(this);
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

    }

    /**
     * Use to make the robot move forward a specified distance at a given speed
     *
     * @param inches The distance to move forward, in inches
     * @param speed  The speed to travel (range: 0.0 to 1.0)
     */
    public void forward(double inches, double speed) {
        robot.moveToPosition(inches, speed);
    }

    /**
     * Use to make the robot move backward a specified distance at a given speed
     *
     * @param inches The distance to move backward, in inches
     * @param speed  The speed to travel (range: 0.0 to 1.0)
     */
    public void backward(double inches, double speed) {
        robot.moveToPosition(-inches, speed);
    }

    /**
     * Use to make the robot rotate left by the specified number of degrees at a given speed
     *
     * @param degrees The angle to rotate left, in degrees
     * @param speed   The speed of rotation (range: 0.0 to 1.0)
     */
    public void turnLeft(double degrees, double speed) {
        robot.turnWithGyro(degrees, -speed);
    }

    /**
     * Use to make the robot rotate right by the specified number of degrees at a given speed
     *
     * @param degrees The angle to rotate right, in degrees
     * @param speed   The speed of rotation (range: 0.0 to 1.0)
     */
    public void turnRight(double degrees, double speed) {
        robot.turnWithGyro(degrees, speed);
    }

    /**
     * Use to make the robot strafe left by the specified distance at a given speed
     *
     * @param inches The distance to strafe left, in inches
     * @param speed  The speed of strafing (range: 0.0 to 1.0)
     */
    public void strafeLeft(double inches, double speed) {
        robot.strafeToPosition(-inches, speed);
    }

    /**
     * Use to make the robot strafe right by the specified distance at a given speed
     *
     * @param inches The distance to strafe right, in inches
     * @param speed  The speed of strafing (range: 0.0 to 1.0)
     */
    public void strafeRight(double inches, double speed) {
        robot.strafeToPosition(inches, speed);
    }

    /**
     * Controls the intake system by the specified ticks
     *
     * @param ticks Number of ticks to move
     */
    public void intake(int ticks) {
        robot.setIntake(ticks);
    }

    /**
     * Controls the flywheel using the specified mode
     *
     * @param state Enum defining the flywheel state (ON and OFF)
     */
    public void flywheel(RobotHardware.flywheelState state) {
        robot.setFlywheelState(state);
    }

    /**
     * Controls the flipper using the specified state
     *
     * @param state Enum defining the flipper state (UP and DOWN)
     */
    public void flipper(RobotHardware.flipperState state) {
        robot.setFlipperState(state);
    }
}