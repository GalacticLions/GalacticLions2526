package org.firstinspires.ftc.team18443;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// ****************************************************************************
//  Auto_StraferBase.java                                    GalacticLions2526
// ****************************************************************************
//   Description:
//      Text
//      The code is structured as a LinearOpMode
//
//   Usage:
//      - Extend this class to create new autonomous OpModes
//
// ****************************************************************************
// This program is released under the BSD-3-Clause-Clear License
// See LICENSE file in root of this repository
// ****************************************************************************

@Autonomous(name="Auto_StraferBase", group="Auto", preselectTeleOp ="Strafer Tele Op")
public class Auto_StraferBase extends LinearOpMode {

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
        
    }

    /**
     * Use to make the robot move forward a specified distance at a given speed
     *
     * @param inches The distance to move forward, in inches
     * @param speed  The speed to travel (range: 0.0 to 1.0)
     */
    public void forward(double inches, double speed){
        robot.moveToPosition(inches, speed);
    }

    /**
     * Use to make the robot move backward a specified distance at a given speed
     *
     * @param inches The distance to move backward, in inches
     * @param speed  The speed to travel (range: 0.0 to 1.0)
     */
    public void backward(double inches, double speed){
        robot.moveToPosition(-inches, speed);
    }

    /**
     * Use to make the robot rotate left by the specified number of degrees at a given speed
     *
     * @param degrees The angle to rotate left, in degrees
     * @param speed   The speed of rotation (range: 0.0 to 1.0)
     */
    public void turnLeft(double degrees, double speed){
        robot.turnWithGyro(degrees, -speed);
    }

    /**
     * Use to make the robot rotate right by the specified number of degrees at a given speed
     *
     * @param degrees The angle to rotate right, in degrees
     * @param speed   The speed of rotation (range: 0.0 to 1.0)
     */
    public void turnRight(double degrees, double speed){
        robot.turnWithGyro(degrees, speed);
    }

    /**
     * Use to make the robot strafe left by the specified distance at a given speed
     *
     * @param inches The distance to strafe left, in inches
     * @param speed  The speed of strafing (range: 0.0 to 1.0)
     */
    public void strafeLeft(double inches, double speed){
        robot.strafeToPosition(-inches, speed);
    }

    /**
     * Use to make the robot strafe right by the specified distance at a given speed
     *
     * @param inches The distance to strafe right, in inches
     * @param speed  The speed of strafing (range: 0.0 to 1.0)
     */
    public void strafeRight(double inches, double speed){
        robot.strafeToPosition(inches, speed);
    }
}