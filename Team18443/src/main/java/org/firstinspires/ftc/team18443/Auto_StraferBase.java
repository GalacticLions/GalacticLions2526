package org.firstinspires.ftc.team18443;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// ****************************************************************************
//  Auto_StraferBase.java                                    GalacticLions2526
// ****************************************************************************
//   Description:
//      Text
//
//   Usage:
//      - Text
//
// ****************************************************************************
// This program is released under the BSD-3-Clause-Clear License
// See LICENSE file in root of this repository
// ****************************************************************************

@Autonomous(name="Auto_StraferBase", group="Auto", preselectTeleOp ="Strafer Tele Op")
public class Auto_StraferBase extends LinearOpMode {

    @Override
    public void runOpMode() {

// ----------------------------------------------------------------------------
//    Define and Initialize the Hardware Map
// ----------------------------------------------------------------------------

        RobotHardware robot = new RobotHardware(hardwareMap);
        robot.init();
        robot.resetYaw();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses START)
        waitForStart();

// ----------------------------------------------------------------------------
//    Autonomous Function Calling Zone
// ----------------------------------------------------------------------------
        
    }

// ----------------------------------------------------------------------------
//    Methods for Movement & Mechanisms
// ----------------------------------------------------------------------------

    /**
     * Use to make the robot go forward a number of inches
     *  @param inches The distance to travel in inches
     *  @param speed Has a range of [0,1]
     */
    public void forward(double inches, double speed){ moveToPosition(inches, speed); }

    /**
     * Use to make the robot go backward a number of inches
     *  @param inches The distance to travel in inches
     *  @param speed Has a range of [0,1]
     */
    public void backward(double inches, double speed){ moveToPosition(-inches, speed); }

    /**
     * Use to make the robot rotate left
     *  @param degrees The amount of degrees to rotate
     *  @param speed Has a range of [0,1]
     */
    public void turnLeft(double degrees, double speed){ turnWithGyro(degrees, -speed); }

    /**
     * Use to make the robot rotate right
     *  @param degrees The amount of degrees to rotate
     *  @param speed Has a range of [0,1]
     */
    public void turnRight(double degrees, double speed){ turnWithGyro(degrees, speed); }

    /**
     * Use to make the robot strafe left
     *  @param inches The distance in inches to strafe
     *  @param speed Has a range of [0,1]
     */
    public void strafeLeft(double inches, double speed){ strafeToPosition(-inches, speed); }

    /**
     * Use to make the robot strafe right
     *  @param inches The distance in inches to strafe
     *  @param speed Has a range of [0,1]
     */
    public void strafeRight(double inches, double speed){ strafeToPosition(inches, speed); }
}