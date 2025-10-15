package org.firstinspires.ftc.team18443;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
//    (Section Name)
// ----------------------------------------------------------------------------

    /**
     * Use to make the robot go forward a number of inches
     *  @param inches - distance to travel in inches
     *  @param speed - range of [0,1]
     */
    public void forward(double inches, double speed){ moveToPosition(inches, speed); }

    /**
     * Use to make the robot go backward a number of inches
     *  @param inches - distance to travel in inches
     *  @param speed - range of [0,1]
     */
    public void back(double inches, double speed){ moveToPosition(-inches, speed); }

    /**
     * Use to make the robot rotate left
     *  @param degrees - amount of degrees to rotate
     *  @param speed - range of [0,1]
     */
    public void turnLeft(double degrees, double speed){ turnWithGyro(degrees, -speed); }

    /**
     * Use to make the robot rotate right
     *  @param degrees - amount of degrees to rotate
     *  @param speed - range of [0,1]
     */
    public void turnRight(double degrees, double speed){ turnWithGyro(degrees, speed); }

    /**
     * Use to make the robot strafe left
     *  @param inches - distance in inches to strafe
     *  @param speed - range of [0,1]
     */
    public void strafeLeft(double inches, double speed){ strafeToPosition(-inches, speed); }

    /**
     * Use to make the robot strafe right
     *  @param inches - distance in inches to strafe
     *  @param speed - range of [0,1]
     */
    public void strafeRight(double inches, double speed){ strafeToPosition(inches, speed); }
}