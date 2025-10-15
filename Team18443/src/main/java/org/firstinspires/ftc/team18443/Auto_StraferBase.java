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

@Autonomous(name="Auto_StraferBase", group="Auto")
public class Auto_StraferBase extends LinearOpMode {

    private RobotHardware robot;

    @Override
    public void runOpMode() {

// ----------------------------------------------------------------------------
//    Define and Initialize the Hardware Map
// ----------------------------------------------------------------------------

        robot = new RobotHardware(hardwareMap);
        robot.init();
        robot.resetYaw();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses START)
        waitForStart();
        initGyro();;

// ----------------------------------------------------------------------------
//    Autonomous Functions
// ----------------------------------------------------------------------------
        
    }
}