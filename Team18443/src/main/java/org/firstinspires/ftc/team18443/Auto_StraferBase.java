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

    @Override
    public void runOpMode() {

// -------------------------------------------------------------------------------------------------
//    Define and Initialize the Hardware Map
// -------------------------------------------------------------------------------------------------

        RobotHardware robot = new RobotHardware(hardwareMap);
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
}