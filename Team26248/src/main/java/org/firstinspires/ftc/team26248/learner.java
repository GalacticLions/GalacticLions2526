package org.firstinspires.ftc.team26248;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name="Learner",group="linear OpMode")
public class learner extends LinearOpMode {
    public void runOpMode() {
        //This is where MOST of my robot code will go.
        DcMotor leftMotor = hardwareMap.get(DcMotor.class, "Dis");
        DcMotor rightMotor = hardwareMap.get(DcMotor.class, "Dat");
        Servo doze = hardwareMap.get(Servo.class, "doze");
        doze.setPosition(1);
        CRServo deez = hardwareMap.get(CRServo.class, "deez");
        waitForStart();
        while (opModeIsActive()) {
            //The code in this loop will happen when i press play.
//            if (gamepad1.right_bumper){
//                rightMotor.setPower(.5);
//            }
//            else{
//                rightMotor.setPower(0);
//            }
//            if(gamepad1.left_bumper){
//                leftMotor.setPower(-.5);
//
//            }
//            else {leftMotor.setPower(0);
            //driving using triggers:
            //rightMotor.setPower(gamepad1.right_trigger);
            //leftMotor.setPower(gamepad1.right_trigger);
            //true tank drive with the thumbsticks
            rightMotor.setPower(-gamepad1.right_stick_y);
            leftMotor.setPower(gamepad1.left_stick_y);
            if(gamepad1.a){
                doze.setPosition(0);
            }
            if(gamepad1.b){
                doze.setPosition(.5);
            }
            if(gamepad2.a){
                deez.setPower(1);
            }
            else{
                deez.setPower(0);
            }
                   if(gamepad2.b) {
                       deez.setPower(-1);
                   }
                   else{
                       deez.setPower(0);
                   }


        }
    }
}