package org.firstinspires.ftc.team5898


import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.team5898.Constants.SlideConstants

@TeleOp(name = "Slide Test (Kotlin)", group = "Test")
class SlideTest_Kotlin : OpMode() {
    private lateinit var slideLeft: DcMotor
    private lateinit var slideRight: DcMotor
    private var slideLeftTarget: Int = 0
    private var slideRightTarget: Int = 0
    private var slideLeftPosition: Int = 0
    private var slideRightPosition: Int = 0
    private var joystickYR: Double = 0.0


    private var slidePower: Double = SlideConstants.movePower
    private var holdPower: Double = SlideConstants.holdPower
    val panelsTelemetry = PanelsTelemetry.telemetry


    override fun init() {
        slideLeft = hardwareMap.dcMotor["LS"]
        slideRight = hardwareMap.dcMotor["RS"]
        slideLeft.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        slideRight.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        slideLeft.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        slideRight.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        slideLeft.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        slideRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        slideLeftTarget = 0
        slideRightTarget = SlideConstants.Offset
        panelsTelemetry.update()

    }

    override fun loop() {

        slidePower = SlideConstants.movePower
        holdPower = SlideConstants.holdPower
        slideLeft.targetPosition = slideLeftTarget
        slideRight.targetPosition = slideRightTarget
        if (gamepad1.dpad_up) {
            slideRightTarget += 10
            slideLeftTarget -= 10

        } else if (gamepad1.dpad_down) {
            slideRightTarget -= 10
            slideLeftTarget += 10
        }

        val leftPos = slideLeft.currentPosition
        val rightPos = slideRight.currentPosition

        slideLeftPosition = slideLeft.currentPosition
        slideRightPosition = slideRight.currentPosition

        val leftError = Math.abs(slideLeftPosition - slideLeftTarget)
        val rightError = Math.abs(slideRightPosition - slideRightTarget)



        if (leftError > 5) {
            slideLeft.targetPosition = slideLeftTarget
            slideLeft.mode = DcMotor.RunMode.RUN_TO_POSITION
            slideLeft.power = slidePower
        } else {
            slideLeft.power = 0.0
            slideLeft.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }

        if (rightError > 5) {
            slideRight.targetPosition = slideRightTarget
            slideRight.mode = DcMotor.RunMode.RUN_TO_POSITION
            slideRight.power = slidePower
        } else {
            slideRight.power = 0.0
            slideRight.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }


        telemetry.addData("Left Target/Pos", "$slideLeftTarget / $leftPos")
        telemetry.addData("Right Target/Pos", "$slideRightTarget / $rightPos")
        telemetry.addData("Left Power", slideLeft.power)
        telemetry.addData("Right Power", slideRight.power)
        telemetry.addData("Slide Left Encoder", slideLeftPosition)
        telemetry.addData("Slide Right Encoder", slideRightPosition)
        telemetry.update()

        panelsTelemetry.addData("Left Target/Pos", "$slideLeftTarget / $leftPos")
        panelsTelemetry.addData("Right Target/Pos", "$slideRightTarget / $rightPos")
        panelsTelemetry.addData("Left Power", slideLeft.power)
        panelsTelemetry.addData("Right Power", slideRight.power)
        panelsTelemetry.addData("Right Joystick Y", joystickYR)
        panelsTelemetry.addData("Slide Left Encoder", slideLeftPosition)
        panelsTelemetry.addData("Slide Right Encoder", slideRightPosition)
        panelsTelemetry.update()

    }

    override fun stop() {
        slideLeft.power = 0.0
        slideRight.power = 0.0
    }
}

