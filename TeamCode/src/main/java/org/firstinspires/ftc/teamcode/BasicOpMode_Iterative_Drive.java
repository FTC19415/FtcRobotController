package org.firstinspires.ftc.teamcode;/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Mechanum Drive", group="Iterative Opmode")
//@Disabled
public class BasicOpMode_Iterative_Drive extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor carousel = null;
    private DcMotor armObj = null;
    private DcMotor LinearArmObj = null;
    private Servo clawObj = null;
    private TouchSensor ArmStop;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightMotor");
        carousel = hardwareMap.get(DcMotor.class, "carousel");
        armObj = hardwareMap.get(DcMotor.class, "Arm");
        clawObj = hardwareMap.get(Servo.class, "Claw");
        LinearArmObj = hardwareMap.get(DcMotor.class, "LinearArm");
        ArmStop = hardwareMap.get(TouchSensor.class, "ArmStop");
        armObj.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armObj.setTargetPosition(0);
        armObj.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        carousel.setDirection(DcMotor.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each motor to save power level for telemetry
        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;
        double carouselPower;
        double fltForward;
        double fltStrafe;
        double fltPivot;
        double claw;
        double arm;
        double LinearArm;
        int intArmPosition;
        int intArmPositionPick;
        int intArmPositionDropMid;
        int intArmPositionDropUp;
        int intArmPositionDrive;
        double YTimer;
        double YTimerTwo;
        ElapsedTime ElapsedTime2;
        int ghostingTime;
        double fltNormalFactor;



        intArmPosition = 0;
        intArmPositionPick = 5300;
        intArmPositionDropMid = 4200;
        intArmPositionDrive = 2000;
        intArmPositionDropUp = 3200;




        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        YTimer = 0;
        YTimerTwo = 0;
        ElapsedTime2 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ghostingTime = 200;
        fltNormalFactor = 0.4;

        fltForward = -gamepad1.left_stick_y;
        fltStrafe = gamepad1.left_stick_x;
        fltPivot = gamepad1.right_stick_x;


        frontLeftPower    = Range.clip(drive + turn, -0.3, 0.3) ;
        frontRightPower   = Range.clip(drive - turn, -0.3, 0.3) ;
        backLeftPower    = Range.clip(drive + turn, -0.3, 0.3) ;
        backRightPower   = Range.clip(drive - turn, -0.3, 0.3) ;

        if (gamepad1.left_bumper) {
            carousel.setPower(-0.7);
        } else {
            carousel.setPower(0);
        }

        if (gamepad1.right_bumper) {
            carousel.setPower(0.5);
        } else {
            carousel.setPower(0);
        }
        if (gamepad2.b) {
            if (ElapsedTime2.milliseconds() > YTimer) {
                if (armObj.getTargetPosition() == intArmPositionDropUp) {
                    armObj.setTargetPosition(intArmPositionDropMid);
                    armObj.setPower(1);
                } else {
                    armObj.setTargetPosition(intArmPositionDropUp);
                    armObj.setPower(1);
                }
            }
            YTimer = ElapsedTime2.milliseconds() + ghostingTime;
        }
        if (gamepad2.y) {
            if (ElapsedTime2.milliseconds() > YTimer) {
                    armObj.setTargetPosition(intArmPositionDrive);
                    armObj.setPower(1);
                }

            YTimer = ElapsedTime2.milliseconds() + ghostingTime;
        }
        if (gamepad2.a) {
            if (ElapsedTime2.milliseconds() > YTimer) {
                armObj.setTargetPosition(intArmPositionPick);
                armObj.setPower(1);
            }

            YTimer = ElapsedTime2.milliseconds() + ghostingTime;
        }
        if (gamepad2.x) {
            if (ElapsedTime2.milliseconds() > YTimer) {
                armObj.setTargetPosition(intArmPosition);
                armObj.setPower(-1);
            }

            YTimer = ElapsedTime2.milliseconds() + ghostingTime;
        }

        /*if (gamepad2.x) {
            while (!ArmStop.isPressed()) {
                armObj.setPower(-0.3);
            }
            armObj.setPower(0);
            intArmPosition = 0;
        }*/


        if (gamepad2.right_bumper) {
            clawObj.setPosition(0.6);
        } else {
            clawObj.setPosition(0.2);
        }

        if (gamepad2.dpad_up) {
            LinearArmObj.setPower(1);
        } else {
            LinearArmObj.setPower(0);
        }

        if (gamepad2.dpad_down) {
            LinearArmObj.setPower(-1);
        } else {
            LinearArmObj.setPower(0);
        }

        if (gamepad1.right_trigger > fltNormalFactor) {
            fltForward = gamepad1.right_trigger * -gamepad1.left_stick_y;
            fltStrafe = gamepad1.right_trigger * gamepad1.left_stick_x;
            fltPivot = gamepad1.right_trigger * gamepad1.right_stick_x;
        } else {
            fltForward = fltNormalFactor * -gamepad1.left_stick_y;
            fltStrafe = fltNormalFactor * gamepad1.left_stick_x;
            fltPivot = fltNormalFactor * gamepad1.right_stick_x;
        }

        // Send calculated power to wheels
        frontLeftDrive.setPower(fltForward + fltStrafe + fltPivot);
        frontRightDrive.setPower(fltForward + -fltStrafe + -fltPivot);
        backLeftDrive.setPower(fltForward + -fltStrafe + fltPivot);
        backRightDrive.setPower(fltForward + fltStrafe + -fltPivot);



        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
