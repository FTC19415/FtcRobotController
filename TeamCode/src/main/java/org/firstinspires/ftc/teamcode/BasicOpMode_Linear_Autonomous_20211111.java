/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
//import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Autonomous 11", group="Linear Opmode")
@Disabled
public class BasicOpMode_Linear_Autonomous_20211111 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor carousel = null;
    private DcMotor arm = null;
    private DcMotor linearArm = null;
    private Servo clawObj = null;
    double frontLeftPower;
    double frontRightPower;
    double backLeftPower;
    double backRightPower;
    double carouselPower;
    private DcMotor armObj = null;
    private TouchSensor ArmStop;
    ElapsedTime ElapsedTime2;
    //private BNO055IMU imu;

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    private static final String VUFORIA_KEY =
            "ATk0yPv/////AAABmfvNKymosUClk3BRqGsFS8l8HTVSGSZ50i5mz4JRA23xJ4/ZYY6Ml1JabRGIg9gh7cuejoL8pqxMFL0vvMdASvosvWCL2z9LMyfeN/4ronRK/QM6QCSTEpjKGSZq8PIswC1jfYRC4hs3oGC0pLA1i2FL8waldHko/cdFu9kWjroE8RFvi92f9TjvZ7TjnjCug6ogHqeiGCqXCCZF1YHftkwxkpcv1jJNKB417r7Jqew0vBn3YlyWlMDyEba6RrA3oLsR44P13dirYt8DWBO8AGfQXHZ34b8/cY3p0PDNGEO6vUZit4KM79LgUCyxwl5SRiuSXLi1IEIo7B8R6ypQ5WlfNOZ5UR26SIGcgAxa32Rx";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        String AllianceColor = null;
        int StartPosition = 0;
       // BNO055IMU.Parameters IMU_Parameters;

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightMotor");
        carousel = hardwareMap.get(DcMotor.class, "carousel");
        clawObj = hardwareMap.get(Servo.class, "Claw");
        armObj = hardwareMap.get(DcMotor.class, "Arm");
        ArmStop = hardwareMap.get(TouchSensor.class, "ArmStop");

        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1, 24.0/4.0);
        }



        // imu = hardwareMap.get(BNO055IMU.class, "imu");

        //initialize the camera

        //run the camera to see where the duck is and set the variable


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        //arm.setMode(DcMotor.RunMode.RUN_WITH_ENCODER);
        //frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       // backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       // backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        carousel.setDirection(DcMotor.Direction.FORWARD);

        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //arm.setTargetPosition(0);
       //arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Input starting position and alliance color by usuing the controllers
        while (!(gamepad1.y)) {
            while (!(gamepad1.a || gamepad1.b)) {
                telemetry.addData("To change color to blue press A : ", "To switch color to red press B");
                //if A is pressed the color changes to blue, if B pressed the color red
                if (gamepad1.a) {
                    // Expect values of red or blue. LOWER CASE!
                    AllianceColor = "blue";
                } else if (gamepad1.b) {
                    // Expect values of red or blue. LOWER CASE!
                    AllianceColor = "red";
                }
                telemetry.update();
            }
            while (!(gamepad1.dpad_up || gamepad1.dpad_right || gamepad1.dpad_down || gamepad1.dpad_left)) {
                telemetry.addData("To switch starting positions Use the D-Pad:", "Up is 1, right is 2, down is 3, left is 4");
                if (gamepad1.dpad_up) {
                    // red side closest to the warehouse
                    StartPosition = 1;
                } else if (gamepad1.dpad_right) {
                    // red side closest to the carousel
                    StartPosition = 2;
                } else if (gamepad1.dpad_down) {
                    // blue side closest to the warehouse
                    StartPosition = 3;
                } else if (gamepad1.dpad_left) {
                    // blue side closest to the carousel
                    StartPosition = 4;
                }
                telemetry.addData("Alliance Color:", AllianceColor);
                telemetry.addData("Starting Position:", StartPosition);
                telemetry.update();
            }
            telemetry.addData("Alliance Color:", AllianceColor);
            telemetry.addData("Starting Position:", StartPosition);
            telemetry.update();
        }
        telemetry.addData("Alliance Color:", AllianceColor);
        telemetry.addData("Starting Position:", StartPosition);
        telemetry.update();

        clawObj.setPosition(.7);

        //initializing parameters
        /*IMU_Parameters = new BNO055IMU.Parameters();
        IMU_Parameters.mode = BNO055IMU.SensorMode.IMU;
        imu.initialize(IMU_Parameters);
        telemetry.addData("Status", "IMU initialized, calibration started");
        telemetry.update();
        sleep(1000);
        while (!IMU_Calibrated()) {
            telemetry.addData("If Calibration", "doesn't complete after 3 seconds, move through 9");
            telemetry.update();
            sleep(1000);
            */
        if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                      telemetry.addData("# Object Detected", updatedRecognitions.size());

                      // step through the list of recognitions and display boundary info.
                      int i = 0;
                      for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                          recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        i++;
                      }
                      telemetry.update();
                    }
                }

            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            runtime.reset();


                //setting a timer to reference later
                ElapsedTime2 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


                if (AllianceColor == "red") {
                    if (StartPosition == 1) {
                        //Warehouse Code that drives into the warehouse
                        move_forward(0.7, 1000);

                    } else if (StartPosition == 2) {
                        // Carousel code that drives to the carousel and goes into the red box
                        move_forward(-0.4, 700);

                        //Putting motor on carousel
                        frontLeftDrive.setPower(-.2);
                        frontRightDrive.setPower(.5);
                        backLeftDrive.setPower(-0.2);
                        backRightDrive.setPower(-.2);

                        sleep(1900);

                        frontLeftDrive.setPower(0);
                        frontRightDrive.setPower(0);
                        backLeftDrive.setPower(0);
                        backRightDrive.setPower(0);

                        run_carousel(-.3, 6000);

                        //turning to wall
                        frontLeftDrive.setPower(-.5);
                        frontRightDrive.setPower(.5);
                        backLeftDrive.setPower(-0.5);
                        backRightDrive.setPower(-.5);

                        sleep(250);

                        frontLeftDrive.setPower(0);
                        frontRightDrive.setPower(0);
                        backLeftDrive.setPower(0);
                        backRightDrive.setPower(0);

                        //driving into the box
                        frontLeftDrive.setPower(0.5);
                        frontRightDrive.setPower(0.5);
                        backLeftDrive.setPower(0.5);
                        backRightDrive.setPower(0.5);

                        sleep(900);

                        frontLeftDrive.setPower(0);
                        frontRightDrive.setPower(0);
                        backLeftDrive.setPower(0);
                        backRightDrive.setPower(0);

                    }
                }
                if (AllianceColor == "blue") {
                    if (StartPosition == 3) {
                        //Warehouse Code: Drive into the warehouse
                        move_forward(0.7, 1000);

                    } else if (StartPosition == 4) {
                        // Courosel code: drop the duck and park in red box
                        move_forward(-0.4, 700);

                        //putting wheel to the carousel
                        frontLeftDrive.setPower(-0.4);
                        frontRightDrive.setPower(0.5);
                        backLeftDrive.setPower(-0.4);
                        backRightDrive.setPower(-0.4);

                        sleep(1000);

                        frontLeftDrive.setPower(0);
                        frontRightDrive.setPower(0);
                        backLeftDrive.setPower(0);
                        backRightDrive.setPower(0);

                        run_carousel(0.3, 6000);

                        //get to wall
                        frontLeftDrive.setPower(-.5);
                        frontRightDrive.setPower(.5);
                        backLeftDrive.setPower(-0.5);
                        backRightDrive.setPower(-.5);

                        sleep(1100);

                        frontLeftDrive.setPower(0);
                        frontRightDrive.setPower(0);
                        backLeftDrive.setPower(0);
                        backRightDrive.setPower(0);

                        //move to blue box
                        frontLeftDrive.setPower(-0.5);
                        frontRightDrive.setPower(-0.5);
                        backLeftDrive.setPower(-0.5);
                        backRightDrive.setPower(-0.5);

                        sleep(500);

                        frontLeftDrive.setPower(0);
                        frontRightDrive.setPower(0);
                        backLeftDrive.setPower(0);
                        backRightDrive.setPower(0);
                    }
                }

        while (!ArmStop.isPressed()) {
            armObj.setPower(-.6);
        }


                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Motors", "left (%.2f), right (%.2f)", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
                telemetry.update();
            }



    // This function is to move the robot forward or backward
    private void move_forward(double fwrdSpeed, int fwrdTime) {

        //setting motors to speed

        telemetry.addData("Motor on", "Run Time" + ElapsedTime2);
        telemetry.update();
        frontLeftPower = fwrdSpeed;
        frontRightPower = fwrdSpeed;
        backLeftPower = fwrdSpeed;
        backRightPower = fwrdSpeed;

        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);

        sleep(fwrdTime);

        telemetry.addData("Motor off", "Run Time" + ElapsedTime2);
        telemetry.update();
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);

    }

    // This function is to use the carousel. Input speed and time
    private void run_carousel(double trnSpeed, int trnTime) {
        carouselPower = trnSpeed;

        carousel.setPower(carouselPower);

        sleep(trnTime);

        carousel.setPower(0);
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    /*private void move_forward(double fwrdSpeed, int fwrdTime) {
        double forwardEndTime;
        double constCorrectionPercentage;
        float YawAngle;

        // This function is to move the robot forward or backward
        // If time is present, time will be used
        // Gyro Code
        forwardEndTime = ElapsedTime2.milliseconds() + fwrdTime;
        frontLeftPower = fwrdSpeed;
        frontRightPower = fwrdSpeed;
        backLeftPower = fwrdSpeed;
        backRightPower = fwrdSpeed;
        constCorrectionPercentage = 0.8;
        if (fwrdSpeed > 0) {
            while (!(ElapsedTime2.milliseconds() >= forwardEndTime || isStopRequested())) {
                YawAngle = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYX, AngleUnit.DEGREES).thirdAngle;
                telemetry.addData("Speed is", fwrdSpeed);
                telemetry.addData("Time", ElapsedTime2.milliseconds());
                telemetry.addData("Yaw angle", YawAngle);
                if (YawAngle > -2) {
                    // Turn Left
                    frontLeftPower = fwrdSpeed * constCorrectionPercentage;
                    frontRightPower = fwrdSpeed * constCorrectionPercentage;
                    backLeftPower = fwrdSpeed;
                    backRightPower = fwrdSpeed;
                    telemetry.addData("correcting", "left");
                } else if (YawAngle < 2) {
                    // Turn Right
                    frontLeftPower = fwrdSpeed;
                    frontRightPower = fwrdSpeed;
                    backLeftPower = fwrdSpeed * constCorrectionPercentage;
                    backRightPower = fwrdSpeed * constCorrectionPercentage;
                    telemetry.addData("correcting", "right");
                } else {
                    // Continue Straight
                    frontLeftPower = fwrdSpeed;
                    frontRightPower = fwrdSpeed;
                    backLeftPower = fwrdSpeed;
                    backRightPower = fwrdSpeed;
                    telemetry.addData("correcting", "straight");
                }
                telemetry.addData("Left Motor Power", frontLeftPower);
                telemetry.addData("Right Motor Power", frontRightPower);
                frontLeftDrive.setPower(frontLeftPower);
                frontRightDrive.setPower(frontRightPower);
                backLeftDrive.setPower(backLeftPower);
                backRightDrive.setPower(backRightPower);
                telemetry.update();
            }
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);
        } else if (fwrdSpeed < 0) {
            while (!(ElapsedTime2.milliseconds() >= forwardEndTime || isStopRequested())) {
                YawAngle = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYX, AngleUnit.DEGREES).thirdAngle;
                telemetry.addData("Speed is", fwrdSpeed);
                telemetry.addData("Time", ElapsedTime2.milliseconds());
                telemetry.addData("Yaw angle", YawAngle);
                if (YawAngle > -2) {
                    // Turn Left
                    frontLeftPower = fwrdSpeed * constCorrectionPercentage;
                    frontRightPower = fwrdSpeed * constCorrectionPercentage;
                    backLeftPower = fwrdSpeed;
                    backRightPower = fwrdSpeed;
                    telemetry.addData("correcting", "left");
                } else if (YawAngle < 2) {
                    // Turn Right
                    frontLeftPower = fwrdSpeed;
                    frontRightPower = fwrdSpeed;
                    backLeftPower = fwrdSpeed * constCorrectionPercentage;
                    backRightPower = fwrdSpeed * constCorrectionPercentage;
                    telemetry.addData("correcting", "right");
                } else {
                    // Continue Straight
                    frontLeftPower = fwrdSpeed;
                    frontRightPower = fwrdSpeed;
                    backLeftPower = fwrdSpeed;
                    backRightPower = fwrdSpeed;
                    telemetry.addData("correcting", "straight");
                }
                telemetry.addData("Left Motor Power", frontLeftPower);
                telemetry.addData("Right Motor Power", frontRightPower);
                frontLeftDrive.setPower(frontLeftPower);
                frontRightDrive.setPower(frontRightPower);
                backLeftDrive.setPower(backLeftPower);
                backRightDrive.setPower(backRightPower);
                telemetry.update();
            }
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);
        }
    }
*/
    //Describe this function...

    /*private boolean IMU_Calibrated() {
        telemetry.addData("IMU Calibration Status", imu.getCalibrationStatus());
        telemetry.addData("Gyro Calibrated", imu.isGyroCalibrated() ? "True" : "False");
        telemetry.addData("System Satus", imu.getSystemStatus());
        return imu.isGyroCalibrated();
    }*/
}

