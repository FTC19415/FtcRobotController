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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
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

@Autonomous(name="DEV Autonomous 12/17", group="Linear Opmode")
//@Disabled
public class BasicOpMode_Linear_Autonomous_20211217 extends LinearOpMode {

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
    boolean runAutonomous;
    String AllianceColor = null;
    boolean autonomousSimplicity;
    String simpWhereTo = null;
    boolean wrsDelay;
    boolean wrsMove;
    String crsRoute = null;
    int StartPosition = 0;
    String whatIsDetected = null;
    double elementPosition;
    int elementDropLevel;
    int elementDropLevelDegrees;
    int autonomousSelection;
    //private BNO055IMU imu;
    List<Recognition> recognitions;

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

        armObj.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armObj.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armObj.setTargetPosition(0);
        armObj.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
            tfod.setZoom(1, 9.0/5.0);
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

        // Input starting position and alliance color by using the controllers
        while (!(gamepad1.left_bumper)) {
            if (autonomousSimplicity) {
                telemetry.addData("Autonomous Simplicity:", autonomousSimplicity);
                telemetry.addData("Autonomous Where to:", simpWhereTo);
                telemetry.addData("Warehouse Delay:", wrsDelay);
            } else {
                telemetry.addData("Alliance Color:", AllianceColor);
                if (StartPosition == 1 || StartPosition == 3) {
                    telemetry.addData("Starting Position:", "Warehouse");
                    telemetry.addData("Warehouse Move:", wrsMove);
                } else if (StartPosition == 2 || StartPosition == 4) {
                    telemetry.addData("Starting Position:", "carousel");
                    telemetry.addData("Carousel Route:", crsRoute);
                }
            }
            telemetry.update();
            while (!(gamepad1.a || gamepad1.b)) {
                telemetry.addData("To run autonomous press A : ", "To do nothing press B");
                //if A is pressed the color changes to blue, if B pressed the color red
                if (gamepad1.a) {
                    // Expect values of red or blue. LOWER CASE!
                    runAutonomous = true;
                } else if (gamepad1.b) {
                    // Expect values of red or blue. LOWER CASE!
                    runAutonomous = false;
                }
                telemetry.update();
            }
            if (runAutonomous) {
                while (!(gamepad1.x || gamepad1.y)) {
                    telemetry.addData("To change color to blue press x : ", "To switch color to red press y");
                    //if A is pressed the color changes to blue, if B pressed the color red
                    if (gamepad1.x) {
                        // Expect values of red or blue. LOWER CASE!
                        AllianceColor = "blue";
                    } else if (gamepad1.y) {
                        // Expect values of red or blue. LOWER CASE!
                        AllianceColor = "red";
                    }
                    telemetry.addData("Alliance Color:", AllianceColor);
                    telemetry.update();
                }

                if (AllianceColor == "red") {
                    while (!(gamepad1.dpad_up || gamepad1.dpad_right || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.y || gamepad1.b ||gamepad1.a)) {

                        telemetry.addData("To Select Autonomous: ", "D-pad up = 1");
                        telemetry.addData("To Select Autonomous: ", "D-pad right = 2");
                        telemetry.addData("To Select Autonomous: ", "D-pad down = 3");
                        telemetry.addData("To Select Autonomous: ", "D-pad left = 4");
                        telemetry.addData("To Select Autonomous: ", "y button = 5");
                        telemetry.addData("To Select Autonomous: ", "b button = 6");
                        telemetry.addData("To Select Autonomous: ", "a button = 7");
                        if (gamepad1.dpad_up) {
                            // Expect values of red or blue. LOWER CASE!
                            autonomousSelection = 1;

                        } else if (gamepad1.dpad_right) {
                            // Expect values of red or blue. LOWER CASE!
                            autonomousSelection = 2;

                        } else if (gamepad1.dpad_down) {
                            // Expect values of red or blue. LOWER CASE!
                            autonomousSelection = 3;

                        } else if (gamepad1.dpad_left) {
                            // Expect values of red or blue. LOWER CASE!
                            autonomousSelection = 4;

                        } else if (gamepad1.y) {
                            // Expect values of red or blue. LOWER CASE!
                            autonomousSelection = 5;

                        } else if (gamepad1.b) {
                            // Expect values of red or blue. LOWER CASE!
                            autonomousSelection = 6;

                        } else if (gamepad1.a) {
                            // Expect values of red or blue. LOWER CASE!
                            autonomousSelection = 7;
                        }
                        telemetry.addData("Alliance Color:", AllianceColor);
                        telemetry.addData("Autonomous Selection:", autonomousSelection);
                        telemetry.update();

                    }
                }

                if (AllianceColor == "blue") {
                    while (!(gamepad1.dpad_up || gamepad1.dpad_right || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.y || gamepad1.b || gamepad1.a)) {

                            telemetry.addData("To Select Autonomous: ", "D-pad up = 8");
                            telemetry.addData("To Select Autonomous: ", "D-pad right = 9");
                            telemetry.addData("To Select Autonomous: ", "D-pad down = 10");
                            telemetry.addData("To Select Autonomous: ", "D-pad left = 11");
                            telemetry.addData("To Select Autonomous: ", "y button = 12");
                            telemetry.addData("To Select Autonomous: ", "b button = 13");
                            telemetry.addData("To Select Autonomous: ", "a button = 14");
                            if (gamepad1.dpad_up) {
                                // Expect values of red or blue. LOWER CASE!
                                autonomousSelection = 8;

                            } else if (gamepad1.dpad_right) {
                                // Expect values of red or blue. LOWER CASE!
                                autonomousSelection = 9;

                            } else if (gamepad1.dpad_down) {
                                // Expect values of red or blue. LOWER CASE!
                                autonomousSelection = 10;

                            } else if (gamepad1.dpad_left) {
                                // Expect values of red or blue. LOWER CASE!
                                autonomousSelection = 11;

                            } else if (gamepad1.y) {
                                // Expect values of red or blue. LOWER CASE!
                                autonomousSelection = 12;

                            } else if (gamepad1.b) {
                                // Expect values of red or blue. LOWER CASE!
                                autonomousSelection = 13;

                            } else if (gamepad1.a) {
                                // Expect values of red or blue. LOWER CASE!
                                autonomousSelection = 14;
                            }
                            telemetry.addData("Alliance Color:", AllianceColor);
                            telemetry.addData("Autonomous Selection:", autonomousSelection);
                            telemetry.update();
                        }
                    }
 //                while (!(gamepad1.a || gamepad1.b)) {
//                    telemetry.addData("To run regular autonomous press A : ", "To run simple autonomous press B");
//                    //if A is pressed the color changes to blue, if B pressed the color red
//                    if (gamepad1.a) {
//                        // Expect values of red or blue. LOWER CASE!
//                        autonomousSimplicity = false;
//                    } else if (gamepad1.b) {
//                        // Expect values of red or blue. LOWER CASE!
//                        autonomousSimplicity = true;
//                    }
//                    telemetry.addData("Alliance Color:", AllianceColor);
//                    telemetry.addData("Autonomous Simplicity:", autonomousSimplicity);
//                    telemetry.addData("Autonomous Where to:", simpWhereTo);
//                    telemetry.addData("Warehouse Delay:", wrsDelay);
//                    telemetry.update();
//                }
//                if (autonomousSimplicity) {
//                    while (!(gamepad1.x || gamepad1.y)) {
//                        telemetry.addData("Go to warehouse press x : ", "Go to Storage Unit press y");
//                        //if A is pressed the color changes to blue, if B pressed the color red
//                        if (gamepad1.x) {
//                            // Expect values of red or blue. LOWER CASE!
//                            simpWhereTo = "warehouse";
//                        } else if (gamepad1.y) {
//                            // Expect values of red or blue. LOWER CASE!
//                            simpWhereTo = "storage unit";
//                        }
//                        telemetry.addData("Alliance Color:", AllianceColor);
//                        telemetry.addData("Autonomous Simplicity:", autonomousSimplicity);
//                        telemetry.addData("Autonomous Where to:", simpWhereTo);
//                        telemetry.addData("Warehouse Delay:", wrsDelay);
//                        telemetry.update();
//                    }
//                    if (simpWhereTo == "warehouse") {
//                        while (!(gamepad1.a || gamepad1.b)) {
//                            telemetry.addData("Need a Delay press A : ", "Don't need a Delay press B");
//                            //if A is pressed the color changes to blue, if B pressed the color red
//                            if (gamepad1.a) {
//                                // Expect values of red or blue. LOWER CASE!
//                                wrsDelay = true;
//                            } else if (gamepad1.b) {
//                                // Expect values of red or blue. LOWER CASE!
//                                wrsDelay = false;
//                            }
//                            telemetry.addData("Alliance Color:", AllianceColor);
//                            telemetry.addData("Autonomous Simplicity:", autonomousSimplicity);
//                            telemetry.addData("Autonomous Where to:", simpWhereTo);
//                            telemetry.addData("Warehouse Delay:", wrsDelay);
//                            telemetry.update();
//                        }
//                    }
//                    if (autonomousSimplicity) {
//                        telemetry.addData("Autonomous Simplicity:", autonomousSimplicity);
//                        telemetry.addData("Autonomous Where to:", simpWhereTo);
//                        telemetry.addData("Warehouse Delay:", wrsDelay);
//                    }
//                } else {
//                    while (!(gamepad1.dpad_up || gamepad1.dpad_down)) {
//                        telemetry.addData("To switch starting positions Use the D-Pad:", "Up is warehouse, down is carousel");
//                        if (AllianceColor == "red") {
//                            if (gamepad1.dpad_up) {
//                                // red side closest to the warehouse
//                                StartPosition = 1;
//                            } else if (gamepad1.dpad_down) {
//                                // red side closest to the carousel
//                                StartPosition = 2;
//                            }
//                        } else if (AllianceColor == "blue") {
//                            if (gamepad1.dpad_up) {
//                                // blue side closest to the warehouse
//                                StartPosition = 3;
//                            } else if (gamepad1.dpad_down) {
//                                // blue side closest to the carousel
//                                StartPosition = 4;
//                            }
//                        }
//                        telemetry.addData("Alliance Color:", AllianceColor);
//                        if (StartPosition == 1 || StartPosition == 3) {
//                            telemetry.addData("Starting Position:", "Warehouse");
//                            telemetry.addData("Warehouse Move:", wrsMove);
//                        } else if (StartPosition == 2 || StartPosition == 4) {
//                            telemetry.addData("Starting Position:", "carousel");
//                            telemetry.addData("Carousel Route:", crsRoute);
//                        }
//                        telemetry.update();
//                    }
//                    if (StartPosition == 1 || StartPosition == 3) {
//                        while (!(gamepad1.a || gamepad1.b)) {
//                            telemetry.addData("Move over in warehouse press A : ", "Don't need to move over press B");
//                            //if A is pressed the color changes to blue, if B pressed the color red
//                            if (gamepad1.a) {
//                                // Expect values of red or blue. LOWER CASE!
//                                wrsMove = true;
//                            } else if (gamepad1.b) {
//                                // Expect values of red or blue. LOWER CASE!
//                                wrsMove = false;
//                            }
//                            telemetry.addData("Alliance Color:", AllianceColor);
//                            if (StartPosition == 1 || StartPosition == 3) {
//                                telemetry.addData("Starting Position:", "Warehouse");
//                                telemetry.addData("Warehouse Move:", wrsMove);
//                            } else if (StartPosition == 2 || StartPosition == 4) {
//                                telemetry.addData("Starting Position:", "carousel");
//                                telemetry.addData("Carousel Route:", crsRoute);
//                            }
//                            telemetry.update();
//                        }
//                    } else if (StartPosition == 2 || StartPosition == 4) {
//                        while (!(gamepad1.a || gamepad1.b)) {
//                            telemetry.addData("Use wall for carousel press A : ", "Use barriers press B");
//                            //if A is pressed the color changes to blue, if B pressed the color red
//                            if (gamepad1.a) {
//                                // Expect values of red or blue. LOWER CASE!
//                                crsRoute = "wall";
//                            } else if (gamepad1.b) {
//                                // Expect values of red or blue. LOWER CASE!
//                                crsRoute = "barrier";
//                            }
//                            telemetry.addData("Alliance Color:", AllianceColor);
//                            if (StartPosition == 1 || StartPosition == 3) {
//                                telemetry.addData("Starting Position:", "Warehouse");
//                                telemetry.addData("Warehouse Move:", wrsMove);
//                            } else if (StartPosition == 2 || StartPosition == 4) {
//                                telemetry.addData("Starting Position:", "carousel");
//                                telemetry.addData("Carousel Route:", crsRoute);
//                            }
//                            telemetry.update();
//                        }
//                    }
//                    if (autonomousSimplicity != true) {
//                        telemetry.addData("Alliance Color:", AllianceColor);
//                        if (StartPosition == 1 || StartPosition == 3) {
//                            telemetry.addData("Starting Position:", "Warehouse");
//                            telemetry.addData("Warehouse Move:", wrsMove);
//                        } else if (StartPosition == 2 || StartPosition == 4) {
//                            telemetry.addData("Starting Position:", "carousel");
//                            telemetry.addData("Carousel Route:", crsRoute);
//                        }
//                    }
//                    telemetry.update();
//                }
            } else {
                telemetry.addData("Run Autonomous:", runAutonomous);
            }

            if (AllianceColor == "red") {

                if (autonomousSelection == 1) {
                    StartPosition = 1;

                }else if (autonomousSelection == 2) {
                    StartPosition = 1;
                    wrsMove = true;

                }else if (autonomousSelection == 3) {
                    StartPosition = 2;
                    crsRoute = "barrier";

                }else if (autonomousSelection == 4){
                    StartPosition =2;
                    crsRoute = "wall";

                }else if (autonomousSelection == 5) {
                    autonomousSimplicity = true;
                    simpWhereTo = "warehouse";

                }else if (autonomousSelection == 6) {
                    autonomousSimplicity = true;
                    simpWhereTo = "warehouse";
                    wrsDelay = true;

                }else if (autonomousSelection == 7) {
                    autonomousSimplicity = true;
                    simpWhereTo = "storage unit";
                }
            }else if (AllianceColor == "blue") {

                if (autonomousSelection == 8) {
                    StartPosition = 3;

                }else if (autonomousSelection == 9) {
                    StartPosition = 3;
                    wrsMove = true;

                }else if (autonomousSelection == 10) {
                    StartPosition = 4;
                    crsRoute = "barrier";

                }else if (autonomousSelection == 11) {
                    StartPosition = 4;
                    crsRoute = "wall";

                }else if (autonomousSelection == 12) {
                    autonomousSimplicity = true;
                    simpWhereTo = "warehouse";

                }else if (autonomousSelection == 13) {
                    autonomousSimplicity = true;
                    simpWhereTo = "warehouse";
                    wrsDelay = true;

                }else if (autonomousSelection == 14) {
                    simpWhereTo = "storage unit";
                }
            }

            if (autonomousSimplicity) {
                telemetry.addData("Autonomous Simplicity:", autonomousSimplicity);
                telemetry.addData("Autonomous Where to:", simpWhereTo);
                telemetry.addData("Warehouse Delay:", wrsDelay);
            } else {
                telemetry.addData("Alliance Color:", AllianceColor);
                if (StartPosition == 1 || StartPosition == 3) {
                    telemetry.addData("Starting Position:", "Warehouse");
                    telemetry.addData("Warehouse Move:", wrsMove);
                } else if (StartPosition == 2 || StartPosition == 4) {
                    telemetry.addData("Starting Position:", "carousel");
                    telemetry.addData("Carousel Route:", crsRoute);
                }
            }
            telemetry.update();
        }

        telemetry.update();

        clawObj.setPosition(.7);


//        if (tfod != null) {
//                    // getUpdatedRecognitions() will return null if no new information is available since
//                    // the last time that call was made.
//                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//                    if (updatedRecognitions != null) {
//                      telemetry.addData("# Object Detected", updatedRecognitions.size());
//
//                      // step through the list of recognitions and display boundary info.
//                      int i = 0;
//                      for (Recognition recognition : updatedRecognitions) {
//                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
//                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
//                                          recognition.getLeft(), recognition.getTop());
//                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
//                                recognition.getRight(), recognition.getBottom());
//                        i++;
//
//                          // check label to see if the camera now sees a Cube
//                          if (recognition.getLabel().equals("Cube")) {
//                              whatIsDetected = "Cube";
//                              telemetry.addData("Object Detected", whatIsDetected);
//                              elementPosition = recognition.getLeft();
//                          } else if (recognition.getLabel().equals("Marker")) {
//                              whatIsDetected = "Marker";
//                              telemetry.addData("Object Detected", whatIsDetected);
//                              elementPosition = recognition.getLeft();
//                          }
//                      }
//                      telemetry.update();
//                    }
//                }
//
//        if (whatIsDetected == "Cube") {
//            if (elementPosition >= 500) {
//                //right most spot
//                elementDropLevel = 1;
//            }else if (elementPosition >= 200 && elementPosition <= 475) {
//                elementDropLevel = 2;
//            }
//        }else if (whatIsDetected == "Marker") {
//            elementDropLevel = 3;
//        }

        telemetry.addData("Autonomous Selected is: ", autonomousSelection);

        if (autonomousSimplicity == true) {
            telemetry.addData("Autonomous Simplicity:", autonomousSimplicity);
            telemetry.addData("Autonomous Where to:", simpWhereTo);
            telemetry.addData("Warehouse Delay:", wrsDelay);
        }else if (autonomousSimplicity == false) {
            telemetry.addData("Alliance Color:", AllianceColor);
            if (StartPosition == 1 || StartPosition == 3) {
                telemetry.addData("Starting Position:", "Warehouse");
                telemetry.addData("Warehouse Move:", wrsMove);
            } else if (StartPosition == 2 || StartPosition == 4) {
                telemetry.addData("Starting Position:", "carousel");
                telemetry.addData("Carousel Route:", crsRoute);
            }
                telemetry.addData("Object Detected", whatIsDetected);

        }


        //telemetry.addData(telemetry.addData(recognitions.getLabel);
        telemetry.addData("Status", "Initialized");
        telemetry.update();



            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            runtime.reset();


                //setting a timer to reference later
                ElapsedTime2 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


                if (AllianceColor == "red") {
                    strafe(.5, 400, "right");
                    findElement();
                    sleep(5000);
                    strafe(.5, 600, "left");
                    if (autonomousSimplicity) {
                        if (simpWhereTo == "warehouse") {
                            if (wrsDelay) {
                                sleep(6000);
                                move_forward(1, 400);
                            }
                            move_forward(1, 700);
                        } else if (simpWhereTo == "storage unit") {
                            strafe(.5, 500, "right");
                            move_forward(-.5, 1000);
                            turn(.5, 500, "right");
                            move_forward(-.5, 400);

                        }
                    }else  if (StartPosition == 1) {
                        //Warehouse Code that drives into the warehouse
                        move_forward(-.5, 600);
                        turn(0.5, 600, "left");
                        armObj.setTargetPosition(elementDropLevelDegrees);
                        armObj.setPower(1);
                        sleep(1000);
                        move_forward(-0.3, 900);
                        move_forward(0.3, 1100);
                        sleep(2000);
                        clawObj.setPosition(.9);
                        sleep(500);
                        move_forward(-0.5, 1050);
                        turn(0.5, 650, "right");
                        move_forward(.5, 2000);

                        if (wrsMove) {
                            strafe(.5, 200, "left");
                        }

                    } else if (StartPosition == 2) {
                        // Carousel code that drives to the carousel and goes into the red box
                        armObj.setTargetPosition(elementDropLevelDegrees);
                        armObj.setPower(1);
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

                        move_forward(-0.2, 100);

                        run_carousel(-.3, 6000);

                        /*
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
                        */

                        //driving into the box
                        frontLeftDrive.setPower(0.5);
                        frontRightDrive.setPower(0.5);
                        backLeftDrive.setPower(0.5);
                        backRightDrive.setPower(0.5);

                        sleep(1000);

                        frontLeftDrive.setPower(0);
                        frontRightDrive.setPower(0);
                        backLeftDrive.setPower(0);
                        backRightDrive.setPower(0);


                        turn(0.5, 600, "right");
                        move_forward(-.3, 800);
                        move_forward(.3, 1900);
                        clawObj.setPosition(.9);
                        sleep(500);
                        move_forward(-.3, 2500);
                        armObj.setTargetPosition(4800);
                        armObj.setPower(1);
                        strafe(0.5, 650, "left");

                        if (crsRoute == "barrier") {
                            move_forward(1, 1950);
                        }else {
                            move_forward(.7, 300);
                            strafe(.7, 1500, "left");
                            move_forward(.75, 1700);
                        }
                    }
                }

                if (AllianceColor == "blue") {
                    strafe(.5, 600, "left");
                    strafe(.5, 400, "right");
                    findElement();
                    sleep(5000);
                    turn(.5, 1200, "right");
                    strafe(.5, 600, "right");
                    if (autonomousSimplicity) {
                        if (simpWhereTo == "warehouse") {
                            if (wrsDelay) {
                                sleep(6000);
                                move_forward(1, 400);
                            }
                            move_forward(1, 700);
                        }else if (simpWhereTo == "storage unit") {
                            strafe(.5, 500, "left");
                            move_forward(-.5, 1000);
                            turn(.5, 500, "left");
                            move_forward(-.3, 400);

                        }
                    }else if (StartPosition == 3) {
                        //Warehouse Code: Drive into the warehouse
                        move_forward(-.5, 600);
                        turn(0.5, 550, "right");
                        armObj.setTargetPosition(elementDropLevelDegrees);
                        armObj.setPower(1);
                        sleep(1000);
                        move_forward(-0.3, 900);
                        move_forward(0.3, 1050);
                        sleep(2000);
                        clawObj.setPosition(.9);
                        sleep(500);
                        move_forward(-0.5, 1050);
                        turn(0.5, 700, "left");
                        move_forward(.5, 2000);

                        if (wrsMove) {
                            strafe(.5, 200, "right");
                        }

                    } else if (StartPosition == 4) {
                        // Courosel code: drop the duck and park in red box
                        armObj.setTargetPosition(elementDropLevelDegrees);
                        armObj.setPower(1);
                        move_forward(-0.4, 700);
                        strafe(0.5, 400, "left");

                        /*
                        //putting wheel to the carousel
                        frontLeftDrive.setPower(0.5);
                        frontRightDrive.setPower(-0.2);
                        backLeftDrive.setPower(-0.2);
                        backRightDrive.setPower(-0.2);

                        sleep(1900);

                        frontLeftDrive.setPower(0);
                        frontRightDrive.setPower(0);
                        backLeftDrive.setPower(0);
                        backRightDrive.setPower(0);
                         */

                        move_forward(-0.2, 1000);

                        run_carousel(0.3, 3000);

                        /*
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
                         */

                        turn(0.5, 600, "right");

                        //move to blue box
                        frontLeftDrive.setPower(0.5);
                        frontRightDrive.setPower(0.5);
                        backLeftDrive.setPower(0.5);
                        backRightDrive.setPower(0.5);

                        sleep(1100);

                        frontLeftDrive.setPower(0);
                        frontRightDrive.setPower(0);
                        backLeftDrive.setPower(0);
                        backRightDrive.setPower(0);

                        turn(0.5, 600, "left");
                        move_forward(-.3, 700);
                        move_forward(.3, 2150);
                        clawObj.setPosition(.9);
                        sleep(500);
                        move_forward(-.3, 2500);
                        armObj.setTargetPosition(4800);
                        armObj.setPower(1);
                        strafe(0.9, 1100, "right");

                        if (crsRoute == "barrier") {
                            move_forward(1, 1950);
                        }else {
                            move_forward(.7, 400);
                            strafe(.7, 1500, "right");
                            move_forward(.75, 1700);
                        }
                    }
                }

        while (!ArmStop.isPressed()) {
            armObj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    private void turn(double trnSpeed, int trnTime, String trnDirection) {

        //setting motors to speed

        if (trnDirection == "right"){

            telemetry.addData("Motor on", "Run Time" + ElapsedTime2);
            telemetry.update();
            frontLeftPower = trnSpeed;
            frontRightPower = -trnSpeed;
            backLeftPower = trnSpeed;
            backRightPower = -trnSpeed;

            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);

            sleep(trnTime);

            telemetry.addData("Motor off", "Run Time" + ElapsedTime2);
            telemetry.update();
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);

        }else if (trnDirection == "left") {

            telemetry.addData("Motor on", "Run Time" + ElapsedTime2);
            telemetry.update();
            frontLeftPower = -trnSpeed;
            frontRightPower = trnSpeed;
            backLeftPower = -trnSpeed;
            backRightPower = trnSpeed;

            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);

            sleep(trnTime);

            telemetry.addData("Motor off", "Run Time" + ElapsedTime2);
            telemetry.update();
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);
        }
    }

    private void strafe(double strfSpeed, int strfTime, String strfDirection) {

        //setting motors to speed

        if (strfDirection == "right"){

            telemetry.addData("Motor on", "Run Time" + ElapsedTime2);
            telemetry.update();
            frontLeftPower = -strfSpeed;
            frontRightPower = strfSpeed;
            backLeftPower = strfSpeed;
            backRightPower = -strfSpeed;

            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);

            sleep(strfTime);

            telemetry.addData("Motor off", "Run Time" + ElapsedTime2);
            telemetry.update();
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);

        }else if (strfDirection == "left") {

            telemetry.addData("Motor on", "Run Time" + ElapsedTime2);
            telemetry.update();
            frontLeftPower = strfSpeed;
            frontRightPower = -strfSpeed;
            backLeftPower = -strfSpeed;
            backRightPower = strfSpeed;

            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);

            sleep(strfTime);

            telemetry.addData("Motor off", "Run Time" + ElapsedTime2);
            telemetry.update();
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);
        }
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
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.

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

    private void findElement() {
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

                    // check label to see if the camera now sees a Cube
                    if (recognition.getLabel().equals("Cube")) {
                        whatIsDetected = "Cube";
                        telemetry.addData("Object Detected", whatIsDetected);
                        elementPosition = recognition.getLeft();
                        telemetry.addData("Element Position", elementPosition);
                    } else if (recognition.getLabel().equals("Marker")) {
                        whatIsDetected = "Marker";
                        telemetry.addData("Object Detected", whatIsDetected);
                        elementPosition = recognition.getLeft();
                        telemetry.addData("Element Position", elementPosition);
                    }
                    telemetry.update();
                }
                telemetry.update();
            }
        }

        if (AllianceColor == "red") {
            if (whatIsDetected == "Cube") {
                if (elementPosition >= 600) {
                    //right most spot
                    elementDropLevel = 1;
                } else if (elementPosition >= 200 && elementPosition <= 575) {
                    elementDropLevel = 2;
                } else if (elementPosition <= 190) {
                    elementDropLevel = 3;
                }
            } else if (whatIsDetected == "Marker") {
                if (elementPosition <= 190) {
                    elementDropLevel = 3;
                }
            }
        }else if (AllianceColor == "blue")
            if (whatIsDetected == "Cube") {
                if (elementPosition <= 190) {
                    //right most spot
                    elementDropLevel = 1;
                } else if (elementPosition >= 200 && elementPosition <= 400) {
                    elementDropLevel = 2;
                } else if (elementPosition >= 500) {
                    elementDropLevel = 3;
                }
            } else if (whatIsDetected == "Marker") {
                if (elementPosition >= 600) {
                    elementDropLevel = 3;
                }
            }

        if (elementDropLevel == 1) {

            elementDropLevelDegrees = 3700;

        }else if (elementDropLevel == 2) {

            elementDropLevelDegrees = 4800;

        }else if (elementDropLevel == 3){

            elementDropLevelDegrees = 5600;

        }else{
            elementDropLevelDegrees = 3700;
        }

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

