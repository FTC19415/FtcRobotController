
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.cv.CameraPosition;
import org.firstinspires.ftc.teamcode.cv.FtcCamera;
import org.firstinspires.ftc.teamcode.cv.OpenCVWrapper;
import org.firstinspires.ftc.teamcode.cv.TeamMarkerPosition;
import org.firstinspires.ftc.teamcode.cv.TeamMarkerPositionDetector;
import org.firstinspires.ftc.teamcode.cv.Webcam;
import org.opencv.core.Mat;

import java.io.PrintWriter;
import java.io.StringWriter;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;



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

@Autonomous(name="DEV Autonomous 01/01", group="Linear Opmode")
@Disabled
public class BasicOpMode_Linear_Autonomous_20220101 extends LinearOpMode {

    // Declare OpMode members.
    public final FtcCamera webcam = new Webcam();
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
    String simpWhereT0 = null;
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

       // initVuforia();
       // initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/


        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        carousel.setDirection(DcMotor.Direction.FORWARD);


        // Input starting position and alliance color by using the controllers
        while (!(gamepad1.left_bumper)) {
            if (autonomousSimplicity) {
                telemetry.addData("Autonomous Simplicity:", autonomousSimplicity);
                telemetry.addData("Autonomous Where to:", simpWhereT0);
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
                while (!(gamepad2.x || gamepad2.y)) {
                    telemetry.addData("To change color to blue press x : ", "To switch color to red press y");
                    //if x is pressed the color changes to blue, if y pressed the color red
                    if (gamepad1.y) {
                        // Expect values of red or blue. LOWER CASE!
                        AllianceColor = "red";
                    } else if (gamepad1.x) {
                        // Expect values of red or blue. LOWER CASE!
                        AllianceColor = "blue";
                    }
                    telemetry.addData("Alliance Color:", AllianceColor);
                    telemetry.update();
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
                if (AllianceColor == "red") {
                    while (!(gamepad1.dpad_up || gamepad1.dpad_right || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.y || gamepad1.b || gamepad1.a)) {

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
            } else {
                telemetry.addData("Run Autonomous:", runAutonomous);
            }
        }

        if (AllianceColor == "red") {

            if (autonomousSelection == 1) {
                StartPosition = 1;

            } else if (autonomousSelection == 2) {
                StartPosition = 1;
                wrsMove = true;

            } else if (autonomousSelection == 3) {
                StartPosition = 2;
                crsRoute = "barrier";

            } else if (autonomousSelection == 4) {
                StartPosition = 2;
                crsRoute = "wall";

            } else if (autonomousSelection == 5) {
                autonomousSimplicity = true;
                simpWhereT0 = "warehouse";

            } else if (autonomousSelection == 6) {
                autonomousSimplicity = true;
                simpWhereT0 = "warehouse";
                wrsDelay = true;

            } else if (autonomousSelection == 7) {
                autonomousSimplicity = true;
                simpWhereT0 = "storage unit";
            }
        } else if (AllianceColor == "blue") {

            if (autonomousSelection == 8) {
                StartPosition = 3;

            } else if (autonomousSelection == 9) {
                StartPosition = 3;
                wrsMove = true;

            } else if (autonomousSelection == 10) {
                StartPosition = 4;
                crsRoute = "barrier";

            } else if (autonomousSelection == 11) {
                StartPosition = 4;
                crsRoute = "wall";

            } else if (autonomousSelection == 12) {
                autonomousSimplicity = true;
                simpWhereT0 = "warehouse";

            } else if (autonomousSelection == 13) {
                autonomousSimplicity = true;
                simpWhereT0 = "warehouse";
                wrsDelay = true;

            } else if (autonomousSelection == 14) {
                simpWhereT0 = "storage unit";
            }
        }

        if (autonomousSimplicity) {
            telemetry.addData("Autonomous Simplicity:", autonomousSimplicity);
            telemetry.addData("Autonomous Where to:", simpWhereT0);
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

        clawObj.setPosition(.7);


        telemetry.addData("Autonomous Selected is: ", autonomousSelection);

        if (autonomousSimplicity == true) {
            telemetry.addData("Autonomous Simplicity:", autonomousSimplicity);
            telemetry.addData("Autonomous Where to:", simpWhereT0);
            telemetry.addData("Warehouse Delay:", wrsDelay);
        } else if (autonomousSimplicity == false) {
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


// TODO: Make an FtcCamera WebCam here and initialize it
        OpenCVWrapper.load();
        webcam.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // TODO: Start the camera and call grabFrame, process the frame, and call deinit
        AtomicReference<Mat> img = new AtomicReference<>();
        AtomicReference<TeamMarkerPosition> teamMarkerPosition =
                new AtomicReference<>(TeamMarkerPosition.RIGHT);
//        Thread worker =
//                new Thread(
//                        () -> {
//                            webcam.start();
//                            img.set(webcam.grabFrame());
//                        });
//        worker.start();
//
//        try {
//            worker.join();
//            worker =
//                    new Thread(
//                            () ->
//                                    teamMarkerPosition.set(
//                                            new TeamMarkerPositionDetector()
//                                                    .calculateTeamMarkerPosition(
//                                                            img.get(), CameraPosition.SIDE)));
//            worker.start();
//        } catch (InterruptedException e) {
//            StringWriter sw = new StringWriter();
//            PrintWriter pw = new PrintWriter(sw);
//            e.printStackTrace(pw);
//        }
//        try {
//            worker.join();
//        } catch (InterruptedException e) {
//            StringWriter sw = new StringWriter();
//            PrintWriter pw = new PrintWriter(sw);
//            e.printStackTrace(pw);
//        }
        webcam.start();
        img.set(webcam.grabFrame());

        teamMarkerPosition.set(
                new TeamMarkerPositionDetector()
                        .calculateTeamMarkerPosition(
                                img.get(), CameraPosition.SIDE));


            switch (teamMarkerPosition.get()) {
                case LEFT:
                    elementDropLevel = 3;
                    break;
                case CENTER:
                    elementDropLevel = 2;
                    break;
                case RIGHT:
                    elementDropLevel = 1;
                    break;
            }
        runtime.reset();
        telemetry.addData("Drop Level:", teamMarkerPosition.get());
        telemetry.update();

        if (elementDropLevel == 1) {

            elementDropLevelDegrees = 3600;

        }else if (elementDropLevel == 2) {

            elementDropLevelDegrees = 4700;

        }else if (elementDropLevel == 3){

            elementDropLevelDegrees = 6000;

        }else{
            elementDropLevelDegrees = 3600;
        }

        //setting a timer to reference later
        ElapsedTime2 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


        if (AllianceColor == "red") {
            sleep(3000);
            if (autonomousSimplicity) {
                if (simpWhereT0 == "warehouse") {
                    if (wrsDelay) {
                        sleep(6000);
                        move_forward(1, 400);
                    }
                    move_forward(1, 700);
                } else if (simpWhereT0 == "storage unit") {
                    strafe(.5, 500, "right");
                    move_forward(-.5, 1000);
                    turn(.5, 500, "right");
                    move_forward(-.5, 400);

                }
            } else if (StartPosition == 1) {
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
                while (!ArmStop.isPressed()) {
                    armObj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    armObj.setPower(-.6);
                }
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
                armObj.setTargetPosition(elementDropLevelDegrees);
                armObj.setPower(1);
                strafe(0.5, 650, "left");
                while (!ArmStop.isPressed()) {
                    armObj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    armObj.setPower(-.6);
                }

                if (crsRoute == "barrier") {
                    move_forward(1, 1950);
                } else {
                    move_forward(.7, 300);
                    strafe(.7, 1500, "left");
                    move_forward(.75, 1700);
                }
            }
        }

        if (AllianceColor == "blue") {
//            strafe(.5, 600, "left");
//            strafe(.5, 400, "right");
//            //findElement();
            sleep(3000);
            turn(.5, 1200, "right");
            strafe(.5, 600, "right");
            if (autonomousSimplicity) {
                if (simpWhereT0 == "warehouse") {
                    if (wrsDelay) {
                        sleep(6000);
                        move_forward(1, 400);
                    }
                    move_forward(1, 700);
                } else if (simpWhereT0 == "storage unit") {
                    strafe(.5, 500, "left");
                    move_forward(-.5, 1000);
                    turn(.5, 500, "left");
                    move_forward(-.3, 400);

                }
            } else if (StartPosition == 3) {
                //Warehouse Code: Drive into the warehouse
                move_forward(-.5, 600);
                turn(0.5, 550, "right");
                armObj.setTargetPosition(elementDropLevelDegrees);
                armObj.setPower(1);
                sleep(1000);
                move_forward(-0.3, 900);
                move_forward(0.3, 1100);
                sleep(2000);
                clawObj.setPosition(.9);
                sleep(500);
                move_forward(-0.5, 1050);
                turn(0.5, 700, "left");
                while (!ArmStop.isPressed()) {
                    armObj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    armObj.setPower(-.6);
                }
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


                move_forward(-0.2, 1000);

                run_carousel(0.3, 3100);


                turn(0.5, 600, "right");

                //move to blue box
                frontLeftDrive.setPower(0.5);
                frontRightDrive.setPower(0.5);
                backLeftDrive.setPower(0.5);
                backRightDrive.setPower(0.5);

                sleep(1000);

                frontLeftDrive.setPower(0);
                frontRightDrive.setPower(0);
                backLeftDrive.setPower(0);
                backRightDrive.setPower(0);


                turn(0.5, 600, "left");
                move_forward(-.3, 700);
                move_forward(.3, 2100);
                clawObj.setPosition(.9);
                sleep(500);
                move_forward(-.3, 2500);
                armObj.setTargetPosition(elementDropLevelDegrees);
                armObj.setPower(1);
                strafe(0.5, 1100, "right");
                while (!ArmStop.isPressed()) {
                    armObj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    armObj.setPower(-.6);
                }

                if (crsRoute == "barrier") {
                    frontLeftDrive.setPower(0.7);
                    frontRightDrive.setPower(0.8);
                    backLeftDrive.setPower(0.7);
                    backRightDrive.setPower(0.8);

                    sleep(1000);

                    frontLeftDrive.setPower(1);
                    frontRightDrive.setPower(0.9);
                    backLeftDrive.setPower(1);
                    backRightDrive.setPower(0.9);

                    sleep(1000);

                    frontLeftDrive.setPower(0);
                    frontRightDrive.setPower(0);
                    backLeftDrive.setPower(0);
                    backRightDrive.setPower(0);
                } else {
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

        onStop();


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


//    //@Override
    public void onStop() {
        webcam.deinit();
    }
}