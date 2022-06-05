
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
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

@Autonomous(name="Autonomous 01/18", group="Linear Opmode")
@Disabled
public class BasicOpMode_Linear_Autonomous_20220118 extends LinearOpMode {

    // Declare OpMode members.
    public final FtcCamera webcam = new Webcam();
    private Camera camera;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor carousel = null;
    private DcMotor arm = null;
    private DcMotor linearArm = null;
    private Servo clawObj = null;
    private Servo wristLPos = null;
    private Servo wristRNeg = null;
    private DcMotor turretObj = null;
    double dblWristPosition;
    private double wristMax = 0.858;
    private double wristMin = 0.204;
    double frontLeftPower;
    double frontRightPower;
    double backLeftPower;
    double backRightPower;
    double carouselPower;
    private DcMotor armObj = null;
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
    private CameraCaptureSession session;

    private static final String VUFORIA_KEY =
            "ATk0yPv/////AAABmfvNKymosUClk3BRqGsFS8l8HTVSGSZ50i5mz4JRA23xJ4/ZYY6Ml1JabRGIg9gh7cuejoL8pqxMFL0vvMdASvosvWCL2z9LMyfeN/4ronRK/QM6QCSTEpjKGSZq8PIswC1jfYRC4hs3oGC0pLA1i2FL8waldHko/cdFu9kWjroE8RFvi92f9TjvZ7TjnjCug6ogHqeiGCqXCCZF1YHftkwxkpcv1jJNKB417r7Jqew0vBn3YlyWlMDyEba6RrA3oLsR44P13dirYt8DWBO8AGfQXHZ34b8/cY3p0PDNGEO6vUZit4KM79LgUCyxwl5SRiuSXLi1IEIo7B8R6ypQ5WlfNOZ5UR26SIGcgAxa32Rx";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;




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
        turretObj = hardwareMap.get(DcMotor.class, "turret");
        wristLPos = hardwareMap.get(Servo.class, "LeftWrist");
        wristRNeg = hardwareMap.get(Servo.class, "RightWrist");


        armObj.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armObj.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armObj.setTargetPosition(0);
        armObj.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/

        initVuforia();

        deinit();

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

        //Make an FtcCamera WebCam here and initialize it
        OpenCVWrapper.load();
        webcam.init(hardwareMap);


        //telemetry.addData(telemetry.addData(recognitions.getLabel);
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
// Start the camera and call grabFrame, process the frame, and call deinit
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


        //assigning the drop level based on what the camera sees
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

        //setting the arm degree position based on the drop level
        if (elementDropLevel == 1) {

            elementDropLevelDegrees = 4500;

        }else if (elementDropLevel == 2) {

            elementDropLevelDegrees = 5570;

        }else if (elementDropLevel == 3){

            elementDropLevelDegrees = 6560;
            //0.4

        }else{
            elementDropLevelDegrees = 4500;
        }

        //setting a timer to reference later
        ElapsedTime2 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


        if (AllianceColor == "red") {
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
                move_forward(-.6, 600);
                turn(0.5, 675, "left");
                armObj.setTargetPosition(elementDropLevelDegrees);
                armObj.setPower(1);
                if (elementDropLevel == 3 ){
                    setWristPosition(0.4);
                    sleep(1000);
                }else {
                    setWristPosition(0.5);
                }
                sleep(1000);
                move_forward(-0.3, 1000);
                move_forward(0.35, 1100);
                sleep(1000);
                clawObj.setPosition(.9);
                sleep(500);
                move_forward(-0.5, 1050);
                turn(0.5, 850, "right");
                armObj.setTargetPosition(1000);
                armObj.setPower(1);
                move_forward(.5, 2000);

                if (wrsMove) {
                    strafe(.5, 900, "right");
                }

            } else if (StartPosition == 2) {
                // Carousel code that drives to the carousel and goes into the red box
                armObj.setTargetPosition(elementDropLevelDegrees);
                armObj.setPower(1);
                if (elementDropLevel == 3 ){
                    setWristPosition(0.4);
                }else {
                    setWristPosition(0.5);
                }
                move_forward(-0.5, 750);

                //Putting motor on carousel
                frontLeftDrive.setPower(-.25);
                frontRightDrive.setPower(.55);
                backLeftDrive.setPower(-0.25);
                backRightDrive.setPower(-.25);

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


                turn(0.5, 650, "right");
                move_forward(-.3, 800);
                move_forward(.3, 1850);
                clawObj.setPosition(.9);
                sleep(500);
                move_forward(-.3, 2500);
                armObj.setTargetPosition(1000);
                armObj.setPower(1);
                strafe(0.5, 1600, "left");

                if (crsRoute == "barrier") {
                    turn(0.3, 125, "right");
                    move_forward(.5, 4500);
                } else {
                    move_forward(.5, 300);
                    strafe(.5, 1500, "left");
                    move_forward(.5, 3500);
                }
            }
        }

        if (AllianceColor == "blue") {
            turn(.7, 900, "right");
            strafe(.6, 900, "right");
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
                armObj.setTargetPosition(elementDropLevelDegrees);
                armObj.setPower(1);
                if (elementDropLevel == 3 ){
                    setWristPosition(0.4);
                }else {
                    setWristPosition(0.5);
                }
                turn(0.5, 675, "right");
                sleep(1000);
                move_forward(-0.3, 900);
                move_forward(0.3, 1100);
                sleep(2000);
                clawObj.setPosition(.9);
                sleep(500);
                move_forward(-0.5, 1050);
                turn(0.5, 850, "left");
                armObj.setTargetPosition(1000);
                armObj.setPower(1);
                move_forward(.5, 2000);

                if (wrsMove) {
                    strafe(.5, 700, "left");
                }

            } else if (StartPosition == 4) {
                // Courosel code: drop the duck and park in red box
                armObj.setTargetPosition(elementDropLevelDegrees);
                armObj.setPower(1);
                if (elementDropLevel == 3 ){
                    setWristPosition(0.4);
                }else {
                    setWristPosition(0.5);
                }
                move_forward(-0.4, 700);
                strafe(0.5, 400, "left");

                move_forward(-0.2, 700);

                run_carousel(0.3, 6000);

                turn(0.5, 600, "right");

                move_forward(0.5, 1200);

                turn(0.5, 600, "left");
                move_forward(-.3, 625);
                move_forward(.3, 1800);
                clawObj.setPosition(.9);
                sleep(500);
                move_forward(-.3, 2500);
                armObj.setTargetPosition(1000);
                armObj.setPower(1);
                strafe(0.5, 1600, "right");

                if (crsRoute == "barrier") {
                    turn(0.3, 50, "left");
                    move_forward(.5, 5000);
                } else {
                    move_forward(.5, 400);
                    strafe(.5, 1500, "right");
                    move_forward(.5, 3500);
                }
            }
        }

            armObj.setTargetPosition(0);
            armObj.setPower(-.6);

            sleep(3000);

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

    private void setWristPosition(double inputPosition) {

        double position = .5;
        if (inputPosition <= wristMax && inputPosition >= wristMin) {
            position = inputPosition;
        }else if (inputPosition > wristMax){
            position = wristMax;
        }else if (inputPosition < wristMin){
            position = wristMin;
        }
        wristLPos.setPosition(position);
        wristRNeg.setPosition(((position - 0.5) - 0.5) * (-1));
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.FRONT;


        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
    public synchronized void deinit() {
        try {
            if (session != null) {
                session.stopCapture();
                session.close();
                session = null;
            }
            if (camera != null) {
                camera.close();
                camera = null;
            }
        } catch (Exception e) {
            StringWriter sw = new StringWriter();
            PrintWriter pw = new PrintWriter(sw);
            e.printStackTrace(pw);
        }
    }




//    //@Override
    public void onStop() {
        deinit();
    }
}