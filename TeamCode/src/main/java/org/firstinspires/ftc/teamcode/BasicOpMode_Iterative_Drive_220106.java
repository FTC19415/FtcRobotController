package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Mechanum Drive New Robot 01/06", group="Iterative Opmode")
@Disabled
public class BasicOpMode_Iterative_Drive_220106 extends OpMode
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
    private DcMotor turretObj = null;
    private Servo clawObj = null;
    private TouchSensor ArmStop;
    private boolean isArmButtonPressed = true;
    private Servo wristLPos = null;
    private Servo wristRNeg = null;
    double dblWristPosition;
    private double wristMax = 0.858;
    private double wristMin = 0.204;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        dblWristPosition = 0.5;

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightMotor");
        carousel = hardwareMap.get(DcMotor.class, "carousel");
        armObj = hardwareMap.get(DcMotor.class, "Arm");
        turretObj = hardwareMap.get(DcMotor.class, "turret");
        clawObj = hardwareMap.get(Servo.class, "Claw");
        LinearArmObj = hardwareMap.get(DcMotor.class, "LinearArm");
        ArmStop = hardwareMap.get(TouchSensor.class, "ArmStop");
        wristLPos = hardwareMap.get(Servo.class, "LeftWrist");
        wristRNeg = hardwareMap.get(Servo.class, "RightWrist");

        LinearArmObj.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LinearArmObj.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turretObj.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretObj.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armObj.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armObj.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armObj.setTargetPosition(0);
        armObj.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //armObj.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
        isArmButtonPressed = true;
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
        double carouselPower = 0.3;
        double fltForward;
        double fltStrafe;
        double fltPivot;
        double fltArm;
        double fltTurret;
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
        int currentLinearArmPosition = 0;
        ElapsedTime2 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ghostingTime = 200;
        fltNormalFactor = 0.4;
        intArmPosition = 0;
        intArmPositionPick = 7086; //new robot changed from 5800 to 6000 20211114
        intArmPositionDropMid = 4700;
        intArmPositionDrive = 2600;
        intArmPositionDropUp = 3700;
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        double up = gamepad2.left_stick_y;
        double down = -gamepad2.right_stick_y;
        double right = gamepad2.right_stick_x;
        double left = -gamepad2.right_stick_x;
        YTimer = 0;
        YTimerTwo = 0;


        frontLeftPower    = Range.clip(drive + turn, -0.3, 0.3) ;
        frontRightPower   = Range.clip(drive - turn, -0.3, 0.3) ;
        backLeftPower    = Range.clip(drive + turn, -0.3, 0.3) ;
        backRightPower   = Range.clip(drive - turn, -0.3, 0.3) ;

        double armPowerDown = Range.clip(down, -.7, -.05);
        double armPowerUp = Range.clip(up, .05, .7);

        if  (gamepad1.left_stick_y < 0) {
            fltForward = -(gamepad1.left_stick_y * gamepad1.left_stick_y);
        }else if (gamepad1.left_stick_y > 0) {
            fltForward = (gamepad1.left_stick_y * gamepad1.left_stick_y);
        }
        fltForward = -gamepad1.left_stick_y;
        fltStrafe = gamepad1.left_stick_x;
        fltPivot = gamepad1.right_stick_x;

        fltArm = -gamepad2.left_stick_y;

        fltTurret = gamepad2.right_stick_x;

        //turretObj.setPower(fltTurret);

        //Turret code to use once we have found the bounds
        //TODO: Need to slow down motors before reaching the bounds(Use linear arm code)

        if (gamepad2.right_stick_x <= 1 && gamepad2.right_stick_x > 0.05) {
            if (turretObj.getCurrentPosition() >= -1776) {
                turretObj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                //armObj.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                turretObj.setPower(fltTurret);
            }else{
                turretObj.setPower(0);
            }
        }else if(gamepad2.right_stick_x >= -1 && gamepad2.right_stick_x < -0.05){
            if (turretObj.getCurrentPosition() < 2761){
                turretObj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                //armObj.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                turretObj.setPower(fltTurret);
            }else{
                turretObj.setPower(0);
            }
        }else{
            turretObj.setPower(0);
        }


        // crazy Shayne code
        int isPositive = -1;
        if(fltArm > 0.0)
        {
            isPositive = 1;
        }
        fltArm = isPositive * (fltArm * fltArm);
        // end crazy Shayne code


        //This controls ALL arm movement
            if (gamepad2.left_stick_y <= 1 && gamepad2.left_stick_y > 0.05) {
                if (armObj.getCurrentPosition() >= -850) {
                    armObj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    //armObj.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    armObj.setPower(fltArm);
                    isArmButtonPressed = false;
                } else {
                    armObj.setPower(0);
                }
            } else if (gamepad2.left_stick_y >= -1 && gamepad2.left_stick_y < -0.05){
                if (armObj.getCurrentPosition() < intArmPositionPick - 50) {
                    armObj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    //armObj.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    armObj.setPower(fltArm);
                    isArmButtonPressed = false;
                } else {
                    armObj.setPower(0);
                }

            } else if (gamepad2.b) {
                if (ElapsedTime2.milliseconds() > YTimer) {
                    //armObj.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    //armObj.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armObj.setTargetPosition(intArmPositionDropMid);
                    armObj.setPower(1);
                    isArmButtonPressed = true;
                }
                YTimer = ElapsedTime2.milliseconds() + ghostingTime;
            } else if (gamepad2.y) {
                if (ElapsedTime2.milliseconds() > YTimer) {
                    //armObj.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    //armObj.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armObj.setTargetPosition(intArmPositionDropUp);
                    armObj.setPower(1);
                    isArmButtonPressed = true;
                }

                YTimer = ElapsedTime2.milliseconds() + ghostingTime;
            } else if (gamepad2.a) {
                if (ElapsedTime2.milliseconds() > YTimer) {

                    armObj.setTargetPosition(intArmPositionPick);
                    armObj.setPower(1);
                    isArmButtonPressed = true;
                }

                YTimer = ElapsedTime2.milliseconds() + ghostingTime;
            } else if (gamepad2.x) {
                armObj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if (gamepad2.left_stick_y <= 1 && gamepad2.left_stick_y > 0.05) {
                    if (armObj.getCurrentPosition() >= -9000) {
                        armObj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        //armObj.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                        armObj.setPower(fltArm);
                        isArmButtonPressed = false;
                    } else {
                        armObj.setPower(0);
                        armObj.setTargetPosition(armObj.getCurrentPosition());
                        armObj.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        armObj.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                } else if (gamepad2.left_stick_y >= -1 && gamepad2.left_stick_y < -0.05) {
                    if (armObj.getCurrentPosition() < intArmPositionPick - 50) {
                        armObj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        //armObj.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                        armObj.setPower(fltArm);
                        isArmButtonPressed = false;
                    } else {

                    }
                }

                armObj.setPower(0);
                armObj.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armObj.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armObj.setTargetPosition(0);
                armObj.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                isArmButtonPressed = true;

            } else if (isArmButtonPressed) {
                // Do nothing but keep running to position
            } else {
                //stop the arm if the arm stick is not active
                armObj.setPower(0);
                armObj.setTargetPosition(armObj.getCurrentPosition());
                armObj.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armObj.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }


        // Carousel Spinner motor section
        if (gamepad1.left_trigger > fltNormalFactor) {
            carouselPower = 0.5;
            if (gamepad1.left_bumper) {
                carousel.setPower(-carouselPower);
            } else if (gamepad1.right_bumper) {
                carousel.setPower(carouselPower);  //changed from .5 20211114
            } else {
                // Shut off the motor
                carousel.setPower(0);
            }
        }else

    {
        if (gamepad1.left_bumper) {
            carousel.setPower(-carouselPower);
        } else if (gamepad1.right_bumper) {
            carousel.setPower(carouselPower);  //changed from .5 20211114
        } else {
            // Shut off the motor
            carousel.setPower(0);
        }
    }


// If it was flat use open-0.7, close-1
        // fancy rubber arm open-.65, close-1
        //OG sttings open-0.2, close-0.6

        //Claw code
        if (gamepad2.right_bumper) {
            //Close the claw
            clawObj.setPosition(.7);
        } else if (gamepad2.left_bumper) {
            //Full open claw
            clawObj.setPosition(1);
        }else{
            //Regular open claw
            clawObj.setPosition(.9);
        }

        //Wrist code
        if (gamepad2.dpad_left) {
            setWristPosition(0.3);
        }
        if (gamepad2.dpad_right) {
            setWristPosition(0.5);
        }

        //Linear Arm movement
        if (gamepad2.dpad_up) {
            //Extend
            if (LinearArmObj.getCurrentPosition() < 2250) {
                LinearArmObj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                //armObj.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                LinearArmObj.setPower(1);
            }else if (LinearArmObj.getCurrentPosition() < 2750){

                LinearArmObj.setPower(.4);
                //LinearArmObj.setTargetPosition(currentLinearArmPosition + 100);
                //currentLinearArmPosition = LinearArmObj.getTargetPosition();
            }else {
                telemetry.addData("Extend limit reached: ", LinearArmObj.getCurrentPosition());
                LinearArmObj.setPower(0);
            }
        } else if (gamepad2.dpad_down) {
            //Retract
                if (LinearArmObj.getCurrentPosition() > 700) {
                    LinearArmObj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    LinearArmObj.setPower(-1);
                    //LinearArmObj.setTargetPosition(currentLinearArmPosition - 100);
                    //currentLinearArmPosition = LinearArmObj.getTargetPosition();
                }else if (LinearArmObj.getCurrentPosition() > 75){

                    LinearArmObj.setPower(-.4);
                    //LinearArmObj.setTargetPosition(currentLinearArmPosition + 100);
                    //currentLinearArmPosition = LinearArmObj.getTargetPosition();
                }else {
                    LinearArmObj.setPower(0);
                    telemetry.addData("Extend limit reached: ", LinearArmObj.getCurrentPosition());
                }
        } else {
            LinearArmObj.setPower(0);
        }

        //I am speed code
        if (gamepad1.right_trigger > fltNormalFactor) {
            fltForward = gamepad1.right_trigger * -gamepad1.left_stick_y;
            fltStrafe = gamepad1.right_trigger * gamepad1.left_stick_x;
            fltPivot = gamepad1.right_trigger * gamepad1.right_stick_x;
        } else {
            fltForward = fltNormalFactor * -gamepad1.left_stick_y;
            fltStrafe = fltNormalFactor * gamepad1.left_stick_x;
            fltPivot = fltNormalFactor * gamepad1.right_stick_x;
        }


        // Send calculated power to wheels/ drive code
        frontLeftDrive.setPower(fltForward + fltStrafe + fltPivot);
        frontRightDrive.setPower(fltForward + -fltStrafe + -fltPivot);
        backLeftDrive.setPower(fltForward + -fltStrafe + fltPivot);
        backRightDrive.setPower(fltForward + fltStrafe + -fltPivot);



        // Show the elapsed game time and wheel power.
        telemetry.addData("Linear Arm position:", LinearArmObj.getCurrentPosition());
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Is Arm Stop pressed:", ArmStop.isPressed());
        telemetry.addData("Arm Position:", armObj.getCurrentPosition());
        telemetry.addData("Turret Position:", turretObj.getCurrentPosition());
        telemetry.addData("Wrist L Position:", wristLPos.getPosition());
        telemetry.addData("Wrist R Position:", wristRNeg.getPosition());
        telemetry.addData("Was Arm button last action?:", isArmButtonPressed);
        telemetry.addData("Stick Movement:", gamepad2.left_stick_y);
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
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

}
