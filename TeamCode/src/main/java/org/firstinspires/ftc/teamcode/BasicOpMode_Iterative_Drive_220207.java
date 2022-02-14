package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

;


@TeleOp(name="Mechanum Drive New Robot 02/07", group="Iterative Opmode")
@Disabled
public class BasicOpMode_Iterative_Drive_220207 extends OpMode
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
    private LED LEDBR = null;
    private LED LEDBG = null;
    private LED LEDFR = null;
    private LED LEDFG = null;
    private LED LEDTR = null;
    private LED LEDTG = null;
    private boolean isArmButtonPressed = true;
    private Servo wristLPos = null;
    private Servo wristRNeg = null;
    double dblWristPosition;
    private double wristMax = 0.858;
    private double wristMin = 0.204;
    double cappingTimer = 0;
    boolean cappingTrigger = false;
    double carouselPower = 0.3;
    ElapsedTime ElapsedTime2 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    double YTimer = 0;
    double YTimerTwo = 0;
    AnalogInput armPotentiometer;
    AnalogInput turretPotentiometer;

    // Define variable for the current voltage
    double armCurrentVoltage;
    double turretCurrentVoltage;


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
        wristLPos = hardwareMap.get(Servo.class, "LeftWrist");
        wristRNeg = hardwareMap.get(Servo.class, "RightWrist");
        LEDBR = hardwareMap.get(LED.class, "LEDBR");
        LEDBG = hardwareMap.get(LED.class, "LEDBG");
        LEDFR = hardwareMap.get(LED.class, "LEDFR");
        LEDFG = hardwareMap.get(LED.class, "LEDFG");
        LEDTR = hardwareMap.get(LED.class, "LEDTR");
        LEDTG = hardwareMap.get(LED.class, "LEDTG");
        armPotentiometer = hardwareMap.get(AnalogInput.class, "armPot");
        turretPotentiometer = hardwareMap.get(AnalogInput.class, "turretPot");

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
        int ghostingTime;
        double fltNormalFactor;
        double wristFlow;
        double trigger;
        int currentLinearArmPosition = 0;
        ghostingTime = 200;
        fltNormalFactor = 0.4;
        trigger = 0.2;
        intArmPosition = 0;
        intArmPositionPick = 6860; //new robot changed from 5800 to 6000 20211114
        intArmPositionDropMid = 4700;
        intArmPositionDrive = 2600;
        intArmPositionDropUp = 3700;
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        double up = gamepad2.left_stick_y;
        double down = -gamepad2.right_stick_y;
        double right = gamepad2.right_stick_x;
        double left = -gamepad2.right_stick_x;

        //
        //      Driving
        //

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

        //
        //  All turret code
        //


        if (gamepad2.right_stick_x >= -1 && gamepad2.right_stick_x < -0.05) {
            if(turretPotentiometer.getVoltage() < 3.0){
                turretObj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                turretObj.setPower(fltTurret * 0.5);

            }else if (turretPotentiometer.getVoltage() < 3.3) {
                turretObj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                turretObj.setPower(fltTurret * 0.3);
            }else{
                turretObj.setPower(0);
            }
        }else if(gamepad2.right_stick_x <= 1 && gamepad2.right_stick_x > 0.05){
            if(turretPotentiometer.getVoltage() >= 1.84){
                turretObj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                turretObj.setPower(fltTurret * 0.5);

            }else if (turretPotentiometer.getVoltage() >= 0.685){
                turretObj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                //armObj.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                turretObj.setPower(fltTurret * 0.3);
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

        //Capping
        if (gamepad2.left_trigger > trigger) {
                if (cappingTrigger == false) {
                    armObj.setPower(0);
                    armObj.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    armObj.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    cappingTimer = runtime.milliseconds();
                    cappingTrigger = true;
                    armObj.setTargetPosition(3575);
                    armObj.setPower(1);
                }else {
                    if (cappingTimer + 250 <= runtime.milliseconds()) {
                        LinearArmObj.setTargetPosition(1700);
                        setWristPosition(0.75);
                        if (LinearArmObj.getCurrentPosition() >= 1700) {
                            LinearArmObj.setPower(0);
                        } else {
                            LinearArmObj.setPower(0.7);
                        }
                    }
                }

        // gamepad2 B
        } else if (gamepad2.b) {
//            if (ElapsedTime2.milliseconds() > YTimer) {
//                LinearArmObj.setTargetPosition(1700);
//                armObj.setTargetPosition(4218);
//                armObj.setPower(1);
//                if (LinearArmObj.getCurrentPosition() >= 1700){
//                    LinearArmObj.setPower(0);
//                }else {
//                    LinearArmObj.setPower(0.7);
//                }
//                setWristPosition(0.6);
//            }
//            YTimer = ElapsedTime2.milliseconds() + ghostingTime;
//            cappingTrigger = false;

        // gamepad2 y
        } else if (gamepad2.y) {
//            if (ElapsedTime2.milliseconds() > YTimer) {
//                //armObj.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                //armObj.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                armObj.setTargetPosition(intArmPositionDropUp);
//                armObj.setPower(1);
//                isArmButtonPressed = true;
////            }
//
//            YTimer = ElapsedTime2.milliseconds() + ghostingTime;
//            cappingTrigger = false;

//      // gamepad2 a
        } else if (gamepad2.a) {
//            if (ElapsedTime2.milliseconds() > YTimer) {
//                LinearArmObj.setTargetPosition(1472);
//                armObj.setTargetPosition(6430);
//                armObj.setPower(1);
//                if (LinearArmObj.getCurrentPosition() >= 1472){
//                    LinearArmObj.setPower(0);
//                }else {
//                    LinearArmObj.setPower(0.7);
//                }
//                setWristPosition(0.5);
//            }
//            YTimer = ElapsedTime2.milliseconds() + ghostingTime;
//            cappingTrigger = false;

//      // gamepad2 x, reset arm
        }else if (gamepad2.x) {
            setArmPosition(2.7);
            isArmButtonPressed = true;
            cappingTrigger = false;
            isArmButtonPressed = false;

//      // gamepad2 arm stick movement
        }else if (gamepad2.left_stick_y <= 1 && gamepad2.left_stick_y > 0.05) {
                if (armPotentiometer.getVoltage() <= 3) {
                    armObj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    //armObj.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    armObj.setPower(fltArm);
                } else if (armPotentiometer.getVoltage() <= 3.3){
                    armObj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    //armObj.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    armObj.setPower(fltArm * 0.5);
                }else{
                    armObj.setPower(0);
                }

                cappingTrigger = false;
        } else if (gamepad2.left_stick_y >= -1 && gamepad2.left_stick_y < -0.05){
                if (armPotentiometer.getVoltage() > 0.2) {
                    armObj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    //armObj.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    armObj.setPower(fltArm);
                    isArmButtonPressed = false;
                } else if (armPotentiometer.getVoltage() > 0.029) {
                    armObj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    //armObj.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    armObj.setPower(fltArm * 0.5);
                    isArmButtonPressed = false;
                }else{
                    armObj.setPower(0);
                }
                cappingTrigger = false;

        } else if (isArmButtonPressed) {
                // Do nothing but keep running to position
                cappingTrigger = false;
        }else if (gamepad2.dpad_up) {
            //Extend
                if (LinearArmObj.getCurrentPosition() < 2250) {
                    LinearArmObj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    //armObj.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    LinearArmObj.setPower(1);
                }else if (LinearArmObj.getCurrentPosition() < 2750) {

                    LinearArmObj.setPower(.4);
                    //LinearArmObj.setTargetPosition(currentLinearArmPosition + 100);
                    //currentLinearArmPosition = LinearArmObj.getTargetPosition();
             }
                cappingTrigger = false;
        } else if (gamepad2.dpad_down) {
                //Retract
                if (LinearArmObj.getCurrentPosition() > 700) {
                    LinearArmObj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    LinearArmObj.setPower(-1);
                    //LinearArmObj.setTargetPosition(currentLinearArmPosition - 100);
                    //currentLinearArmPosition = LinearArmObj.getTargetPosition();
                }else if (LinearArmObj.getCurrentPosition() > 75) {

                    LinearArmObj.setPower(-.4);
                    //LinearArmObj.setTargetPosition(currentLinearArmPosition + 100);
                    //currentLinearArmPosition = LinearArmObj.getTargetPosition();
                }
                cappingTrigger = false;
        }else {
                LinearArmObj.setPower(0);
                cappingTrigger = false;
                //stop the arm if the arm stick is not active
                armObj.setPower(0);
                armObj.setTargetPosition(armObj.getCurrentPosition());
                armObj.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armObj.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }


        // Carousel Spinner motor section
        if (gamepad1.left_trigger > trigger) {
            carouselPower = 0.5;
            if (gamepad1.left_bumper) {
                carousel.setPower(-carouselPower);
            } else if (gamepad1.right_bumper) {
                carousel.setPower(carouselPower);  //changed from .5 20211114
            } else {
                // Shut off the motor
                carousel.setPower(0);
            }
        }else{
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
        //OG stings open-0.2, close-0.6


        //Claw code
        if (gamepad2.right_bumper) {
            //Close the claw
            clawObj.setPosition(.65);
        } else if (gamepad2.left_bumper) {
            //Full open claw
            clawObj.setPosition(1);
        }else{
            //Regular open claw
            clawObj.setPosition(.9);
        }

        //Wrist code
        if (gamepad2.back) {
            setWristPosition(0.5);
        }
        if (gamepad2.dpad_left) {
            wristFlow = (wristLPos.getPosition() + 0.05);
            setWristPosition(wristFlow);
        }
        if (gamepad2.dpad_right) {
            wristFlow = (wristLPos.getPosition() - 0.05);
            setWristPosition(wristFlow);
        }

        //Linear Arm movement


        //I am speed code
        if (gamepad1.right_trigger > trigger) {
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

        if (runtime.milliseconds() >= 85000){
            LEDBR.enable(true);
            LEDFR.enable(true);
            LEDFR.enable(true);
            LEDBG.enable(true);
            LEDFG.enable(true);
            LEDTG.enable(true);
        }else if (runtime.milliseconds() >= 115000){
            LEDBG.enable(false);
            LEDFG.enable(false);
            LEDTG.enable(false);
            LEDBR.enable(true);
            LEDFR.enable(true);
            LEDTR.enable(true);
        }else{
            LEDBG.enable(true);
            LEDFG.enable(true);
            LEDTG.enable(true);
            LEDBR.enable(false);
            LEDFR.enable(false);
            LEDTR.enable(false);
        }

        armCurrentVoltage = armPotentiometer.getVoltage();
        turretCurrentVoltage = turretPotentiometer.getVoltage();

        // Show the potentiometer’s voltage in telemetry
        telemetry.addData(" Arm Potentiometer voltage", armCurrentVoltage);
        telemetry.addData("Turret Potentiometer voltage", turretCurrentVoltage);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Linear Arm position:", LinearArmObj.getCurrentPosition());
        telemetry.addData("Status", "Run Time: " + runtime.milliseconds());
        telemetry.addData("Status", "Capping Timer: " + cappingTimer);
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

    private void setArmPosition(double inputPosition){
// "Up" = is a negative value to make the number go up, "down" = opposite
        armObj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armCurrentVoltage = armPotentiometer.getVoltage();

        if (inputPosition > armCurrentVoltage){
            if (armCurrentVoltage <= (inputPosition + 0.05) && armCurrentVoltage >= (inputPosition - 0.05)){
                armObj.setPower(0);
            }else{
                telemetry.addData("Potentiometer up arm voltage", armCurrentVoltage);
                telemetry.update();
                armObj.setPower(-0.3);

            }
        }else if(inputPosition < armCurrentVoltage){
            if (armCurrentVoltage <= (inputPosition + 0.05) && armCurrentVoltage >= (inputPosition - 0.05)){
                armPotentiometer.getVoltage();
                armObj.setPower(0);
            }else{
                telemetry.addData("Potentiometer down arm voltage", armCurrentVoltage);
                telemetry.update();
//                    while (!(armPotentiometer.getVoltage() == 2)) {
//                        armPotentiometer.getVoltage();
                armObj.setPower(0.3);
//                    }
//                    armObj.setPower(0);
//                    armPotentiometer.getVoltage();
            }
        }

    }

}
