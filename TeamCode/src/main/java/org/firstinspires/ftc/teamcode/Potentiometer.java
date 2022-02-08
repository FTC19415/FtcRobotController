package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Potentiometer extends LinearOpMode {
    // Define variables for our potentiometer and motor
    AnalogInput armPotentiometer;
    DcMotor armObj = null;

    // Define variable for the current voltage
    double currentVoltage;

    @Override
    public void runOpMode() {
        // Get the potentiometer and motor from hardwareMap
        armPotentiometer = hardwareMap.get(AnalogInput.class, "armPot");
        armObj = hardwareMap.get(DcMotor.class, "Arm");


        // Loop while the Op Mode is running
        waitForStart();
        while (opModeIsActive()) {
            // Update currentVoltage from the potentiometer
            currentVoltage = armPotentiometer.getVoltage();

            if (gamepad2.y) {
                if (currentVoltage <= 2.1 && currentVoltage >= 2.05){
                    armPotentiometer.getVoltage();
                    armObj.setPower(0);
                }else{
                    telemetry.addData("Potentiometer arm voltage", currentVoltage);
                    telemetry.update();
//                    while (!(armPotentiometer.getVoltage() == 2)) {
//                        armPotentiometer.getVoltage();
                        armObj.setPower(0.3);
//                    }
//                    armObj.setPower(0);
//                    armPotentiometer.getVoltage();
                }
            }
            armPotentiometer.getVoltage();

            if (currentVoltage <= 2.6 && currentVoltage >= 2.55){
                armPotentiometer.getVoltage();
                armObj.setPower(0);
            }else{
                telemetry.addData("Potentiometer arm voltage", currentVoltage);
                telemetry.update();
//                    while (!(armPotentiometer.getVoltage() == 2)) {
//                        armPotentiometer.getVoltage();
                armObj.setPower(0.3);
//                    }
//                    armObj.setPower(0);
//                    armPotentiometer.getVoltage();
            }
            armPotentiometer.getVoltage();

            // Show the potentiometer’s voltage in telemetry
//            telemetry.addData("Potentiometer voltage", currentVoltage);
//            telemetry.update();
        }
    }
}