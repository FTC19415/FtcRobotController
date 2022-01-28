package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Potentiometer extends LinearOpMode {
    // Define variables for our potentiometer and motor
    AnalogInput potentiometer;
    DcMotor ArmObj;

    // Define variable for the current voltage
    double currentVoltage;

    @Override
    public void runOpMode() {
        // Get the potentiometer and motor from hardwareMap
        potentiometer = hardwareMap.get(AnalogInput.class, "Potentiometer");
        ArmObj = hardwareMap.get(DcMotor.class, "Arm");
        //motor = hardwareMap.get(DcMotor.class, "Motor");

        // Loop while the Op Mode is running
        waitForStart();
        while (opModeIsActive()) {
            // Update currentVoltage from the potentiometer
            currentVoltage = potentiometer.getVoltage();

            // Turn the motor on or off based on the potentiometer’s position
            if (currentVoltage < 1.65) {
                ArmObj.setPower(0);
            } else {
                ArmObj.setPower(0.3);
            }

            // Show the potentiometer’s voltage in telemetry
            telemetry.addData("Potentiometer voltage", currentVoltage);
            telemetry.update();
        }
    }
}