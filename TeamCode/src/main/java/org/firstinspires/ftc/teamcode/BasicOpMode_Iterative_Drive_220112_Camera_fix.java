package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.cv.CameraPosition;
import org.firstinspires.ftc.teamcode.cv.FtcCamera;
import org.firstinspires.ftc.teamcode.cv.OpenCVWrapper;
import org.firstinspires.ftc.teamcode.cv.TeamMarkerPosition;
import org.firstinspires.ftc.teamcode.cv.TeamMarkerPositionDetector;
import org.firstinspires.ftc.teamcode.cv.Webcam;
import org.opencv.core.Mat;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;


@TeleOp(name="OpenCV 01/12", group="Iterative Opmode")
//@Disabled
public class BasicOpMode_Iterative_Drive_220112_Camera_fix extends OpMode
{
    // Declare OpMode members.
    public final FtcCamera webcam = new Webcam();
    String whatIsDetected = null;
    int elementDropLevel;
    //private BNO055IMU imu;
    List<Recognition> recognitions;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initializing Camera");
        telemetry.update();
        OpenCVWrapper.load();
        webcam.init(hardwareMap);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
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
        //runtime.reset();
//        OpenCVWrapper.load();
//        webcam.init(hardwareMap);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each motor to save power level for telemetry


//        if (gamepad1.a) {
//            webcam.init(hardwareMap);
//
//        } else if (gamepad1.b) {
//            webcam.deinit();
//        }

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
        //runtime.reset();
        telemetry.addData("Drop Level:", teamMarkerPosition.get());
        telemetry.update();




        /*
         * Code to run ONCE after the driver hits STOP
         */
    }
    @Override
    public void stop() {
    }
}

