package org.firstinspires.ftc.teamcode.cv;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.opencv.core.Mat;

public interface FtcCamera {
    void init(HardwareMap hardwareMap);
    void start();
    Mat grabFrame();
    void deinit();
}
