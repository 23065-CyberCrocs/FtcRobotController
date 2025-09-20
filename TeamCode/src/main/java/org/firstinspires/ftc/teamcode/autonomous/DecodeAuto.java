package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Webcam;

import java.util.Arrays;

@Autonomous(name = "Concept: Obelisk Detection")
public class DecodeAuto extends OpMode {
    String[] currentGamePattern = {"None", "None", "None"};


    @Override
    public void init() {
        Webcam.initWebcam(hardwareMap);
    }
    public void init_loop() {
        Webcam.obeliskDetect(currentGamePattern);
        telemetry.addData("Current Game Pattern", Arrays.toString(currentGamePattern));
        telemetry.update();
    }
    @Override
    public void loop() {
    }

    @Override
    public void stop() {
        Webcam.closeVision();
    }

}