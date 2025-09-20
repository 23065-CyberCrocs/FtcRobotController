package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

public class Webcam extends SubsystemBase {
    static int idDetected = 0;
    private static AprilTagProcessor obeliskTag;
    private static VisionPortal visionPortal;
    public static void obeliskDetect(String[] gamePattern) {
        if (obeliskTag == null) return;
        List<AprilTagDetection> currentTags = obeliskTag.getDetections();
        if (currentTags == null || currentTags.isEmpty()) return;

        for (AprilTagDetection detection : currentTags) {
            if (detection.id == 21) {
                gamePattern[0] = "Green";
                gamePattern[1] = "Purple";
                gamePattern[2] = "Purple";
            } else if (detection.id == 22) {
                gamePattern[0] = "Purple";
                gamePattern[1] = "Green";
                gamePattern[2] = "Purple";
            } else if (detection.id == 23) {
                gamePattern[0] = "Purple";
                gamePattern[1] = "Purple";
                gamePattern[2] = "Green";
            }
        }
    }

    public static void initWebcam(HardwareMap hwMap) {
        obeliskTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hwMap.get(WebcamName.class, "Webcam 1"), obeliskTag);
    }

    public static void closeVision() {
        visionPortal.close();
    }
}