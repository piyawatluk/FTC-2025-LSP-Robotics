package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Collections;
import java.util.List;

public class AprilTagEasyHelper {

    private final boolean useWebcam;
    private final String webcamName;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private boolean initialized = false;

    public AprilTagEasyHelper(boolean useWebcam, String webcamName) {
        this.useWebcam = useWebcam;
        this.webcamName = webcamName;
    }

    /**
     * Initializes the AprilTag processor and VisionPortal.
     *
     * @param hardwareMap FTC HardwareMap
     */
    public void initialize(HardwareMap hardwareMap) {
        if (initialized) return;
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        if (useWebcam) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, webcamName), aprilTag
            );
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag
            );
        }
        initialized = true;
    }

    /**
     * Gets a list of current AprilTag detections.
     */
    public List<AprilTagDetection> getDetections() {
        if (!initialized || aprilTag == null) return Collections.emptyList();
        return aprilTag.getDetections();
    }

    /**
     * Stops streaming and closes the vision portal to save resources.
     */
    public void shutdown() {
        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }
        initialized = false;
    }

    /**
     * Pause camera streaming.
     */
    public void stopStreaming() {
        if (visionPortal != null) visionPortal.stopStreaming();
    }

    /**
     * Resume camera streaming.
     */
    public void resumeStreaming() {
        if (visionPortal != null) visionPortal.resumeStreaming();
    }

}