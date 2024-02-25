package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.PlacementPosition;
import org.firstinspires.ftc.teamcode.PropDetectionBlue;
import org.firstinspires.ftc.teamcode.PropDetectionRed;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class OpenCVPropDetector {
    int cameraMonitorViewId;
    private OpenCvCamera camera;
    private String webcamName = "Webcam 1";
    PropDetectionBlue propDetectionBlue;
    PropDetectionRed propDetectionRed;
    public OpenCVPropDetector(HardwareMap hardwareMap){
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        propDetectionRed = new PropDetectionRed();
        propDetectionBlue = new PropDetectionBlue();

    }
    public PlacementPosition detectRed(){
        camera.setPipeline(propDetectionRed);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });
        return propDetectionRed.getPlacementPosition();
    }
    public PlacementPosition detectBlue(){
        camera.setPipeline(propDetectionBlue);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });
        return propDetectionRed.getPlacementPosition();
    }
}
