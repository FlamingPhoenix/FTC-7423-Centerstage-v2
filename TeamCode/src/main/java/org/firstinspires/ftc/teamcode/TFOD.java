package org.firstinspires.ftc.teamcode;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

public class TFOD {

    private static TfodProcessor tfod;
    public static void initTfod(WebcamName webcam, VisionPortal visionPortal,String TFOD_MODEL_ASSET) {
        tfod = new TfodProcessor.Builder().setModelFileName(TFOD_MODEL_ASSET).build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(webcam);
        builder.setCameraResolution(new Size(640, 640));
        builder.addProcessor(tfod);
        visionPortal = builder.build();
        tfod.setMinResultConfidence(0.7f);
    }
    public static int getDetection() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        int pos = 3;
        // Step through the list of recognitions and display info for each one.
        if(currentRecognitions.size()>0){
            Recognition recognition = currentRecognitions.get(0);
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            //double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
            if(x>=0 && x<=213){
                pos = 0;
            }else if(x>213 && x<=426){
                pos = 1;
            }else if(x>426 && x<=640){
                pos = 2;
            }
        }   // end for() loop
        return pos;
    }   // end method telemetryTfod()

    public static int getPos() throws InterruptedException {
        int[] counts = new int[4];
        for(int i = 0; i<20; i++){
            counts[getDetection()]++;  //get 10 detections & note the outputs
            Thread.sleep(10);
        }
        int max = 0;
        int maxIndex = 0;
        for(int i = 0; i<4; i++){
            if(counts[i]>max){
                max = counts[i]; // get the max count from the detections -- this is the most common output
                maxIndex = i;
            }
        }
        return maxIndex;
    }
}
