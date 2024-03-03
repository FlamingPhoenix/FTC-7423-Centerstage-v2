package org.firstinspires.ftc.teamcode;

import android.provider.CalendarContract;
import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvWebcam;

public abstract class AutoBase extends LinearOpMode {
    DcMotor fr;
    DcMotor fl;
    DcMotor br;
    DcMotor bl;
    DcMotor lift;
    DistanceSensor distanceLeft, distanceMid, distanceRight;
    SampleMecanumDrive drive;
    NormalizedColorSensor colorSensorLeft;
    NormalizedColorSensor colorSensorRight;
    NormalizedColorSensor colorSensorGrabber;
    NormalizedRGBA colorsLeft;
    NormalizedRGBA colorsRight;
    NormalizedRGBA colorsGrabber;


    Servo intakeLeft, intakeRight; //intakeLeft is not used because one servo is enough
    Servo vbarLeft, vbarRight;
    Servo grabber;

    float pos = 0.5f;
    float vposR = 0.69f;
    float vposL = 0.7f;
    float fpos = 0.2f;

    public Servo pivotLeft;
    public Servo pivotRight;

    public DigitalChannel touch;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;


    //BNO055IMU imu;

    public void initialize() {
        initAprilTag();

        fr = hardwareMap.dcMotor.get("fr");
        fl = hardwareMap.dcMotor.get("fl");
        br = hardwareMap.dcMotor.get("br");
        bl = hardwareMap.dcMotor.get("bl");


        // set the digital channel to input.
        //touch.setMode(DigitalChannel.Mode.INPUT);

    }
    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
    }
    public Pose2d aprilTagRelocalize(int tag){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for(AprilTagDetection detection:currentDetections){
            if(detection.id == tag){
                Pose2d returnPose = new Pose2d(detection.ftcPose.x, detection.ftcPose.y, Math.toRadians(180));
                return returnPose;
            }
        }
        return null;
    }



}