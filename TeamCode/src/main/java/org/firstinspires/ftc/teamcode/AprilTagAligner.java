package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class AprilTagAligner{
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag = AprilTagProcessor.easyCreateWithDefaults();
    DcMotor fl, fr, bl, br;

    private AprilTagDetection desiredTag;
    boolean targetFound = false;
    double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;
    double drive, strafe, turn;
    double rangeError, headingError, yawError = 999.0;
    public AprilTagAligner(HardwareMap hardwareMap, String webcamName) throws InterruptedException {
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, webcamName))
                .addProcessor(aprilTag)
                .build();
        setManualExposure(6,250);
        fl = hardwareMap.dcMotor.get("fl");
        bl = hardwareMap.dcMotor.get("bl");
        fr = hardwareMap.dcMotor.get("fr");
        br = hardwareMap.dcMotor.get("br");
        //reverse motors because they are facing the opposite direction
        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);

    }
    public List<AprilTagDetection> getDetections(){
        return aprilTag.getDetections();
    }

    /**
     * Aligns the robot to the desired april tag
     * takes time!!!!!!!
     * @param id the id of the tag to align to
     * @param desiredDistance the distance to the tag
     */
    public void alignRobot(int id, double desiredDistance) throws InterruptedException {
        DESIRED_DISTANCE = desiredDistance;
        while(rangeError > 0.1 && headingError > 2.0 && yawError > 0.1) {
            List<AprilTagDetection> detections = getDetections();
            for (AprilTagDetection detection : detections) {
                if (detection.metadata != null) {
                    if (detection.id == id) {
                        desiredTag = detection;
                        targetFound = true;
                    } else {
                        targetFound = false;
                    }
                }
            }
            if (targetFound) {

                rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                headingError = desiredTag.ftcPose.bearing;
                yawError = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                moveRobot(drive, strafe, turn);
            }
            sleep(30);
        }
    }

    /**
     * Aligns the robot to the desired april tag
     * Can be used in TeleOp
     * @param id the id of the tag to align to
     */
    public void alignRobot2(int id){
            List<AprilTagDetection> detections = getDetections();
            for (AprilTagDetection detection : detections) {
                if (detection.metadata != null) {
                    if (detection.id == id) {
                        desiredTag = detection;
                        targetFound = true;
                    } else {
                        targetFound = false;
                    }
                }
            }
            if (targetFound) {

                rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                headingError = desiredTag.ftcPose.bearing;
                yawError = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                moveRobot(drive, strafe, turn);
            }
    }
    private void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        fl.setPower(leftFrontPower);
        fr.setPower(rightFrontPower);
        bl.setPower(leftBackPower);
        br.setPower(rightBackPower);
    }
    private void    setManualExposure(int exposureMS, int gain) throws InterruptedException {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {

            while ((visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }

        }

        // Set camera controls unless we are stopping.

            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);

    }
}
