package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utility.AxonServo;
import org.firstinspires.ftc.teamcode.utility.LinkageArm;
import org.firstinspires.ftc.teamcode.utility.ServoDegreeController;
import org.firstinspires.ftc.teamcode.utility.ServoSpeedController;

public class PerfectPixelPlacement {
    LinkageArm arm;
    DistanceSensor inputDistanceSensor;
    AxonServo armServo;
    ServoSpeedController ssc;
    ServoDegreeController wrist;
    boolean useDistanceSensorDirect = false;
    double defaultArmlen;

    // CALCUATION CONSTANTS //
    private final double sin60deg = Math.sin(Math.toRadians(60));
    private final double sin120deg = Math.sin(Math.toRadians(120));
    private final double sec30deg = 1/Math.cos(Math.toRadians(30));
    private final double cos30deg = Math.cos(Math.toRadians(30));
    double pivotOffsetX = 0;
    double pivotOffsetY = 0;

    /**
     * Initialize PerfectPixelPlacement
     * @param arm LinkageArm for controlling the length of the arm
     * @param armServo The servo used to pivot the linkageArm
     * @param inputDistanceSensor DistanceSensor for reading the distance to the target
     */
    // code for usage with distancesensor
    public PerfectPixelPlacement(LinkageArm arm,AxonServo armServo,ServoDegreeController wrist, DistanceSensor inputDistanceSensor){
        this.arm = arm;
        this.inputDistanceSensor = inputDistanceSensor;
        this.armServo = armServo;
        this.useDistanceSensorDirect = true;
        ssc = new ServoSpeedController(armServo);
    }
    /**
     * Initialize PerfectPixelPlacement
     * To be used with an external distance sensor
     * @param arm LinkageArm for controlling the length of the arm
     * @param armServo The servo used to pivot the linkageArm
     */
    // code for usage without distancesensor
    public PerfectPixelPlacement(LinkageArm arm, AxonServo armServo, ServoDegreeController wrist){
        this.arm = arm;
        this.armServo = armServo;
        this.useDistanceSensorDirect = false;
        ssc = new ServoSpeedController(armServo);
    }
    // default arm length
    public PerfectPixelPlacement(double defaultArmlen, AxonServo armServo, ServoDegreeController wrist){
        this.defaultArmlen = defaultArmlen;
        this.armServo = armServo;
        this.useDistanceSensorDirect = false;
        ssc = new ServoSpeedController(armServo);
    }
    public void setOffsets(double ox, double oy){
        this.pivotOffsetX = ox;//8x3.2 in //81.28
        this.pivotOffsetY = oy;//203.2
    }
    public void setSpeed(double speed){
        ssc.setSpeed(speed);
    }
    // main function with distance sensor
    public void executeWithSensor(double targetHeight) throws InterruptedException {
        double distance = inputDistanceSensor.getDistance(DistanceUnit.MM); // distance in mm from sensor
        double sc = distance+0.5*(pivotOffsetY / sin60deg)+ pivotOffsetX;
        double sb = sec30deg*(cos30deg*targetHeight - pivotOffsetY);
        double sa = Math.sqrt(Math.pow(sb,2)+Math.pow(sc,2)-2*sb*sc*-0.5);
        double angle = FastMath.asin((sin60deg*sb)/sa); //IN RADIANS


        arm.setLen(sa-172);
        Thread.sleep(20);
        armServo.setPosRadians(angle);
    }
    // faster arm function with distance sensor
    public void executeWithSensorSpeededArm(double targetHeight) throws InterruptedException {
        double distance = inputDistanceSensor.getDistance(DistanceUnit.MM); // distance in mm from sensor
        double sc = distance+0.5*(pivotOffsetY /sin60deg)+ pivotOffsetX;
        double sb = sec30deg*(cos30deg*targetHeight- pivotOffsetY);
        double sa = Math.sqrt(Math.pow(sb,2)+Math.pow(sc,2)-2*sb*sc*-0.5);
        double angle = FastMath.asin((sin60deg*sb)/sa); //IN RADIANS
        double wristAngle = 60 - Math.toDegrees(angle);

        arm.setLen(sa-172);
        Thread.sleep(20);
        ssc.setTargetPos(armServo.getPosFromAngle(Math.toDegrees(angle)));
        ssc.loopEvery();
    }
    // height and distance function without distance sensor
    public void executeWithReading(double targetHeight,double distance){
        double sc = distance+0.5*(pivotOffsetY / sin60deg)+ pivotOffsetX;
        double sb = sec30deg*(cos30deg*targetHeight- pivotOffsetY);
        double sa = Math.sqrt(Math.pow(sb,2)+Math.pow(sc,2)-2*sb*sc*-0.5);
        double angle = FastMath.asin((sin60deg*sb)/sa); //IN RADIANS
        double wristAngle = 60 - Math.toDegrees(angle);
        armServo.setPosRadians(angle);
        arm.setLen(sa-172);
    }
    // height and distance function without distance sensor, faster
    public void executeWithReadingSpeededArm(double targetHeight,double distance){
        double sc = distance+0.5*(pivotOffsetY / sin60deg)+ pivotOffsetX;
        double sb = sec30deg*(cos30deg*targetHeight- pivotOffsetY);
        double sa = Math.sqrt(Math.pow(sb,2)+Math.pow(sc,2)-2*sb*sc*-0.5);
        double angle = FastMath.asin((sin60deg*sb)/sa); //IN RADIANS
        double wristAngle = 60 - Math.toDegrees(angle);
        ssc.setTargetPos(armServo.getPosFromAngle(Math.toDegrees(angle)));
        ssc.loopEvery();
        arm.setLen(sa-172);
    }

    // set position for auto
    public void executeForAuto(double distance, double targetHeight){
        double sc = distance+0.5*(pivotOffsetY /sin60deg)+ pivotOffsetX;
        double ac= FastMath.asin((sin120deg/defaultArmlen)*sc);
        double aa=180-120-ac;

        armServo.setPosRadians(aa);
    }
    // test function
    public double[] test(double targetHeight, double distance){
        double sc = distance+0.5*(pivotOffsetY / sin60deg)+ pivotOffsetX;
        double sb = sec30deg*(cos30deg*targetHeight- pivotOffsetY);
        double sa = Math.sqrt(Math.pow(sb,2)+Math.pow(sc,2)-2*sb*sc*-0.5);
        double angle = FastMath.asin((sin60deg*sb)/sa); //IN RADIANS
        double wristAngle = 60 - Math.toDegrees(angle);
        return new double[]{sa,angle, wristAngle};
    }
}
