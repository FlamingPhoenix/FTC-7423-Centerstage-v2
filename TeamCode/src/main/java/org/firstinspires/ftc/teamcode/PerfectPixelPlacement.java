package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utility.AxonServo;
import org.firstinspires.ftc.teamcode.utility.LinkageArm;
import org.firstinspires.ftc.teamcode.utility.ServoSpeedController;

public class PerfectPixelPlacement {
    LinkageArm arm;
    DistanceSensor input;
    AxonServo armServo;
    ServoSpeedController ssc;
    boolean useDistanceSensorDirect = false;

    // CALCUATION CONSTANTS //
    private final double sin60deg = Math.sin(Math.toRadians(60));
    private final double sec30deg = 1/Math.cos(Math.toRadians(30));
    private final double cos30deg = Math.cos(Math.toRadians(30));
    double pivotOffsetX = 0;
    double pivotOffsetY = 0;

    /**
     * Initialize PerfectPixelPlacement
     * @param arm LinkageArm for controlling the length of the arm
     * @param armServo The servo used to pivot the linkageArm
     * @param input DistanceSensor for reading the distance to the target
     */
    public PerfectPixelPlacement(LinkageArm arm,AxonServo armServo, DistanceSensor input){
        this.arm = arm;
        this.input = input;
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
    public PerfectPixelPlacement(LinkageArm arm, AxonServo armServo){
        this.arm = arm;
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
    public void executeWithSensor(double targetHeight){
        double distance = input.getDistance(DistanceUnit.MM);
        double sc = distance+0.5*(pivotOffsetY / sin60deg)+ pivotOffsetX;
        double sb = sec30deg*(cos30deg*targetHeight - pivotOffsetY);
        double sa = Math.sqrt(Math.pow(sb,2)+Math.pow(sc,2)-2*sb*sc*-0.5);
        double angle = FastMath.asin((sin60deg*sb)/sa); //IN RADIANS

        armServo.setPosRadians(angle);
        arm.setLen(sa);
    }
    public void executeWithSensorSpeededArm(double targetHeight){
        double distance = input.getDistance(DistanceUnit.MM);
        double sc = distance+0.5*(pivotOffsetY /sin60deg)+ pivotOffsetX;
        double sb = sec30deg*(cos30deg*targetHeight- pivotOffsetY);
        double sa = Math.sqrt(Math.pow(sb,2)+Math.pow(sc,2)-2*sb*sc*-0.5);
        double angle = FastMath.asin((sin60deg*sb)/sa); //IN RADIANS

        ssc.setTargetPos(Math.toDegrees(angle)/255);
        ssc.loopEvery();
        arm.setLen(sa);
    }
    public void executeWithReading(double targetHeight,double distance){
        double sc = distance+0.5*(pivotOffsetY /sin60deg)+ pivotOffsetX;
        double sb = sec30deg*(cos30deg*targetHeight- pivotOffsetY);
        double sa = Math.sqrt(Math.pow(sb,2)+Math.pow(sc,2)-2*sb*sc*-0.5);
        double angle = FastMath.asin((sin60deg*sb)/sa); //IN RADIANS

        armServo.setPosRadians(angle);
        arm.setLen(sa);
    }
    public void executeWithReadingSpeededArm(double targetHeight,double distance){
        double sc = distance+0.5*(pivotOffsetY /sin60deg)+ pivotOffsetX;
        double sb = sec30deg*(cos30deg*targetHeight- pivotOffsetY);
        double sa = Math.sqrt(Math.pow(sb,2)+Math.pow(sc,2)-2*sb*sc*-0.5);
        double angle = FastMath.asin((sin60deg*sb)/sa); //IN RADIANS

        ssc.setTargetPos(Math.toDegrees(angle)/255);
        ssc.loopEvery();
        arm.setLen(sa);
    }
    public double[] test(double targetHeight, double distance){
        double sc = distance+0.5*(pivotOffsetY /sin60deg)+ pivotOffsetX;
        double sb = sec30deg*(cos30deg*targetHeight- pivotOffsetY);
        double sa = Math.sqrt(Math.pow(sb,2)+Math.pow(sc,2)-2*sb*sc*-0.5);
        double angle = FastMath.asin((sin60deg*sb)/sa); //IN RADIANS
        return new double[]{sa,angle};
    }
}
