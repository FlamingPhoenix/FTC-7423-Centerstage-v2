package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utility.AxonServo;
import org.firstinspires.ftc.teamcode.utility.LinkageArm;

public class PerfectPixelPlacement {
    LinkageArm arm;
    DistanceSensor input;
    AxonServo armServo;
    boolean useDistanceSensorDirect = false;

    // CALCUATION CONSTANTS //
    double sin60deg = Math.sin(Math.toRadians(60));
    double sec30deg = 1/Math.cos(Math.toRadians(30));
    double cos30deg = Math.cos(Math.toRadians(30));
    double ox = 0;
    double oy = 0;
    public PerfectPixelPlacement(LinkageArm arm,AxonServo armServo, DistanceSensor input){
        this.arm = arm;
        this.input = input;
        this.armServo = armServo;
        this.useDistanceSensorDirect = true;
    }
    public PerfectPixelPlacement(LinkageArm arm, AxonServo armServo){
        this.arm = arm;
        this.armServo = armServo;
        this.useDistanceSensorDirect = false;
    }
    public void executeWithSensor(double targetHeight){
        double distance = input.getDistance(DistanceUnit.MM);
        //double len = Math.sqrt(Math.pow(distance,2)+Math.pow(targetHeight,2)-2*distance*targetHeight*-0.5);
        //double langle = Math.asin((sin60deg*targetHeight)/len); //IN RADIANS
        double sc = (distance+0.5)*(oy/60)+ox;
        double sb = sec30deg*(cos30deg*targetHeight-oy);
        double sa = Math.sqrt(Math.pow(sb,2)+Math.pow(sc,2)-2*sb*sc*-0.5);
        double angle = Math.asin((sin60deg*sb)/sa); //IN RADIANS

        armServo.setPosRadians(angle);
        arm.setLen(sa);
    }
}
