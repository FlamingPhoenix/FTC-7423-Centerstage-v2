package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.PerfectPixelPlacement;
import org.firstinspires.ftc.teamcode.utility.AxonServo;
import org.firstinspires.ftc.teamcode.utility.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.utility.LinkageArm;
import org.firstinspires.ftc.teamcode.utility.ServoDegreeController;

@TeleOp
public class ppptest extends OpMode {
    PerfectPixelPlacement ppp;
    FieldCentricDrive drive;
    LinkageArm linkage;
    AxonServo armServo;
    ServoDegreeController wrist;
    DistanceSensor fcDistanceSensor;
    @Override
    public void init(){
        drive = new FieldCentricDrive(hardwareMap);
        linkage = new LinkageArm(hardwareMap.servo.get("linkage"), 175, 236);
        armServo = new AxonServo(hardwareMap.servo.get("armservo"), hardwareMap.analogInput.get("axonin"));
        wrist = new ServoDegreeController(hardwareMap.servo.get("wrist"), 300, 0.5);
        fcDistanceSensor = hardwareMap.get(DistanceSensor.class, "fc");
        ppp = new PerfectPixelPlacement(linkage, armServo, wrist,fcDistanceSensor);
        ppp.setOffsets(81.28,203.2);
    }
    @Override
    public void loop(){
        if(gamepad1.left_bumper) {

            try {
                ppp.executeWithSensor(500);
                double[] values = ppp.test(500, fcDistanceSensor.getDistance(DistanceUnit.MM));
                telemetry.addData("linkageLen", values[0]);
                telemetry.addData("armServo", values[1]);
                telemetry.addData("armval", armServo.getPosFromAngle(values[1]));
                telemetry.addData("wrist", values[2]);
                telemetry.addData("distance", fcDistanceSensor.getDistance(DistanceUnit.MM));
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
        drive.drive(gamepad1);
        telemetry.update();
    }
}
