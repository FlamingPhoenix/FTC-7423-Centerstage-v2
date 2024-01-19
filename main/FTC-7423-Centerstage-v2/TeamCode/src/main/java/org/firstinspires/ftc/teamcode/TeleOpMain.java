package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utility.AxonServo;
import org.firstinspires.ftc.teamcode.utility.Claw;
import org.firstinspires.ftc.teamcode.utility.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.utility.LinkageArm;

import java.util.concurrent.TimeUnit;

@TeleOp
public class TeleOpMain extends OpMode {
    ElapsedTime et;
    FieldCentricDrive drive;
    Claw claw;
    AxonServo armServo;
    LinkageArm arm;
    PerfectPixelPlacement perfectPixelPlacement;
    DistanceSensor fc;
    boolean debug = true;
    double height = 203;
    int tolerance = 50;
    @Override
    public void init() {
        et = new ElapsedTime();
        fc = hardwareMap.get(DistanceSensor.class, "fc");
        drive = new FieldCentricDrive(hardwareMap);
        claw = new Claw(hardwareMap.servo.get("claw"),0,0,0,debug);
        arm = new LinkageArm(hardwareMap.servo.get("linkage"), 175,236);
        armServo = new AxonServo(hardwareMap.servo.get("armservo"), hardwareMap.analogInput.get("axonin"));
        perfectPixelPlacement = new PerfectPixelPlacement(arm,armServo,fc);
        perfectPixelPlacement.setOffsets(81.28,203.2);
        perfectPixelPlacement.setSpeed(1);

    }

    @Override
    public void loop() {
        // CLAW LOGIC  // CLAW LOGIC  // CLAW LOGIC  // CLAW LOGIC  //
        if(gamepad2.x){
            claw.open();
        }else if(gamepad2.a){
            claw.close();
        }else if(gamepad2.b){
            claw.halfOpen();
        }
        double currentTime = et.time(TimeUnit.MILLISECONDS);
        if(-gamepad2.left_stick_y>0.1){
            if((currentTime % 50) < 50){
                height += 3;
            }
        }else if(-gamepad2.left_stick_y<-0.1){
            if((currentTime % 50) < 50){
                height -= 3;
            }
        }
        if(gamepad2.left_bumper){
            tolerance = 10;
        }else{
            tolerance = 50;
        }
        // ARM LOGIC  // ARM LOGIC  // ARM LOGIC  // ARM LOGIC  //
        if(gamepad2.right_trigger>0.1){
            perfectPixelPlacement.executeWithSensorSpeededArm(height);
        }
        // DRIVE //
        drive.drive(gamepad1);


        //TELEMETRY //
        telemetry.addData("height",height);
        telemetry.addData("distance",fc.getDistance(DistanceUnit.MM));
        telemetry.addData("time",currentTime);
        telemetry.update();
    }
}
