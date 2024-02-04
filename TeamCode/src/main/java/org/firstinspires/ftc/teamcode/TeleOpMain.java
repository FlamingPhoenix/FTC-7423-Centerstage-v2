package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utility.AxonServo;
import org.firstinspires.ftc.teamcode.utility.Claw;
import org.firstinspires.ftc.teamcode.utility.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.utility.LinkageArm;
import org.firstinspires.ftc.teamcode.utility.ServoDegreeController;

import java.util.concurrent.TimeUnit;

@TeleOp
public class TeleOpMain extends OpMode {
    ElapsedTime timer;
    FieldCentricDrive drive;
    Claw claw;
    AxonServo armServo;
    LinkageArm arm;
    PerfectPixelPlacement perfectPixelPlacement;
    DistanceSensor frontDistanceSensor;
    ServoDegreeController wrist;
    ServoStates servoController;
    boolean debug = true;
    double height = 203;
    double speedMultiplier = 1;

    boolean throwErrors = true; // SET TO FALSE IN COMPETITION ( KEEP TRUE DURING PRACTICE )
    @Override
    public void init() {
        try {
            timer = new ElapsedTime();
            frontDistanceSensor = hardwareMap.get(DistanceSensor.class, "fc");
            drive = new FieldCentricDrive(hardwareMap);
            claw = new Claw(hardwareMap.servo.get("claw"), 0, 0, 0,0, debug);//TODO set positions
            arm = new LinkageArm(hardwareMap.servo.get("linkage"), 175, 236);
            armServo = new AxonServo(hardwareMap.servo.get("armservo"), hardwareMap.analogInput.get("axonin"));
            wrist = new ServoDegreeController(hardwareMap.servo.get("wrist"), 300, 0.5);//TODO: set max and min (or zero pos)
            servoController = new ServoStates(new Servo[]{hardwareMap.servo.get("linkage"),hardwareMap.servo.get("wrist"),hardwareMap.servo.get("claw"),hardwareMap.servo.get("armservo")});

            servoController.addState("transfer",new double[]{0.62,0.5,0,0.135});
            servoController.addState("low",new double[]{0.22333,0.41333,0.10166,0.7});
            servoController.addState("high",new double[]{0.22222,0.2655,0.1277,0.595});
            servoController.addState("intake",new double[]{0.303,0.803,0.176,0.033});
            perfectPixelPlacement = new PerfectPixelPlacement(arm, armServo,wrist, frontDistanceSensor);
            perfectPixelPlacement.setOffsets(81.28, 203.2);
            perfectPixelPlacement.setSpeed(1);


            servoController.setState("transfer");
        }catch(Exception e){
            if(throwErrors){
                throw e;
            }else {
                telemetry.addData("ERROR", e.getMessage());
                telemetry.update();
            }
        }

    }

    @Override
    public void loop() {
        try {
//            if (gamepad2.left_bumper) {
//                speedMultiplier = 0.2;
//            } else {
//                speedMultiplier = 1;
//            }

            if(gamepad1.left_bumper){
                drive.setSpeed(0.3);
            } else {
                drive.setSpeed(0.5);
            }
            double currentTime = timer.time(TimeUnit.MILLISECONDS);
            // CLAW LOGIC  // CLAW LOGIC  // CLAW LOGIC  // CLAW LOGIC  //
            if (gamepad2.x) {
                claw.open();
            } else if (gamepad2.a) {
                claw.close();
            } else if (gamepad2.b) {
                claw.halfOpen();
            }
//            //USE THIS FOR PERFECTPIXELPLACEMENT!!!!
//
//            // ARM LOGIC  // ARM LOGIC  // ARM LOGIC  // ARM LOGIC  //
//            if (-gamepad2.left_stick_y > 0.1) {
//                height += -gamepad2.left_stick_y*3*speedMultiplier;
//            } else if (-gamepad2.left_stick_y < -0.1) {
//                height += -gamepad2.left_stick_y*3*speedMultiplier;
//            }
//            // 300 low, 600 mid 800 high
//            if(-gamepad2.right_stick_y>0.1) {
//                armServo.setPos(armServo.getPos()+0.01*speedMultiplier*Math.abs(-gamepad2.right_stick_y));
//            } else if(-gamepad2.right_stick_y<-0.1) {
//                armServo.setPos(armServo.getPos()-0.01*speedMultiplier*Math.abs(-gamepad2.right_stick_y));
//            }

            if(gamepad1.left_trigger>0.1) {
                  perfectPixelPlacement.executeWithSensorSpeededArm(height);
            }
//            if(gamepad2.right_bumper) {
//                arm.setLen(500);
//            }else{
//                arm.setLen(0);
//            }

            if(gamepad2.dpad_left){
                servoController.setState("transfer"); // start position, claw tucked in
            } else if(gamepad2.dpad_up){
                servoController.setState("high"); // backdrop high
            } else if(gamepad2.dpad_right){
                servoController.setState("low"); // backdrop low
            } else if(gamepad2.dpad_down){
                servoController.setState("intake"); // arm extended, claw open on ground.
            }

            // DRIVE //
            drive.drive(gamepad1);
            //TELEMETRY //
            telemetry.addData("height", height);
            telemetry.addData("distance", frontDistanceSensor.getDistance(DistanceUnit.MM));
            telemetry.addData("time", currentTime);
            telemetry.addData("currentState",servoController.getCurrentState());
            telemetry.update();
        }catch(Exception e) {
            if (throwErrors) {
                throw new RuntimeException(e);
            } else {
                telemetry.addData("ERROR", e.getMessage());
                telemetry.update();
            }
        }
    }
}