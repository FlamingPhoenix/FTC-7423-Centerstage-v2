package org.firstinspires.ftc.teamcode;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utility.AxonServo;
import org.firstinspires.ftc.teamcode.utility.Claw;
import org.firstinspires.ftc.teamcode.utility.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.utility.LinkageArm;
import org.firstinspires.ftc.teamcode.utility.ServoDegreeController;
import org.firstinspires.ftc.teamcode.utility.ServoRelease;
import org.firstinspires.ftc.teamcode.utility.utils;

import java.util.concurrent.TimeUnit;

@TeleOp
public class TeleOpMain extends OpMode {
    FieldCentricDrive drive;
    Claw claw;
    AxonServo armServo;
    LinkageArm arm;
    ServoDegreeController wrist;
    ServoStates servoController;
    Servo drone;
    @Override
    public void init() {
        drive = new FieldCentricDrive(hardwareMap);
        claw = new Claw(hardwareMap.servo.get("claw"), 0, 0.411, 0.592,0.362, true);
        arm = new LinkageArm(hardwareMap.servo.get("linkage"), 175, 236);
        armServo = new AxonServo(hardwareMap.servo.get("armservo"), hardwareMap.analogInput.get("axonin"));
        wrist = new ServoDegreeController(hardwareMap.servo.get("wrist"), 300, 0.5);
        servoController = new ServoStates(new Servo[]{hardwareMap.servo.get("linkage"),hardwareMap.servo.get("wrist"),hardwareMap.servo.get("claw"),hardwareMap.servo.get("armservo")});

        servoController.addState("transfer",new double[]{0.88,0.567,-1,0.035});
        servoController.addState("transferIntake",new double[]{0.88,0.889,-1,0.035});

        servoController.addState("transferInter",new double[]{0.836,0.62777,-1,0.10});
        servoController.addState("first",new double[]{1,0.748,-1,0.62});

        servoController.addState("intermediate",new double[]{0.836,-1,0,0.4});
        servoController.addState("low",new double[]{0.31733,0.41333,-1,0.7});
        servoController.addState("high",new double[]{0.31722,0.748,-1,0.595});
        servoController.addState("mid",new double[]{0.556,0.748,-1,0.595});
        servoController.addState("intakeNew",new double[]{0.300,0.3188,0.592,0.75});
        servoController.addState("intakeNew",new double[]{0.300,0.3188,0.592,0.75});

        servoController.addState("intakeStack",new double[]{0.88,0.2994,0.592,0.0096666});

        drone = hardwareMap.servo.get("drone");
        drone.setPosition(0.753f);
        servoController.setState("transfer");
    }

    @Override
    public void loop() {
        if (gamepad1.right_stick_button){
            drive.resetIMU();
        }
        // CLAW LOGIC  // CLAW LOGIC  // CLAW LOGIC  // CLAW LOGIC  //
        if (gamepad1.x) {
            claw.close();
        } else if (gamepad1.a) {
            claw.halfOpen();
        } else if (gamepad1.b) {
            claw.open();
        }

        if(-gamepad2.right_stick_y>0.1) {
            armServo.setPosition(armServo.getPosition()+0.01*Math.abs(-gamepad2.right_stick_y));
        } else if(-gamepad2.right_stick_y<-0.1) {
            armServo.setPosition(armServo.getPosition()-0.01*Math.abs(-gamepad2.right_stick_y));
        }
        if(-gamepad2.left_stick_y>0.1) {
            wrist.setPosition(wrist.getPosition()+0.01*Math.abs(-gamepad2.left_stick_y));
        } else if(-gamepad2.left_stick_y<-0.1) {
            wrist.setPosition(wrist.getPosition()-0.01*Math.abs(-gamepad2.left_stick_y));
        }
        if(gamepad2.right_trigger>0.1) {
            arm.setPosition(arm.getPosition()+0.5*Math.abs(gamepad2.right_trigger));
        } else if(gamepad2.left_trigger>0.1) {
            arm.setPosition(arm.getPosition()-0.05*Math.abs(gamepad2.left_trigger));
        }
        if(gamepad2.right_bumper && -0.001+claw.getPosition()>0){
            claw.setPosition(claw.getPosition()-0.005);
        }
        if(gamepad2.left_bumper && 0.001+claw.getPosition()<1){
            claw.setPosition(claw.getPosition()+0.005);
        }

        if(gamepad2.dpad_left){
            servoController.setState("transfer");

        } else if(gamepad2.dpad_up){
            if(servoController.getCurrentState() == "transferIntake"){
                servoController.setState("intermediate");
                drive.stop();

                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
            servoController.setState("high");
        } else if(gamepad2.dpad_right){
            claw.setPosition(0f);
            servoController.setState("transferIntake");
        } else if(gamepad2.dpad_down){
            servoController.setState("intake"); // arm extended, claw open on ground.
        } else if(gamepad2.right_bumper){
            servoController.setState("first"); // arm extended, claw open on ground.
        }
        else if(gamepad2.x){
            servoController.setState("intakeNew");
        }
        else if(gamepad2.left_bumper){
            servoController.setState("mid");
        }

        if(gamepad2.a){
            drone.setPosition(0.44f);
        }



        // DRIVE //
        drive.drive(gamepad1);
        // TELEMETRY //
        telemetry.addData("currentState",servoController.getCurrentState());
        telemetry.update();
    }
}