package org.firstinspires.ftc.teamcode;
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
        claw = new Claw(hardwareMap.servo.get("claw"), 0, 0.43, 0.562,0.562, true);
        arm = new LinkageArm(hardwareMap.servo.get("linkage"), 175, 236);
        armServo = new AxonServo(hardwareMap.servo.get("armservo"), hardwareMap.analogInput.get("axonin"));
        wrist = new ServoDegreeController(hardwareMap.servo.get("wrist"), 300, 0.5);
        servoController = new ServoStates(new Servo[]{hardwareMap.servo.get("linkage"),hardwareMap.servo.get("wrist"),hardwareMap.servo.get("claw"),hardwareMap.servo.get("armservo")});

        servoController.addState("transfer",new double[]{0.88,0.567,-1,0.035});
        servoController.addState("transferIntake",new double[]{0.88,0.889,-1,0.035});

        servoController.addState("transferInter",new double[]{0.836,0.62777,-1,0.10});

        servoController.addState("intermediate",new double[]{0.836,-1,0,0.4});
        servoController.addState("low",new double[]{0.31733,0.41333,-1,0.7});
        servoController.addState("high",new double[]{0.31722,0.748,-1,0.595});
        servoController.addState("intake",new double[]{0.88,0.347,0.562,0.005});

        drone = hardwareMap.servo.get("drone");

        servoController.setState("transfer");
    }

    @Override
    public void loop() {
        if (gamepad1.right_stick_button){
            drive.resetIMU();
        }
        // CLAW LOGIC  // CLAW LOGIC  // CLAW LOGIC  // CLAW LOGIC  //
        if (gamepad2.x) {
            claw.close();
        } else if (gamepad2.a) {
            claw.halfOpen();
        } else if (gamepad2.b) {
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

            }
            servoController.setState("high");
        } else if(gamepad2.dpad_right){
            claw.setPosition(0f);
            servoController.setState("transferIntake");
        } else if(gamepad2.dpad_down){
            servoController.setState("intake"); // arm extended, claw open on ground.
        }

        if(gamepad1.x){
            drone.setPosition(0.7f);
        }
        if(gamepad1.a){
            drone.setPosition(0.9f);
        }


        // DRIVE //
        drive.drive(gamepad1);
        // TELEMETRY //
        telemetry.addData("currentState",servoController.getCurrentState());
        telemetry.update();
    }
}