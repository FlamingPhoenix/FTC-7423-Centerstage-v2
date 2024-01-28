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
import org.firstinspires.ftc.teamcode.utility.ServoDegreeController;

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
    ServoDegreeController wrist;
    boolean debug = true;
    double height = 203;
    double speedMultiplier = 1;

    boolean throwErrors = true;
    @Override
    public void init() {
        try {
            et = new ElapsedTime();
            fc = hardwareMap.get(DistanceSensor.class, "fc");
            drive = new FieldCentricDrive(hardwareMap);
            claw = new Claw(hardwareMap.servo.get("claw"), 0, 0, 0, debug);//TODO set positions
            arm = new LinkageArm(hardwareMap.servo.get("linkage"), 175, 236);
            armServo = new AxonServo(hardwareMap.servo.get("armservo"), hardwareMap.analogInput.get("axonin"));
            wrist = new ServoDegreeController(hardwareMap.servo.get("wrist"), 300, 0.5);//TODO: set max and min (or zero pos)
            perfectPixelPlacement = new PerfectPixelPlacement(arm, armServo,wrist, fc);
            perfectPixelPlacement.setOffsets(81.28, 203.2);
            perfectPixelPlacement.setSpeed(1);
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
            if (gamepad2.left_bumper) {
                speedMultiplier = 0.2;
            } else {
                speedMultiplier = 1;
            }
            double currentTime = et.time(TimeUnit.MILLISECONDS);
            // CLAW LOGIC  // CLAW LOGIC  // CLAW LOGIC  // CLAW LOGIC  //
            if (gamepad2.x) {
                claw.open();
            } else if (gamepad2.a) {
                claw.close();
            } else if (gamepad2.b) {
                claw.halfOpen();
            }
            // ARM LOGIC  // ARM LOGIC  // ARM LOGIC  // ARM LOGIC  //
            if (-gamepad2.left_stick_y > 0.1) {
                height += -gamepad2.left_stick_y*3*speedMultiplier;
            } else if (-gamepad2.left_stick_y < -0.1) {
                height += -gamepad2.left_stick_y*3*speedMultiplier;
            }
            if(-gamepad2.right_stick_y>0.1) {
                armServo.setPos(armServo.getPos()+0.01*speedMultiplier);
            } else if(-gamepad2.right_stick_y<-0.1) {
                armServo.setPos(armServo.getPos()-0.01*speedMultiplier);
            }
            perfectPixelPlacement.executeWithSensorSpeededArm(height);
            // DRIVE //
            drive.drive(gamepad1);
            //TELEMETRY //
            telemetry.addData("height", height);
            telemetry.addData("distance", fc.getDistance(DistanceUnit.MM));
            telemetry.addData("time", currentTime);
            telemetry.update();
        }catch(Exception e) {
            if (throwErrors) {
                throw e;
            } else {
                telemetry.addData("ERROR", e.getMessage());
                telemetry.update();
            }
        }
    }
}
