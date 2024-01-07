package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utility.AxonServo;
import org.firstinspires.ftc.teamcode.utility.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.utility.LinkageArm;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class LinkageArmTest extends OpMode {
    FieldCentricDrive drive;
    AxonServo armServo;
    LinkageArm arm;
    @Override
    public void init() {
        arm = new LinkageArm(hardwareMap.servo.get("linkage"), 175,236);
        drive = new FieldCentricDrive(hardwareMap);
        armServo = new AxonServo(hardwareMap.servo.get("armservo"), hardwareMap.analogInput.get("axonin"));
    }

    @Override
    public void loop() {
        drive.drive(gamepad1);
        if(gamepad1.a){
            arm.setLen(300);
        }
        if(gamepad1.b){
            arm.setLen(175);
        }
        if(gamepad1.x){
            arm.setLen(100);
        }
        if(gamepad1.y){
            arm.setLen(50);
        }
    }
}
