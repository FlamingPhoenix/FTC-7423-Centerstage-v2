package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.AxonServo;
import org.firstinspires.ftc.teamcode.utility.Claw;
import org.firstinspires.ftc.teamcode.utility.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.utility.LinkageArm;

@TeleOp
public class TeleOpMain extends OpMode {
    FieldCentricDrive drive;
    Claw claw;
    AxonServo armServo;
    LinkageArm arm;
    boolean debug = true;
    @Override
    public void init() {
        drive = new FieldCentricDrive(hardwareMap);
        claw = new Claw(hardwareMap.servo.get("claw"),0,0,0,debug);
        arm = new LinkageArm(hardwareMap.servo.get("linkage"), 175,236);
        armServo = new AxonServo(hardwareMap.servo.get("armservo"), hardwareMap.analogInput.get("axonin"));

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



        drive.drive(gamepad1);
        telemetry.update();
    }
}
