package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Disabled
public class TeleOpTemplate extends OpMode {
    FieldCentricDrive drive;
    @Override
    public void init() {
        drive = new FieldCentricDrive(hardwareMap);
    }
    @Override
    public void loop() {
        drive.drive(gamepad1);
    }
}
