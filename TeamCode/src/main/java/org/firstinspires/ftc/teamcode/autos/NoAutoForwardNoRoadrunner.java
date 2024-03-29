package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.utility.utils;

@Autonomous(name = "No Auto Forward Without Roadrunner")

public class NoAutoForwardNoRoadrunner extends LinearOpMode {

    private DcMotor leftFront, leftBack, rightFront, rightBack;

    @Override
    public void runOpMode() {

        // declare motors
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");

        // set direction to reverse for the left side motors
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        double power = 0.2;
        double target = 1440; // This is an estimate by me, it is not exact, change depending on the encoder

        leftFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
        rightFront.setPower(power);

        int current = 0;
        while (current < target) {
            current = (int) utils.average(leftFront.getCurrentPosition(), leftBack.getCurrentPosition(), rightFront.getCurrentPosition(), rightBack.getCurrentPosition());
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);

    }
}
