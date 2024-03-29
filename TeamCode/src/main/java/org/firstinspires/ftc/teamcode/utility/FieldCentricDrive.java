package org.firstinspires.ftc.teamcode.utility;

import static java.lang.Math.abs;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class FieldCentricDrive {
    DcMotor fl, fr, bl, br;
    IMU imu;
    double motorspeeed = 0.5;
    /**
     * Initialize motors
     * @param hardwareMap hardwareMap from opmode
     */
    public FieldCentricDrive(HardwareMap hardwareMap){
        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");
        //reverse motors
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);
        imu.resetYaw();
    }
    public void resetIMU(){
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);
        imu.resetYaw();
    }
    public void setSpeed(double motorspeeed){
        this.motorspeeed = motorspeeed;
    }
    /**
     * Initialize motors
     * @param hardwareMap hardwareMap from opmode
     * @param motorspeed speed of motors
     */
    public FieldCentricDrive(HardwareMap hardwareMap, double motorspeed){
        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");
        //reverse motors
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);
        this.motorspeeed = motorspeed;
    }
    /**
     * Drive based off of gamepad inputs
     * @param gpx gamepad x input
     * @param gpy gamepad y input
     * @param rx gamepad rotation input
     */
    public void drive(double gpx, double gpy, double rx) {
        double botHeading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);//might be degrees
        double rotX = gpx * Math.cos(botHeading) - gpy * Math.sin(botHeading);
        double rotY = gpx * Math.sin(botHeading) + gpy * Math.cos(botHeading);


        double denominator = Math.max(abs(gpy) + abs(gpx) + abs(rx), 1);
        double flp = -(rotY + rotX + rx) / denominator;
        double blp = -(rotY - rotX + rx) / denominator;
        double frp = -(rotY - rotX - rx) / denominator;
        double brp = -(rotY + rotX - rx) / denominator;

        fl.setPower(motorspeeed*flp);
        bl.setPower(motorspeeed*blp);
        fr.setPower(motorspeeed*frp);
        br.setPower(motorspeeed*brp);
    }
    /*
    dx = distance sensor x
    dy = distance sensor y
    targetx
    targety
    posx = 72 - dx
    posy = -(72 - dy)
    delta_x = targetx - posx
    delta_y = targety - posy
    magnitude = sqrt(delta_x^2 + delta_y^2)   * 1.4
    drive(delta_x/magnitude, delta_y/magnitude, 0)
     */
    /**
     * Drive based off of gamepad
     * More functionality because of more access to controls
     * @param gamepad1 gamepad 1
     */
    public void drive(Gamepad gamepad1){
        double x = gamepad1.left_stick_x*1.1;
        double y = -gamepad1.left_stick_y;
        double rx = -gamepad1.right_stick_x;
        double botHeading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);//might be degrees
        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);


        double denominator = Math.max(abs(y) + abs(x) + abs(rx), 1);
        double flp = -(rotY + rotX + rx) / denominator;
        double blp = -(rotY - rotX + rx) / denominator;
        double frp = -(rotY - rotX - rx) / denominator;
        double brp = -(rotY + rotX - rx) / denominator;

        flp = flp * (1 - gamepad1.right_trigger);
        blp = blp * (1 - gamepad1.right_trigger);
        frp = frp * (1 - gamepad1.right_trigger);
        brp = brp * (1 - gamepad1.right_trigger);
        flp = flp * (1 + 2*gamepad1.left_trigger);
        blp = blp * (1 + 2*gamepad1.left_trigger);
        frp = frp * (1 + 2*gamepad1.left_trigger);
        brp = brp * (1 + 2*gamepad1.left_trigger);


        fl.setPower(motorspeeed*flp);
        bl.setPower(motorspeeed*blp);
        fr.setPower(motorspeeed*frp);
        br.setPower(motorspeeed*brp);
    }
    public void orient(double heading){
        double currentHeading = getHeading();
        double error = heading - currentHeading;
        while(abs(error) > 10){
            currentHeading = getHeading(AngleUnit.DEGREES);
            error = heading - currentHeading;
            if(error > 0){
                fl.setPower(0.2);
                bl.setPower(0.2);
                fr.setPower(-0.2);
                br.setPower(-0.2);
            }else{
                fl.setPower(-0.2);
                bl.setPower(-0.2);
                fr.setPower(0.2);
                br.setPower(0.2);
            }
        }
    }
    /**
     * Get robot heading in radians
     * @return robot heading
     */
    public double getHeading(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
    /**
     * Get robot heading in a specified angle unit
     * @param angleUnit angle unit to return heading in
     * @return robot heading
     */
    public double getHeading(AngleUnit angleUnit){
        return imu.getRobotYawPitchRollAngles().getYaw(angleUnit);
    }

    /**
     * Stop all motors
     */
    public void stop(){
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
    }
    public void setAll(double power){
        fl.setPower(power);
        bl.setPower(power);
        fr.setPower(power);
        br.setPower(power);
    }

    /**
     * Turn to specific angle relative to starting position
     */
    public void orientToAngle(double targetAngle) {
        final double TOLERANCE = 0.5; // Degrees, adjust as needed for precision
        final double MIN_SPEED = 0.2; // Minimum motor speed to overcome static friction
        final double MAX_SPEED = 0.7; // Maximum motor speed for rotation
        double currentHeading = getHeading(AngleUnit.DEGREES); // Current robot heading
        double error = AngleUnit.normalizeDegrees(targetAngle - currentHeading); // Calculate initial error

        // Continue adjusting while the error is outside the tolerance
        while (Math.abs(error) > TOLERANCE) {
            currentHeading = getHeading(AngleUnit.DEGREES); // Update current heading
            error = AngleUnit.normalizeDegrees(targetAngle - currentHeading); // Recalculate error

            // Proportional control for motor speed based on error magnitude
            double speed = (error / 180) * (MAX_SPEED - MIN_SPEED) + MIN_SPEED;
            speed = Math.copySign(Math.max(Math.abs(speed), MIN_SPEED), error); // Ensure minimum speed and correct direction

            // Adjust motor speeds for rotation
            if (error > 0) {
                fl.setPower(speed);
                bl.setPower(speed);
                fr.setPower(-speed);
                br.setPower(-speed);
            } else {
                fl.setPower(-speed);
                bl.setPower(-speed);
                fr.setPower(speed);
                br.setPower(speed);
            }
        }

        // Stop the robot once the target heading is reached
        stop();
    }

}
