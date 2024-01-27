package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp()

public class JuggernutsOpMode extends LinearOpMode {
    //define vars
    public DcMotorEx topLeft;
    public DcMotorEx topRight;
    public DcMotorEx bottomLeft;
    public DcMotorEx bottomRight;
    public DcMotorEx liftOne;
    public DcMotorEx liftTwo;
    public DcMotorEx intakeTilt;
    public DcMotorEx liftTilt;
    public Servo LGrab;
    public Servo RGrab;
    boolean canMoveUp;
    boolean canMoveDown;

    //code
    public void runOpMode() {
        //intilizing
        topLeft = hardwareMap.get(DcMotorEx.class, "topLeft");
        topRight = hardwareMap.get(DcMotorEx.class, "topRight");
        bottomLeft = hardwareMap.get(DcMotorEx.class, "bottomLeft");
        bottomRight = hardwareMap.get(DcMotorEx.class, "bottomRight");
        liftOne = hardwareMap.get(DcMotorEx.class, "liftOne");
        liftTwo = hardwareMap.get(DcMotorEx.class, "liftTwo");
        intakeTilt = hardwareMap.get(DcMotorEx.class, "intakeTilt");
        liftTilt = hardwareMap.get(DcMotorEx.class, "liftTilt");
        LGrab = hardwareMap.get(Servo.class, "LGrab");
        RGrab = hardwareMap.get(Servo.class, "RGrab");

        topRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomRight.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeTilt.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Hardware", "initialized");
        telemetry.update();
        //done initializing
        
        LGrab.setPosition(0);
        RGrab.setPosition(1);
        
        //waiting for start
        telemetry.addData("software", "Waiting for start");
        telemetry.update();
        waitForStart();
        telemetry.addData("software", "Start!!");
        telemetry.update();
        //started

        //emergency stop
        if (isStopRequested()) return;
        topLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        topRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        bottomLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        bottomRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        liftOne.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        liftTwo.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        while(opModeIsActive()) {
            //sticks
            double LY1 = gamepad1.left_stick_y;
            double LX1 = gamepad1.left_stick_x;
            double LT1 = gamepad1.left_trigger;
            double RX1 = gamepad1.right_stick_x;
            double RY1 = gamepad1.right_stick_y;
            double RT1 = gamepad1.right_trigger;
            double LY2 = gamepad2.left_stick_y;
            double LX2 = gamepad2.left_stick_x;
            double LT2 = gamepad2.left_trigger;
            double RX2 = gamepad2.right_stick_x;
            double RY2 = gamepad2.right_stick_y;
            double RT2 = gamepad2.right_trigger;
            double LGrabPos = LGrab.getPosition();
            double RGrabPos = RGrab.getPosition();
            double liftOnePos = liftOne.getCurrentPosition();
            double liftTwoPos = liftTwo.getCurrentPosition();
            double liftTiltPos = liftTilt.getCurrentPosition();
            double intakeTiltPos = intakeTilt.getCurrentPosition();
            double topLeftPos = topLeft.getCurrentPosition();
            double topRightPos = topRight.getCurrentPosition();
            double bottomLeftPos = bottomLeft.getCurrentPosition();
            double bottomRightPos = bottomRight.getCurrentPosition();
            

            //varibles
            double DSpeed = 1;
            double joystickDeadzone = 0.1;
            double liftSpeed = 0.75;
            double intakeTiltSpeed = 0.5;
            
            //functions

            //movement
            double topLeftPower = ((LY1 - LX1) - RX1);
            double bottomLeftPower = ((LY1 + LX1) - RX1);
            double topRightPower = ((LY1 + LX1) + RX1);
            double bottomRightPower = ((LY1 - LX1) + RX1);
            //boost
            if(gamepad1.left_bumper) {
                topLeft.setPower(topLeftPower * DSpeed);
                bottomLeft.setPower(bottomLeftPower * DSpeed);
                topRight.setPower(topRightPower * DSpeed);
                bottomRight.setPower(bottomRightPower * DSpeed);
            }
            //drive normal
            else if(!gamepad1.left_bumper) {
                topLeft.setPower(topLeftPower * 0.5);
                bottomLeft.setPower(bottomLeftPower * 0.5);
                topRight.setPower(topRightPower * 0.5);
                bottomRight.setPower(bottomRightPower * 0.5);
            }
            
            //lift
            if (gamepad2.dpad_up) {
                liftOne.setPower(-liftSpeed);
                liftTwo.setPower(liftSpeed);
            }
            else if (gamepad2.dpad_down) {
                liftOne.setPower(liftSpeed);
                liftTwo.setPower(-liftSpeed);
            }
            else {
                liftOne.setPower(0);
                liftTwo.setPower(0);
            }
            //tilt
            liftTilt.setPower(RY2);
            //intaketilt
            if (gamepad2.dpad_left) {
                intakeTilt.setPower(intakeTiltSpeed);
            }
            else if (gamepad2.dpad_right) {
                intakeTilt.setPower(-intakeTiltSpeed);
            }
            else {
                intakeTilt.setPower(0);
            }
            //intake
            if (LT2 > 0.75){
                LGrab.setPosition(1);
            }
            else if (gamepad2.left_bumper) {
                LGrab.setPosition(0);
            }
            if (RT2 > 0.75){
                RGrab.setPosition(0);
            }
            else if (gamepad2.right_bumper) {
                RGrab.setPosition(1);
            }
            //motorout
            telemetry.addData("topLeftPower", topLeftPower);
            telemetry.addData("topLeftPos", topLeftPos);
            telemetry.addData("topRightPower", topRightPower);
            telemetry.addData("topRightPos", topRightPos);
            telemetry.addData("bottomLeftPower", bottomLeftPower);
            telemetry.addData("bottomLeftPos", bottomLeftPos);
            telemetry.addData("bottomRightPower", bottomRightPower);
            telemetry.addData("bottomRightPos", bottomRightPos);
            telemetry.addData("liftOnePower", LY2);
            telemetry.addData("liftOnePos", liftOnePos);
            telemetry.addData("liftTwoPower", LY2);
            telemetry.addData("liftTwoPos", liftTwoPos);
            telemetry.addData("intakeTiltPos", intakeTiltPos);
            telemetry.update();
        }
    }
}