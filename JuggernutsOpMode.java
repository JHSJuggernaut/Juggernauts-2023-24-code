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
    public DcMotorEx IntakeTilt;
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
        IntakeTilt = hardwareMap.get(DcMotorEx.class, "IntakeTilt");
        liftTilt = hardwareMap.get(DcMotorEx.class, "liftTilt");
        LGrab = hardwareMap.get(Servo.class, "LGrab");
        RGrab = hardwareMap.get(Servo.class, "RGrab");

        topRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomRight.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addData("Hardware", "initialized");
        telemetry.update();
        //done initializing
        
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
            double RX1 = gamepad1.right_stick_x;
            double RY1 = gamepad1.right_stick_y;
            double LY2 = gamepad2.left_stick_y;
            double LX2 = gamepad2.left_stick_x;
            double RX2 = gamepad2.right_stick_x;
            double RY2 = gamepad2.right_stick_y;

            //varibles
            double DSpeed = 1;
            double LP = 0;
            double LPDiff = 0.025;
            
            //functions
            liftOne.setPower(LP+(LPDiff/2));
            liftTwo.setPower(LP-(LPDiff/2));

            //movement
            double topLeftPower = ((LY1 - LX1) - RX1);
            double bottomLeftPower = ((LY1 + LX1) - RX1);
            double topRightPower = ((LY1 + LX1) + RX1);
            double bottomRightPower = ((LY1 - LX1) + RX1);
            //boost
            if(gamepad1.left_bumper) {
                topLeft.setPower(topLeft * DSpeed);
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
            LP  = LY2;
            //tilt
            IntakeTilt.setPower(LX2);
            
            //contorls done

            //motorout
            telemetry.addData("topLeftPower", topLeftPower);
            telemetry.addData("bottomLeftPower", bottomLeftPower);
            telemetry.addData("topRightPower", topRightPower);
            telemetry.addData("bottomRightPower", bottomRightPower);
            telemetry.update();
        }
    }
}