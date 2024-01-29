package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous()

public class JuggernutsAuto extends LinearOpMode {
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
        
        //emergency stop
        if (isStopRequested()) return;
        topLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        topRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        bottomLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        bottomRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        liftOne.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        liftTwo.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeTilt.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        liftTilt.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        
        //start possion
        LGrab.setPosition(1);
        RGrab.setPosition(0);

        //possions
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
        boolean canMoveUp;
        boolean canMoveDown;
        double DSpeed = 0.5;
        double intakeTiltSpeed = 0.5;
        

        //waiting for start
        telemetry.addData("software", "Waiting for start");
        telemetry.update();
        waitForStart();
        telemetry.addData("software", "Start!!");
    telemetry.update();
        //code
        strafe(DSpeed,1);
        pause(1);
        strafe(-DSpeed,1);
        pause(1);
        drive(DSpeed,1);
        pause(1);
        drive(-DSpeed,1);
        pause(1);
        trunonpoint(DSpeed,1);
        pause(1);
        trunonpoint(-DSpeed,1);
        pause(1);
        
    }
    //functions
    public void pause(int time) {
        topLeft.setPower(0);
        topRight.setPower(0);
        bottomLeft.setPower(0);
        bottomRight.setPower(0);
        sleep(time*1000);
    }
    public void drive(double power,int time) {
        topLeft.setPower(-power);
        topRight.setPower(-power);
        bottomLeft.setPower(-power);
        bottomRight.setPower(-power);
        sleep(time*1000);
    }
    public void strafe(double power,int time) {
        topLeft.setPower(-power);
        topRight.setPower(power);
        bottomLeft.setPower(power);
        bottomRight.setPower(-power);
        sleep(time*1000);
    }
    public void trunonpoint (double power,int time) {
        topLeft.setPower(-power);
        topRight.setPower(power);
        bottomLeft.setPower(-power);
        bottomRight.setPower(power);
        sleep(time*1000);
    }
}