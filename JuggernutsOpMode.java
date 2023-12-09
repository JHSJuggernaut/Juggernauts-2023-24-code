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
    public DcMotorEx PlaneLauncher;
    public DcMotorEx Intake;
    public Servo Servo0;
    public Servo Servo1;
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
        PlaneLauncher = hardwareMap.get(DcMotorEx.class, "PlaneLauncher");
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        Servo0 = hardwareMap.get(CRServo.class, "Servo0");
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
            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double topLeftPower = ((y - -x) - rx);
            double bottomLeftPower = ((-y + x) + rx);
            double topRightPower = ((y + -x) + rx);
            double bottomRightPower = ((-y - x) - rx);

            //drive
            //boost
            if(gamepad1.left_bumper) {
                topLeft.setPower(topLeftPower * 0.625);
                bottomLeft.setPower(bottomLeftPower * 0.625);
                topRight.setPower(topRightPower * 0.625);
                bottomRight.setPower(bottomRightPower * 0.625);
            }
            //drive normal
            else if(!gamepad1.left_bumper) {
                topLeft.setPower(topLeftPower * 0.25);
                bottomLeft.setPower(bottomLeftPower * 0.25);
                topRight.setPower(topRightPower * 0.25);
                bottomRight.setPower(bottomRightPower * 0.25);
            }
            //pully
            //pully up
            if(gamepad1.dpad_up) {
                liftOne.setPower(1);
                liftTwo.setPower(-0.975);
            }
            else if(gamepad1.dpad_up && liftOne.getCurrentPosition() < 0 && liftTwo.getCurrentPosition() > 0) {
                liftOne.setPower(-1);
                liftTwo.setPower(0.975);
            }
            else {
                liftOne.setPower(0);
                liftTwo.setPower(0);
            }
            //pully down
            if(gamepad1.dpad_down) {
                liftOne.setPower(-1);
                liftTwo.setPower(1);
            }
            else if(gamepad1.dpad_down && liftOne.getCurrentPosition() < 0 && liftTwo.getCurrentPosition() > 0) {
                liftOne.setPower(1);
                liftTwo.setPower(-1);
            }
            else {
                liftOne.setPower(0);
                liftTwo.setPower(0);
            }
            //plane
            if(gamepad1.right_bumper){
                PlaneLauncher.setPower(1);
            }
            else {
                PlaneLauncher.setPower(0);
            }
            //Intake
            //intake out
            if(gamepad1.dpad_left){
                Intake.setPower(-1);
            }
            else {
                Intake.setPower(0);
            }
            //intake in
            if(gamepad1.dpad_right){
                Intake.setPower(1);
            }
            else {
                Intake.setPower(0);
            }
            //contorls done
        }
    }
}