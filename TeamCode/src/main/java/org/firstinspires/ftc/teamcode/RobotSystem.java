package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevColorSensorV3;




public class RobotSystem {

    //Variables for positions/etc
    public double liftTiltPark = 1;
    public double liftTiltUp = .28;
    public double liftTiltHang1 = 0;
    public double armExtend = .4;
    public double armRetract = 1;
    public double armMiddle = .6;
    public double gripClose = .275;
    public double gripOpen = .5;
    public int armUp = 2000;
    public int armOffFloor = 150;
    public int armToPickup = 10;
    public int TOarmtolowbasket = -950;
    public int TOarmUp = 0;
    public int TOarmOffFloor = -1850;
    public int TOarmToPickup = -2000;



    // Set these for the lift measurement in Ticks  -- dg- still need to be determined
    public int liftLowBar = 1000;
    public int liftHighBar = 2020;
    public int liftDistanceforClip = 525;//amount to drop when clipping specimen
    public int currentLiftHeight =0;
    public boolean liftAutoMode = false;


    public final DcMotorEx leftLift, rightLift, arm;
    public final Servo tiltLift, gripper, extender,  wire;
    public final CRServo intake;
    public RevColorSensorV3 colorSensor;

    public RobotSystem(HardwareMap hardwareMap) {

        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        arm = hardwareMap.get(DcMotorEx.class, "arm");

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setTargetPosition(0);
//        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setTargetPosition(0);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


 //       rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
 //       rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setPower(-.5);
        rightLift.setPower(.5);

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(.4);


        tiltLift = hardwareMap.get(Servo.class, "tiltLift");
        gripper = hardwareMap.get(Servo.class, "gripper");
        intake = hardwareMap.get(CRServo.class, "intake");
        extender = hardwareMap.get(Servo.class, "extender");
//        swingArm = hardwareMap.get(Servo.class, "swingArm");
        wire = hardwareMap.get(Servo.class, "wire");

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");


    }



     public int PrepareToCliponBar(int barnumber) {   // specify the bar number 1 or 2 for clipping

        int runto = 0;
        if (barnumber == 1) {
            runto = liftLowBar;
        } else
            runto = liftHighBar;

        leftLift.setTargetPosition(-runto);
        rightLift.setTargetPosition(runto);
         leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       leftLift.setPower(-.5);
        rightLift.setPower(.5);

        return runto;
    }

    final int ClipOntoBar (int barnumber) {
        int runto = 0;
        if (barnumber == 1) {
            runto = liftLowBar - liftDistanceforClip;
        } else
            runto = liftHighBar - liftDistanceforClip;

        leftLift.setTargetPosition(-runto);
        rightLift.setTargetPosition(runto);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLift.setPower(-.3);
        rightLift.setPower(.3);

        return runto;
    }

    public int LowerLifttoBottom () {
        leftLift.setTargetPosition(0);
        rightLift.setTargetPosition(0);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLift.setPower(.3);
        rightLift.setPower(-.3);
    return 0;
    }


//    public SetLiftPower(double input) {
//        leftLift.setPower(input);
//        rightLift.setPower(-input);
//    }

}
