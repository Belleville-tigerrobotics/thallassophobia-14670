/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.Vector;

/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */ // test

@TeleOp(name="Basic: Linear OpMode", group="Linear OpMode")
//@Disabled
public class BasicOpMode_Linear extends LinearOpMode {

    // Declare OpMode members.
    private double trigger = 0;

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    private MecanumDrive drive;
    private RobotSystem system;
    private static double cubicDelinear(double input){
        if (input < 0){
            return -(input*input*input*input*.9);
        }else{
            return input*input*input*input*.9;
        }
    }

//    public static Vector2d rotateVector(Vector2d input, double roatationAngle){
//
//    }
    private double SLEW_RATE= 0.15;
    private double prevX = 0;
    private double prevY = 0;
    private double prevR = 0;
    private double returnvalue;


    private double slewX(double inputX) {
        if (SLEW_RATE < Math.abs(inputX - prevX)) {
            returnvalue=inputX;// Can slew
            if (inputX < prevX) {
                returnvalue = prevX - SLEW_RATE;
            } else if (inputX > prevX) {
                returnvalue = prevX + SLEW_RATE;
            }
        }
        prevX=inputX;
        if (inputX == 0) {
            prevX=0;
            returnvalue=0;
        }
        return returnvalue; // Close enough that you can just use input
    }
    private double slewY(double inputY) {
        if (SLEW_RATE < Math.abs(inputY - prevY)) {
            returnvalue=inputY;// Can slew
            if (inputY < prevY) {
                returnvalue = prevY - SLEW_RATE;
            } else if (inputY > prevY) {
                returnvalue = prevY + SLEW_RATE;
            }
        }
        prevY=inputY;
        if (inputY==0) {
            prevY=0;
            returnvalue=0;
        }
        return returnvalue; // Close enough that you can just use input
    }

    private double slewR(double inputR) {
        if (SLEW_RATE < Math.abs(inputR - prevR)) {
            returnvalue=inputR;// Can slew
            if (inputR < prevY) {
                returnvalue = prevR - SLEW_RATE;
            } else if (inputR > prevR) {
                returnvalue = prevR + SLEW_RATE;
            }
        }
        prevR=inputR;
        if (inputR==0) {
            prevR=0;
            returnvalue=0;
        }
        return returnvalue; // Close enough that you can just use input
    }



    public double robotdirection ;

    private Vector2d rotateMe(double x, double y, double angle) {

        double x1 = (double)(x * Math.cos(angle) - y * Math.sin(angle));

        double y1 = (double)(x * Math.sin(angle) + y * Math.cos(angle)) ;

        return new Vector2d(x1, y1);
    }


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        system = new RobotSystem(hardwareMap);
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
//        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
//        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

  //set the intiial drive pose
            drive.pose = (new Pose2d(10, 15, Math.toRadians(0)));//was 90


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
// DG-This is handled in the MecanumDrive class
//        leftDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            //just reset the pose
            if (gamepad1.back) {
                drive.pose = (new Pose2d(10, 15, Math.toRadians(0)));//was 90
            }

            Pose2d poseEstimate = drive.pose;

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            double yval;
//            yval = slewY(gamepad1.left_stick_y);
            yval = gamepad1.left_stick_y;

            double xval;
//            xval = slewX(gamepad1.left_stick_x);
            xval = gamepad1.left_stick_x;


            robotdirection = poseEstimate.heading.toDouble();
            Vector2d adjustedVector = rotateMe(-yval,-xval, -robotdirection );

            PoseVelocity2d input = new PoseVelocity2d(
                    new Vector2d(adjustedVector.x,adjustedVector.y) ,0);

       //     PoseVelocity2d i2 = new PoseVelocity2d(new Rotation2d(poseEstimate.heading.toDouble()).times(new pose2d(5,10,0)));
            telemetry.addData("Direction",robotdirection);
            telemetry.addData("y",yval);
            telemetry.addData("y2", input.linearVel.y);

            telemetry.addData("x",xval);
            telemetry.addData("x2",input.linearVel.x);
            telemetry.addData("heading",poseEstimate.heading);
            telemetry.addData("pose",poseEstimate);
            telemetry.addData("poseHeading",poseEstimate.heading.toDouble());
telemetry.update();

            PoseVelocity2d rotatedinput = new PoseVelocity2d(
  //                  input.linearVel,  (slewR(cubicDelinear(-gamepad1.right_stick_x)*.7)));
                    input.linearVel,  (cubicDelinear(-gamepad1.right_stick_x)*.7));

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately

            drive.setDrivePowers(  rotatedinput );

//don't forget to do this every loop to ensure our location gets updated
            drive.updatePoseEstimate();
// Here's where we put the other stuff
            if (gamepad1.dpad_up) {
                system.arm.setPower(.4);
            } else if (gamepad1.dpad_down) {
                    system.arm.setPower(-.4);
                } else {
                system.arm.setPower(0);
            }
            if (gamepad1.y) {
                system.intake.setPower(-.9);
            } else if (gamepad1.x) {
                system.intake.setPower(1);
            } else {
                system.intake.setPower(-.05);
            }

            if (gamepad1.left_bumper) {
                system.tiltLift.setPosition(.28);
            }
            if (gamepad1.right_bumper) {
                system.tiltLift.setPosition(1);
            }
            if (gamepad1.a) {
                system.gripper.setPosition(.5);
            }
            if (gamepad1.b) {
                system.gripper.setPosition(0);  //0 is closed
            }

            trigger = (gamepad1.left_trigger - gamepad1.right_trigger )*.5 ;

            system.leftLift.setPower(trigger);
            system.rightLift.setPower(-trigger);

            if (gamepad1.dpad_right) {
                system.extender.setPosition(.4);
            }
            if (gamepad1.dpad_left) {
                system.extender.setPosition(1);
            }


        }
    }
}
