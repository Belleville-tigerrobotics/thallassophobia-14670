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
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;



/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="TigerTeleop1", group="")
//@Disabled
public class TigerTeleop1 extends OpMode
{


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    RobotSystem system ;
    MecanumDrive drive ;
    public boolean modeSelection = false;
    public boolean triggeriszero = true;
    public double speedx = 0;
    public double speedy = 0;
    public double speedr = 0;
    public double slewrate = .0001;
    public boolean pivotarmmovement = false;
    public int liftdirection = 0; // (-1, 0, 1)
    public double previousLift = 0;
    public double liftTelemetry = 0;
    public boolean liftInMotion = false;
    public double previousTrigger = 0;
    public double trigger=0;
    public boolean rightbumperpressed = false;

    public PoseVelocity2d ourpose;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        //Initialize the bot

        system = new RobotSystem(hardwareMap);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        system.gripper.setPosition(system.gripClose);  //0 is closed

        // Open the robot up for readyness
//        system.arm.setTargetPosition(1710);
//        system.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        system.arm.setPower(.5);
//        system.tiltLift.setPosition(system.liftTiltUp);

        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
        system.wrist.setPosition(system.wristExtend);

    }


    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {


        previousLift = liftTelemetry;
        liftTelemetry = system.rightLift.getCurrentPosition();
        telemetry.addData( "LiftLeft",system.leftLift.getCurrentPosition());
        telemetry.addData( "LiftRight",system.rightLift.getCurrentPosition());

        if (previousLift < liftTelemetry) {
            liftdirection = 1; //moving up
        } else if (previousLift > liftTelemetry) {
            liftdirection = -1; //moving down
        } else {
            liftdirection =0; //not moving
        }

        double armTelemetry = system.arm.getCurrentPosition();
        telemetry.addData( "Arm",armTelemetry);
        //Some special Modes for the Accessories Driver (controller b)

        if (gamepad2.back) { // here we put some special 2 button mode selectors
            modeSelection = true;
            if (gamepad2.a) {
                system.wrist.setPosition(system.wristExtend);
            }
            if (gamepad2.b) {
                system.wrist.setPosition(system.wristRetract);
            }
            if (gamepad2.y) {
                system.SetLiftToGrabFromWall();

            }
        } else {modeSelection = false;}


        if (gamepad1.y) {
            system.SetLiftToGrabFromWall();
        }
        if (gamepad1.a) {
            system.SetLiftTopullClip ();
        }

// Use the #2 bumpers for the riser

        //this code block manages the riser manual movements.  works together with auto-mode as well
        previousTrigger = trigger;
        trigger = (gamepad2.left_trigger - gamepad2.right_trigger )*.5 ;
        if (system.liftAutoMode) {
            system.currentLiftHeight=system.rightLift.getCurrentPosition();
            if (Math.abs(trigger)>.1) { system.liftAutoMode = false; }
        }
        else {
            if (trigger > .4) {
                system.currentLiftHeight = system.currentLiftHeight + 20;
            }
            if (trigger > .2) {
                system.currentLiftHeight = system.currentLiftHeight + 5;
            }
            if (trigger < -.4) {
                system.currentLiftHeight = system.currentLiftHeight - 20;
            }
            if (trigger < -.2) {
                system.currentLiftHeight = system.currentLiftHeight - 5;
            }
            if (system.currentLiftHeight < 0) {
                system.currentLiftHeight = 0;
            }
            if (system.currentLiftHeight > 3000) {
                system.currentLiftHeight = 3000;
            }
            system.leftLift.setTargetPosition(-system.currentLiftHeight);
            system.rightLift.setTargetPosition(system.currentLiftHeight);
        }


// Operate the intake--runs always.  X will expel
        if (gamepad2.x &&!modeSelection) {
            system.intake.setPower(-1);}
        else  {
            system.intake.setPower(1);
        }

//Operate the gripper @2 A and b
        if (gamepad2.a &&!modeSelection) {
            system.gripper.setPosition(system.gripOpen);
        } else {
            system.gripper.setPosition(system.gripClose);  //0 is closed
        }

// #2 use the bumpers for the arm pivot
        if (rightbumperpressed && !gamepad2.right_bumper) {
            system.tiltLift.setPosition(system.liftTiltPark);//park the lift to get it out of the way

            system.arm.setTargetPosition(system.TOarmOffFloor);
            pivotarmmovement = true;
            rightbumperpressed = false;
        }
        if (gamepad2.right_bumper) {
            system.tiltLift.setPosition(system.liftTiltPark);//park the lift to get it out of the way
            system.wrist.setPosition(system.wristExtend);
            system.arm.setTargetPosition(system.TOarmToPickup);
            pivotarmmovement = true;
            rightbumperpressed = true;
        }
        if (gamepad2.left_bumper) {
            system.arm.setTargetPosition(system.TOarmUp);
            pivotarmmovement = false;
        }
        if (gamepad2.y &&!modeSelection) {
            system.arm.setTargetPosition(system.TOarmtohighbasket);
        }

//arm extend/retract
        if (gamepad2.dpad_right) {
            system.extender.setPosition(system.armExtend);
        }
        if (gamepad2.dpad_left) {
            system.extender.setPosition(system.armRetract);
        }
        if (gamepad2.dpad_up) {
            system.extender.setPosition(system.armMiddle);
        }




//Riser tilt
        if (gamepad1.dpad_up) {
            system.tiltLift.setPosition(system.liftTiltUp);
        }
        if (gamepad1.dpad_down) {
            system.tiltLift.setPosition(system.liftTiltPark);
        }
        if (gamepad1.dpad_left) {
            system.tiltLift.setPosition(system.liftTiltHang1);
        }

//        if (gamepad1.a) {
//            system.swingArm.setPosition(1);//
    //    }
 //       if (gamepad1.b) {
 //           system.swingArm.setPosition(0);
//        }
        if (gamepad1.x) {
            system.wire.setPosition(1);
        }
        if (gamepad1.y) {
            system.wire.setPosition(0);
        }



        //DriveCode
        //just reset the pose
        if (gamepad1.back) {
            drive.pose = (new Pose2d(10, 15, Math.toRadians(0)));//was 90
        }

        Pose2d poseEstimate = drive.pose;

        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading
        double yval;
//            yval = slewY(gamepad1.left_stick_y);
        yval = gamepad1.left_stick_y *.6;

        double xval;
//            xval = slewX(gamepad1.left_stick_x);
        xval = gamepad1.left_stick_x *.6;


        double robotdirection = poseEstimate.heading.toDouble();
        Vector2d adjustedVector = rotateMe(-yval,-xval, -robotdirection );

        PoseVelocity2d input = new PoseVelocity2d(
                new Vector2d(adjustedVector.x,adjustedVector.y) ,0);

        PoseVelocity2d rotatedinput = new PoseVelocity2d(
                //                  input.linearVel,  (slewR(cubicDelinear(-gamepad1.right_stick_x)*.7)));
                input.linearVel,  (cubicDelinear(-gamepad1.right_stick_x)*.6));

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately

        drive.setDrivePowers(  rotatedinput );

//don't forget to do this every loop to ensure our location gets updated
        drive.updatePoseEstimate();




        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }








    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    private Vector2d rotateMe(double x, double y, double angle) {

        double x1 = (double)(x * Math.cos(angle) - y * Math.sin(angle));

        double y1 = (double)(x * Math.sin(angle) + y * Math.cos(angle)) ;

        return new Vector2d(x1, y1);
    }

    private static double cubicDelinear(double input){
        if (input < 0){
            return -(input*input*input*input*.9);
        }else{
            return input*input*input*input*.9;
        }
    }


}
