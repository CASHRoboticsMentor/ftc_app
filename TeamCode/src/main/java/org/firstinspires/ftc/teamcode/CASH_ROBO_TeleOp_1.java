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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains the Teleop mode that controls CASHBot
 */

@TeleOp(name="CASH_ROBO_TeleOp_nc1", group="Linear Opmode")
//@Disabled
public class CASH_ROBO_TeleOp_1 extends LinearOpMode {

    // Declare OpMode members.
    Robot1Hardware         robot   = new Robot1Hardware();   // Use a CASHBot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    private boolean prvLeftSamplerUp = true;  //false is up
    private boolean prvRightSamplerUp = true;  // false is up

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        double slope = (robot.PUSHER_MOTOR_MAX_DOWN-robot.PUSHER_MOTOR_MAX_UP)/(2);
        double bIntercept = - slope;

        //We want to initilize with pusher up.
        //robot.pusherServo.setPosition(robot.PUSHER_MAX_UP);

        //Feedback from the left and right motors for control
        double leftPosition;
        double rightPosition;

        //Variables to drive the left and right drive motors
        double leftPower;
        double rightPower;

        double turnRate = .5;
        //button inputs for using samplers to get gems.

        driveServoToPosition(robot.RT_SAMPLER_UP,robot.right_sampler);
        driveServoToPosition(robot.LEFT_SAMPLER_UP,robot.left_sampler);
        robot.pusherDrive.setTargetPosition((int)robot.PUSHER_MOTOR_MAX_UP);
        double currentPusherPos = robot.pusherDrive.getCurrentPosition();
        driveServoToPosition(robot.ICON_SERVO_UP,robot.iconServo);

        //float r_samplerRate = gamepad2.right_stick_x;
        float pusherRate=0;
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runtime.reset();

        double startTime ;

        //////////////////////////TEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEESSSSSSSSSST
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.pusherDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            startTime = runtime.milliseconds();
            // Setup a variable for each drive wheel to save power level for telemetry
//            double rightSplPosition = gamepad2.right_stick_y;
//            rightSplPosition=Range.clip(rightSplPosition, -1.0, 1.0) ;
//            driveServoToPosition(rightSplPosition,robot.iconServo);
//            telemetry.addData("rightPos",robot.iconServo.getPosition());
//            double leftSplPosition = gamepad2.right_stick_x;
//            leftSplPosition=Range.clip(leftSplPosition, 0, 1.0) ;
//            driveServoToPosition(leftSplPosition,robot.left_sampler);
//            telemetry.addData("leftPos",robot.left_sampler.getPosition());

           // IF WE NEED TO USE MOTOR THIS WILL HAVE TO CHANGE
            pusherRate = gamepad2.left_stick_y;
            double pusherChange = Range.clip(pusherRate, -1.0, 1.0) ;

            double newPosition = currentPusherPos + pusherChange*4;
            currentPusherPos = newPosition;
            telemetry.addData("newPosition: ",(int)newPosition);
            robot.pusherDrive.setTargetPosition((int)newPosition);
            robot.pusherDrive.setPower(1);
            //robot.pusherDrive.setPower(pusherPwr);
            telemetry.addData("Pusher Position",robot.pusherDrive.getCurrentPosition());
           // driveServoByRate(pusherRate,robot.pusherServo);


            //Control of Left Sampler
//            if (prvLeftSamplerUp && leftSplToggle){
//                //down
//                driveServoToPosition(robot.LEFT_SAMPLER_DOWN,robot.left_sampler);
//                prvLeftSamplerUp = false; //true
//            }
//            else {
//                //left up
//                driveServoToPosition(robot.LEFT_SAMPLER_UP,robot.left_sampler);
//                prvLeftSamplerUp = true; //true
//            }
//
//            //Control of Right Sampler
//            if (prvRightSamplerUp && rightSplToggle){
//                //down
//                driveServoToPosition(robot.RT_SAMPLER_DOWN,robot.right_sampler);
//                prvRightSamplerUp = false; //true
//            }
//            else{
//                //right up
//                driveServoToPosition(robot.RT_SAMPLER_UP,robot.right_sampler);
//                prvRightSamplerUp = true; //true
//            }
//            driveServoToPosition(robot.RT_SAMPLER_DOWN,robot.right_sampler);

            // Send calculated power to wheels
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            turn = Range.clip(turn, -turnRate,turnRate);

            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;


            robot.leftDrive.setPower(leftPower );
            robot.rightDrive.setPower(rightPower );
            leftPosition = robot.leftDrive.getCurrentPosition();
            rightPosition = robot.rightDrive.getCurrentPosition();
            telemetry.addData("Left Position (counts)", leftPosition);
            telemetry.addData("Right Position (counts)", rightPosition);

            telemetry.addData("looptime",runtime.milliseconds()-startTime);
            telemetry.update();

            sleep(50);
        }
    }

       public void driveServoByRate(float desDegreesPerSec, Servo servo)
    {

        telemetry.addData("InIconDriver!",servo.getPosition());
        double MaxDegreesPerSec = .05;
        double cmd = MaxDegreesPerSec * desDegreesPerSec;
        double newcmd;
        newcmd = servo.getPosition() + cmd;
        if (newcmd >=1)
        {
            newcmd = 1;
        }
        else if (newcmd <=0)
        {
            newcmd = 0;
        }
        servo.setPosition(newcmd);
    }

    public void driveServoToPosition(double pos, Servo servo)
    {

        servo.setPosition(pos);
        double error = pos - servo.getPosition();
        //telemetry.addData("waiting To Get to Position: ",error);
        //while(Math.abs(error) <= .01 )
        {
            //telemetry.addLine("waiting To Get to Position: ");
            error = pos - servo.getPosition();
        }
    }

    public void driveMotorByRate(double speed)
    {

    }
}
