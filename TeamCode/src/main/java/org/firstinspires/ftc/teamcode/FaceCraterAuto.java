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

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Robot1: FaceCraterAuto", group="Robot1")
//@Disabled
public class FaceCraterAuto extends LinearOpMode {

    /* Declare OpMode members. */
    Robot1Hardware         robot   = new Robot1Hardware();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();
    static final double     WHEEL_DIAMETER_INCHES   = 4.72;
    static final double     DRIVE_GEAR_REDUCTION    = 2.00;
    static final double     COUNTS_PER_MOTOR_REV    = 1120; //1120  240 CoreHex
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        //
        double up = .9;
        double down = .45;
        float iconUpPos = (float)up;
        float iconDownPos = (float)down;
        double upP = .55;
        double downP = .97;
        float pusherUpPos = (float)upP;
        float pusherDownPos = (float)downP;

        driveServoToPositionPause(iconUpPos,robot.iconServo,200); //up
        driveServoToPositionPause((float)robot.RT_SAMPLER_DOWN,robot.right_sampler,1); //up
        driveServoToPositionPause((float)robot.LEFT_SAMPLER_DOWN,robot.left_sampler,1); //up

        //NEED TO PUT MOTOR UP POSITION
        // driveServoToPositionPause(pusherUpPos,robot.pusherServo,200); //up

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.pusherDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.pusherDrive.setTargetPosition(60);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //while(opModeIsActive()) {
        {
            //Driving Steps
            double fwdToSamples = 24.0;  //Drive to the Samples inches of travel
            double fwdTomoveSample = 4;  //just move the sample
            double revForTurn = 14;  //After Sampling need to reverse to get out of way
            double TurnLeft1 = 7.50;  //Turn left around leg of lander
            double fwdCloserToWall = 13; // Drive Fwd to get close to the outside wall
            double turnLeft2 = 5;  //Get aligned to the home base
            double DriveToWall = 32; // Drive Fwd until we get to side wall
            double TurnLeft3 = 3.50; // Line up to home
            double DriveToHome = 54; // Drive Fwd to home
            double TurnLeftToAlign = 1; // Turn a little and line up
            double revToCrater= 87; // Backup until at crater
            driveServoToPositionPause(pusherDownPos,robot.pusherServo,200); //up
            DriveForwardToPosition(1,(int)(fwdToSamples*COUNTS_PER_INCH));
            sleep(100);

            ////Need code to do the sampling
//            driveServoToPositionPause((float)robot.RT_SAMPLER_DOWN,robot.right_sampler,1); //up
//            driveServoToPositionPause((float)robot.LEFT_SAMPLER_DOWN,robot.left_sampler,1); //up
            sleep(1500);
            boolean isYellowAtRight = isGemYellow(robot.rtSamplerColor);
            boolean isYellowAtLeft = isGemYellow(robot.leftSamplerColor);

            if (isYellowAtRight)
            {
                driveServoToPositionPause((float)robot.LEFT_SAMPLER_UP,robot.left_sampler,500);
                //Need to raise center
            }
            else if (isYellowAtLeft)
            {
                driveServoToPositionPause((float)robot.RT_SAMPLER_UP,robot.right_sampler,500);
                //Need to raise Center
            }
            else
            {
                driveServoToPositionPause((float)robot.LEFT_SAMPLER_UP,robot.left_sampler,500);
                driveServoToPositionPause((float)robot.RT_SAMPLER_UP,robot.right_sampler,500);
            }

            ////
            DriveForwardToPosition(1,(int)(fwdTomoveSample*COUNTS_PER_INCH));
            driveServoToPositionPause((float)robot.LEFT_SAMPLER_UP,robot.left_sampler,500);
            driveServoToPositionPause((float)robot.RT_SAMPLER_UP,robot.right_sampler,500);

            DriveReverseToPosition(1,(int)(revForTurn*COUNTS_PER_INCH));
            sleep(100);
            TurnLeftToPosition( 1,(int)(TurnLeft1*COUNTS_PER_INCH));
            sleep(100);
            DriveForwardToPosition(1,(int)(fwdCloserToWall*COUNTS_PER_INCH));
            sleep(100);
            TurnLeftToPosition(1,(int)(turnLeft2*COUNTS_PER_INCH));
            sleep(100);
            DriveForwardToPosition(1,(int)(DriveToWall*COUNTS_PER_INCH));
            sleep(100);
            TurnLeftToPosition(1,(int)(TurnLeft3*COUNTS_PER_INCH));
            sleep(100);
            DriveForwardToPosition(1,(int)(DriveToHome*COUNTS_PER_INCH));
            sleep(100);

            TurnLeftToPosition(1,(int)(TurnLeftToAlign*COUNTS_PER_INCH));
            sleep(100);
            // DROP ICON
            driveServoToPositionPause(iconDownPos,robot.iconServo,1000);
            ///
            DriveReverseToPosition(1,(int)(revToCrater*COUNTS_PER_INCH));
            sleep(100);
            Stop();
            idle();
        }

    }

    public void DriveForwardToPosition(double power,int distance)
    {
        //Reset encoders
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set Target Position
        robot.leftDrive.setTargetPosition(distance);
        robot.rightDrive.setTargetPosition(distance);

        //Set To runto position
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        DriveForward(power);
        double leftPosition, rightPosition;

        while (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())
        {
            leftPosition = (double)robot.leftDrive.getCurrentPosition()/COUNTS_PER_INCH;
            rightPosition = (double)robot.rightDrive.getCurrentPosition()/COUNTS_PER_INCH;
            telemetry.addData("Left Position (inch)", leftPosition);
            telemetry.addData("Right Position (inch)", rightPosition);
            telemetry.update();

        }
        //StopDriving();
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Stop();


    }
    public void DriveReverseToPosition(double power,int distance)
    {
        //Reset encoders
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set Target Position
        robot.leftDrive.setTargetPosition(-distance);
        robot.rightDrive.setTargetPosition(-distance);

        //Set To runto position
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        DriveForward(-power);
        double leftPosition, rightPosition;
        while (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())
        {
            leftPosition = (double)robot.leftDrive.getCurrentPosition()/COUNTS_PER_INCH;
            rightPosition = (double)robot.rightDrive.getCurrentPosition()/COUNTS_PER_INCH;
            telemetry.addData("Left Position (inch)", leftPosition);
            telemetry.addData("Right Position (inch)", rightPosition);
            telemetry.update();

        }
        //StopDriving();
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Stop();
    }

    public void TurnLeftToPosition(double power,int distance)
    {
        //Reset encoders
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set Target Position
        robot.leftDrive.setTargetPosition(-distance);
        robot.rightDrive.setTargetPosition(distance);

        //Set To runto position
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        DriveLeft(power);
        double leftPosition, rightPosition;
        while (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())
        {
            leftPosition = (double)robot.leftDrive.getCurrentPosition()/COUNTS_PER_INCH;
            rightPosition = (double)robot.rightDrive.getCurrentPosition()/COUNTS_PER_INCH;
            telemetry.addData("Left Position (inch)", leftPosition);
            telemetry.addData("Right Position (inch)", rightPosition);
            telemetry.update();

        }
        //StopDriving();
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Stop();
    }
    public void TurnRightToPosition(double power,int distance)
    {
        //Reset encoders
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set Target Position
        robot.leftDrive.setTargetPosition(distance);
        robot.rightDrive.setTargetPosition(-distance);

        //Set To runto position
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        DriveRight(power);
        double leftPosition, rightPosition;
        while (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())
        {
            leftPosition = (double)robot.leftDrive.getCurrentPosition()/COUNTS_PER_INCH;
            rightPosition = (double)robot.rightDrive.getCurrentPosition()/COUNTS_PER_INCH;
            telemetry.addData("Left Position (inch)", leftPosition);
            telemetry.addData("Right Position (inch)", rightPosition);
            telemetry.update();

        }
        //StopDriving();
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Stop();

    }


    public void DriveForward(double power)
    {
        robot.leftDrive.setPower(power);
        robot.rightDrive.setPower(power);
    }
    public void DriveLeft(double power)
    {
        robot.leftDrive.setPower(-power);
        robot.rightDrive.setPower(power);
    }
    public void DriveRight(double power)
    {
        robot.leftDrive.setPower(power);
        robot.rightDrive.setPower(-power);
    }
    public void Stop()
    {
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }
    public void driveServoToPositionPause(float position,Servo servo, long milliseconds)
    {
        servo.setPosition(position);
        sleep(milliseconds);
    }

    public boolean isGemYellow(ColorSensor thisColSensor){
        // hsvValues is an array that will hold the hue, saturation, and value information.
        boolean foundYellow;
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        int count = 0;
        int numOfSamples = 5;
        long sleepTime = 200;
        float sampleData = 0;

        while(count < numOfSamples){
            Color.RGBToHSV((int) (thisColSensor.red() * SCALE_FACTOR),
                    (int) (thisColSensor.green() * SCALE_FACTOR),
                    (int) (thisColSensor.blue() * SCALE_FACTOR),
                    hsvValues);
            sampleData = sampleData + hsvValues[0];
            telemetry.addData("this sample",hsvValues[0]);
            count = count + 1;
            telemetry.update();
            sleep(sleepTime);
        }
        float averageValue = sampleData/numOfSamples;
        telemetry.addData("Average is: ",averageValue);
        if (averageValue < 75)
        {
            foundYellow = true;
        }
        else
        {
            foundYellow = false;
        }
        return foundYellow;
    }
}
