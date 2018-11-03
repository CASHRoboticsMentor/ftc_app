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

import org.firstinspires.ftc.teamcode.Robot1Hardware;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

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

@Autonomous(name="Robot1: FaceHomeAuto", group="Robot1")
//@Disabled
public class FaceHomeAuto extends LinearOpMode {

    /* Declare OpMode members. */
    Robot1Hardware         robot   = new Robot1Hardware();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();
    static final double     WHEEL_DIAMETER_INCHES   = 4.72;
    static final double     DRIVE_GEAR_REDUCTION    = 1.33;
    static final double     COUNTS_PER_MOTOR_REV    = 240;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);



    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        double leftPosition;
        double rightPosition;
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        //while(opModeIsActive()) {
        {
            //Drive Forward
            double forwardDist = 14.72;  //inches of travel
            double reverseDist = 14.72;  //inches of travel
            DriveForwardToPosition(.6,(int)(forwardDist*COUNTS_PER_INCH));
            sleep(1000);
            DriveReverseToPosition(.6,(int)(reverseDist*COUNTS_PER_INCH));
            sleep(1000);
            TurnLeftToPosition(.5,(int)(forwardDist*COUNTS_PER_INCH));
            sleep(1000);
            TurnRightToPosition(.5,(int)(forwardDist*COUNTS_PER_INCH));
            sleep(1000);

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
        robot.leftDrive.setTargetPosition(distance);
        robot.rightDrive.setTargetPosition(-distance);

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
        robot.leftDrive.setTargetPosition(-distance);
        robot.rightDrive.setTargetPosition(distance);

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
        robot.leftDrive.setPower(power);
        robot.rightDrive.setPower(-power);
    }
    public void DriveRight(double power)
    {
        robot.leftDrive.setPower(-power);
        robot.rightDrive.setPower(power);
    }
    public void Stop()
    {
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }
}
