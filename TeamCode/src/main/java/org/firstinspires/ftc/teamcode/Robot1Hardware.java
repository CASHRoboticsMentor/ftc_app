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

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.tree.DCTree;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a CASHBot.
 *
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Right drive motor:        "right_drive"


 */
public class Robot1Hardware
{
    /* Public OpMode members. */
    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public DcMotor  pusherDrive = null;

    ///Servo Definitions
    public Servo    iconServo    = null;
    public Servo    pusherServo = null;
    public Servo    right_sampler = null;
    public Servo    left_sampler = null;

    //Color Sensor
    public ColorSensor rtSamplerColor = null;
    public DistanceSensor rtSamplerDist = null;
    public ColorSensor leftSamplerColor = null;
    public DistanceSensor leftSamplerDist = null;

    //CONSTANTS
    public static final double ICON_SERVO_UP =  1;
    public static final double ICON_SERVO_DOWN = .25;

//    public static final double PUSHER_MAX_UP = 0;
//    public static final double PUSHER_MAX_DOWN = -90;
    public static final double PUSHER_MOTOR_MAX_DOWN = -90;
    public static final double PUSHER_MOTOR_MAX_UP = -1;


    public static final double RT_SAMPLER_DOWN = .74;
    public static final double RT_SAMPLER_UP = .26;
    public static final double LEFT_SAMPLER_DOWN = .05;
    public static final double LEFT_SAMPLER_UP = .52;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Robot1Hardware(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDrive  = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        pusherDrive = hwMap.get(DcMotor.class, "pusher_drive");

        // Define and Initialize Servos
        iconServo  = hwMap.get(Servo.class, "icon_servo");
        pusherServo= hwMap.get(Servo.class, "pusher_servo");

        // Sampler servo/color sensor configures
        right_sampler= hwMap.get(Servo.class, "right_sampler");
        rtSamplerColor=hwMap.get(ColorSensor.class,"sampler_rt");
        rtSamplerDist=hwMap.get(DistanceSensor.class,"sampler_rt");

        left_sampler= hwMap.get(Servo.class, "left_sampler");
        leftSamplerColor=hwMap.get(ColorSensor.class,"sampler_left");
        leftSamplerDist=hwMap.get(DistanceSensor.class,"sampler_left");

        //Initilization of Motors
        leftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        pusherDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        pusherDrive.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pusherDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
 }

