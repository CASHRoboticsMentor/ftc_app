package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Simple Driver Linear Op Mode", group="Linear Opmode")
//@Disabled

public class SimpleDriveOpMode extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //STEP1:  Define the two motors.
    // You need to define two motors.  Make these relevent to what the robot has.
    // Example context:  private DcMotor MOTORNAME = null;


    @Override
    public void runOpMode() {
        //green texts is what you want it to say.  Try changing it to something else.
        telemetry.addData("Status", "Initialized");
        telemetry.update(); //this sends the data to the phone

        // Initialize the hardware variables. Note that the strings (green text) used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        //
        // leftDrive  = hardwareMap.get(DcMotor.class, "STRING_NAME_IN_ROBOT_CONFIG");

        // Think about where the motors are and which go forward vs reverse.
        //MOTOR_NAME.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            //double leftMotorSpeed;
            //double rightMotorSpeed;

            // This is where your code goes to read the gamepad inputs for going
            // forward/reverse/turn left/turn right
            //

            //need a variable to contain the forward command
            //NOTE: The game pad stick provides a value of -1 to 1 along the x and y directions
            //  so if you move the left stick all the way to the left, the value is -1.
            //  if you let the joystick go to the center, the value is 0
            //  all the way to the right is 1.
            //  Similar with the y direction but up is -1 and down is 1.
            //double drive = -gamepad1.left_stick_y;


            //need a variable to contatin the turning direction
            // double turn  =  gamepad1.right_stick_x;

            //need to set the leftMotorSpeed and right motor speed accordingly so that it does
            //what you want it to do.
            //leftMotorSpeed    = Range.clip(COMMAND, -1.0, 1.0) ;
            //rightMotorSpeed   = Range.clip(COMMAND, -1.0, 1.0) ;

            // Send calculated power to wheels
            //MOTOR_NAME.setPower(leftMotorSpeed);
            //MOTOR_NAME2.setPower(rightMotorSpeed);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }

}
