/* Copyright (c) 2021 FIRST. All rights reserved.
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

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Omni Linear OpMode", group="Linear OpMode")
//@Disabled
public class TeleOpInitial extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor vertSlide = null;
    private DcMotor arm = null;
  //  private DcMotor ascent = null;

    private Servo bucket = null;
    private Servo elbow = null;
    private Servo wrist = null;
    private Servo claw = null;

    public static final double BUCKET_OUT = 0.25;

    public static final double BUCKET_IN = 1;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "lf");
        leftBackDrive = hardwareMap.get(DcMotor.class, "lb");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rb");
        vertSlide = hardwareMap.get(DcMotor.class, "vertSlide");
    //    ascent = hardwareMap.get(DcMotor.class, "ascent");
        arm = hardwareMap.get(DcMotor.class, "arm");
        elbow = hardwareMap.get(Servo.class, "elbow");
        wrist = hardwareMap.get(Servo.class, "wrist");
        bucket = hardwareMap.get(Servo.class, "bucket");
        claw = hardwareMap.get(Servo.class, "claw");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        vertSlide.setDirection(DcMotor.Direction.FORWARD);
    //    ascent.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);
        elbow.setDirection((Servo.Direction.FORWARD));
        wrist.setDirection((Servo.Direction.FORWARD));
        bucket.setDirection((Servo.Direction.FORWARD));
        claw.setDirection((Servo.Direction.FORWARD));

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //arm.setPower(0.1);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // VERTICAL SLIDE CODE //

            double elbowPos = 0.5;

            if (gamepad2.left_trigger > .95) { // Slide goes up
                vertSlide.setPower(.55);
            } else if (gamepad2.right_trigger > .95) { // Slide goes down
                vertSlide.setPower(-.55);
            } else {
                vertSlide.setPower(0.00);
            }

            // ARM CODE //
            if (gamepad2.left_stick_y > .5) { // Arm goes towards robot front
                arm.setPower(.40);
            } else if (gamepad2.left_stick_y < -.5) { // Arm moves towards bucket
                arm.setPower(-.40);
            } else {
                arm.setPower(0.00);
            }

            // ASCENT/LIFT CODE //
//            if (gamepad1.left_bumper) { // Ascent goes up
//                ascent.setPower(.55);
//            } else if (gamepad1.right_bumper) { // Ascent goes down
//                ascent.setPower(-.55);
//            } else {
//                ascent.setPower(0);
//            }

            // Bucket CODE //
            if (gamepad2.dpad_up) { // Bucket tips out
                 bucket.setPosition(BUCKET_OUT);
            }
            if (gamepad2.dpad_down) { // Bucket "resets"
                bucket.setPosition(BUCKET_IN);
            }

            // Elbow CODE //
            if (gamepad2.right_stick_x > 0) { // elbow rotates counter clockwise
                elbowPos = elbowPos - 0.9;
                elbow.setPosition(elbowPos);
            }
            if (gamepad2.right_stick_x < 0) { // elbow rotates clockwise
                elbowPos = elbowPos + 0.2;
                elbow.setPosition(elbowPos);
            }


            // WRIST CODE //
            if (gamepad2.a) { // Rotate Wrist Towards
                wrist.setPosition(0.7);
            }
            if (gamepad2.b) {  // Rotate Wrist Away
                wrist.setPosition(0.0);
            }

            // CLAW CODE //
            if (gamepad2.y) { // Claw close
                claw.setPosition(0.75);
            }
            if (gamepad2.x) { // Claw open (og=8,4)
                claw.setPosition(0.3);
            }


            // SLOW MO - CHANGE THESE VALUES ACCORDING TO SPEED!!!!!!!
            if (gamepad1.dpad_left) {
                leftFrontDrive.setPower(0.1);
                rightFrontDrive.setPower(-0.1);
                leftBackDrive.setPower(0.1);
                rightBackDrive.setPower(-0.1);
            }
            if (gamepad1.dpad_right) {
                leftFrontDrive.setPower(-0.1);
                rightFrontDrive.setPower(0.1);
                leftBackDrive.setPower(-0.1);
                rightBackDrive.setPower(0.1);
            }
            if (gamepad1.dpad_down) {
                leftFrontDrive.setPower(-0.1);
                rightFrontDrive.setPower(-0.1);
                leftBackDrive.setPower(-0.1);
                rightBackDrive.setPower(-0.1);
            }
            if (gamepad1.dpad_up) {
                leftFrontDrive.setPower(0.1);
                rightFrontDrive.setPower(0.1);
                leftBackDrive.setPower(0.1);
                rightBackDrive.setPower(0.1);
            }


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }
    }
    }























