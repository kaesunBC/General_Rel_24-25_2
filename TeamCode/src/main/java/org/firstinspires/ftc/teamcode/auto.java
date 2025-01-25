package org.firstinspires.ftc.teamcode;

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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Robot: Auto Drive By Time", group="Robot")
//@Disabled
public class auto extends LinearOpMode {

    /* Declare OpMode members. */

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor vertSlide = null;
    private DcMotor arm = null;
    private DcMotor ascent = null;

    private Servo bucket = null;
    private Servo wrist = null;
    private Servo claw = null;



    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    static final double SLIDE_SPEED = 0.95;
    static final double SLOW_SPEED = 0.09;
    static final double SLOW_SPEEDY = 0.3;
    double leftArmServoPos = 0.5;
    double rightArmServoPos = 0.5;
    double funnelPos = 0.5;


    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        vertSlide = hardwareMap.get(DcMotor.class, "vertSlide");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        runtime.reset();
        //robut runs forwards for 1.5 seconds
        while (opModeIsActive() && (runtime.seconds() < 1.5)) {
            leftFrontDrive.setPower(FORWARD_SPEED);
            leftBackDrive.setPower(FORWARD_SPEED);
            rightFrontDrive.setPower(FORWARD_SPEED);
            rightBackDrive.setPower(FORWARD_SPEED);
            telemetry.addData("forwards Successful", getRuntime());
            telemetry.update();
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();



            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
// robots stops driving
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
        }
// vertical slide moves up for 2.8 seconds
        while (opModeIsActive() && (runtime.seconds() < 3.1 && runtime.seconds() > 0.3)) {
            vertSlide.setPower(.95);//slides up
            telemetry.addData("Runtime", getRuntime());
            telemetry.update();
        }
//bucket "opens"
        while (opModeIsActive() && (runtime.seconds() < 6 && runtime.seconds() > 3.1)) {
            vertSlide.setPower((-.10));
            bucket.setPosition(0.9);
        }

        while (opModeIsActive() && (runtime.seconds() < 6.5 && runtime.seconds() > 6)) {
            leftFrontDrive.setPower(TURN_SPEED);
            leftBackDrive.setPower(TURN_SPEED);
            rightFrontDrive.setPower(-TURN_SPEED);
            rightBackDrive.setPower(-TURN_SPEED);
        }
        while (opModeIsActive() && (runtime.seconds() < 8 && runtime.seconds() > 6.5)) {
            leftFrontDrive.setPower(TURN_SPEED);
            leftBackDrive.setPower(TURN_SPEED);
            rightFrontDrive.setPower(TURN_SPEED);
            rightBackDrive.setPower(TURN_SPEED);
        }

        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        vertSlide.setPower(0);
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
