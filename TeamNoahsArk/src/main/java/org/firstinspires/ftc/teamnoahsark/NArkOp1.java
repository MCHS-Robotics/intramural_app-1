/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamnoahsark;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="NArkOp1.2", group="TeleOp")  // @Autonomous(...) is the other common choice
//@Disabled
public class NArkOp1 extends OpMode
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();    //How long the robot has been running
    private DcMotor left, right;    //Declare left and right motors
    private DcMotor lift;   //Declare lift motor
    private int lowerLiftLim;   //Lowest position the lift arm will hold
    private double liftThresh;  //Minimum value for triggers before lift will move

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        lift = hardwareMap.dcMotor.get("lift");

        right.setDirection(DcMotorSimple.Direction.REVERSE);
        left.setDirection(DcMotorSimple.Direction.FORWARD);

        lowerLiftLim = lift.getCurrentPosition();
        liftThresh = 0.075;
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());
        telemetry.addData("Motor Power:", "Left: %f  Right %f", left.getPower(), right.getPower());
        telemetry.addData("Lift Power:", "Lift: " + lift.getPower());
        telemetry.addData("Lift Position:", "%d encoder units", lift.getCurrentPosition());

        /**
         * Set motor power according to joystick readings
         * Left joystick Y-axis controls forward/backward
         * Right joystick X-axis controls rotation
         */
        left.setPower(Range.clip(-gamepad1.right_stick_x+gamepad1.left_stick_y, -1, 1));
        right.setPower(Range.clip(gamepad1.right_stick_x+gamepad1.left_stick_y, -1, 1));

        /**
         * Set lift motor power according to gamepad trigger values
         * Right trigger raises lift arm
         * Left trigger lowers lift arm
         */
        if(gamepad1.right_trigger > liftThresh){    //raise lift arm
            lift.setPower(gamepad1.right_trigger);  //encoder units decrease
        }
        else if(gamepad1.left_trigger > liftThresh){    //lower lift arm
            if(lift.getCurrentPosition() < lowerLiftLim)    //prevent lift from going underneath a certain position
                lift.setPower(-gamepad1.left_trigger);  //encoder units increase
            else
                lift.setPower(0);
        }
        else{
            lift.setPower(0);
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        lift.setPower(0);
        left.setPower(0);
        right.setPower(0);
    }

}
