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
package org.firstinspires.ftc.teamfreedomfriesstrikesback;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="FreedomFriesTemp", group="TeleOp")  // @Autonomous(...) is the other common choice
public class FreedomFriesTemp extends LinearOpMode{
    DcMotor left, right;// lift; /*sweep*/;
   // Servo tilt;
    //double mPower;

    @Override
    public void runOpMode(){
        telemetry.addData("Status", "Initialized");

        left = hardwareMap.dcMotor.get("fL");
        right = hardwareMap.dcMotor.get("fR");
        //sweep = hardwareMap.dcMotor.get("sweep");
        //lift = hardwareMap.dcMotor.get("lift");
        //tilt = hardwareMap.servo.get("tilt");

        right.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("fL", left.getPower());
        telemetry.addData("fR", right.getPower());
        telemetry.addData("Status", "Initialized");

        waitForStart();

        while(opModeIsActive()){
            forward(gamepad1.left_stick_y);
            turn(gamepad1.right_stick_x);

            /*if(gamepad1.a)
                sweep.setPower(0.20);
            */
            /*if(gamepad2.a){
                lift(-0.20);
            }
            if(gamepad2.b){
                lift(0.20);
            }*/
        }
    }

    public void forward(double power){
        left.setPower(power);
        right.setPower(power);
    }

    public void turn(double power){
        left.setPower(-power);
        right.setPower(power);
    }

    /*public void lift(double power){
        lift.setPower(power);
    }*/
}
