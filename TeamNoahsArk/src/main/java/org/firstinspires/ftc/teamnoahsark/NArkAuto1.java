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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="NArkAuto1.3", group="Autonomous")  // @Autonomous(...) is the other common choice
//@Disabled
public class NArkAuto1 extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime mTimer = new ElapsedTime();
    DcMotor left = null;
    DcMotor right = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

         left  = hardwareMap.dcMotor.get("left");
         right = hardwareMap.dcMotor.get("right");

         left.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
         right.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

//        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Drive Position:", "L: %d   R: %d", left.getCurrentPosition(), right.getCurrentPosition());
            telemetry.update();

            mTimer.reset();

            //fwd(1000, 0.25);
            //bck(1000, 0.40);

            while(runtime.milliseconds() < 4500){
                setMotorPower(0.3);
            }
            while(runtime.milliseconds() >=4500 && runtime.milliseconds() < 6000){
                setMotorPower(-0.35);
            }

            setMotorPower(0);




        }
    }

    private void setMotorPower(double n){
        left.setPower(n);
        right.setPower(n);
    }

    /**
     * lol i forgot we dont have encoders so forget all this stuff here :P
     */
    /*private void fwd(int enc, double pwr){
        int lTarg = left.getCurrentPosition() + enc;
        int rTarg = right.getCurrentPosition() + enc;

        left.setTargetPosition(lTarg);
        right.setTargetPosition(rTarg);

        left.setPower(pwr);
        right.setPower(pwr);
    }

    private void bck(int enc, double pwr){
        int lTarg = left.getCurrentPosition() - enc;
        int rTarg = right.getCurrentPosition() - enc;

        left.setTargetPosition(lTarg);
        right.setTargetPosition(rTarg);

        left.setPower(pwr);
        right.setPower(pwr);
    }

    private void turnL(int enc, double pwr){
        int lTarg = left.getCurrentPosition() - enc;
        int rTarg = right.getCurrentPosition() + enc;

        left.setTargetPosition(lTarg);
        right.setTargetPosition(rTarg);

        left.setPower(pwr);
        right.setPower(pwr);
    }

    private void turnR(int enc, double pwr){
        int lTarg = left.getCurrentPosition() + enc;
        int rTarg = right.getCurrentPosition() - enc;

        left.setTargetPosition(lTarg);
        right.setTargetPosition(rTarg);

        left.setPower(pwr);
        right.setPower(pwr);
    }
*/
}
