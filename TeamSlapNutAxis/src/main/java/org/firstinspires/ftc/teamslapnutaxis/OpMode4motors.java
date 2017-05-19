package org.firstinspires.ftc.teamslapnutaxis;

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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Tele Op code for SlapNutAxis that currnetly allows for movement and changeing of speed
 */

@TeleOp(name = "teleSlapNut4Motor", group = "Slap")
//@Disabled
public class OpMode4motors extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor R, L,R2,L2;//declares the left and right robot motors
    double x, y, x2;//declares doubles to hold joystick values
    double forwardPower,turnPower,dPadForwardPower,dPadTurnPower;//declares doubles to hold movement power
    double speed = .3;//Speed < 1;
    final double speedModifier = .25;//speedModifier is what speed is multipied by to change it
    boolean modified = false;// allows for switching between upper and lower speeds
    boolean modState = false;// solves debouncing

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Initialize motors & do pre game setup
        R = hardwareMap.dcMotor.get("r");
        L = hardwareMap.dcMotor.get("l");
        R2 = hardwareMap.dcMotor.get("R");
        L2 = hardwareMap.dcMotor.get("L");
        R.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        L.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        R2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        L2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        R.setDirection(DcMotor.Direction.REVERSE);
        R2.setDirection(DcMotor.Direction.REVERSE);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()){
        /*Movement*/
            x = gamepad1.left_stick_x;// gets the 1st gamepad's left thumbstick's x value
            y = -gamepad1.left_stick_y;// gets the 1st gamepad's left thumbstick's y value
            x2 = gamepad1.right_stick_x;// gets the 1st gamepad's right thumbstick's x value

            forwardPower = y*speed;//get the forward move power
            turnPower = x2*speed/2;//get the turn move power
            if(gamepad1.dpad_up){
                dPadForwardPower = .8 * speed;//gets dpad forward power
            }else if(gamepad1.dpad_down) {
                dPadForwardPower = -.8 * speed;//gets dpad backward power
            }else dPadForwardPower = 0;//resets dPad Forward Power
                if(gamepad1.dpad_left){
                    dPadTurnPower = .8 * speed;//gets dpad left turning power
            }else if(gamepad1.dpad_right){
                    dPadTurnPower = -.8*speed;//gets dpad right turning power
            }else dPadTurnPower = 0;//resets dPad Forward Power

            R.setPower(forwardPower + dPadForwardPower/*<- sets the forward power of the motor*/ - turnPower - dPadTurnPower/*<- sets the turning power of the motor*/);
            R2.setPower(R.getPower());
            L.setPower(forwardPower + dPadForwardPower/*<- sets the forward power of the motor*/ + turnPower  + dPadTurnPower/*<- sets the turning power of the motor*/);
            L2.setPower(L.getPower());
            /*Speed*/
            if(gamepad1.left_bumper/* <- button to change speed*/ && modified && !modState){
                speed*=speedModifier;// speed -> speed*speedModifier
                modified = false;
                modState = true;
            }
            if(gamepad1.left_bumper/* <- button to change speed*/ && !modified && !modState){
                speed*=1/speedModifier;// changes speed back to initial
                modified = true;
                modState = true;
            }
            if(!gamepad1.left_bumper && modState){//waits for button to be released
                modState = false;
            }

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}



