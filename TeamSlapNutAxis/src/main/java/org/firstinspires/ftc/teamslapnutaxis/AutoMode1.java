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

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.ColorSensor;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
        import com.qualcomm.robotcore.hardware.DigitalChannelController;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.hardware.GyroSensor;
//import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * OpMode for testing correct connection and direction of drive motors.
 * Spins each motor starting with the FL and going clockwise.
 */

@Autonomous(name = "autoSlapNut", group = "Slap")
//@Disabled

public class AutoMode1 extends LinearOpMode {

    //private ElapsedTime runtime = new ElapsedTime();


    DcMotor L, R;
    static final int LED_CHANNEL = 5;
    public final double WHEELDIAMETER = 0;
    public final double CIRCLEDIAMETER = 0;
    public final int ENCODERCOUNT = 0;

    ColorSensor sensorRGB;
    static DeviceInterfaceModule cdim;
    @Override
    public void runOpMode() throws InterruptedException {


        L = hardwareMap.dcMotor.get("fl");
        R = hardwareMap.dcMotor.get("fr");
        L.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        R.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        L.setDirection(DcMotor.Direction.REVERSE);
        cdim = hardwareMap.deviceInterfaceModule.get("dim");
        cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);
        sensorRGB = hardwareMap.colorSensor.get("sensor_color");
        cdim.setDigitalChannelState(LED_CHANNEL, false);
        idle();
        telemetry.addData("Status", "Initializing");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        telemetry.addData("Status", "Running");
        telemetry.update();
        /*Insert Code Here*/

        /*Insert Code Here*/
        telemetry.addData("Status", "Complete");
        telemetry.update();
    }

    public void moveForward(int inches){
        L.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        R.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double target = inches/(Math.PI * WHEELDIAMETER);
        target*= ENCODERCOUNT;
        L.setPower(.3);
        R.setPower(.3);
        while(L.getCurrentPosition() < target){}
        L.setPower(0);
        R.setPower(0);
        L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void moveBackward(int inches){
        L.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        R.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double target = -inches/(Math.PI * WHEELDIAMETER);
        target*= ENCODERCOUNT;
        L.setPower(-.3);
        R.setPower(-.3);
        while(L.getCurrentPosition() > target){}
        L.setPower(0);
        R.setPower(0);
        L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void turnLeft(double degrees){
        L.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        R.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double target = degrees * ENCODERCOUNT * CIRCLEDIAMETER/ (360 * WHEELDIAMETER);
        L.setPower(-.3);
        R.setPower(.3);
        while(R.getCurrentPosition() < target){}
        L.setPower(0);
        R.setPower(0);
        L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void turnRight(double degrees){
        L.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        R.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double target = degrees * ENCODERCOUNT * CIRCLEDIAMETER/ (360 * WHEELDIAMETER);
        L.setPower(.3);
        R.setPower(-.3);
        while(L.getCurrentPosition() < target){}
        L.setPower(0);
        R.setPower(0);
        L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public boolean isRed(){
        //soundPlayer.play(hardwareMap.appContext,0);
        cdim.setDigitalChannelState(LED_CHANNEL, true);
        try {
            Thread.sleep(100);
        }catch(Exception e){

        }
        boolean isR = sensorRGB.red()>sensorRGB.blue();
        cdim.setDigitalChannelState(LED_CHANNEL, false);
        return isR;
    }

}
