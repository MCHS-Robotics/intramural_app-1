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
        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.hardware.ColorSensor;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
        import com.qualcomm.robotcore.hardware.DigitalChannelController;
        import com.qualcomm.robotcore.util.ElapsedTime;

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
    public final double WHEELDIAMETER = 4;
    public final double CIRCLEDIAMETER = 16.75;
    public final int ENCODERCOUNT = 1120;
    public ElapsedTime runtime;
    public float speed = .3f;
    ColorSensor sensorRGB;
    static DeviceInterfaceModule cdim;
    @Override
    /**
     * runs the autonomous code for the robot
     * @throws InterruptedException the robot has been stopped
     */
    public void runOpMode() throws InterruptedException {
        runtime = new ElapsedTime();
        L = hardwareMap.dcMotor.get("l");
        R = hardwareMap.dcMotor.get("r");
        L.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        R.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        R.setDirection(DcMotor.Direction.REVERSE);
        //cdim = hardwareMap.deviceInterfaceModule.get("dim");
        //cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);
        //sensorRGB = hardwareMap.colorSensor.get("sensor_color");
        //cdim.setDigitalChannelState(LED_CHANNEL, false);
        idle();
        telemetry.addData("Status", "Initializing");
        telemetry.update();
        int sleeper = 100;
        while(!gamepad1.a && !opModeIsActive()){
            if(gamepad1.left_stick_y < -.5){
                speed+=.01f;
            }
            else if(gamepad1.left_stick_y > .5){
                speed-=.01f;
            }
            telemetry.addData("Status","Speed:" + speed);
            telemetry.addData("Status","Y val" + gamepad1.left_stick_y);
            telemetry.update();
            sleep(sleeper);
        }
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();//resets the timer so the autonomous runs with the timer at 0
        telemetry.addData("Status", "Running");
        telemetry.update();
        /*Insert Code Here*/
        //leftOffset(.1f);
        moveForwardWithTime(3);
        //leftOffset(0);
        sleep(2000);
        moveBackwardWithTime(3);
        /*Insert Code Here*/
        telemetry.addData("Status", "Complete");
        telemetry.update();
    }

    /**
     * moves the robot forward for <b>inches</b> inches
     * @param  inches  the amount of inches to move the robot forward by
     * */
    public void moveForward(int inches){
        L.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//sets motor L to run with encoder
        R.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//sets motor R to run with encoder
        double target = inches/(Math.PI * WHEELDIAMETER);//sets the target motor encoder value to move to (it will move the robot for inches inches)
        target*= ENCODERCOUNT;//works with line above to get the right motor encoder target
        setPower(.3,.3);//sets the left and right motor to .3 power
        while(-L.getCurrentPosition() < target && opModeIsActive()){}//stops the robot untill left motor has reached the target motor encoder target
        setPower(0,0);//stops both motors
        L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//reset encoder in left motor
        R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//reset encoder in right motor
    }

    /**
     * moves the robot backward for <b>inches</b> inches
     * @param  inches  the amount of inches to move the robot backward by
     * */
    public void moveBackward(int inches){//refer to moveForward() for more information
        L.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        R.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double target = -inches/(Math.PI * WHEELDIAMETER);// -inches due to reversing
        target*= ENCODERCOUNT;
        setPower(-.3,-.3);//sets both motors to -.3
        while(L.getCurrentPosition() > target){}
        setPower(0,0);
        L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * turns the robot clockwise by <b>degrees</b> degrees
     * @param  degrees  the amount of degrees to turn the robot clockwise
     * */
    public void turnClockwise(double degrees){//refer to moveForward() for more information
        L.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        R.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double target = degrees * ENCODERCOUNT * CIRCLEDIAMETER/ (360 * WHEELDIAMETER);//sets the target motor encoder value to turn to (it will turn the robot clockwise for degrees degrees)
        setPower(-.3,.3);//moves right motor forward and left motor backwards
        while(R.getCurrentPosition() < target){}
        setPower(0,0);
        L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * turns the robot counter clockwise by <b>degrees</b> degrees
     * @param  degrees  the amount of degrees to turn the robot counter clockwise
     * */
    public void turnCounterClockwise(double degrees){//refer to turnClockwise() for more information
        L.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        R.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double target = degrees * ENCODERCOUNT * CIRCLEDIAMETER/ (360 * WHEELDIAMETER);
        setPower(.3,-.3);//moves left motor forward and right motor backward
        while(L.getCurrentPosition() < target){}//checks L instead of R due to turning in the opposite direction
        setPower(0,0);
        L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * checks to see if color detected by color sensor is more red than blue
     * @return  boolean that is true if color is more red than blue
     * */
    public boolean isRed(){
        //soundPlayer.play(hardwareMap.appContext,0);
        cdim.setDigitalChannelState(LED_CHANNEL, true);//turns on the led
        try {
            Thread.sleep(100);//sleeps the robot for 100 milliseconds
        }catch(Exception e){

        }
        boolean isR = sensorRGB.red()>sensorRGB.blue();//checks if the red detected is more than the blue detected
        cdim.setDigitalChannelState(LED_CHANNEL, false);//turns off led
        return isR;//returns boolean

    }

    /**
     * moves the robot forward for <b>seconds</b> seconds
     * @param  seconds  the amount of seconds to move the robot for
     * */
    public void moveForwardWithTime(double seconds){
        L.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        R.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setPower(speed,speed);//Sets Left and Right to Positive -> Move Forward
        double run = runtime.seconds();//sets run to current time
        while(opModeIsActive()&&runtime.seconds()-run < seconds) {//Checks elapsed time = seconds -> For seconds you want, motor moves forward for this amount of seconds
            if(L.getCurrentPosition() > R.getCurrentPosition()){
                R.setPower(R.getPower()+.00001);
            }
            else if(R.getCurrentPosition() > L.getCurrentPosition()){
                R.setPower(R.getPower()- .00001);
            }
            telemetry.addData("Encoder", "Motor L" + L.getCurrentPosition());
            telemetry.addData("Encoder", "Motor R" + R.getCurrentPosition());
            telemetry.update();
        }
        setPower(0,0);//stops robot
        L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * moves the robot backward for <b>seconds</b> seconds
     * @param  seconds  the amount of seconds to move the robot for
     * */
    public void moveBackwardWithTime(double seconds){
        L.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        R.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setPower(-speed,-speed);//moves robot backwards
        //Sets Left and Right to Negative -> Move Backward
        double run = runtime.seconds();//sets run to current time
        while(opModeIsActive()&&runtime.seconds()-run < seconds){//Checks elapsed time = seconds -> For seconds you want, motor moves forward for this amount of seconds
            telemetry.addData("Encoder", "Motor L" + L.getCurrentPosition());
            telemetry.addData("Encoder", "Motor R" + R.getCurrentPosition());
            telemetry.update();
        }
        setPower(0,0);//stops robot
        L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * turns the robot left for <b>seconds</b> seconds
     * @param  seconds  the amount of seconds to turn the robot for
     * */
    public void turnLeftWithTime(double seconds){
        setPower(-.25,.25);//moves left motor back & moves right motor forward
        //Sets Left to Negative and Right to Positive -> Turn Left
        double run = runtime.seconds();//sets run to current time
        while(opModeIsActive()&&runtime.seconds()-run < seconds);//Checks elapsed time = seconds -> For seconds you want, motor moves forward for this amount of seconds
        setPower(0,0);//stops robot
    }

    /**
     * turns the robot right for <b>seconds</b> seconds
     * @param  seconds  the amount of seconds to turn the robot for
     * */
    public void turnRightWithTime(double seconds){
        setPower(.25,-.25);//moves left motor forward & moves right motor back
        //Sets Left to Positive and Right to Negative -> Turn Right
        double run = runtime.seconds();//sets run to current time
        while(opModeIsActive()&&runtime.seconds()-run < seconds);//Checks elapsed time = seconds -> For seconds you want, motor moves forward for this amount of seconds
        setPower(0,0);//stops robot
    }

    /**
     * stops the robot for <b>seconds</b> seconds
     * @param  seconds  the amount of seconds for the robot to wait
     * */
    public void wait(double seconds){
        double run = runtime.seconds();//sets run to current time
        while(this.opModeIsActive()&&runtime.seconds()-run < seconds);//Checks elapsed time(current - start)
    }

    public float lOffset = 0;// L's power = power + lOffset
    public float rOffset = 0;// R's power = power + rOffset

    /**
     * changes the offset of the left motor (power of L = L + offset)
     * @param change offset that motor L will have
     */
    public void leftOffset(float change){
       lOffset = change;
    }

    /**
     * changes the offset of the right motor (power of R = R + offset)
     * @param change offset that motor R will have
     */
    public void rightOffset(float change){
        rOffset = change;
    }

    /**
     * sets the left motor to <b>left</b> power + lOffset and the right motor to <b>right</b> power + rOffset
     * @param  left  the power to set the left motor to
     * @param  right  the power to set the right motor to
     * */
    public void setPower(double left,double right){
        L.setPower(left+lOffset);//sets L's power to left
        R.setPower(right+rOffset);//sets R's power to right
    }
}
