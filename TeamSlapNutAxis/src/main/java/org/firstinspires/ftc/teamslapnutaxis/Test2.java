package org.firstinspires.ftc.teamslapnutaxis;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.hardware.GyroSensor;
//import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * OpMode for testing correct connection and direction of drive motors.
 * Spins each motor starting with the FL and going clockwise.
 */

@Autonomous(name = "auto2", group = "templates")
//@Disabled
public class Test2 extends LinearOpMode {

    //private ElapsedTime runtime = new ElapsedTime();

    DcMotor L, R;

    public final double WHEELDIAMETER = 4;
    public final double CIR = 2*WHEELDIAMETER*Math.PI;
    public final int ENCODERCOUNT = 1120;
    //public ElapsedTime runtime;
    public double threshold = 3.05;
    public AnalogInput lightSensor;
    static ColorSensor sensorRGB;
    int numInAvg = 6;
    double[] valueArray;
    int current = 0;
    static final int LED_CHANNEL = 5;
    static DeviceInterfaceModule cdim;
    @Override
    public void runOpMode() throws InterruptedException {
        valueArray = new double[numInAvg];
        for (int i = 0; i < numInAvg; i++) {
            valueArray[i] = 4;
        }
        lightSensor = hardwareMap.analogInput.get("lS");
        L = hardwareMap.dcMotor.get("l");
        R = hardwareMap.dcMotor.get("r");
        L.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        R.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        R.setDirection(DcMotor.Direction.REVERSE);
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
        moveTillWhite(.123451);



        /*Insert Code Here*/
        telemetry.addData("Status", "Complete");
        telemetry.update();
    }

    public void redBlue(){
        if(isRed()){
            L.setPower(.1);
            R.setPower(-.1);
            sleep(2000);
            L.setPower(0);
            R.setPower(0);
        }else{
            L.setPower(-.1);
            R.setPower(.1);
            sleep(2000);
            L.setPower(0);
            R.setPower(0);
        }
    }

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

    public void moveTillWhite(double power) {
        L.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        R.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //////////
        L.setPower(power);
        R.setPower(power);
        //////////
        while (!isWhite()) {
            telemetry.addData("Status", lightSensor.getVoltage());
            telemetry.update();
        }
        //////////
        L.setPower(0);
        R.setPower(0);
        //////////
        L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(100);
        if (!isWhite()) {
            moveTillWhite(-(power / Math.abs(power)) * .05);
        }
    }

    public void addToArray(double val) {
        valueArray[current] = val;
        current++;
        if (current == numInAvg) {
            current = 0;
        }
    }

    public double average() {
        double total = 0;
        for (int i = 0; i < numInAvg; i++) {
            total += valueArray[i];
        }
        return total / (numInAvg);
    }

    public boolean isWhite() {
        addToArray(lightSensor.getVoltage());
        return average() < threshold;
    }

    public void forward(int inches,double power){
        L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        R.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        L.setTargetPosition((int)Math.round(ENCODERCOUNT/CIR*inches));
        L.setPower(power);
        R.setPower(power);
        while(L.isBusy()){}
        L.setPower(0);
        R.setPower(0);
        L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void rightTurn(int inches, double power) {
        L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        R.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        L.setTargetPosition((int) Math.round(ENCODERCOUNT / CIR * inches));
        L.setPower(power);
        R.setPower(-power);
        while (L.isBusy()) {
        }
        L.setPower(0);
        R.setPower(0);
        L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void circle(int inches, double Lp,double Rp){
        L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        R.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        L.setTargetPosition((int) Math.round(ENCODERCOUNT / CIR * inches));
        L.setPower(Lp);
        R.setPower(Rp);
        while (L.isBusy()) {
        }
        L.setPower(0);
        R.setPower(0);
        L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}