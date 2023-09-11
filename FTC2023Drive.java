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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;


@TeleOp(name="FTC2023Drive", group="Iterative Opmode")
//@Disabled
public class FTC2023Drive extends OpMode
{
    // VARIABLES
    //    |    Ad = Adelante    |    At = Atrás    |    Der = Derecha    |    Izq  =  Izquierda    |
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor MotorIzqAd;
    public DcMotor MotorIzqAt;
    public DcMotor MotorDerAd;
    public DcMotor MotorDerAt;
    public DcMotor Grua;
    public Servo Garra;
    public double angle;
    public ColorSensor color;
    public ColorSensor color2;
    
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // MAPING Y AJUSTE
        MotorIzqAd = hardwareMap.get(DcMotor.class, "MotorIzquierdoAdelante");
        MotorIzqAt  = hardwareMap.get(DcMotor.class, "MotorIzquierdoAtras");
        MotorDerAd = hardwareMap.get(DcMotor.class, "MotorDerechoAdelante");
        MotorDerAt= hardwareMap.get(DcMotor.class, "MotorDerechoAtras");
        Grua = hardwareMap.get(DcMotor.class, "MotorGrua");
        Garra = hardwareMap.servo.get("ServoGarra");
        color = hardwareMap.get(ColorSensor.class,"sensorColorRange");
        MotorIzqAd.setDirection(DcMotor.Direction.REVERSE);
        MotorIzqAt.setDirection(DcMotor.Direction.REVERSE);
        MotorDerAd.setDirection(DcMotor.Direction.FORWARD);
        MotorDerAt.setDirection(DcMotor.Direction.FORWARD);
        Grua.setDirection(DcMotor.Direction.FORWARD);
        Garra.setDirection(Servo.Direction.FORWARD);
        color2 = hardwareMap.get(ColorSensor.class,"sensorColor2");
      //  double Ir = color.getIR();
        

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }
public double pgrua=0;
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
        // Setup a variable for each drive wheel to save power level for telemetry
        double max;
        double gruaPoder;
        double garraPoder;
        double nerf = 0.8 ;
       


        // POV MODE
        double JoyIzY   = -gamepad1.left_stick_y;  //Axial
        double JoyIzX =  gamepad1.left_stick_x; //Lateral
        double yaw     =  gamepad1.right_stick_x; //yaw
        //angle = Math.atan(-gamepad1.left_stick_y/gamepad1.left_stick_x);
        double PoderX = JoyIzX;// JoyIzX * Math.cos(angle) + JoyIzY * Math.sin(angle);
        double PoderY = JoyIzY; //JoyIzX * Math.sin(angle) + JoyIzY * Math.cos(angle);


        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = (PoderY + PoderX + yaw)* (nerf);
        double rightFrontPower = (PoderY - PoderX - yaw)*(nerf);
        double leftBackPower   = (PoderY - PoderX + yaw)*(nerf);
        double rightBackPower  = (PoderY + PoderX - yaw)*(nerf);

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
       


        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.addData("RED", color.red());
        telemetry.addData("BLUE", color.blue());
        telemetry.addData("GREEN", color.green());
        
       // telemetry.addData("Ir:", color.getIR());
        
        telemetry.addData("RED2", color2.red());
        telemetry.addData("BLUE2", color2.blue());
        telemetry.addData("GREEN2", color2.green());
        

        //Drive

        MotorIzqAd.setPower(leftFrontPower);
        MotorDerAd.setPower(rightFrontPower);
        MotorIzqAt.setPower(leftBackPower);
        MotorDerAt.setPower(rightBackPower);

        if(gamepad2.dpad_up){ //grua
            Grua.setPower(5.00);

        }
        else if(gamepad2.dpad_down){
            Grua.setPower(-5.00);

        }
        else{
            Grua.setPower(0.00);
        }           //Garra

        if (gamepad2.right_trigger > 0){
            Garra.setPosition(-1);
        }
        else if (gamepad2.left_trigger > 0){
            Garra.setPosition(0.7);
        }
        else{

        }

        if(gamepad2.a && pgrua == 0){ //grua
         runtime.reset();
         while(runtime.seconds()<= 1.8){
            Grua.setPower(5);
         }
         pgrua=1;
        }
        else {
         }
             
         if(gamepad2.b && pgrua == 0){ //grua
         runtime.reset();
         while(runtime.seconds()<= 3.1){
            Grua.setPower(5);
         }
         pgrua=2;
        }
        else {
         }
         
        if(gamepad2.y && pgrua == 0){ //grua
         runtime.reset();
         while(runtime.seconds()<= 4.8){
            Grua.setPower(5);
         }
         pgrua=3;
        }
        else {
         }

        if(gamepad2.x){ //grua
         
         if(pgrua==1){
             runtime.reset();
             pgrua=0;
            while(runtime.seconds()<= 1.5){
            Grua.setPower(-5);
         }
         }
         
        else if(pgrua==2) {
            runtime.reset();
            pgrua=0;
            while(runtime.seconds()<= 2.6){
            Grua.setPower(-5);
         }
         }
         else if(pgrua==3) {
             runtime.reset();
             pgrua=0;
            while(runtime.seconds()<= 4.1){
            Grua.setPower(-5);
         }
         }
         else {
         }
        }
        /*if (JoyIzX < -0.1){  //
            MotorIzqAd.setPower(leftFrontPower);
            MotorDerAd.setPower(rightFrontPower);
            MotorIzqAt.setPower(leftBackPower);
            MotorDerAt.setPower(rightBackPower);

        }

        */



    }



    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }


}
