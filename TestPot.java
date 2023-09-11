package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;


@Autonomous(name= "TestPot", group="Robot")
//@Autonomous(name="BlueIzquierda", group="Linear Opmode")

public class TestPot extends LinearOpMode {

    // Declare OpMode members.
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
    int ColorDrift = 65;
    double TiempoFijo = 0.85;
    double poder = 0.3; //0.3
    public double colorD;
    public double diferensia = 0.03;
    public double driftDerAd = 1.012 + diferensia; 
    public double driftDerAt = 1 + diferensia;
    public double driftIzqAd = 1; 
    public double driftIzqAt = 1;
    
    
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        MotorIzqAd = hardwareMap.get(DcMotor.class, "MotorIzquierdoAdelante");
        MotorIzqAt  = hardwareMap.get(DcMotor.class, "MotorIzquierdoAtras");
        MotorDerAd = hardwareMap.get(DcMotor.class, "MotorDerechoAdelante");
        MotorDerAt= hardwareMap.get(DcMotor.class, "MotorDerechoAtras");
        Grua = hardwareMap.get(DcMotor.class, "MotorGrua");
        Garra = hardwareMap.servo.get("ServoGarra");
        color = hardwareMap.get(ColorSensor.class,"sensorColorRange");
        color2 = hardwareMap.get(ColorSensor.class,"sensorColor2");
//
        MotorIzqAd.setDirection(DcMotor.Direction.REVERSE);
        MotorIzqAt.setDirection(DcMotor.Direction.REVERSE);
        MotorDerAd.setDirection(DcMotor.Direction.FORWARD);
        MotorDerAt.setDirection(DcMotor.Direction.FORWARD);
        Grua.setDirection(DcMotor.Direction.FORWARD);
        Garra.setDirection(Servo.Direction.FORWARD);
        
        waitForStart();
        runtime.reset();
        if (opModeIsActive()){
       //Aqui empieza el programa
       
            //prueba de distancia
           //  Adelante(10); //la llanta izquierda adelate va m'as lento
             
             //Izquierda(200);
             //MotorDerAd.setPower(0.05);
             
            //Izquierda(0.3);
            Derecha(10);
            
            sleep(20000);
       
           // int reD=color2.red();
        //    int bluE=color2.blue();
    //        int greeN=color2.green();
          
    
     }
    }
    
    public void GruaBajar (double time){
      
        runtime.reset();
        while(runtime.seconds() <= time){
            Grua.setPower(-1);
        }
        Grua.setPower(0);
        
    }
    public void GruaSubir (double time){
     
        runtime.reset();
        while(runtime.seconds()<= time){
            Grua.setPower(1);
        
        }
       Grua.setPower(0);
    }
    public void GarraAbrir (double time){
        runtime.reset();
        while(runtime.seconds() <= time){
            Garra.setPosition(0.7);
        }
        Garra.setPosition(0);
    }
    public void GarraCerrar (double time){
        runtime.reset();
        while(runtime.seconds() <= time){
            Garra.setPosition(-1);
        }
        Garra.setPosition(0);
    }
    
    public void Derecha (double time){
    
        runtime.reset();
        while(runtime.seconds() <= time){
          MotorIzqAd.setPower(poder*driftIzqAd);
        MotorDerAd.setPower(-poder*driftDerAd);
        MotorIzqAt.setPower(-poder*driftIzqAt);
        MotorDerAt.setPower(poder* driftDerAd);
        }
        MotorIzqAd.setPower(0);
        MotorDerAd.setPower(0);
        MotorIzqAt.setPower(0);
        MotorDerAt.setPower(0);
        
    }
    public void Izquierda (double time){
   
        runtime.reset();
        while(runtime.seconds() <= time){
            MotorIzqAd.setPower(-poder);
        MotorDerAd.setPower(poder);
        MotorIzqAt.setPower(poder);
        MotorDerAt.setPower(-poder);
        }
       MotorIzqAd.setPower(0);
        MotorDerAd.setPower(0);
        MotorIzqAt.setPower(0);
        MotorDerAt.setPower(0);
        
    }
    public void Adelante(double time){
      
        runtime.reset();
        while(runtime.seconds() <= time){
        MotorIzqAd.setPower(poder*driftIzqAd);
        MotorDerAd.setPower(poder*driftDerAd);
        MotorIzqAt.setPower(poder*driftIzqAt);
        MotorDerAt.setPower(poder*driftDerAt);
            //Cambiar a mover robot hacia adelante
            
        }
        MotorIzqAd.setPower(0);
        MotorDerAd.setPower(0);
        MotorIzqAt.setPower(0);
        MotorDerAt.setPower(0);
        
    }
    public void Atras (double time){
      
        runtime.reset();
        while(runtime.seconds() <= time){
        MotorIzqAd.setPower(-poder);
        MotorDerAd.setPower(-poder*1.1);
        MotorIzqAt.setPower(-poder);
        MotorDerAt.setPower(-poder*1.1);
            //Cambiar a mover robot hacia atras
        }
        MotorIzqAd.setPower(0);
        MotorDerAd.setPower(0);
        MotorIzqAt.setPower(0);
        MotorDerAt.setPower(0);
        
    }
    
    public void VueltaDer(double time){
      
        runtime.reset();
        while(runtime.seconds() <= time){
             MotorIzqAd.setPower(1);
        MotorDerAd.setPower(-1.3);
        MotorIzqAt.setPower(1);
        MotorDerAt.setPower(-1.3);
            //Cambiar a girar robot a la derecha
            
        }
        MotorIzqAd.setPower(0);
        MotorDerAd.setPower(0);
        MotorIzqAt.setPower(0);
        MotorDerAt.setPower(0);
        
    }
    
    public void VueltaIzq(double time){
   
        runtime.reset();
        while(runtime.seconds() <= time){
        MotorIzqAd.setPower(-1);
        MotorDerAd.setPower(1.3);
        MotorIzqAt.setPower(-1);
        MotorDerAt.setPower(1.3);
        
            
        }
         MotorIzqAd.setPower(0);
        MotorDerAd.setPower(0);
        MotorIzqAt.setPower(0);
        MotorDerAt.setPower(0);
        
    
    }
    
    public void LeerColor(int rojo,int azul,int verde){
    telemetry.addData("RED", color.red());
    telemetry.addData("BLUE", color.blue());
    telemetry.addData("GREEN", color.green());
    telemetry.update();
    sleep(10);
        if(rojo>azul && rojo>verde && rojo>ColorDrift-(ColorDrift*0.80)){ // rojo 1 iz
            colorD = 1;
            Izquierda(3.4);
            Adelante(1.5);
            
        }
        else if(azul>rojo && azul>verde && azul>ColorDrift){  // azul 3 der
            colorD = 3;
            Izquierda(3.4);
            Atras(1.5);
        }
        else if(verde>rojo && verde>azul && verde>ColorDrift+(ColorDrift*0.60)){ // verde 2  cent
            colorD = 2;
            Izquierda(3.4);
           
        
        }
        else{
        colorD = 0;
        MotorIzqAd.setPower(0);
        MotorDerAd.setPower(0);
        MotorIzqAt.setPower(0);
        MotorDerAt.setPower(0);
        }
    telemetry.addData("color es:",colorD);
    telemetry.update();
    Esperar(1);
    sleep(5000);
    }
    
    public void Esperar (double time){
        runtime.reset();
        while(runtime.seconds() <= time){
        
        }
        
    }
}
