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


@Autonomous(name="AutonomoDeLadoIzquierda", group="Robot")
//@Autonomous(name="BlueIzquierda", group="Linear Opmode")

public class AutoDeLadoIzquierda extends LinearOpMode {

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
    double poder = 0.3;
    public double colorD;
    public double driftAdelante = 0;
    public double driftDerecha = 0;
    public double driftAtras = 0;
    public double driftIzquierda = -0.05;

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
           Garra.setPosition(0);
            sleep(1500);
           GruaSubir(0.2);
           sleep(500);
            Izquierda(2.2);
            sleep(500);
            int reD=color.red();
            int bluE=color.blue();
            int greeN=color.green();
            //color2 a color
            sleep(500);
            Izquierda(2.8);
            sleep(100);
            Derecha(0.6);
            sleep(500);
            GruaSubir(3.5);
            sleep(500);
            Adelante(0.1);
            sleep(500);
            GruaBajar(0.5);
            sleep(500);
            Garra.setPosition(0.5);
            sleep(500);
            GruaSubir(0.5); //ajustar
            sleep(200);
            Atras(0.21);
            sleep(500);
            LeerColor(reD,bluE,greeN);
            GruaBajar(3.2); //ajustar
            
    
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
          MotorIzqAd.setPower(poder);
        MotorDerAd.setPower(-poder*(1+driftDerecha));
        MotorIzqAt.setPower(-poder);
        MotorDerAt.setPower(poder);
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
        MotorIzqAt.setPower(poder*(1+driftIzquierda));
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
        MotorIzqAd.setPower(poder*(1+driftAdelante));
        MotorDerAd.setPower(poder);
        MotorIzqAt.setPower(poder);
        MotorDerAt.setPower(poder);
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
        MotorDerAd.setPower(-poder);
        MotorIzqAt.setPower(-poder);
        MotorDerAt.setPower(-poder*(1+driftAtras));
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
            Derecha(3.4);
            Atras(1.5);
            
        }
        else if(azul>rojo && azul>verde && azul>ColorDrift){  // azul 3 der
            colorD = 3;
            Derecha(3.4);
            Adelante(1.5);
        }
        else if(verde>rojo && verde>azul && verde>ColorDrift+(ColorDrift*0.60)){ // verde 2  cent
            colorD = 2;
            Derecha(3.4);
           
        
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
