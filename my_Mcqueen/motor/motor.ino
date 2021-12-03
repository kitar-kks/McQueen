#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <math.h>
#define pi 3.141592

unsigned long currentTime, previousTime;
double kp=0.5,ki=0,kd=0.3;
double elapsedTime,error,lastError,cumError,rateError;
double angle,abs_angle,setPoint=45,rad_pid;
int ref_angle=90;

std_msgs::Float32 angle_msg;
ros::NodeHandle  nh;
ros::Publisher pub_angle("angle", &angle_msg);
//String pwd = String(1.20,3);
Servo fmotor,fservo,bmotor,bservo;

void messageCb( const std_msgs::String& msg){
  String srt=msg.data;
  float slope = srt.toFloat();
  nh.loginfo(msg.data);
  double rad = atan(slope);
  rad_pid = rad*(360/(2*pi));
  angle = computePID(rad_pid);
  if(slope > 0){
    abs_angle = abs(110+angle);
    if(abs_angle > 145){
      abs_angle = 145;
    }
  }
  if(slope<0){
    abs_angle = abs(110-angle);
    if(abs_angle <= 45){
      abs_angle = 45;
    }
  }
  //int steering=map(angle,-20,65,45,135);
  angle_msg.data = abs_angle;
  pub_angle.publish(&angle_msg);
  fservo.write(abs_angle);
  bmotor.write(110);
}
void stops( const std_msgs::String& speeds){
  String srts = speeds.data;
  nh.loginfo(speeds.data);
  if(srts == "stop"){
    bmotor.write(91);
    fservo.write(90); 
  }
}

ros::Subscriber<std_msgs::String> sub("slope", &messageCb );
ros::Subscriber<std_msgs::String> stopspeed("stop_car", &stops );

void setup()
{ 
  nh.initNode();
  nh.advertise(pub_angle);
  nh.subscribe(sub);
  nh.subscribe(stopspeed);
  fmotor.attach(3,1000,2000);
  fservo.attach(5,1000,2000);
  bmotor.attach(6,1000,2000);
  //bservo.attach(5,1000,2000);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}

double computePID(double inp){     
        currentTime = millis();                //get current time
        elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation
        
        error = setPoint - inp;                                // determine error
        cumError += error * elapsedTime;                // compute integral
        rateError = (error - lastError)/elapsedTime;   // compute derivative
 
        double out = kp*error + ki*cumError + kd*rateError;                //PID output               
 
        lastError = error;                                //remember current error
        previousTime = currentTime;                        //remember current time
 
        return out;                                        //have function return the PID output
}
