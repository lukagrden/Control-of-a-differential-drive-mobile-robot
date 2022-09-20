#include <Arduino.h>
#include <elapsedMillis.h>
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
 

ros::NodeHandle nh;               // objekt za teensy čvor


// ********************** klasa regulatora************************

class RegulatorPI {
  private:
    float Pcw, Icw, Dcw;          // parametri regulatora
    float ewprev, ew;             // pogreska procesa i integracija pogreske procesa
    int PWMmax;                   // zasicenje regulatora
    int PWMmin;                   // mrtva zona motora
    float yP = 0;
    float yI = 0;
    float yD = 0;
    float yC = 0;
 
  public:
    RegulatorPI()
      : Pcw(0.0), Icw(0.0), Dcw(0.0), ewprev(0.0), PWMmax(255), PWMmin(55)  {}    // KONSTRUKTOR, početna inicializacija parametara

    void Parametri(float PcwIn, float IcwIn, float DcwIn) {                       // inicializacija parametara regulatora
      Pcw = PcwIn; Icw = IcwIn; Dcw = DcwIn;
    }

    // wC - upravljacki signal
    // wM - mjerena brzina
    // wR - referenca brzine
    // T  - period
    // PWM - PWM
    // direction - smijer

    void wC(float wM, float wR, float T, int &PWM, int &direction) {   // upravljacki signal regulatora (& - referenca memorijske lokacije)

      ew = wR - wM;
      //ewinteg = ewinteg + ew*T;          // integriranje greske, implicito
      yP = Pcw*ew;                         // proporcionalno djelovanje
      yI = yI+Icw*ew*T;                    // integralno djelovanje
      yD = Dcw*(ew-ewprev)/T;              // derivacijsko djelovanje
      yC = yP + yI + yD;                   // izlazni signal regulatora
      ewprev = ew;              
      
      PWM = (int) fabs(yC);                // upravljacki PWM signal zaokruzen na abs int
      if( PWM > PWMmax ) {                 // max PWM signal je 255
        PWM = PWMmax;
        yI = yC - yP - yD;                 // resetiranje integratora pri zasicenju
      }
      // else if( PWM>2 && PWM<PWMmin ) {    
      //   PWM = PWMmin;                   // kada padne ispod PWMmin onda pada u mrtvu zonu
      //   yI = yC - yP;                   // resetiranje integratora za mrtvu zonu
      //}
      
      direction = 1;                        // postavljeni smjer vrtnje motora
      if( yC < 0 ) {                        // ovisno vrijednosti regulatora
        direction = -1;                     // smjer se mjenja
      }
      else if( PWM<2 ) {                    // gašenje motora
        direction = 0;                
      }
  
//       Serial.print("e : ");
//       Serial.println(ew);
//       Serial.print("einteg : ");
//       Serial.println(ewinteg);
//       Serial.print("yC : ");
//       Serial.println(yC);
//       Serial.print("wR : ");
//       Serial.println(wR)

     }
};

// **************** varijable i pinovi *******************
#define nMotora  2                // broj motora
#define M1  0                     // motor 1
#define M2  1                     // motor 2
RegulatorPI regulatorW[nMotora];  // dvije (nMotora) instance regulatora

const int EncoderA[] = {6,9};     // encoderA motora 1 i 2
const int EncoderB[] = {7,8};     // encoderB    -||-
const int PWMpin[] = {0,3};       // PWMovi za motor 1 i 2
const int INA[] = {2,5};          // smjer + za motor 1 i 2
const int INB[] = {1,4};          // smjer -     -||-

volatile int ticks[] = {0,0};     // tickovi pojedinih enkodera u ReadEncoder()
long position[] = {0,0};          // suma tickova novo
long oldposition[] = {0,0};       // suma tickova staro
long dposition[] = {0,0};         // razlika tickova

double theta = 0;
double x = 0;
double y = 0;

unsigned long oldtime = 0;
elapsedMillis elapsedT;           // vrijeme u programu
unsigned int sampT=10;            // vrijeme uzorkovanja [mikrosekunde]

float wM[] = {0,0};               // mjerena brzina kotača    
float wMprev[] = {0,0};           // mjerena brtina kotača n-1 
float wMfilt[] = {0,0};           // filtritana brzina kotača

float wR[]={0,0};                 // referentna brzina kotača
float vrobota[]={0,0};            // linearna i kutna brzina robota
float vrobotaM[]={0,0,0};         // lineatna x,y i kutna mjerena brzina robota

unsigned int noCommMax = 40;      // maksimalan broj petlji u kojima komunikacija nije ostvarena 
unsigned int noComm = 0;          // broj petlji u kojima komunikacija nije ostvarena

float r=0.165/2;                  // radius kotača
float L=0.380;                    // međuosovinski razmak

float rad = 0.0;                  // varijabla za sinusnu pobudu
unsigned long Tref;               // varijabla za početak pobude

// ********************* funkcije ***********************

void SetMotor(int INA, int INB, int PWMpin, int PWM, int direction)  {
// funkcija postavlja brzinu i smjer okretanja motora preko PWMa  
  analogWrite(PWMpin, PWM);            
  if(direction == 1) {                
    digitalWrite(INA, HIGH);       
    digitalWrite(INB, LOW);
  }
  else if(direction == -1) {
    digitalWrite(INA, LOW);
    digitalWrite(INB, HIGH);
  }
  else {
    digitalWrite(INA, HIGH);
    digitalWrite(INB, HIGH);
  }
}
 
template <int i>
void ReadEncoderA() {                        
  if(digitalRead(EncoderA[i])==HIGH) {  
    if(digitalRead(EncoderB[i])==HIGH) {
      ticks[i]++; }
    else {
      ticks[i]--; }                             
  }
  else {
    if(digitalRead(EncoderB[i])==HIGH) {
      ticks[i]--; }
    else {
      ticks[i]++; }
  }
}

template <int i>
void ReadEncoderB() {                        
  if(digitalRead(EncoderB[i])==HIGH) {  
    if(digitalRead(EncoderA[i])==HIGH) {
      ticks[i]--; }
    else {
      ticks[i]++; }                             
  }
  else {
    if(digitalRead(EncoderA[i])==HIGH) {
      ticks[i]++; }
    else {
      ticks[i]--; }
  }
}

void invKinematics(float Vtrans, float Wrot) {
// inverzna kinematika diferencialnog robota
  if (Vtrans>1) Vtrans=1;  // ogranicenje linearne brzine 
  if (Wrot>2) Wrot=2;      // ogranicenje kutne brzine
 
  wR[0]=-1/r*(Vtrans-L*Wrot)*60/(2*PI);
  wR[1]=1/r*(Vtrans+L*Wrot)*60/(2*PI);
}

void dirKinematics(float wM0, float wM1, double theta) {
// direktna kinematika diferencialnog robota
  vrobotaM[0] = r*PI*(wM1-wM0)*cos(theta)/(60);
  vrobotaM[1] = r*PI*(wM1-wM0)*sin(theta)/(60);
  vrobotaM[2] = r*PI*(wM0+wM1)/(60*L);
}

void cmd_vel_Cb( const geometry_msgs::Twist& twist){   
// ROS subscriber na topic cmd_vel  
 noComm = 0;                    // resetiranje variable
 vrobota[0]= twist.linear.x;
 vrobota[1]= twist.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &cmd_vel_Cb );
tf::TransformBroadcaster broadcaster;
nav_msgs::Odometry odom_msg;
ros::Publisher odom_pub("odom", &odom_msg);
geometry_msgs::TransformStamped t; 

char base_link[] = "/base_link";
char odom[] = "/odom";

void setup() {
  
  nh.getHardware()->setBaud(115200);
  nh.initNode();                                // inicijalizacija komunikacije
  nh.subscribe(sub);
  nh.advertise(odom_pub);
  broadcaster.init(nh); 
  
  for (int i = 0; i < nMotora; i++ ) {          // deklaritanje pinova za nMotora
    pinMode(EncoderA[i], INPUT);        
    pinMode(EncoderB[i], INPUT);        
    pinMode(PWMpin[i], OUTPUT);
    pinMode(INA[i], OUTPUT);
    pinMode(INB[i], OUTPUT);
    analogWriteFrequency(PWMpin[i],40000);      // postavljanje frekvencije PWM signala
    regulatorW[i].Parametri(1.5,6,0);           // postavljanje parametara regulatora motora
  }
  attachInterrupt(digitalPinToInterrupt(EncoderA[M1]), ReadEncoderA<M1>, CHANGE);   // interupt za ReadEncoderA, kanal A M1
  attachInterrupt(digitalPinToInterrupt(EncoderB[M1]), ReadEncoderB<M1>, CHANGE);   // interupt za ReadEncoderB, kanal B M1
  attachInterrupt(digitalPinToInterrupt(EncoderA[M2]), ReadEncoderA<M2>, CHANGE);   // interupt za ReadEncoderA, kanal A M2
  attachInterrupt(digitalPinToInterrupt(EncoderB[M2]), ReadEncoderB<M2>, CHANGE);   // interupt za ReadEncoderB, kanal B M2
  }
    
void loop() {
  nh.spinOnce();                                 // pokretanje ROS čvorova za komunikaciju
  invKinematics(vrobota[0],vrobota[1]);          // postavljanje referenci kotača 
   
  if (elapsedT >= sampT) {                       // period uzorkovanja
    noInterrupts();                              // gasenje interupta za sigurno očitanje pozicije
    for(int j = 0; j < nMotora; j++){
      position[j] = ticks[j];                    // ocitanje pozicija obaju motora u vidu tickova
    }
    interrupts();
    float T = elapsedT/(1.0e3);                  // T [s]
    
    for(int i = 0; i < nMotora; i++){  
      dposition[i] = position[i]-oldposition[i];
      wM[i]=(dposition[i]*60)/(T*46.8512*12*4);                  // izracun brzine motora [RPM]
      oldposition[i]=position[i];
      wMprev[i]=wM[i];
      wMfilt[i]=0.7265*wMfilt[i]+0.1367*wM[i]+0.1367*wMprev[i];  // butterworth filter cutoff na 5Hz
    }

    for (int k = 0; k < nMotora; k++){                           // izlaz regulatora
      int PWM, direction;
      if ( noComm>=noCommMax ) {
        wR[k]=0;
        regulatorW[k].wC(wMfilt[k], wR[k], T, PWM, direction);
        SetMotor(INA[k], INB[k], PWMpin[k], PWM, direction);
      }
      regulatorW[k].wC(wMfilt[k], wR[k], T, PWM, direction);
      SetMotor(INA[k], INB[k], PWMpin[k], PWM, direction);
    }

    // direknta kinematika, integracija položaja
    theta += (2*r*PI*(dposition[0]+dposition[1]))/(46.8512*12*4*L);
    if (theta > PI) theta -= TWO_PI;
    if (theta < (-PI)) theta += TWO_PI;
    x += ((dposition[1]-dposition[0])*cos(theta))/2;
    y += ((dposition[1]-dposition[0])*sin(theta))/2;

    dirKinematics(wMfilt[0],wMfilt[1],theta);                  // direktna kinematika mobilnog robota

    t.header.frame_id = odom;
    t.child_frame_id = base_link;
    
    t.transform.translation.x = x/1000;   
    t.transform.translation.y = y/1000;
    t.transform.rotation.z = theta;
    t.transform.rotation = tf::createQuaternionFromYaw(theta);
    t.header.stamp = nh.now();
             
    broadcaster.sendTransform(t);

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = nh.now();
    odom_msg.header.frame_id = odom;
    
    odom_msg.pose.pose.position.x = x/1000;
    odom_msg.pose.pose.position.y = y/1000;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = tf::createQuaternionFromYaw(theta);

    odom_msg.child_frame_id = base_link;
    odom_msg.twist.twist.linear.x = vrobotaM[0] ;       // forward linear velovity
    odom_msg.twist.twist.linear.y = vrobotaM[1];        // forward linear velovity                               
    odom_msg.twist.twist.angular.z = vrobotaM[2] ;      // anglular velocity
    odom_pub.publish(&odom_msg);
      
    noComm++;
    elapsedT=0;
  }
              
}









  
