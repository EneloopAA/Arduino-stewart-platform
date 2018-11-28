#include <Servo.h>
#include "PVector.h"
#include <SoftwareSerial.h>

#define DEB true;
//#define SELF_CONT
#define PRINTD

#define myLed  13
#define pi 3.1416
#define SBUF_SIZE 64

#define rotX 0
#define rotY 0

char sbuf[SBUF_SIZE];
signed int sbuf_cnt=0;

Servo servos[6];
SoftwareSerial myread(8,7);

const int zero_angles[6] = {1560,1520,1590,1520,1550,1560};
const int max_angles[6] = {1850,1800,1880,1800,1820,1830};
const int min_angles[6] = {1280,1230,1300,1250,1260,1300};

const float platformAngles[6]  = {260, 280, 20, 40, 140, 160};
const float baseAngles[6] = {245,295,5,55,125,175 };
const float beta[] = {2*PI/3, PI/3, -2*PI/3, -PI, 0, -PI/3};
float angle[6] = {0,};

const float INITIAL_HEIGHT = 514.352;//  250
const float BASE_RADIUS = 375; //140
const float PLATFORM_RADIUS = 325;
const float HORN_LENGTH = 150;
const float LEG_LENGTH = 555; // 270

float params[3] = {0,0,0};
float prevE[3] = {0,0,0};
float alpha[6];
float xdiff,ydiff;

PVector initialHeight(0,0,INITIAL_HEIGHT);
PVector baseJoint[6],platformJoint[6];
PVector translation,rotation;
PVector q[6],l[6],A[6];
PVector p1(0,0,0);

//float P=15,I=0.3,D=0.5;
float P=1,I=0,D=0;
float xError,yError;
float totalXerror,totalYerror;

void setup() {
    Serial.begin(115200);
    myread.begin(4800);
    pinMode(myLed, OUTPUT);
    digitalWrite(myLed, HIGH);

    for(int i=0;i<6;i++){
        servos[i].writeMicroseconds(zero_angles[i]);
    }

    servos[0].attach(3,min_angles[0],max_angles[0]);
    servos[1].attach(5,min_angles[1],max_angles[1]);
    servos[2].attach(6,min_angles[2],max_angles[2]);
    servos[3].attach(9,min_angles[3],max_angles[3]);
    servos[4].attach(10,min_angles[4],max_angles[4]);
    servos[5].attach(11,min_angles[5],max_angles[5]);

    for (int i=0; i<6; i++) {
        float mx = BASE_RADIUS*cos(radians(baseAngles[i]));
        float my = BASE_RADIUS*sin(radians(baseAngles[i]));
        baseJoint[i] = PVector(mx,my,0);
    }
    
    for (int i=0; i<6; i++) {
        float mx = PLATFORM_RADIUS*cos(radians(platformAngles[i]));
        float my = PLATFORM_RADIUS*sin(radians(platformAngles[i]));
        platformJoint[i] = PVector(mx, my, 0);
        q[i] = PVector(0, 0, 0);
        l[i] = PVector(0, 0, 0);
        A[i] = PVector(0, 0, 0);
    }
    
}

void loop() {

  PIDread();

  if(EBimuAsciiParser(params, 3)){

    xError = rotX - params[1];
    yError = rotY - params[0];
    
    totalYerror += yError;
    totalXerror += xError;
          
    xdiff = xError - prevE[1];
    ydiff = yError - prevE[0];

#ifdef PRINTD
    Serial.print(params[1]);
    Serial.print(" ");
    Serial.println(params[0]);
#endif

    PVector p2( ((P*yError) + (I*totalYerror) + (D*ydiff) )*pi/360 ,((P*xError) + (I*totalXerror) + (D*xdiff) )*pi/360,0);
    applyTranslationAndRotation(p1,p2);

    prevE[0] = xError;
    prevE[1] = yError;
    
  }    
}


void calcQ(){
    for(int i=0;i<6;i++){
        q[i].x = cos(rotation.z)*cos(rotation.y)*platformJoint[i].x +
                 (-sin(rotation.z)*cos(rotation.x)+cos(rotation.z)*sin(rotation.y)*sin(rotation.x))*platformJoint[i].y +
                 (sin(rotation.z)*sin(rotation.x)+cos(rotation.z)*sin(rotation.y)*cos(rotation.x)) *platformJoint[i].z;

        q[i].y = sin(rotation.z)*cos(rotation.y)*platformJoint[i].x +
                 (cos(rotation.z)*cos(rotation.x)+sin(rotation.z)*sin(rotation.y)*sin(rotation.x))*platformJoint[i].y +
                 (-cos(rotation.z)*sin(rotation.x)+sin(rotation.z)*sin(rotation.y)*cos(rotation.x))*platformJoint[i].z;

        q[i].z = -sin(rotation.y)*platformJoint[i].x +
                 cos(rotation.y)*sin(rotation.x)*platformJoint[i].y +
                 cos(rotation.y)*cos(rotation.x)*platformJoint[i].z;

        PVector temp = translation;
        temp.add(initialHeight);
        q[i].add(temp);

        l[i] = q[i];
        l[i].sub(baseJoint[i]);

    }
}

void calcAlpha(){
    for(int i=0;i<6;i++){
        float L = l[i].magSq()-(LEG_LENGTH*LEG_LENGTH)+(HORN_LENGTH*HORN_LENGTH);
        float M = 2*HORN_LENGTH*(q[i].z-baseJoint[i].z);
        float N = 2*HORN_LENGTH*(cos(beta[i])*(q[i].x-baseJoint[i].x) + sin(beta[i])*(q[i].y-baseJoint[i].y));
        alpha[i] = asin(L/sqrt(M*M+N*N)) - atan2(N, M);
    }
}


void applyTranslationAndRotation(PVector t, PVector r) {
  
#ifdef PRINTD
    Serial.println(" this is the rotation : ");
#endif

    boolean err = false;
    rotation.set(r);
    translation.set(t);
    calcQ();
    calcAlpha();
    for(int i=0;i<6;i++){
        if(isnan(alpha[i])){
            err = true;
            
#ifdef PRINTD
            Serial.println("nan");
            Serial.println(alpha[i]);
#endif

        }
    }
    if(!err){
        for(int j=0;j<6;j++){
            digitalWrite(myLed, HIGH);
            float mult = degrees(alpha[j]) / 90;
            //if(j%2 == 0){
            if(j%2 == 1){
                if(alpha[j] >= 0){
                    servos[j].writeMicroseconds(zero_angles[j] + ((max_angles[j]-zero_angles[j]) * mult));
#ifdef PRINTD
          Serial.print(j);
          Serial.print(". angle : ");
          Serial.print(degrees(alpha[j]));
          Serial.print(" ");
          Serial.println(zero_angles[j] + ((max_angles[j]-zero_angles[j]) * mult));
#endif

                }else{
                    servos[j].writeMicroseconds(zero_angles[j] + ((zero_angles[j]-min_angles[j]) * mult));
#ifdef PRINTD
          Serial.print(j);
          Serial.print(". angle : ");
          Serial.print(degrees(alpha[j]));
          Serial.print(" ");
          Serial.println(zero_angles[j] + ((zero_angles[j]-min_angles[j]) * mult));
#endif
                }
            }else{
                if(alpha[j] >= 0){
                    servos[j].writeMicroseconds(zero_angles[j] - ((zero_angles[j]-min_angles[j]) * mult));
#ifdef PRINTD
          Serial.print(j);
          Serial.print(". angle : ");
          Serial.print(degrees(alpha[j]));
          Serial.print(" ");
          Serial.println(zero_angles[j] - ((zero_angles[j]-min_angles[j]) * mult));
#endif

                }else{
                    servos[j].writeMicroseconds(zero_angles[j] - ((max_angles[j]-zero_angles[j]) * mult));
#ifdef PRINTD
          Serial.print(j);
          Serial.print(". angle : ");
          Serial.print(degrees(alpha[j]));
          Serial.print(" ");
          Serial.println(zero_angles[j] - ((max_angles[j]-zero_angles[j]) * mult));
#endif

                }
            }
        }
    }
}

int EBimuAsciiParser(float *item, int number_of_item)
{
  int n,i;
  int rbytes;
  char *addr; 
  int result = 0;
  
  rbytes = Serial.available();
  for(n=0;n<rbytes;n++)
  {
    sbuf[sbuf_cnt] = Serial.read();
    if(sbuf[sbuf_cnt]==0x0a)
       {
           addr = strtok(sbuf,",");
           for(i=0;i<number_of_item;i++)
           {
              item[i] = atof(addr);
              addr = strtok(NULL,",");
           }

           result = 1;

         // Serial.print("\n\r");
         // for(i=0;i<number_of_item;i++)  {  Serial.print(item[i]);  Serial.print(" "); }
       }
     else if(sbuf[sbuf_cnt]=='*')
       {   sbuf_cnt=-1;
       }

     sbuf_cnt++;
     if(sbuf_cnt>=SBUF_SIZE) sbuf_cnt=0;
  }
  
  return result;
}

void PIDread(){
  
  if(myread.available()){ // rx on pin8
    
    P = myread.parseInt();
    myread.read();
    I = (float)(myread.parseInt())/100;
    myread.read();
    D = (float)(myread.parseInt())/100;

    #ifdef PIDINPUT
    Serial.print(P);
    Serial.print(" : ");
    Serial.print(I);
    Serial.print(" : ");
    Serial.print(D);
    Serial.print("read P,I,D data");
    #endif
    
    digitalWrite(myLed, HIGH);
  }
  
}

