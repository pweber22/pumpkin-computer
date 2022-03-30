#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GPS.h>
#include "Display.h"
#include "Pumpkin.h"
#include "Target.h"
#include "Wind.h"

#define doLogging true

#define time_zone 19

Adafruit_SSD1306 display(128, 32, &Wire1, 0);
Adafruit_GPS GPS(&Wire1);

unsigned int state;

uint8_t c;

//float pumpkin_cd=1.00; // coefficient of drag of the pumpkin
//float pumpkin_mass=0.830;  // mass of the pumpkin in kg
//float pumpkin_circumference=0.40;  // circumference of the pumpkin in meters

double plane_lat;
double plane_lon;
int alt_ft;
float vel_mph;

float bearingRad;
float windX;
float windY;

long mPerLat;
long mPerLon;

float plane_ground_speed_mph;
float plane_gnd_speed;
int drop_height_ft;
float pumpkin_cross_section;
long err;

float time_step=0.01;

float dropX;
float dropY;
float pathX;
float pathY;
double m;
double b;

#define g -9.81 // acc from gravity
#define pi 3.14159265 // pi

float dropDistance;
int time_to_drop;
float drop_time;

void setup() {
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  for(int i=0;i<8;i++){
    pinMode(i, INPUT_PULLUP); 
  }

  analogWrite(9, 200);
  analogWrite(10, 200); 
  analogWrite(11, 200);
  
  Serial.begin(115200);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3c);
  display.clearDisplay();
  //display.drawBitmap(0, 0, image_data_frame000, 64, 32, 1);
  //display.display();
  pinMode(13, OUTPUT);
  
  GPS.begin(0x10);
  
  display.setTextColor(1);
  display.setCursor(0,0);
  display.write("NO GPS");
  display.display();

  while(!GPS.newNMEAreceived()){
    c=GPS.read();
  }
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);

  display.clearDisplay();
  display.setCursor(0,0);
  display.write(GPS.lastNMEA());
  display.display();
  getSetting();
  delay(1000);

  // configure wind, convert to m/s and get x&y components
  float wind_speed=wind_speed_mph*0.44704;
  windX=sin(wind_dir*pi/180)*wind_speed;
  windY=cos(wind_dir*pi/180)*wind_speed;

  // calculate meters per degree latitude and longitude at target location
  bearingRad = bearing*pi/180;
  float radLat=targetLat*pi/180;
  mPerLat = (111132.92-559.82*cos(2*radLat)+1.175*cos(4*radLat)-0.0023*cos(6*radLat));
  mPerLon = (111412.84*cos(radLat)-93.5*cos(3*radLat)+0.118*cos(5*radLat));


  
#if doLogging
  //Serial.println("hour,minute,second,state,fix,sats,latitude,longitude,altft,hdg,vel_mph,error,timetodrop,tgtbearing,dropDistance");
#endif
  setState_aqi();
}

void loop() {
  while(!GPS.newNMEAreceived()){
    c=GPS.read();
   }
  GPS.parse(GPS.lastNMEA());

  if(state==0){
    char msg[9];
    for(int i=128; i>-65;i-=6){
      for(int j=0; j<6; j+=2){
        display.clearDisplay();
        display.drawBitmap(i-j, 0, image_data_frame000, 64, 32, 2);
        display.setCursor(80,25);
        sprintf(msg, "%02d:%02d:%02d", (GPS.hour+time_zone)%24, GPS.minute, GPS.seconds);
        display.write(msg);
        display.display();
        delay(20);
      }
      while(!GPS.newNMEAreceived())
        c=GPS.read();
      GPS.parse(GPS.lastNMEA());
    }
    for(int i=0; i<10; i++){
      while(!GPS.newNMEAreceived()){
        c=GPS.read();
      }
      GPS.parse(GPS.lastNMEA());
      display.clearDisplay();
      display.setCursor(80,25);
      sprintf(msg, "%02d:%02d:%02d", (GPS.hour+time_zone)%24, GPS.minute, GPS.seconds);
      display.write(msg);
      display.display();
      #if doLogging
      SerialLog();
      #endif
    }
    if(GPS.fix)
      setState_run();
  }

  if(state==1){
    //get plane coordinates in decimal degrees
    plane_lat=(int)(GPS.latitude/100);
    plane_lat+=fmod(GPS.latitude, 100)/60;
    plane_lon=(int)(GPS.longitude/100);
    plane_lon+=fmod(GPS.longitude, 100)/60;
    if(GPS.lon == 'W')
      plane_lon = -plane_lon;

    //get plane coordinates in local xy system
    float planeX=mPerLon*(plane_lon-targetLon);
    float planeY=mPerLat*(plane_lat-targetLat);

    //update ground speed and agl height with new gps data
    plane_ground_speed_mph=(GPS.speed*1.151);
    drop_height_ft=(GPS.altitude*3.28-targetAlt_ft);

    //run simulation and update drop coordinates
    dropSim();

    //calculate flight path (y=mx+b in local coordinates, target is origin)
    m=tan(-(bearing+90.0)*pi/180.0);
    b=dropY-m*dropX;

    //calculate nearest point along planned flight path
    pathX=(abs(((-m)*planeX+planeY)-b))/sqrt(sq(m)+1);
    pathY=m*pathX+b;

    //calculate distance to flight path
    err=sqrt(sq(pathX-planeX)+sq(pathY-planeY));
    if(bearing <180){
      if(planeY>m*planeX+b)
        err*=-1;
    }
    else{
      if(planeY<m*planeX+b)
        err*=-1;
    }
    err*= 3.281;  //convert error to feet

    //calculate time to drop
    dropDistance=sqrt(sq(planeX-dropX)+sq(planeY-dropY));
    if(plane_gnd_speed>1)
      time_to_drop=dropDistance/plane_gnd_speed;
    else time_to_drop=599;

    //send new data to display and serial

    #if doLogging
      SerialLog();
    #endif
    
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(1);
    display.setCursor(0,0);
    display.print(plane_lat,6);
    display.setCursor(0,8);
    display.print(plane_lon,6);
    display.setCursor(0,16);
    display.print(GPS.altitude*3.28);
    display.print(" ft");
    display.setCursor(0,24);
    display.print(GPS.speed*1.151);
    display.print(" mph");

    display.setCursor(80,0);
    display.print("hdg: ");
    display.print((int)GPS.angle);
    display.display();

    if(time_to_drop<=3.0){
      setState_terminal();
    }
    
    if(!GPS.fix)
      setState_aqi();
  } //if(state==1)

  while(state==2){
    int terminal_count=drop_time-millis();
    long gps_timer=millis();
    while(millis()-gps_timer<200){
      terminal_count=drop_time-millis();
      if(terminal_count<=0) 
        setState_drop();
    Serial.println("TIMR,"+String(terminal_count));
    }
    
    while(!GPS.newNMEAreceived()){
      c=GPS.read();
    }
    GPS.parse(GPS.lastNMEA());
    
    //get plane coordinates in decimal degrees
    plane_lat=(int)(GPS.latitude/100);
    plane_lat+=fmod(GPS.latitude, 100)/60;
    plane_lon=(int)(GPS.longitude/100);
    plane_lon+=fmod(GPS.longitude, 100)/60;
    if(GPS.lon == 'W')
      plane_lon = -plane_lon;
    
    //get plane coordinates in local xy system
    float planeX=mPerLon*(plane_lon-targetLon);
    float planeY=mPerLat*(plane_lat-targetLat);
    
    //calculate nearest point along planned flight path
    pathX=(m*(((1/m)*planeX+planeY)-b))/sq(m)+1;
    pathY=m*pathX+b;

    //calculate distance to flight path
    err=(long)sqrt(sq(pathX-planeX)+sq(pathY-planeY));
    if(bearing <180){
      if(planeY>m*planeX+b)
        err*=-1;
    }
    else{
      if(planeY<m*planeX+b)
        err*=-1;
    }
    err*= 3.281;  //convert error to feet
    SerialLog();
  }

  if(state==3){
    SerialLog();
  }
}

void setState_aqi(){
  state=0;
  analogWrite(9, 256);
  analogWrite(10, 256);
  analogWrite(11, 200);
}

void setState_run(){
  state=1;
  analogWrite(9,256);
  analogWrite(10,200);
  analogWrite(11,256);
}

void setState_terminal(){
  state=2;
  float dist_to_drop = sqrt(pow(pathX-dropX,2)+pow(pathY-dropY,2));
  drop_time=millis()+(1000*dist_to_drop/plane_gnd_speed);
}

void setState_drop(){
  state=3;
  analogWrite(9, 200);
  analogWrite(10, 256);
  analogWrite(11, 256);

  //get plane coordinates in decimal degrees
  plane_lat=(int)(GPS.latitude/100);
  plane_lat+=fmod(GPS.latitude, 100)/60;
  plane_lon=(int)(GPS.longitude/100);
  plane_lon+=fmod(GPS.longitude, 100)/60;
  if(GPS.lon == 'W')
    plane_lon = -plane_lon;
  
  //get plane coordinates in local xy system
  float planeX=mPerLon*(plane_lon-targetLon);
  float planeY=mPerLat*(plane_lat-targetLat);

  //update ground speed and agl height with new gps data
  float final_ground_speed_mph=(GPS.speed*1.151);
  int final_drop_height=(GPS.altitude*3.28-targetAlt_ft);
  
  String logBuffer="DROP,";
  logBuffer+=String(GPS.hour)+',';
  logBuffer+=String(GPS.minute)+',';
  logBuffer+=String(GPS.seconds)+',';
  logBuffer+=String(state)+',';
  logBuffer+=String(GPS.fix)+',';
  logBuffer+=String(GPS.satellites)+',';
  logBuffer+=String(plane_lat,6)+',';
  logBuffer+=String(plane_lon,6)+',';
  logBuffer+=String(drop_height_ft)+',';
  logBuffer+=String((int)GPS.angle)+',';
  logBuffer+=String(plane_ground_speed_mph)+',';
  logBuffer+=String(err)+',';
  logBuffer+=String(time_to_drop)+',';
  logBuffer+=String(bearing)+',';
  logBuffer+=String(planeX-dropX)+',';
  logBuffer+=String(planeY-dropY)+',';
  logBuffer+=String(final_ground_speed_mph)+',';
  logBuffer+=String(final_drop_height);
  Serial.println(logBuffer);
}

void dropSim(){
  analogWrite(11,200);
  plane_gnd_speed=plane_ground_speed_mph*0.44704;
  float drop_height=drop_height_ft*0.3048;
  
  //initialize sim variables, orient plane in local coordinate system
  float sim_time=0.0;
  float pumpkin_x=0.0;
  float pumpkin_y=0.0;
  float pumpkin_z=drop_height;
  float pumpkin_x_gnd_speed=sin(bearingRad)*plane_gnd_speed;
  float pumpkin_y_gnd_speed=cos(bearingRad)*plane_gnd_speed;
  float pumpkin_y_air_speed=pumpkin_y_gnd_speed+windY;
  float pumpkin_x_air_speed=pumpkin_x_gnd_speed+windX;
  float pumpkin_vertical_speed=0.0;

  float drag_z;
  float drag_y;
  float drag_x;
  
  while(pumpkin_z>0.0){
    sim_time+=time_step;

    drag_z=pumpkin_cd*pumpkin_cross_section*air_density*pow(pumpkin_vertical_speed,2)/2;
    pumpkin_vertical_speed+=(g*time_step)+(drag_z/pumpkin_mass)*time_step;
    pumpkin_z+=pumpkin_vertical_speed*time_step;

    drag_y=pumpkin_cd*pumpkin_cross_section*air_density*pow(pumpkin_y_air_speed,2)/2;
    pumpkin_y_gnd_speed-=(drag_y/pumpkin_mass)*time_step;
    pumpkin_y+=pumpkin_y_gnd_speed*time_step;
    pumpkin_y_air_speed=pumpkin_y_gnd_speed+windY;

    drag_x=pumpkin_cd*pumpkin_cross_section*air_density*pow(pumpkin_x_air_speed,2)/2;
    pumpkin_x_gnd_speed+=(drag_x/pumpkin_mass)*time_step;
    pumpkin_x+=pumpkin_x_gnd_speed*time_step;
    pumpkin_x_air_speed=pumpkin_x_gnd_speed+windX;
  }
  dropX=-pumpkin_x;
  dropY=-pumpkin_y;
  analogWrite(11,256);
}

void SerialLog(){
  String logBuffer="FDAT,";
  logBuffer+=String(GPS.hour)+',';
  logBuffer+=String(GPS.minute)+',';
  logBuffer+=String(GPS.seconds)+',';
  logBuffer+=String(state)+',';
  logBuffer+=String(GPS.fix)+',';
  logBuffer+=String(GPS.satellites)+',';
  logBuffer+=String(plane_lat,6)+',';
  logBuffer+=String(plane_lon,6)+',';
  logBuffer+=String(drop_height_ft)+',';
  logBuffer+=String((int)GPS.angle)+',';
  logBuffer+=String(plane_ground_speed_mph)+',';
  logBuffer+=String(err)+',';
  logBuffer+=String(time_to_drop)+',';
  logBuffer+=String(bearing)+',';
  logBuffer+=String((int)dropDistance);
  Serial.println(logBuffer);
}

void getSetting(){
  byte upper=0;
  for(int i=0; i<4; i++){
    upper=upper << 1;
    upper+=1 ^ digitalRead(i);
  }
  byte lower=0;
  for(int i=4; i<8; i++){
    lower=lower << 1;
    lower+=1 ^ digitalRead(i);
  }

  wind_speed_mph=upper;

  wind_dir=30*lower;
  
  //display.clearDisplay();
  display.setCursor(0,17);
  display.print("wind speed: ");
  display.print(wind_speed_mph);
  display.setCursor(0,24);
  display.print("wind dir: ");
  display.print(wind_dir);
  display.display();
  delay(1500);
}
