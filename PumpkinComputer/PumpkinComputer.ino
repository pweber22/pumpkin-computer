#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_GPS.h>
#define BUZZER 3
#define leds 0x20
#define green_led 0b11111110
#define red_led 0b11111101
#define yellow_led 0b11111011
#define time_zone 18

const float pumpkin_cd=1.00; // coefficient of drag of the pumpkin
const float pumpkin_mass=0.830;  // mass of the pumpkin in kg
const float pumpkin_circumference=0.40;  // circumference of the pumpkin in meters

const int wind_speed_mph=0;  // wind speed in mph
const int wind_dir=0;  // wind direction degrees
const float air_density=1.225; // air density in kg/m^3

#define targetLat 41.927015 // target latitude in decimal degrees
#define targetLon -91.425713  // target longitude in decimal degrees
const int targetAlt_ft=885;  // target altitude in feet MSL
const int bearing=343; // bearing of the flight path over the target

int state;

float bearingRad;
float windX;
float windY;

float mPerLat;
float mPerLon;

float plane_ground_speed_mph;
float plane_gnd_speed;
int drop_height_ft;
float pumpkin_cross_section;

float time_step=0.01;  //sim time step

float dropX;
float dropY;

#define g -9.81 // acc from gravity
#define pi 3.14159265 // pi

int time_to_drop;
bool buzzer_on;

LiquidCrystal_I2C lcd(0x27,16,2); //create lcd with address, columns, and rows
Adafruit_GPS gps(&Wire);

byte plane_char[8]={
  0b00000,
  0b00000,
  0b00100,
  0b01010,
  0b10001,
  0b00000,
  0b00000,
  0b00000};

byte blank[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

byte on_target[8]={
  0b11111,
  0b10001,
  0b10001,
  0b10001,
  0b10001,
  0b10001,
  0b10001,
  0b11111};

byte left_track[8]={
  0b00010,
  0b00110,
  0b01110,
  0b11110,
  0b01110,
  0b00110,
  0b00010,
  0b00000};

byte right_track[8]={
  0b01000,
  0b01100,
  0b01110,
  0b01111,
  0b01110,
  0b01100,
  0b01000,
  0b00000};

void setup() {
  setLeds(red_led & yellow_led & green_led);
  tone(BUZZER,1500);
  
  for(int i=0;i<8;i++){
	pinMode(i,INPUT_PULLUP);
  }
  
  // configure wind, convert to m/s and get x&y components
  float wind_speed=wind_speed_mph*0.44704;
  windX=sin(wind_dir*pi/180)*wind_speed;
  windY=cos(wind_dir*pi/180)*wind_speed;

  pumpkin_cross_section = pi*sq(pumpkin_circumference/(2*pi));

  // calculate meters per degree latitude and longitude at target location
  bearingRad = bearing*pi/180;
  float radLat=targetLat*pi/180;
  mPerLat = 111132.92-559.82*cos(2*radLat)+1.175*cos(4*radLat)-0.0023*cos(6*radLat);
  mPerLon = 111412.84*cos(radLat)-93.5*cos(3*radLat)+0.118*cos(5*radLat);

  // start gps communication
  gps.begin(0x10);
  gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  gps.sendCommand(PMTK_SET_NMEA_UPDATE_2HZ);

  // initialize lcd and set custom characters
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, blank);
  lcd.createChar(1, plane_char);
  lcd.createChar(2, on_target);
  lcd.createChar(3, left_track);
  lcd.createChar(4, right_track);
  
  delay(200);
  noTone(BUZZER);
  buzzer_on=false;
  setState_aqi();
}

void loop() {
  for(int i=0; i<2; i++){
    char c;
    while(!gps.newNMEAreceived())
      c=gps.read();
    gps.parse(gps.lastNMEA());
  }

  if(state==0){
    if(gps.fix)
      setState_run();
    return;
  }

  if(state==1){

    //get plane coordinates in decimal degrees
    char buf[11];
    float plane_lat=(int)gps.latitude/100;
    plane_lat+=fmod(gps.latitude, 100)/60;
    float plane_lon=(int)gps.longitude/100;
    plane_lon+=fmod(gps.longitude, 100)/60;

    if(gps.lon == 'W')
      plane_lon = -plane_lon;

    //get plane coordinates in local xy system
    float planeX=mPerLon*(plane_lon-targetLon);
    float planeY=mPerLat*(plane_lat-targetLat);
    
    //update ground speed and agl height with new gps data
    plane_ground_speed_mph=(gps.speed*1.151);
    drop_height_ft=(gps.altitude*3.28-targetAlt_ft);
    
    //run simulation and update drop coordinates
    dropSim();

    //calculate flight path (y=mx+b in local coordinates, target is origin)
    float m=tan(-(bearing+90)*pi/180);
    float b=dropY-m*dropX;
    
    //calculate point along planned flight path
    float pathX=(m*(((1/m)*planeX+planeY)-b))/sq(m)+1;
    float pathY=m*pathX+b;

    //calculate distance to flight path
    int err=(int)sqrt(sq(pathX-planeX)+sq(pathY-planeY));
    if(bearing <180){
      if(planeY>m*planeX+b)
        err*=-1;
    }
    else{
      if(planeY<m*planeX+b)
        err*=-1;
    }

    //calculate time to drop
    float dropDistance=sqrt(sq(planeX-dropX)+sq(planeY-dropY));
    if(plane_gnd_speed>1)
      time_to_drop=dropDistance/plane_gnd_speed;
    else time_to_drop=599;
    printTime(time_to_drop);

    // send new data to lcd
    printHeading((int)gps.angle);
    printTargetBearing(bearing);
    printTrack(err);
    printRange(dropDistance);
    printTime(time_to_drop);
    printAGL(drop_height_ft);
    printSpeed(plane_ground_speed_mph);
    
    if(!gps.fix)
      setState_aqi();
      
  }
}

void printSpeed(int gnd_speed){
  char str[4];
  sprintf(str, "%03d", gnd_speed);
  lcd.setCursor(0,0);
  lcd.print(str);
}

void printTime(int countdown){
  if(countdown<=0){
    countdown=0;
    if(not(buzzer_on)){
      tone(BUZZER, 1800, 500);
      buzzer_on=true;
    }
  }
  int seconds=countdown%60;
  int minutes=countdown/60;
  char str[5];
  sprintf(str, "%1d:%02d", minutes, seconds);
  lcd.setCursor(4,0);
  lcd.print(str);
}

void printRange(int x){
  char str[4];
  sprintf(str, "%3d", x);
  lcd.setCursor(9,0);
  lcd.print(str);
}

void printTargetBearing(int b){
  char str[4];
  sprintf(str, "%03d", b);
  lcd.setCursor(13,0);
  lcd.print(str);
}

void printHeading(int heading){
  char str[4];
  sprintf(str, "%03d", heading);
  lcd.setCursor(13,1);
  lcd.print(str);
}

void printAGL(int AGL){
  char str[5];
  sprintf(str, "%04d", AGL);
  lcd.setCursor(0,1);
  lcd.print(str);
}

void printTrack(int error){
  byte track_display[9]={0,0,0,0,'I',0,0,0,0};
  error/=5;
  if(abs(error)<1)
    track_display[4]=0x02;
  else{
    if(abs(error)>4){
      if(error<0)
        track_display[0]=0x03;
      else
        track_display[8]=0x04;
    }
    else{
      track_display[4+error]=0x01;
    }
  }
  lcd.setCursor(4,1);
  for(int i=0; i<9; i++)
    lcd.write(track_display[i]);
}

void dropSim(){
  setLeds(yellow_led & green_led);
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
  setLeds(green_led);
}

void setLeds(byte lights){
  Wire.beginTransmission(leds);
  Wire.write(lights);
  Wire.endTransmission();
}

void setState_aqi(){
  state=0;
  lcd.clear();
  setLeds(red_led);
  lcd.setCursor(0,0);
  lcd.print("No GPS fix!");
  tone(BUZZER, 1700, 100);
  delay(200);
  tone(BUZZER, 1700, 100);
  delay(200);
  tone(BUZZER, 1700, 100);
}

void setState_run(){
  setLeds(green_led);
  state=1;
  lcd.clear();
  tone(BUZZER, 2000, 200);
}

void getSetting(){
	byte upper=0;
	for(int i=0; i<4; i++){
		upper+=digitalRead(i);
		upper=upper<<1;
	}
	byte lower=0;
	for(int i=4; i<8; i++){
		lower+=digitalRead(i);
		lower=lower<<1;
	}
}