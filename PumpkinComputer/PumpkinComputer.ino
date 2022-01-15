#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_GPS.h>
#define BUZZER 3
#define leds 0x20
#define green_led 0b11111110
#define red_led 0b11111101
#define yellow_led 0b11111011
#define time_zone 18

int state;

float pumpkin_cd=1.00;  //pumpkin coefficient of drag
float pumpkin_mass=0.830; //kg
float pumpkin_circumference=0.40; //meters
float pumpkin_cross_section=0.1;
int wind_speed_mph=0;
int wind_dir=0;
float targetLat=41.927338;
float targetLon=-91.425406;
float targetAlt_ft=860;
int bearing=309; //bearing of the flight path over the target

float bearingRad;
float windX;
float windY;

float mPerLat;
float mPerLon;

float air_density=1.225;  //kg/m^3
int plane_ground_speed_mph;  //plane ground speed in mph
float plane_gnd_speed;
int drop_height_ft;  //drop height in feet
int head_wind;
int cross_wind;

float time_step=0.01;  //sim time step

float pumpkin_air_time;

float dropX;
float dropY;

float g=-9.81;
float pi=3.14159265;

long start_time;
int time_to_drop;
bool buzzer_on;

LiquidCrystal_I2C lcd(0x27,16,2); //init lcd with address, columns, and rows
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


void dropSim(){
  setLeds(yellow_led || red_led);
  plane_gnd_speed=plane_ground_speed_mph*0.44704;
  float drop_height=drop_height_ft*0.3048;
  float plane_air_speed=plane_gnd_speed+head_wind;
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

  //float pumpkin_cross_section=pi*pow(pumpkin_circumference/(2*pi),2);

  float drag_z;
  float drag_y;
  float drag_x;
  float accel_y;
  
  while(pumpkin_z>0.0){
    sim_time+=time_step;

    drag_z=pumpkin_cd*pumpkin_cross_section*air_density*pow(pumpkin_vertical_speed,2)/2;
    pumpkin_vertical_speed+=(g*time_step)+(drag_z/pumpkin_mass)*time_step;
    pumpkin_z+=pumpkin_vertical_speed*time_step;

    drag_y=pumpkin_cd*pumpkin_cross_section*air_density*pow(pumpkin_y_air_speed,2)/2;
    accel_y=drag_y/pumpkin_mass;
    pumpkin_y_gnd_speed-=accel_y*time_step;
    pumpkin_y_air_speed=pumpkin_y_gnd_speed+head_wind;
    pumpkin_y+=pumpkin_y_gnd_speed*time_step;

    drag_x=pumpkin_cd*pumpkin_cross_section*air_density*pow(pumpkin_x_air_speed,2)/2;
    pumpkin_x_gnd_speed+=(drag_x/pumpkin_mass)*time_step;
    pumpkin_x+=pumpkin_x_gnd_speed*time_step;
    pumpkin_x_air_speed=cross_wind-pumpkin_x_gnd_speed;
  }
  pumpkin_air_time=sim_time;
  dropX=-pumpkin_x;
  dropY=-pumpkin_y;
  setLeds(green_led);
}


void setup() {
  Serial.begin(115200);
  Serial.println("pumpkin-computer Debug edition!");
  float wind_speed=wind_speed_mph*0.44704;
  head_wind=cos((wind_dir-bearing)*pi/180)*wind_speed;
  cross_wind=-sin((wind_dir-bearing)*pi/180)*wind_speed;
  windX=sin(wind_dir*pi/180)*wind_speed;
  windY=cos(wind_dir*pi/180)*wind_speed;
  Serial.print("windX: "); Serial.println(windX);
  Serial.print("windY: "); Serial.println(windY);

  bearingRad = bearing*pi/180;
  float radLat=targetLat*pi/180;
  mPerLat = 111132.92-559.82*cos(2*radLat)+1.175*cos(4*radLat)-0.0023*cos(6*radLat);
  mPerLon = 111412.84*cos(radLat)-93.5*cos(3*radLat)+0.118*cos(5*radLat);

  gps.begin(0x10);
  gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  gps.sendCommand(PMTK_SET_NMEA_UPDATE_2HZ);
  
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, blank);
  lcd.createChar(1, plane_char);
  lcd.createChar(2, on_target);
  lcd.createChar(3, left_track);
  lcd.createChar(4, right_track);

  start_time=millis();
  
  setLeds(red_led & yellow_led & green_led);
  for(int i=0; i<4; i++){
    tone(BUZZER,1500,50);
    delay(100);
  }
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

    Serial.print("plane_lat: "); Serial.println(plane_lat, 6);
    Serial.print("plane_lon: "); Serial.println(plane_lon, 6);
    
    //get plane coordinates in local xy system
    float planeX=mPerLon*(plane_lon-targetLon);
    float planeY=mPerLat*(plane_lat-targetLat);
    Serial.print("planeX: "); Serial.println(planeX);
    Serial.print("planeY: "); Serial.println(planeY);

    //update variables with new gps data
    plane_ground_speed_mph=(gps.speed*1.151);
    printSpeed(plane_ground_speed_mph);
    drop_height_ft=(gps.altitude*3.28-targetAlt_ft);
    Serial.print("gps.altitude: "); Serial.println(gps.altitude);

    //run simulation and update drop coordinates
    dropSim();
    printAGL(drop_height_ft);
    printHeading((int)gps.angle);
    printTargetBearing(bearing);

    //calculate flight path
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
    printTrack(err);
    printRange(err);

    //calculate time to drop
    float dropDistance=sqrt(sq(pathX-dropX)+sq(pathY-dropY));
    if(plane_gnd_speed>1)
      time_to_drop=dropDistance/plane_gnd_speed;
    else time_to_drop=599;
    printTime(time_to_drop);
    
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

void printTargetBearing(int bearing){
  char str[4];
  sprintf(str, "%03d", bearing);
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
