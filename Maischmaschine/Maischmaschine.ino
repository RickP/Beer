/* Maische Maschine//

 * Sketch by elaz
  * v. 0.9 - basic functions 

  ATMega 2560 interrupts 
    2 int.0 -> RFM12B Module
    3 int.1 -> Heater LED
    21 (SCL) int.2  -> 
    20 (SDA) int.3  -> 
    19 int.4  -> 
    18 int.5  -> 

  *
  */  
#define DEBUG  0                    // set to 1 for console output, 0 for silent mode


/* ################### ###################  Variable            ###################  ###################  ###################  */ 
int heaterPin = 8;                  // relais input for heating
int LM335Pin = A0;                  // Pin A0 has an LM335 temperatur sensor 

int heaterLed1 = 20; 
int interruptServicePin = 40;                     
volatile int state = LOW;               // interrupt variable for blink or fade function call 

int speakerPin = A1;                                   // piezo Speaker on PIN A0
int speakerGround = A2;                                // piezo speaker ground port 0V
int length = 4;                                        // the number of notes
char notes[] = "cegC ";                                // a space represents a rest
int beats[] = { 1, 1, 1, 4 };                          // beats for note set
int tempo = 100;                

// Arrays for rfm12B Transmisson
String configName[10] = {{"not Set"},{"not Set"},{"not Set"},{"not Set"},{"not Set"},{"not Set"},{"not Set"},{"not Set"},{"not Set"},{"not Set"}};  ;    // array for serverside confignames
String currentConfig = "0";      // for menu to hold the current config to load
unsigned int heaterconfig[5][2] = {{60,20},{0,0},{0,0},{0,0},{0,0}};      // array for heater configuration data
// runtime vaibels
long startTime = 0;                  // set start time
int runProgram = 0;                  // switch for running program mode or on demand
float Off = 0;                       // how much degrees before switching the heater off
float On = 0;                        // how much degrees before switching the heater on
float errorcorrection = 0;           // how mich degrees to add to sensor value
float temperatur = 0;                // current temperatur 
float temp[4];                       // helper array for temp calculation
int cycle = 0;                       // helper int for reading 
int currentRaste = 0;                // current Raste start set
int heaterState = 0;                // state od heater 0 = off, 1 = on



/* ################### ###################  LCD-Display variables ###################  ###################  ###################  

 Hitachi HD44780 driver. 
 
  The circuit:
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)

 http://www.r-site.net/site/struct.asp?lang=en&at=//op[@id=%273090%27] */

#include <HardwareSerial.h> 
#include <LiquidCrystal.h>
#include <menu.h>               //menu macros and objects
#include <menuFields.h>         // to include fields and its functions
#include <pcint.h>              //this is incompatible with software serial (arduino needs an handler!)
#include <quadEncoder.h>        //quadrature encoder driver and fake stream
#include <keyStream.h>          //keyboard driver and fake stream (for the encoder button)
#include <chainStream.h>        // concatenate multiple input streams (this allows adding a button to the encoder)
#include "menuLCD.h"


 #if defined(__AVR_ATmega2560__)
  //LCD wired on my AtMega2560
  #define RS 44
  #define RW 46
  #define EN 48
  LiquidCrystal lcd1(RS, RW, EN, 43, 45, 47);
  /*
  LiquidCrystal lcd1(RS, RW, EN, 43, 45, 47, 49 ); For bigger display on digit more
  */
#else 
  #error "DEFINE YOUR LCD WIRING HERE (search for this message on code)"
#endif

////////////////////////////////////////////
// ENCODER (aka rotary switch) PINS
// rotary
#if defined(__AVR_ATmega328__)||defined(__AVR_ATmega328P__)// uno and breaduinos
  
  #define encA 2
  #define encB 4
  //this encoder has a button here
  #define encBtn A0
  #define LEDPIN 13
  
#elif defined(__AVR_ATmega2560__)

  #define encA 10
  #define encB 12
  #define encBtn 13
  #define LEDPIN 5

  
#endif

//functions to wire as menu actions

void nothing() {}
void ledOn() {digitalWrite(LEDPIN,0);}
void ledOff() {digitalWrite(LEDPIN,0);}

/////////////////////////////////////////////////////////////////////////
// MENU DEFINITION http://www.r-site.net/site/struct.asp?lang=en&at=//op[@id=%273090%27]
/* here we define the menu structure and wire actions functions to it empty options are just for scroll testing
*/
    CHOOSE(currentConfig,chooseConfig,"Choose Config",
      //VALUE("Config:" + configName[0] ,configName[0]), // try to display name of config from variable here
      VALUE("Config 0" ,configName[0]),           
      VALUE("Config 1",configName[1]),
      VALUE("Config 2",configName[2]),
      VALUE("Config 3",configName[3]),
      VALUE("Config 4",configName[4]),
      VALUE("Config 5",configName[5]),
      VALUE("Config 6",configName[6]),
      VALUE("Config 7",configName[7]),
      VALUE("Config 8",configName[8]),
      VALUE("Config 9",configName[9])
      );

  MENU(subMenu1,"START / STOP",
    OP("Start Program",startProgram),
    OP("Stop Program",stopProgram),
    SUBMENU(chooseConfig)
  );  
// SUb sub menue 2 - Programming "ratse"
  
    MENU(subsubMenu21,"Warm up",
    FIELD(heaterconfig[0][1],"Temp:"," Celcius", 0, 100, 1, 1),
    FIELD(heaterconfig[0][2], "Time:", " Minutes", 0, 240, 1, 1)
    );
    MENU(subsubMenu22,"Set Raste 1",
    FIELD(heaterconfig[1][1],"Temp:"," Celsius", 0, 100, 1, 1),
    FIELD(heaterconfig[1][2], "Time:", " Minutes", 0, 240, 1, 1)
    );
    MENU(subsubMenu23,"Set Raste 2",
    FIELD(heaterconfig[2][1],"Temp:"," Celsius", 0, 100, 1, 1),
    FIELD(heaterconfig[2][2], "Time:", " Minutes", 0, 240, 1, 1)
    );
    MENU(subsubMenu24,"Set Raste 3",
    FIELD(heaterconfig[3][1],"Temp:"," Celsius", 0, 100, 1, 1),
    FIELD(heaterconfig[3][2], "Time:", " Minutes", 0, 240, 1, 1)
    );
    MENU(subsubMenu25,"Set Raste 4",
    FIELD(heaterconfig[4][1],"Temp:"," Celsius", 0, 100, 1, 1),
    FIELD(heaterconfig[4][2], "Time:", " Minutes", 0, 240, 1, 1)
    );

  MENU(subMenu2,"PROGRAM Heater",
    OP("Send config to Server",sendConfig),
    SUBMENU(subsubMenu21),
    SUBMENU(subsubMenu22),
    SUBMENU(subsubMenu23),
    SUBMENU(subsubMenu24),
    SUBMENU(subsubMenu25)
  );
// SUb Sub meue wireless
    MENU(subsubMenu31,"Display Wireless Setup",
    OP("Display Setup",ledOn),
    OP("Programm")
    );
    MENU(subsubMenu32,"Disable / Enable",
    OP("Display Setup",ledOn),
    OP("Programm")
    );
    MENU(subsubMenu33,"Change Setup",
    OP("Display Setup",ledOn),
    OP("Programm")
    );

  MENU(subMenu3,"WIRELESS SETUP",
    SUBMENU(subsubMenu31),
    SUBMENU(subsubMenu32),
    SUBMENU(subsubMenu33),
    OP("Programm")
  );


MENU(mainMenu,"Main Menu",
  SUBMENU(subMenu1),
  SUBMENU(subMenu2),
  SUBMENU(subMenu3),
  //SUBMENU(subMenu4),
  OP("Just heat 100°C",nothing)
);




//the quadEncoder
quadEncoder quadEncoder(encA,encB);//simple quad encoder driver
quadEncoderStream enc(quadEncoder,5);// simple quad encoder fake Stream

//a keyboard with only one key :D, this is the encoder button
keyMap encBtn_map[]={{-encBtn,menu::enterCode}};//negative pin numbers means we have a pull-up, this is on when low
keyLook<1> encButton(encBtn_map);

//multiple inputs allow conjugation of the quadEncoder with a single key keyboard that is the quadEncoder button
Stream* in[]={&enc,&encButton};
chainStream<2> quadEncoder_button(in);

//alternative to previous but now we can input from Serial too...
Stream* in3[]={&enc,&encButton,&Serial};
chainStream<3> allIn(in3);

//describing a menu output, alternatives so far are Serial or LiquidCrystal LCD
menuLCD lcd(lcd1,16,1);



/* ###################  ###################  RF12 Communications  ###################  ###################  ###################  
#define RFM_IRQ     2                    (IRQ) 
#define SS_DDR      DDRB
#define SS_PORT     PORTB
#define SS_BIT      0

#define SPI_SS      53    // PB0, pin 19  (SEL)
#define SPI_MOSI    51    // PB2, pin 21  (SDI)
#define SPI_MISO    50    // PB3, pin 22  (SDO)
#define SPI_SCK     52    // PB1, pin 20  (SCK)
PD2    <---------> IRQ
SS      <---------> SEL
MOSI     <---------> SDI
MISO     <---------> SDO
SCK      <---------> SCK
*/

#include <JeeLib.h>                     //https://github.com/jcw/jeelib

#define RF12_GROUPID 210                // All nodes must be a member of the same group
#define RF12_NODEID_LOCAL  1            // Local node within a group must have a unique ID
#define RF12_NODEID_REMOTE   16         // Remote node is derver node here
#define RF12_WAIT 500                   //ms Time to wait for Remote node to respond to a data request
#define ACK_TIME 10                     // number of milliseconds to wait for an ack
#define RFID_CHK  50                    // Checksum of RFID sender in case All Char into byte and +

unsigned int pingInterval = 0;          // Define Intervall for sending data to server
unsigned int lastSend = 0;              // time variable for last send
                                        // incoming dataset
typedef struct
{
  int nodeId;                           // node ID severd from server
  int payloadtype;                      // 0 = standard , 1 raste config name load , 2 = raste config data load      
  int yearMonth;                        // current server yearMonth
  int dayHour;                         // current server dayHour
  int value1;                           // raste
  int value2;                           // temperatur
  int value3;                           // time in minutes
  
} incomingPayload;                       // data structure comming in

incomingPayload PayloadRX;

                                        //outgoing dataset
typedef struct 
{ 
  int nodeId;                           // node ID severd from server
  int payloadtype;                      // 0 = send current raste data (1-4), 1 = ask to load heater config names(1-2), 2 = ask next packet for heater data load (1-3), 3 send heaterconfig (1-3)
  int yearMonth;                        // current yearMonth
  int dayHour;                          // current dayHour               
  int value1;                           // raste
  int value2;                           // temperature
  int value3;                           // time in seconds or minutes (if payloadtype = 1)
  int value4;                           // heating state (0 = cooling, 1= heating) 
 
} outgoingPayload;                    // structure going out
outgoingPayload PayloadTX;

// #######################            RFM12B send & receiver function
// function loadConfig
void loadConfigNames()
{
  lastSend = sendRFM(1,1,0,0,0);        // send request to server payload type = 1 for config name
  readRFM();                            // read it
  //PayloadRX.payloadtype = 0;
}

// function loadConfig
void loadConfig()                       // load config data
{
  PayloadTX.yearMonth = currentConfig.substring(0,3).toInt();   //convert und cut configname into 2 int values
  PayloadTX.dayHour = currentConfig.substring(4,7).toInt();     //convert und cut configname into 2 int values
  lastSend = sendRFM(2,1,0,0,0);        // send request to server payload type = 2 for config data, 1 for first raste
  readRFM();                            // read it until 5. raste
  //PayloadRX.payloadtype = 0;
}

// function safe config
void sendConfig(){
  
  for (int i; i < 5; i++)                 // iterate trough config array
  {  
    if(heaterconfig[i][2] != 0)           // send only if time is set
  sendRFM(3, i, heaterconfig[i][1], heaterconfig[i][2], 0);
  }
  lcd1.print("Config Data send");
}

void readRFM()
{
if (rf12_recvDone() && rf12_crc == 0 && (rf12_hdr & RF12_HDR_CTL) == 0)
  {
    
    int node_id = (rf12_hdr & 0x1F);
    if (node_id == RF12_NODEID_REMOTE)  // listen only to server address set abouve
    {
      // The packet data is contained in rf12_data, the *(incomingPayload*) part tells the compiler
      // what the format of the data is so that it can be copied correctly
      PayloadRX = *(incomingPayload*) rf12_data;
     
      if (PayloadRX.nodeId == RF12_NODEID_LOCAL)    // copy incoming time to outgoing packages
      {
        PayloadTX.nodeId = PayloadRX.nodeId;

        if (PayloadRX.payloadtype == 2)         // if payloadtype is set to 2 heater config data fill array 
        {
          if (PayloadRX.value1 == 1) 
          {
            heaterconfig[0][0] = PayloadRX.value1; // raste 1
            heaterconfig[0][1] = PayloadRX.value2; // temperatur 
            heaterconfig[0][2] = PayloadRX.value3; // time in minutes
            lastSend = sendRFM(2,2,0,0,0);         // ask for next packet
          } else if (PayloadRX.value1 == 2)
          {
            heaterconfig[1][0] = PayloadRX.value1; // raste 2
            heaterconfig[1][1] = PayloadRX.value2; // temperatur 
            heaterconfig[1][2] = PayloadRX.value3; // time in minutes
            lastSend = sendRFM(2,3,0,0,0);         // ask for next packet
          } else if (PayloadRX.value1 == 3)
          {
            heaterconfig[2][0] = PayloadRX.value1; // raste 3
            heaterconfig[2][1] = PayloadRX.value2; // temperatur 
            heaterconfig[2][2] = PayloadRX.value3; // time in minutes
            lastSend = sendRFM(2,4,0,0,0);         // ask for next packet
          } else if (PayloadRX.value1 == 4)
          {
            heaterconfig[3][0] = PayloadRX.value1; // raste 4
            heaterconfig[3][1] = PayloadRX.value2; // temperatur 
            heaterconfig[3][2] = PayloadRX.value3; // time in minutes
            lastSend = sendRFM(2,5,0,0,0);         // ask for next packet
          } else if (PayloadRX.value1 == 5)
          {
            heaterconfig[4][0] = PayloadRX.value1; // raste 5
            heaterconfig[4][1] = PayloadRX.value2; // temperatur 
            heaterconfig[4][2] = PayloadRX.value3; // time in minutes
            PayloadRX.payloadtype = 0;
          }
        }else if (PayloadRX.payloadtype == 1)  // if payloadtype ist set to 1 config names will be loaded
          {
              // load names here into configName[100][2];
              int brewNameSet = PayloadRX.value1;
              int nextbrewNameSet = PayloadRX.value1 +1;
              configName[brewNameSet] = String(PayloadRX.yearMonth) + String(PayloadRX.dayHour);
              if(PayloadRX.value2 == 1){
                lastSend = sendRFM(1,nextbrewNameSet,0,0,0);
              } else
              {
                PayloadRX.payloadtype = 0;
              }
              /*
            if (PayloadRX.value1 == 1) 
            {
              configName[0][0] = PayloadRX.yearMonth;
              configName[0][1] = PayloadRX.dayHour;
              lastSend = sendRFM(2,1,0,0,0);
            } 
            if (PayloadRX.value1 == 2) 
            {
              configName[1][0] = PayloadRX.yearMonth;
              configName[1][1] = PayloadRX.dayHour;
              lastSend = sendRFM(2,2,0,0,0);
            } 
            if (PayloadRX.value1 == 3) 
            {
              configName[2][0] = PayloadRX.yearMonth;
              configName[2][1] = PayloadRX.dayHour;
              lastSend = sendRFM(2,3,0,0,0);
            } 
            if (PayloadRX.value1 == 4) 
            {
              configName[3][0] = PayloadRX.yearMonth;
              configName[3][1] = PayloadRX.dayHour;
              lastSend = sendRFM(2,4,0,0,0);
            } 
            if (PayloadRX.value1 == 5) 
            {
              configName[4][0] = PayloadRX.yearMonth;
              configName[4][1] = PayloadRX.dayHour;
            } */
          }
          PayloadTX.yearMonth = PayloadRX.yearMonth;
          PayloadTX.dayHour = PayloadRX.dayHour;
          PayloadTX.payloadtype = PayloadRX.payloadtype;
          lastSend = millis()/1000; // set time of last sending
      }  // end nodeID-LOCAL if                              
    }    // end nodeID-REMOTE if     
  }     // if receive done
} // function end


int sendRFM(unsigned int ptype, unsigned int v1, unsigned int v2, unsigned int v3, unsigned int v4)
{
  PayloadTX.payloadtype = ptype;
  PayloadTX.yearMonth = PayloadRX.yearMonth;
  PayloadTX.dayHour = PayloadRX.dayHour;
  PayloadTX.value1 = v1;
  PayloadTX.value2 = v2;
  PayloadTX.value3 = v3;
  PayloadTX.value4 = v4;
   
  if (rf12_recvDone())
  {
   rf12_sendNow(RF12_HDR_ACK, &PayloadTX, sizeof PayloadTX);  // send data

#if DEBUG
  Serial.println();
#endif
  // Acknogle would be good here before return
  
  }
  byte acked = waitForAck();
   if (acked) 
   {
    return millis()/1000; // set time of last sending
    }else
    {
    rf12_sendNow(RF12_HDR_ACK, &PayloadTX, sizeof PayloadTX);  // send data again
    /*
    if (rf12_recvDone())  // send again if there was no replay
      sendRFM(PayloadTX.payloadtype,PayloadTX.value1,PayloadTX.value2,PayloadTX.value3,PayloadTX.value4);*/
    }
}

// function waitForAck: wait a few milliseconds for proper ACK to me, return true if indeed received
static byte waitForAck() 
{
   
  MilliTimer ackTimer;

  while (!ackTimer.poll(50)) 
  {
  /* rf12_hdr = "193" value is the decimal representation of a byte with several bits set. 
  If we decompose this as binary, it's: 128 + 64 + 1. These values have the following meaning:
  128 = RF12_HDR_CTL, this is a special control packet (an ACK) , 
  64 = RF12_HDR_DST, the lower bits represent a _destination+ node ID, 
  1 = this packet is intended for node ID 1, which is indeed the number assigned to this n
  
  -> recieve statement:  && rf12_hdr = RF12_HDR_CTL + RF12_HDR_DST + emontx.nodeId
  */
    if (rf12_recvDone() && rf12_crc == 0 && RF12_HDR_CTL == 128 && RF12_HDR_DST == 64 && (rf12_hdr & 0x1F) == PayloadTX.nodeId  )
    {
#if DEBUG
        Serial.print("rf12_crc: ");Serial.println(rf12_crc);
        Serial.print("HDR_DST: ");Serial.println(RF12_HDR_DST);
        Serial.print("HDR_CLT: ");Serial.println(RF12_HDR_CTL);
        Serial.print("rf12_hdr: ");Serial.println(rf12_hdr);
        Serial.print("Node: ");Serial.println(int(rf12_hdr & 0x1F));
#endif       
    return 1;
    }
   }
  return 0;   
}

/* ############################################################# LOOP functions #############################################*/
// function beep on speaker

void piezoSound() {
  for (int i = 0; i < length; i++) {
    if (notes[i] == ' ') {
      delay(beats[i] * tempo); // rest
    } else {
      playNote(notes[i], beats[i] * tempo);
    }

    // pause between notes
    delay(tempo / 2); 
  }
}



void playTone(int tone, int duration) {
  for (long i = 0; i < duration * 1000L; i += tone * 2) {
    digitalWrite(speakerPin, HIGH);
    delayMicroseconds(tone);
    digitalWrite(speakerPin, LOW);
    delayMicroseconds(tone);
  }
}

void playNote(char note, int duration) {
  char names[] = { 'c', 'd', 'e', 'f', 'g', 'a', 'b', 'C' };
  int tones[] = { 1915, 1700, 1519, 1432, 1275, 1136, 1014, 956 };

  // play the tone corresponding to the note name
  for (int i = 0; i < 8; i++) {
    if (names[i] == note) {
      playTone(tones[i], duration);
    }
  }
}


// Display Temperatur & time
void displayTemp()
{
  /*lcd1.clear(); 
  lcd1.print("Temperatur: ");
  lcd1.print(temperatur);*/
  lcd.setCursor(0,1);
  lcd1.print(currentRaste);
  lcd1.print(" ");
  if (startTime != 0){
    long t = (millis()/1000) - startTime;
    if (t <= 60){
      lcd1.print(t);
    }
    else
    {
      int m = t/60;
      lcd1.print(m);
      lcd1.print("m");
      lcd1.print(t-(m*60));
    }
  }
  else{
    lcd1.print("0");
  }
  lcd1.print("s ");
  lcd.setCursor(9,1);
  lcd1.print(temperatur);
  lcd1.print("C");
}



// function start Program
void startProgram()
{
  runProgram = 1;
  startTime = millis()/1000;
}

// function stop Program
void stopProgram()
{
  runProgram = 0;
  startTime = 0;
}

// function to calculate timing and raste
int setRaste(long currentTime)
{
   
  if (heaterconfig[currentRaste][2] > (currentTime - startTime)){
    
    currentRaste++;
    startTime = 0;
    piezoSound();
  }
#if DEBUG
  Serial.println("");
  Serial.print("currentRaste");Serial.println(currentRaste);
#endif 
  return currentRaste;
}

// function readsensor:  int heaterconfig[5][2]; 
int readSensor(int heaterState){

  temp[cycle] = analogRead(LM335Pin) * 0.004882812 * 100;
  //delay(10);

  if (cycle >= 4)                     // send data via rfm12 if read 5 cycles
  {
    temperatur = (temp[0] + temp[1] + temp[2] + temp[3] + temp[4]) / cycle;
    int v2 = int(temperatur);
    int v3 = millis()/1000;
    int v4 = heaterState;
    /*
    int v1;                           // raste
    int v2;                           // temperature
    int v3;                           // time in seconds or minutes (if payloadtype = 1)
    int v4;                           // heating state (0 = cooling, 1= heating) 
    */
    sendRFM(0, currentRaste, v2, v3, v4);
    cycle = 0; 
  }else
  {
    cycle++; 
  }
#if DEBUG
  Serial.println("");
  Serial.print("Temperatur - current");Serial.println(temp[cycle]);
  Serial.print("Temperatur - avarage");Serial.println(temperatur);
#endif 
  return temp[cycle];
}

// function setHeater:  to calculate heater setting (ON / OFF)
int setHeater(float temp)
{
  if(temperatur <= heaterconfig[currentRaste][1] && runProgram == 1)
  {
    digitalWrite(heaterPin, HIGH);
    heaterState = 1;
  }else if (temperatur > heaterconfig[currentRaste][1] && runProgram == 1)
  {
    digitalWrite(heaterPin, LOW);
    heaterState = 0;
  }else if (runProgram == 0)
  {
    digitalWrite(heaterPin, LOW);
    heaterState = 0; 
  }
#if DEBUG
  Serial.println("");
  Serial.print("heaterState");Serial.println(heaterState);
#endif 
  return heaterState;
}



/* ############################################################## setup #####################################################*/

 void setup() { 

//                   Rotation Encoder and lcd 
  quadEncoder.begin();
  
  lcd1.begin(16,2);

  pinMode(encBtn, INPUT);
  digitalWrite(encBtn,1);
  
  pinMode(LEDPIN,OUTPUT);

  delay(300);

//                 Serial declartion
  Serial.begin (9600);
  //Serial2.begin (9600);
  //Serial3.begin (9600);
  Serial.println("LDC startup");


//                  RFM12B module setup
  rf12_config();              // read settings from Eprom
  delay(50);
  rf12_initialize(RF12_NODEID_LOCAL,RF12_868MHZ,RF12_GROUPID); // set NodeID, Frequency, Group
  Serial.println("Loading RFM12B configuration");
  Serial.print("Current nodeID: ");Serial.println(RF12_NODEID_LOCAL);
  lcd1.print("Current nodeID: ");
  lcd1.setCursor(0, 1); 
  lcd1.print(RF12_NODEID_LOCAL);
  //delay(4000);
  lcd1.setCursor(0, 0); 
  Serial.print("Current networkgroup: ");Serial.print(RF12_GROUPID);
  lcd1.print("Current net: ");
  lcd1.setCursor(0, 1); 
  lcd1.println(RF12_GROUPID);
  Serial.println();
  Serial.println("Configuration finished, startup done");
  Serial.println(RF12_NODEID_REMOTE);  
  //delay(4000);
  lcd1.setCursor(0, 0); 
  lcd1.print("Let's make some nice");
  lcd1.setCursor(0, 1);
  lcd1.print("----BEEEEER----");
  //delay(4000); 
  // clear screen for the next loop:
  lcd1.clear();
 } 
/* ############################################################## Main loop #####################################################*/

void loop()
{
  
    currentRaste = setRaste(millis()/1000);                      // calculate raste timer
    temperatur = readSensor(heaterState);                        // read sensor value with int heaterState)
    heaterState = setHeater(temperatur);                         // calculate heater setting by temperatur
 

  mainMenu.poll(lcd,allIn);
  displayTemp();
  digitalWrite(heaterLed1, state);                 // let LED blink
  
  if (millis()/1000 >= lastSend + pingInterval)   // only start sending / recieve on interval setting  
  { 
    readRFM();                                    // read config from serverside and send keepalive with data
  }
#if DEBUG
  Serial.println("");
  Serial.println("ConfigName 0");Serial.println(configName[0]);
  Serial.println("ConfigName 1");Serial.println(configName[1]);
  Serial.println("ConfigName 2");Serial.println(configName[2]);
  Serial.println("ConfigName 3");Serial.println(configName[3]);
  Serial.println("ConfigName 4");Serial.println(configName[4]);
  Serial.println("ConfigName 5");Serial.println(configName[5]);
  Serial.println("ConfigName 6");Serial.println(configName[6]);
  Serial.println("ConfigName 7");Serial.println(configName[7]);
  Serial.println("ConfigName 8");Serial.println(configName[8]);
  Serial.println("ConfigName 9");Serial.println(configName[9]);

  Serial.println("Current Config");Serial.println(currentConfig);

  Serial.println("HeaterConfig Raste 0, Temp");Serial.println(heaterconfig[0][1);
  Serial.println("HeaterConfig Raste 0, time");Serial.println(heaterconfig[0][2);
  Serial.println("HeaterConfig Raste 1, Temp");Serial.println(heaterconfig[1][1);
  Serial.println("HeaterConfig Raste 1, time");Serial.println(heaterconfig[1][2);
  Serial.println("HeaterConfig Raste 2, Temp");Serial.println(heaterconfig[2][1);
  Serial.println("HeaterConfig Raste 2, time");Serial.println(heaterconfig[2][2);
  Serial.println("HeaterConfig Raste 3, Temp");Serial.println(heaterconfig[3][1);
  Serial.println("HeaterConfig Raste 3, time");Serial.println(heaterconfig[3][2);
  Serial.println("HeaterConfig Raste 4, Temp");Serial.println(heaterconfig[4][1);
  Serial.println("HeaterConfig Raste 4, time");Serial.println(heaterconfig[4][2);
#endif 

}


//                    LED blink function
void blink()
{
  state = !state;
}

