// TAYLOR, BRIAN R.
// brtaylor@umn.edu
// 2015-11-17
// 
// v2.2
//

#include <ADC.h>            // ADC library
#include <i2c_t3.h>         // I2C library

byte pack[55];      // array of bytes for sending
byte unpack[18];    // array of bytes for receiving
int k;
ADC *adc = new ADC(); // an ADC object
int ain[6];           // array to store ADC values

// UBLOX NAV PVT Data Structure
struct NAV_POSPVT {
  unsigned char cls;
  unsigned char id;
  unsigned short len;
  unsigned long iTOW;
  unsigned short utcYear;
  unsigned char utcMonth;
  unsigned char utcDay;
  unsigned char utcHour;
  unsigned char utcMin;
  unsigned char utcSec;
  byte valid;
  unsigned long timeAcc;
  long nanoSec;
  unsigned char fixType;
  byte flags;
  unsigned char reserved1;
  unsigned char satVisible;
  long longitude;
  long lattitude;
  long height;
  long hMSL;
  unsigned long hAcc;
  unsigned long vAcc;
  long navvn;
  long navve;
  long navvd;
  long gspeed;
  long heading;
  unsigned long sAcc;
  unsigned long headingAcc;
  unsigned short pDOP;
  unsigned short reserved2;
  unsigned long reserved3;
};

// UBLOX UBX header definition
const unsigned char UBX_HEADER[] = { 0xB5, 0x62 };

// a POS PVT structure
NAV_POSPVT pospvt;

// calculate the GPS data packet checksum
void calcChecksum(unsigned char* CK) {
  memset(CK, 0, 2);
  for (int i = 0; i < (int)sizeof(NAV_POSPVT); i++) {
    CK[0] += ((unsigned char*)(&pospvt))[i];
    CK[1] += CK[0];
  }
}

// process and parse the GPS data packet
bool processGPS() {
  static int fpos = 0;
  static unsigned char checksum[2];
  const int payloadSize = sizeof(NAV_POSPVT);
  while ( Serial1.available() ) {
    byte c = Serial1.read();
    if ( fpos < 2 ) {
      if ( c == UBX_HEADER[fpos] ){
        fpos++;
      }
      else
        fpos = 0;
    }
    else {
      if ( (fpos-2) < payloadSize )
        ((unsigned char*)(&pospvt))[fpos-2] = c;

      fpos++;

      if ( fpos == (payloadSize+2) ) {
        calcChecksum(checksum);
      }
      else if ( fpos == (payloadSize+3) ) {
        if ( c != checksum[0] )
          fpos = 0;
      }
      else if ( fpos == (payloadSize+4) ) {
        fpos = 0;
        if ( c == checksum[1] ) {
          return true;
        }
      }
      else if ( fpos > (payloadSize+4) ) {
        fpos = 0;
      }
    }
  }
  return false;
}

union{
  long val;
  byte b[4];
}longitude;

union{
  long val;
  byte b[4];
}lattitude;

union{
  long val;
  byte b[4];
}hMSL;

union{
  long val;
  byte b[4];
}navvn;

union{
  long val;
  byte b[4];
}navve;

union{
  long val;
  byte b[4];
}navvd;

union{
  unsigned long val;
  byte b[4];
}hAcc;

unsigned char satVisible = 0;
unsigned long iTOW;
unsigned long oldTOW = 0;
byte gpsUpdated = 0;
unsigned char fixType = 0;

int incomingByte; // byte received from Goldy

byte ps_byte[2];  // air data bytes
byte pd_byte[2];  // air data bytes

byte pwm_byte[8]; // pwm input bytes

// manual / auto GPIO mode
int modePin = 2;
byte modeVal;

// LED blinking stuff
int j = 0;

// servos
uint8_t servoindex[] = {23,22,21,3,4,5,6,10};     // servo indices
uint16_t pwmcmd[9];                               // array of microsecond commands
float cmd;

void setup() {
  // port for debugging
  Serial.begin(9600);

  // serial connection with GPS
  Serial1.begin(115200);

  // I2C for air data and PWM input
  Wire.begin(I2C_MASTER, 0, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);

   // I2C for data transfer with MPC5200
  Wire1.begin(I2C_SLAVE, 0x03, I2C_PINS_29_30, I2C_PULLUP_EXT, I2C_RATE_400);
  Wire1.onRequest(requestEvent);
  Wire1.onReceive(receiveEvent);

  // sets the digital mode pin as input
  pinMode(modePin, INPUT);

  // sets the LED pin as an output
  pinMode(13, OUTPUT);

  // setting up the analog frequencies and resolutions for servos
  analogWriteResolution(16);
  for(uint8_t i = 0; i < sizeof(servoindex); i++){
    analogWriteFrequency(servoindex[i],333);
  }
  analogWriteFrequency(32,50);
  
  // setting up ADC0
  adc->setAveraging(1);
  adc->setResolution(16);
  adc->setConversionSpeed(ADC_HIGH_SPEED_16BITS);
  adc->setSamplingSpeed(ADC_HIGH_SPEED_16BITS);

  // setting up ADC1
  adc->setAveraging(1,ADC_1);
  adc->setResolution(16,ADC_1);
  adc->setConversionSpeed(ADC_HIGH_SPEED_16BITS,ADC_1);
  adc->setSamplingSpeed(ADC_HIGH_SPEED_16BITS,ADC_1);
}

void loop() {
  // read GPS data
  if( processGPS() ){
    longitude.val = pospvt.longitude;
    lattitude.val = pospvt.lattitude;
    hMSL.val = pospvt.hMSL;
    navvn.val = pospvt.navvn;
    navve.val = pospvt.navve;
    navvd.val = pospvt.navvd;
    satVisible = pospvt.satVisible;
    iTOW = pospvt.iTOW;
    fixType = pospvt.fixType;
    hAcc.val = pospvt.hAcc;
  }
}

void requestEvent()
{
  // make the LED blink at roughly 1 Hz
  if(j > 100){
    digitalWrite(13,LOW);
  }
  else{
    digitalWrite(13,HIGH);
  }
  if(j > 200){
    j = 0;
  }
  j++;
  
  // read ADC
  ain[0] = adc->analogRead(A0, ADC_0);
  ain[1] = adc->analogRead(A2, ADC_1);
  ain[2] = adc->analogRead(A1, ADC_0);
  ain[3] = adc->analogRead(A3, ADC_1);
  ain[4] = adc->analogRead(A6, ADC_0);
  ain[5] = adc->analogRead(A15, ADC_1);
  
  // read air data
  Wire.requestFrom(2,2); // 2 bytes from address 0x02
  ps_byte[0] = Wire.read(); // put the data somewhere
  ps_byte[1] = Wire.read();

  Wire.requestFrom(3,2); // 2 bytes from address 0x03
  pd_byte[0] = Wire.read(); // put the data somewhere
  pd_byte[1] = Wire.read();
  
  // read PWM input
  Wire.requestFrom(0x79,8); // 8 bytes from address 0x79
  for(int i = 0; i<8; i++){
    pwm_byte[i] = Wire.read(); // put the data somewhere
Serial.print(pwm_byte[i]);
Serial.print("\t");
  }

  // only bother if GPS has a 3D fix
  if(fixType == 3){

    // only bother if GPS has been updated
    if((iTOW - oldTOW) > 0){
      gpsUpdated = 1;
      oldTOW = iTOW;
    }
    else{
      gpsUpdated = 0;
    }
  }

  // read GPIO, manual / auto mode
  if(digitalRead(modePin) == HIGH){
    modeVal = 1;
  }
  else{
    modeVal = 0;
  }

Serial.print(modeVal);
Serial.println();

  /* Analog Data */
  pack[0]   = (byte) (ain[0] & 0xff);
  pack[1]   = (byte) ((ain[0] >> 8) & 0xff);
  pack[2]   = (byte) (ain[1] & 0xff);
  pack[3]   = (byte) ((ain[1] >> 8) & 0xff);
  pack[4]   = (byte) (ain[2] & 0xff);
  pack[5]   = (byte) ((ain[2] >> 8) & 0xff);
  pack[6]   = (byte) (ain[3] & 0xff);
  pack[7]   = (byte) ((ain[3] >> 8) & 0xff);
  pack[8]   = (byte) (ain[4] & 0xff);
  pack[9]   = (byte) ((ain[4] >> 8) & 0xff);
  pack[10]  = (byte) (ain[5] & 0xff);
  pack[11]  = (byte) ((ain[5] >> 8) & 0xff);

  /* Air Data */
  pack[12] = ps_byte[0];
  pack[13] = ps_byte[1];
  pack[14] = pd_byte[0];
  pack[15] = pd_byte[1];

  /* PWM Data */
  pack[16] = pwm_byte[0];
  pack[17] = pwm_byte[1];
  pack[18] = pwm_byte[2];
  pack[19] = pwm_byte[3];
  pack[20] = pwm_byte[4];
  pack[21] = pwm_byte[5];
  pack[22] = pwm_byte[6];
  pack[23] = pwm_byte[7];

  /* GPIO Data */
  pack[24] = modeVal;

  /* GPS Data */
  pack[25] = gpsUpdated;
  
  pack[26] = satVisible;
  
  pack[27] = lattitude.b[0];
  pack[28] = lattitude.b[1];
  pack[29] = lattitude.b[2];
  pack[30] = lattitude.b[3];
  
  pack[31] = longitude.b[0];
  pack[32] = longitude.b[1];
  pack[33] = longitude.b[2];
  pack[34] = longitude.b[3];
  
  pack[35] = hMSL.b[0];
  pack[36] = hMSL.b[1];
  pack[37] = hMSL.b[2];
  pack[38] = hMSL.b[3];

  pack[39] = navvn.b[0];
  pack[40] = navvn.b[1];
  pack[41] = navvn.b[2];
  pack[42] = navvn.b[3];

  pack[43] = navve.b[0];
  pack[44] = navve.b[1];
  pack[45] = navve.b[2];
  pack[46] = navve.b[3];
  
  pack[47] = navvd.b[0];
  pack[48] = navvd.b[1];
  pack[49] = navvd.b[2];
  pack[50] = navvd.b[3];

  pack[51] = hAcc.b[0];
  pack[52] = hAcc.b[1];
  pack[53] = hAcc.b[2];
  pack[54] = hAcc.b[3];

  Wire1.write(pack, 55); 
}

void receiveEvent(size_t howMany)
{
  // read bytes off the i2c bus
  for(int k = 0; k<18; k++){
    unpack[k] = Wire1.read();
  }

  // assign bytes to microsecond commands
  pwmcmd[0] = ((unsigned int)unpack[1] << 8) + unpack[0];
  pwmcmd[1] = ((unsigned int)unpack[3] << 8) + unpack[2];
  pwmcmd[2] = ((unsigned int)unpack[5] << 8) + unpack[4];
  pwmcmd[3] = ((unsigned int)unpack[7] << 8) + unpack[6];
  pwmcmd[4] = ((unsigned int)unpack[9] << 8) + unpack[8];
  pwmcmd[5] = ((unsigned int)unpack[11] << 8) + unpack[10];
  pwmcmd[6] = ((unsigned int)unpack[13] << 8) + unpack[12];
  pwmcmd[7] = ((unsigned int)unpack[15] << 8) + unpack[14];
  pwmcmd[8] = ((unsigned int)unpack[17] << 8) + unpack[16];

  // setting microsecond commands
  for(uint8_t i = 0; i < sizeof(servoindex); i++){
    cmd = (float) pwmcmd[i];
    analogWrite(servoindex[i],cmd/3003.0*65535.0);
  }
  cmd = (float) pwmcmd[8];
  analogWrite(32,cmd/20000.0*65535.0);  
}

