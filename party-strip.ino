#include <Adafruit_NeoPixel.h>
#include <SerialFlash.h>
#include <nRF24L01.h>
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#define BASE_GAIN .2
#define OP_GAIN_MIN 0
#define OP_GAIN_MAX 20

#define BRIGHTNESS_MAX 255
#define BRIGHTNESS_MIN 0

#define mirf_CONFIG ((1<<EN_CRC) | (1<<CRCO) )
#define NUM_RESPONSES 140
#define NUM_ANSWERS 3

uint8_t addr[3] = {0x56, 0x34, 0x12};
uint8_t tx_addr[3];
uint8_t rec[4] = {0x00, 0x00, 0x00, 0x00};
uint8_t ok_resp[1] = {0x11};
uint8_t channel = 53;
uint8_t cePin = 9;
uint8_t csnPin = 10;
uint8_t PTX;

unsigned long lastPoll = 0;
int pollInterval = 100;



#define NUM_LEDS 52
#define LED_PIN 6
#define initFac 10000

// GUItool: begin automatically generated code
AudioInputAnalog         adc1;           //xy=202,291
AudioAnalyzeFFT1024     fft256;      //xy=433,289
AudioConnection          patchCord1(adc1, fft256);
// GUItool: end automatically generated code

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
double gr = .08;//70;//118;
//double lgr = 4;

uint8_t opGain = 5;
uint8_t opMode = 0;
uint8_t prevMode = 0;

long lightTimer = 0;
int lightUpdateTimeout = 10;
uint8_t lightBrightness = 100;

uint16_t wheelOffset = 0;
uint8_t wheelIncrement = 1;

double ledScales[NUM_LEDS];
double lightGain = 1;
bool killLights = 0;

void setup() {
  Serial.begin(115200);

  AudioMemory(12);
  strip.begin();
  strip.show();

  strip.setBrightness(lightBrightness);
  strip.show();

  fft256.windowFunction(AudioWindowHanning1024);
  int i;

  for ( i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 255));
    strip.show();
    delay(7);
  }
  for ( i = NUM_LEDS - 1; i >= 0; i--) {
    strip.setPixelColor(i, strip.Color(0, 0, 0));
    strip.show();
    delay(7);
    ledScales[i] = 0;
  }


  // configure radio
  pinMode(cePin,OUTPUT);
  pinMode(csnPin,OUTPUT);

  ceLow();
  csnHi();
  
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  
  configure_as_receiver();
  
}

void loop() {
  int i;
  int j;


  // keep the colors cycling
  if ( millis() - lightTimer > lightUpdateTimeout ) {
    //Serial.println("updating wheel");
    lightTimer = millis();
    
    advanceColors();
    
    switch(opMode) {
    case '1':
      updateWheel();
      break;
      
    case '2':
      updateChase();
      break;

    case '3':
      updateCollide();
      break;

    case '8':
      changeBrightness();
      break;
      
    case '?':
      changeGain();
      break;
    
    case '0':
      lightsOff();
      break;

    }
  }

  if( millis() - lastPoll > pollInterval ){
    //Serial.println("polling");
    lastPoll = millis();
    prevMode = opMode;
    opMode = pollClicker();
  }

  if(fft256.available()) {
    //Serial.println("fft begin");
    
    int low = 0;
    int high = 1;
    
    for(i = 0; i < NUM_LEDS; i++) {
      
      double c = 500*pow(fft256.read(low, high), 2);
      low = high;
      
      if( i < 25 ){ 
         high++;
      } else if( i >= 25 && i < 40 ){
        high += 2;
        c*=1.5;
      } else {
        high += 3;
        c*=2;
      }
      
      ledScales[i] = c > 1 ? 1 : c;

    }
  }

}

uint8_t pollClicker() {
  //configure_as_receiver();
  //Serial.println("Listening...");
  
  uint8_t i;
  powerUpRx();
  if( dataReady() ){
    getData(rec, 4);
    delay(1);


    Serial.print("recieved: ");
    Serial.print(rec[3]-49);

    
    tx_addr[0] = rec[2];
    tx_addr[1] = rec[1];
    tx_addr[2] = rec[0];
    
    writeRegister(TX_ADDR, tx_addr, 3);
    
    RFsend(ok_resp, 1);
    delay(20);
    RFsend(ok_resp, 1);
    delay(10);
    RFsend(ok_resp, 1);
    delay(10);
    RFsend(ok_resp, 1);
    delay(10);
    RFsend(ok_resp, 1);
    
    
    // we're still in send mode so get ready to receive again
    powerUpRx();
    return rec[3];
  } else {
    return prevMode;
  }
}


boolean dataReady() 
// Checks if data is available for reading
{
  // See note in getData() function - just checking RX_DR isn't good enough
  uint8_t status = getStatus();
  // We can short circuit on RX_DR, but if it's not set, we still need
  // to check the FIFO for any pending packets
  if ( status & (1 << RX_DR) ) return 1;
  
  return !rxFifoEmpty();
}

boolean rxFifoEmpty(){
  uint8_t fifoStatus;
  readRegister(FIFO_STATUS,&fifoStatus,sizeof(fifoStatus));
  return (fifoStatus & (1 << RX_EMPTY));
}

void transmitSync(uint8_t *dataout,uint8_t len){
  uint8_t i;
  for(i = 0;i < len;i++){
    SPI.transfer(dataout[i]);
  }
}

void transferSync(uint8_t *dataout,uint8_t *datain,uint8_t len){
  uint8_t i;
  for(i = 0;i < len;i++){
    datain[i] = SPI.transfer(dataout[i]);
  }
}

void configRegister(uint8_t reg, uint8_t value)
// Clocks only one byte into the given MiRF register
{
  csnLow();
  SPI.transfer(W_REGISTER | (REGISTER_MASK & reg));
  SPI.transfer(value);
  csnHi();
}

void readRegister(uint8_t reg, uint8_t * value, uint8_t len)
// Reads an array of bytes from the given start position in the MiRF registers.
{
    csnLow();
    SPI.transfer(R_REGISTER | (REGISTER_MASK & reg));
    transferSync(value,value,len);
    csnHi();
}

void writeRegister(uint8_t reg, uint8_t * value, uint8_t len) 
// Writes an array of bytes into inte the MiRF registers.
{
  csnLow();
  SPI.transfer(W_REGISTER | (REGISTER_MASK & reg));
  transmitSync(value,len);
  csnHi();
}

uint8_t getStatus(){
  uint8_t rv;
  readRegister(NRF_STATUS,&rv,1);
  return rv;
}

void ceHi(){
  digitalWrite(cePin,HIGH);
}

void ceLow(){
  digitalWrite(cePin,LOW);
}

void csnHi(){
  digitalWrite(csnPin,HIGH);
}

void csnLow(){
  digitalWrite(csnPin,LOW);
}

void powerUpRx(){
  PTX = 0;
  ceLow();
  configRegister(NRF_CONFIG, mirf_CONFIG | ( (1<<PWR_UP) | (1<<PRIM_RX) ) );
  ceHi();
  configRegister(NRF_STATUS,(1 << TX_DS) | (1 << MAX_RT)); 
}

void flushRx(){
    csnLow();
    SPI.transfer( FLUSH_RX );
    csnHi();
}

void powerUpTx(){
  PTX = 1;
  configRegister(NRF_CONFIG, mirf_CONFIG | ( (1<<PWR_UP) | (0<<PRIM_RX) ) );
}

boolean isSending(){
  uint8_t status;

  if(PTX){
    status = getStatus();
    /*
     *  if sending successful (TX_DS) or max retries exceded (MAX_RT).
     */
    if((status & ((1 << TX_DS)  | (1 << MAX_RT)))){
      powerUpRx();
      return false; 

    }
    return true;
  }
  return false;
}

void singleSend(uint8_t * value) 
// Sends a data package to the default address. Be sure to send the correct
// amount of bytes as configured as payload on the receiver.
{
  uint8_t status;
  status = getStatus();   
  while (PTX) {
    status = getStatus();
    if((status & ((1 << TX_DS)  | (1 << MAX_RT)))){
      PTX = 0;
      break;
    }
  }                  // Wait until last paket is send
  ceLow();
  powerUpTx();       // Set to transmitter mode , Power up
  csnLow();                    // Pull down chip select
  SPI.transfer( FLUSH_TX );     // Write cmd to flush tx fifo
  csnHi();                    // Pull up chip select
  csnLow();                    // Pull down chip select
  SPI.transfer( W_TX_PAYLOAD ); // Write cmd to write payload
  transmitSync(value,1);   // Write payload
  csnHi();    // Pull up chip select
  ceHi(); // Start transmission
}

void RFsend(uint8_t * value, uint8_t len) 
// Sends a data package to the default address. Be sure to send the correct
// amount of bytes as configured as payload on the receiver.
{
  uint8_t status;
  status = getStatus();   
  while (PTX) {
    status = getStatus();
    if((status & ((1 << TX_DS)  | (1 << MAX_RT)))){
      PTX = 0;
      break;
    }
  }                  // Wait until last paket is send
  ceLow();
  powerUpTx();       // Set to transmitter mode , Power up
  csnLow();                    // Pull down chip select
  SPI.transfer( FLUSH_TX );     // Write cmd to flush tx fifo
  csnHi();                    // Pull up chip select
  csnLow();                    // Pull down chip select
  SPI.transfer( W_TX_PAYLOAD ); // Write cmd to write payload
  transmitSync(value, len);   // Write payload
  csnHi();    // Pull up chip select
  ceHi(); // Start transmission
}

void getData(uint8_t * data, uint8_t len) 
// Reads payload bytes into data array
{
  csnLow();                               // Pull down chip select
  SPI.transfer( R_RX_PAYLOAD );            // Send cmd to read rx payload
  transferSync(data, data, len); // Read payload

  csnHi();                               // Pull up chip select
  configRegister(NRF_STATUS, (1<<RX_DR));   // Reset status register
}

void configure_as_clicker() {
  configRegister(RX_PW_P0, 1);           // payload length of 1 for a clicker
  writeRegister(TX_ADDR, addr, 3);       // set transmit address
}

void configure_as_receiver() {
  configRegister(EN_AA, 0);              // disable shockburst
  configRegister(SETUP_AW, 0x01);        // 3 byte address
  configRegister(RF_CH, channel);             // channel 41
  configRegister(RF_SETUP, 0b00000110);  // 1Mbps
  configRegister(SETUP_RETR, 0x00);      // no retry
  writeRegister(RX_ADDR_P0, addr, 3);    // set receiving address
  configRegister(RX_PW_P0, 4);           // payload length of 4
  
  powerUpRx();  // set to receive
  flushRx();    // clear shit out just in case
}

// from adafruit neopixel lib
// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos, double scale) {
  WheelPos = 255 - WheelPos;
  
  if (WheelPos < 85) {
    return strip.Color(scale*(255 - WheelPos * 3), 0, scale*(WheelPos * 3));
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, scale*(WheelPos * 3), scale*(255 - WheelPos * 3));
  }
  WheelPos -= 170;
  return strip.Color(scale*(WheelPos * 3), scale*(255 - WheelPos * 3), 0);
}

void updateWheel() {
  uint16_t i;
  if( killLights ) killLights = 0;
  
  for (i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + wheelOffset) & 255, calcScale(i) ));
  }
  strip.show();
}

void updateChase() {
  uint16_t i;
  if( killLights ) killLights = 0;
  
  
  for (i = 0; i < strip.numPixels(); i++) {
    if( abs(i - wheelOffset % NUM_LEDS) < 5 ){
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + wheelOffset) & 255, calcScale(i) ));
    } else {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + wheelOffset) & 255, 0));
    }
  }
  strip.show();
}


void updateCollide() {
  uint16_t i;
  if( killLights ) killLights = 0;
  
  for (i = 0; i < strip.numPixels(); i++) {
    if( abs(i - wheelOffset % NUM_LEDS) < 3 ){
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + wheelOffset) & 255, calcScale(i) ));
    } else if( abs( (strip.numPixels()-i) - wheelOffset % NUM_LEDS) < 3 ) {
      strip.setPixelColor(i, Wheel((( (strip.numPixels()-i) * 256 / strip.numPixels()) + wheelOffset) & 255, calcScale(i) ));
    } else {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + wheelOffset) & 255, 0));
    }
  }
  
  strip.show();
}

void changeGain() {
  uint8_t inData;

  do {
    inData = pollClicker();
    
    if( inData == '6' )
      opGain = opGain >= OP_GAIN_MAX ? OP_GAIN_MAX : opGain+1;
    if( inData == '9' )
      opGain = opGain <= OP_GAIN_MIN ? OP_GAIN_MIN : opGain-1;

    visualizeMetric(opGain, OP_GAIN_MAX, strip.Color(0,0,100));
    
  } while( inData != '?' );
  
  opMode = prevMode;
}

void changeBrightness() {
  uint8_t inData;

  do {
    inData = pollClicker();
    
    if( inData == '6' )
      lightBrightness = lightBrightness >= BRIGHTNESS_MAX ? BRIGHTNESS_MAX : lightBrightness+20;
    if( inData == '9' )
      lightBrightness = lightBrightness <= BRIGHTNESS_MIN ? BRIGHTNESS_MIN : lightBrightness-20;

    visualizeMetric(lightBrightness, BRIGHTNESS_MAX, strip.Color(0,100,0));
    
    strip.setBrightness(lightBrightness);
    
  } while( inData != '?' );
  
  opMode = prevMode;
}

void lightsOff() {
  uint8_t i;
  
  if( killLights ) return;
  killLights = 1;
  for (i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(0,0,0));
  }
  strip.show();
}

void visualizeMetric(double metric, double maxVal, unsigned long color) {
  uint8_t i;
  
  for (i = 0; i < strip.numPixels(); i++) {
    if( i <= (float)(metric/maxVal)*strip.numPixels() )
      strip.setPixelColor(i, color);
    else 
      strip.setPixelColor(i, strip.Color(0,0,0));
  }
  strip.show();
}

void advanceColors() {
  wheelOffset += wheelIncrement;
  wheelOffset = wheelOffset % (256*5);
}

double calcScale(uint8_t index) {
  return ledScales[index]*(BASE_GAIN+(float)(opGain/10.0));
}

