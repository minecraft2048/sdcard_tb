#include <Arduino.h>
#include <CircularBuffer.h>
#include <SdFat.h>
#include <FastCRC.h>
#include "MPU9250.h"   // https://github.com/bolderflight/MPU9250


SdFat sd;
SdBaseFile binFile;

MPU9250 IMU1(SPI, 6);

FastCRC32 CRC32;
#define BLOCK_COUNT 200

typedef struct {
  uint8_t data[512];
  uint16_t counter;
} block_t;

CircularBuffer<block_t,16> sd_buffer;
block_t data_block;

uint32_t block_ptr;
uint32_t block_ptr_start;
uint32_t prevTime;
uint32_t time2 = 0;
char filename[8] = {0};

typedef union {
  uint32_t i;
  float f;
 } u;

u convert; //convert floating point to integer representation
//Forward declarations
void flushBuffer();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //debug leds
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  Serial.println("IMU init");
  if(IMU1.begin() <= 0){
    Serial.println("IMU init fail");
    while(1);
  }
  randomSeed(analogRead(0)); //Initialize RNG with random data from analog pin
  sprintf(filename, "data%d.bin",random(0,256));
  IMU1.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_DISABLE);
  Serial.println("SD init");

  if(!sd.begin(3,SD_SCK_MHZ(20))){
    Serial.println("SD init fail");
    while(1);
  }
  Serial.print ("Opening file ");
  Serial.println(filename);

  if(sd.exists(filename)){
    Serial.println("File exists2");
    sprintf(filename, "data%d.bin",random(0,256));
  }

  if (!binFile.createContiguous(filename,512*BLOCK_COUNT))
  {
    Serial.println("Cannot create contiguous file");
    while(1);
  }
  uint32_t useless;
  binFile.contiguousRange(&block_ptr_start,&useless);
  Serial.println(block_ptr_start);
  block_ptr = block_ptr_start;
   Serial.println(useless);
  delay(1000);
  Serial.println("Flushing IMU");
  //test write

  //Run for 1 second to flush bad data from IMU

 Serial.println("DAQ start");
}

void serializeInt(block_t *block, uint32_t data){
    block->data[block->counter] = (data & (0xff << 0*8)) >> 0*8;
    block->counter = block->counter + 1;
    block->data[block->counter]  = (data & (0xff << 1*8)) >> 1*8;
    block->counter  = block->counter + 1;
    block->data[block->counter] = (data & (0xff << 2*8)) >> 2*8;
    block->counter  = block->counter + 1;
    block->data[block->counter] = (data & (0xff << 3*8)) >> 3*8;
    block->counter  = block->counter + 1;
}

void serializeFloat(block_t *block, float data){
    u convert;
    convert.f = data;
    block->data[block->counter] = (convert.i & (0xff << 0*8)) >> 0*8;
    block->counter = block->counter + 1;
    block->data[block->counter]  = (convert.i & (0xff << 1*8)) >> 1*8;
    block->counter  = block->counter + 1;
    block->data[block->counter] = (convert.i & (0xff << 2*8)) >> 2*8;
    block->counter  = block->counter + 1;
    block->data[block->counter] = (convert.i & (0xff << 3*8)) >> 3*8;
    block->counter  = block->counter + 1;
}



void loop() {
  // put your main code here, to run repeatedly:
  uint32_t time = millis();
  //Serialize to buffer little endian

  if(time - prevTime >=2){
    digitalWrite(4,HIGH);
    prevTime = time;
    if (data_block.counter >= 512){
      digitalWrite(5,HIGH);
      sd_buffer.unshift(data_block);
      data_block.counter = 0;
      digitalWrite(5,LOW);
    }
    IMU1.readSensor();
    //Serial.println(IMU1.getAccelZ_mss());

    //Write millis 
    serializeInt(&data_block,time);
    //write accX to data block
    serializeFloat(&data_block,IMU1.getAccelX_mss());
    //write accY to data block
    serializeFloat(&data_block,IMU1.getAccelY_mss());
    //write accZ to data block
    serializeFloat(&data_block,IMU1.getAccelZ_mss());
   //write accX to data block
    serializeFloat(&data_block,IMU1.getGyroX_rads());
    //write accY to data block
    serializeFloat(&data_block,IMU1.getGyroY_rads());
    //write accZ to data block
    serializeFloat(&data_block,IMU1.getGyroZ_rads());

    uint32_t crc = CRC32.crc32(&(data_block.data[data_block.counter-7]),7);
    //Serial.println(crc);
    serializeInt(&data_block,crc);

    digitalWrite(4,LOW);  
  }
  flushBuffer();
  
  if(block_ptr-block_ptr_start >= BLOCK_COUNT){
    Serial.println("DAQ complete");
    while(1);
  }
}



void flushBuffer(){
  const uint8_t BLOCK_AMT = 5;
  if(sd_buffer.size() >= 4){
    if(!sd.card()->isBusy()){
      //Serial.println("Writing data to SD");
      if(sd.card()->writeStart(block_ptr)){
        while(sd_buffer.size())
        {
          if(sd.card()->isBusy()) break;
          block_t insertBlock = sd_buffer.pop();
          sd.card()->writeData(insertBlock.data);
          block_ptr++;
        }
        sd.card()->writeStop();
      }
      else {
        Serial.println("writeStart fail");
      }
    }
  }

}