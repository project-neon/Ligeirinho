#include <Arduino.h>
#include <SPI.h>
#include "constants.h"

// Forward declare methods
bool Mouse_init(int pin);
void Mouse_begin();
void Mouse_end();
void Mouse_write(char reg_addr, char data);
uint8_t Mouse_read(char reg_addr);
bool Mouse_uploadFirmware();

// Initialize Mouse
bool Mouse_init(int PinCSN){
  MousePinCSN = PinCSN;

  // Check connection
  // Mouse_read(REG_Product_ID, )

  // Setup CSN pin as output
  pinMode(MousePinCSN, OUTPUT);

  // Ensure that the serial port is reset
  Mouse_end();
  Mouse_begin();
  Mouse_end();

  // Force reset
  Mouse_write(REG_Power_Up_Reset, 0x5a);

  // Wait for it to reboot
  delay(50);

  // Read registers 0x02 to 0x06 (and discard the data)
  Mouse_read(REG_Motion);
  Mouse_read(REG_Delta_X_L);
  Mouse_read(REG_Delta_X_H);
  Mouse_read(REG_Delta_Y_L);
  Mouse_read(REG_Delta_Y_H);

  // Upload the firmware
  if(!Mouse_uploadFirmware())
    return false;

  delay(10);

  // Enable laser(bit 0 = 0b), in normal mode (bits 3,2,1 = 000b)
  // reading the actual value of the register is important because the real
  // default value is different from what is said in the datasheet, and if you
  // change the reserved bytes (like by writing 0x00...) it would not work.
  byte laser_ctrl0 = Mouse_read(REG_LASER_CTRL0);
  Mouse_write(REG_LASER_CTRL0, laser_ctrl0 & 0xf0 );

  // Set to 400DPI                 //00010100
  Mouse_write(REG_Configuration_I, uint8_t(20));

  // Finish up setup
  delay(1);

  return true;
}

// Begin SPI Transaction
void Mouse_begin(){
  digitalWrite(MousePinCSN, LOW);
}

// End SPI Transaction
void Mouse_end(){
  digitalWrite(MousePinCSN, HIGH);
}

// Read register
uint8_t Mouse_read(char reg_addr){
  Mouse_begin();

  // Send adress of the register, with MSBit = 0 to indicate it's a read
  SPI.transfer(reg_addr & 0x7f );
  delayMicroseconds(100); // tSRAD
  // Read data
  uint8_t data = SPI.transfer(0);
  // tSCLK-NCS for read operation is 120ns
  delayMicroseconds(1);
  Mouse_end();
  // tSRW/tSRR (=20us) minus tSCLK-NCS
  delayMicroseconds(19);

  return data;
}

// Write register
void Mouse_write(char reg_addr, char data){
  Mouse_begin();

  // Send adress of the register, with MSBit = 1 to indicate it's a write
  SPI.transfer(reg_addr | 0x80 );
  SPI.transfer(data);
  // tSCLK-NCS for write operation
  delayMicroseconds(20);
  Mouse_end();
  // tSWW/tSWR (=120us) minus tSCLK-NCS.
  // Could be shortened, but is looks like a safe lower bound
  delayMicroseconds(100);
}

// Upload firmware to Mouse
bool Mouse_uploadFirmware(){
  // set the configuration_IV register in 3k firmware mode
  // bit 1 = 1 for 3k mode, other bits are reserved
  Mouse_write(REG_Configuration_IV, 0x02);
  // write 0x1d in SROM_enable reg for initializing
  Mouse_write(REG_SROM_Enable, 0x1d);
  // wait for more than one frame period
  // assume that the frame rate is as low as 100fps...
  // even if it should never be that low
  delay(10);
  // write 0x18 to SROM_enable to start SROM download
  Mouse_write(REG_SROM_Enable, 0x18);
  // write the SROM file (=firmware data)
  Mouse_begin();
  // write burst destination adress
  SPI.transfer(REG_SROM_Load_Burst | 0x80);
  delayMicroseconds(15);

  // send all bytes of the firmware
  unsigned char c;
  for(int i = 0; i < firmware_length; i++){
    c = (unsigned char)pgm_read_byte(firmware_data + i);
    SPI.transfer(c);
    delayMicroseconds(15);
  }
  Mouse_end();

  return true;
}

// Reads Motion
void Mouse_readXY(int &x, int &y){
  // Freeze motion
  uint8_t mot = Mouse_read(REG_Motion);

  //if(!(mot & 0b10000000))
    //return;

  // Reads X
  x = uint16_t(Mouse_read(REG_Delta_X_L)) | uint16_t(Mouse_read(REG_Delta_X_H) << 8);
  y = uint16_t(Mouse_read(REG_Delta_Y_L)) | uint16_t(Mouse_read(REG_Delta_Y_H) << 8);

}

int Mouse_readSqual() {
  return Mouse_read(REG_SQUAL);
}

// Prints useful stuff
void Mouse_debug(){
  static int oreg[7] = {
    REG_Product_ID,
    REG_Inverse_Product_ID,
    REG_SROM_ID,
    REG_Motion,
    REG_Configuration_I,
    REG_SQUAL
  };

  static char* oregname[] = {
    "Product_ID",
    "Inverse_Product_ID",
    "SROM_Version",
    "Motion",
    "DPI",
    "SQUAL",
  };

  byte regres;

  Mouse_begin();

  int rctr=0;
  for(rctr=0; rctr < sizeof(oregname) / sizeof(oregname[0]); rctr++){
    SPI.transfer(oreg[rctr]);
    delay(1);
    Serial.println("---");
    Serial.println(oregname[rctr]);
    Serial.println(oreg[rctr],HEX);
    regres = SPI.transfer(0);
    Serial.print(regres,HEX);
    Serial.write('\t');
    Serial.print(regres,BIN);
    Serial.write('\t');
    Serial.println(regres);
    delay(1);
  }

  Mouse_end();
}

void setup() {
  // Initialize Serial
  Serial.begin(115200);

  // Initialize SPI
  SPI.begin(SCK, MISO, MOSI, SS);
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3));
  // SPI.setDataMode(SPI_MODE3);
  // SPI.setBitOrder(MSBFIRST);
  // SPI.setClockDivider(8);
  pinMode(MISO, INPUT_PULLDOWN);
  pinMode(MOSI, OUTPUT);
  pinMode(SCK, OUTPUT);
  pinMode(SS, OUTPUT);

  // Initialize Pins
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  if(!Mouse_init(MousePinCSN)) {
    Serial.println("Deu erro para iniciar");
  } else {
    Serial.println("Deu certo para iniciar");
    digitalWrite(LED_BUILTIN, HIGH);
  }
  delay(2000); 

}

void loop() {
  // Read mouse motion
  int dx = 0;
  int dy = 0;
  Mouse_readXY(dx, dy);
  Serial.print(dx);
  Serial.print("   ");
  Serial.println(dy);
}