//Libraries
  #include <Arduino.h>
  #include <OneWire.h>  //OneWire

  /* RFM69 library and code by Felix Rusu - felix@lowpowerlab.com
  Aquamon
  // **********************************************************************************/

  #include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
  #include <SPI.h>
  #include <stdio.h>
  #include <avr/dtostrf.h>


//Global variables anc constants
//*********************************************************************************************
// *********** IMPORTANT SETTINGS - YOU MUST CHANGE/ONFIGURE TO FIT YOUR HARDWARE *************
//*********************************************************************************************
#define NETWORKID     100  // The same on all nodes that talk to each other
#define NODEID        2    // The unique identifier of this node
#define RECEIVER      1    // The recipient of packets

//Match frequency to the hardware version of the radio on your Feather
#define FREQUENCY     RF69_433MHZ
#define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#define IS_RFM69HCW   true // set to 'true' if you are using an RFM69HCW module

//*********************************************************************************************
#define SERIAL_BAUD   115200

// for Feather M0 Radio
#define RFM69_CS      8
#define RFM69_IRQ     3
#define RFM69_IRQN    3  // Pin 3 is IRQ 3!
#define RFM69_RST     4

/*Feather m0 w/wing
#define RFM69_RST     11   // "A"
#define RFM69_CS      10   // "B"
#define RFM69_IRQ     6    // "D"
#define RFM69_IRQN    digitalPinToInterrupt(RFM69_IRQ )
*/

#define LED           13  // onboard blinky

int16_t packetnum = 0;  // packet counter, we increment per xmission

RFM69 radio = RFM69(RFM69_CS, RFM69_IRQ, IS_RFM69HCW, RFM69_IRQN);

// For Onewire
OneWire  ds(10);  // on pin 10 (a 4.7K resistor is necessary)

//For photoresistor
int lightPin = A1;


void setup() { // Setup, mostly radio stuff

  //  while (!Serial); // wait until serial console is open, remove if not tethered to computer. Delete this line on ESP8266
  Serial.begin(SERIAL_BAUD);

  Serial.println("Feather RFM69HCW Transmitter");

  // Hard Reset the RFM module
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH);
  delay(100);
  digitalWrite(RFM69_RST, LOW);
  delay(100);

  // Initialize radio
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
  if (IS_RFM69HCW) {
    radio.setHighPower();    // Only for RFM69HCW & HW!
  }
  radio.setPowerLevel(31); // power output ranges from 0 (5dBm) to 31 (20dBm)

  radio.encrypt(ENCRYPTKEY);

  pinMode(LED, OUTPUT);
  Serial.print("\nTransmitting at ");
  Serial.print(FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  Serial.println(" MHz");

}

void loop() {
Tempread();
}

void Tempread(){
  // <Insert> and transmit photoresistor read once trasmit radiopacket function is in place. analogRead(lightPin)
  // TEMPERATURE READS ON OneWire
    // Temperature variables.
    byte i;
    byte present = 0;
    byte type_s;
    byte data[12];
    byte addr[8];
    float celsius, fahrenheit;

    // Checks for the presence of OneWire probes and returns if none found.
    if ( !ds.search(addr)) {
      Serial.println("No more addresses.");
      Serial.println();
      ds.reset_search();
      delay(250); //Delay in cycles between temperature reads.
      return;
    }

    Serial.print("ROM =");
    String ROM ="ROM";
    for( i = 0; i < 8; i++) {
      Serial.write(' ');
      Serial.print(addr[i]); //Can add ,HEX if want to represent as HEX
      ROM = ROM+(addr[i]); //Assembles ROM string.
    }

    //No idea what this does!
    if (OneWire::crc8(addr, 7) != addr[7]) {
        Serial.println("CRC is not valid!");
        return;
    }

    Serial.println();

    // the first ROM byte indicates which chip
    switch (addr[0]) {
      case 0x10:
        Serial.println("  Chip = DS18S20");  // or old DS1820
        type_s = 1;
        break;
      case 0x28:
        Serial.println("  Chip = DS18B20");
        type_s = 0;
        break;
      case 0x22:
        Serial.println("  Chip = DS1822");
        type_s = 0;
        break;
      default:
        Serial.println("Device is not a DS18x20 family device.");
        return;
    }

    ds.reset();
    ds.select(addr);
    ds.write(0x44, 1);        // start conversion, with parasite power on at the end

    delay(1000);     // maybe 750ms is enough, maybe not
    // we might do a ds.depower() here, but the reset will take care of it.

    present = ds.reset();
    ds.select(addr);
    ds.write(0xBE);         // Read Scratchpad

    Serial.print("  Data = ");
    Serial.print(present, HEX);
    Serial.print(" ");
    for ( i = 0; i < 9; i++) {           // we need 9 bytes
      data[i] = ds.read();
      Serial.print(data[i], HEX);
      Serial.print(" ");
    }

    Serial.print(" CRC=");
    Serial.print(OneWire::crc8(data, 8), HEX);
    Serial.println();

    // Convert the data to actual temperature
    // because the result is a 16 bit signed integer, it should
    // be stored to an "int16_t" type, which is always 16 bits
    // even when compiled on a 32 bit processor.
    int16_t raw = (data[1] << 8) | data[0];
    if (type_s) {
      raw = raw << 3; // 9 bit resolution default
      if (data[7] == 0x10) {
        // "count remain" gives full 12 bit resolution
        raw = (raw & 0xFFF0) + 12 - data[6];
      }
    } else {
      byte cfg = (data[4] & 0x60);
      // at lower res, the low bits are undefined, so let's zero them
      if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
      else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
      else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
      //// default is 12 bit resolution, 750 ms conversion time
    }

    celsius = (float)raw / 16.0;
    fahrenheit = celsius * 1.8 + 32.0;
    Serial.print("  Temperature = ");
    Serial.print(celsius);
    Serial.print(" Celsius, ");
    Serial.print(fahrenheit);
    Serial.println(" Fahrenheit");

  //Building pre-radiopacket
    String radheader = "NetworkID=";
    radheader.concat(NETWORKID);
    radheader.concat("%NodeId=");
    radheader.concat(NODEID)+"%";
    String toradiopacket = radheader+"%Sensor="+ROM+"%"+"Data="+= celsius;

  //Building radiopacket
    int rpacketlen = toradiopacket.length()+1;
    char radiopacket[rpacketlen];
    toradiopacket.toCharArray(radiopacket,rpacketlen);

    Serial.println(radiopacket);

  //TRANSMISSION <-- <Turn this into a function.>
    delay(1000);  // Wait 1 second between transmits, could also 'sleep' here!

    Serial.print("Sending "); Serial.println(radiopacket);

    if (radio.sendWithRetry(RECEIVER, radiopacket, strlen(radiopacket))) { //target node Id, message as string or byte array, message length
      Serial.println("OK");
      Blink(LED, 50, 3); //blink LED 3 times, 50ms between blinks
    }

    radio.receiveDone(); //put radio in RX mode
    Serial.flush(); //make sure all serial data is clocked out before sleeping the MCU
  }

  //Blink function
  void Blink(byte PIN, byte DELAY_MS, byte loops)
  {
    for (byte i=0; i<loops; i++)
    {
      digitalWrite(PIN,HIGH);
      delay(DELAY_MS);
      digitalWrite(PIN,LOW);
      delay(DELAY_MS);
    }
}
