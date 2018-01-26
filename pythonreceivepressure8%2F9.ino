#include <Wire.h>

#include <Adafruit_MCP4725.h>

Adafruit_MCP4725 dac;
int readline(int readch, char *buffer, int len)
{
  static int pos = 0;
  int rpos;

  if (readch > 0) {
    switch (readch) {
      case '\n': // Ignore new-lines
        break;
      case ';': // Return on CR
        rpos = pos;
        pos = 0;  // Reset position index ready for next time
        return rpos;
      default:
        if (pos < len-1) {
          buffer[pos++] = readch;
          buffer[pos] = 0;
        }
    }
  }
  // No end of line has been found, so return -1.
  return -1;
}
/*
  ReadAnalogVoltage
  Reads an analog input on pin 0, converts it to voltage, and prints the result to the serial monitor.
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

 This example code is in the public domain.
 */
long PsensorValue;
long CH1Value;
long CH2Value;
long CH3Value;
//Measured value of the 5v line using a power adapter
float five_volt_line = 4.977;
float pressure_correction = + 0.8; //on AUG 9 2016
uint32_t arduino_value_of_voltage_12_bit = 0;


//#############################
float reference_three_volt = 3.261; ////////////////CHEEEEK mEEEEEEeeeEe
//#############################


// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);
//  analogWriteResolution(12);
  //analogReadResolution(12);
  dac.begin(0x62);
  //Serial.println("Mason 112 Pressure and voltage sensor\n### V1.1 ###\nData update rate = 1 Hz\nData adquisition rate = 100 Hz\nUnits Pressure = kPa\nUnits Voltage = mV\n Angle (degrees), P (kPa), Ch1 (mV), Ch2 (mV), Ch3 (mV)");
  //Serial.println("### V1.1 ###");
  //Serial.println("Data update rate = 1 Hz");
  //Serial.println("Data adquisition rate = 100 Hz");
  //Serial.println("Units Pressure = kPa");
  //Serial.println("Units Voltage = mV");
  //Serial.println(" Angle (degrees), P (kPa), Ch1 (mV), Ch2 (mV), Ch3 (mV)");
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
  PsensorValue = 0;
  CH1Value =0;
  CH2Value =0;
  CH3Value =0;
  for(int i = 0; i<10 ; i++){
    PsensorValue =PsensorValue+ analogRead(A0);
    CH1Value =CH1Value+ analogRead(A1);
    CH2Value =CH2Value+ analogRead(A2);
    //CH3Value =CH3Value+ analogRead(A3);
    //delay(1);
    }

  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  //equation according to datasheet for   
  float pressure = pressure_correction + (((((PsensorValue/10.0)/1023.0)) - 0.04)/(0.00369));
  //pressure=10.07*(pressure+0.51);
  float ch1_voltage = (CH1Value/10.0)/1023.0*five_volt_line;
 // ch1_voltage = ch1_voltage*2520;
  float ch2_voltage = (CH2Value/10.0)/1023.0*five_volt_line;
  //float ch3_voltage = (CH3Value/100.0)/1023.0*five_volt_line;
 // float ch2_voltage = (CH2Value);
  //float ch3_voltage = (CH3Value);
 // float current=ch3_voltage-ch2_voltage;

  // print out the value you read:
    static char buffer[80];

  if (readline(Serial.read(), buffer, 80) > 0) {
      float myvoltage = (buffer[0]-48) + (buffer[2]-48)*0.1 +(buffer[3]-48)*0.01 +(buffer[4]-48)*0.001 +(buffer[5]-48)*0.0001 +(buffer[6]-48)*0.0001 + (buffer[7]-48)*0.00001;
      Serial.print(" ");
      Serial.print(myvoltage);
      Serial.print(" , ");
    //Serial.print(buffer);
    //Serial.print("\t,\t");
      Serial.print(pressure);
      Serial.print(" , ");
      Serial.print(ch1_voltage);
      Serial.print(" , ");
    //Serial.print(current); 
    //  Serial.print(", ");
      Serial.print(ch2_voltage);
      Serial.print(" , ");
      //Serial.println(ch3_voltage);
      
      arduino_value_of_voltage_12_bit = int((myvoltage)* 4096.0/five_volt_line);
      Serial.print(arduino_value_of_voltage_12_bit);
      //analogWrite(DAC1, arduino_value_of_voltage_12_bit);
      dac.setVoltage(arduino_value_of_voltage_12_bit,false);
      Serial.print(";");
  }
}
