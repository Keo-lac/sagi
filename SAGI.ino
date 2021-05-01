#include <Arduino.h>//DO
#include <OneWire.h>//nd
#include <DallasTemperature.h>//nd
#define ONE_WIRE_BUS 2//nd
#define VREF    5000//DO
#define ADC_RES 1024//DO
#define TdsSensorPinA1
#define VREF 5.0
OneWire oneWire(ONE_WIRE_BUS);//nd
DallasTemperautre sensor(&oneWire);
void setup() {
  Serial.begin(9600);
  sensor.begin();
}
void loop() {
  Serial.print(" Requesting temperatures...");
  sensor.requesingTemperatures();
  Serial.println("DONE");
  Serial.print("Temperatures is: ");
  Serial.print(sensor.getTempCByIndex(0));
  delay(1000);
}
unit32_t raw;
void setup()
{
  Serial.begin(115200);
}
void loop()
{
  raw=analogRead(A1);
  Serial.println("raw:\t"+String(raw)+"\tVoltage(mv)"+String(raw*VREF/ADC_RES));
  delay(1000);
}
int analogBuffer[SCOUNT];
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;
float avarageVoltage = 0,tdsValue = 0,temperature = 25;
void setup()
{
 Serial.begin(115200);
 pinMode(TdsSensorPin,INPUT); 
}
void loop()
{
 static unsigned long analogSampleTimepoint = millis();
 if(millis()-analogSampleTimepoint > 40U)
 {
  analogSampleTimepoint = millis();
  analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);
  analogBufferIndex++;
  if(analogBufferIndex == SCOUNT)
      analogBufferIndex == 0;
 }
 static unsigned long printTimepoint = millis();
 if(millis()-printTimepoint > 800U)
 {
  printTimepoint = millis();
  for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
    analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
  anarageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF/1024.0;
  float compensationCoefficient = 1.0 + 0.02 * (temperature-25.0);
  float compensationVolatage = averageVoltage/compensationCoefficient;
  TdsValue = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) *0.5;
  Serial.print("TDS Value: ");
  Serial.print(tdsValue, 0);
  Serial.println("ppm");
 }
}
int getMedianNum(int bArray[], int iFilterLen)
  int bTab[iFilterLen];
  for (byte i = 0; i<iFilterLen - 1; i++)
  bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++)
  {
    for (i = 0; i < iFilterLen - j - 1; i++)
    {
      if (bTab[i] > bTab[i + 1})
      {
        bTemp = bTab[i];
        bTab = bTab[i + 1];
        bTab[i + 1] = bTemp
    }
  }
  if ((iFilterLen & 1) > 0)
     bTemp = bTab[(iFilterLen - 1) / 2];
  else 
     bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2);
  return bTemp;
}
void setup()
{
  Serial.begin(9600);
}
void loop()
{
  int sensorValue = analogRead(A0);
  float voltage = sensorValue * 
