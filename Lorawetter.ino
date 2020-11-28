// Test for ESP32;
// sending fix String; Heltec WiFi LoRa 32
// NEW: temp format corrected for always sending 3 digits
// NEW: blue LED is lightning while sending

///////////////////////////////////////////////////////////

long myTimer = 0;
long myTimeout = 60; // ON-AIR interval 60 seconds

const byte lora_PNSS = 18;   // pin number where the NSS line for the LoRa device is connected.
const byte lora_PReset = 14; // pin where LoRa device reset line is connected
const byte PLED1 = 2;        // pin number for LED on Tracker
const uint8_t gaugePin = 36; // the number of the gauge pin
#define richtungpin 34       // analog pin Windrichtung

String Outstring = "";

#include "LoRaTX.h" // Libraries from DJ7OO
#include "RTClib.h"
#include <Adafruit_BME280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <Wire.h>

// DISPLAY address
#define SSD1306_ADDRESS 0x3C
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 16

// Wind
int anopin = 12;    //PIN for anemometer input
int wind_m_sec = 3; //Wind measure time (seconds)
float wind;
int wind_cnt;
int wind_m;

// Regen
byte pinState = LOW;
volatile unsigned long IRQcount = 0;
#define RAIN_FACTOR 0.4
float result;
double getRain(unsigned long readings)
{
    return (double)readings * RAIN_FACTOR;
}
int dailyRain = 0;  // rain accumulated for the day
int hourlyRain = 0; // rain accumulated for one hour
int dailyRain_till_LastHour = 0;
bool first;

// Windrichtung
double uges = 0;
double vges = 0;
double u = 0;
double v = 0;
double uabs = 0;
double vabs = 0;
double uvarc = 0;
double winkel = 0;
int zaehlerrichtung = 0;
int windrichtung = 0;
const int messZeit = 30;
double pi = 3.14;
int spannung;

// BME280
#define SEALEVELPRESSURE_HPA (1019)
Adafruit_BME280 bme;
float SLpressure_hPa;
float temp, tempf, humi, alti, pres;
int cnt = 1;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

RTC_Millis rtc;

/////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
    Serial.begin(9600); // Serial console ouput
    //delay(3000);
    Wire.begin(4, 15);
    Serial.println("Ready!!!");
    rtc.begin(DateTime(__DATE__, __TIME__));
    pinMode(gaugePin, INPUT_PULLUP);
    pinMode(richtungpin, INPUT);

    // Attach an interrupt to the ISR vector
    attachInterrupt(36, pin_ISR, FALLING);

    bool bme_status;
    bme_status = bme.begin(0x76); //address either 0x76 or 0x77
    if (!bme_status)
    {
        Serial.print("No valid BME280 found");
    }

    display.begin(SSD1306_SWITCHCAPVCC, 0x3c);
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println("  LoRa-WX ");
    display.setCursor(0, 20);
    display.println("  Encoder");
    display.setTextSize(1);
    display.setCursor(25, 55);
    display.println("(C)OE7MFI 2020");
    display.display();
    delay(3000);

    pinMode(lora_PReset, OUTPUT);   // RFM98 reset line
    digitalWrite(lora_PReset, LOW); // Reset RFM98
    pinMode(lora_PNSS, OUTPUT);     // set the slaveSelectPin as an output:
    digitalWrite(lora_PNSS, HIGH);
    pinMode(PLED1, OUTPUT); // for shield LED

    SPI.begin(); // initialize SPI:
    SPI.setClockDivider(SPI_CLOCK_DIV2);
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);

    lora_ResetDev(); // Reset the device
    lora_Setup();    // Do the initial LoRa Setup

    // LoRa frequency calculation (sample for 434.4 MHz):
    // ------------------------------------
    // 434400000/61.03515625 = 71172096
    // 71172096 (DEC) = 6C 99 99 (HEX)
    // 6C 99 99 (HEX) = 108 153 153 (DEC)

    lora_SetFreq(108, 113, 153); //433.775 MHz                                                    // Set the LoRa frequency, 434.400 Mhz
                                 // lora_SetFreq(108, 153, 153);  //434.400 MHz                                                  // Set the LoRa frequency, 434.400 Mhz
}

///////////////////////////////////////////////////////////////////////////////////////
void loop()
{
    get_sensor_data();
    bme.takeForcedMeasurement();
    temp = bme.readTemperature();
    humi = bme.readHumidity();
    alti = bme.readAltitude(SEALEVELPRESSURE_HPA);
    pres = bme.readPressure() / 100.0F;

    do // Die Schleiffe Zählt die angegebene Messzeit durch
    {

        windrose(analogRead(richtungpin)); // Einlesen des Analogwert der Windrichtung und übergabe an die Funktion. 1x Pro Sekunde
    } while (zaehlerrichtung >= messZeit);

    u = uges / zaehlerrichtung; // Durchschnitt errechnung der Vektoren
    uges = 0;
    v = vges / zaehlerrichtung;
    vges = 0;

    uabs = abs(u); // Zurückrechnung der Vektoren in ein Winkel
    vabs = abs(v);
    uvarc = uabs / vabs;
    winkel = atan(uvarc);
    winkel = winkel * 180 / pi;
    if (u >= 0 && v >= 0)
        windrichtung = winkel;
    else if (u >= 0 && v < 0)
        windrichtung = 180 - winkel;
    else if (u < 0 && v >= 0)
        windrichtung = 360 - winkel;
    else if (u < 0 && v < 0)
        windrichtung = 180 + winkel;
    zaehlerrichtung = 0;

    spannung = analogRead(13);

    cli(); //disable interrupts
    double result = getRain(IRQcount);
    sei(); //enable interrupts

    dailyRain = result;

    /////////////////////// Hier wird Der stündliche Regen und Regen Seit Mitternacht zurückgesetzt///////////
    DateTime now = rtc.now();

    if (now.minute() == 0)
    {

        hourlyRain = dailyRain - dailyRain_till_LastHour; // calculate the last hour's rain
        dailyRain_till_LastHour = dailyRain;              // update the rain till last hour for next calculation
    }
    if (now.hour() == 0)
    {
        dailyRain = 0;               // clear daily-rain at midnight
        dailyRain_till_LastHour = 0; // we do not want negative rain at 01:00
    }

    byte i;
    byte ltemp;
    tempf = (temp * 1.8) + 32; //celsius to fahrenheit

    /////////////////////////////// Aprs String ////////////////////////////////////////////
    Outstring = "OE7MFI-7>APRS:!4731.41N/01234.62E"; //Hier Rufzeichen Eintragen
    Outstring += ("_");
    if (windrichtung < 100)
    {
        Outstring += "0";
    }
    if (windrichtung < 10)
    {
        Outstring += "0";
    }
    Outstring += String(windrichtung); // Windrichtung
    Outstring += "/";
    if (wind_m < 100)
    {
        Outstring += "0";
    }
    if (wind_m < 10)
    {
        Outstring += "0";
    }
    Outstring += String(wind_m); // Windgeschwindigkeit
    Outstring += ("g");
    if (wind_m < 100)
    {
        Outstring += "0";
    }
    if (wind_m < 10)
    {
        Outstring += "0";
    }
    Outstring += String(wind_m); // Wind Böhen nicht konfiguriert
    Outstring += String("t");
    if (tempf < 100)
    {
        Outstring += "0";
    }
    if (tempf < 10)
    {
        Outstring += "0";
    }
    Outstring += String(tempf, 0); //Temperatur
    Outstring += String("");
    Outstring += String("r"); // Stündlicher Regen
    if (hourlyRain < 100)
    {
        Outstring += "0";
    }
    if (hourlyRain < 10)
    {
        Outstring += "0";
    }
    Outstring += String(hourlyRain);
    Outstring += String("p000");
    Outstring += String("P"); // Regen Seit Mitternacht
    if (dailyRain < 100)
    {
        Outstring += "0";
    }
    if (dailyRain < 10)
    {
        Outstring += "0";
    }
    Outstring += String(dailyRain);
    Outstring += String("h");
    Outstring += String(humi, 0);
    Outstring += ("b");
    if (pres < 1000)
    {
        Outstring += "0";
    }
    Outstring += String(pres, 0);
    Outstring += ("0 A=");
    Outstring += String(alti, 0);
    Outstring += ("m ");
    Outstring +=("Batterie Spannung = ");
    Outstring +=(spannung);
    Outstring +=(" mV");

    ////////////////////////////////////////////////////////////////////////////////////////

    // Serial.print("sending: ");
    // Serial.println(Outstring);

    ltemp = Outstring.length();
    lora_SetModem(lora_BW125, lora_SF12, lora_CR4_5, lora_Explicit, lora_LowDoptON); // Setup the LoRa modem parameters
    lora_PrintModem();                                                               // Print the modem parameters
    lora_TXStart = 0;
    lora_TXEnd = 0;
    for (i = 0; i <= ltemp; i++)
    {
        lora_TXBUFF[i] = Outstring.charAt(i);
    }
    i--;
    lora_TXEnd = i;
    digitalWrite(PLED1, HIGH); // LED ON on during sending

    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(3);
    display.setCursor(18, 10);
    display.print("ON AIR");
    display.setTextSize(2);
    display.setCursor(50, 40);
    if (cnt < 1000)
    {
        display.print("0");
    }
    if (cnt < 100)
    {
        display.print("0");
    }
    if (cnt < 10)
    {
        display.print("0");
    }
    display.print(cnt);
    display.display();

    lora_Send(lora_TXStart, lora_TXEnd, 60, 255, 1, 10, 17); //Letzter wert (17) ist die leistung in dp
    digitalWrite(PLED1, LOW); // LED OFF after sending
    lora_TXBuffPrint(0);
    make_display();
    cnt = cnt + 1;
    if (cnt > 9999)
    {
        cnt = 0;
    }
    //////////////////Einstelung TX-Delay/////////////////////////////////
    int z, calc;
    calc = 3600; // 1800sec. = 30 Min.
               //   calc = 900;   // 900sec.  = 15 Min.
               //   calc = 600;   // 600sec.  = 10 Min.
               //   calc = 300;   // 300sec.  = 5 Min.
               //   calc = 60;    //  60sec.  = 1 Min.
               //   calc = 30;    //  60sec.  = 30 Sec.

    Serial.println(" ");
    Serial.print("TX-Delay: "),
        Serial.print(calc),
        Serial.println("sec.");
    for (z = 1; z < (calc); z++)
    {
        delay(1000);
    }
}
void get_sensor_data()
{
    meassure_wind();
}

/////////////////////////////////auslesen und umrechnen Windgeschwindigkeit //////////////////////////////

void meassure_wind()
{
    wind_cnt = 0;
    attachInterrupt(digitalPinToInterrupt(anopin), intrrupt_cnt, RISING);
    delay(wind_m_sec * 1000); //Time for measure counts
    detachInterrupt(digitalPinToInterrupt(anopin));
    wind = ((float)wind_cnt / (float)wind_m_sec * 2.4) / 2; //Convert counts & time to km/h
    wind_m = (wind / 3.6);
}
void intrrupt_cnt()
{               //On count detected
    wind_cnt++; //Add one count
}

///////////////////////////////////Für die Regen Wippe verzögerung das es nicht zu doppel kommt/////////////////

void pin_ISR()
{
    static unsigned long last_interrupt_time = 0;
    unsigned long debounce_time = millis();
    // If interrupts come faster than 100ms, assume it's a bounce and ignore
    if (debounce_time - last_interrupt_time > 1500)
    {
        IRQcount++;
        pinState = digitalRead(gaugePin);
    }
    last_interrupt_time = millis();
}

///////////////////Windrichtung Einstellung Richtung mV///////////////////////////////////

void windrose(int w) // Hier wird der Analoge eingelesene Wert in ein Winkel Umgewandelt
                     // und Zusammengezählt als Vektoren
{
    if (w > 700 && w < 1000)
        winkel = 0;
    else if (w > 1970 && w < 2032)
        winkel = 22.5;
    else if (w > 2800 && w < 3753)
        winkel = 45;
    else if (w > 3754 && w < 3793)
        winkel = 67.5;
    else if (w > 3794 && w < 3867)
        winkel = 90;
    else if (w > 3868 && w < 3967)
        winkel = 112.5;
    else if (w > 2700 && w < 3200)
        winkel = 135;
    else if (w > 3201 && w < 3699)
        winkel = 157.5;
    else if (w > 2700 && w < 2750)
        winkel = 180;
    else if (w > 1300 && w < 2700)
        winkel = 202.5;
    else if (w > 1299 && w < 1535)
        winkel = 225;
    else if (w > 1300 && w < 1400)
        winkel = 247.5;
    else if (w > 100 && w < 150)
        winkel = 270;
    else if (w > 151 && w < 200)
        winkel = 292.5;
    else if (w > 201 && w < 300)
        winkel = 315;
    else if (w > 301 && w < 700)
        winkel = 337.5;
    uges = uges + (sin(winkel * pi / 180));
    vges = vges + (cos(winkel * pi / 180));
    zaehlerrichtung++;
}

/////////////////////////////////////////////Ausgabe serial Monitor///////////////////////////////
void make_display()
{
    Serial.println(" ");
    Serial.print("Temperature             = ");
    Serial.print(temp, 1);
    Serial.println("°C");
    Serial.print("Humidiy                 = ");
    Serial.print(humi, 0);
    Serial.println("%");
    Serial.print("Pressure                = ");
    Serial.print(pres, 0);
    Serial.println("hPa");
    Serial.print("Altitude                = ");
    Serial.print(alti, 0);
    Serial.println("m");
    Serial.print("Wind                    = ");
    Serial.print(wind);
    Serial.println("km/h");
    Serial.print("Niederschlag 24h        = ");
    Serial.print(dailyRain);
    Serial.println("mm");
    Serial.print("Niederschlag Letzte 1h  = ");
    Serial.print(hourlyRain);
    Serial.println("mm");
    Serial.print("Windrichtung            = ");
    Serial.print(windrichtung);
    Serial.println(" grad");
    Serial.print("Spannung Akku           = ");
    Serial.print(spannung);
    Serial.println(" milliVolt");

    /////////////////////// Ausgabe Display/////////////////////////////////////////

    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print("Temp:");
    display.print(temp, 1);
    display.println("C");
    display.setCursor(0, 16);
    display.print("Humi:");
    display.print(humi, 0);
    display.println("%");
    display.setCursor(0, 32);
    display.print("Wind:");
    display.print(wind, 0);
    display.println("km/h");
    display.setCursor(0, 48);
    display.print("Prs:");
    display.print(pres, 0);
    display.println("hPa");
    display.display();
}