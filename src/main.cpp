


//
//El string de altitud con el filtro ya esta funcionandoy se esta enviando a Serial, aun no se envia el cliente via WiFi
//El string NMEA solo envia presion y Checksum, los otros valores no se envian, altura, vario y temperatura.
//Siguiente paso enviar el string a Putty, enviando a Putty persistente y en lineas separadas.
//Ya se puede ver el string a el Kobo, decidi usar el Software Serial, este elemino los mensajes de GPTXT Unknown message 58
//Uso de la libreria TinyGPS funciono!!!! Limpar el codigo comentado
//Aun esta trabajando conectado al router, sigue configurarlo como Access Point y mas carinitos
// el onbord LED prende titila cuando hay un cliente conectado
#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <SoftwareSerial.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_Sensor.h>
#include <TinyGPS.h>
// Connect the GPS TX (transmit) pin to Digital 16
// Connect the GPS RX (receive) pin to Digital 17

#define ONBOARD_LED  2  // Onboard LED prende cuando esta el cliente conectado

TinyGPS gps;
SoftwareSerial ss(16, 17);

Adafruit_BMP085 bmp; 

long Pressure = 0;                   //Pressure reading
const float p0 = 101325;             //pressure at sea level, used as reference on sensor initialization
#define SAMPLES_ARR 15               // part of signal filter
static long k[SAMPLES_ARR];         // part of signal filter
unsigned long get_time3= millis();   //Variables for String Creation
long Temperature = 0;

bool newData = false;
  
//const char *ssid = "TP-LINK_40E7"; 
const char *ssid = "ParaSensor"; 
const char *pw = "Bogota2017";
const int port = 6000;

// byte command; // fromsine wave code

WiFiServer server(port);
WiFiClient client;

uint8_t buf1[1024];
uint8_t i1=0;

uint8_t buf2[1024];
uint8_t i2=0;

String strNMEA;
String strChecksum;


IPAddress local_IP(192,168,4,22);
IPAddress gateway(192,168,4,9);
IPAddress subnet(255,255,255,0);

// Signal Smoothing Filter
static long Averaging_Filter(long input);
static long Averaging_Filter(long input)
{
  long sum=0;
//   for(int i=0; i<SAMPLES_ARR; i++){
//     k[i]=k[i+1];
//   }
  k[SAMPLES_ARR-1]=input;
  for(int i=0;i<SAMPLES_ARR;i++){
    sum+=k[i];
  }
  return (sum / SAMPLES_ARR );
}

//////////////////////////////////////////// Setup Section ///////////////////////////////////////////////////
void setup() {

 // set the data rate for the SoftwareSerial port
 Serial.begin(9600);
 ss.begin(9600);       // GPS Baud Rate
        
// Access Point Configuration
// https://arduino-esp8266.readthedocs.io/en/latest/esp8266wifi/soft-access-point-class.html 

  Serial.print("Setting soft-AP configuration ... ");
  Serial.println(WiFi.softAPConfig(local_IP, gateway, subnet) ? "Ready" : "Failed!");

  Serial.print("Setting soft-AP ... ");
  Serial.println(WiFi.softAP(ssid) ? "Ready" : "Failed!");

  Serial.print("Soft-AP SSID = ");
  Serial.println(ssid);
  
  Serial.print("Soft-AP IP address = ");
  Serial.println(WiFi.softAPIP());
  
  server.begin(); //start TCP Server

 // configuracion para conectarse via el router
 
// WiFi.mode(WIFI_STA);
//  WiFi.begin(ssid, pw); //Connect to wifi
// 
//  // Wait for connection  
//  Serial.println("Connecting to Wifi");
//  while (WiFi.status() != WL_CONNECTED) {   
//    delay(500);
//    Serial.print(".");
//    delay(500);
//  }
//
//  Serial.println("");
//  Serial.print("Connected to ");
//  Serial.println(ssid);
//
//  Serial.print("IP address: ");
//  Serial.println(WiFi.localIP());  
//  server.begin();
//  Serial.print("Open Telnet and connect to IP:");
//  Serial.print(WiFi.localIP());
//  Serial.print(" on port ");
//  Serial.println(port);

// Inicializacion del LED del controlador, este prende cuando hay un cliente conectado
pinMode(ONBOARD_LED,OUTPUT);
  // inicializacion del sensor de presion
  if (!bmp.begin()) {
  Serial.println("Could not find a valid BMP085 sensor, check wiring!");
  }
}
/////////////////////////////////////////////////////////////////////// VOID LOOP ///////////////////////////////////////////////
void loop() {

   
//TCP Connection Code
  WiFiClient client = server.available();

  if (client) {
    if(client.connected())
    {
      Serial.println("Client Connected");      
    }
    
    while(client.connected()){  

//////// mientras el cliente este conectado entonces encienda el led onboard         
      digitalWrite(ONBOARD_LED,HIGH);
      

////////////////// BMP Sensor NMEA Message creation////////////////
              Pressure= bmp.readPressure();                                    //get one sample from BMP085 in every loop
              long average_pressure = Averaging_Filter(Pressure);              //put it in filter and take average
              Temperature = bmp.readTemperature();
                        
              String str_out =     //combine all values and create part of NMEA data string output
                String("LK8EX1"+String(",")+String(average_pressure,DEC)+ String(",")+String("99999")+String(",")+
                String("9999")+String(",")+String(Temperature, DEC)+String(",")+String("999")+String(","));
                
            // Calculating checksum for data string
                unsigned int checksum_end,ai,bi;                                                
                for (checksum_end = 0, ai = 0; ai < str_out.length(); ai++)
                {
                  bi = (unsigned char)str_out[ai];
                  checksum_end ^= bi;
                  strChecksum = String(checksum_end,HEX);
                  strChecksum.toUpperCase();  
                } 
            // end of checksum calculation
               
            // creacion de un solo String con toda la fraese NMEA, para luego almacenarla en la Charr Array
                String strNMEA = '$' + str_out ;
                String str = strNMEA + '*' + strChecksum;
                int str_len = str.length() + 1; 
                char char_array[str_len];
                str.toCharArray(char_array, str_len);
                  for (int i = 0; i < (str_len)-1; i++) {   // imprimir el Array con todo la frase NMEA
                      Serial.print(char_array[i]);
                   }
                Serial.println();
              // Fin de String to Array
        client.write((char*)char_array, str_len);
        client.println();

 ////////////////// GPS Data //////////////////////////////////////
      
      unsigned long chars;
      unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
      for (unsigned long start = millis(); millis() - start < 1000;)
    {
      while (ss.available())
      {
        char c = ss.read();
        Serial.write(c); // uncomment this line if you want to see the GPS data flowing
        client.write(c);
        if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
      }
    }
    gps.stats(&chars, &sentences, &failed);
    if (chars == 0)
    Serial.println("** No characters received from GPS: check wiring **");
         
  }    // corchete del While. Client connected

    client.stop();
    Serial.println("Client disconnected"); 
    digitalWrite(ONBOARD_LED,LOW); //// apague el LED Onboard   


 }  // corchete if.client
}// fin de VOID Loop 
