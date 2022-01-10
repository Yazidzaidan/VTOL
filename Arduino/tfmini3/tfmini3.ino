#include <SoftwareSerial.h>

SoftwareSerial SerialTFMini(2, 3); //The only value that matters here is the first one, 2, Rx

int dist;
int strength;
int check;
int uart[9];
int i; 
const int HEADER=0x59;


void setup() { 
  Serial.begin(115200);
  SerialTFMini.begin(115200);  
}

void sendData(int val){
  int toleransi = 10;
  while(1){
    if(SerialTFMini.available()){
      if(SerialTFMini.read()==HEADER){
        uart[0]=HEADER;
        if(SerialTFMini.read()==HEADER){
          uart[1]=HEADER;
          for(i=2;i<9;i++){
            uart[i]=SerialTFMini.read();      
          }
          check=uart[0]+uart[1]+uart[2]+uart[3]+uart[4]+uart[5]+uart[6]+uart[7];
          if(uart[8]==(check&0xff)){
            dist=uart[2]+uart[3]*256;
            strength=uart[4]+uart[5]*256;
//            Serial.println(dist);
            if (dist <= val && dist != -1){
              Serial.println('1');
              break; //keluar program, kalau udah nge deteksi
            } else{
              Serial.println('0');
            }
          }
        }
      }
    }
  }
}

void loop() {
  if (Serial.available() > 0) {
    char pyData = Serial.read();
//    Serial.print("pyData");
//    Serial.print(pyData);
    if (pyData == '0') { //B
      sendData(50); 
    } else if (pyData == '1'){ //AC
      sendData(80);
    } else if (pyData == '2'){ //Wall
      sendData(100);
    } else if (pyData == '3'){ //Wall
      sendData(50);
    }
    while (Serial.available() > 0){
      char pyData = Serial.read();
    }
    delay(500);
  }
}
