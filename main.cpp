#include "DHT11.h"
#include <DS3231.h>
#include <SPI.h>
#include <elapsedMillis.h>
//#include "WorkScheduler.h"
//#include "Timer.h"

//WorkScheduler *Update;
elapsedMillis timeElapsed;
//WorkScheduler *_preProcessing;

/*Dinh nhia chan cua DHT sensor*/
#define DHT_Pin 6
#define PIR_Pin 7

/*Dinh nghia mang cac so tu 0 - 9 cho viec hien thi gio*/
#define NUM0 {B01110000,B10101000,B10101000,B10101000,B10101000,B01110000,B00000000,B00000000,B00000000,B00000000}
#define NUM1 {B00100000,B01100000,B10100000,B00100000,B00100000,B01110000,B00000000,B00000000,B00000000,B00000000}
#define NUM2 {B01110000,B10001000,B00001000,B01110000,B10000000,B11111000,B00000000,B00000000,B00000000,B00000000}
#define NUM3 {B11110000,B00001000,B00001000,B01111000,B00001000,B11110000,B00000000,B00000000,B00000000,B00000000}
#define NUM4 {B10001000,B10001000,B10001000,B11111000,B00001000,B00001000,B00000000,B00000000,B00000000,B00000000}
#define NUM5 {B11111000,B10000000,B11110000,B00001000,B10001000,B01110000,B00000000,B00000000,B00000000,B00000000}
#define NUM6 {B11111000,B10000000,B11111000,B10001000,B10001000,B11111000,B00000000,B00000000,B00000000,B00000000}
#define NUM7 {B11111000,B00001000,B00001000,B01111000,B00001000,B00001000,B00000000,B00000000,B00000000,B00000000}
#define NUM8 {B11111000,B10001000,B11111000,B10001000,B10001000,B11111000,B00000000,B00000000,B00000000,B00000000}
#define NUM9 {B11111000,B10001000,B11111000,B00001000,B00001000,B11111000,B00000000,B00000000,B00000000,B00000000}
#define COLON{B00000000,B00100000,B00000000,B00000000,B00100000,B00000000,B00000000,B00000000,B00000000,B00000000}
#define EMPTY{B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000}

/*Dinh nghia cac so va ky hieu cho viec hien thi nhiet do va do am*/
#define num0{B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B11100000,B10100000,B11100000}
#define num1{B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B10000000,B10000000,B10000000}
#define num2{B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B01100000,B00100000,B00110000}
#define num3{B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B11110000,B00110000,B11110000}
#define num4{B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B10010000,B11110000,B00010000}
#define num5{B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B01100000,B01000000,B11000000}
#define num6{B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B11110000,B10000000,B11100000,B11100000}
#define num7{B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B11110000,B00010000,B00110000,B00010000}
#define num8{B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B01100000,B01100000,B01100000,B01100000}
#define num9{B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B01110000,B01110000,B00010000,B11110000}
#define CELIUS{B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B01000000,B00110000,B00100000,B00110000}
#define PERCENT{B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B10010000,B00100000,B01000000,B10010000}

#define NUM0{B01110000,B10011000,B10101000,B10101000,B11001000,B01110000,B00000000,B00000000,B00000000,B00000000}
#define NUM1{B00100000,B01100000,B10100000,B00100000,B00100000,B01110000,B00000000,B00000000,B00000000,B00000000}
#define NUM2{B01110000,B10001000,B00001000,B01110000,B10000000,B11111000,B00000000,B00000000,B00000000,B00000000}
#define NUM3{B11110000,B00001000,B00001000,B01111000,B00001000,B11110000,B00000000,B00000000,B00000000,B00000000}
#define NUM4{B10001000,B10001000,B10001000,B11111000,B00001000,B00001000,B00000000,B00000000,B00000000,B00000000}
#define NUM5{B11111000,B10000000,B11110000,B00001000,B10001000,B01110000,B00000000,B00000000,B00000000,B00000000}
#define NUM6{B11111000,B10000000,B11111000,B10001000,B10001000,B11111000,B00000000,B00000000,B00000000,B00000000}
#define NUM7{B11111000,B00001000,B00001000,B01111000,B00001000,B00001000,B00000000,B00000000,B00000000,B00000000}
#define NUM8{B11111000,B10001000,B11111000,B10001000,B10001000,B11111000,B00000000,B00000000,B00000000,B00000000}
#define NUM9{B11111000,B10001000,B11111000,B00001000,B00001000,B11111000,B00000000,B00000000,B00000000,B00000000}

#define BA {B01111110,B01000010,B01000010,B01000010,B01000010,B01111110,B01000010,B01000010,B01000010,B01000010}
#define BB {B01111110,B01000010,B01000010,B01000010,B01000010,B01111110,B01000010,B01000010,B01000010,B01111110}
#define BC {B01111110,B01000000,B01000000,B01000000,B01000000,B01000000,B01000000,B01000000,B01000000,B01111110}
#define BD {B11111110,B01000010,B01000010,B01000010,B01000010,B01000010,B01000010,B01000010,B01000010,B11111110}
#define BE {B01111110,B01000000,B01000000,B01000000,B01000000,B01111110,B01000000,B01000000,B01000000,B01111110}
#define BF {B01111110,B01000000,B01000000,B01000000,B01000000,B01111110,B01000000,B01000000,B01000000,B01000000}
#define BG {B00111100,B01000100,B01000000,B01000000,B01001100,B01000010,B01000010,B01000010,B01000010,B00111100}
#define BH {B01000010,B01000010,B01000010,B01000010,B01000010,B01111110,B01000010,B01000010,B01000010,B01000010}
#define BI {B01111110,B00011000,B00011000,B00011000,B00011000,B00011000,B00011000,B00011000,B00011000,B01111110}
#define BJ {B01111110,B00001000,B00001000,B00001000,B00001000,B00001000,B00001000,B00001000,B00001000,B01110000}
#define BK {B01000010,B01000100,B01001000,B01010000,B01100000,B01100000,B01010000,B01001000,B01000100,B01000010}
#define BL {B01000000,B01000000,B01000000,B01000000,B01000000,B01000000,B01000000,B01000000,B01000000,B01111110}
#define BM {B10000001,B11000011,B11000011,B10100101,B10100101,B10011001,B10000001,B10000001,B10000001,B10000001}
#define BN {B10000001,B10000001,B11000001,B10100001,B10010001,B10001001,B10000101,B10000011,B10000001,B10000001}
#define BO {B00111100,B01000010,B01000010,B01000010,B01000010,B01000010,B01000010,B01000010,B01000010,B00111100}
#define BP {B00111100,B01000010,B01000010,B01000010,B01000010,B01111100,B01000000,B01000000,B01000000,B01000000}
#define BQ {B01111000,B10000100,B10000100,B10000100,B10000100,B10000100,B10000100,B01111100,B00000100,B00000111}
#define BR {B00111000,B01000100,B01000100,B01000100,B01000100,B01111000,B01100000,B01010000,B01001000,B01000100}
#define BS {B00111110,B01000000,B01000000,B00100000,B00010000,B00001000,B00000100,B00000010,B00000010,B01111100}
#define BT {B11111110,B00010000,B00010000,B00010000,B00010000,B00010000,B00010000,B00010000,B00010000,B00010000}
#define BU {B01000010,B01000010,B01000010,B01000010,B01000010,B01000010,B01000010,B01000010,B01000010,B00111100}
#define BV {B10000010,B10000010,B10000010,B10000010,B10000010,B10000010,B10000010,B01000100,B00101000,B00010000}
#define BW {B10000001,B10000001,B10000001,B10000001,B10000001,B10011001,B10011001,B10100101,B11000011,B10000001}
#define BX {B00000000,B10000001,B01000010,B00100100,B00011000,B00011000,B00100100,B01000010,B10000001,B00000000}

unsigned int arrayNum[10][10] {
  NUM0, NUM1, NUM2, NUM3, NUM4, NUM5, NUM6, NUM7, NUM8, NUM9
};

unsigned int arraynum[10][10] {
  num0, num1, num2, num3, num4, num5, num6, num7, num8, num9
};

unsigned int celius[10] = CELIUS;
unsigned int percent[10] = PERCENT;
unsigned int colon[10] = COLON;
unsigned int empty[10] = EMPTY;

static unsigned int **arrayTemp;
static unsigned int **ptrHour;
static unsigned int **ptrMin;
static unsigned int **ptrTemp;
static unsigned int **ptrHumi;
static unsigned int temperature;
static unsigned int humidity;

/*Khoi tao doi tuong RTC*/
DS3231 rtc(SDA, SCL);

/*Khoi tao doi tuong DHT11*/
dht11 dht;

/*Khoi tao doi tuong SPI*/
SPISettings LEDSettings(16000000, MSBFIRST, SPI_MODE0);

/*Khai bao cac chan dieu khien IC 4017*/
const int _clock = 9;
const int _reset = 8;

int my_i;
int i = 0;
long scrolling_word[16];
int array_turn=0;
byte your_text[10][10]={NUM0,NUM1,NUM2,NUM3,NUM4,NUM5,NUM6,NUM7,NUM8,NUM9};
//static bool allocated = false;


/*Functions Prototype*/
void scan(unsigned int **arrayDisp);
unsigned int** seprateNum(int num, char c);
unsigned int** dataProcessing(void);
void dataTransfering(unsigned int data);
void dataDeleting(void);
void readEnvironment(void);
void preProcessing(void);
void turnOff();
byte make_word (long posistion,byte turn);
void finish_scroll(int delay_scroll);
void display_word(int loops,byte word_print[][10],int num_patterns,int delay_langth);
void allocateMemory();
void deallocateMemory();
void task1(void);
void task2(void);
void task3(void);
//void ptrFunc(void);
//void printTest();

void setup() {
  Serial.begin(115200);

  /*RTC init*/
  rtc.begin();
  rtc.setTime(12, 14, 60);
  rtc.setDate(8, 12, 2016);
  rtc.setDOW(THURSDAY);

  pinMode(_clock, OUTPUT);
  pinMode(_reset, OUTPUT);
  pinMode(PIR_Pin, INPUT);

  /*SPI init*/
  SPI.begin();

  /*Cap nhat nhiet do va do am ngay ban dau dung cho viec hien thi*/
  preProcessing();

  delay(1000*60);

  /*Timer::getInstance()->initialize();
  Update = new WorkScheduler(2000UL,printTest);*/
}


/***********************************************************/
void loop() {
  task1();
  task2();
  task3();
}
/***********************************************************/

void task1(){
  scan(dataProcessing());
}

void task2(){
  if(timeElapsed > 1000*10){
    turnOff();
    display_word(2,your_text,10,3);
    preProcessing();
    //delay(1000);
    timeElapsed = 0;
  }
}

void task3(){
  Time t;
  t = rtc.getTime();
  if( (t.hour>=0) && (t.hour<=5) ){
    
  }
}
void scan(unsigned int **arrayDisp) {
  SPI.beginTransaction(LEDSettings);
  for (my_i = 0; my_i < 3; my_i ++) {
    for (int j = 0; j < 10; j++) {
      /*Bat dau giao tiep SPI giua Master va Slave*/
      digitalWrite(SS, LOW); // SS is latch Pin of 74HC595

      /*Bat dau qua trinh truyen du lieu, du lieu duoc truyen tung byte mot*/
      dataTransfering(arrayDisp[my_i][j]);

      /*Ket thuc qua trinh truyen du lieu SPI VA
        day du lieu ra OUTPUT PIN cua IC 595*/
      digitalWrite(SS, HIGH);

      if (j == 0) {
        digitalWrite(_reset, HIGH);
        digitalWrite(_reset, LOW);
        goto a;
      }

      /*Active row current or through Ground*/
      digitalWrite(_clock, HIGH); // kich xung output cua IC 4017
      digitalWrite(_clock, LOW);

a: delayMicroseconds(350);

      /*Turn the row current off to preparing active next row*/
      dataDeleting();
      if (j == 9) {
        digitalWrite(_reset, HIGH);
        digitalWrite(_reset, LOW);
      }
    }

    /*Ket thuc qua trinh truyen nhan du lieu*/
    SPI.endTransaction();
  }

  /*Thu hoi vung nho cua mang hai chieu duoc cap phat dong cho viec hien thi*/
  deallocateMemory();
}

/*Ham nay co tac dung tach mot so co 2 chu so thanh 2 so rieng biet*/
unsigned int** seprateNum(int num, char c) {
  static unsigned int *ptrNumHour[2], *ptrNumMin[2], *ptrNumTemp[2], *ptrNumHumi[2], **NumHour, **NumMin, **NumTemp, **NumHumi;
  unsigned int temp[2];
  if (num >= 10) {
    byte j = 0;

    /*Neu input la gio*/
    if (c == 'h') {
      while (num != 0) {
        temp[j] = num % 10;
        (j == 0) ? ptrNumHour[j] = &temp[j] : ptrNumHour[j] = &temp[j]; // ptrNum[0] la hang don vi
        j++;                          //   ptrNum[1] la hang chuc
        num /= 10;
      }
      for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 10; j++) {
          if (*ptrNumHour[i] == j) ptrNumHour[i] = &arrayNum[j][0];
        }
      }
      NumHour = ptrNumHour;
      return NumHour;
    }

    /*Neu input la phut*/
    if (c == 'm') {
      while (num != 0) {
        temp[j] = num % 10;
        (j == 0) ? ptrNumMin[j] = &temp[j] : ptrNumMin[j] = &temp[j];
        j++;
        num /= 10;
      }
      for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 10; j++) {
          if (*ptrNumMin[i] == j) ptrNumMin[i] = &arrayNum[j][0];
        }
      }
      NumMin = ptrNumMin;
      return NumMin;
    }

    /*Neu input la nhiet do*/
    if (c == 't') {
      while (num != 0) {
        temp[j] = num % 10;
        (j == 0) ? ptrNumTemp[j] = &temp[j] : ptrNumTemp[j] = &temp[j];
        j++;
        num /= 10;
      }
      for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 10; j++) {
          if (*ptrNumTemp[i] == j) ptrNumTemp[i] = &arraynum[j][0];
        }
      }
      NumTemp = ptrNumTemp;
      return NumTemp;
    }

    /*Neu input la do am*/
    if (c == 'H') {
      while (num != 0) {
        temp[j] = num % 10;
        (j == 0) ? ptrNumHumi[j] = &temp[j] : ptrNumHumi[j] = &temp[j];
        j++;
        num /= 10;
      }
      for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 10; j++) {
          if (*ptrNumHumi[i] == j) ptrNumHumi[i] = &arraynum[j][0];
        }
      }
      NumHumi = ptrNumHumi;
      return NumHumi;
    }
  }
  else {
    if (c == 'h') {
      for (int i = 0; i < 10; i++) {
        if (num == i) {
          ptrNumHour[0] = &arrayNum[i][0];
          ptrNumHour[1] = empty;
        }
      }
      NumHour = ptrNumHour;
      return NumHour;
    }
    else {
      for (int i = 0; i < 10; i++) {
        if (num == i) {
          ptrNumMin[0] = &arrayNum[i][0];
          ptrNumMin[1] = &arrayNum[0][0];
        }
      }
      NumMin = ptrNumMin;
      return NumMin;
    }
  }
}

unsigned int** dataProcessing(void) {
  Time t;
  t = rtc.getTime();
  ptrHour = seprateNum((unsigned int)t.hour, 'h');
  ptrMin = seprateNum((unsigned int)t.min, 'm');

    /*Cap phat bo nho dong cho mang hai chieu dung de hien thi*/
    arrayTemp = new unsigned int *[3];
    for (int i = 0; i < 3; i++) {
      arrayTemp[i] = new unsigned int [10];
    }

    /*Xoa cac gia tri rac nam trong mang 2 chieu sau khi cap phat dong */
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 10; j++) {
        arrayTemp[i][j] = 0;
      }
    }

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 10; j++) {
      if (i == 0) {
        arrayTemp[i][j] |= *(*(ptrHour + 1) + j) | (*(*(ptrHour + 0) + j) >> 5) | *(*(ptrTemp + 1) + j) | (*(*(ptrTemp + 0) + j) >> 4);
      }
      else if (i == 1) {
        arrayTemp[i][j] |= (*(*(ptrHour + 0) + j) << 3) | (*(*(ptrMin + 1) + j) >> 6) | (colon[j] >> 1) | (celius[j]) | (*(*(ptrHumi + 1) + j) >> 4);
      }
      else {
        arrayTemp[i][j] |= (*(*(ptrMin + 1) + j) << 2) | (*(*(ptrMin + 0) + j) >> 3) | *(*(ptrHumi + 0) + j) | (percent[j] >> 4);
      }
    }
  }
  return arrayTemp;
}

void dataTransfering(unsigned int data) {
  for (int k = 0; k < 3; k++) {
    if (my_i == 0) {
      if (k == 0) {
        SPI.transfer(data);
      }
      else {
        SPI.transfer(0);
      }
    }
    else if (my_i == 1) {
      if (k == 1) {
        SPI.transfer(data);
      }
      else {
        SPI.transfer(0);
      }
    }
    else if (my_i == 2) {
      if (k == 2) {
        SPI.transfer(data);
      }
      else {
        SPI.transfer(0);
      }
    }
  }
}

void dataDeleting(void) {
  digitalWrite(SS, LOW);
  SPI.transfer(0);
  SPI.transfer(0);
  SPI.transfer(0);
  digitalWrite(SS, HIGH);
}

void readEnvironment() {
  bool _error = false;
  do
  {
    int returnValue = dht.read(DHT_Pin);
    switch (returnValue) {
      case DHTLIB_OK:
        _error = false;
        break;
      case DHTLIB_ERROR_CHECKSUM:
        _error = true;
        break;
      case DHTLIB_ERROR_TIMEOUT:
        _error = true;
        break;
    }
  }
  while (_error);
  temperature = dht.temperature;
  humidity = dht.humidity;
}

void preProcessing(){
  turnOff();
  readEnvironment();
  ptrTemp = seprateNum(temperature, 't');
  ptrHumi = seprateNum(humidity, 'H');
}

void turnOff(void){
  allocateMemory();
  for(int i=0; i<3; i++){
    for(int j=0; j<10; j++){
      arrayTemp[i][j] = 0;
    }
  }
  scan(arrayTemp);
}

byte make_word (long posistion,byte turn){
  byte dummy_word = 0;
  for(int q=0;q<8;q++){
    if(scrolling_word[turn] & (posistion<<q))
      dummy_word |= 0x01<<q;
  }
  return dummy_word;
}   

void finish_scroll(int delay_scroll){// this function is the same as the funcion above, it just finishing scrolling
  for (int n=0;n<42;n++){
        for(int h=0;h<10;h++)
          scrolling_word[h] = scrolling_word[h] << 1;
      for(int w=0;w<delay_scroll;w++){
        for(int k=0;k<10;k++){
          if(i == 10){
            digitalWrite(_reset,HIGH);
            digitalWrite(_reset,LOW);
            i = 0;
          }
           digitalWrite(SS,LOW);
          SPI.transfer(make_word(0x01000000,k));// sending the data
          SPI.transfer(make_word(0x00010000,k));
          SPI.transfer(make_word(0x00000100,k));
         
          digitalWrite(SS,HIGH);
          delayMicroseconds(800);//waiting a bit
          
          digitalWrite(SS,LOW);
          SPI.transfer(0);// clearing the data
          SPI.transfer(0);
          SPI.transfer(0);
          digitalWrite(SS,HIGH);

          digitalWrite(_clock,HIGH);//counting up with the 4017
          digitalWrite(_clock,LOW);
          i++;
        }
      }
    }
}

void display_word(int loops, byte word_print[][10], int num_patterns, int delay_langth){// this function displays your symbols

  /*Tat led matrix truoc khi hien thi hieu ung*/
  turnOff();
  i = 0;// resets the counter fot the 4017
  for(int g=0;g<10;g++)//resets the the long int where your word goes
    scrolling_word[g] = 0;
  for(int x=0;x<num_patterns;x++){//main loop, goes over your symbols
   // you will need to find a better way to make the symbols scroll my way is limited for 24 columns

   for(int r=0;r<10;r++)//puts the buildes the first symbol
      scrolling_word[r] |= word_print[x][r]; 
    for (int z=0;z<10;z++){//the sctolling action
        for(int p=0;p<10;p++)
          scrolling_word[p] = scrolling_word[p] << 1;
// end of the scrolling funcion
      for(int t=0;t<delay_langth;t++){// delay function, it just loops over the same display
        for(int y=0;y<10;y++){// scaning the display
          if(i == 10){// counting up to 6 with the 4017
            digitalWrite(_reset,HIGH);
            digitalWrite(_reset,LOW);
            i = 0;
          }
          digitalWrite(SS,LOW);
          SPI.transfer(make_word(0x01000000,y));// sending the data
          SPI.transfer(make_word(0x00010000,y));
          SPI.transfer(make_word(0x00000100,y));
         
          digitalWrite(SS,HIGH);
          delayMicroseconds(800);//waiting a bit
          
          digitalWrite(SS,LOW);
          SPI.transfer(0);// clearing the data
          SPI.transfer(0);
          SPI.transfer(0);
          digitalWrite(SS,HIGH);

          digitalWrite(_clock,HIGH);//counting up with the 4017
          digitalWrite(_clock,LOW);
          i++;
        }
      }
    }
  }
  finish_scroll(delay_langth);
}

void ptrFunc(void){
  display_word(1,your_text,10,3);
}

void allocateMemory(){
  arrayTemp = new unsigned int *[3];
  for(int i=0; i<3; i++){
    arrayTemp[i] = new unsigned int [10];
  }
  //allocated = true;
}

void deallocateMemory(){
  for(int i=0; i<3; i++){
    delete []arrayTemp[i];
  }
  delete []arrayTemp;
}

/*void printTest(){
  Serial.println("TEST");
}*/
