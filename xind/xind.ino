#include <Adafruit_NeoPixel.h>
//led

#define dataPin   A3     //  SDI
#define clockPin  A2     //  SCLK
#define latchPin  A1     //  LOAD

#define PIN 12  //灯
Adafruit_NeoPixel strip = Adafruit_NeoPixel(14, PIN, NEO_GRB + NEO_KHZ800);
uint8_t rb[7][3] = { {255, 0, 0}, {255, 125, 0}, {255, 255, 0},
  {0, 255, 0}, {0, 155, 255}, {0, 0, 255}, {139, 0, 255}
};

#define CLK   A3
#define CS    A2
#define DIN   A1 //这里定义了那三个脚
#define SENSOR  A0  //心跳传感器 /震动传感器 ，输入用一个引脚，在输入处做判断。
#define ZHENGD  13

//#define ZHENGDONG A3 //震动传感器 输入就一个

unsigned char disp1[10][8] = {
  0x0, 0x66, 0xFF, 0xFF, 0xFF, 0x7E, 0x3C, 0x18, //最大心
  0x0, 0x0, 0x66, 0x7E, 0x7E, 0x3C, 0x18, 0x0,
  0x0, 0x0, 0x0, 0x24, 0x3C, 0x18, 0x0, 0x0,
  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,// 最小心
  0x0, 0x0, 0x0, 0x24, 0x3C, 0x18, 0x0, 0x0,
  0x0, 0x0, 0x66, 0x7E, 0x7E, 0x3C, 0x18, 0x0,
  0x0, 0x66, 0xFF, 0xFF, 0xFF, 0x7E, 0x3C, 0x18, //最大
  0x0, 0x8, 0xC, 0xFE, 0xFF, 0xFE, 0xC, 0x8,
  0x0, 0x10, 0x30, 0x7F, 0xFF, 0x7F, 0x30, 0x10,
  0x3C, 0x7E, 0xFF, 0xFF, 0xFF, 0xFF, 0x7E, 0x3C
};

void Delay(double x)
{
  double i;
  for (i = 0; i < x; i++);
}


void Delay_xms(unsigned int x)
{
  unsigned int i, j;
  for (i = 0; i < x; i++)
    for (j = 0; j < 112; j++);
}
//--------------------------------------------
//功能：向MAX7219(U3)写入字节
//入口参数：DATA
//出口参数：无
//说明：
void Write_Max7219_byte(unsigned char DATA)
{
  unsigned char i;
  digitalWrite(CS, LOW);
  for (i = 8; i >= 1; i--)
  {
    digitalWrite(CLK, LOW);
    if (DATA & 0X80)
      digitalWrite(DIN, HIGH);
    else
      digitalWrite(DIN, LOW);

    DATA <<= 1;
    digitalWrite(CLK, HIGH);
  }
}
//-------------------------------------------
//功能：向MAX7219写入数据
//入口参数：address、dat
//出口参数：无
//说明：
void Write_Max7219(unsigned char address, unsigned char dat)
{
  digitalWrite(CS, LOW);
  Write_Max7219_byte(address); //写入地址，即数码管编号
  Write_Max7219_byte(dat); //写入数据，即数码管显示数字
  digitalWrite(CS, HIGH);
}


void Init_MAX7219(void)
{
  Write_Max7219(0x09, 0x00); //译码方式：BCD码
  Write_Max7219(0x0a, 0x03); //亮度
  Write_Max7219(0x0b, 0x07); //扫描界限；4个数码管显示
  Write_Max7219(0x0c, 0x01); //掉电模式：0，普通模式：1
  Write_Max7219(0x0f, 0x00); //显示测试：1；测试结束，正常显示：0
}
/************************************************************/



void show(int n) {
  for (unsigned int i = 1; i < 9; i++)
    Write_Max7219(i, disp1[n][i - 1]);
}

int show_heart(int sens) {
  unsigned int j = sens / 100;
  int s = 0;
  switch (j) {
    case 10:
    case 9:
    case 8:
      show(0);
      s = 1;
      break;
    case 7:
      show(1);
      s = 1;
      break;
    case 6:
    case 5:
      show(2);
      break;
    case 4:
    case 3:
    case 2:
    case 1:
    case 0:
      show(3);
  }
  return s;
}

void readLine( ) {
  int gg = Serial.read( );

} // readLine(

void setup() {
  // put your setup code here, to run once:
  pinMode(CLK, OUTPUT);
  pinMode(CS, OUTPUT);
  pinMode(DIN, OUTPUT); //让三个脚都是输出状态
  pinMode(SENSOR, INPUT);
  pinMode(ZHENGD, OUTPUT);
  Init_MAX7219();
  Serial.begin(9600);
  strip.begin();
  strip.setBrightness(50);
  //hat_lb();
}

#define LEN   16
char buff [LEN];

void SerialRead(char *buf) {
  if ( Serial.available() > 0 ) {
    if (Serial.read() == 0xa) {
      int i = 0;
      char b = Serial.read();
      while (b != 0xd && i < LEN - 4) {
        buf[i++] = b;
        b = Serial.read();
      }
      buf[i] = 0x0;
      //Serial.println(buf);
    }
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  int sensorValue = analogRead(SENSOR);
  Serial.println(sensorValue);
  //SerialRead(buff);

  //int res = atoi(buff);
  int res = sensorValue;
  int s = show_heart(res);

  if (0)
    ;
  else
    hei();
  //lop();
  delay(50);
}

void bai() {
  for (int8_t i = 0; i <= 14; i++)
    strip.setPixelColor(i, strip.Color(255, 255, 255));
  strip.show();
}

void hei() {
  for (int8_t i = 0; i <= 14; i++)
    strip.setPixelColor(i, strip.Color(0, 0, 0));
  strip.show();
}

void hong() {
  for (int8_t i = 0; i <= 14; i++)
    strip.setPixelColor(i, strip.Color(255, 0, 0));
  strip.show();
}

void lv() {
  for (int8_t i = 0; i <= 14; i++)
    strip.setPixelColor(i, strip.Color(0, 255, 0));
  strip.show();
}

void lan() {
  for (int8_t i = 0; i <= 14; i++)
    strip.setPixelColor(i, strip.Color(0, 0, 255));
  strip.show();
}

uint64_t aaa = 0;
uint8_t j = 0, ii = 0;
void lop() {
  aaa++;
  //if (aaa % 1 != 0) return;
  if (j > 14) j = 0;
  //for (uint8_t j = 0; j < 14; j++) {
  //for (uint8_t k = 0; k < 14; k++)
  //strip.setPixelColor(k, 0);
  for (int8_t i = 0; i <= j; i++)
    strip.setPixelColor(i, strip.Color(rb[(j - i) % 7][0], rb[(j - i) % 7][1], rb[(j - i) % 7][2]));
  strip.show();
  //delay(20);
  j++;
  //}
}


