#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SH1106.h>
#include "INA226.h"

INA226 INA(0x40);

#define TIMER1_PRELOADING 45535

// 1m
#define DISPLAY_TIMEOUT 6000

#define BAT_LEVEL_MIN 14.0
#define BAT_LEVEL_MAX 20.5
#define BAT_RESOLVE_MIN 12.0
#define BAT_RESOLVE_MAX 22.0

#define BAT1_CAPACITY 5.461
#define BAT2_CAPACITY 5.709
#define BAT3_CAPACITY 5.389

#define K 0.88

#define LED_PIN 13
#define BUT_PIN 2
#define BUZZ_PIN 3

#define SCR_W 128
#define SCR_H 64
#define BAT_W 13
#define BAT_H 7

#define HREF  1.1

#define OLED_RESET 4
Adafruit_SH1106 display(OLED_RESET);

uint8_t beep_type = 0;
uint8_t beep_ticks = 0;

// input voltage
float vin_prev = -24;
float vin = 0;
// battery charger voltage
float vc1 = 0;
float vc2 = 0;
float vc3 = 0;
// resolved battery voltage
float vb1 = 0;
float vb2 = 0;
float vb3 = 0;
// battery charging flag
uint8_t bc1 = 0;
uint8_t bc2 = 0;
uint8_t bc3 = 0;
// out calculation
float bus = 0;
float cur = 0;
float pwr = 0;
float pwr_prev = -100;
// current screen
uint8_t screen = 1;
int screen_timeout = 0;

uint8_t but_ticks = 0;

uint8_t blink_10 = 0;
uint8_t blink_2 = 0;

float bat_level(float vb);

/**
 *  0 - toggle 1 or 2 screen
 *  1 - display screen 1
 *  2 - display screen 2
 *  3 - display screen 3
 * >3 - only update timeout
 */
void display_screen(uint8_t scr);

/**
 * Beep type:
 *  1 - single
 *  2 - double
 *  3 - long beep
 */ 
void beep(uint8_t type);

void render();

void resolveBatVoltage(float vc, float* vbp, uint8_t* bcp);



// void printBits(Print& pr, uint8_t value) {
//   for(int i = 7; i>= 0; i--) {
//     pr.print(bitRead(value,i));
//   }
// }

void printFormatted(Print& pr, float v, int dd = 2, int digits = 1) {
  float tv = v;
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i) rounding /= 10.0;
  tv += rounding;
  int i = 0;
  int p = tv;
  while (p > 0) {
    i++;
    p = p / 10.0;
  }
  if (i == 0) i++;
  if (v < 0) i++;
  for (int idx = 0; idx < dd - i; idx++) {
    pr.print(" ");
  }
  pr.print(v, digits);
}


ISR(TIMER1_OVF_vect)
{
  TCNT1 = TIMER1_PRELOADING; // Timer Preloading
  int but = digitalRead(BUT_PIN);
  if (but > 0 && but_ticks < 10000) {
    but_ticks++;
  }
  if (but_ticks > 0 && but <= 0) {
    beep(1);
    if (but_ticks > 100) {
      display_screen(3);
    } else {
      display_screen(0);
    }
    but_ticks = 0;
  }  
  if (screen_timeout > 0) screen_timeout--;

  // beep
  int _beep = digitalRead(BUZZ_PIN);
  if ((beep_type == 1 || beep_type == 3 || (beep_type == 2 && (beep_ticks > 0 && beep_ticks < 50) || beep_ticks > 100)) && beep_ticks > 0 && !_beep) {
    // BUZZ_PIN D3 PD3
    PORTD |= 1 << 3;
  }
  if ((beep_type == 2 && (beep_ticks >= 50 && beep_ticks < 100) && beep_ticks > 0 && _beep)) {
    // BUZZ_PIN D3 PD3
    PORTD &= ~(1 << 3);
  }
  if (beep_ticks > 0) beep_ticks--;
  if (beep_ticks == 0) beep_type = 0;
  if (beep_type == 0 && beep_ticks == 0 && _beep) {
    // BUZZ_PIN D3 PD3
    PORTD &= ~(1 << 3);
  }

}

void setup()
{
  // https://deepbluembedded.com/arduino-timer-calculator-code-generator/
  cli();
  // 10ms
  TCCR1A = 0;                // Init Timer1A
  TCCR1B = 0;                // Init Timer1B
  TCCR1B |= B00000010;       // Prescaler = 8
  TCNT1 = TIMER1_PRELOADING; // Timer Preloading
  TIMSK1 |= B00000001;       // Enable Timer Overflow Interrupt
  sei();

  analogReference(INTERNAL);

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);

  pinMode(BUT_PIN, INPUT);
  pinMode(BUZZ_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  Serial.begin(115200);
  delay(100);
  
  Wire.begin();
  if (!INA.begin() )
  {
    Serial.println("INA could not connect. Fix and Reboot");
  }
  // INA.setMaxCurrentShunt(1, 0.002);
  // INA.setMaxCurrentShunt(0.8F, 0.1F);
  INA.configure(0.00417, 2, 0, 9978.38017628472);

  INA.setAverage(INA226_1024_SAMPLES);

  display.begin(SH1106_SWITCHCAPVCC, 0x3C);

  // display.display();
  // delay(1000);
  display.clearDisplay();

  // Serial.print(">>> TCCR0A: ");
  // printBits(Serial, TCCR0A);
  // Serial.println();
  // Serial.print(">>> TCCR0B: ");
  // printBits(Serial, TCCR0B);
  // Serial.println();
  // Serial.print(">>> TCCR1A: ");
  // printBits(Serial, TCCR1A);
  // Serial.println();
  // Serial.print(">>> TCCR1B: ");
  // printBits(Serial, TCCR1B);
  // Serial.println();
  // Serial.print(">>> TCCR2A: ");
  // printBits(Serial, TCCR2A);
  // Serial.println();
  // Serial.print(">>> TCCR2B: ");
  // printBits(Serial, TCCR2B);
  // Serial.println();

  // Serial.print(">>> TIMSK0: ");
  // printBits(Serial, TIMSK0);
  // Serial.println();
  // Serial.print(">>> TIMSK1: ");
  // printBits(Serial, TIMSK1);
  // Serial.println();
  // Serial.print(">>> TIMSK2: ");
  // printBits(Serial, TIMSK2);
  // Serial.println();

}



void loop() {

  vin = (HREF * analogRead(A0)) / 1024 * 30;
  // vin changed
  if (abs(vin_prev - vin) > 12) {
    vin_prev = vin;
    display_screen(4);
    if (vin > 12) {
      beep(2);
    } else {
      beep(3);
    }
  }
  vc1 = (HREF * analogRead(A1)) / 1024 * 30;
  vc2 = (HREF * analogRead(A2)) / 1024 * 30;
  vc3 = (HREF * analogRead(A3)) / 1024 * 30;
  resolveBatVoltage(vc1, &vb1, &bc1);
  resolveBatVoltage(vc2, &vb2, &bc2);
  resolveBatVoltage(vc3, &vb3, &bc3);

  bus = INA.getBusVoltage();
  cur = INA.getCurrent();
  pwr = INA.getPower();
  // consuming changed
  if (abs(pwr_prev - pwr) > 10) {
    pwr_prev = pwr;
    display_screen(4);
  }

  render();

  blink_10++;
  if (blink_10 > 10) blink_10 = 0;
  blink_2 = blink_10 < 5 ? 0 : 1;

  delay(1);  
}


void beep(uint8_t type) {
  switch (type) {
  case 1: // one beep
    beep_ticks = 7; // 70ms
    beep_type = type;
    break;
  case 2: // double beep
    beep_ticks = 150; // .5 + .5s
    beep_type = type;
    break;
  case 3: // long beep
    beep_ticks = 200; // 2s
    beep_type = type;
    break;  
  default:
    beep_ticks = 0;
    beep_type = 0;
    break;
  }  
}


void renderHeader() {
  float vbs[3] = {vb1, vb2, vb3};
  float bcap[3] = {BAT1_CAPACITY, BAT2_CAPACITY, BAT3_CAPACITY};
  float bat_total = 0;
  float batcap_total = 0;
  uint8_t bat_count = 0;
  for (int i = 0; i < 3; i++) {
    if (vbs[i] > 12) {
      bat_total += vbs[i];
      batcap_total += bcap[i];
      bat_count++;
    }
  }
  //bat_level
  uint8_t bl = bat_count > 0 ? bat_level(bat_total / bat_count) : 0;


  display.setCursor(0,0);
  display.setTextSize(1);
  display.print("vin:");
  if (vin > 12) {
    printFormatted(display, vin, 2, 0);
    display.print("V");
  } else if (blink_2) {
    printFormatted(display, vin, 2, 0);
    display.print("V");
  } else {
    display.print("   ");
  }
  // https://www.powersol.com.ua/ibp/kak-rasschitat-vremya-avtonomnoj-raboty-ibp/#:~:text=%D0%92%D0%BE%D0%BE%D1%80%D1%83%D0%B6%D0%B8%D0%B2%D1%88%D0%B8%D1%81%D1%8C%20%D0%BA%D0%B0%D0%BB%D1%8C%D0%BA%D1%83%D0%BB%D1%8F%D1%82%D0%BE%D1%80%D0%BE%D0%BC%2C%20%D1%80%D0%B0%D1%81%D1%87%D0%B5%D1%82%20%D0%BC%D0%BE%D0%B6%D0%BD%D0%BE%20%D0%BE%D1%81%D1%83%D1%89%D0%B5%D1%81%D1%82%D0%B2%D0%B8%D1%82%D1%8C,%D0%BF%D0%BE%D0%BB%D1%83%D1%87%D0%B0%D0%B5%D0%BC%20%3D%20%D0%9A%D0%BE%D0%BB%D0%B8%D1%87%D0%B5%D1%81%D1%82%D0%B2%D0%BE%20%D1%87%D0%B0%D1%81%D0%BE%D0%B2%20%D0%BD%D0%B5%D0%BF%D1%80%D0%B5%D1%80%D1%8B%D0%B2%D0%BD%D0%BE%D0%B9%20%D1%80%D0%B0%D0%B1%D0%BE%D1%82%D1%8B.
  // estimate working time
  if (vin < 12 && bat_count > 0) {
    int et_total = 60.0 * bus * batcap_total * K * (bat_level(bus) / 100.0) / pwr;
    int et_d = et_total / (60 * 24);
    int et_h = (et_total % (60 * 24)) / 60;
    int et_m = et_total % (60 * 24) % 60;
    display.print(" ");
    display.print(et_d);
    display.print("d");
    display.print(et_h);
    display.print("h");
    display.print(et_m);
    display.print("m");
  }

  display.drawRoundRect(SCR_W - BAT_W, 0, BAT_W - 1, BAT_H, 2, WHITE);
  display.drawLine(SCR_W - 1, 2, SCR_W - 1, BAT_H - 3, WHITE);
  if (bc1 || bc2 || bc3) {
    for (int i = 0; i < min(blink_10, bl / 10); i++) {
      display.drawLine(SCR_W - BAT_W + 1 + i, 1, SCR_W - BAT_W + 1 + i, BAT_H - 2, WHITE);
    }
  } else {
    for (int i = 0; i < bl / 10; i++) {
      display.drawLine(SCR_W - BAT_W + 1 + i, 1, SCR_W - BAT_W + 1 + i, BAT_H - 2, WHITE);
    }
  }

}

void renderBat(uint8_t idx, float vc, float vb, uint8_t bc, uint8_t offcet, uint8_t width) {
  display.drawRect(offcet, 8, width, SCR_H - 8, WHITE);
  display.drawRect(offcet, 8, 10, 12, WHITE);
  display.setCursor(offcet + 2, 10);
  display.setTextSize(1);
  display.print(idx);
  if (vb < 12) {
    display.setCursor(offcet + 10, 19 + 8);
    display.setTextSize(2);
    display.print("NA");
  } else {
    display.setCursor(offcet + 8, 19 + 8);
    display.setTextSize(1);
    printFormatted(display, vb, 2, 1);
    display.println("V");
    display.setCursor(offcet + 8, 19 + 8 + 8);
    printFormatted(display, bat_level(min(vb, BAT_LEVEL_MAX)), 4, 0);
    display.println("%");
    if (vin > 5 && bc && blink_2) {
      display.setCursor(offcet + 8, 19 + 8 + 8 + 8);
      display.println("chrg!");
    }
  }
}

void render() {
  display.clearDisplay();
  if (screen_timeout == 0) {
    display.display();
    return;
  }
  display.setTextColor(WHITE);
  if (screen == 1) {
    renderHeader();

    display.drawRect(0, 8, SCR_W, 40, WHITE);
    display.drawRect(0, 47, 64, 16, WHITE);
    display.drawRect(63, 47, 65, 16, WHITE);

    display.setCursor(11, 17);
    display.setTextSize(3);
    printFormatted(display, pwr, 3, 1);
    display.println("W");

    display.setCursor(18, 51);
    display.setTextSize(1);
    printFormatted(display, bus, 2, 1);
    display.println("V");

    display.setCursor(65 + 14, 51);
    display.setTextSize(1);
    printFormatted(display, cur, 3, 1);
    display.println("A");

  } else if (screen == 2) {
    renderHeader();
    renderBat(1, vc1, vb1, bc1, 0, 43);
    renderBat(2, vc2, vb2, bc2, 42, 43);
    renderBat(3, vc3, vb3, bc3, 84, 44);

  } else if (screen == 3) {
    display.setCursor(0,0);
    display.setTextSize(1);
    display.print("bus:");
    printFormatted(display, bus, 2, 4);
    display.println("V");
    display.print("cur:");
    printFormatted(display, cur, 2, 4);
    display.println("A");
    display.print("pwr:");
    printFormatted(display, pwr, 2, 4);
    display.println("W");

    display.print("vin:");
    printFormatted(display, vin, 2, 4);
    display.println("V");
    display.print("vc1:");
    printFormatted(display, vc1, 2, 4);
    display.print("V ");
    printFormatted(display, vb1, 2, 1);
    display.println("V");
    display.print("vc2:");
    printFormatted(display, vc2, 2, 4);
    display.print("V ");
    printFormatted(display, vb2, 2, 1);
    display.println("V");
    display.print("vc3:");
    printFormatted(display, vc3, 2, 4);
    display.print("V ");
    printFormatted(display, vb3, 2, 1);
    display.println("V");
  }
  
  display.display();
}

void resolveBatVoltage(float vc, float* vbp, uint8_t* bcp) {
  if ((vin < 5 || vin > 23) && vc > BAT_RESOLVE_MIN && vc < BAT_RESOLVE_MAX) {
    *vbp = vc;
  }
  *bcp = vin > 23 && *vbp == vc;
}

float bat_level(float vb) {
  if (vb > BAT_LEVEL_MAX) vb = BAT_LEVEL_MAX;
  if (vb < BAT_LEVEL_MIN) vb = BAT_LEVEL_MIN;
  return (vb - BAT_LEVEL_MIN) / (BAT_LEVEL_MAX - BAT_LEVEL_MIN) * 100.0;
}


void display_screen(uint8_t scr) {
  if (screen_timeout > 0) {
    if (scr == 0) {
      screen = screen != 1 ? 1 : 2;
    } else if (scr >=1 && scr <= 3) {
      screen = scr;
    }
  }
  screen_timeout = DISPLAY_TIMEOUT;
}
