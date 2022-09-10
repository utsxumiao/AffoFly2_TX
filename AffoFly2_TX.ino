#include "Bounce2.h"
#include "printf.h"
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <U8g2lib.h>

#define DEBUG
//#define SHOW_OUPUT

#define THROTTLE_PIN      A0
#define YAW_PIN           A1
#define PITCH_PIN         A2
#define ROLL_PIN          A3
#define SWD1_PIN          A6
#define V_BAT_PIN         A7
#define AUX1_PIN          2
#define AUX2_PIN          3
#define AUX3_PIN          4
#define AUX4_PIN          5
#define CPPM_PIN          6
#define FN_PIN            7
#define BUZZER_PIN        8
#define NRF_CE_PIN        9
#define NRF_CSN_PIN       10

#define PROJECT_NAME            "AffoFly2 TX"
#define PROJECT_VERSION         "v0.100"
#define BUTTON_COUNT            4
#define BUTTON_DEBOUNCE_MS      5
#define RADIO_PIPE              0xE8E8F0F0E1LL
#define RADIO_CHANNEL           125
#define RADIO_SECURITY_TOKEN    998789
#define JOYS_VAL_SAMPLE_COUNT   5
#define JOYS_VAL_SAMPLE_ELIMI   1
#define JOYS_CAL_SAMPLE_COUNT   10
#define JOYS_CAL_SAMPLE_ELIMI   2
#define CLOCK_MULTIPLIER        1       // 1 for 8MHZ, 2 for 16MHZ
#define PPM_FRAME_LENGTH        20000
#define PPM_PULSE_LENGTH        400
#define CHANNEL_COUNT           8
#define V_BAT_ALARM_VOLTAGE     3.5
#define WELCOME_SCREEN_DURATION 2000    //2 seconds

struct ControlData {
  uint32_t Token;
  uint16_t Throttle;
  uint16_t Yaw;
  uint16_t Pitch;
  uint16_t Roll;
  uint16_t Aux1;
  uint16_t Aux2;
  uint16_t Aux3;
  uint16_t Aux4;
};

Bounce bounces[BUTTON_COUNT];
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
ControlData controlData;
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);

uint16_t PPM[CHANNEL_COUNT];
int16_t JOYSTICK_THROTTLE_OFFSET = 0;
int16_t JOYSTICK_YAW_OFFSET = 0;
int16_t JOYSTICK_PITCH_OFFSET = 0;
int16_t JOYSTICK_ROLL_OFFSET = 0;
uint8_t aux1State = 0;
uint8_t aux2State = 0;
uint8_t aux3State = 0;
uint8_t aux4State = 0;

float BATTERY_VOLTAGE = 0;
bool LOW_VOLTAGE = false;
// set to your voltage divider resistor values
// voltage is taken on R2
uint32_t voltageDividerR1 = 200000;
uint32_t voltageDividerR2 = 10000;
// if voltage is off, use this value as percentage to shift up or down
// theoretically you can leave it "0" if your resistor is acurate enough
int8_t adjustment = 0;

uint32_t currentTime = 0;
uint16_t reportPerformanceInterval = 1000;
uint32_t lastReportPerformanceMillis = 0;
uint16_t checkButtonInterval = 20;
uint32_t lastCheckButtonMillis = 0;
uint16_t readControllerInterval = 20;
uint32_t lastReadControllerMillis = 0;
uint16_t sendRadioInterval = 0;
uint32_t lastSendRadioMillis = 0;
uint16_t readBatteryVoltageInterval = 1000;
uint32_t lastReadBatteryVoltageMillis = 0;
uint16_t refreshControlScreenInterval = 100;
uint32_t lastRefreshControlScreenMillis = 0;

uint16_t loopCount = 0;
uint16_t buttonCheckCount = 0;
uint16_t readControllerCount = 0;
uint16_t radioCount = 0;
uint32_t ppmCount = 0;
uint16_t batteryCount = 0;
uint16_t screenCount = 0;

void setup() {
  ADCSRA = (ADCSRA & 0xf8) | 0x04;  // set 16 times division to make analogRead faster
#ifdef DEBUG
  Serial.begin(115200);
  Serial.println("System starting...");
#endif
  Button_init();
  Screen_init();
  Screen_showWelcomeScreen();
  Controller_calibrate();
  Radio_init();
  CPPM_init();
}

void loop() {
  currentTime = millis();
  if (currentTime - lastCheckButtonMillis >= checkButtonInterval) {
    lastCheckButtonMillis = currentTime;
    Controller_checkButtons();
  }
  if (currentTime - lastReadControllerMillis >= readControllerInterval) {
    lastReadControllerMillis = currentTime;
    Controller_read();
  }
  if (currentTime - lastSendRadioMillis >= sendRadioInterval) {
    lastSendRadioMillis = currentTime;
    Radio_output();
  }
  if (currentTime - lastReadBatteryVoltageMillis >= readBatteryVoltageInterval) {
    lastReadBatteryVoltageMillis = currentTime;
    Battery_read();
  }
  if (currentTime - lastRefreshControlScreenMillis >= refreshControlScreenInterval) {
    lastRefreshControlScreenMillis = currentTime;
    Screen_showControlScreen();
  }

  loopCount++;

#ifdef DEBUG
  if (currentTime - lastReportPerformanceMillis >= reportPerformanceInterval) {
    lastReportPerformanceMillis = currentTime;
    reportPerformance();
  }
#endif
}

void Button_init() {
#ifdef DEBUG
  Serial.print("Initialising buttons......");
#endif
  Bounce aux1Bounce = Bounce();
  aux1Bounce.attach(AUX1_PIN, INPUT_PULLUP);
  aux1Bounce.interval(BUTTON_DEBOUNCE_MS);
  bounces[0] = aux1Bounce;
  Bounce aux2Bounce = Bounce();
  aux2Bounce.attach(AUX2_PIN, INPUT_PULLUP);
  aux2Bounce.interval(BUTTON_DEBOUNCE_MS);
  bounces[1] = aux2Bounce;
  Bounce aux3Bounce = Bounce();
  aux3Bounce.attach(AUX3_PIN, INPUT_PULLUP);
  aux3Bounce.interval(BUTTON_DEBOUNCE_MS);
  bounces[2] = aux3Bounce;
  Bounce aux4Bounce = Bounce();
  aux4Bounce.attach(AUX4_PIN, INPUT_PULLUP);
  aux4Bounce.interval(BUTTON_DEBOUNCE_MS);
  bounces[3] = aux4Bounce;
#ifdef DEBUG
  Serial.println("Done");
#endif
}

void Controller_checkButtons() {
  bounces[0].update();
  if (bounces[0].fell()) {
    aux1State = !aux1State;
  }
  bounces[1].update();
  if (bounces[1].fell()) {
    aux2State = !aux2State;
  }
  bounces[2].update();
  if (bounces[2].fell()) {
    aux3State = !aux3State;
  }
  bounces[3].update();
  if (bounces[3].fell()) {
    aux4State = !aux4State;
  }

  buttonCheckCount++;
}

void CPPM_init() {
#ifdef DEBUG
  Serial.print("Initialising CPPM......");
#endif
  pinMode(CPPM_PIN, OUTPUT);
  PORTD = PORTD & ~B01000000;  //Set CPPM to low
  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;
  OCR1A = 100;  // compare match register (not very important, sets the timeout for the first interrupt)
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();
#ifdef DEBUG
  Serial.println("Done");
#endif
}

void Controller_calibrate() {
#ifdef DEBUG
  Serial.print("Calibrating Joystick......");
#endif
  JOYSTICK_THROTTLE_OFFSET = getJoystickValue(THROTTLE_PIN, JOYS_CAL_SAMPLE_COUNT, JOYS_CAL_SAMPLE_ELIMI);
  JOYSTICK_YAW_OFFSET = getJoystickValue(YAW_PIN, JOYS_CAL_SAMPLE_COUNT, JOYS_CAL_SAMPLE_ELIMI) - 511;
  JOYSTICK_PITCH_OFFSET = getJoystickValue(PITCH_PIN, JOYS_CAL_SAMPLE_COUNT, JOYS_CAL_SAMPLE_ELIMI) - 511;
  JOYSTICK_ROLL_OFFSET = getJoystickValue(ROLL_PIN, JOYS_CAL_SAMPLE_COUNT, JOYS_CAL_SAMPLE_ELIMI) - 511;
#ifdef DEBUG
  Serial.print("Thr: ");  Serial.print(JOYSTICK_THROTTLE_OFFSET);   Serial.print("  ");
  Serial.print("Yaw: ");  Serial.print(JOYSTICK_YAW_OFFSET);        Serial.print("  ");
  Serial.print("Pitch: ");  Serial.print(JOYSTICK_PITCH_OFFSET);        Serial.print("  ");
  Serial.print("Roll: ");  Serial.print(JOYSTICK_ROLL_OFFSET);        Serial.print("  ");
  Serial.println("Done");
#endif
}

void Controller_read() {
  int16_t throttle = getJoystickValue(THROTTLE_PIN, JOYS_VAL_SAMPLE_COUNT, JOYS_VAL_SAMPLE_ELIMI) - JOYSTICK_THROTTLE_OFFSET;
  throttle = constrain(throttle, 0, 1023);
  int16_t yaw = getJoystickValue(YAW_PIN, JOYS_VAL_SAMPLE_COUNT, JOYS_VAL_SAMPLE_ELIMI) - JOYSTICK_YAW_OFFSET;
  yaw = constrain(yaw, 0, 1023);
  int16_t pitch = getJoystickValue(PITCH_PIN, JOYS_VAL_SAMPLE_COUNT, JOYS_VAL_SAMPLE_ELIMI) - JOYSTICK_PITCH_OFFSET;
  pitch = constrain(pitch, 0, 1023);
  int16_t roll = getJoystickValue(ROLL_PIN, JOYS_VAL_SAMPLE_COUNT, JOYS_VAL_SAMPLE_ELIMI) - JOYSTICK_ROLL_OFFSET;
  roll = constrain(roll, 0, 1023);

  controlData.Token = RADIO_SECURITY_TOKEN;
  controlData.Throttle = mapContollerValue(throttle, 0, 511, 1023, false);
  controlData.Yaw = mapContollerValue(yaw, 0, 511, 1023, true);
  controlData.Pitch = mapContollerValue(pitch, 0, 511, 1023, false);
  controlData.Roll = mapContollerValue(roll, 0, 511, 1023, true);
  controlData.Aux1 = mapContollerValue(aux1State * 1023, 0, 511, 1023, false);
  controlData.Aux2 = mapContollerValue(aux2State * 1023, 0, 511, 1023, false);
  controlData.Aux3 = mapContollerValue(aux3State * 1023, 0, 511, 1023, false);
  controlData.Aux4 = mapContollerValue(aux4State * 1023, 0, 511, 1023, false);

  readControllerCount++;
#ifdef SHOW_OUPUT
  Serial.print("Throttle: ");     Serial.print(controlData.Throttle);   Serial.print("    ");
  Serial.print("Yaw: ");          Serial.print(controlData.Yaw);        Serial.print("    ");
  Serial.print("Pitch: ");        Serial.print(controlData.Pitch);      Serial.print("    ");
  Serial.print("Roll: ");         Serial.print(controlData.Roll);       Serial.print("    ");
  Serial.print("Aux1: ");         Serial.print(controlData.Aux1);       Serial.print("    ");
  Serial.print("Aux2: ");         Serial.print(controlData.Aux2);       Serial.print("    ");
  Serial.print("Aux3: ");         Serial.print(controlData.Aux3);       Serial.print("    ");
  Serial.print("Aux4: ");         Serial.print(controlData.Aux4);       Serial.print("    ");
  Serial.println("");
#endif
}

uint16_t getJoystickValue(uint8_t pin, uint8_t sampleCount, uint8_t eliminator) {
  uint16_t result = 0;
  uint16_t values[sampleCount];

  for (uint8_t i = 0; i < sampleCount; i++) {
    values[i] = analogRead(pin);
  }
  sort(values, sampleCount);
  //printArray(values, sampleCount);
  for (uint8_t i = eliminator; i < sampleCount - eliminator; i++) {
    result += values[i];
  }
  result /= sampleCount - (eliminator * 2);
  return result;
}

uint16_t mapContollerValue(uint16_t val, uint16_t lower, uint16_t middle, uint16_t upper, bool reverse) {
  val = constrain(val, lower, upper);
  if (val < middle) val = map(val, lower, middle, 1000, 1500);
  else val = map(val, middle, upper, 1500, 2000);
  return reverse ? 3000 - val : val;
}

void sort(int *a, int n) {
  for (int i = 1; i < n; ++i) {
    int j = a[i];
    int k;
    for (k = i - 1; (k >= 0) && (j < a[k]); k--)
    {
      a[k + 1] = a[k];
    }
    a[k + 1] = j;
  }
}

void printArray(int *a, int n) {
  for (int i = 0; i < n; i++)
  {
    Serial.print(a[i], DEC);
    Serial.print(' ');
  }
  Serial.println();
}

void Radio_init() {
#ifdef DEBUG
  Serial.print("Initialising Radio......");
#endif
  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.setAutoAck(false);
  radio.setChannel(RADIO_CHANNEL);
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(RADIO_PIPE);
  radio.stopListening();
#ifdef DEBUG
  radio.printDetails();
  Serial.println("Done");
#endif
}

void Radio_output() {
  //uint32_t before = millis();
  radio.write(&controlData, sizeof(ControlData));
  //  uint32_t after = millis();
  //  if(after - before > 10){
  //    //TODO: once radio stuck, it will not revived by itself, intervention required.
  //    Serial.print("Radio Slow! took: ");   Serial.println(after - before);
  //  }
  radioCount++;
}

void Screen_init() {
  u8g2.begin();
  u8g2.setFont(u8g2_font_profont12_tr);
}

void Screen_showWelcomeScreen() {
  u8g2.firstPage();
  do {
    u8g2.drawStr(30, 30, PROJECT_NAME);
    u8g2.drawStr(40, 50, PROJECT_VERSION);
  } while (u8g2.nextPage());
  delay(WELCOME_SCREEN_DURATION);
}

void Screen_showControlScreen() {
  u8g2.firstPage();
  do {
    // Battery Voltage
    if (LOW_VOLTAGE) {
      u8g2.setFontMode(1);
      u8g2.drawBox(0, 0, 25, 12);
      u8g2.setDrawColor(2);
    }
    u8g2.setCursor(1, 10);
    u8g2.print(BATTERY_VOLTAGE);
    u8g2.setCursor(26, 10);
    u8g2.print("v");
    if (LOW_VOLTAGE) {
      u8g2.setFontMode(0);
      u8g2.setDrawColor(1);
    }
  }while (u8g2.nextPage());

  screenCount++;
}

void Battery_read() {
  float resolutionVoltage = 0.00107422; // AREF(1.1v) / 1024
  analogReference(INTERNAL);
  uint16_t vReading = analogRead(V_BAT_PIN);
  delay(5); //Based on my testing 5ms the reading reaches stable state
  vReading = analogRead(V_BAT_PIN);
  analogReference(DEFAULT);
  BATTERY_VOLTAGE = (vReading * resolutionVoltage * ((voltageDividerR1 + voltageDividerR2) / voltageDividerR2) * (100 + adjustment)) / 100;
#ifdef DEBUG
  Serial.print("V_BAT: "); Serial.println(BATTERY_VOLTAGE);
#endif

  if (BATTERY_VOLTAGE <= V_BAT_ALARM_VOLTAGE) {
    LOW_VOLTAGE = true;
  }

  batteryCount++;
}

void reportPerformance() {
  Serial.print("Loop: ");       Serial.print(loopCount);            Serial.print("    ");
  Serial.print("Button: ");     Serial.print(buttonCheckCount);     Serial.print("    ");
  Serial.print("Controller: "); Serial.print(readControllerCount);  Serial.print("    ");
  Serial.print("Radio: ");      Serial.print(radioCount);           Serial.print("    ");
  Serial.print("PPM: ");        Serial.print(ppmCount);             Serial.print("    ");
  Serial.print("Battery: ");    Serial.print(batteryCount);         Serial.print("    ");
  Serial.print("Screen: ");     Serial.print(screenCount);          Serial.print("    ");
  Serial.println();
  loopCount = 0;
  buttonCheckCount = 0;
  readControllerCount = 0;
  radioCount = 0;
  ppmCount = 0;
  batteryCount = 0;
  screenCount = 0;
}

ISR(TIMER1_COMPA_vect) {
  PPM[0] = controlData.Throttle;
  PPM[1] = controlData.Yaw;
  PPM[2] = controlData.Pitch;
  PPM[3] = controlData.Roll;
  PPM[4] = controlData.Aux1;
  PPM[5] = controlData.Aux2;
  PPM[6] = controlData.Aux3;
  PPM[7] = controlData.Aux4;

  static boolean state = true;
  TCNT1 = 0;
  if ( state ) {
    //end pulse
    PORTD = PORTD & ~B01000000; // turn pin 6 off.
    OCR1A = PPM_PULSE_LENGTH * CLOCK_MULTIPLIER;
    state = false;
  } else {
    //start pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;

    PORTD = PORTD | B01000000; // turn pin 6 on.
    state = true;

    if (cur_chan_numb >= CHANNEL_COUNT) {
      cur_chan_numb = 0;
      calc_rest += PPM_PULSE_LENGTH;
      OCR1A = (PPM_FRAME_LENGTH - calc_rest) * CLOCK_MULTIPLIER;
      calc_rest = 0;
    }
    else {
      OCR1A = (PPM[cur_chan_numb] - PPM_PULSE_LENGTH) * CLOCK_MULTIPLIER;
      calc_rest += PPM[cur_chan_numb];
      cur_chan_numb++;
    }
  }

  ppmCount++;
}
