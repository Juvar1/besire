// Radio related
#include <SI4735.h>
// Display related
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include <TouchScreen.h>
// Timekeeping related
#include <TimeLib.h>
// Power saving related
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/power.h>

// Uncomment to disable radio and focus on human interface development
//#define TFT_DEBUG

#define TFT_DC 9
#define TFT_CS 10
#define RESET_PIN 15
#define INT_PIN 2
#define AUDIO_MUTE_PIN 8
#define LW_PIN 6
#define MW_PIN 7

#define EEPROM_ADDR 0x50

// TFT maximum brightness, max is 255
#define TFT_MAXBL 80
#define TFT_BL_PIN 3

#define LONG_PRESS_TRIG 1500

// touchscreen pins
#define YP A2  // must be an analog pin, use "An" notation!
#define XM A3  // must be an analog pin, use "An" notation!
#define YM 5   // can be a digital pin
#define XP 4   // can be a digital pin
// touchscreen calibration data
#define TSMINX 11
#define TSMINY 0
#define TSMAXX 750
#define TSMAXY 750

#define FM_BAND_TYPE 0
#define MW_BAND_TYPE 1
#define SW_BAND_TYPE 2
#define LW_BAND_TYPE 3

const char bandName[][4] = {"VHF", "MW ", "SW ", "LW "};
const char modName[][4] = {"FM ", "AM ", "LSB", "USB"};
uint8_t currentMod = 0;

bool standby = false;
uint8_t volume = 15;

uint8_t bwIdxSSB = 0;
const uint8_t bwSSB[] = {12, 22, 30, 40, 5, 10}; // divide by ten

uint8_t bwIdxAM = 3;
const uint8_t bwAM[] = {60, 40, 30, 20, 10, 18, 25}; // divide by ten

typedef struct {
  uint8_t x; // x position
  uint8_t y; // y position
  const char *text; // button text
  uint16_t color; // button color
  bool inUse; // visible button
  uint8_t active; // active button
  void (*callbackF)(uint8_t r); // callback function
} Button;

typedef struct {
  uint8_t bandType;     // Band type (FM, MW or SW)
  uint16_t minimumFreq; // Minimum frequency of the band
  uint16_t maximumFreq; // maximum frequency of the band
  uint16_t currentFreq; // Default frequency or current frequency
  uint16_t currentStep; // Default step (increment and decrement)
} Band;

Band band[] = {
  {FM_BAND_TYPE, 8750, 10800, 8750, 10},   // VHF with 100kHz step
  {LW_BAND_TYPE, 100, 510, 100, 1},
  {MW_BAND_TYPE, 540, 1600, 540, 10},      // MW North America with 10kHz step
  {MW_BAND_TYPE, 522, 1710, 522, 9},       // MW Europe
  {SW_BAND_TYPE, 1800, 3500, 1900, 1},     // 160 meters
  {SW_BAND_TYPE, 3500, 4500, 3700, 1},     // 80 meters
  {SW_BAND_TYPE, 4500, 5500, 4850, 5},
  {SW_BAND_TYPE, 5600, 6300, 6000, 5},
  {SW_BAND_TYPE, 6800, 7800, 7200, 5},     // 40 meters
  {SW_BAND_TYPE, 9200, 10000, 9600, 5},
  {SW_BAND_TYPE, 10000, 11000, 10100, 1},  // 30 meters
  {SW_BAND_TYPE, 11200, 12500, 11940, 5},
  {SW_BAND_TYPE, 13400, 13900, 13600, 5},
  {SW_BAND_TYPE, 14000, 14500, 14200, 1},  // 20 meters
  {SW_BAND_TYPE, 15000, 15900, 15300, 5},
  {SW_BAND_TYPE, 17200, 17900, 17600, 5},
  {SW_BAND_TYPE, 18000, 18300, 18100, 1},  // 17 meters
  {SW_BAND_TYPE, 21000, 21900, 21200, 1},  // 15 mters
  {SW_BAND_TYPE, 24890, 26200, 24940, 1},  // 12 meters
  {SW_BAND_TYPE, 26965, 27405, 26965, 10}, // CEPT CB band (11 meters)
  {SW_BAND_TYPE, 28000, 30000, 28400, 1}   // 10 meters
};

Button button[] = {
  {2, 2, "VOL\x18", ILI9341_GREEN, true, 1, 0},
  {2, 1, "VOL\x19", ILI9341_GREEN, true, 1, 0},
  {1, 2, "FRQ\x18", ILI9341_YELLOW, true, 1, 0},
  {1, 1, "FRQ\x19", ILI9341_YELLOW, true, 1, 0},
  {0, 2, "BAND", ILI9341_RED, true, 1, 0},
  {0, 1, "MOD", ILI9341_BLUE, true, 1, 0},
  {0, 0, "BW", ILI9341_DARKGREY, true, 1, 0},
  {1, 0, "STP\x19", ILI9341_LIGHTGREY, true, 1, 0},
  {2, 1, "OFF", ILI9341_RED, false, 1, 0},
  {2, 0, "STP\x18", ILI9341_LIGHTGREY, true, 1, 0},
  {1, 2, "SEEK", ILI9341_YELLOW, false, 1, 0}
};

const uint8_t lastBand = (sizeof band / sizeof(Band)) - 1;
uint8_t currentBand = 0;

// Long button press sense
unsigned long duration = 0;
// Update RDS and RSSI display
bool update = true;

// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);
SI4735 rx;

// For better pressure precision, we need to know the resistance
// between X+ and X- Use any multimeter to read it
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);

// canvas for non flickering volume display
GFXcanvas1 canvas(tft.width(), 8); // 1-bit, width*8 pixels

void setup() {
  button[0].callbackF = volumeUp;
  button[1].callbackF = volumeDown;
  button[2].callbackF = freqUp;
  button[3].callbackF = freqDown;
  button[4].callbackF = bandSel;
  button[5].callbackF = modSel;
  button[6].callbackF = bandwidthSel;
  button[7].callbackF = stepDown;
  button[8].callbackF = standbyMode;
  button[9].callbackF = stepUp;
  button[10].callbackF = seekMode;
  
  analogWrite(TFT_BL_PIN, TFT_MAXBL);
  
  pinMode(LW_PIN, OUTPUT);
  pinMode(MW_PIN, OUTPUT);
  digitalWrite(LW_PIN, HIGH);
  digitalWrite(MW_PIN, HIGH);

#ifndef TFT_DEBUG
  rx.setAudioMuteMcuPin(AUDIO_MUTE_PIN);
  delay(500);
  rx.setup(RESET_PIN, INT_PIN, FM_BAND_TYPE);
  rx.setPowerUp(1, 0, 0, 1, 0, SI473X_ANALOG_AUDIO);
  rx.radioPowerUp();
  rx.setFM(band[currentBand].minimumFreq, band[currentBand].maximumFreq, band[currentBand].currentFreq, band[currentBand].currentStep);
  delay(500);
  rx.setRdsConfig(3, 3, 3, 3, 3);
  rx.setFifoCount(1);
  rx.setVolume(volume);
#endif

  setTime(12, 00, 00, 1, 1, 2025);
  tft.begin();
  tft.fillScreen(ILI9341_BLACK);
  updateDisplay();
}

void loop() {
  // update RDS and RSSI display
  if (second() % 10 == 0 && update) {
    update = false;
    if (second() == 0) {updateRDS();}
    updateSignalStrength();
  } else if (second() % 10 != 0) {
    update = true;
  }
  
  drawButtons();

  if (ts.pressure() > ts.pressureThreshhold) {
    for (uint8_t i = 0; i < 100; i++) {
      delay(1);
      if (ts.pressure() < ts.pressureThreshhold) {
        return;
      }
    }
  } else {
    if (standby) {enterStandby();}
    return;
  }

  if (standby) {
    standby = false;
    tft.sendCommand(ILI9341_SLPOUT); // hidden command to wake up TFT
    delay(100);
#ifndef TFT_DEBUG
    rx.radioPowerUp();
    rx.setFrequency(band[currentBand].currentFreq);
#endif
    for (uint8_t i = 0; i < TFT_MAXBL; i++) {
      analogWrite(3, i);
      delay(10);
    }
    return;
  }
  
  TSPoint p = ts.getPoint();
  updateButtons(p.x, p.y);
}

void tftprint(const char *buf, int16_t x, int16_t y, uint8_t size, uint16_t color) {
  tft.setTextSize(size);
  tft.setTextColor(color, ILI9341_BLACK);
  tft.setCursor(x, y);
  tft.print(buf);
}

void updateVolume() {
  canvas.fillScreen(0);
  canvas.drawRect(0, 0, canvas.width(), canvas.height(), 1);
  canvas.fillRect(0, 0, map(volume, 0, 63, 0, canvas.width()), canvas.height(), 1);
  tft.drawBitmap(0, tft.height() / 5 * 2 - 11, canvas.getBuffer(), canvas.width(), canvas.height(), ILI9341_GREEN, ILI9341_BLACK);  
}

void updateRDS() {
  const char buf[32];
#ifndef TFT_DEBUG
  if (band[currentBand].bandType == FM_BAND_TYPE) {
    uint16_t y, m, d, h, min;
    rx.rdsBeginQuery(); // gets RDS information
    rx.getRdsDateTime(&y, &m, &d, &h, &m);
    setTime(h, min, second(), d, m, y);
    tftprint(rx.getRdsStationName(), 0, 0, 1, ILI9341_WHITE);
    tftprint(rx.getRdsProgramInformation(), 0, tft.height() / 5, 1, ILI9341_WHITE);
  }
#endif
  sprintf(buf, "%02d.%02d.%d %02d:%02d", day(), month(), year(), hour(), minute());
  tftprint(buf, 0, tft.height() / 5 * 2 - 20, 1, ILI9341_WHITE);
}

void updateSignalStrength() {
  char buf[40];
  buf[0] = 0;
  strcpy(buf, "SNR: 100 dB | RSSI: 100 dBuV");
#ifndef TFT_DEBUG
  rx.getStatus();
  rx.getCurrentReceivedSignalQuality();
  sprintf(buf, "SNR: %3d dB | RSSI: %3d dBuV", rx.getCurrentSNR(), rx.getCurrentRSSI());
#endif
  tftprint(buf, 0, 42, 1, ILI9341_WHITE);
}

void updateBwStep() {
  const char buf[40];
  uint16_t bw = 1000;
  uint16_t step = band[currentBand].currentStep * 10;
  if (band[currentBand].bandType != FM_BAND_TYPE) {
    step = band[currentBand].currentStep;
    switch (currentMod) {
      case 1:
        bw = bwAM[bwIdxAM];
        break;
      case 2:
      case 3:
        bw = bwSSB[bwIdxSSB];
        break;
    }
  }
  // Workaround because seems that we can't print float numbers with "%f"
  sprintf(buf, "BW: %3d.%d kHz | STEP: %3d kHz", bw / 10, bw % 10, step);
  tftprint(buf, 0, 51, 1, ILI9341_WHITE);
}

void updateBandType() {
  tftprint(bandName[band[currentBand].bandType], tft.width() - 40, 0, 2, ILI9341_RED);
}

void updateMod() {
  tftprint(modName[currentMod], tft.width() - 40, 17, 2, ILI9341_BLUE);
}

void updateFreq() {
  char buf[10];
  if (band[currentBand].bandType == FM_BAND_TYPE) {
    sprintf(buf, "%06u0", band[currentBand].currentFreq);
  } else {
    sprintf(buf, "%07u", band[currentBand].currentFreq);
  }
  tftprint(buf, 0, 10, 4, ILI9341_YELLOW);
  dtostrf(299792.458F / atol(buf), 3, 2, buf);
  strcat(buf, " m");
  for (uint8_t i = strlen(buf); i < 9; *(buf + i++) = 32);
  buf[9] = 0;
  tftprint(buf, tft.width() >> 1, tft.height() / 5 * 2 - 20, 1, ILI9341_WHITE);
}

void updateDisplay() {
  updateVolume();
  updateRDS();
  updateSignalStrength();
  updateBwStep();
  updateBandType();
  updateMod();
  updateFreq();

  tftprint("kHz", tft.width() - 40, 34, 2, ILI9341_YELLOW);
}

void volumeUp(uint8_t r) {
  if (volume == 0) {
    button[1].inUse = true;
    button[8].inUse = false;
    button[1].active = 1;
  }
  volume = min(volume++, 63);
#ifndef TFT_DEBUG
  rx.setVolume(volume);
#endif
  updateVolume();
}

void volumeDown(uint8_t r) {
  volume = max(volume--, 0);
#ifndef TFT_DEBUG
  rx.setVolume(volume);
#endif
  if (volume == 0) {
    button[1].inUse = false;
    button[8].inUse = true;
    button[8].active = 1;
  }
  updateVolume();
}

void freqUp(uint8_t r) {
  if (r > 10) {
    button[2].inUse = false;
    button[10].y = 2;
    button[10].inUse = true;
    button[10].active = 1;
#ifndef TFT_DEBUG
    rx.seekStationProgress(showSeekProgress, abortSeek, 1);
#endif
    return;
  }
  band[currentBand].currentFreq += band[currentBand].currentStep;
  if (band[currentBand].currentFreq > band[currentBand].maximumFreq) {
    band[currentBand].currentFreq = band[currentBand].minimumFreq;
  }
#ifndef TFT_DEBUG
  rx.setFrequency(band[currentBand].currentFreq);
#endif
  updateFreq();
}

void freqDown(uint8_t r) {
  if (r > 10) {
    button[3].inUse = false;
    button[10].y = 1;
    button[10].inUse = true;
    button[10].active = 1;
#ifndef TFT_DEBUG
    rx.seekStationProgress(showSeekProgress, abortSeek, 0);
#endif
    return;
  }
  band[currentBand].currentFreq -= band[currentBand].currentStep;
  if (band[currentBand].currentFreq < band[currentBand].minimumFreq) {
    band[currentBand].currentFreq = band[currentBand].maximumFreq;
  }
#ifndef TFT_DEBUG
  rx.setFrequency(band[currentBand].currentFreq);
#endif
  updateFreq();
}

void bandSel(uint8_t r) {
  currentBand++;
  if (currentBand > lastBand) {currentBand = 0;}

  switch (band[currentBand].bandType) {
    case FM_BAND_TYPE:
      currentMod = 0;
      break;
    case SW_BAND_TYPE:
      if (currentMod >= 1) {currentMod--;} // for modSel() function
      digitalWrite(LW_PIN, HIGH);
      digitalWrite(MW_PIN, HIGH);
      break;
    case MW_BAND_TYPE:
      if (currentMod >= 1) {currentMod--;} // for modSel() function
      digitalWrite(LW_PIN, HIGH);
      digitalWrite(MW_PIN, LOW);
      break;
    case LW_BAND_TYPE:
      if (currentMod >= 1) {currentMod--;} // for modSel() function
      digitalWrite(LW_PIN, LOW);
      digitalWrite(MW_PIN, HIGH);
      break;
  }
  updateBandType();
  updateFreq();
  updateBwStep();

  modSel(0);
}

void modSel(uint8_t r) {
  if (band[currentBand].bandType != FM_BAND_TYPE) {
    currentMod++;
    if (currentMod > 3) {currentMod = 1;}
#ifndef TFT_DEBUG
    if (currentMod > 1) {
      loadSSB();
      delay(100);
      rx.setSSB(band[currentBand].minimumFreq, band[currentBand].maximumFreq, band[currentBand].currentFreq, band[currentBand].currentStep, currentMod - 1);
      delay(100);
    } else {
      rx.setAM(band[currentBand].minimumFreq, band[currentBand].maximumFreq, band[currentBand].currentFreq, band[currentBand].currentStep);
    }
  } else {
    rx.setFM(band[currentBand].minimumFreq, band[currentBand].maximumFreq, band[currentBand].currentFreq, band[currentBand].currentStep);
#endif
  }
  updateMod();
}

void bandwidthSel(uint8_t r) {
  if (band[currentBand].bandType != FM_BAND_TYPE) {
    switch (currentMod) {
      case 1:
        bwIdxAM++;
        if (bwIdxAM == sizeof bwAM) bwIdxAM = 0;
#ifndef TFT_DEBUG
        rx.setBandwidth(bwIdxAM, 0);
#endif
        break;
      case 2:
      case 3:
        bwIdxSSB++;
        if (bwIdxSSB == sizeof bwSSB) bwIdxSSB = 0;
#ifndef TFT_DEBUG
        rx.setSSBAudioBandwidth(bwIdxSSB);
        if (bwIdxSSB == 0 || bwIdxSSB == 4 || bwIdxSSB == 5) {
          rx.setSSBSidebandCutoffFilter(1);
        } else {
          rx.setSSBSidebandCutoffFilter(0);
        }
#endif
        break;
    }
  }
  updateBwStep();
}

void stepUp(uint8_t r) {
  if (band[currentBand].currentStep < 100) {band[currentBand].currentStep++;}
  updateBwStep();
}

void stepDown(uint8_t r) {
  if (band[currentBand].currentStep > 0) {band[currentBand].currentStep--;}
  updateBwStep();
}

void seekMode(uint8_t r) {
  if (r > 5) {abortSeek();}
}

void standbyMode(uint8_t r) {
  if (r < 7) {return;}
  standby = true;
  for (uint8_t i = TFT_MAXBL; i > 0; i--) {
    analogWrite(3, i);
    delay(10);
  }
  tft.sendCommand(ILI9341_SLPIN); // hidden command to sleep TFT
#ifndef TFT_DEBUG
  rx.powerDown();
#endif
  enterStandby();
}

bool abortSeek() {
  button[10].inUse = false;
  button[2].inUse = true;
  button[3].inUse = true;
  button[2].active = 1;
  button[3].active = 1;
  return true;
}

void showSeekProgress(uint16_t freq) {
  button[2].inUse = true;
  button[10].inUse = false;
  band[currentBand].currentFreq = freq;
  updateFreq();
}

// Loads the contents of the EEPROM into the CI (Si4735) and starts the radio on SSB mode.
void loadSSB() {
  rx.reset();
  rx.queryLibraryId(); // Is it really necessary here?
  rx.patchPowerUp();
  delay(50);
  rx.setI2CFastMode(); // Recommended
  rx.downloadPatchFromEeprom(EEPROM_ADDR);
  rx.setI2CStandardMode(); // goes back to default (100kHz)
  // Parameters
  // AUDIOBW - SSB Audio bandwidth; 0 = 1.2kHz (default); 1=2.2kHz; 2=3kHz; 3=4kHz; 4=500Hz; 5=1kHz;
  // SBCUTFLT SSB - side band cutoff filter for band pass and low pass filter ( 0 or 1)
  // AVC_DIVIDER  - set 0 for SSB mode; set 3 for SYNC mode.
  // AVCEN - SSB Automatic Volume Control (AVC) enable; 0=disable; 1=enable (default).
  // SMUTESEL - SSB Soft-mute Based on RSSI or SNR (0 or 1).
  // DSP_AFCDIS - DSP AFC Disable or enable; 0=SYNC MODE, AFC enable; 1=SSB MODE, AFC disable.
  rx.setSSBConfig(bwIdxSSB, 1, 0, 1, 0, 1);
}

void drawButtons() {
  for (uint8_t i = 0; i < sizeof button / sizeof(Button); i++) {
    if (button[i].inUse && button[i].active > 0 && millis() - duration > LONG_PRESS_TRIG) {
      button[i].active = 0;
      drawButton(button[i].x, button[i].y, button[i].text, button[i].color);
    }
  }
}

void updateButtons(int16_t tsx, int16_t tsy) {
  const int16_t x = constrain(map(tsx, TSMINX, TSMAXX, 0, 2), 0, 2);
  const int16_t y = constrain(map(tsy, TSMINY, TSMAXY, 4, 0), 0, 4);

  /*tft.setTextSize(1);
  tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
  tft.setCursor(0, 0);
  tft.print("       ");
  tft.setCursor(0, 0);
  tft.print(newDuration);*/

  for (uint8_t i = 0; i < sizeof button / sizeof(Button); i++) {
    if (button[i].inUse) {
      if (button[i].x == x && button[i].y == y) {
        button[i].active++;
        drawButton(button[i].x, button[i].y, button[i].text, ILI9341_WHITE);
        button[i].callbackF(button[i].active);
      }
    }
  }
  duration = millis();
}

void drawButton(int8_t posx, int8_t posy, const char *text, uint16_t color) {
  const int16_t sizex = tft.width() / 3;
  const int16_t sizey = tft.height() / 5;

  int16_t x, y;
  uint16_t w, h;
  char buf[7];
  uint8_t length = min(strlen(text), 5);
  buf[0] = 32;
  buf[1] = text[0];
  buf[2] = text[1];
  buf[3] = text[2];
  buf[4] = text[3];
  buf[length + 1] = 32;
  buf[length + 2] = 0;
  tft.setTextSize(2);
  tft.getTextBounds(buf, 1, 1, &x, &y, &w, &h);
  const int16_t x1 = sizex * posx + ((sizex >> 1) - (w >> 1));
  const int16_t y1 = sizey * (4 - posy) + (sizey >> 1) - (h >> 1);
  tft.setCursor(x1, y1);
  tft.setTextColor(color, ILI9341_BLACK);
  tft.print(buf);

  tft.drawRoundRect(sizex * posx + 6, sizey * (4 - posy) + 6, sizex - 12, sizey - 12, 6, color);
  for (uint8_t c = 0; c < 5; c++) {
    tft.drawRoundRect(sizex * posx + 2, sizey * (4 - posy) + 2, sizex - 4 - c, sizey - 4 - c, 6, color);
  }
}

void enterStandby() {
  power_timer2_disable();
  power_adc_disable();
  power_timer1_disable();
  power_timer0_disable();
  power_spi_disable();
  power_usart0_disable();
  power_twi_disable();

  // clear various "reset" flags
  MCUSR = 0;     
  // allow changes, disable reset
  WDTCSR = bit(WDCE) | bit(WDE);
  // set interrupt mode and an 4 seconds interval 
  WDTCSR = bit(WDIE) | bit(WDP3);

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_cpu();
  
  sleep_disable();

  power_adc_enable();
  power_timer2_enable();
  power_timer1_enable();
  power_timer0_enable();
  power_spi_enable();
  power_usart0_enable();
  power_twi_enable();
}

ISR(WDT_vect) {
	// WDIE & WDIF is cleared in hardware upon entering this ISR
	wdt_disable();
}