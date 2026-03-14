/*
 * Project: City Sensory AI - The Ear (Final Compile Fix)
 * Board: Arduino Nano RP2040 Connect
 * Description: Generative audio engine based on GPS location and GSR bio-signals.
 * * --- Wiring Instructions (MAX98357A I2S DAC) ---
 * BCLK -> D7
 * LRCK -> D8 (The library defaults to BCLK + 1)
 * DIN  -> D9
 * GND  -> GND
 * VIN  -> 5V
 */

#include <ArduinoJson.h>
#include <I2S.h> 
#include "sound_data.h" 
#include <TinyGPS++.h>
#include <SoftwareSerial.h> 

// ==========================================
// 1. Pins & Objects
// ==========================================
const int pBCLK      = D7;  // Bit Clock
// LRCK/WS defaults to D8 (Next pin after BCLK)
const int pDOUT      = D9;  // Data Out (DIN on DAC)

const int GSRPIN     = A0;  // Galvanic Skin Response Sensor
const int GPS_RX_PIN = A2; 
const int GPS_TX_PIN = A3; 

I2S i2s(OUTPUT);
TinyGPSPlus gps;
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);

// ==========================================
// 2. Shared Variables (Inter-core Communication)
// ==========================================
volatile bool system_ready = false; 

// Target volumes for audio layers (received via ESP32)
volatile float share_target_void = 0.5; 
volatile float share_target_bio  = 0.0; 
volatile float share_target_tech = 0.0; 
volatile float share_target_entr = 0.0;

volatile int share_gsr_raw = 0;
volatile float share_gsr_modifier = 1.0;

volatile double share_lat = 0.0;
volatile double share_lon = 0.0;
String share_gps_time_str = "00:00:00"; 

// Audio data pointers (pointing to arrays in sound_data.h)
const int16_t* p_void = (const int16_t*)void_wav;
const int16_t* p_bio  = (const int16_t*)bio_wav;
const int16_t* p_tech = (const int16_t*)tech_wav;
const int16_t* p_entr = (const int16_t*)material_wav;

// ==========================================
// Core 0: Real-time Audio Engine
// ==========================================
void setup() {
  // Configure I2S Pins
  i2s.setBCLK(pBCLK);
  i2s.setDATA(pDOUT);
  
  i2s.setBitsPerSample(16);
  i2s.setFrequency(16000); 
  
  if (!i2s.begin()) { 
    // If I2S initialization fails, enter infinity loop
    while(1); 
  }
}

// Internal states for audio processing
float current_void = 0.5; float current_bio = 0.0;
float current_tech = 0.0; float current_entr = 0.0;
float pos_void = 0; float pos_bio = 0; float pos_tech = 0; float pos_entr = 0;
bool dir_void = true; bool dir_entr = true;

void loop() {
  // 1. Wait for system ready (GPS lock acquired)
  if (!system_ready) {
    // Write zero/silence to I2S (casting to int16_t for ambiguous error fix)
    i2s.write((int16_t)0); 
    i2s.write((int16_t)0); 
    return; 
  }

  // 2. Fetch shared target values
  float t_void = share_target_void;
  float t_bio  = share_target_bio;
  float t_tech = share_target_tech;
  float t_entr = share_target_entr;
  float speed  = share_gsr_modifier;

  // Smooth volume transitions (Interpolation)
  float slow = 0.00003; float fast = 0.00006;
  current_void += (t_void - current_void) * slow;
  current_bio  += (t_bio  - current_bio)  * ((t_bio > current_bio)?slow:fast);
  current_tech += (t_tech - current_tech) * ((t_tech > current_tech)?slow:fast);
  current_entr += (t_entr - current_entr) * ((t_entr > current_entr)?slow:fast);

  int32_t mixed = 0;

  // 3. Mixing Logic
  
  // Layer: The Void (Ambient/Background)
  int idx_v = (int)pos_void;
  if (idx_v >= 0 && idx_v < void_wav_samples) mixed += (int32_t)(p_void[idx_v] * current_void);
  if (dir_void) { pos_void += 0.9 * speed; if(pos_void >= void_wav_samples-1) dir_void=false; }
  else          { pos_void -= 0.9 * speed; if(pos_void <= 1) dir_void=true; }

  // Layer: Bio-Resistance
  if (current_bio > 0.01) {
    int idx_b = (int)pos_bio;
    if (idx_b < bio_wav_samples) mixed += (int32_t)(p_bio[idx_b] * current_bio);
    pos_bio += 1.0 * speed;
    if (pos_bio >= bio_wav_samples) pos_bio = (float)random(0, 4000); 
  }

  // Layer: Technological Unconscious (Bit-crush effect based on GSR)
  if (current_tech > 0.01) {
    int idx_t = (int)pos_tech;
    if (idx_t < tech_wav_samples) {
        int16_t v = p_tech[idx_t];
        if (share_gsr_raw > 600) v &= 0xF000; // GSR-triggered bit-crushing
        mixed += (int32_t)(v * current_tech);
    }
    pos_tech += 1.0 * speed;
    if (pos_tech >= tech_wav_samples) pos_tech = 0;
  }

  // Layer: Material Entropy
  if (current_entr > 0.01) {
    int idx_m = (int)pos_entr;
    if (idx_m >= 0 && idx_m < material_wav_samples) mixed += (int32_t)(p_entr[idx_m] * current_entr);
    if (dir_entr) { pos_entr += 1.0 * speed; if(pos_entr >= material_wav_samples-1) dir_entr=false; }
    else          { pos_entr -= 1.0 * speed; if(pos_entr <= 1) dir_entr=true; }
  }

  // 4. Limiter (Prevent audio clipping)
  if (mixed > 32000) mixed = 32000; 
  if (mixed < -32000) mixed = -32000;
  
  // 5. Output I2S signal
  i2s.write((int16_t)mixed); 
  i2s.write((int16_t)mixed); 
}

// ==========================================
// Core 1: Logic & Sensor Management
// ==========================================
void setup1() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200); 
  Serial1.begin(9600);     // Communication with ESP32
  Serial1.setTimeout(100);     
  gpsSerial.begin(9600); 
  pinMode(GSRPIN, INPUT);
  
  delay(1000);
  Serial.println("=== Core 1: System Boot ===");
  Serial.println("Waiting for GPS Fix...");
}

void loop1() {
  // A. Process GPS Data
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    gps.encode(c);
  }

  // B. System Status Check (GPS Lock)
  if (gps.location.isValid()) {
    system_ready = true;
    digitalWrite(LED_BUILTIN, HIGH); 
    
    share_lat = gps.location.lat();
    share_lon = gps.location.lng();
    
    if (gps.time.isValid()) {
        char timeBuffer[10];
        sprintf(timeBuffer, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
        share_gps_time_str = String(timeBuffer);
    }
  } else {
    system_ready = false;
    // Blink LED if searching for satellites
    static unsigned long blinkMs = 0;
    if (millis() - blinkMs > 500) {
        blinkMs = millis();
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); 
    }
    share_lat = 0.0;
    share_lon = 0.0;
    share_gps_time_str = "00:00:00"; 
  }

  // C. Process GSR Sensor (Bio-signal Smoothing)
  int raw = analogRead(GSRPIN);
  static int sm_gsr = 0; 
  sm_gsr = (raw + sm_gsr * 9) / 10; // Low-pass filter
  share_gsr_raw = sm_gsr;
  share_gsr_modifier = map(sm_gsr, 0, 1023, 100, 120) / 100.0;

  // D. Handle ESP32 Serial Communication
  if (Serial1.available()) {
    String line = Serial1.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return; 

    // Send sensor data to ESP32
    if (line == "GET_GSR") {
        JsonDocument respDoc;
        respDoc["GSR"] = share_gsr_raw;
        respDoc["MOD"] = share_gsr_modifier;
        respDoc["LAT"] = share_lat; 
        respDoc["LON"] = share_lon;
        respDoc["TIME"] = share_gps_time_str;

        serializeJson(respDoc, Serial1);
        Serial1.println(); 
        
        if (!system_ready) Serial.println("[Wait] Searching Satellites...");
    }
    
    // Receive volume parameters from ESP32 (Cloud Analysis Result)
    else if (line.indexOf('{') != -1) {
        int startIdx = line.indexOf('{');
        String jsonStr = line.substring(startIdx);
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, jsonStr);
        if (!error) {
            share_target_void = doc["The Void"] | 0.2;
            share_target_bio  = doc["Bio-Resistance"] | 0.0;
            share_target_tech = doc["Technological Unconscious"] | 0.0;
            share_target_entr = doc["Material Entropy"] | 0.0;
        }
    }
  }
  delay(2); 
}