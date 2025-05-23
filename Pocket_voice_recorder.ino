/*
 * ESP32 Pocket Voice Recorder & Player
 * Uses INMP441 microphone, MAX98357 I2S DAC, SSD1306 display and SD card
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <SD.h>

// Custom max function to avoid type mismatches
template <typename T, typename U>
T custom_max(T a, U b) {
  return (a > static_cast<T>(b)) ? a : static_cast<T>(b);
}

// Custom min function to avoid type mismatches
template <typename T, typename U>
T custom_min(T a, U b) {
  return (a < static_cast<T>(b)) ? a : static_cast<T>(b);
}

// For newer ESP32 Arduino core versions (2.0.0+), use these headers
// If compilation fails, try uncommenting the legacy include instead
//#include <driver/i2s.h>
#include <driver/i2s_std.h>
#include <driver/i2s_pdm.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Define if it's not available in your ESP-IDF version
#ifndef I2S_MCLK_MULTIPLE_DEFAULT
#define I2S_MCLK_MULTIPLE_DEFAULT I2S_MCLK_MULTIPLE_128
#endif

// Add GPIO unused definition if not available
#ifndef I2S_GPIO_UNUSED
#define I2S_GPIO_UNUSED (-1)
#endif

// Fix for ESP32 Arduino 3.x using ESP-IDF 5.x
#if defined(I2S_DATA_BIT_WIDTH_16BIT) && !defined(I2S_BITS_PER_SAMPLE_16BIT)
#define I2S_BITS_PER_SAMPLE_16BIT I2S_DATA_BIT_WIDTH_16BIT
#endif

// Display settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1  // Reset pin not used
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Pin definitions - using actual board labels
// NOTE: If your board uses different labels, adjust these definitions
#define I2S_MIC_BCLK_PIN 32    // G32 on ESP32-WROOM-32 boards
#define I2S_MIC_WS_PIN   25    // G25 on ESP32-WROOM-32 boards
#define I2S_MIC_DATA_PIN 33    // G33 on ESP32-WROOM-32 boards

#define I2S_DAC_BCLK_PIN 26    // G26 on ESP32-WROOM-32 boards
#define I2S_DAC_WS_PIN   27    // G27 on ESP32-WROOM-32 boards (shared with MIC)
#define I2S_DAC_DATA_PIN 14    // G14 on ESP32-WROOM-32 boards
// Optional shutdown pin for MAX98357
// Uncomment if you want to control the amplifier power
//#define I2S_DAC_SD_PIN   21    // G21 on ESP32-WROOM-32 boards

#define BTN_RECORD_PIN   13    // G13 on ESP32-WROOM-32 boards
#define BTN_PLAY_PIN     12    // G12 on ESP32-WROOM-32 boards
#define BTN_NEXT_PIN     15    // G15 on ESP32-WROOM-32 boards

// SD card pins - some modules use different pins
#define SD_MISO_PIN      19    // G19 on ESP32-WROOM-32 boards
#define SD_MOSI_PIN      23    // G23 on ESP32-WROOM-32 boards
#define SD_SCK_PIN       18    // G18 on ESP32-WROOM-32 boards
#define SD_CS_PIN        5     // G5 on ESP32-WROOM-32 boards

// I2S Configuration
#define SAMPLE_RATE      44100  // 44.1kHz for better quality
#define SAMPLE_BITS_I2S  I2S_DATA_BIT_WIDTH_16BIT
#define SAMPLE_BITS      16
#define AUDIO_CHANNELS   1      // Mono

// Microphone settings - adjust these for your specific environment
#define MIC_GAIN         50    // Increased from 2.0 to avoid clipping and distortion
#define NOISE_REDUCTION  true   // Enable noise reduction
#define DC_BLOCKING      true   // Enable DC blocking filter
#define HIGH_PASS_FREQ   60     // Reduced from 80 Hz to allow more voice frequencies
#define NOISE_GATE_THRESHOLD 300 // Reduced from 300 for less aggressive gating

#define I2S_PORT_MIC     I2S_NUM_0
#define I2S_PORT_DAC     I2S_NUM_1
#define DMA_BUF_COUNT    16     // Reduced from 32 to 16 for better compatibility
#define DMA_BUF_LEN      1024   // Increased back to 1024 for more data per read

// Button debounce
#define DEBOUNCE_TIME    50     // Debounce time in ms
#define LONG_PRESS_TIME  1000   // Long press threshold in ms

// Button states
struct ButtonState {
  uint8_t pin;
  bool pressed;
  bool lastState;
  unsigned long pressTime;
  unsigned long lastDebounceTime;
};

// System state
enum SystemState {
  STATE_IDLE,
  STATE_RECORDING,
  STATE_PLAYBACK_SELECT,
  STATE_PLAYBACK,
  STATE_PLAYBACK_PAUSE
};

// Audio buffers
#define RECORD_BUFFER_SIZE (DMA_BUF_LEN * 2)
int16_t recordBuffer[RECORD_BUFFER_SIZE];
int16_t playBuffer[RECORD_BUFFER_SIZE];

// I2S channel handles
i2s_chan_handle_t rx_handle;
i2s_chan_handle_t tx_handle;

// System state
SystemState currentState = STATE_IDLE;

// Button variables (actual definitions)
ButtonState btnRecord = {BTN_RECORD_PIN, false, false, 0, 0};
ButtonState btnPlay = {BTN_PLAY_PIN, false, false, 0, 0};
ButtonState btnNext = {BTN_NEXT_PIN, false, false, 0, 0};

// File management
File recordFile;
File playFile;
char currentFilename[32]; // Increased size
int fileIndex = 1;
int selectedFileIndex = 1;
int totalFiles = 0;
char** wavFileNames = NULL; // Array to store all WAV filenames
unsigned long recordingStartTime = 0;
unsigned long recordingDuration = 0;
unsigned long playbackStartTime = 0;
unsigned long playbackPosition = 0;
unsigned long playbackDuration = 0;

// Display management
unsigned long lastActivityTime = 0;
bool displayOn = true;
#define DISPLAY_TIMEOUT 30000  // 30 seconds

// For improved audio quality - INMP441 is left-justified
#define I2S_MIC_LEFT_JUSTIFIED true

// Filter coefficients - precomputed for efficiency
float highPassAlpha = 0.0;  // Will be computed in setup

// Function prototypes
void updateButtonState(ButtonState* btn);
void setupI2S();
void setupButtons();
void setupDisplay();
void setupSDCard();
void handleButtons();
void startRecording();
void stopRecording();
void startPlayback();
void pausePlayback();
void continuePlayback();
void stopPlayback();
void navigateFiles();
void updateDisplay();
void countWavFiles();
void createWavHeader(File file, uint32_t sampleRate, uint16_t numChannels, uint16_t bitsPerSample, uint32_t dataSize);
void parseWavHeader(File file, uint32_t* sampleRate, uint32_t* dataSize);
void enterLightSleep();
void formatTime(unsigned long seconds, char* buffer);
void printDirectory(File dir, int numTabs);

// Helper function for case-insensitive string comparison
bool endsWithCaseInsensitive(const String& str, const String& suffix) {
  if (str.length() < suffix.length()) {
    return false;
  }
  
  int strOffset = str.length() - suffix.length();
  for (size_t i = 0; i < suffix.length(); i++) {
    if (tolower(str.charAt(strOffset + i)) != tolower(suffix.charAt(i))) {
      return false;
    }
  }
  return true;
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Pocket Voice Recorder starting...");
  
  // Compute filter coefficients based on sample rate
  // High-pass filter: alpha = RC/(RC + dt) where RC = 1/(2*PI*cutoff_freq)
  float dt = 1.0f / SAMPLE_RATE;
  float RC = 1.0f / (2.0f * PI * HIGH_PASS_FREQ);
  highPassAlpha = RC / (RC + dt);
  Serial.printf("High-pass filter coefficient: %f for %d Hz cutoff\n", 
                 highPassAlpha, HIGH_PASS_FREQ);
  
  // Optional shutdown pin for MAX98357
  #ifdef I2S_DAC_SD_PIN
  pinMode(I2S_DAC_SD_PIN, OUTPUT);
  digitalWrite(I2S_DAC_SD_PIN, HIGH);  // Enable the amplifier
  #endif
  
  // Print pin configurations for debugging
  Serial.println("Pin Configuration:");
  Serial.printf("SD_MISO_PIN: %d\n", SD_MISO_PIN);
  Serial.printf("SD_MOSI_PIN: %d\n", SD_MOSI_PIN);
  Serial.printf("SD_SCK_PIN: %d\n", SD_SCK_PIN);
  Serial.printf("SD_CS_PIN: %d\n", SD_CS_PIN);
  
  setupButtons();
  setupDisplay();
  setupSDCard();
  
  // Only configure I2S if SD card initialized successfully
  if (SD.begin()) {
    setupI2S();
    countWavFiles();
  } else {
    // Show error but continue with limited functionality
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("SD CARD ERROR");
    display.println("Recording disabled");
    display.println("Check connections");
    display.display();
    delay(3000);
  }
  
  updateDisplay();
  
  lastActivityTime = millis();
  Serial.println("Initialization complete");
}

void loop() {
  handleButtons();
  
  switch (currentState) {
    case STATE_RECORDING:
      processRecording();
      // Small yield to allow other system tasks to run
      yield();
      break;
      
    case STATE_PLAYBACK:
      processPlayback();
      // Small yield to allow other system tasks to run  
      yield();
      break;
      
    case STATE_IDLE:
    case STATE_PLAYBACK_SELECT:
    case STATE_PLAYBACK_PAUSE:
      // Check for display timeout
      if (displayOn && (millis() - lastActivityTime > DISPLAY_TIMEOUT)) {
        display.clearDisplay();
        display.display();
        displayOn = false;
      }
      
      // Go to light sleep if idle for longer
      if (millis() - lastActivityTime > DISPLAY_TIMEOUT * 2) {
        enterLightSleep();
      }
      
      // Give more time for system tasks when idle
      delay(10);
      break;
  }
}

void setupButtons() {
  pinMode(BTN_RECORD_PIN, INPUT_PULLUP);
  pinMode(BTN_PLAY_PIN, INPUT_PULLUP);
  pinMode(BTN_NEXT_PIN, INPUT_PULLUP);
}

void setupDisplay() {
  Wire.begin();
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("SSD1306 allocation failed");
    for(;;); // Don't proceed, loop forever
  }
  
  display.setRotation(2); // Rotate display 180 degrees
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Pocket Recorder");
  display.println("Initializing...");
  display.display();
  delay(1000);
}

void setupSDCard() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Initializing SD card...");
  display.display();
  
  // Initialize SPI for SD card with explicit pin assignments
  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  
  // First, make sure the CS pin is set up properly - add more delay for stability
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  delay(250); // Longer delay to allow SD card to stabilize
  
  // Try different initialization methods
  bool sdInitialized = false;
  
  // First try regular SD.begin with 500ms timeout
  Serial.println("Trying SD init at default speed");
  if (SD.begin(SD_CS_PIN, SPI, 4000000, "/sd", 5, false)) {
    sdInitialized = true;
    Serial.println("SD initialized at 4MHz speed");
  } else {
    // If that failed, try with lower SPI speed (1MHz)
    Serial.println("SD init failed, trying 1MHz speed");
    SPI.setFrequency(1000000);
    if (SD.begin(SD_CS_PIN, SPI, 1000000, "/sd", 5, false)) {
      sdInitialized = true;
      Serial.println("SD initialized at 1MHz speed");
    } else {
      // Try with even lower speed (500kHz)
      Serial.println("SD init failed again, trying 500kHz");
      SPI.setFrequency(500000);
      
      // Attempt SD initialization multiple times
      for (int attempt = 0; attempt < 3; attempt++) {
        Serial.printf("SD initialization attempt %d at 500kHz\n", attempt+1);
        if (SD.begin(SD_CS_PIN, SPI, 500000, "/sd", 5, false)) {
          sdInitialized = true;
          Serial.println("SD initialized successfully after multiple attempts");
          break;
        }
        delay(500); // Wait between attempts
      }
    }
  }
  
  if (!sdInitialized) {
    Serial.println("SD card initialization failed!");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("SD CARD ERROR");
    display.println("Please check:");
    display.println("- Card inserted properly");
    display.println("- FAT32 formatted card");
    display.println("- Wiring connections:");
    display.println("  MISO:" + String(SD_MISO_PIN));
    display.println("  MOSI:" + String(SD_MOSI_PIN));
    display.println("  SCK:" + String(SD_SCK_PIN));
    display.println("  CS:" + String(SD_CS_PIN));
    display.display();
    delay(3000);
  } else {
    Serial.println("SD card initialized.");
    display.println("SD card ready.");
    display.display();
    
    // Check if card is write protected or read-only filesystem
    boolean isWritable = false;
    
    // Try to create a test file in different locations
    const char* testPaths[] = {"/test.txt", "test.txt", "/sd/test.txt"};
    
    for (int i = 0; i < 3; i++) {
      Serial.printf("Trying to create test file at: %s\n", testPaths[i]);
      File testFile = SD.open(testPaths[i], FILE_WRITE);
      
      if (testFile) {
        if (testFile.print("Test file")) {
          isWritable = true;
          Serial.printf("Successfully wrote to %s\n", testPaths[i]);
          testFile.close();
          SD.remove(testPaths[i]);
          break;
        } else {
          Serial.printf("Could open %s but couldn't write to it\n", testPaths[i]);
          testFile.close();
        }
      } else {
        Serial.printf("Failed to create test file at %s\n", testPaths[i]);
      }
    }
    
    if (!isWritable) {
      Serial.println("!!! SD CARD IS READ-ONLY OR WRITE PROTECTED !!!");
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("SD CARD ERROR");
      display.println("Card is read-only!");
      display.println("Check write-protect");
      display.println("switch on SD card");
      display.println("or reformat card.");
      display.display();
      delay(3000);
    } else {
      // Try to get SD card info
      uint8_t cardType = SD.cardType();
      String cardTypeStr = "Unknown";
      if(cardType == CARD_NONE) {
        cardTypeStr = "None";
      } else if(cardType == CARD_MMC) {
        cardTypeStr = "MMC";
      } else if(cardType == CARD_SD) {
        cardTypeStr = "SD";
      } else if(cardType == CARD_SDHC) {
        cardTypeStr = "SDHC";
      }
      
      uint64_t cardSize = SD.cardSize() / (1024 * 1024);
      Serial.printf("SD Card Type: %s\n", cardTypeStr.c_str());
      Serial.printf("SD Card Size: %lluMB\n", cardSize);
      
      display.println();
      display.print("Type: ");
      display.println(cardTypeStr);
      display.print("Size: ");
      display.print(cardSize);
      display.println("MB");
      display.display();
      delay(1000);
    }
  }
}

void setupI2S() {
  Serial.println("Setting up I2S with new API...");

  // I2S Configuration for Microphone (I2S_NUM_0)
  i2s_chan_config_t i2s_mic_chan_cfg = {
    .id = I2S_NUM_0,
    .role = I2S_ROLE_MASTER,
    .dma_desc_num = DMA_BUF_COUNT,
    .dma_frame_num = DMA_BUF_LEN,
    .auto_clear = false,      // Changed back to false
    .auto_clear_before_cb = false,
    .allow_pd = false,
    .intr_priority = 0        // Changed back to 0 for standard priority
  };
  
  // Specific INMP441 configuration
  // The INMP441 microphone works best in standard I2S mode with the following settings:
  // - Left justified format (bit_shift = true for INMP441)
  // - The microphone only puts data on the L channel, so use ONLY_LEFT
  i2s_std_config_t i2s_mic_std_cfg = {
    .clk_cfg = {
      .sample_rate_hz = SAMPLE_RATE,
      .clk_src = I2S_CLK_SRC_DEFAULT,
      .mclk_multiple = I2S_MCLK_MULTIPLE_DEFAULT  // Changed back to DEFAULT
    },
    .slot_cfg = {
      .data_bit_width = SAMPLE_BITS_I2S,
      .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
      .slot_mode = I2S_SLOT_MODE_MONO,
      .slot_mask = I2S_STD_SLOT_LEFT,
      .ws_width = 16,         // Changed back to 16
      .ws_pol = false,
      .bit_shift = I2S_MIC_LEFT_JUSTIFIED, // For INMP441 left-justified mode
      .msb_right = false
    },
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = (gpio_num_t)I2S_MIC_BCLK_PIN,
      .ws = (gpio_num_t)I2S_MIC_WS_PIN,
      .dout = I2S_GPIO_UNUSED,
      .din = (gpio_num_t)I2S_MIC_DATA_PIN,
      .invert_flags = {
        .mclk_inv = false,
        .bclk_inv = false,
        .ws_inv = false
      }
    }
  };
  
  Serial.println("Installing I2S driver for mic...");
  ESP_ERROR_CHECK(i2s_new_channel(&i2s_mic_chan_cfg, NULL, &rx_handle));
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle, &i2s_mic_std_cfg));
  ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));
  
  // Small delay to allow the I2S peripheral to stabilize
  delay(100);
  
  // I2S Configuration for DAC (I2S_NUM_1) - MAX98357
  i2s_chan_config_t i2s_dac_chan_cfg = {
    .id = I2S_NUM_1,
    .role = I2S_ROLE_MASTER,
    .dma_desc_num = DMA_BUF_COUNT,
    .dma_frame_num = DMA_BUF_LEN,
    .auto_clear = true,
    .auto_clear_before_cb = false,
    .allow_pd = false,
    .intr_priority = 0        // Changed back to 0
  };
  
  i2s_std_config_t i2s_dac_std_cfg = {
    .clk_cfg = {
      .sample_rate_hz = SAMPLE_RATE,
      .clk_src = I2S_CLK_SRC_DEFAULT,
      .mclk_multiple = I2S_MCLK_MULTIPLE_DEFAULT  // Changed back to DEFAULT
    },
    .slot_cfg = {
      .data_bit_width = SAMPLE_BITS_I2S,
      .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
      .slot_mode = I2S_SLOT_MODE_MONO,
      .slot_mask = I2S_STD_SLOT_LEFT,
      .ws_width = 16,         // Changed back to 16
      .ws_pol = false,
      .bit_shift = false,
      .msb_right = false
    },
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = (gpio_num_t)I2S_DAC_BCLK_PIN,
      .ws = (gpio_num_t)I2S_DAC_WS_PIN,
      .dout = (gpio_num_t)I2S_DAC_DATA_PIN,
      .din = I2S_GPIO_UNUSED,
      .invert_flags = {
        .mclk_inv = false,
        .bclk_inv = false,
        .ws_inv = false
      }
    }
  };
  
  Serial.println("Installing I2S driver for DAC...");
  ESP_ERROR_CHECK(i2s_new_channel(&i2s_dac_chan_cfg, &tx_handle, NULL));
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &i2s_dac_std_cfg));
  ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));
  
  Serial.println("I2S setup complete");
}

void handleButtons() {
  // Update button states with debouncing
  updateButtonState(&btnRecord);
  updateButtonState(&btnPlay);
  updateButtonState(&btnNext);
  
  // Handle button actions based on system state
  if (btnRecord.pressed) {
    lastActivityTime = millis();
    if (!displayOn) {
      displayOn = true;
      updateDisplay();
    } else {
      switch (currentState) {
        case STATE_IDLE:
          startRecording();
          break;
        case STATE_RECORDING:
          stopRecording();
          break;
        case STATE_PLAYBACK:
        case STATE_PLAYBACK_PAUSE:
        case STATE_PLAYBACK_SELECT:  // Added to handle all enum values
          stopPlayback();
          break;
      }
    }
  }
  
  if (btnPlay.pressed) {
    lastActivityTime = millis();
    if (!displayOn) {
      displayOn = true;
      updateDisplay();
    } else {
      switch (currentState) {
        case STATE_IDLE:
          // If we have files, go to file selection mode first
          if (totalFiles > 0) {
            currentState = STATE_PLAYBACK_SELECT;
            updateDisplay();
          }
          break;
        case STATE_PLAYBACK_SELECT:
          // Start playback of selected file
          if (totalFiles > 0) {
            startPlayback();
          }
          break;
        case STATE_PLAYBACK:
          pausePlayback();
          break;
        case STATE_PLAYBACK_PAUSE:
          continuePlayback();
          break;
        case STATE_RECORDING:  // Added to handle all enum values
          // No action when recording
          break;
      }
    }
  }
  
  if (btnNext.pressed) {
    lastActivityTime = millis();
    if (!displayOn) {
      displayOn = true;
      updateDisplay();
    } else if (currentState == STATE_IDLE || currentState == STATE_PLAYBACK_SELECT) {
      navigateFiles();
    }
  }
}

void updateButtonState(ButtonState* btn) {
  bool reading = !digitalRead(btn->pin);  // Buttons are active low (pulled up)
  
  // If the button state changed, reset the debounce timer
  if (reading != btn->lastState) {
    btn->lastDebounceTime = millis();
  }
  
  btn->pressed = false;  // Reset pressed state
  
  // If the reading has been stable for longer than debounce time
  if ((millis() - btn->lastDebounceTime) > DEBOUNCE_TIME) {
    // If the button is pressed (and wasn't before)
    if (reading && !btn->pressed) {
      btn->pressTime = millis();  // Record the press time
    }
    
    // If the button is released and was pressed before
    if (!reading && btn->pressTime > 0) {
      // Check for short/long press
      unsigned long pressDuration = millis() - btn->pressTime;
      if (pressDuration < LONG_PRESS_TIME) {
        btn->pressed = true;  // Signal a short press
      } else {
        // Handle long press if needed
      }
      btn->pressTime = 0;  // Reset the press time
    }
  }
  
  btn->lastState = reading;  // Save the current state for next comparison
}

void startRecording() {
  // Don't try to record if SD card isn't working
  if (!SD.begin(SD_CS_PIN, SPI, 4000000, "/sd", 5, false)) {
    Serial.println("Cannot start recording - SD card not available");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("SD CARD ERROR");
    display.println("Can't record");
    display.display();
    delay(2000);
    return;
  }

  // Create a new WAV file
  char filename[32]; // Increased buffer size
  
  // Try with and without leading slash
  sprintf(filename, "/REC%03d.WAV", fileIndex);
  Serial.printf("Creating file: %s\n", filename);
  
  // First check if file already exists and remove it
  if (SD.exists(filename)) {
    Serial.printf("File %s already exists, removing...\n", filename);
    SD.remove(filename);
  }
  
  // Try to create the root directory if it doesn't exist
  if (!SD.exists("/")) {
    Serial.println("Creating root directory");
    SD.mkdir("/");
  }
  
  // Open file for writing with error handling
  recordFile = SD.open(filename, FILE_WRITE);
  
  // If that failed, try without leading slash
  if (!recordFile) {
    sprintf(filename, "REC%03d.WAV", fileIndex);
    Serial.printf("First attempt failed. Trying without leading slash: %s\n", filename);
    
    if (SD.exists(filename)) {
      Serial.printf("File %s already exists, removing...\n", filename);
      SD.remove(filename);
    }
    
    recordFile = SD.open(filename, FILE_WRITE);
  }
  
  // If still failed, try in root directory
  if (!recordFile) {
    Serial.println("Still failed. Trying to list root directory contents:");
    File root = SD.open("/");
    if (root) {
      while (File file = root.openNextFile()) {
        Serial.print("Found file: ");
        Serial.println(file.name());
        file.close();
      }
      root.close();
    } else {
      Serial.println("Failed to open root directory!");
    }
    
    // One last attempt with FILE_WRITE mode explicitly
    recordFile = SD.open(filename, FILE_WRITE);
  }
  
  if (!recordFile) {
    Serial.println("Failed to create recording file after multiple attempts!");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("SD ERROR");
    display.println("Can't create file!");
    display.println("Try power cycling");
    display.println("or reformatting SD");
    display.display();
    delay(2000);
    return;
  }
  
  // Store filename for display (without the leading slash)
  if (filename[0] == '/') {
    strcpy(currentFilename, filename + 1);
  } else {
    strcpy(currentFilename, filename);
  }
  
  // Write placeholder WAV header (will be updated later)
  uint32_t sampleRate = SAMPLE_RATE;
  uint16_t numChannels = AUDIO_CHANNELS;
  uint16_t bitsPerSample = SAMPLE_BITS;
  createWavHeader(recordFile, sampleRate, numChannels, bitsPerSample, 0);
  
  // Reset recording buffer and state
  memset(recordBuffer, 0, sizeof(recordBuffer));
  recordingStartTime = millis();
  currentState = STATE_RECORDING;
  
  Serial.print("Recording started: ");
  Serial.println(currentFilename);
  updateDisplay();
}

void processRecording() {
  size_t bytes_read = 0;
  
  // Use portMAX_DELAY to ensure we get data (blocking wait)
  esp_err_t result = i2s_channel_read(rx_handle, recordBuffer, sizeof(recordBuffer), &bytes_read, portMAX_DELAY);
  
  if (result != ESP_OK) {
    Serial.printf("I2S read error: %d\n", result);
    return;
  }
  
  if (bytes_read > 0) {
    int numSamples = bytes_read / sizeof(int16_t);
    
    // Log the actual bytes read occasionally
    static uint32_t lastLogTime = 0;
    if (millis() - lastLogTime > 2000) {
      Serial.printf("Recording: Read %d bytes (%d samples)\n", bytes_read, numSamples);
      lastLogTime = millis();
    }
    
    // Basic audio processing - just enough to improve quality
    float sum = 0;
    float sumSquares = 0;
    
    // Calculate DC offset and RMS
    for (int i = 0; i < numSamples; i++) {
      float sample = recordBuffer[i];
      sum += sample;
      sumSquares += sample * sample;
    }
    
    float dcOffset = sum / numSamples;
    float rms = sqrtf(sumSquares / numSamples);
    
    // Only process if there's something to process
    if (rms > 50) {
      // Apply simple processing: DC offset removal and gain
      for (int i = 0; i < numSamples; i++) {
        // Remove DC offset
        float sample = recordBuffer[i] - dcOffset;
        
        // Apply gain
        sample *= MIC_GAIN;
        
        // Simple clipping protection
        if (sample > 32000.0f) sample = 32000.0f;
        if (sample < -32000.0f) sample = -32000.0f;
        
        recordBuffer[i] = (int16_t)sample;
      }
    }
    
    // Always write data to the file, even if quiet (to maintain timing)
    size_t bytesWritten = recordFile.write((uint8_t*)recordBuffer, bytes_read);
    if (bytesWritten != bytes_read) {
      Serial.printf("SD write warning: Only wrote %d of %d bytes\n", bytesWritten, bytes_read);
    }
    
    // Update display occasionally (not every buffer to reduce overhead)
    static uint32_t lastDisplayUpdate = 0;
    if (millis() - lastDisplayUpdate > 500) {
      updateDisplay();
      lastDisplayUpdate = millis();
    }
  } else {
    // If we got 0 bytes, that's abnormal - report it
    Serial.println("Warning: I2S read returned 0 bytes!");
  }
}

void stopRecording() {
  if (currentState != STATE_RECORDING) return;
  
  // Calculate total recording time
  recordingDuration = (millis() - recordingStartTime) / 1000;
  
  // Update the WAV header with the final file size
  uint32_t fileSize = recordFile.size();
  uint32_t dataSize = fileSize - 44;  // Subtract header size
  
  recordFile.seek(0);
  createWavHeader(recordFile, SAMPLE_RATE, AUDIO_CHANNELS, SAMPLE_BITS, dataSize);
  recordFile.close();
  
  // Update file index for next recording
  fileIndex++;
  
  // Update file count
  countWavFiles();
  
  // Update system state
  currentState = STATE_IDLE;
  Serial.println("Recording stopped");
  updateDisplay();
}

void startPlayback() {
  // Determine if we're playing a custom filename or one of our recorded files
  char filename[32]; // Increased buffer size
  
  // Check if we should try direct playback of a known file on the SD card
  if (currentState == STATE_IDLE && selectedFileIndex <= totalFiles && selectedFileIndex > 0 && wavFileNames != NULL) {
    // Try to play the file directly with its exact name
    Serial.printf("Attempting direct playback of: %s\n", wavFileNames[selectedFileIndex - 1]);
    playFile = SD.open(wavFileNames[selectedFileIndex - 1], FILE_READ);
    
    if (playFile) {
      strcpy(currentFilename, wavFileNames[selectedFileIndex - 1]);
      Serial.printf("Successfully opened file using direct name: %s\n", currentFilename);
      
      // Skip the rest of the path searching logic
      goto file_opened;
    }
  }
  
  // In playback selection mode, we're browsing recorded files
  if (currentState == STATE_PLAYBACK_SELECT) {
    if (wavFileNames != NULL && selectedFileIndex > 0 && selectedFileIndex <= totalFiles) {
      strcpy(filename, wavFileNames[selectedFileIndex - 1]);
    } else {
      sprintf(filename, "REC%03d.WAV", selectedFileIndex);
    }
  } else {
    // We're playing a custom file
    // First check if we have a custom filename stored
    if (strlen(currentFilename) > 0 && 
        (strncmp(currentFilename, "REC", 3) != 0 || currentState == STATE_PLAYBACK)) {
      // Use the custom filename that was already selected
      strcpy(filename, currentFilename);
    } else {
      // Default to most recent recording
      sprintf(filename, "REC%03d.WAV", selectedFileIndex);
    }
  }
  
  Serial.printf("Opening file for playback: %s\n", filename);
  playFile = SD.open(filename, FILE_READ);
  
  // Also try lowercase extension
  if (!playFile) {
    char lowerFilename[32];
    strcpy(lowerFilename, filename);
    
    // Convert extension to lowercase
    char* extPos = strrchr(lowerFilename, '.');
    if (extPos) {
      for (char* p = extPos; *p; p++) {
        *p = tolower(*p);
      }
      Serial.printf("Trying with lowercase extension: %s\n", lowerFilename);
      playFile = SD.open(lowerFilename, FILE_READ);
      
      if (playFile) {
        strcpy(filename, lowerFilename);
      }
    }
  }
  
  // If that failed, try with leading slash
  if (!playFile) {
    char pathWithSlash[32];
    sprintf(pathWithSlash, "/%s", filename);
    Serial.printf("Failed with first path. Trying with leading slash: %s\n", pathWithSlash);
    playFile = SD.open(pathWithSlash, FILE_READ);
    
    if (playFile) {
      strcpy(filename, pathWithSlash);
    }
  }
  
  // If that failed, try with /sd/ prefix
  if (!playFile) {
    char sdPath[32];
    sprintf(sdPath, "/sd/%s", filename);
    Serial.printf("Failed again. Trying with /sd/ prefix: %s\n", sdPath);
    playFile = SD.open(sdPath, FILE_READ);
    
    if (playFile) {
      strcpy(filename, sdPath);
    }
  }
  
  // Debug - list directory to find files
  if (!playFile) {
    Serial.println("Still failed to open file. Checking available files:");
    
    File root = SD.open("/");
    if (root) {
      printDirectory(root, 0);
      root.close();
    } else {
      root = SD.open("");
      if (root) {
        printDirectory(root, 0);
        root.close();
      } else {
        Serial.println("Failed to open any directory for listing");
      }
    }
  }
  
  if (!playFile) {
    Serial.print("Failed to open file for playback after multiple attempts\n");
    
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("File Open Error");
    display.println(filename);
    display.display();
    delay(2000);
    return;
  }
  
file_opened:
  // Store filename for display (strip leading path if present)
  char* filenameOnly = strrchr(filename, '/');
  if (filenameOnly) {
    strcpy(currentFilename, filenameOnly + 1);
  } else {
    strcpy(currentFilename, filename);
  }
  
  // Read and parse WAV header
  uint32_t sampleRate;
  uint32_t dataSize;
  parseWavHeader(playFile, &sampleRate, &dataSize);
  
  // Calculate playback duration in seconds
  playbackDuration = dataSize / (SAMPLE_RATE * (SAMPLE_BITS/8) * AUDIO_CHANNELS);
  playbackStartTime = millis();
  playbackPosition = 0;
  
  // Start playback
  currentState = STATE_PLAYBACK;
  Serial.print("Playback started: ");
  Serial.println(currentFilename);
  updateDisplay();
}

void processPlayback() {
  if (currentState != STATE_PLAYBACK) return;
  
  // Read audio data from file
  size_t bytesRead = playFile.read((uint8_t*)playBuffer, sizeof(playBuffer));
  
  if (bytesRead > 0) {
    // Send audio data to DAC with full timeout
    size_t bytes_written = 0;
    esp_err_t result = i2s_channel_write(tx_handle, playBuffer, bytesRead, &bytes_written, portMAX_DELAY);
    
    if (result != ESP_OK) {
      Serial.printf("I2S write error: %d\n", result);
    }
    
    // Update playback position
    playbackPosition = (millis() - playbackStartTime) / 1000;
    
    // Update display occasionally
    static uint32_t lastDisplayUpdate = 0;
    if (millis() - lastDisplayUpdate > 500) {
      updateDisplay();
      lastDisplayUpdate = millis();
    }
  } else {
    // End of file reached
    stopPlayback();
  }
}

void pausePlayback() {
  if (currentState != STATE_PLAYBACK) return;
  
  // Calculate current position
  playbackPosition = (millis() - playbackStartTime) / 1000;
  currentState = STATE_PLAYBACK_PAUSE;
  updateDisplay();
}

void continuePlayback() {
  if (currentState != STATE_PLAYBACK_PAUSE) return;
  
  // Update start time to maintain correct position
  playbackStartTime = millis() - (playbackPosition * 1000);
  currentState = STATE_PLAYBACK;
  updateDisplay();
}

void stopPlayback() {
  if (playFile) {
    playFile.close();
  }
  
  currentState = STATE_IDLE;
  updateDisplay();
}

void navigateFiles() {
  if (totalFiles == 0) return;
  
  selectedFileIndex++;
  if (selectedFileIndex > totalFiles) {
    selectedFileIndex = 1;
  }
  
  // Update current filename with the selected WAV file
  if (wavFileNames != NULL && selectedFileIndex > 0 && selectedFileIndex <= totalFiles) {
    strcpy(currentFilename, wavFileNames[selectedFileIndex - 1]);
    Serial.printf("Selected file: %s\n", currentFilename);
  }
  
  // If we're in IDLE mode, allow direct browsing of all files
  if (currentState == STATE_IDLE && totalFiles > 0) {
    // Switch to playback selection mode
    currentState = STATE_PLAYBACK_SELECT;
  } else {
    currentState = STATE_PLAYBACK_SELECT;
  }
  
  updateDisplay();
}

void updateDisplay() {
  if (!displayOn) {
    displayOn = true;
  }
  
  display.clearDisplay();
  display.setCursor(0, 0);
  
  switch (currentState) {
    case STATE_IDLE:
      display.println("Pocket Recorder");
      display.println("");
      if (totalFiles > 0) {
        display.println("Files: " + String(totalFiles));
        display.println("Press PLAY to browse");
        display.println("Press REC to record");
      } else {
        display.println("No recordings found");
        display.println("Press REC to start");
      }
      break;
      
    case STATE_RECORDING:
      {
        unsigned long elapsedSec = (millis() - recordingStartTime) / 1000;
        char timeStr[10];
        formatTime(elapsedSec, timeStr);
        
        display.println("RECORDING");
        display.println(currentFilename);
        display.println("");
        display.setTextSize(2);
        display.println(timeStr);
        display.setTextSize(1);
      }
      break;
      
    case STATE_PLAYBACK_SELECT:
      display.println("SELECT RECORDING");
      display.println("");
      display.println(String(selectedFileIndex) + "/" + String(totalFiles));
      
      // Display the current selected filename
      if (wavFileNames != NULL && selectedFileIndex > 0 && selectedFileIndex <= totalFiles) {
        // Show filename, truncate if needed
        String showName = wavFileNames[selectedFileIndex - 1];
        if (showName.length() > 21) { // OLED is about 21 chars wide with this font
          showName = showName.substring(0, 18) + "...";
        }
        display.println(showName);
      } else {
        display.println("REC" + String(selectedFileIndex, 3) + ".WAV");
      }
      
      display.println("");
      display.println("PLAY to select");
      display.println("NEXT to browse");
      break;
      
    case STATE_PLAYBACK:
    case STATE_PLAYBACK_PAUSE:
      {
        char currTimeStr[10];
        char totalTimeStr[10];
        formatTime(playbackPosition, currTimeStr);
        formatTime(playbackDuration, totalTimeStr);
        
        display.println(currentState == STATE_PLAYBACK ? "PLAYING" : "PAUSED");
        
        // Show the filename, truncate if needed
        String showName = currentFilename;
        if (showName.length() > 21) { // OLED is about 21 chars wide with this font
          showName = showName.substring(0, 18) + "...";
        }
        display.println(showName);
        
        display.println("");
        display.println(String(currTimeStr) + "/" + String(totalTimeStr));
        
        // Draw progress bar
        int barWidth = map(playbackPosition, 0, playbackDuration, 0, display.width() - 4);
        display.drawRect(0, 40, display.width(), 8, SSD1306_WHITE);
        display.fillRect(2, 42, barWidth, 4, SSD1306_WHITE);
      }
      break;
  }
  
  display.display();
}

void countWavFiles() {
  // First, free previous storage if any
  if (wavFileNames != NULL) {
    for (int i = 0; i < totalFiles; i++) {
      if (wavFileNames[i] != NULL) {
        free(wavFileNames[i]);
      }
    }
    free(wavFileNames);
    wavFileNames = NULL;
  }
  
  totalFiles = 0;
  int maxFiles = 20; // Start with space for 20 files
  wavFileNames = (char**)malloc(maxFiles * sizeof(char*));
  if (wavFileNames == NULL) {
    Serial.println("Memory allocation failed for wavFileNames array!");
    return;
  }
  
  // Initialize pointers to NULL
  for (int i = 0; i < maxFiles; i++) {
    wavFileNames[i] = NULL;
  }
  
  // Try to open root directory
  File root = SD.open("/");
  
  if (!root) {
    Serial.println("Failed to open root directory!");
    
    // Try alternative paths
    root = SD.open("");
    if (!root) {
      root = SD.open("/sd");
      if (!root) {
        Serial.println("Could not open any directory - SD card may not be mounted correctly");
        return;
      } else {
        Serial.println("Opened '/sd' directory instead of root");
      }
    } else {
      Serial.println("Opened empty path directory");
    }
  }
  
  Serial.println("Listing files in directory:");
  int fileCount = 0;
  int recFileCount = 0;
  
  while (true) {
    File entry = root.openNextFile();
    if (!entry) {
      if (fileCount == 0) {
        Serial.println("No files found in directory!");
      }
      break;
    }
    
    fileCount++;
    String filename = entry.name();
    if (entry.isDirectory()) {
      Serial.printf("  DIR: %s\n", filename.c_str());
    } else {
      Serial.printf("  FILE: %s (%d bytes)\n", filename.c_str(), entry.size());
      
      // Case-insensitive check for .WAV files using our helper function
      if (endsWithCaseInsensitive(filename, ".wav")) {
        // Resize array if needed
        if (totalFiles >= maxFiles) {
          maxFiles *= 2;
          char** newArray = (char**)realloc(wavFileNames, maxFiles * sizeof(char*));
          if (newArray == NULL) {
            Serial.println("Memory reallocation failed!");
            entry.close();
            break;
          }
          wavFileNames = newArray;
          // Initialize new pointers to NULL
          for (int i = totalFiles; i < maxFiles; i++) {
            wavFileNames[i] = NULL;
          }
        }
        
        // Allocate memory for this filename and copy it
        wavFileNames[totalFiles] = (char*)malloc(filename.length() + 1);
        if (wavFileNames[totalFiles] != NULL) {
          strcpy(wavFileNames[totalFiles], filename.c_str());
          totalFiles++;
          
          // Check if it's one of our recorded files - use case-insensitive prefix check
          if (filename.length() >= 3 && 
              tolower(filename.charAt(0)) == 'r' &&
              tolower(filename.charAt(1)) == 'e' &&
              tolower(filename.charAt(2)) == 'c') {
            recFileCount++;
            String numPart = "";
            for (int i = 3; i < filename.length(); i++) {
              if (isDigit(filename[i])) {
                numPart += filename[i];
              } else {
                break;
              }
            }
            
            if (numPart.length() > 0) {
              int thisIndex = numPart.toInt();
              if (thisIndex >= fileIndex) {
                fileIndex = thisIndex + 1;
                Serial.printf("Found file with index %d, next will be %d\n", thisIndex, fileIndex);
              }
            }
          }
        } else {
          Serial.println("Failed to allocate memory for filename!");
        }
      }
    }
    entry.close();
  }
  
  root.close();
  
  // Sort filenames alphabetically for easier browsing
  if (totalFiles > 1) {
    for (int i = 0; i < totalFiles - 1; i++) {
      for (int j = i + 1; j < totalFiles; j++) {
        if (strcmp(wavFileNames[i], wavFileNames[j]) > 0) {
          char* temp = wavFileNames[i];
          wavFileNames[i] = wavFileNames[j];
          wavFileNames[j] = temp;
        }
      }
    }
  }
  
  selectedFileIndex = min(selectedFileIndex, totalFiles);
  if (totalFiles == 0) {
    selectedFileIndex = 1;
    fileIndex = 1;
  }
  
  Serial.printf("Found %d WAV files (%d REC files), next index will be %d\n", 
                totalFiles, recFileCount, fileIndex);
}

void createWavHeader(File file, uint32_t sampleRate, uint16_t numChannels, uint16_t bitsPerSample, uint32_t dataSize) {
  uint32_t byteRate = sampleRate * numChannels * (bitsPerSample / 8);
  uint16_t blockAlign = numChannels * (bitsPerSample / 8);
  uint32_t totalSize = 36 + dataSize;
  
  uint8_t header[44] = {
    // RIFF chunk
    'R', 'I', 'F', 'F',                                   // RIFF identifier
    (uint8_t)(totalSize & 0xff),                          // file size in bytes
    (uint8_t)((totalSize >> 8) & 0xff),
    (uint8_t)((totalSize >> 16) & 0xff),
    (uint8_t)((totalSize >> 24) & 0xff),
    'W', 'A', 'V', 'E',                                   // WAVE identifier
    
    // fmt sub-chunk
    'f', 'm', 't', ' ',                                   // fmt identifier
    16, 0, 0, 0,                                          // subchunk size (16 bytes)
    1, 0,                                                 // audio format (1 = PCM)
    (uint8_t)(numChannels & 0xff),                        // number of channels
    (uint8_t)((numChannels >> 8) & 0xff),
    (uint8_t)(sampleRate & 0xff),                         // sample rate
    (uint8_t)((sampleRate >> 8) & 0xff),
    (uint8_t)((sampleRate >> 16) & 0xff),
    (uint8_t)((sampleRate >> 24) & 0xff),
    (uint8_t)(byteRate & 0xff),                           // byte rate
    (uint8_t)((byteRate >> 8) & 0xff),
    (uint8_t)((byteRate >> 16) & 0xff),
    (uint8_t)((byteRate >> 24) & 0xff),
    (uint8_t)(blockAlign & 0xff),                         // block align
    (uint8_t)((blockAlign >> 8) & 0xff),
    (uint8_t)(bitsPerSample & 0xff),                      // bits per sample
    (uint8_t)((bitsPerSample >> 8) & 0xff),
    
    // data sub-chunk
    'd', 'a', 't', 'a',                                   // data identifier
    (uint8_t)(dataSize & 0xff),                           // data size in bytes
    (uint8_t)((dataSize >> 8) & 0xff),
    (uint8_t)((dataSize >> 16) & 0xff),
    (uint8_t)((dataSize >> 24) & 0xff)
  };
  
  file.seek(0);
  file.write(header, sizeof(header));
}

void parseWavHeader(File file, uint32_t* sampleRate, uint32_t* dataSize) {
  uint8_t header[44];
  file.seek(0);
  file.read(header, sizeof(header));
  
  // Extract sample rate from header bytes 24-27
  *sampleRate = header[24] | (header[25] << 8) | (header[26] << 16) | (header[27] << 24);
  
  // Extract data size from header bytes 40-43
  *dataSize = header[40] | (header[41] << 8) | (header[42] << 16) | (header[43] << 24);
  
  // Position file at the beginning of data (past header)
  file.seek(44);
}

void enterLightSleep() {
  Serial.println("Entering light sleep");
  
  // Configure wake sources
  esp_sleep_enable_ext0_wakeup(gpio_num_t(BTN_RECORD_PIN), LOW);  // Wake on record button
  
  // Enter light sleep
  esp_light_sleep_start();
  
  // After waking up
  lastActivityTime = millis();
  displayOn = true;
  updateDisplay();
}

void formatTime(unsigned long seconds, char* buffer) {
  unsigned long minutes = seconds / 60;
  seconds %= 60;
  sprintf(buffer, "%02lu:%02lu", minutes, seconds);
}

// Helper function to print directory contents recursively
void printDirectory(File dir, int numTabs) {
  while (true) {
    File entry = dir.openNextFile();
    if (!entry) {
      break;
    }
    
    for (uint8_t i=0; i<numTabs; i++) {
      Serial.print('\t');
    }
    
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println("/");
      printDirectory(entry, numTabs+1);
    } else {
      // Files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    entry.close();
  }
} 