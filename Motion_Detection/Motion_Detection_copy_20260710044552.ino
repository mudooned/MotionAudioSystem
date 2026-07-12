#include <SPI.h>
#include <SD.h>

// ---------- Pin definitions ----------
const int speakerPin  = 25; // DAC output pin, feeds the amp
const int buttonPin   = 26; // Mute button
const int irSensorPin = 15; // IR motion sensor
#define SD_CS 5              // SD card chip-select pin

// ---------- Playback settings ----------
#define BUFFER_SIZE 8192     // Ring buffer size in bytes — bigger = more underrun protection
const int GAIN = 3;          // Software gain multiplier, tweak 1-5, back off if it distorts
#define NUM_SONGS 5           // How many wav files are on the card: 1.wav, 2.wav ... 5.wav

// ---------- Button / mute state ----------
bool mute = false;
bool lastButtonState = HIGH;

// ---------- Motion state ----------
bool motionDetected = false;
bool lastMotionState = false; // needed to detect the moment motion STARTS, not every loop it's present

// ---------- WAV file / playback state ----------
File wavFile;
uint16_t sampleRate = 8000;
uint16_t bitsPerSample = 8;
uint16_t numChannels = 1;
volatile bool isPlaying = false;

// ---------- Playback bookkeeping ----------
int lastSongPlayed = -1;       // avoids picking the same track twice in a row

// ---------- Ring buffer, shared between main loop (writer) and ISR (reader) ----------
volatile uint8_t ringBuffer[BUFFER_SIZE];
volatile uint32_t head = 0; // next write position
volatile uint32_t tail = 0; // next read position

hw_timer_t *timer = NULL;

// =================================================================
// ISR — fires once per sample, at exactly the current file's sample
// rate. Pulls one sample out of the ring buffer and outputs it.
// This is what keeps timing precise regardless of what loop() is doing.
// =================================================================
void IRAM_ATTR onTimer() {
  if (!isPlaying) return; // nothing queued up right now

  int bytesPerSample = (bitsPerSample == 16) ? 2 : 1;
  uint32_t available = (head - tail + BUFFER_SIZE) % BUFFER_SIZE;

  if (available < (uint32_t)bytesPerSample) {
    return; // buffer underrun — skip this tick rather than reading garbage
  }

  uint8_t outputSample;
  int centered;

  if (bitsPerSample == 8) {
    outputSample = ringBuffer[tail];
    tail = (tail + 1) % BUFFER_SIZE;
    centered = (int)outputSample - 128;
  } else {
    uint8_t lo = ringBuffer[tail];
    tail = (tail + 1) % BUFFER_SIZE;
    uint8_t hi = ringBuffer[tail];
    tail = (tail + 1) % BUFFER_SIZE;
    int16_t sample16 = (int16_t)((hi << 8) | lo);
    centered = sample16 / 256;
  }

  if (mute) {
    // Muted: still consume the buffer (so timing/position stays in sync)
    // but output silence instead of the real sample. This way unmuting
    // resumes cleanly instead of replaying stale audio.
    dacWrite(speakerPin, 128);
    return;
  }

  centered = centered * GAIN;
  centered = constrain(centered, -128, 127);
  outputSample = (uint8_t)(centered + 128);

  dacWrite(speakerPin, outputSample);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(irSensorPin, INPUT);

  // Seed the random generator using noise from an unused ADC pin,
  // so song selection isn't the same sequence every time the board resets.
  randomSeed(analogRead(34)); // any unconnected ADC-capable pin works

  if (!SD.begin(SD_CS, SPI, 4000000)) {
    Serial.println("SD initialization failed!");
    while (1);
  }
  Serial.println("SD OK");

  // Timer is created once; each new song just updates its alarm interval
  // to match that file's actual sample rate.
  timer = timerBegin(1000000); // 1 MHz base -> 1 tick = 1 microsecond
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, 1000000 / sampleRate, true, 0); // placeholder, updated per song

  Serial.println("Ready. Waiting for motion...");
}

void loop() {
  handleButton();
  checkMotion();
  fillBuffer(); // keep topping up the ring buffer whenever a song is playing

  // Detect playback finishing naturally (file exhausted AND buffer drained).
  // Buffer pointers are read atomically since the ISR can change tail at any moment.
  if (isPlaying && !wavFile.available()) {
    noInterrupts();
    uint32_t remaining = (head - tail + BUFFER_SIZE) % BUFFER_SIZE;
    interrupts();

    if (remaining == 0) {
      isPlaying = false;
      wavFile.close();
      Serial.println("Playback finished.");
    }
  }
}

// -----------------------------------------------------------------
// Checks the mute button and toggles mute on a press (rising edge).
// This ONLY silences output — it does not stop or restart playback.
// -----------------------------------------------------------------
void handleButton() {
  bool buttonState = digitalRead(buttonPin);

  if (buttonState == LOW && lastButtonState == HIGH) {
    mute = !mute;
    Serial.println(mute ? "Muted Audio" : "Audio Enabled");
    delay(200); // simple debounce
  }

  lastButtonState = buttonState;
}

// -----------------------------------------------------------------
// Reads the IR sensor and starts a random song on each fresh motion
// rising edge, as long as nothing is currently playing. If a song is
// already playing, a new trigger is ignored until it finishes.
// -----------------------------------------------------------------
void checkMotion() {
  int sensorState = digitalRead(irSensorPin);
  motionDetected = (sensorState == LOW); // flip to HIGH if your module is active-HIGH

  if (motionDetected && !lastMotionState) {
    Serial.println("Motion Detected");
    if (!isPlaying) {
      playRandomSong();
    }
  }

  lastMotionState = motionDetected;
}

// -----------------------------------------------------------------
// Picks a random track (avoiding an immediate repeat of the last one),
// opens it from the SD card, and parses its WAV header. If a file is
// missing or its header is bad, it tries the other tracks before
// giving up, so one corrupt file doesn't kill the whole playlist.
// -----------------------------------------------------------------
void playRandomSong() {
  for (int attempt = 0; attempt < NUM_SONGS; attempt++) {
    int songNumber = pickNextSongNumber();
    String filename = "/" + String(songNumber) + ".wav";

    wavFile = SD.open(filename.c_str());
    if (!wavFile) {
      Serial.print("Could not open ");
      Serial.println(filename);
      continue; // try another track
    }

    if (!parseWavHeader()) {
      Serial.println("Failed to parse WAV header, skipping.");
      wavFile.close();
      continue; // try another track
    }

    // Reset ring buffer pointers for the new file
    noInterrupts();
    head = 0;
    tail = 0;
    interrupts();

    fillBuffer(); // pre-fill before playback starts, avoids an instant underrun

    // Update the timer to this file's real sample rate — different tracks
    // could have different rates, so this can't be hardcoded once.
    timerAlarm(timer, 1000000 / sampleRate, true, 0);

    lastSongPlayed = songNumber;
    isPlaying = true;
    Serial.print("Playing ");
    Serial.println(filename);
    return;
  }

  // Every track failed to open/parse this round. isPlaying stays false,
  // so the next motion trigger will simply try again.
  Serial.println("No playable songs found this round.");
}

// -----------------------------------------------------------------
// Returns a random song number 1..NUM_SONGS, avoiding an immediate
// repeat of lastSongPlayed when there's more than one track to choose from.
// -----------------------------------------------------------------
int pickNextSongNumber() {
  if (NUM_SONGS <= 1) return 1;

  int songNumber;
  do {
    songNumber = random(1, NUM_SONGS + 1); // random(min, maxExclusive)
  } while (songNumber == lastSongPlayed);

  return songNumber;
}

// -----------------------------------------------------------------
// Reads the RIFF/WAVE header from the currently open wavFile and
// fills in sampleRate, bitsPerSample, numChannels globally.
// Returns true if a data chunk was found, false otherwise.
// -----------------------------------------------------------------
bool parseWavHeader() {
  char chunkId[5] = {0};
  wavFile.read((uint8_t*)chunkId, 4);              // "RIFF"
  wavFile.seek(wavFile.position() + 4);            // skip chunk size
  wavFile.read((uint8_t*)chunkId, 4);              // "WAVE"

  bool foundFmt = false;
  bool foundData = false;

  while (wavFile.available() && !(foundFmt && foundData)) {
    char subChunkId[5] = {0};
    wavFile.read((uint8_t*)subChunkId, 4);

    uint32_t subChunkSize;
    wavFile.read((uint8_t*)&subChunkSize, 4);

    if (strcmp(subChunkId, "fmt ") == 0) {
      uint16_t audioFormat;
      wavFile.read((uint8_t*)&audioFormat, 2);
      wavFile.read((uint8_t*)&numChannels, 2);
      wavFile.read((uint8_t*)&sampleRate, 4);
      wavFile.seek(wavFile.position() + 6);        // skip byteRate + blockAlign
      wavFile.read((uint8_t*)&bitsPerSample, 2);

      long extra = subChunkSize - 16;
      if (extra > 0) wavFile.seek(wavFile.position() + extra);

      foundFmt = true;
    }
    else if (strcmp(subChunkId, "data") == 0) {
      foundData = true; // file position is now at the start of sample data
    }
    else {
      wavFile.seek(wavFile.position() + subChunkSize); // skip unknown/extra chunk
    }
  }

  Serial.print("Sample rate: "); Serial.println(sampleRate);
  Serial.print("Bits per sample: "); Serial.println(bitsPerSample);
  Serial.print("Channels: "); Serial.println(numChannels);

  return foundData;
}

// -----------------------------------------------------------------
// Reads ahead from the SD card into the ring buffer in large chunks
// (not byte-by-byte) whenever there's room, so the ISR never runs
// dry mid-song. Assumes MONO files — see note below for stereo.
// -----------------------------------------------------------------
void fillBuffer() {
  if (!isPlaying || !wavFile) return;

  const int CHUNK = 512;
  uint8_t tempBuf[CHUNK];

  while (wavFile.available()) {
    noInterrupts();
    uint32_t used = (head - tail + BUFFER_SIZE) % BUFFER_SIZE;
    interrupts();
    uint32_t freeSpace = BUFFER_SIZE - 1 - used;

    if (freeSpace < CHUNK) break; // wait for the ISR to consume more first

    int toRead = min((long)CHUNK, (long)wavFile.available());
    int bytesRead = wavFile.read(tempBuf, toRead);
    if (bytesRead <= 0) break;

    for (int i = 0; i < bytesRead; i++) {
      ringBuffer[head] = tempBuf[i];
      head = (head + 1) % BUFFER_SIZE;
    }
  }
}
