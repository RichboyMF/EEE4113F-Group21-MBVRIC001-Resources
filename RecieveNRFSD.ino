#include <SPI.h>
#include <RF24.h>

#define CE_PIN   4
#define CSN_PIN  5
#define SCK_PIN  18
#define MOSI_PIN 16
#define MISO_PIN 17
#define LED_PIN  2

RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "00001";

#define PKT_START    1
#define PKT_DATA     0
#define PKT_LINE_END 3
#define PKT_END      2
#define PKT_FILE     4   // file stream header

struct Packet {
  uint8_t  type;
  uint16_t lineNum;
  uint8_t  chunkIdx;
  char     data[28];
};

char     assembledLine[256] = "";
char     currentFile[16]    = "sensors";  // active stream: "sensors" or "surface"
uint16_t totalLines         = 0;
uint16_t receivedLines      = 0;
uint16_t missedLines        = 0;
uint16_t lastLineNum        = 0;
uint16_t lastChunkIdx       = 255;
uint16_t lastLineEndNum     = 0;

void sendACK() {
  radio.stopListening();
  uint8_t ack = 0xAA;
  radio.write(&ack, 1);
  radio.startListening();
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  delay(1000);

  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CSN_PIN);
  if (!radio.begin(&SPI)) {
    Serial.println("[Receiver] ERROR: NRF24 not found!");
    Serial.println("[LOG] HARDWARE_ERROR NRF24 init failed");
    while (true) delay(500);
  }

  radio.openReadingPipe(1, address);
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setCRCLength(RF24_CRC_16);
  radio.setRetries(5, 15);
  radio.startListening();

  Serial.println("[Receiver] Ready. Waiting for transfer...");
  Serial.println("[LOG] SYSTEM_READY Receiver initialized");
}

void loop() {
  if (!radio.available()) return;

  Packet pkt;
  radio.read(&pkt, sizeof(pkt));
  sendACK();

  if (pkt.type == PKT_START) {
    // Reset all transfer state so back-to-back transfers in a session
    // don't accumulate stale counters or partial line data.
    totalLines     = atoi(pkt.data);
    receivedLines  = 0;
    missedLines    = 0;
    lastLineNum    = 0;
    lastLineEndNum = 0;
    lastChunkIdx   = 255;
    strncpy(currentFile, "sensors", sizeof(currentFile) - 1);
    memset(assembledLine, 0, sizeof(assembledLine));

    Serial.println("\n[Receiver] === TRANSFER STARTED ===");
    Serial.print("[Receiver] Expecting "); Serial.print(totalLines);
    Serial.println(" lines.\n");
    Serial.print("[LOG] TRANSFER_START expecting=");
    Serial.println(totalLines);

    digitalWrite(LED_PIN, HIGH); delay(300); digitalWrite(LED_PIN, LOW);
  }

  else if (pkt.type == PKT_FILE) {
    // Switch active stream context
    strncpy(currentFile, pkt.data, sizeof(currentFile) - 1);
    currentFile[sizeof(currentFile) - 1] = '\0';
    memset(assembledLine, 0, sizeof(assembledLine));
    lastChunkIdx = 255;

    Serial.print("[LOG] FILE_START stream=");
    Serial.println(currentFile);
  }

  else if (pkt.type == PKT_DATA) {
    if (pkt.lineNum != lastLineNum || pkt.chunkIdx != lastChunkIdx) {
      if (pkt.lineNum != lastLineNum) {
        memset(assembledLine, 0, sizeof(assembledLine));
        lastChunkIdx = 255;
      }
      if (pkt.chunkIdx == (uint8_t)(lastChunkIdx + 1) || lastChunkIdx == 255) {
        strncat(assembledLine, pkt.data,
                sizeof(assembledLine) - strlen(assembledLine) - 1);
        lastChunkIdx = pkt.chunkIdx;
        lastLineNum  = pkt.lineNum;
      }
    } else {
      Serial.print("[LOG] RETRY_DETECTED line=");
      Serial.print(pkt.lineNum);
      Serial.print(" chunk=");
      Serial.println(pkt.chunkIdx);
    }
  }

  else if (pkt.type == PKT_LINE_END) {
    if (pkt.lineNum == lastLineEndNum) {
      Serial.print("[LOG] RETRY_DETECTED line=");
      Serial.print(pkt.lineNum);
      Serial.println(" chunk=LINE_END");
      return;
    }
    lastLineEndNum = pkt.lineNum;

    if (lastLineNum > 0 && (int32_t)pkt.lineNum - (int32_t)lastLineNum > 1) {
      uint16_t gap = pkt.lineNum - lastLineNum - 1;
      missedLines += gap;
      Serial.print("[LOG] MISSING_LINES from=");
      Serial.print(lastLineNum + 1);
      Serial.print(" to=");
      Serial.print(pkt.lineNum - 1);
      Serial.print(" count="); Serial.println(gap);
    }

    lastLineNum  = pkt.lineNum;
    lastChunkIdx = 255;
    receivedLines++;

    // Tag output with stream name so the website knows which file it belongs to
    Serial.print("[Receiver] #"); Serial.print(pkt.lineNum);
    Serial.print(" ["); Serial.print(currentFile); Serial.print("]: ");
    Serial.println(assembledLine);

    Serial.print("[DATA:"); Serial.print(currentFile); Serial.print("] ");
    Serial.println(assembledLine);

    memset(assembledLine, 0, sizeof(assembledLine));

    if (receivedLines % 100 == 0) {
      Serial.print("[LOG] PROGRESS received=");
      Serial.print(receivedLines);
      Serial.print(" of "); Serial.println(totalLines);
      digitalWrite(LED_PIN, HIGH); delay(100); digitalWrite(LED_PIN, LOW);
    }
  }

  else if (pkt.type == PKT_END) {
    bool complete = (receivedLines == totalLines);

    Serial.println("\n[Receiver] === TRANSFER COMPLETE ===");
    Serial.print("[Receiver] Received: "); Serial.print(receivedLines);
    Serial.print(" / "); Serial.println(totalLines);

    if (complete) {
      Serial.print("[LOG] TRANSFER_END status=SUCCESS received=");
      Serial.print(receivedLines);
      Serial.print(" total="); Serial.println(totalLines);
    } else {
      Serial.print("[LOG] TRANSFER_END status=INCOMPLETE received=");
      Serial.print(receivedLines);
      Serial.print(" total="); Serial.print(totalLines);
      Serial.print(" missed="); Serial.println(missedLines);
    }

    digitalWrite(LED_PIN, HIGH); delay(1000); digitalWrite(LED_PIN, LOW);

    // Signal readiness for the next transfer.
    // The wizard/emulator watches for SYSTEM_READY to confirm the
    // receiver has reset and is ready to accept another PKT_START.
    // All internal state was reset on the last PKT_START so no
    // explicit counter reset is needed here.
    Serial.println("[Receiver] Ready. Waiting for transfer...");
    Serial.println("[LOG] SYSTEM_READY Receiver ready for next transfer");
  }
}
