#include <Arduino.h>

constexpr uint8_t DATA_PIN = 2;
constexpr uint8_t CLK_AZ_PIN = 3;
constexpr uint8_t CLK_ALT_PIN = 4;

constexpr uint16_t BIT_TIME_US = 96;
constexpr uint32_t ACK_TIMEOUT_US = BIT_TIME_US * 8UL;

uint8_t clockPinForAxis(uint8_t axis) {
  return axis == 1 ? CLK_AZ_PIN : CLK_ALT_PIN;
}

void dataRelease() {
  pinMode(DATA_PIN, INPUT_PULLUP);
}

void dataLow() {
  digitalWrite(DATA_PIN, LOW);
  pinMode(DATA_PIN, OUTPUT);
}

void clockIdle(uint8_t pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
}

bool waitForData(uint8_t expected) {
  const uint32_t start = micros();

  while (digitalRead(DATA_PIN) != expected) {
    if ((uint32_t)(micros() - start) >= ACK_TIMEOUT_US) {
      return false;
    }
  }

  return true;
}

bool startTransaction(uint8_t clockPin) {
  dataRelease();

  digitalWrite(clockPin, LOW);
  delayMicroseconds(BIT_TIME_US / 2);

  if (!waitForData(LOW)) {
    digitalWrite(clockPin, HIGH);
    dataRelease();
    return false;
  }

  digitalWrite(clockPin, HIGH);
  delayMicroseconds(BIT_TIME_US / 2);

  if (!waitForData(HIGH)) {
    dataRelease();
    return false;
  }

  delayMicroseconds(BIT_TIME_US);
  return true;
}

void sendBit(uint8_t value, uint8_t clockPin) {
  if (value) {
    dataRelease();
  } else {
    dataLow();
  }

  delayMicroseconds(BIT_TIME_US / 2);

  digitalWrite(clockPin, LOW);
  delayMicroseconds(BIT_TIME_US);

  digitalWrite(clockPin, HIGH);
  delayMicroseconds(BIT_TIME_US / 2);
}

void sendByte(uint8_t value, uint8_t clockPin) {
  for (uint8_t mask = 0x80; mask != 0; mask >>= 1) {
    sendBit((value & mask) != 0, clockPin);
  }

  dataRelease();
  delayMicroseconds(BIT_TIME_US);
}

uint8_t receiveBit(uint8_t clockPin) {
  dataRelease();

  digitalWrite(clockPin, LOW);
  delayMicroseconds(BIT_TIME_US / 2);

  const uint8_t value = digitalRead(DATA_PIN) ? 1 : 0;

  delayMicroseconds(BIT_TIME_US / 2);
  digitalWrite(clockPin, HIGH);
  delayMicroseconds(BIT_TIME_US);

  return value;
}

uint8_t receiveByte(uint8_t clockPin) {
  uint8_t value = 0;

  for (uint8_t i = 0; i < 8; ++i) {
    value = static_cast<uint8_t>(
      (value << 1) | receiveBit(clockPin)
    );
  }

  return value;
}

void printHexByte(uint8_t value) {
  if (value < 0x10) {
    Serial.print('0');
  }

  Serial.print(value, HEX);
}

bool parseHexByte(const char *text, uint8_t &value) {
  char *end = nullptr;
  const unsigned long result = strtoul(text, &end, 16);

  if (text == end || *end != '\0' || result > 0xFF) {
    return false;
  }

  value = static_cast<uint8_t>(result);
  return true;
}

void readFixedResponse(uint8_t command, uint8_t clockPin) {
  // 0x08: three bytes plus one bit.
  if (command == 0x08) {
    Serial.print(F("RX: "));

    for (uint8_t i = 0; i < 3; ++i) {
      printHexByte(receiveByte(clockPin));
      Serial.print(' ');
    }

    Serial.print(F("BIT="));
    Serial.println(receiveBit(clockPin));
  }
  // 0x09 and 0x0B: one response byte.
  else if (command == 0x09 || command == 0x0B) {
    Serial.print(F("RX: "));
    printHexByte(receiveByte(clockPin));
    Serial.println();
  }
  // Other commands have no response bytes.
  else {
  }
}

void executeCommand(
    uint8_t axis,
    uint8_t command,
    const uint8_t *parameters,
    uint8_t parameterCount,
    bool hasFinalBit,
    uint8_t finalBit
) {
  if (axis != 1 && axis != 2) {
    Serial.println(F("ERROR: axis must be 1 or 2"));
    return;
  }

  const uint8_t clockPin = clockPinForAxis(axis);
  clockIdle(clockPin);

  if (!startTransaction(clockPin)) {
    Serial.println(F("ERROR: no controller acknowledgement"));
    return;
  }

  sendByte(command, clockPin);

  for (uint8_t i = 0; i < parameterCount; ++i) {
    sendByte(parameters[i], clockPin);
  }

  if (hasFinalBit) {
    sendBit(finalBit, clockPin);
    dataRelease();
    delayMicroseconds(BIT_TIME_US);
  }

  readFixedResponse(command, clockPin);

  dataRelease();
  digitalWrite(clockPin, HIGH);
}

void processLine(char *line) {
  char *tokens[12];
  uint8_t tokenCount = 0;

  char *token = strtok(line, " \t");

  while (token != nullptr && tokenCount < 12) {
    tokens[tokenCount++] = token;
    token = strtok(nullptr, " \t");
  }

  if (tokenCount < 2) {
    Serial.println(F("Usage: axis command [byte1] [byte2] [byte3] [bit]"));
    return;
  }

  uint8_t axis;
  uint8_t command;

  if (!parseHexByte(tokens[0], axis) ||
      !parseHexByte(tokens[1], command)) {
    Serial.println(F("ERROR: invalid axis or command"));
    return;
  }

  uint8_t parameters[3];
  uint8_t parameterCount = 0;
  bool hasFinalBit = false;
  uint8_t finalBit = 0;

  uint8_t lastToken = tokenCount - 1;

  // A single final 0 or 1 is treated as one transmitted bit.
  if (tokenCount > 2 &&
      strlen(tokens[lastToken]) == 1 &&
      (tokens[lastToken][0] == '0' ||
       tokens[lastToken][0] == '1')) {
    hasFinalBit = true;
    finalBit = static_cast<uint8_t>(tokens[lastToken][0] - '0');
    tokenCount--;
  }

  for (uint8_t i = 2; i < tokenCount; ++i) {
    if (parameterCount >= 3) {
      Serial.println(F("ERROR: maximum three byte parameters"));
      return;
    }

    if (!parseHexByte(tokens[i], parameters[parameterCount])) {
      Serial.println(F("ERROR: invalid parameter"));
      return;
    }

    parameterCount++;
  }

  executeCommand(
      axis,
      command,
      parameters,
      parameterCount,
      hasFinalBit,
      finalBit
  );
}

void printHelp() {
  Serial.println(F("ETX/HBX Nano bridge"));
  Serial.println(F("Format: axis command [byte1] [byte2] [byte3] [bit]"));
  Serial.println(F("Examples:"));
  Serial.println(F("  1 0B       Get Azimuth motor type"));
  Serial.println(F("  2 0B       Get Altitude motor type"));
  Serial.println(F("  1 08       Get Azimuth status"));
  Serial.println(F("  2 08       Get Altitude status"));
  Serial.println(F("  1 05       Stop Azimuth"));
  Serial.println(F("  2 05       Stop Altitude"));
  Serial.println(F("  1 02 00 00 0"));
  Serial.println(F("  1 02 00 00 1"));
}

void setup() {
  Serial.begin(115200);

  dataRelease();
  clockIdle(CLK_AZ_PIN);
  clockIdle(CLK_ALT_PIN);

  Serial.println(F("Ready. Type help for commands."));
}

void loop() {
  if (!Serial.available()) {
    return;
  }

  char line[80];
  size_t length = Serial.readBytesUntil('\n', line, sizeof(line) - 1);
  line[length] = '\0';

  while (length > 0 &&
         (line[length - 1] == '\r' ||
          line[length - 1] == ' ' ||
          line[length - 1] == '\t')) {
    line[--length] = '\0';
  }

  if (strcmp(line, "help") == 0) {
    printHelp();
  } else if (length > 0) {
    processLine(line);
  }
}