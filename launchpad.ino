#include "KerbalSimpit.h"
#include <SPI.h>

// -----------Pins----------------
#define RCS_LED_PIN       30
#define SAS_LED_PIN       31
#define THROTTLE_PIN      PIN_A7

// Joystick 1
#define JOY1_X_PIN         PIN_A1
#define JOY1_Y_PIN         PIN_A2
#define JOY1_Z_PIN         PIN_A3
#define JOY1_PB_PIN        5

// Joystick 2
#define JOY2_X_PIN         PIN_A4
#define JOY2_Y_PIN         PIN_A5
#define JOY2_Z_PIN         PIN_A6
#define JOY2_PB_PIN        6
// -------------------------------

#define DGAUGES_LATCH_PIN 8

// -----------Settings-----------
// Joystick
#define JOY_DEADZONE 1500

// Digital gauges
#define NB_DGAUGES 1
#define DGAUGES_OX 0
// ------------------------------

// Declare a KerbalSimpit object that will
// communicate using the "Serial" device.
KerbalSimpit simpitClient(Serial);

// Create a SPI settings object for the digital gauges
SPISettings dgaugesSpi(10000000UL, MSBFIRST, SPI_MODE0);
// Ship fuel tanks in percent
byte shipFuelPercent_ba[NB_DGAUGES] = {0};

void setup()
{
  // Open the serial connection.
  Serial.begin(115200);

  // Set up the build in LED, and turn it on.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Set up the RCS and SAS LEDs.
  pinMode(RCS_LED_PIN, OUTPUT);
  digitalWrite(RCS_LED_PIN, LOW);
  pinMode(SAS_LED_PIN, OUTPUT);
  digitalWrite(SAS_LED_PIN, LOW);

  pinMode(DGAUGES_LATCH_PIN, OUTPUT);
  digitalWrite(DGAUGES_LATCH_PIN, HIGH);

  // Setup SPI for the digital gauges
  SPI.begin();
  SPI.beginTransaction(dgaugesSpi);

  // Subsystems initialization
  dGaugesManagement();

  // This loop continually attempts to handshake with the plugin.
  // It will keep retrying until it gets a successful handshake.
  while (!simpitClient.init())
  {
    delay(100);
  }

  // Turn off the built-in LED to indicate handshaking is complete.
  digitalWrite(LED_BUILTIN, LOW);

  // Display a message in KSP to indicate handshaking is complete.
  simpitClient.printToKSP("Connected", PRINT_TO_SCREEN);

  // Sets our callback function. The KerbalSimpit library will
  // call this function every time a packet is received.
  simpitClient.inboundHandler(messageHandler);

  // Register Altitude channel.

  simpitClient.registerChannel(SAS_MODE_INFO_MESSAGE);

  // Register Oxidizer channel.
  simpitClient.registerChannel(OX_MESSAGE);
}

void loop()
{
  // Check for new serial messages.
  simpitClient.update();
  
  // subsystems management
  //joystickManagement();
  //throttleManagement();
  dGaugesManagement();
}

void joystickManagement()
{
  rotationMessage rotationMsg;

  // Read the joystick position from the potentiometers.
  // Rotation joystick
  int joyXRead = analogRead(JOY1_X_PIN);
  int joyYRead = analogRead(JOY1_Y_PIN);
  int joyZRead = analogRead(JOY1_Z_PIN);

  // Scale the joystick values to the range of a signed 16-bit integer and
  // apply a deadzone.
  int scaledX = map(joyYRead, 0, 1023, INT16_MIN, INT16_MAX);
  if (scaledX < JOY_DEADZONE && scaledX > -JOY_DEADZONE)
  {
    scaledX = 0;
  }
  int scaledY = map(joyXRead, 0, 1023, INT16_MIN, INT16_MAX);
  if (scaledY < JOY_DEADZONE && scaledY > -JOY_DEADZONE)
  {
    scaledY = 0;
  }
  int scaledZ = map(joyZRead, 0, 1023, INT16_MIN, INT16_MAX);
  if (scaledZ < JOY_DEADZONE && scaledZ > -JOY_DEADZONE)
  {
    scaledZ = 0;
  }

  // Set the rotation message
  rotationMsg.mask = PITCH_ROT | YAW_ROT | ROLL_ROT;
  rotationMsg.setPitch(scaledX);
  rotationMsg.setYaw(scaledY);
  rotationMsg.setRoll(scaledZ);

  // Send the message
  simpitClient.send(ROTATION_MESSAGE, rotationMsg);
}

void throttleManagement()
{
  throttleMessage throttleMsg;

  // Read the throttle position from the potentiometer.
  int throttleRead = analogRead(THROTTLE_PIN);
  // Scale the throttle value to the range of a signed 16-bit integer.
  throttleMsg.throttle = map(throttleRead, 0, 1023, 0, INT16_MAX);
  // Send the message
  simpitClient.send(THROTTLE_MESSAGE, throttleMsg);
}

#define DGAUGE_NORMAL 0
#define DGAUGE_BLINK  1
#define DGAUGE_DIM    2
#define DGAUGE_OFF    3

// Digital gauges management
void dGaugesManagement()
{
  static word dGaugeMgmtCallCount = 0;
  byte dgaugeMode_ba[NB_DGAUGES] = {DGAUGE_NORMAL};
  byte dgaugeBrightness_ba[NB_DGAUGES] = {100};
  byte fuelNbLedOn_ba[NB_DGAUGES] = {0};
  word fuelGaugesBitmask_wa[NB_DGAUGES] = {0};

  for(int i = 0; i < NB_DGAUGES; i++)
  {
    if(shipFuelPercent_ba[i] < 1) // If fuel is empty, turn off the gauge
    {
      dgaugeMode_ba[i] = DGAUGE_OFF;
    }
    else if (shipFuelPercent_ba[i] < 10) // If total fuel is under 10%, blink the gauge
    {
      dgaugeMode_ba[i] = DGAUGE_BLINK;
    }
    else // Otherwise, dim the curent led progressively
    {
      dgaugeMode_ba[i] = DGAUGE_DIM;
      dgaugeBrightness_ba[i] = 10 - (shipFuelPercent_ba[i] % 10);
    }

    // Get the number of leds to light up
    fuelNbLedOn_ba[i] = ceil(((double)shipFuelPercent_ba[i] / 10.0));
    
    // Build the bitmask
    for(int j = 0; j < fuelNbLedOn_ba[i]; j++)
    {
      fuelGaugesBitmask_wa[i] |= (1 << j);
    }
  }

  // Un-latch the shift registers
  digitalWrite(DGAUGES_LATCH_PIN, LOW);
  // Send the message to the shift registers via SPI
  for(int i = 0; i < NB_DGAUGES; i++)
  {
    if(dgaugeMode_ba[i] == DGAUGE_DIM)
    {
      if ((dGaugeMgmtCallCount/1) % (dgaugeBrightness_ba[i]) && dgaugeBrightness_ba[i] != 10)
      {
        fuelGaugesBitmask_wa[i] = fuelGaugesBitmask_wa[i] >> 1;
      }
    }
    else if (dgaugeMode_ba[i] == DGAUGE_BLINK)
    {
      if ((dGaugeMgmtCallCount/1000) % 2)
      {
        fuelGaugesBitmask_wa[i] = 0;
      }
    }
    else if (dgaugeMode_ba[i] == DGAUGE_OFF)
    {
      fuelGaugesBitmask_wa[i] = 0;
    }

    SPI.transfer(highByte(fuelGaugesBitmask_wa[i]));
    SPI.transfer(lowByte(fuelGaugesBitmask_wa[i]));
  }
  // latch the shift registers
  digitalWrite(DGAUGES_LATCH_PIN, HIGH);

  dGaugeMgmtCallCount++;
}

void messageHandler(byte messageType, byte msg[], byte msgSize)
{
  switch (messageType)
  {
  case SAS_MODE_INFO_MESSAGE:
    // Checking if the message is the size we expect is a very basic
    // way to confirm if the message was received properly.
    if (msgSize == sizeof(SASInfoMessage))
    {
      // Create a new SASInfoMessage struct
      SASInfoMessage sasInfo;
      // Convert the message we received to an SASInfoMessage struct.
      sasInfo = parseMessage<SASInfoMessage>(msg);
      // Turn the LED on if SAS is enabled Otherwise turn it off.
      if (sasInfo.currentSASMode != 255)
      {
        digitalWrite(SAS_LED_PIN, HIGH);
      }
      else
      {
        digitalWrite(SAS_LED_PIN, LOW);
      }
    }
    break;
    case OX_MESSAGE:
    // Checking if the message is the size we expect is a very basic
    // way to confirm if the message was received properly.
    if (msgSize == sizeof(resourceMessage))
    {
      // Create a new SASInfoMessage struct
      resourceMessage oxInfo;
      // Convert the message we received to an resourceMessage struct.
      oxInfo = parseMessage<resourceMessage>(msg);
      shipFuelPercent_ba[DGAUGES_OX] = (byte)map(oxInfo.available, 0, oxInfo.total, 0, 100);
    }
  }
}
