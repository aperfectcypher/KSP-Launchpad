#include "KerbalSimpit.h"
#include <SPI.h>
#include <U8g2lib.h>
#include <CapacitiveSensor.h>

// ============Defines============

// Types
typedef uint32_t dword;
typedef int32_t sdword;
typedef uint16_t word;
typedef int16_t sword;
typedef uint8_t byte;
typedef int8_t sbyte;

// LCD display modes
#define NB_LCD_DISPLAY_MODES          2
#define LCD_DISPLAY_MODE_IDLE         -1
#define LCD_DISPLAY_MODE_CONNECTED    -2
#define LCD_DISPLAY_MODE_APSIDES      0
#define LCD_DISPLAY_MODE_ALT          1


// -----------Pins----------------
// LEDs
#define RCS_LED_PIN       30
#define SAS_LED_PIN       31

// Throttle
#define THROTTLE_INPUT_PIN         PIN_A7
#define THROTTLE_MOT_FWD_PIN       11
#define THROTTLE_MOT_REV_PIN       12
#define THROTTLE_TOUCH_SEND_PIN    51
#define THROTTLE_TOUCH_RECV_PIN    52
/* PLACEHOLDER TODO CORRECT PIN & NAME */
#define pb1                        24
#define pb2                        25

// Joystick 1
#define JOY1_X_PIN         PIN_A1
#define JOY1_Y_PIN         PIN_A2
#define JOY1_Z_PIN         PIN_A3
#define JOY1_PB_PIN        22

// Joystick 2
#define JOY2_X_PIN         PIN_A4
#define JOY2_Y_PIN         PIN_A5
#define JOY2_Z_PIN         PIN_A6
#define JOY2_PB_PIN        23

// Digital gauges
#define DGAUGES_LATCH_PIN  34
//      DGAUGES_DATA_PIN   21 /* DATA/CLK = ATmega2560 SPI */
//      DGAUGES_CLOCK_PIN  20

// Analog gauge
#define AGAUGE_EC_PIN      13

// LCD Display
#define LCD_DISPLAY_CS_PIN          36
#define LCD_DISPLAY_A0_PIN          37
//      LCD_DISPLAY_SDA_PIN         21 /* DATA/CLK = ATmega2560 SPI */
//      LCD_DISPLAY_SCL_PIN         20
#define LCD_DISPLAY_MODE_UP_PIN     2
#define LCD_DISPLAY_MODE_DOWN_PIN   3
// -------------------------------

// -----------Settings-----------
// Joystick
#define JOY_DEADZONE 1500

// Digital gauges
#define NB_DGAUGES 1
#define DGAUGES_OX 0

// Throttle
#define THROTTLE_MOT_TORQUE 255u // 0-255, pwm value
#define THROTTLE_TOUCH_THRESHOLD 35u
// ------------------------------

// Declare a KerbalSimpit object that will
// communicate using the "Serial" device.
KerbalSimpit simpitClient(Serial);

// LCD Display (Hardware SPI, CS=5, A0=7)
U8G2_ST7565_NHD_C12832_F_4W_HW_SPI u8g2(U8G2_R2, LCD_DISPLAY_CS_PIN, LCD_DISPLAY_A0_PIN);
char lcdLine1_ba[16] = {0};
char lcdLine2_ba[16] = {0};

// Create a SPI settings object for the digital gauges
SPISettings dgaugesSpi(10000000UL, MSBFIRST, SPI_MODE0);

// Create a CapacitiveSensor object for the throttle
CapacitiveSensor throttleTouchSensor(THROTTLE_TOUCH_SEND_PIN, THROTTLE_TOUCH_RECV_PIN);

// ---- Globals ----
// Ship fuel tanks in percent
byte shipFuelPercent_ba[NB_DGAUGES] = {0};
// Ship electric charge percent mapped on a byte (0-100%/0-255)
byte shipECPercent_b = 0;
// LCD display mode
sbyte lcdDisplayMode_sb = LCD_DISPLAY_MODE_IDLE;
// Ship apsides
String periapsis_s;
String apoapsis_s;
// Ship altitude
String alt_sea_s;
String alt_sur_s;
// Throttle motor target: -1 = no action, 0-1024 = target position
sword throttle_motorTarget_sw = -1;
word throttle_sliderMax_w = 1023;
word throttle_sliderMin_w = 0;
// -----------------

void setup()
{
  // Open the serial connection.
  Serial.begin(115200);

  // ---- IO initialization ----

  // Setup the build in LED, and turn it on.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Setup the RCS and SAS LEDs.
  pinMode(RCS_LED_PIN, OUTPUT);
  digitalWrite(RCS_LED_PIN, LOW);
  pinMode(SAS_LED_PIN, OUTPUT);
  digitalWrite(SAS_LED_PIN, LOW);

  // Setup digital gauges latch pin
  pinMode(DGAUGES_LATCH_PIN, OUTPUT);
  digitalWrite(DGAUGES_LATCH_PIN, HIGH);

  // Setup throttle motor pins
  pinMode(THROTTLE_MOT_FWD_PIN, OUTPUT);
  pinMode(THROTTLE_MOT_REV_PIN, OUTPUT);
  pinMode(THROTTLE_INPUT_PIN, INPUT);
  /* DEBUG TODO RM*/
  pinMode(pb1, INPUT);
  pinMode(pb2, INPUT);

  // Setup SPI for the digital gauges / LCD display
  SPI.begin();
  SPI.beginTransaction(dgaugesSpi);

  // Setup analog gauge
  pinMode(AGAUGE_EC_PIN, OUTPUT);
  // ----------------------------


  // ---- Subsystems initialization ----
  
  // Gauges init
  //dGaugesManagement();
  aGaugeManagement();

  // LCD Display init
  u8g2.begin();
  pinMode(LCD_DISPLAY_MODE_UP_PIN, INPUT_PULLUP);
  pinMode(LCD_DISPLAY_MODE_DOWN_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LCD_DISPLAY_MODE_UP_PIN), lcdDisplayModeUp, FALLING);
  attachInterrupt(digitalPinToInterrupt(LCD_DISPLAY_MODE_DOWN_PIN), lcdDisplayModeDown, FALLING);
  lcdDisplayManagement();
  // ----------------------------------

  // Throttle motor init
  throttleCalibration();

  // ---- KerbalSimpit initialization ----

  // This loop continually attempts to handshake with the plugin.
  // It will keep retrying until it gets a successful handshake.
  while (!simpitClient.init())
  {
    delay(100);
  }
  // Display a message on the LCD display to indicate handshaking is complete.
  lcdDisplayMode_sb = LCD_DISPLAY_MODE_APSIDES;
  lcdDisplayManagement();

  // Turn off the built-in LED to indicate handshaking is complete.
  digitalWrite(LED_BUILTIN, LOW);

  // Display a message in KSP to indicate handshaking is complete.
  simpitClient.printToKSP("Launchpad connected", PRINT_TO_SCREEN);

  // Sets our callback function. The KerbalSimpit library will
  // call this function every time a packet is received.
  simpitClient.inboundHandler(messageHandler);

  // Register SAS Mode channel.
  simpitClient.registerChannel(SAS_MODE_INFO_MESSAGE);

  // Register Oxidizer channel.
  simpitClient.registerChannel(OX_MESSAGE);

  // Register Apoapsis / Periapsis channel.
  simpitClient.registerChannel(APSIDES_MESSAGE);

  // Register Altitude channel.
  simpitClient.registerChannel(ALTITUDE_MESSAGE);

  // Register Electric charge channel.
  simpitClient.registerChannel(ELECTRIC_MESSAGE);

  // Register Scene change channel.
  //simpitClient.registerChannel(SCENE_CHANGE_MESSAGE);
  // -------------------------------------
}

dword loopCount_dw = 0;

void loop()
{
  // Check for new serial messages.
  simpitClient.update();
  
  // subsystems management
  //joystickManagement();
  throttleManagement();
  //dGaugesManagement();
  aGaugeManagement();
  if(loopCount_dw % 2500 == 0)
    lcdDisplayManagement();

  loopCount_dw++;
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
  word throttleRead_w = analogRead(THROTTLE_INPUT_PIN);
  // Scale the throttle value to the range of a signed 16-bit integer.
  throttleMsg.throttle = map(throttleRead_w, 0, 1023, 0, INT16_MAX);
  // Send the message
  simpitClient.send(THROTTLE_MESSAGE, throttleMsg);

  // Throttle motor management
  if (digitalRead(pb1) == HIGH)
  {
    throttle_motorTarget_sw = throttle_sliderMax_w;
  }
  else if (digitalRead(pb2) == HIGH)
  {
    throttle_motorTarget_sw = throttle_sliderMin_w;
  }

  // Capacitive touch safety
  long cap = throttleTouchSensor.capacitiveSensor(30);
  if (cap >= THROTTLE_TOUCH_THRESHOLD)
  {
    // Turn off motor
    analogWrite(THROTTLE_MOT_FWD_PIN, LOW);
    analogWrite(THROTTLE_MOT_REV_PIN, LOW);
    throttle_motorTarget_sw = -1;
  }

  if(throttle_motorTarget_sw != -1)
  {
    if(throttleRead_w != throttle_motorTarget_sw)
    {
      if (throttleRead_w > throttle_motorTarget_sw)
      {
        analogWrite(THROTTLE_MOT_FWD_PIN, LOW);
        analogWrite(THROTTLE_MOT_REV_PIN, THROTTLE_MOT_TORQUE);
      }
      else
      {
        analogWrite(THROTTLE_MOT_FWD_PIN, THROTTLE_MOT_TORQUE);
        analogWrite(THROTTLE_MOT_REV_PIN, LOW);
      }
    }
    else
    {
      // Turn off motor
      analogWrite(THROTTLE_MOT_FWD_PIN, LOW);
      analogWrite(THROTTLE_MOT_REV_PIN, LOW);
      throttle_motorTarget_sw = -1;
    }
  }
}

void throttleCalibration()
{
  // Set the capacitive touch sensor to never re-calibrate.
  throttleTouchSensor.set_CS_AutocaL_Millis(0xFFFFFFFF);
  throttleTouchSensor.set_CS_Timeout_Millis(10);

  // Move the throttle to the maximum position.
  analogWrite(THROTTLE_MOT_FWD_PIN, THROTTLE_MOT_TORQUE);
  analogWrite(THROTTLE_MOT_REV_PIN, LOW);
  // Wait for the throttle to reach the maximum position.
  delay(350u);
  // Read the throttle position from the potentiometer.
  throttle_sliderMax_w = analogRead(THROTTLE_INPUT_PIN);
  // Move the throttle to the minimum position.
  analogWrite(THROTTLE_MOT_FWD_PIN, LOW);
  analogWrite(THROTTLE_MOT_REV_PIN, THROTTLE_MOT_TORQUE);
  // Wait for the throttle to reach the minimum position.
  delay(350u);
  // Read the throttle position from the potentiometer.
  throttle_sliderMin_w = analogRead(THROTTLE_INPUT_PIN);
  // Turn off the motor.
  analogWrite(THROTTLE_MOT_FWD_PIN, LOW);
  analogWrite(THROTTLE_MOT_REV_PIN, LOW);
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

void lcdDisplayManagement(void)
{
  u8g2.clearBuffer();

  switch (lcdDisplayMode_sb)
  {
  case LCD_DISPLAY_MODE_IDLE:
    sprintf(lcdLine1_ba, "KSP Launchpad");
    sprintf(lcdLine2_ba, "Disconnected");

    u8g2.setFont(u8g2_font_9x18_tf);
    u8g2.drawStr(0, 18, lcdLine1_ba);
    u8g2.setFont(u8g2_font_7x13_tf);
    u8g2.drawStr(20, 31, lcdLine2_ba);
    break;

  case LCD_DISPLAY_MODE_CONNECTED:
    sprintf(lcdLine1_ba, "KSP Launchpad");
    sprintf(lcdLine2_ba, "Connected");

    u8g2.setFont(u8g2_font_9x18_tf);
    u8g2.drawStr(0, 18, lcdLine1_ba);
    u8g2.setFont(u8g2_font_7x13_tf);
    u8g2.drawStr(20, 31, lcdLine2_ba);
    break;

  case LCD_DISPLAY_MODE_APSIDES:
    apoapsis_s.toCharArray(lcdLine1_ba, 16);
    periapsis_s.toCharArray(lcdLine2_ba, 16);

    u8g2.setFont(u8g2_font_8x13_mf);
    u8g2.drawStr(0, 13, lcdLine1_ba);
    u8g2.drawStr(0, 30, lcdLine2_ba);
    break;

  case LCD_DISPLAY_MODE_ALT:
    alt_sea_s.toCharArray(lcdLine1_ba, 16);
    alt_sur_s.toCharArray(lcdLine2_ba, 16);

    u8g2.setFont(u8g2_font_8x13_mf);
    u8g2.drawStr(0, 13, lcdLine1_ba);
    u8g2.drawStr(0, 30, lcdLine2_ba);
    break;
  
  default:
    break;
  }
  u8g2.sendBuffer();
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
        // Create a new resourceMessage struct
        resourceMessage oxInfo;
        // Convert the message we received to an resourceMessage struct.
        oxInfo = parseMessage<resourceMessage>(msg);
        shipFuelPercent_ba[DGAUGES_OX] = (byte)map(oxInfo.available, 0, oxInfo.total, 0, 100);
      }
    break;

    case ELECTRIC_MESSAGE:
      // Checking if the message is the size we expect is a very basic
      // way to confirm if the message was received properly.
      if (msgSize == sizeof(resourceMessage))
      {
        // Create a new resourceMessage struct
        resourceMessage electricInfo;
        // Convert the message we received to an resourceMessage struct.
        electricInfo = parseMessage<resourceMessage>(msg);
        shipECPercent_b = (byte)map(electricInfo.available, 0, electricInfo.total, 0, 255);
      }
      break;

    case APSIDES_MESSAGE:
      // Checking if the message is the size we expect is a very basic
      // way to confirm if the message was received properly.
      if (msgSize == sizeof(apsidesMessage))
      {
        // Convert the message we received to an apsidesMessage struct.
        apsidesMessage apsides_s;
        apsides_s = parseMessage<apsidesMessage>(msg);

        String apo_string = "A:";
        String peri_string = "P:";

        float apoapsis = apsides_s.apoapsis;
        float periapsis = apsides_s.periapsis;

        if (abs(apoapsis) < 5000)
        {
          apoapsis_s = apo_string + apoapsis + "m";
        }
        else if ((abs(apoapsis) >= 5000) && (apoapsis < pow(10, 6)))
        {
          apoapsis_s = apo_string + apoapsis / pow(10, 3) + "Km";
        }
        else
        {
          apoapsis_s = apo_string + apoapsis / pow(10, 6) + "Mm";
        }

        if (abs(periapsis) < 5000)
        {
          periapsis_s = peri_string + periapsis + "m";
        }
        else if ((abs(periapsis) >= 5000) && (periapsis < pow(10, 6)))
        {
          periapsis_s = peri_string + periapsis / pow(10, 3) + "Km";
        }
        else
        {
          periapsis_s = peri_string + periapsis / pow(10, 6) + "Mm";
        }
      }
    break;

    case ALTITUDE_MESSAGE:
      // Checking if the message is the size we expect is a very basic
      // way to confirm if the message was received properly.
      if (msgSize == sizeof(altitudeMessage))
      {
        // Convert the message we received to an altitudeMessage struct.
        altitudeMessage altitude_s;
        altitude_s = parseMessage<altitudeMessage>(msg);

        String altitude_sea_string = "Sea:";
        String altitude_surface_string = "Gnd:";
        float alt_sea = altitude_s.sealevel;
        float alt_sur = altitude_s.surface;

        if (abs(alt_sea) < 5000)
        {
          alt_sea_s = altitude_sea_string + alt_sea + "m";
        }
        else if ((abs(alt_sea) >= 5000) && (alt_sea < pow(10, 6)))
        {
          alt_sea_s = altitude_sea_string + alt_sea / pow(10, 3) + "Km";
        }
        else
        {
          alt_sea_s = altitude_sea_string + alt_sea / pow(10, 6) + "Mm";
        }

        if (abs(alt_sur) < 5000)
        {
          alt_sur_s = altitude_surface_string + alt_sur + "m";
        }
        else if ((abs(alt_sur) >= 5000) && (alt_sur < pow(10, 6)))
        {
          alt_sur_s = altitude_surface_string + alt_sur / pow(10, 3) + "Km";
        }
        else
        {
          alt_sur_s = altitude_surface_string + alt_sur / pow(10, 6) + "Mm";
        }
      }
    break;
    default:
    break;
  }
}

void lcdDisplayModeUp(void)
{
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  // debounce
  if (interrupt_time - last_interrupt_time > 350)
  {
    last_interrupt_time = interrupt_time;
    lcdDisplayMode_sb++;
    if (lcdDisplayMode_sb > NB_LCD_DISPLAY_MODES - 1)
    {
      lcdDisplayMode_sb = 0;
    }
  }
}

void lcdDisplayModeDown(void)
{
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  // debounce
  if(interrupt_time - last_interrupt_time > 350)
  {
    if (lcdDisplayMode_sb == 0)
    {
      lcdDisplayMode_sb = NB_LCD_DISPLAY_MODES - 1;
    }
    else
    {
      lcdDisplayMode_sb--;
    }
  }
}

void aGaugeManagement(void)
{
  analogWrite(AGAUGE_EC_PIN, shipECPercent_b);
}
