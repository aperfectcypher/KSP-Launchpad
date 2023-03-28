#include "KerbalSimpit.h"

#define RCS_LED_PIN  30
#define SAS_LED_PIN  31
#define PB0_PIN      22
#define THROTTLE_PIN PIN_A0

#define JOY_X_PIN    PIN_A1
#define JOY_Y_PIN    PIN_A2
#define JOY_Z_PIN    PIN_A3
#define JOY_DEADZONE 1500

// Declare a KerbalSimpit object that will
// communicate using the "Serial" device.
KerbalSimpit simpitClient(Serial);

void setup()
{
  // Open the serial connection.
  Serial.begin(115200);

  // Set up the build in LED, and turn it on.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Set up the RCS and SAS LEDs.
  pinMode(RCS_LED_PIN, OUTPUT);
  pinMode(SAS_LED_PIN, OUTPUT);

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

  // Send a message to the plugin registering for the Altitude channel.
  // The plugin will now regularly send Altitude messages while the
  // flight scene is active in-game.
  simpitClient.registerChannel(SAS_MODE_INFO_MESSAGE);
}

void loop()
{
  // Check for new serial messages.
  simpitClient.update();
  
  // subsystems management
  joystickManagement();
  throttleManagement();
}

void joystickManagement()
{
  rotationMessage rotation_msg;

  // Read the joystick position from the potentiometers.
  // Rotation joystick
  int joy_x_read = analogRead(JOY_X_PIN);
  int joy_y_read = analogRead(JOY_Y_PIN);
  int joy_z_read = analogRead(JOY_Z_PIN);

  // Scale the joystick values to the range of a signed 16-bit integer and
  // apply a deadzone.
  int scaled_x = map(joy_y_read, 0, 1023, INT16_MIN, INT16_MAX);
  if (scaled_x < JOY_DEADZONE && scaled_x > -JOY_DEADZONE)
  {
    scaled_x = 0;
  }
  int scaled_y = map(joy_x_read, 0, 1023, INT16_MIN, INT16_MAX);
  if (scaled_y < JOY_DEADZONE && scaled_y > -JOY_DEADZONE)
  {
    scaled_y = 0;
  }
  int scaled_z = map(joy_z_read, 0, 1023, INT16_MIN, INT16_MAX);
  if (scaled_z < JOY_DEADZONE && scaled_z > -JOY_DEADZONE)
  {
    scaled_z = 0;
  }

  // Set the rotation message
  rotation_msg.mask = PITCH_ROT | YAW_ROT | ROLL_ROT;
  rotation_msg.setPitch(scaled_x);
  rotation_msg.setYaw(scaled_y);
  rotation_msg.setRoll(scaled_z);

  // Send the message
  simpitClient.send(ROTATION_MESSAGE, rotation_msg);
}

void throttleManagement()
{
  throttleMessage throttle_msg;

  // Read the throttle position from the potentiometer.
  int throttle_read = analogRead(THROTTLE_PIN);
  // Scale the throttle value to the range of a signed 16-bit integer.
  throttle_msg.throttle = map(throttle_read, 0, 1023, 0, INT16_MAX);
  // Send the message
  simpitClient.send(THROTTLE_MESSAGE, throttle_msg);
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
  }
}
