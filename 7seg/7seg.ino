// https://forum.arduino.cc/index.php?topic=518797.0
// ST_CP = SCK
// SH_CP = RCK
// SDI   = DIO
// Common anode
#define DIO 12
#define SCK 15
#define RCK 14
#define SPEED 500

boolean numbersDef[10][8] = 
{
  {1,1,1,1,1,1,0}, //zero
  {0,1,1,0,0,0,0}, //one
  {1,1,0,1,1,0,1}, //two
  {1,1,1,1,0,0,1}, //three
  {0,1,1,0,0,1,1}, //four
  {1,0,1,1,0,1,1}, //five
  {1,0,1,1,1,1,1}, //six
  {1,1,1,0,0,0,0}, //seven
  {1,1,1,1,1,1,1}, //eight
  {1,1,1,1,0,1,1}  //nine
};

boolean digitsTable[8][8] =
{
  {0,0,0,0,1,0,0,0}, // first digit
  {0,0,0,0,0,1,0,0}, // second
  {0,0,0,0,0,0,1,0}, // third
  {0,0,0,0,0,0,0,1},  // 8th 
  {1,0,0,0,0,0,0,0}, // forth
  {0,1,0,0,0,0,0,0}, // fifth
  {0,0,1,0,0,0,0,0},  // sixth  
  {0,0,0,1,0,0,0,0} // 7th
};

void setup() {
  pinMode(DIO, OUTPUT);
  pinMode(SCK, OUTPUT);
  pinMode(RCK, OUTPUT);
  digitalWrite(DIO, LOW);
  digitalWrite(SCK, LOW);
  digitalWrite(RCK, LOW);
}

boolean display_buffer[32];

void prepareDisplayBuffer(int number, int digit_order, boolean showDot)
{
  for(int index=7; index>=0; index--)
  {
    display_buffer[index] = digitsTable[digit_order-1][index];
  }
  for(int index=14; index>=8; index--)
  {
    display_buffer[index] = !numbersDef[number-1][index]; //because logic is sanity, right?
  }
  if(showDot == true)
    display_buffer[15] = 0;
  else
    display_buffer[15] = 1;
}

void writeDigit(int number, int order, bool showDot = false)
{
  prepareDisplayBuffer(number, order, showDot);
  digitalWrite(RCK, LOW);
  for(int i=15; i>=0; i--)
  {
      digitalWrite(SCK, LOW);
      digitalWrite(DIO, display_buffer[i]); //output LOW - enable segments, HIGH - disable segments
      digitalWrite(SCK, HIGH);
   }
  digitalWrite(RCK, HIGH);
}

void writeNumber(double number, char nb_decimals = 0)
{
  long integer = (long)(number*(pow(10.0,(double)nb_decimals)));
  
  if(integer > 99999999)
  {
    integer = 99999999;
  }

  char digit = 8;
  while(digit > 0)
  {
    long mod = integer % 10;
    integer /= 10;
    writeDigit(mod, digit, 8-nb_decimals == digit);
    digit--;
  }
}

double num = 123.456;

void loop() {
  writeNumber(num,2);
  num = num + 0.0021;
}
