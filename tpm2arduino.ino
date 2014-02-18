  
#include <pt.h>
#define FORCE_SOFTWARE_SPI
#define FORCE_SOFTWARE_PINS
#include <FastSPI_LED2.h>

/*==============================================================================*/
/* LED und Arduino Variablen */
/*==============================================================================*/

#define NUM_LEDS             214                // Number of LEDs
#define MAX_ARGS             10                 // Max Number of command arguments
#define BAUDRATE             500000             // Baudrate
#define SERIAL               Serial             // Serial port for communication
#define SERIAL_DEBUG         Serial             // Serial port for debugging
#define DATA_PIN             3                  // PIN where LEDs are connected/Used for TM1809/WS2811 chipsets, because they dont use SPI
//#define CLOCK_PIN           4                  // used for some SPI chipsets, e.g. WS2801 
#define BRIGHTNESS           255                // define global brightness 0..255

//choose your LED chipset in void setup()

/*==============================================================================*/
/* TPM2 Variablen */
/*==============================================================================*/

enum Protocol
{
   // positions
   posStart      = 0,
   posType       = 1,
   posFsHigh     = 2,
   posFsLow      = 3,
   posData       = 4,

   // bytes 
   tpm2Start     = 0xc9,
   tpm2DataFrame = 0xda,
   tpm2Command   = 0xc0,
   tpm2Answer    = 0xaa,
   tpm2End       = 0x36,
   tpm2Ack       = 0xAC
};

enum Mode
{
   mNone,
   mCommunication,
   mProgram
};

struct Data
{
   int pos;
   uint8_t type;
   uint16_t fs;
   uint8_t command;
   CRGB rgb[NUM_LEDS];
} data;

byte args[MAX_ARGS];
unsigned long lastDataAt = 0;
int program = 0;            
int mode = mNone;
int effectDelay = 100;
static struct pt pt1;

/*==============================================================================*/
/* Variablen für Effekte */
/*==============================================================================*/
 
int BOTTOM_INDEX = 0;
int TOP_INDEX = int(NUM_LEDS/2);
int EVENODD = NUM_LEDS%2;

/*==============================================================================*/
/* debug code
/*==============================================================================*/

// comment this line out to disable debugging 

//#define DEBUG
 
#ifdef DEBUG

void debug(const char* str)
{
   SERIAL_DEBUG.println(str);
}

void debug(const char* str, uint16_t val, int fmt = DEC)
{   
   SERIAL_DEBUG.print(str);
   SERIAL_DEBUG.println(val, fmt);
}

int freeRam() 
{
   extern int __heap_start, *__brkval; 
   int v; 
   return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

#else
#  define debug( ... ) 
#endif

void setup()
{
     
   memset(data.rgb, 0, sizeof(struct CRGB) * NUM_LEDS); 

   // Uncomment one of the following lines for your leds arrangement.
   // FastLED.addLeds<TM1803, DATA_PIN, RGB>(data.rgb, NUM_LEDS);
   // FastLED.addLeds<TM1804, DATA_PIN, RGB>(data.rgb, NUM_LEDS);
   // FastLED.addLeds<TM1809, DATA_PIN, RGB>(data.rgb, NUM_LEDS);
   // FastSPI_LED2.addLeds<WS2811, DATA_PIN, GRB>(data.rgb+18, NUM_LEDS/3);
   // FastLED.addLeds<WS2811, 8, RGB>(data.rgb + 225, NUM_LEDS/4);
   // FastLED.addLeds<WS2812, DATA_PIN, RGB>(data.rgb, NUM_LEDS);
   // FastLED.addLeds<WS2812B, DATA_PIN, RGB>(data.rgb, NUM_LEDS);
   // FastLED.addLeds<NEOPIXEL, DATA_PIN, RGB>(data.rgb, NUM_LEDS);
   // FastLED.addLeds<WS2811_400, DATA_PIN, RGB>(data.rgb, NUM_LEDS);
   // FastLED.addLeds<UCS1903, DATA_PIN, RGB>(data.rgb, NUM_LEDS);
   // FastLED.addLeds<WS2801, RGB>(data.rgb, NUM_LEDS);
   // FastLED.addLeds<SM16716, RGB>(data.rgb, NUM_LEDS);
   // FastLED.addLeds<LPD8806, RGB>(data.rgb, NUM_LEDS);
   // FastLED.addLeds<WS2801, DATA_PIN, CLOCK_PIN, RGB>(data.rgb, NUM_LEDS);
   // FastLED.addLeds<SM16716, DATA_PIN, CLOCK_PIN, RGB>(data.rgb, NUM_LEDS);
   // FastLED.addLeds<LPD8806, DATA_PIN, CLOCK_PIN, RGB>(data.rgb, NUM_LEDS);
   
   FastLED.addLeds<WS2811, DATA_PIN, GRB>(data.rgb, NUM_LEDS);
   FastLED.setBrightness(BRIGHTNESS);
   
   oneColorAll(10,10,10);
#ifdef DEBUG
   SERIAL_DEBUG.begin(BAUDRATE);
   // wait for serial port to connect. Needed for Leonardo only
   while (!Serial)
      delay(1);
   SERIAL_DEBUG.println("Setup done");
#endif
   SERIAL.begin(BAUDRATE);
   memset(args, 0, MAX_ARGS);
   PT_INIT(&pt1);
   resetVars();
}

/*==============================================================================*/
/* Thread für Programm/Effekte
/*==============================================================================*/

static int playProgramThread(struct pt *pt) 
{
   static unsigned long timestamp = 0;
   PT_BEGIN(pt);
   while(1) 
   {
      PT_WAIT_UNTIL(pt, millis() - timestamp > effectDelay);
      playProgram();
      timestamp = millis();
   }
   PT_END(pt);
}

/*==============================================================================*/
/* loop
/*==============================================================================*/

void loop()
{
  while (1)
  {
   // if data available switch to communication mode
   if (SERIAL.available() > 0)
   {
      if (mode != mCommunication)
      { 
         debug("switching to communication mode");
         mode = mCommunication;
         resetVars();
      }
      doCommunication();
   }
   else
   // communication timeout after 0.5 seconds
   while (SERIAL.available() == 0 && millis()-lastDataAt > 1000)
   {
      if (mode != mProgram)
      {
         debug("switching to prg mode, ignoring ", data.pos);
         mode = mProgram;
         resetVars();

      }
      playProgramThread(&pt1);
   }
  }
}

/*==============================================================================*/
/* do communication
/*==============================================================================*/

void doCommunication()
{
#ifdef DEBUG
   int count = 0;
#endif

   // read ...

   while (SERIAL.available() > 0)
   {
     
      byte val = SERIAL.read();
      lastDataAt = millis();

#ifdef DEBUG
      debug("got: ", val, HEX);
      debug("at pos: ", data.pos, DEC);
      count++;
#endif

      if (data.pos == posStart && val == tpm2Start)                                    // Startbyte
         resetVars();

      else if (data.pos == posType && (val == tpm2DataFrame || val == tpm2Command))    // Block-Art
         data.type = val;

      else if (data.pos == posFsHigh)                                                  // Framesize (High Bit)
      {
         data.fs = (val << 8) & 0xFF00;
         if (data.fs > NUM_LEDS*3)
         {
           debug("framsize too high: ", data.fs);
           resetVars();
           continue;
         } 
      }
      else if (data.pos == posFsLow)                                                   // Framesize (Low byte)
      {
         data.fs += val & 0x00FF;
         if (data.fs > NUM_LEDS*3)
         {
           debug("framsize too high: ", data.fs);
           resetVars();
           continue;
         } 
      }
      else if (data.pos == posData + data.fs && val == tpm2End)                        // End Byte
         parsePacket();

      else if (data.pos >= posData && data.pos < posData+data.fs)                      // Bytes zwischen Header und Ende lesen
         evaluateData(val);

      else                                                                             // protocol violation ...
      {
         if (data.pos != 0)
         {
            debug("protocol violation at pos: ", data.pos);
            debug("val was: ", val);
         }
          debug("Error");
         resetVars();
         continue;
      }
      data.pos++;
   }

#ifdef DEBUG
   if (count)
      debug("received", count, DEC);
#endif
}

/*==============================================================================*/
/* evaluate data
/*==============================================================================*/

void evaluateData(byte val)
{
   if (data.type == tpm2DataFrame)        // frame data
   {
      uint8_t* rgb = (uint8_t*)data.rgb;
      rgb[data.pos-posData] = val;
   }

   else                                  // command data
   {
      if (data.pos == posData)
      {
         data.command = val;
         memset(args, 0, sizeof(args));  
      }
      else
         args[data.pos-posData-1] = val;
   }
}

/*==============================================================================*/
/* reset variables
/*==============================================================================*/

void resetVars()
{
   debug("Reset");
   memset(&data, 0, sizeof(Data));
   //data.rgb = (struct CRGB*)FastSPI_LED.getRGBData();
   memset(data.rgb, 0,  NUM_LEDS * sizeof(struct CRGB));
}

/*==============================================================================*/
/* parse packet
/*==============================================================================*/

void parsePacket()
{
   debug("Parse");

   switch (data.type)
   {
      case tpm2DataFrame:
      {
         showLeds();
         break;
      }
      case tpm2Command: 
      {
         setProgram();
         break;
      }
      default: 
         break;
   }

   SERIAL.write(tpm2Ack);
   resetVars();
   data.pos = -1;
}

/*==============================================================================*/
/* set LED color
/*==============================================================================*/

void setLedColor(int led, uint8_t r, uint8_t g, uint8_t b)
{
   data.rgb[led].r = r; 
   data.rgb[led].g = g; 
   data.rgb[led].b = b; 
}

/*==============================================================================*/
/* one Color All
/*==============================================================================*/

void oneColorAll(uint8_t r, uint8_t g, uint8_t b)
{
   memset(data.rgb, 0, NUM_LEDS * sizeof(struct CRGB));

   for (int led = 0; led < NUM_LEDS; led++)
      setLedColor(led, r, g, b);

   showLeds();
   effectDelay = 1000;
}

/*==============================================================================*/
/* Output Leds
/*==============================================================================*/

void showLeds()
{
    FastLED.show();
}


void setProgram()
{
   program = data.command;
}

void playProgram()
{
   switch (program)
   {
      case  0: oneColorAll(args[0],args[1],args[2]);   break;
      case  1: loopRGBPixel(50);                       break;
      case  2: rainbow_fade(20);                       break;
      case  3: rainbow_loop(20);                       break;
      case  4: random_burst(20);                       break;
      case  5: flicker(200,255);                       break;
      case  6: colorBounce(200,50);                    break;
      case  7: pulse_oneColorAll(200,50,100,0);        break;
      case  8: pulse_oneColorAll(0,50,100,1);          break;
      case  9: police_light_strobo(50);                break;
      case 10: police_lightsALL(20);                   break;
      case 11: police_lightsONE(20);                   break;
      default: oneColorAll(0,0,0);        break;
   }
}

/* Set LED Color of given LED */

void oneColorAllNOSHOW(int r, int g, int b)
{
   resetVars();
   for (int led = 0; led < NUM_LEDS; led++)
   {
      setLedColor(led, r, g, b);
   }

}

/*==============================================================================*/
/* Effect 0: Fixed color - Arguments RR GG BB
/*==============================================================================*/


/*==============================================================================*/
/* Effect 1: Loops each RGB color through each Pixel
/*==============================================================================*/

void loopRGBPixel(int idelay) //OK
{
   static int c = 0;
   static int i = 0;

   if (i > NUM_LEDS-1)
   {
      i = 0;
      c++;
   }
   if (c == 3)
      c = 0;

   memset(data.rgb, 0, NUM_LEDS*3);

   switch (c) 
   { 
      case 0:   
         data.rgb[i].r =200; 
         break; 
      case 1: 
         data.rgb[i].g =200; 
         break;
      case 2: 
         data.rgb[i].b =200;
         break;
   }
   showLeds();
   effectDelay = idelay;
   i++;
}

/*==============================================================================*/
/* Effect 2: Fade through raibow colors over all LEDs
/*==============================================================================*/

void rainbow_fade(int idelay) { //OK //-FADE ALL LEDS THROUGH HSV RAINBOW
   static int ihue = -1;
   ihue++;
   if (ihue >= 255) {
      ihue = 0;
   }
   for(int idex = 0 ; idex < NUM_LEDS; idex++ ) {
      data.rgb[idex] = CHSV(ihue, 255, 255);
   }
   showLeds();
   effectDelay = idelay;
}

/*==============================================================================*/
/* Effect 3: Loops rainbow colors around the stripe
/*==============================================================================*/

void rainbow_loop(int idelay) { //-LOOP HSV RAINBOW
   static double idex = 0;
   static double ihue = 0;
   double steps = (double)255/NUM_LEDS; 
   
   for(int led = 0 ; led < NUM_LEDS; led++ ) {
      ihue = led * steps + idex;
      if (ihue >= 255) 
         ihue -= 255;

      data.rgb[led] = CHSV((int)ihue, 255, 255);

      if (led == 0)
         idex += steps;
      if (idex >= 255) 
         idex = 0;
   }  
   showLeds();
   effectDelay = idelay;
}

/*==============================================================================*/
/* Effect 4: Random burst - Randowm colors on each LED
/*==============================================================================*/

void random_burst(int idelay) { //-RANDOM INDEX/COLOR
   static int idex = 0;
   static int ihue = 0;

   idex = random(0,NUM_LEDS-1);
   ihue = random(0,255);

   data.rgb[idex] = CHSV(ihue, 255, 255);

   showLeds();
   effectDelay = idelay;
}

/*==============================================================================*/
/* Effect 5: Flicker effect - random flashing of all LEDs
/*==============================================================================*/

void flicker(int thishue, int thissat) {
   int ibright = random(0,255);
   int random_delay = random(10,100);

   for(int i = 0 ; i < NUM_LEDS; i++ ) {
      data.rgb[i] = CHSV(thishue, thissat, ibright); 
   }

   showLeds();
   effectDelay = random_delay;
}

/*==============================================================================*/
/* Effect 6: Color bounce - bounce a color through whole stripe
/*==============================================================================*/

void colorBounce(int ihue, int idelay) { //-BOUNCE COLOR (SINGLE LED)
  static int bouncedirection = 0;
  static int idex = 0;
  
  if (bouncedirection == 0) {
    idex = idex + 1;
    if (idex == NUM_LEDS) {
      bouncedirection = 1;
      idex = idex - 1;
    }
  }
  if (bouncedirection == 1) {
    idex = idex - 1;
    if (idex == 0) {
      bouncedirection = 0;
    }
  }  
  for(int i = 0; i < NUM_LEDS; i++ ) {
    if (i == idex) {
      data.rgb[i] = CHSV(ihue, 255, 255);
    }
    else {
      setLedColor(i, 0, 0, 0);
    }
  }
  showLeds();
  effectDelay = idelay;
}

/*==============================================================================*/
/* Effect 7/8: Fade in/out a color using brightness/saturation
/*==============================================================================*/

void pulse_oneColorAll(int ahue, int idelay, int steps, int useSat) { //-PULSE BRIGHTNESS ON ALL LEDS TO ONE COLOR 
  static int bouncedirection = 0;
  static int idex = 0;
  int isteps = 255/steps;
  static int ival = 0;
  
  static int xhue = 0;
  
  if (bouncedirection == 0) {
    ival += isteps;
    if (ival >= 255) {
      bouncedirection = 1;
    }
  }
  if (bouncedirection == 1) {
    ival -= isteps;
    if (ival <= 0) {
      bouncedirection = 0;
     xhue = random(0, 255);
    }         
  }  

  for(int i = 0 ; i < NUM_LEDS; i++ ) {
    if (useSat == 0)
      data.rgb[i] = CHSV(xhue, 255, ival);
    else
      data.rgb[i] = CHSV(xhue, ival, 255);
  }
  showLeds();
  effectDelay = idelay;
}


/*==============================================================================*/
/* Effect 9: Police light - red/blue strobo on each half of stripe
/*==============================================================================*/

void police_light_strobo(int idelay)
{
  int middle = NUM_LEDS/2;
  static int color = 0;  
  static int left_right = 0;
  
  if (left_right > 19)
    left_right = 0;
  
  if (color == 1)
    color = 0;
  else
    color = 1;
    
  for (int i = 0; i < NUM_LEDS; i++)
  {
    if (i <= middle && left_right < 10)
    {
      if (color)
        setLedColor(i, 0, 0, 255);
      else
        setLedColor(i, 0, 0, 0);
    }
    else
    if (i > middle && left_right >= 10)
    {
      if (color)
        setLedColor(i, 255, 0, 0);
     else
        setLedColor(i, 0, 0, 0);
    }
  }
  showLeds();
  effectDelay = idelay;
  
  left_right++;
  
}

/*==============================================================================*/
/* Effect 10: Police Light all LEDs 
/*==============================================================================*/

void police_lightsALL(int idelay) { //-POLICE LIGHTS (TWO COLOR SOLID)
  static int idex = 0;
  
  int idexR = idex;
  int idexB = antipodal_index(idexR);
  setLedColor(idexR, 255, 0, 0);
  setLedColor(idexB, 0, 0, 255);
  showLeds();
  effectDelay = idelay;
  idex++;
  if (idex >= NUM_LEDS) {
    idex = 0;
  }
}

/*==============================================================================*/
/* Effect 11: Police Light one LED blue and red
/*==============================================================================*/

void police_lightsONE(int idelay) { //-POLICE LIGHTS (TWO COLOR SINGLE LED)
  static int idex = 0;
   
  int idexR = idex;
  int idexB = antipodal_index(idexR);  
  for(int i = 0; i < NUM_LEDS; i++ ) {
    if (i == idexR) {
      setLedColor(i, 255, 0, 0);
    }
    else if (i == idexB) {
      setLedColor(i, 0, 0, 255);
    }    
    else {
      setLedColor(i, 0, 0, 0);
    }
  }
  showLeds();
  effectDelay = idelay;
  idex++;
  if (idex >= NUM_LEDS) {
    idex = 0;
  }
}
                       
/*==============================================================================*/
/* Util func Effekte */
/*==============================================================================*/

//-FIND INDEX OF ANTIPODAL OPPOSITE LED
int antipodal_index(int i) {
  //int N2 = int(NUM_LEDS/2);
  int iN = i + TOP_INDEX;
  if (i >= TOP_INDEX) {
    iN = ( i + TOP_INDEX ) % NUM_LEDS; 
  }
  return iN;
}

