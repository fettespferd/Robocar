#include <Pololu3pi.h>
#include <PololuQTRSensors.h>
#include <OrangutanLCD.h>
#include <OrangutanPushbuttons.h>
#include <OrangutanBuzzer.h>
#include <OrangutanMotors.h>
#include <OrangutanSerial.h>

OrangutanLCD lcd;
OrangutanBuzzer buzzer;
OrangutanMotors motors;

Pololu3pi robot;

bool once = true;

// Data for generating the characters used in load_custom_characters
// and display_readings.  By reading levels[] starting at various
// offsets, we can generate all of the 7 extra characters needed for a
// bargraph.  This is also stored in program space.
const char levels[] PROGMEM = {
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111
};

// This function loads custom characters into the LCD.  Up to 8
// characters can be loaded; we use them for 7 levels of a bar graph.
void load_custom_characters()
{
  OrangutanLCD::loadCustomCharacter(levels + 0, 0); // no offset, e.g. one bar
  OrangutanLCD::loadCustomCharacter(levels + 1, 1); // two bars
  OrangutanLCD::loadCustomCharacter(levels + 2, 2); // etc...
  OrangutanLCD::loadCustomCharacter(levels + 3, 3);
  OrangutanLCD::loadCustomCharacter(levels + 4, 4);
  OrangutanLCD::loadCustomCharacter(levels + 5, 5);
  OrangutanLCD::loadCustomCharacter(levels + 6, 6);
  OrangutanLCD::clear(); // the LCD must be cleared for the characters to take effect
}

char buffer[100];
unsigned char read_index = 0;

bool read_command_string()
{
  if(serial_get_received_bytes() == 0)
    return false;

  while(true)
  {
    while(serial_get_received_bytes() == read_index);
    char ret = buffer[read_index];
    read_index++; 

    if (ret == '\0')
    {
      read_index = 0;
      return true;
    }
  }
}

void setup() {
  robot.init(2000);

  serial_set_baud_rate(115200);
  serial_receive(buffer, 100);
  
  while(!OrangutanPushbuttons::isPressed(BUTTON_B))
  {
    delay(100);
  }
  OrangutanPushbuttons::waitForRelease(BUTTON_B);
  delay(1000);
  
  for (int counter=0; counter<80; counter++)
  {
    if (counter < 20 || counter >= 60)
      OrangutanMotors::setSpeeds(40, -40);
    else
      OrangutanMotors::setSpeeds(-40, 40);

    // This function records a set of sensor readings and keeps
    // track of the minimum and maximum values encountered.  The
    // IR_EMITTERS_ON argument means that the IR LEDs will be
    // turned on during the reading, which is usually what you
    // want.
    robot.calibrateLineSensors(IR_EMITTERS_ON);

    // Since our counter runs to 80, the total delay will be
    // 80*20 = 1600 ms.
    delay(20);
  }
  OrangutanMotors::setSpeeds(0, 0);
  load_custom_characters();
  lcd.print("KILL ME");
}



// This function displays the sensor readings using a bar graph.
void display_readings(const unsigned int *calibrated_values)
{
  unsigned char i;
  OrangutanLCD::gotoXY(0, 1);

  for (i=0;i<5;i++) {
    // Initialize the array of characters that we will use for the
    // graph.  Using the space, an extra copy of the one-bar
    // character, and character 255 (a full black box), we get 10
    // characters in the array.
    const char display_characters[10] = { ' ', 0, 0, 1, 2, 3, 4, 5, 6, 255 };

    // The variable c will have values from 0 to 9, since
    // calibrated values are in the range of 0 to 1000, and
    // 1000/101 is 9 with integer math.
    char c = display_characters[calibrated_values[i] / 101];

    // Display the bar graph character.
    OrangutanLCD::print(c);
  }
}

int stops = 2;
int ticks_after_stop = 0;
int last_proportional;
bool first_plan = true;

enum class EventDecision { FORWARD, LEFT, RIGHT };
EventDecision *decisions = NULL;
int decision_length = 0;

String input_string;
bool string_complete;

bool read_decisions()
{
    if (buffer[0] == 's')
    {
      OrangutanMotors::setSpeeds(0, 0);
      OrangutanBuzzer::playNote(NOTE_G(5), 500, 15);
      delay(500);
      
      buffer[0] = '\0';
      serial_receive(buffer, 100);

      return false;
    }
    
    if (decisions)
      delete[] decisions;

    decisions = new EventDecision[strlen(buffer)];
    decision_length = strlen(buffer);
    
    for (int i = 0; i < decision_length; ++i)
      switch(buffer[i])
      {
        case 'f':
          decisions[i] = EventDecision::FORWARD;
          break;
        case 'l':
          decisions[i] = EventDecision::LEFT;
          break;
        case 'r':
          decisions[i] = EventDecision::RIGHT;
          break;
        default:
          decisions[i] = EventDecision::FORWARD;
          break;
      }

   buffer[0] = '\0';
   serial_receive(buffer, 100);

   return true;
}

int current_decision = 0;

const int maximum = 80;
const int turn_speed = 50;

char decision_point_buffer[1] = {1};

void loop() {
  if (read_command_string())
  {
    lcd.clear();
    lcd.print(buffer);
    if (read_decisions())
    {
      current_decision = 0;
      if (!first_plan)
      {
        OrangutanMotors::setSpeeds(40, -40);
        delay(1000);
      }
      else
        first_plan = false;
    }
  }
  
  if (decision_length == 0)
  {
    OrangutanMotors::setSpeeds(0, 0);
    delay(1000);
    return;
  }
  
  unsigned int sensor_values[5];
  unsigned int position = robot.readLine(sensor_values, IR_EMITTERS_ON);
  lcd.print(position);
  display_readings(sensor_values);

  int count_high = 0;
  for (int i = 0; i < 5; ++i)
    if (sensor_values[i] > 700)
      count_high++;

  if (current_decision == decision_length)
  {
    int count_high = 0;
    for (int i = 0; i < 5; ++i)
      if (sensor_values[i] > 400)
        count_high++;
        
    if (count_high == 0)
    {
      OrangutanMotors::setSpeeds(maximum, maximum);
      delay(300);

      OrangutanBuzzer::playNote(NOTE_G(5), 1000, 15);
      OrangutanMotors::setSpeeds(0, 0);
      while(true);
    }
  }

  if (count_high > 2)
  {
    OrangutanMotors::setSpeeds(0, 0);
   
    switch (decisions[current_decision])
    {
      case EventDecision::FORWARD:
        OrangutanMotors::setSpeeds(maximum, maximum);
        delay(200);
        break;
      case EventDecision::LEFT:
        OrangutanMotors::setSpeeds(round(turn_speed * 0.27), turn_speed);
        delay(700);
        break;
      case EventDecision::RIGHT:
        OrangutanMotors::setSpeeds(turn_speed, round(turn_speed * 0.27));
        delay(700);
        break;
    }

    serial_send(decision_point_buffer, 1);

    current_decision++;
  }
  else
  {
    int proportional = (int)position - 2000;
    int derivative = proportional - last_proportional;
    last_proportional = proportional;

    int power_difference = proportional/20 + derivative * 3 / 2;

    power_difference = min(power_difference, maximum);
    power_difference = max(power_difference, -maximum);

    if (power_difference < 0)
      OrangutanMotors::setSpeeds(maximum + power_difference, maximum);
    else
      OrangutanMotors::setSpeeds(maximum, maximum - power_difference);
  }
}
