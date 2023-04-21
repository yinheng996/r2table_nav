#include <Keypad.h>

#define ROW_NUM     4 // four rows
#define COLUMN_NUM  3 // three columns

char keys[ROW_NUM][COLUMN_NUM] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};

byte pin_rows[ROW_NUM] = {33, 25, 26, 27}; // GIOP18, GIOP5, GIOP17, GIOP16 connect to the row pins
byte pin_column[COLUMN_NUM] = {14, 12, 13};  // GIOP4, GIOP0, GIOP2 connect to the column pins

Keypad keypad = Keypad( makeKeymap(keys), pin_rows, pin_column, ROW_NUM, COLUMN_NUM );

void setup() {
  Serial.begin(9600);
}
  
void loop() {
  char key = keypad.getKey(); // Read the key

  // Print if key pressed
  if (key) {
    Serial.println(key);
    if (key >= '1' && key <= '6') {   // Check if key is between '1' and '6'
      int table_num = key - '0';      // Convert key char to int
      Serial.println("table_num is: ");
      Serial.println(table_num);
    }
  }
}