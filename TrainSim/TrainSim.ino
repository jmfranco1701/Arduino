#include <Keypad.h>
#include <Joystick.h>

#define NUMBUTTONS 4
#define NUMROWS 1
#define NUMCOLS 4

// AXES CALIBRATION TABLE (This is the default table unless calibration mode is activated)
int minL1 = 0;
int maxL1 = 870;
int minL2 = 0;
int maxL2 = 860;
int minL3 = 0;
int maxL3 = 895;
int minL4 = 0;
int maxL4 = 865;
int minL5 = 0;
int maxL5 = 895;
int minJ1 = 0;
int maxJ1 = 1023;
int minJ2 = 0;
int maxJ2 = 1023;

int l1 = 0;
int l2 = 0;
int l3 = 0;
int l4 = 0;
int l5 = 0;
int j1 = 0;
int j2 = 0;

int lastJoyBtnState;

byte buttons[NUMROWS][NUMCOLS] = {
  {0,1,2,3},
};

byte rowPins[NUMROWS] = {1}; 
byte colPins[NUMCOLS] = {0,4,2,5};



Keypad buttbx = Keypad( makeKeymap(buttons), rowPins, colPins, NUMROWS, NUMCOLS); 

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, 
  JOYSTICK_TYPE_JOYSTICK, 5, 0,
  true, true, true, true, true, true, true, true,  false, false, false);

void setup() {
  // put your setup code here, to run once:
  Joystick.begin();

  pinMode(6, INPUT_PULLUP);  
  lastJoyBtnState = !digitalRead(6);
//  Joystick.setButton(4, lastJoyBtnState);
}

void loop() {
  // put your main code here, to run repeatedly:
  CheckAllButtons();
  ReadLeverControls();
  ReadJoystick();
}

void CheckAllButtons(void) {

  int joyBtnState = !digitalRead(6);

  if (lastJoyBtnState != joyBtnState) {
    delay(50);  // debounce time
    
    lastJoyBtnState = joyBtnState;
    Joystick.setButton(4, joyBtnState);
  }
  
  if (buttbx.getKeys()) {
    for (int i=0; i<LIST_MAX; i++) {
      if ( buttbx.key[i].stateChanged ) {
        switch (buttbx.key[i].kstate) {  
          case PRESSED:
          case HOLD:
            Joystick.setButton(buttbx.key[i].kchar, 1);
            break;
          case RELEASED:
          case IDLE:
            Joystick.setButton(buttbx.key[i].kchar, 0);
            break;
        }
      }   
    }
  }
}

void ReadLeverControls(void) {
    l1 = analogRead(3); //Pin A3
    l1 = map(l1,minL1,maxL1,1023,0);
    Joystick.setZAxis(l1);
  
    l2 = analogRead(2); //A2
    l2 = map(l2,minL2,maxL2,1023,0);
    Joystick.setRyAxis(l2);

    l3 = analogRead(1); //A1
    l3 = map(l3,minL3,maxL5,1023,0);
    Joystick.setRzAxis(l3);

    l4 = analogRead(0); //A0
    l4 = map(l4,minL4,maxL4,1023,0);
    Joystick.setThrottle(l4);

    l5 = analogRead(10); //A10
    l5 = map(l5,minL5,maxL5,1023,0);
    Joystick.setRxAxis(l5);
}

void ReadJoystick(void) {
  j1 = analogRead(8); //Pin A8
  j1 = map(j1,minJ1,maxJ1,0,1023);
  Joystick.setYAxis(j1);

  j2 = analogRead(9); //Pin A9 
  j2 = map(j2,minJ2,maxJ2,1023,0);
  Joystick.setXAxis(j2);  
}


