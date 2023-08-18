#include <Keypad.h>
#include <ros.h> 
#include <autominy_msgs/SpeedPWMCommand.h>
#include <autominy_msgs/NormalizedSteeringCommand.h>
//V. globales
int V=0;
int S=0.0;
//================================
//          INSTANCIAS DE ROS
//================================
ros::NodeHandle nh_sw;
autominy_msgs::SpeedPWMCommand idata1;
autominy_msgs::NormalizedSteeringCommand idata2;
ros::Publisher Cspeed("/actuators/speed_pwm", &idata1);
ros::Publisher Csteering("/actuators/steering_normalized", &idata2);
//================================
//          KEYBOARD
//================================
const byte ROWS = 4; //four rows
const byte COLS = 4; //four columns
//define the cymbols on the buttons of the keypads
char hexaKeys[ROWS][COLS] = {
  {'0','1','2','3'},
  {'4','5','6','7'},
  {'8','9','A','B'},
  {'C','D','E','F'}
};
byte rowPins[ROWS] = {9, 8, 7, 6}; //row pinouts of the keypad
byte colPins[COLS] = {5, 4, 3, 2}; //column pinouts of the keypad
Keypad customKeypad = Keypad( makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS); 
//================================
//          SETUP
//================================
void setup(){
  nh_sw.initNode();
  nh_sw.advertise(Csteering);
  nh_sw.advertise(Cspeed);
  delay(100);
}
//================================
//          LOOP
//================================
void loop(){
  char customKey = customKeypad.getKey();
  S = analogRead(A0);       //Steering del volante
  S = map(S,270,780,-1.0,1.0); 
  if(S>=1.0){S=1.0;}        //Saturacin del steering
  if(S<=-1.0){S=-1.0;}

  //Control del speed  
  if (customKey=='7'){  //boton UP=hexadecimal 7
    V = 75;            //Adelante
    }  
  if (customKey=='A'){   //boton DOWN=hexadecimal A
    V = -75;           //Atras
    }
  if (customKey=='6'){  //boton 3=hexadecimal 6
    V = 0;              //Detenerse
  }

  //Publica los comandos en ROS
  idata1.value = V;
  idata2.value = S;
  Cspeed.publish(&idata1);
  Csteering.publish(&idata2);
  nh_sw.spinOnce();          //Funcion que crea un bucle
  delay(50);
}
