
// #include "AS5600.h"


// AS5600 as5600;   //  use default Wire


// void setup()
// {
//   Serial.begin(115200);
//   Serial.println(__FILE__);
//   Serial.print("AS5600_LIB_VERSION: ");
//   Serial.println(AS5600_LIB_VERSION);

//   Wire.begin();

//   as5600.begin(14);  //  set direction pin.
//   as5600.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.a
//   as5600.setMPosition(4095);
//   as5600.setZPosition(0);
//   as5600.setMaxAngle(4095);  // Set full 360° range
//   as5600.setOutputMode(1);
//   delay(10);
// //   Serial.println(as5600.getAddress());
// //   Serial.println(as5600.getAngularSpeed());
// //   Serial.println(as5600.getConfigure());
// //   Serial.println(as5600.getCumulativePosition());
// //   Serial.println(as5600.getDirection());
// //   Serial.println(as5600.getFastFilter());
// //   Serial.println(as5600.getHysteresis());
// //   Serial.println(as5600.getMaxAngle());
// //   Serial.println(as5600.getMPosition());
// //   Serial.print("Offset: ");
// //   Serial.println(as5600.getOffset());
// //   Serial.println(as5600.getOutputMode());
// //   Serial.println(as5600.getPowerMode());
// //   Serial.println(as5600.getPWMFrequency());
// //   Serial.println(as5600.getRevolutions());
// //   Serial.println(as5600.getSlowFilter());
// //   Serial.println(as5600.getWatchDog());
// //   Serial.println(as5600.getZMCO());
// //   Serial.println(as5600.getZPosition());
//     pinMode(34, INPUT);
//     // delay(3000);
//     Serial.println(as5600.getZMCO());
//     // Serial.println(as5600.getZPosition());
//     // delay(1000);
//     if (as5600.detectMagnet()){
//         Serial.println("Magnet Present, beginning burn sequence");
//         Serial.println(as5600.getZMCO());
//     Serial.println("Burning in 3");
//     delay(1000);
//     Serial.println("2");
//     delay(1000);
//     Serial.println("1");
//     delay(1000);
//     if (as5600.detectMagnet()){
//         Serial.println("Magnet Present, burning now");
//         delay(100);
//         // as5600.burnSetting();
//         as5600.burnAngle();
//     }}
//     else{
//         Serial.println("Magnet not present");
//     }
//     delay(1000);
// //   as5600.setOutputMode()
// }


// void loop()
// {
//   Serial.print(millis());
//   Serial.print("\t");
//   Serial.print(as5600.readAngle());
//   Serial.print("\t");
//   Serial.println(as5600.rawAngle() * AS5600_RAW_TO_DEGREES);
//   Serial.print("Analog Read Value: " + String(analogRead(34)));

//   delay(100);
// }