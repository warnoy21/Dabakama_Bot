/* Header file for hc_sr04_publisher.ino*/

#include "mbed.h" 


#define PinPE_3 PE_3  // Trigger pin connected to PE_3 (GPIO_4 on Portenta H7)
#define PinPG_3 PG_3  // Echo pin connected to PG_3 (GPIO_5 on Portenta H7)

// Create objects for controlling the trigger and reading the echo pin
mbed::DigitalOut trigPin(PinPE_3);  // Digital output for triggering the ultrasonic sensor
mbed::DigitalIn echoPin(PinPG_3);   // Digital input for receiving the echo signal from the sensor

long microsecondsToInches(long microseconds) {
  // According to Parallax's datasheet for the PING))), there are 73.746
  // microseconds per inch (i.e. sound travels at 1130 feet per second).
  // This gives the distance travelled by the ping, outbound and return,
  // so we divide by 2 to get the distance of the obstacle.
  // See: https://www.parallax.com/package/ping-ultrasonic-distance-sensor-downloads/
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the object we
  // take half of the distance travelled.
  return microseconds / 29 / 2;
}
String ultraSonicDistance(void){

  // establish variables for duration of the ping, and the distance result
  // in inches and centimeters:
  long duration, inches, cm;
  String wheels,message;

  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  trigPin = 0;
  delayMicroseconds(2);
  trigPin = 1;
  delayMicroseconds(10);
  trigPin = 0;

  // Measure the duration of the Echo pulse
  while (echoPin == 0); // Wait for the Echo pin to go HIGH
  unsigned long start_time = micros();
  while (echoPin == 1); // Wait for the Echo pin to go LOW
  unsigned long end_time = micros();
  duration = end_time - start_time;  // Get the duration in microseconds

  // convert the time into a distance
  inches = microsecondsToInches(duration);
  cm = microsecondsToCentimeters(duration);
  if (inches >= 5){
    wheels = "GO";
  }
  else{
    wheels ="STOP";
  }
  
  message= String(inches)+"."+ String(cm)+ ","+ wheels;

  return message;

}

