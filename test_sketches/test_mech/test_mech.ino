#include <Servo.h>

Servo servo_arm;
Servo servo_tray;

#define const_servo_pos_tray_up 78
#define const_servo_pos_tray_down 105
#define const_servo_pos_arm_in 145
#define const_servo_pos_arm_out 25

#define port_servo_arm 9
#define port_servo_tray 10

void setupServos()
{
    servo_arm.write(const_servo_pos_arm_in);
    servo_tray.write(const_servo_pos_tray_up);

    servo_arm.attach(port_servo_arm);
    servo_tray.attach(port_servo_tray);

}

void setup()
{
    Serial.begin(9600);
    setupServos();
    Serial.println("Closed");

    delay(1000);
}

void loop()
{
    // Open

    servo_arm.write(const_servo_pos_arm_out);
    servo_tray.write(const_servo_pos_tray_down);

    Serial.println("open");
    delay(3000);

    // Closed

    servo_arm.write(const_servo_pos_arm_in);
    servo_tray.write(const_servo_pos_tray_up);

    Serial.println("closed");
    delay(3000);
    
}
