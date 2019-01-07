#include <Servo.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

class Car {
public:
    void initControler();

    void armControler();

    void writeToControler(int, int);

    float limitMotor(int);

    float limitServo(int);
  
private:
    const int MOTOR_PIN = 10;
    const int SERVO_PIN = 9;

    const int MOTOR_MAX = 2000;
    const int MOTOR_MIN = 1600;
    const int MOTOR_STOP = 1300;

    const int STEER_MIN = 1100;
    const int STEER_MAX = 1900;
    const int STEER_CENTER = 1500;

    const int noAction = 0;

    Servo motor;
    Servo steering;
};

Car car;
int motor_speed;
int steer;
volatile unsigned long dt;
volatile unsigned long t0;

ros::NodeHandle nh;
geometry_msgs::Twist vel_angular_back;

void controlCallback(const geometry_msgs::Twist& ecu) {
    car.writeToControler(ecu.linear.x, ecu.angular.z);
    vel_angular_back.linear.x = ecu.linear.x;
    vel_angular_back.angular.z = ecu.angular.z;
}


ros::Publisher ecu_back("ecu_back", &vel_angular_back);
ros::Subscriber<geometry_msgs::Twist> sub_control("control", controlCallback);

/**
 * ARDUINO INITIALIZATION
 */
void setup() {
    car.initControler();
    car.armControler();
    t0 = millis();
    
    nh.getHardware()->setBaud(57600);
    nh.initNode();
    
    nh.advertise(ecu_back);
    nh.subscribe(sub_control);
}

/**
 * ARDUINO MAIN lOOP
 */
void loop() {
    dt = millis() - t0;
    
    
    ecu_back.publish(&vel_angular_back);
    
    if (dt > 50) {
        t0 = millis();
        // to do
    }
    
    nh.spinOnce();
}

/**
 * CAR CLASS IMPLEMENTATION
 */
void Car::initControler() {
    motor.attach(MOTOR_PIN);
    steering.attach(SERVO_PIN);
}

void Car::armControler() {
    motor.writeMicroseconds(1500);
    steering.writeMicroseconds(STEER_CENTER);
    delay(3000);
}

float Car::limitMotor(int x) {
    if (x == noAction) {
        return MOTOR_STOP;
    }

    if (x > MOTOR_MAX) {
        x = MOTOR_MAX;
    } else if (x < MOTOR_MIN) {
        x = MOTOR_MIN;
    }
    return x;
}

float Car::limitServo(int x) {
    if (x == noAction) {
        return STEER_CENTER;
    }

    if (x > STEER_MAX) {
        x = STEER_MAX;
    } else if (x < STEER_MIN) {
        x = STEER_MIN;
    }
    return x;
}

void Car::writeToControler(int motor_speed, int steer) {
    int motorCMD, servoCMD;
    motorCMD = car.limitMotor(motor_speed);
    servoCMD = car.limitServo(steer);
    //motorCMD = motor_speed;
    //servoCMD = steer;
    motor.writeMicroseconds(motorCMD);
    steering.writeMicroseconds(servoCMD);
}

