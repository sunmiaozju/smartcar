

#include <Servo.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>

class Car {
public:
    void initControler();    
    void initEncoders();
    
    void armControler();

    void writeToControler(int, int);

    void incBL();
    int  readEncoder();
    
    float limitMotor(int);
    float limitServo(int);
    
    // Used for copying variables shared with interrupts to avoid read/write
    // conflicts later
    void readAndCopyInputs();
    
    const int MOTOR_PIN = 10;
    const int SERVO_PIN = 9;
    const int ENC_BL_PIN = 4;

    const int MOTOR_MAX = 1900;
    const int MOTOR_MIN = 1600;
    const int MOTOR_STOP = 1300;

    const int STEER_MIN = 1100;
    const int STEER_MAX = 1900;
    const int STEER_CENTER = 1500;

    const int noAction = 0;
    Servo motor;
    Servo steering;
    
    volatile uint8_t updateFlagsShared;
    uint8_t updateFlags;
    const int BL_FLAG = 1;
    
    volatile int BL_count_shared = 0;
    int BL_count = 0;
    int val_BL_shared = 0;
    int val_BL = 0;
};

Car car;
int motor_speed;
int steer;
volatile unsigned long dt;
volatile unsigned long t0;

ros::NodeHandle nh;
geometry_msgs::Twist vel_angular_back;
std_msgs::Int32 encoder;

void controlCallback(const geometry_msgs::Twist& ecu) {
    int pwm_speed = abs(ecu.linear.x) / 2 * (car.MOTOR_MAX - car.MOTOR_MIN) + 1550;
    int pwm_angular = ecu.angular.z / 0.06 * (car.STEER_MAX - car.STEER_MIN)/2 + car.STEER_CENTER;
    car.writeToControler(pwm_speed, pwm_angular);
    vel_angular_back.linear.x = pwm_speed;
    vel_angular_back.angular.z = pwm_angular;
}

void incBLCallback() {
  car.incBL();
}

ros::Publisher ecu_back("ecu_back", &vel_angular_back);
ros::Subscriber<geometry_msgs::Twist> sub_control("/ctrl_cmd", controlCallback);
ros::Publisher pub_encoder("hall_sensor", &encoder);

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
    nh.advertise(pub_encoder);
    nh.subscribe(sub_control);
   
}

/**
 * ARDUINO MAIN lOOP
 */
void loop() {
    dt = millis() - t0;
    
    ecu_back.publish(&vel_angular_back);
    car.val_BL = car.readEncoder();
      
    if (dt > 100) {
        t0 = millis();
        encoder.data = car.val_BL;
        pub_encoder.publish(&encoder); 
            
    }
    
    nh.spinOnce();
}

/**
 * CAR CLASS IMPLEMENTATION
 */
void Car::initEncoders() {
  pinMode(ENC_BL_PIN, INPUT_PULLUP);
  //enableInterrupt(ENC_BL_PIN, incBLCallback, CHANGE);
  //enableInterrupt(ENC_BL_PIN, incBLCallback, RISING);
  //attachInterrupt(ENC_BL_PIN, incBLCallback, CHANGE);
} 

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

void Car::incBL() {
  BL_count_shared++;
  val_BL = digitalRead(ENC_BL_PIN);
  updateFlagsShared |= BL_FLAG;
} 

void Car::readAndCopyInputs() {
  // check shared update flags to see if any channels have a new signal
  if (updateFlagsShared) {
    // Turn off interrupts, make local copies of variables,then turn interrupts back on. 
    // Without doing this, an interrupt could update a shared  variable while the loop is in the middle of reading it
    noInterrupts(); 
    
    updateFlags = updateFlagsShared;
    if(updateFlags & BL_FLAG) {
      BL_count = BL_count_shared;
      val_BL = val_BL_shared;
      BL_count_shared = 0;
    }
    
    // clear shared update flags and turn interrupts back on
    updateFlagsShared = 0;
    interrupts();
  }
}

int Car::readEncoder(){
    return digitalRead(ENC_BL_PIN);
}
