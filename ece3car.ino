#include <ECE3.h>


#define LED RED_LED

int looped = 0;

// initialize leds for wall count visualization
const int LED_RF = 41;
const int LED_LB = 57;
const int LED_RB = 58;

// bool set according to track type (ribbon/straight track)
const bool ribbon = true;
const float turnval = 5000.0;

// constants for max car speed and turn speed
const int maxspd = 150;
const int minspd = 20;
const int turnspd = 70;
const int turntime = 890;

// intialize sensor array
uint16_t sensorValues[8];

// kp and kd constants
const double kp = 0.001;
const double kd = 35;
//const double ki = 1.0;

// pin numbers
const int left_nslp_pin = 31;
const int left_dir_pin = 29;
const int left_pwm_pin = 40;

const int right_nslp_pin = 11;
const int right_dir_pin = 30;
const int right_pwm_pin = 39;

// initial speed for the car
int leftSpd = maxspd - 10;
int rightSpd = maxspd - 10;

//uint16_t errorarr[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int lasterror;

bool startwall = true;
int skipwall = 0;

void setup()
{
  lasterror = 0;
  pinMode(LED, OUTPUT);
  pinMode(LED_RF, OUTPUT);
  pinMode(LED_LB, OUTPUT);
  pinMode(LED_RB, OUTPUT);

  pinMode(left_nslp_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);
  pinMode(left_pwm_pin, OUTPUT);

  pinMode(right_nslp_pin, OUTPUT);
  pinMode(right_dir_pin, OUTPUT);
  pinMode(right_pwm_pin, OUTPUT);

  digitalWrite(left_dir_pin, LOW);
  digitalWrite(left_nslp_pin, HIGH);
  digitalWrite(right_dir_pin, LOW);
  digitalWrite(right_nslp_pin, HIGH);

  ECE3_Init();
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  delay(2000);


}

double fusion() {
  //normalize vals
  float v1 = (sensorValues[0] - 687) * (1000.0 / 1813.0);
  float v2 = (sensorValues[1] - 505) * (1000.0 / 1830.0);
  float v3 = (sensorValues[2] - 507) * (1000.0 / 1993.0);
  float v4 = (sensorValues[3] - 371) * (1000.0 / 1491.0);
  float v5 = (sensorValues[4] - 619) * (1000.0 / 1629.0);
  float v6 = (sensorValues[5] - 416) * (1000.0 / 2084.0);
  float v7 = (sensorValues[6] - 461) * (1000.0 / 2039.0);
  float v8 = (sensorValues[7] - 596) * (1000.0 / 1904.0);

  if (v1 < 0.0) {
    v1 = 0.0;
  }
  if (v2 < 0.0) {
    v2 = 0.0;
  }
  if (v3 < 0.0) {
    v3 = 0.0;
  }
  if (v4 < 0.0) {
    v4 = 0.0;
  }
  if (v5 < 0.0) {
    v5 = 0.0;
  }
  if (v6 < 0.0) {
    v6 = 0.0;
  }
  if (v7 < 0.0) {
    v7 = 0.0;
  }
  if (v8 < 0.0) {
    v8 = 0.0;
  }
  //fuse all normalized vals
  float fusedvalue = (-15 * v1) + (-14 * v2) + (-12 * v3) + (-8 * v4) + (8 * v5) + (12 * v6) + (14 * v7) + (15 * v8);
  return fusedvalue;
}

void turnaround() {
  skipwall = 0;
  analogWrite(left_pwm_pin, 0);
  analogWrite(right_pwm_pin, 0);
  digitalWrite(LED, HIGH);
  delay(100);
  digitalWrite(left_dir_pin, HIGH);
  analogWrite(left_pwm_pin, turnspd);
  analogWrite(right_pwm_pin, turnspd);
  delay(turntime);
  digitalWrite(LED, LOW);
  digitalWrite(left_dir_pin, LOW);
  analogWrite(left_pwm_pin, 0);
  analogWrite(right_pwm_pin, 0);
  //delay(100);
}

// total unweighted value from all sensors
float checkwall() {
  float unweighted = 0;
  float v1 = (sensorValues[0] - 687) * (1000.0 / 1813.0);
  float v2 = (sensorValues[1] - 505) * (1000.0 / 1830.0);
  float v3 = (sensorValues[2] - 507) * (1000.0 / 1993.0);
  float v4 = (sensorValues[3] - 371) * (1000.0 / 1491.0);
  float v5 = (sensorValues[4] - 619) * (1000.0 / 1629.0);
  float v6 = (sensorValues[5] - 416) * (1000.0 / 2084.0);
  float v7 = (sensorValues[6] - 461) * (1000.0 / 2039.0);
  float v8 = (sensorValues[7] - 596) * (1000.0 / 1904.0);
  unweighted = v1 + v2 + v3 + v4 + v5 + v6 + v7 + v8;
  return unweighted;
}

void loop()
{
  if (looped != 2) {
    if (skipwall == 0) {
      digitalWrite(LED_RF, HIGH);
      digitalWrite(LED_LB, LOW);
      digitalWrite(LED_RB, LOW);
    }
    if (skipwall == 1) {
      digitalWrite(LED_LB, HIGH);
      digitalWrite(LED_RF, LOW);
      digitalWrite(LED_RB, LOW);
    }
    if (skipwall == 2) {
      digitalWrite(LED_RB, HIGH);
      digitalWrite(LED_RF, LOW);
      digitalWrite(LED_LB, LOW);
    }

    int perr = 0;
    int derror = 0;
    //int ierror = 0;

    // read raw sensor values
    ECE3_read_IR(sensorValues);



    //CHECK WALL STUFF
    float total = checkwall();
    if (ribbon) {
      if (total > turnval) {
        if (skipwall == 2) {
          digitalWrite(LED_RF, LOW);
          digitalWrite(LED_LB, LOW);
          digitalWrite(LED_RB, LOW);
          if (looped == 0)
            turnaround();
          looped++;
          startwall = true;
        }

        else if (startwall) {
          startwall = false;

        }
      }
      if (!startwall && total < turnval) {
        startwall = true;
        skipwall++;
      }
    }

    if (!ribbon) {
      if (total > turnval) {
        turnaround();
      }
    }

    float fusedvalue = fusion();

    //add up raw vals for line
    //counter to keep track of intersection or wall


    //CORRECTION NUMBERS
    int leftCorrection = 0;
    int rightCorrection = 0;

    //P Controller
    perr = kp * fusedvalue;

    if (perr < 0 && perr > -80) {
      leftCorrection += abs(perr);
      rightCorrection -= abs(perr);

    }
    else if (perr > 0 && perr < 80) {
      leftCorrection -= abs(perr);
      rightCorrection += abs(perr);

    }

    //D Controller
    /*
      int lasterr = errorarr[9];
      for(int i = 0; i < 9; i++) {
       errorarr[i + 1] = errorarr[i];
      }
      errorarr[0] = perr;
    */

    derror = kd * (perr - lasterror);
    lasterror = perr;

  // calculate correction for left and right wheels
    if (derror < 0) {
      if (perr < 0) {
        leftCorrection += abs(derror);
        rightCorrection -= abs(derror);
      }
      if (perr > 0) {
        leftCorrection += abs(derror);
        rightCorrection -= abs(derror);
      }
    }
    else if (derror > 0) {
      if (perr < 0) {
        leftCorrection -= abs(derror);
        rightCorrection += abs(derror);
      }
      if (perr > 0) {
        leftCorrection -= abs(derror);
        rightCorrection += abs(derror);
      }
    }



    //ADD CORRECTION TO SPEED
    if (leftCorrection < maxspd) {
      leftSpd += leftCorrection;
    }
    if (rightCorrection < maxspd) {
      rightSpd += rightCorrection;
    }


  // limit the speed of each wheel
    if (leftSpd > maxspd) {
      leftSpd = maxspd;
    }
    if (rightSpd > maxspd) {
      rightSpd = maxspd;
    }
    if (leftSpd < minspd) {
      leftSpd = minspd;
    }
    if (rightSpd < minspd) {
      rightSpd = minspd;
    }
  // stop car after two full loops
    if (looped == 2) {
      analogWrite(left_pwm_pin, 0);
      analogWrite(right_pwm_pin, 0);
    }
    else {
      if (leftSpd >= minspd && leftSpd <= maxspd && rightSpd >= minspd && rightSpd <= maxspd) {
        analogWrite(left_pwm_pin, leftSpd);
        analogWrite(right_pwm_pin, rightSpd);
      }
    }


  }


}
