#include "ballsensor.h"

Adafruit_MCP3008 BallSensor::adc1;
Adafruit_MCP3008 BallSensor::adc2;
Adafruit_MCP3008 BallSensor::adc3;
float BallSensor::cosVals[BallSensorConstants::BALL_SENSORS] = {0};
float BallSensor::sinVals[BallSensorConstants::BALL_SENSORS] = {0};
float angles[BallSensorConstants::BALL_SENSORS] = {0};
float BallSensor::ball_angle_rad = 0;
PrideUtils::AngleDeg BallSensor::ball_angle_deg = PrideUtils::AngleDeg(0);
float BallSensor::ball_mag = 0;

int BallSensor::ballValues[BallSensorConstants::BALL_SENSORS] = {0};

int ballSensorIndexArray[BallSensorConstants::BALL_SENSORS]; 

// sets sensors to appropriate index b/c electrically they are different
// sets input 1-24
int mapInputToOutput(int input) {
	int output = -1;
	if (input >= 1 && input <= 5) {
		output = input + 11;
	} else if (input <= 13) {
		output = input - 5;
	} else if (input <= 21) {
		output = input + 3;
	} else if (input <= 24) {
		output = input - 13;
	}
	return output;
}

void BallSensor::setup() {
    adc1.begin(27, 11, 12, BallSensorConstants::CS1);
    adc2.begin(27, 11, 12, BallSensorConstants::CS2);
    adc3.begin(27, 11, 12, BallSensorConstants::CS3);

    for (int i = 0; i < BallSensorConstants::BALL_SENSORS; i++) {
        // fixes mapInputToOutput to proper index of [0, 23]
        ballSensorIndexArray[i] = mapInputToOutput(i + 1);
        // Serial.println("Ball Sensor Index: " + String(i) + " " + String(ballSensorIndexArray[i]));
    }

    float step = 0;
    float increment = 360 / BallSensorConstants::BALL_SENSORS;
    
    // setting up the cosVals, sinVals, and angles array to corresponding values
    for (int i = 0; i < BallSensorConstants::BALL_SENSORS; i++) {
        cosVals[i] = cos(step * PI / 180);
        sinVals[i] = sin(step * PI / 180);
        angles[i] = step;
        step += increment;
        // Serial.println("i: " + String(i) + " cosVal: " + cosVals[i] + " sinVal: " + sinVals[i]);
    }
};

// gets IR sensor input & converts it to appropriate human usage b/c sensor gives high value at 0 detected
void BallSensor::read() {
    for (int i = 0; i < BallSensorConstants::BALL_SENSORS; i++) {
        ballValues[i] = 1050 - readBallSensor(i);
        // Serial.print(String(ballValues[i]) + " ");
        // Serial.println("Ball Value: " + String(i) + " " + String(ballValues[i]));
    }
    // Serial.println();
};

int BallSensor::readBallSensor(int x) {
  
  // y is electrical index
  int y = ballSensorIndexArray[x]-1;
  // Serial.println("x: " + String(x) + " y: " +  String(y));
  
  // makes sure to read appropriate adc (analog digital converter)
  int reading;
  if (y < 8) {
    reading = adc1.readADC(y);
  } else if (y < 16) {
    // Serial.println(y-8);
    reading = adc2.readADC(y - 8);
    // Serial.println(reading);
  } else {
    reading = adc3.readADC(y - 16);
  }
  
  return reading;
}
// <angle, magnitude>
tuple<float, float> BallSensor::getBallAngleVector(bool refreshValues) {
    
    //updates IR sensor data
    if (refreshValues) {
        BallSensor::read();
    }
    
    float x = 0;
    float y = 0;

    // gets highest IR sensor value index
    int maxIndex = 0;
    for (int i = 1; i < BallSensorConstants::BALL_SENSORS; ++i) {
      if (ballValues[i] > ballValues[maxIndex]) {
        maxIndex = i;
      }
    }

    // weighting each IR sensor value with 1/12 shit
    int weightedBallValues[BallSensorConstants::BALL_SENSORS] = {0};
    for (int i = 0; i < BallSensorConstants::BALL_SENSORS; i++) {
      int dist = abs(i-maxIndex);
      if (dist > 12) {
        dist = 24 - dist;
      }
      weightedBallValues[i] = ballValues[i] * (12 - dist) / 12;   
    }    

    // final vector sum
    for (int i = 0; i < BallSensorConstants::BALL_SENSORS; i++) {
        x += cosVals[i] * weightedBallValues[i];
        y += sinVals[i] * weightedBallValues[i];
    }

    // calculating radius
    ball_angle_rad = atan2(y, x);
    PrideUtils::AngleRad temp = PrideUtils::AngleRad(ball_angle_rad);
    ball_angle_deg = temp.toDeg();

    // setting magnitude from [0, 1]
    ball_mag = min(sqrt(x * x + y * y) / 3600, 1.0);
    return make_tuple(ball_angle_rad, ball_mag);
}
