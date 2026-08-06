#include <Wire.h>
#include <avr/wdt.h>
#include <PID_v2.h>


const float TARGET_TIME_S = 61.0f;    // announced target time in seconds
const float TOTAL_COURSE_CM = 630.0f; // sum of ALL forward()/backward() distances in loop() below
const int TOTAL_TURNS = 9;            // total number of turns in your route


// PIN CONST
const int STBY = 13, START_SW = 12;
const int PWMA = 6, AIN1 = 7, AIN2 = 8, ENCA_L = 2, ENCB_L = 4;
const int PWMB = 9, BIN1 = 10, BIN2 = 11, ENCA_R = 3, ENCB_R = 5;
const int QTR_L = A0, QTR_R = A1;

// HARDWARE CONST
float WHEELBASE_CM = 13.75;
const float CM_PER_TICK = 0.018850f;

// QTR-SPECIFIC CONST
int qtr_baseline_L = 0;
int qtr_baseline_R = 0;
const int QTR_THRESHOLD = 200;
const float FINISH_CHECK_CM = 25.0f;
bool isFinalLeg = false;

// IMU-SPECIFIC CONST
float ideal_heading = 0.0;
volatile long left_counts = 0, right_counts = 0;
long prev_left_counts = 0, prev_right_counts = 0;
float gyro_z_offset = 0.0;
double current_heading = 0.0;
float raw_odometry_heading = 0.0;
float distance_traveled = 0.0;

// FEED-FORWARD & PID TUNING CONST
double Kff_L = 2.3699;
double Kff_R = 2.5371;

double Kp_vel = 1.2, Ki_vel = 0.4, Kd_vel = 0.0;
double Kp_head = 0.7, Ki_head = 0.05, Kd_head = 0.0;

// PID VAR
double target_heading = 0.0;
double steering_correction = 0.0;
double base_forward_speed = 0.0;
double target_speed_L = 0.0, target_speed_R = 0.0;
double actual_speed_L = 0.0, actual_speed_R = 0.0;
double output_PWM_L = 0.0, output_PWM_R = 0.0;

// PID CONTROLLER OBJECTS
PID headingPID(&current_heading, &steering_correction, &target_heading,
               Kp_head, Ki_head, Kd_head, DIRECT);
PID leftVelPID(&actual_speed_L, &output_PWM_L, &target_speed_L,
               Kp_vel, Ki_vel, Kd_vel, DIRECT);
PID rightVelPID(&actual_speed_R, &output_PWM_R, &target_speed_R,
                Kp_vel, Ki_vel, Kd_vel, DIRECT);

// MOTION CONST
const float MIN_SPEED_CMPS = 4.0;
const float TURN_SPEED = 14.0;
const float TURN_MIN_SPEED = 6.0;
const float TURN_THRESHOLD_DEG = 0.55;
const unsigned long TURN_SETTLE_MS = 100;
const unsigned long MOTION_TIMEOUT_MS = 25000;

float filt_speed_L = 0.0, filt_speed_R = 0.0;
const float SPEED_ALPHA = 0.35f;

// TIMING VAR
unsigned long run_start_ms = 0;
bool run_started = false;
float course_dist_covered = 0.0f;
int turns_completed = 0;
float time_per_turn_estimate = 3.68f; 

// MVMT DECLARATIONS
void forward(float target_distance, float max_speed = 50.0, bool reverse = false);
void baseDrive(float target_distance, float max_speed, bool reverse = false);
void backward(float dist, float speed = 40.0);
void left();
void right();
void turnToHeading(float delta_degrees);

// ENCODER INTERPRETER
void read_encoder_L() {
  left_counts += (digitalRead(ENCA_L) == digitalRead(ENCB_L)) ? -1 : 1;
}
void read_encoder_R() {
  right_counts += (digitalRead(ENCA_R) == digitalRead(ENCB_R)) ? 1 : -1;
}

// IMU CALIBRATION ON STARTUP
void setup() {

  MCUSR = 0;
  wdt_disable();

  Serial.begin(9600);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, LOW);
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(START_SW, INPUT_PULLUP);
  pinMode(ENCA_L, INPUT);
  pinMode(ENCB_L, INPUT);
  pinMode(ENCA_R, INPUT);
  pinMode(ENCB_R, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA_L), read_encoder_L, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA_R), read_encoder_R, CHANGE);

  delay(500);

  Wire.begin();
  Wire.setClock(100000);
  Wire.setWireTimeout(3000, true);  

  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  delay(250);

  calibrateIMU();
  wdt_enable(WDTO_1S);
  wdt_reset();
  calibrateQTR();

  headingPID.SetMode(AUTOMATIC);
  headingPID.SetOutputLimits(-12.0, 12.0);
  headingPID.SetSampleTime(20);

  leftVelPID.SetMode(AUTOMATIC);
  leftVelPID.SetOutputLimits(-75.0, 75.0);
  leftVelPID.SetSampleTime(20);

  rightVelPID.SetMode(AUTOMATIC);
  rightVelPID.SetOutputLimits(-75.0, 75.0);
  rightVelPID.SetSampleTime(20);

  Serial.println("Ready!");
}

// RESET ALL TEMP NAVIGATION-BASED VAR ON LEG END
void resetNavigation() {
  wdt_reset();
  delay(300);
  noInterrupts();
  left_counts = right_counts = prev_left_counts = prev_right_counts = 0;
  interrupts();
  distance_traveled = 0.0;
  raw_odometry_heading = 0.0;
  current_heading = 0.0;
  target_heading = 0.0;
  ideal_heading = 0.0;
  steering_correction = 0.0;
  filt_speed_L = 0.0;
  filt_speed_R = 0.0;
  isFinalLeg = false;

  course_dist_covered = 0.0f;
  turns_completed = 0;
  time_per_turn_estimate = 3.68f;

  flushAllPIDs();
  run_started = false;
  Serial.println("Navigation reset.");
}

// FLUSH PID; PREVENT ERROR WINDUP
void flushAllPIDs() {
  leftVelPID.SetMode(MANUAL);
  rightVelPID.SetMode(MANUAL);
  headingPID.SetMode(MANUAL);

  output_PWM_L = 0.0;
  output_PWM_R = 0.0;
  steering_correction = 0.0;

  leftVelPID.SetMode(AUTOMATIC);
  rightVelPID.SetMode(AUTOMATIC);
  headingPID.SetMode(AUTOMATIC);
}

// MAIN CODE TO EDIT AT COMPETITION: ctrl f (for easy access)
void loop() {
  wdt_reset();
  if (digitalRead(START_SW) == LOW) {
    resetNavigation();
    delay(500);
    forward(30);
    right();
    forward(100);
    left();
    forward(50);
    right();
    forward(50);
    left();
    forward(50);
    backward(50);
    left();
    forward(50);
    right();
    forward(50);
    left();
    forward(50);
    right();
    forward(50);
    right();
    forward(50);
    isFinalLeg = true;
    backward(50);
    isFinalLeg = false;

    while (digitalRead(START_SW) == LOW) {
      wdt_reset();
      delay(10);
    }
    wdt_reset();
    delay(500);
  }
}

// forward logic; dynamic speed based on turns and time remaining 
void forward(float target_distance, float max_speed, bool reverse) {
  if (!run_started) {
    run_start_ms = millis();
    run_started = true;
  }

  float elapsed_s = (millis() - run_start_ms) / 1000.0f;
  int turns_remaining = TOTAL_TURNS - turns_completed;
  float future_turn_budget_s = turns_remaining * time_per_turn_estimate;
  float remaining_time_s = TARGET_TIME_S - elapsed_s - future_turn_budget_s;
  float remaining_dist_cm = TOTAL_COURSE_CM - course_dist_covered;

  float segment_max_speed = max_speed;
  if (remaining_time_s > 0.5f && remaining_dist_cm > 1.0f) {
    float required_speed = remaining_dist_cm / remaining_time_s;
    segment_max_speed = constrain(required_speed, (float)MIN_SPEED_CMPS, max_speed);

    Serial.print("Seg max speed: ");
    Serial.print(segment_max_speed);
    Serial.print(" | Remaining time: ");
    Serial.print(remaining_time_s);
    Serial.print(" | Remaining dist: ");
    Serial.println(remaining_dist_cm);
  }

  baseDrive(target_distance, segment_max_speed, reverse);
  course_dist_covered += target_distance;
}

// use forward logic for moving backwards
void backward(float dist, float speed) {
  forward(dist, speed, true);
}

// movement logic
void baseDrive(float target_distance, float max_speed, bool reverse) {
  digitalWrite(STBY, HIGH);
  delay(50);
  noInterrupts();
  left_counts = right_counts = prev_left_counts = prev_right_counts = 0;
  interrupts();

  distance_traveled = 0.0;
  filt_speed_L = 0.0;
  filt_speed_R = 0.0;
  target_heading = ideal_heading;
  steering_correction = 0.0;

  float dir = reverse ? -1.0f : 1.0f;
  current_heading = current_heading;  // empirical evidence shows current_heading leads to better results; could maybe experiment with using ideal_heading

  flushAllPIDs();

  unsigned long prev_micros = micros();
  unsigned long start_millis = millis();

  const float ACCEL = 40.0f;
  float dynamic_decel = 40.0f;
  bool reached_max_speed = false;
  float current_target_speed = MIN_SPEED_CMPS;
  long snap_L = 0, snap_R = 0;
  int stall_ticks_L = 0, stall_ticks_R = 0;
  float boost_L = 0.0f, boost_R = 0.0f;
  const int STALL_CONFIRM_TICKS = 2;
  const float BOOST_RATE = 15.0f;
  const float BOOST_MAX = 60.0f;
  const float BOOST_DECAY = 20.0f;
  noInterrupts();
  snap_L = left_counts;
  snap_R = right_counts;
  interrupts();
  while (true) {
    // Attempt to create a trapezoidal acceleration profile
    unsigned long now_micros = micros();

    if (now_micros - prev_micros >= 20000UL) {
      wdt_reset();
      float dt = (now_micros - prev_micros) / 1000000.0f;
      prev_micros = now_micros;

      updateStateEstimator(dt, false);
      float remaining = target_distance - distance_traveled;
      if (checkFinishLine(remaining)) {
        activeBrake();
        return;
      }
      if (millis() - start_millis > MOTION_TIMEOUT_MS) {
        emergencyStop();
        return;
      }

      if (!reached_max_speed && current_target_speed >= max_speed) {
        reached_max_speed = true;
        float vf_sq = max_speed * max_speed;
        float vi_sq = MIN_SPEED_CMPS * MIN_SPEED_CMPS;
        if (distance_traveled > 0.5f && vf_sq > vi_sq) {
          dynamic_decel = (vf_sq - vi_sq) / (2.0f * distance_traveled);
          dynamic_decel = constrain(dynamic_decel, 10.0f, 80.0f);
        }
      }

      float avg_speed = (abs(actual_speed_L) + abs(actual_speed_R)) / 2.0f;
      // for if robot physically coast after brake
      float stop_trigger = max(0.1f, avg_speed * 0.03f);
      if (remaining <= stop_trigger) {
        Serial.print("Move finished, dist: ");
        Serial.println(distance_traveled);
        activeBrake();
        return;
      }

      float safe_braking_speed = sqrt(2.0f * dynamic_decel * remaining);
      float desired_speed = min(max_speed, safe_braking_speed);
      desired_speed = max((float)MIN_SPEED_CMPS, desired_speed);

      if (current_target_speed < desired_speed) {
        current_target_speed += ACCEL * dt;
        current_target_speed = min(current_target_speed, desired_speed);
      } else {
        current_target_speed -= dynamic_decel * dt;
        current_target_speed = max(current_target_speed, desired_speed);
      }

      base_forward_speed = current_target_speed;

      headingPID.Compute();

      target_speed_L = (dir * base_forward_speed) - steering_correction;
      target_speed_R = (dir * base_forward_speed) + steering_correction;

      noInterrupts();
      long cur_L = left_counts;
      long cur_R = right_counts;
      interrupts();
      // prevent stall from too little speed; relies on encoder reading to judge whether robot is moving with traction on ground
      bool wantMove = (current_target_speed > 1.0f);

      if (wantMove && abs(cur_L - snap_L) < 5) {
        stall_ticks_L++;
        if (stall_ticks_L >= STALL_CONFIRM_TICKS) boost_L = min(boost_L + BOOST_RATE, BOOST_MAX);
      } else {
        stall_ticks_L = 0;
        boost_L = max(boost_L - BOOST_DECAY, 0.0f);
      }
      snap_L = cur_L;

      if (wantMove && abs(cur_R - snap_R) < 5) {
        stall_ticks_R++;
        if (stall_ticks_R >= STALL_CONFIRM_TICKS) boost_R = min(boost_R + BOOST_RATE, BOOST_MAX);
      } else {
        stall_ticks_R = 0;
        boost_R = max(boost_R - BOOST_DECAY, 0.0f);
      }
      snap_R = cur_R;

      double dir_L = (target_speed_L >= 0) ? 1.0 : -1.0;
      double dir_R = (target_speed_R >= 0) ? 1.0 : -1.0;
      leftVelPID.Compute();
      rightVelPID.Compute();
      double final_pwm_L = constrain(output_PWM_L + Kff_L * target_speed_L + dir_L * boost_L, -255.0, 255.0);
      double final_pwm_R = constrain(output_PWM_R + Kff_R * target_speed_R + dir_R * boost_R, -255.0, 255.0);
      driveMotor(1, final_pwm_L);
      driveMotor(2, final_pwm_R);
    }
  }
}

// turning logics; repositions robot to desired dowel position through translations
void left() {
  unsigned long t_start = millis();
  baseDrive(5.2, 10.0, false);
  turnToHeading(90.0);
  baseDrive(3.75, 10.0, true);
  float actual_turn_s = (millis() - t_start) / 1000.0f;
  time_per_turn_estimate = (0.7f * time_per_turn_estimate) + (0.3f * actual_turn_s);
  turns_completed++;
}

void right() {
  unsigned long t_start = millis();
  baseDrive(3.75, 10.0, false);
  turnToHeading(-90.0);
  baseDrive(5.20, 10.0, true);
  float actual_turn_s = (millis() - t_start) / 1000.0f;
  time_per_turn_estimate = (0.7f * time_per_turn_estimate) + (0.3f * actual_turn_s);
  turns_completed++;
}
// entirely IMU-based turn system; encoder was tested but unfortunately too inaccurate esp with water bottle
void turnToHeading(float delta_degrees) {
  unsigned long turn_start_ms = millis();
  digitalWrite(STBY, HIGH);
  noInterrupts();
  left_counts = right_counts = prev_left_counts = prev_right_counts = 0;
  interrupts();
  distance_traveled = 0.0;
  flushAllPIDs();

  ideal_heading += delta_degrees;
  float goal_heading = ideal_heading;

  const float T_ACCEL = 30.0f;
  float T_DECEL = 30.0f;            
  float current_min_speed = TURN_MIN_SPEED;
  float current_turn_speed = TURN_MIN_SPEED;
  bool detected_heavy = false;

  unsigned long settled_since = 0;
  bool settling = false;
  unsigned long prev_micros = micros();

  long snap_L = 0, snap_R = 0;
  int stall_ticks_L = 0, stall_ticks_R = 0;
  float boost_L = 0.0f, boost_R = 0.0f;
  const int STALL_CONFIRM_TICKS = 2;
  const float BOOST_RATE = 15.0f;
  const float BOOST_MAX = 60.0f;
  const float BOOST_DECAY = 20.0f;
  noInterrupts();
  snap_L = left_counts;
  snap_R = right_counts;
  interrupts();

  while (true) {
    unsigned long now_micros = micros();
    if (now_micros - prev_micros >= 20000UL) {
      wdt_reset();
      float dt = (now_micros - prev_micros) / 1000000.0f;
      prev_micros = now_micros;
      updateStateEstimator(dt, true);

      if (millis() - turn_start_ms > MOTION_TIMEOUT_MS) {
        emergencyStop();
        return;
      }

      // higher floor speed & brakes earlier if heavier item detected (i.e. a water bottle from course)
      if (!detected_heavy && (boost_L > 20.0f || boost_R > 20.0f)) {
        detected_heavy = true;
        T_DECEL = 45.0f;
        current_min_speed = 10.0f;
        Serial.println("Heavy load detected - adjusting braking curve");
      }

      float error = goal_heading - current_heading;
      float abs_error = abs(error);

      if (abs_error <= TURN_THRESHOLD_DEG) {
        if (!settling) {
          settling = true;
          settled_since = millis();
          flushAllPIDs();
          boost_L = boost_R = 0.0f;
        }
        if (millis() - settled_since >= TURN_SETTLE_MS) {
          activeBrake();
          return;
        }
        current_turn_speed = 0.0f;
      } else {
        settling = false;
        float remaining_dist_cm = abs_error * (PI / 180.0f) * (WHEELBASE_CM / 2.0f);
        float safe_brake = sqrt(2.0f * T_DECEL * remaining_dist_cm);
        float desired_speed = max(current_min_speed, min((float)TURN_SPEED, safe_brake));
        if (current_turn_speed < desired_speed)
          current_turn_speed = min(current_turn_speed + T_ACCEL * dt, desired_speed);
        else
          current_turn_speed = max(current_turn_speed - T_DECEL * dt, desired_speed);
      }

      if (error > 0) {
        target_speed_L = -current_turn_speed;
        target_speed_R = current_turn_speed;
      } else {
        target_speed_L = current_turn_speed;
        target_speed_R = -current_turn_speed;
      }

      noInterrupts();
      long cur_L = left_counts;
      long cur_R = right_counts;
      interrupts();
      bool wantMove = (abs(target_speed_L) > 1.0f);

      if (wantMove && abs(cur_L - snap_L) < 5) {
        stall_ticks_L++;
        if (stall_ticks_L >= STALL_CONFIRM_TICKS) boost_L = min(boost_L + BOOST_RATE, BOOST_MAX);
      } else {
        stall_ticks_L = 0;
        boost_L = max(boost_L - BOOST_DECAY, 0.0f);
      }
      snap_L = cur_L;

      if (wantMove && abs(cur_R - snap_R) < 5) {
        stall_ticks_R++;
        if (stall_ticks_R >= STALL_CONFIRM_TICKS) boost_R = min(boost_R + BOOST_RATE, BOOST_MAX);
      } else {
        stall_ticks_R = 0;
        boost_R = max(boost_R - BOOST_DECAY, 0.0f);
      }
      snap_R = cur_R;

      double dir_L = (target_speed_L >= 0) ? 1.0 : -1.0;
      double dir_R = (target_speed_R >= 0) ? 1.0 : -1.0;
      leftVelPID.Compute();
      rightVelPID.Compute();
      double pwm_L = constrain(output_PWM_L + Kff_L * target_speed_L + dir_L * boost_L, -255.0, 255.0);
      double pwm_R = constrain(output_PWM_R + Kff_R * target_speed_R + dir_R * boost_R, -255.0, 255.0);
      driveMotor(1, pwm_L);
      driveMotor(2, pwm_R);
    }
  }
}

// estimates the state of robot at all times
void updateStateEstimator(float dt, bool is_turning) {
  noInterrupts();
  long cur_left = left_counts;
  long cur_right = right_counts;
  interrupts();

  long dL = cur_left - prev_left_counts;
  long dR = cur_right - prev_right_counts;
  prev_left_counts = cur_left;
  prev_right_counts = cur_right;

  float inst_speed_L = (dL * CM_PER_TICK) / dt;
  float inst_speed_R = (dR * CM_PER_TICK) / dt;

  filt_speed_L = (SPEED_ALPHA * inst_speed_L) + ((1.0f - SPEED_ALPHA) * filt_speed_L);
  filt_speed_R = (SPEED_ALPHA * inst_speed_R) + ((1.0f - SPEED_ALPHA) * filt_speed_R);

  actual_speed_L = filt_speed_L;
  actual_speed_R = filt_speed_R;

  float dist_L = dL * CM_PER_TICK;
  float dist_R = dR * CM_PER_TICK;
  distance_traveled += abs((dist_L + dist_R) / 2.0f);

  float delta_theta_odom_deg = ((dist_R - dist_L) / WHEELBASE_CM) * (180.0f / PI);
  raw_odometry_heading += delta_theta_odom_deg;

  float gyro_rate_dps = 0.0f;
  Wire.beginTransmission(0x68);
  Wire.write(0x47);
  if (Wire.endTransmission(false) == 0) {
    uint8_t bytes_received = Wire.requestFrom(0x68, 2, true);
    if (bytes_received >= 2) {
      int16_t gyro_z_raw = (Wire.read() << 8) | Wire.read();
      gyro_rate_dps = (gyro_z_raw - gyro_z_offset) / 129.3f;
      float gyro_deadzone = is_turning ? 0.15f : 0.5f;
      if (abs(gyro_rate_dps) < gyro_deadzone) gyro_rate_dps = 0.0f;
    }
  }
  current_heading += gyro_rate_dps * dt;
}

// hardware helpers
void activeBrake() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, 255);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMB, 255);
  smartDelay(80, true);
  digitalWrite(STBY, LOW);
}

void emergencyStop() {
  digitalWrite(STBY, LOW);
}

void driveMotor(int motor, double pwm) {
  int pwm_val = (int)abs(pwm);
  pwm_val = constrain(pwm_val, 0, 255);
  bool fwd = (pwm >= 0);
  if (motor == 1) {
    digitalWrite(AIN1, fwd ? LOW : HIGH);
    digitalWrite(AIN2, fwd ? HIGH : LOW);
    analogWrite(PWMA, pwm_val);
  } else {
    digitalWrite(BIN1, fwd ? LOW : HIGH);
    digitalWrite(BIN2, fwd ? HIGH : LOW);
    analogWrite(PWMB, pwm_val);
  }
}

void calibrateIMU() {
  wdt_disable();
  while (true) {
    Serial.println("Calibrating IMU... DO NOT MOVE.");
    delay(1000);

    long gyro_sum = 0;
    int16_t min_val = 32767;
    int16_t max_val = -32768;
    int valid_reads = 0;

    for (int i = 0; i < 200; i++) {
      Wire.beginTransmission(0x68);
      Wire.write(0x47);
      if (Wire.endTransmission(false) != 0) {
        delay(5);
        continue;
      }
      if (Wire.requestFrom(0x68, 2, true) < 2) {
        delay(5);
        continue;
      }

      int16_t val = (Wire.read() << 8) | Wire.read();
      gyro_sum += val;
      if (val < min_val) min_val = val;
      if (val > max_val) max_val = val;
      valid_reads++;
      delay(5);
    }

    if (valid_reads < 150) {
      Serial.println("IMU read failure during calibration. Retrying...");
      continue;
    }

    int32_t spread = (int32_t)max_val - (int32_t)min_val;
    if (spread < 130) {
      gyro_z_offset = gyro_sum / (float)valid_reads;
      Serial.println("IMU calibrated. Ready. Flip switch to start.");
      return;
    } else {
      Serial.print("Movement detected! Spread: ");
      Serial.println(spread);
    }
  }
}

void smartDelay(unsigned long ms, bool is_turning) {
  unsigned long start_millis = millis();
  unsigned long prev_micros = micros();

  while (millis() - start_millis < ms) {
    wdt_reset();
    unsigned long now_micros = micros();
    if (now_micros - prev_micros >= 20000UL) {
      float dt = (now_micros - prev_micros) / 1000000.0f;
      prev_micros = now_micros;
      updateStateEstimator(dt, is_turning);
    }
  }
}
bool checkFinishLine(float remaining_cm) {
  if (!isFinalLeg) return false;
  if (remaining_cm > FINISH_CHECK_CM) return false;

  int valL = analogRead(QTR_L);
  int valR = analogRead(QTR_R);

  bool hitL = abs(valL - qtr_baseline_L) > QTR_THRESHOLD;
  bool hitR = abs(valR - qtr_baseline_R) > QTR_THRESHOLD;

  if (hitL || hitR) {
    Serial.print("FINISH LINE DETECTED! L=");
    Serial.print(valL);
    Serial.print(" R=");
    Serial.println(valR);
    return true;
  }
  return false;
}
void calibrateQTR() {
  long sumL = 0, sumR = 0;
  for (int i = 0; i < 100; i++) {
    sumL += analogRead(QTR_L);
    sumR += analogRead(QTR_R);
    delay(5);
  }
  qtr_baseline_L = sumL / 100;
  qtr_baseline_R = sumR / 100;
  Serial.print("QTR baseline L: ");
  Serial.println(qtr_baseline_L);
  Serial.print("QTR baseline R: ");
  Serial.println(qtr_baseline_R);
}