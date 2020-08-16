//Based on https://github.com/m5stack/M5-ProductExampleCodes/tree/master/AtomBase/AtomFLY

//Press the Atom button, test the propeller rotates in turn, and the serial monitor outputs the IMU status

#include <M5Atom.h>
#include "AtomFly.h"

CRGB led(0, 0, 0);
CRGB HSVtoRGB(uint16_t h, uint16_t s, uint16_t v);

double pitch, roll;
double Calibrationbuff[3];
double r_rand = 180 / PI;

double last_pitch = 0, last_roll = 0;

uint8_t mode = 0;
uint32_t on_time, off_time = 0;

//PID calculations
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;

//PID gain and limit settings
float pid_p_gain_roll = 0.5;               //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.04;              //Gain setting for the roll I-controller
float pid_d_gain_roll = 10.0;              //Gain setting for the roll D-controller
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)


#define SERIAL_DEBUG 1

#if SERIAL_DEBUG
  #define SERIAL_PRINTF Serial.printf
#else
  #define SERIAL_PRINTF 
#endif

#define FLY_DURATION (10 * 1000)  // 10 secs

void setup()
{
    M5.begin(true, false, true);

    Calibrationbuff[0] = 0;
    Calibrationbuff[1] = 0;
    Calibrationbuff[2] = 0;

    fly.begin();
    if ( fly.initFly() != 0)
    {
        Serial.println("faild");
        led = CRGB(0,255,0);
        M5.dis.drawpix(0, led);
        while(1)
        {
            delay(100);
        }
    }

    Serial.println("OK");
    led = CRGB(255,0,0);
    M5.dis.drawpix(0, led);
    /*
    for (int i = 0; i < 360; i++)
    {
        led = HSVtoRGB(i, 100, 100);
        M5.dis.drawpix(0, led);

        if (i % 10 == 0)
        {
            fly.getAttitude(&pitch, &roll);
            Calibrationbuff[0] += pitch;
            Calibrationbuff[1] += roll;
        }
        delay(5);
    }
    */
    Calibrationbuff[0] = Calibrationbuff[0] / 36;
    Calibrationbuff[1] = Calibrationbuff[1] / 36;

    SERIAL_PRINTF("Calibration:%.2f,%.2f\n", Calibrationbuff[0], Calibrationbuff[1]);

    led = HSVtoRGB(0, 0, 100);
    M5.dis.drawpix(0, led);

    //PID reset
    pid_roll_setpoint = 0;
    pid_pitch_setpoint = 0;

    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    //pid_i_mem_yaw = 0;
    //pid_last_yaw_d_error = 0;    
}

void calculate_pid()
{
  //PID reset, temporary
  pid_roll_setpoint = 0;
  pid_pitch_setpoint = 0;

  pid_i_mem_roll = 0;
  pid_last_roll_d_error = 0;
  pid_i_mem_pitch = 0;
  pid_last_pitch_d_error = 0;

  //Roll calculations
  pid_error_temp = roll - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;

  if (pid_i_mem_roll > pid_max_roll)
    pid_i_mem_roll = pid_max_roll;
  else if (pid_i_mem_roll < -pid_max_roll)
    pid_i_mem_roll = -pid_max_roll;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if (pid_output_roll > pid_max_roll)
    pid_output_roll = pid_max_roll;
  else if (pid_output_roll < -pid_max_roll)
    pid_output_roll = -pid_max_roll;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = pitch - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if (pid_i_mem_pitch > pid_max_pitch)
    pid_i_mem_pitch = pid_max_pitch;
  else if (pid_i_mem_pitch < -pid_max_pitch)
    pid_i_mem_pitch = -pid_max_pitch;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if (pid_output_pitch > pid_max_pitch)
    pid_output_pitch = pid_max_pitch;
  else if (pid_output_pitch < -pid_max_pitch)
    pid_output_pitch = -pid_max_pitch;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
//   pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
//   pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
//   if (pid_i_mem_yaw > pid_max_yaw)
//     pid_i_mem_yaw = pid_max_yaw;
//   else if (pid_i_mem_yaw < -pid_max_yaw)
//     pid_i_mem_yaw = -pid_max_yaw;

//   pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
//   if (pid_output_yaw > pid_max_yaw)
//     pid_output_yaw = pid_max_yaw;
//   else if (pid_output_yaw < -pid_max_yaw)
//     pid_output_yaw = -pid_max_yaw;

//   pid_last_yaw_d_error = pid_error_temp;

  pid_output_yaw = 0;

  on_time = millis();
}

//kalman filter test, tried to see if it reduces noise
// problem is, it keeps values 10% around the initial value
//  check KALMAN_INIT comment below
//  to revisit in future
void loop2()
{
  //initial values for the kalman filter
  float x_est_last = 0;
  float P_last = 0;
  //the noise in the system
  float Q = 0.022;
  float R = 0.617;
  
  float K;
  float P;
  float P_temp;
  float x_temp_est;
  float x_est;
  float z_measured; //the 'noisy' value we measured
  float z_real = 0.5; //the ideal value we wish to measure
  
  float accX, accY, accZ;

  //initialize with a measurement
  fly.getAccelData(&accX, &accY, &accZ);
  x_est_last = z_real + accX * 0.09;      //KALMAN_INIT
  
  float sum_error_kalman = 0;
  float sum_error_measure = 0;
  
  for (int i=0;i<30;i++) {
      //do a prediction
      x_temp_est = x_est_last;
      P_temp = P_last + Q;

      //calculate the Kalman gain
      K = P_temp * (1.0/(P_temp + R));

      //measure
      fly.getAccelData(&accX, &accY, &accZ);

      z_measured = z_real + accX * 0.09; //the real measurement plus noise
      
      //correct
      x_est = x_temp_est + K * (z_measured - x_temp_est); 
      P = (1- K) * P_temp;
      
      //we have our new system
      
      printf("Ideal    position: %6.3f \n",z_real);
      printf("Mesaured position: %6.3f [diff:%.3f]\n",z_measured,fabs(z_real-z_measured));
      printf("Kalman   position: %6.3f [diff:%.3f]\n",x_est,fabs(z_real - x_est));
      
      sum_error_kalman += fabs(z_real - x_est);
      sum_error_measure += fabs(z_real-z_measured);
      
      //update our last's
      P_last = P;
      x_est_last = x_est;
  }
  
  printf("Total error if using raw measured:  %f\n",sum_error_measure);
  printf("Total error if using kalman filter: %f\n",sum_error_kalman);
  printf("Reduction in error: %d%% \n",100-(int)((sum_error_kalman/sum_error_measure)*100));

}

void loop()
{

#if 0 //average of 10 reads from accel
    #define ACC_AVG (10)
    double avgX = 0, avgY = 0, avgZ = 0;
    float accX, accY, accZ;

    for (int ix = 0; ix < ACC_AVG; ix++)
    {
      fly.getAccelData(&accX, &accY, &accZ);
      avgX += accX;
      avgY += accY;
      avgZ += accZ;
    }

    accX = avgX / ACC_AVG;
    accY = avgY / ACC_AVG;
    accZ = avgZ / ACC_AVG;
#endif

    float accX, accY, accZ;
    fly.getAccelData(&accX, &accY, &accZ);

    //SERIAL_PRINTF("t %d\t ax %.2f\t ay %.2f\t az %.2f\n", millis(), accX, accY, accZ);

    if ((accX < 1) && (accX > -1))
    {
        pitch = asin(-accX) * 57.295;
    }
    if (accZ != 0)
    {
        roll = atan(accY / accZ) * 57.295;
    }


#if 1  // a simple filter
    float alpha = 0.20;
    last_pitch = pitch = (alpha * pitch ) + (1 - alpha) * last_pitch;
    last_roll = roll = (alpha * roll ) + (1 - alpha) * last_roll;
#endif

    float arc = atan2(pitch, roll) * r_rand + 180;
    float dist = sqrt(pitch * pitch + roll * roll);

    calculate_pid();

    int throttle = 900;
    int esc_A = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW) //A
    int esc_B = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW) //B
    int esc_C = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW) //C
    int esc_D = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW) //D

    //scale down pwm values
    esc_A /= 10;
    esc_B /= 10;
    esc_C /= 10;
    esc_D /= 10;

    //SERIAL_PRINTF("p %.2f\t r %.2f\t a %.2f\t d %.2f\t", pitch, roll, arc, dist);
    //SERIAL_PRINTF("A %d  B %d  C %d  D %d", esc_A, esc_B, esc_C, esc_D);

    SERIAL_PRINTF("%.2f  r %.2f  ", pitch, roll);
    SERIAL_PRINTF("A %d  B %d  C %d  D %d\n", esc_A, esc_B, esc_C, esc_D);

    // if (mode == 1)
    //   SERIAL_PRINTF("\t on\n");
    // else
    //   SERIAL_PRINTF("\n");

    if (mode == 1 && millis() > off_time)
    {
        fly.WritePWM(AtomFly::kMotor_A, 0);
        fly.WritePWM(AtomFly::kMotor_B, 0);
        fly.WritePWM(AtomFly::kMotor_C, 0);
        fly.WritePWM(AtomFly::kMotor_D, 0);
        mode = 0;

        SERIAL_PRINTF("turn off\n");
    }

    if (mode == 1)
    {
        fly.WritePWM(AtomFly::kMotor_A, 90);  //esc_A
        fly.WritePWM(AtomFly::kMotor_B, 90);  //esc_B
        fly.WritePWM(AtomFly::kMotor_C, 90);  //esc_C
        fly.WritePWM(AtomFly::kMotor_D, 90);  //esc_D
    }
    else if (M5.Btn.wasPressed())
    {
        SERIAL_PRINTF("turn on\n");

        mode = 1;
        on_time = millis();
		    off_time = millis() + FLY_DURATION;
    }

#if 0 //update LED based on TOF
    //SERIAL_PRINTF("p %.2f\t r %.2f\t a %.2f\t v %.2f\n", pitch, roll, arc, val);

    // uint16_t tofData = fly.readTOF();
    // tofData = (tofData > 500) ? 500 : tofData;

    // float val = (dist * 6) > 100 ? 100 : dist * 6;

    // led = HSVtoRGB(arc, val, 100 - (tofData / 5));
    // M5.dis.drawpix(0, led);
#endif

    M5.update();

#if SERIAL_DEBUG
    //delay(10);
#endif
}

// R,G,B from 0-255, H from 0-360, S,V from 0-100

CRGB HSVtoRGB(uint16_t h, uint16_t s, uint16_t v)
{
    CRGB ReRGB(0, 0, 0);
    int i;
    float RGB_min, RGB_max;
    RGB_max = v * 2.55f;
    RGB_min = RGB_max * (100 - s) / 100.0f;

    i = h / 60;
    int difs = h % 60;
    float RGB_Adj = (RGB_max - RGB_min) * difs / 60.0f;

    switch (i)
    {
    case 0:

        ReRGB.r = RGB_max;
        ReRGB.g = RGB_min + RGB_Adj;
        ReRGB.b = RGB_min;
        break;
    case 1:
        ReRGB.r = RGB_max - RGB_Adj;
        ReRGB.g = RGB_max;
        ReRGB.b = RGB_min;
        break;
    case 2:
        ReRGB.r = RGB_min;
        ReRGB.g = RGB_max;
        ReRGB.b = RGB_min + RGB_Adj;
        break;
    case 3:
        ReRGB.r = RGB_min;
        ReRGB.g = RGB_max - RGB_Adj;
        ReRGB.b = RGB_max;
        break;
    case 4:
        ReRGB.r = RGB_min + RGB_Adj;
        ReRGB.g = RGB_min;
        ReRGB.b = RGB_max;
        break;
    default: // case 5:
        ReRGB.r = RGB_max;
        ReRGB.g = RGB_min;
        ReRGB.b = RGB_max - RGB_Adj;
        break;
    }

    return ReRGB;
}
