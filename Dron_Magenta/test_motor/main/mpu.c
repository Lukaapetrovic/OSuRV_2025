#include <stdio.h>
#include <math.h>

#include "driver/i2c.h"    //config, read, write
#include "driver/ledc.h"   //za mototre
#include "driver/gpio.h"   //GPIO_PULLUP_ENABLE

#include "freertos/FreeRTOS.h"  //makroi
#include "freertos/task.h"      //pravljenje taskova
#include "mpu.h"                //definicije pinova, adrese MPU6050    

uint32_t calculate_duty(int microseconds)
{
    return (uint32_t)((microseconds / 20000.0) * 8192.0);     //T = 1 / f -> T = 1 / 50 = 0,02s = 20000us (period)
                                            //PWM rezolucija 13bit
}
 
//inicijalizacija I2C konfiguracije
void i2c_init()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,                //inicira komunikaciju (slave salje podatke)
        .sda_io_num = I2C_MASTER_SDA_IO,        //SDA pin -data
        .scl_io_num = I2C_MASTER_SCL_IO,        //SCL pin -clock
        .sda_pullup_en = GPIO_PULLUP_ENABLE,    //ukljucenje otpornika koji drzi liniju na HIGH dok uredjaji ne spoje na GND da bi poslali 0 
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ  //brzina clock-a
    };

    i2c_param_config(I2C_MASTER_NUM, &conf);  //ucitava konfiguraciju u I2C kontroler  (hardver) koji koristim - I2C_NUM_0
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);      //drajver postaje aktivan
}                     //broj ,rezim rada, receive, transmit, interrupt

//pozivajne senzora, budjenje
void mpu6050_init()
{
    uint8_t data[2] = {0x6B, 0x00};   // podesavanje 0x6B registra- izadji iz sleep mode
    i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR, data, 2, pdMS_TO_TICKS(100));  //port, adresa uredjaja, bafer za niz koji saljem, velicina, cekanje taska da se zavrsi
}

int16_t make_int16(uint8_t high, uint8_t low)
{
    return (int16_t)((high << 8) | low);
}

void mpu6050_read(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t reg = 0x3B;    //pocetni citanje od registra ACCEL_XOUT_H.
    uint8_t data[14];

    i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_ADDR, &reg, 1, data, 14, pdMS_TO_TICKS(100));       //citamo 14 bajtova u bafer

    //data[0..5] accel, data[6..7] temp, data[8..13] gyro

    *ax = make_int16(data[0],  data[1]);
    *ay = make_int16(data[2],  data[3]);
    *az = make_int16(data[4],  data[5]);
    *gx = make_int16(data[8],  data[9]);
    *gy = make_int16(data[10], data[11]);
    *gz = make_int16(data[12], data[13]);
}

//PID KONTROLER
float pid_update(pid_controller_t *pid, float setpoint, float measured, float dt)
{ 
    float error = setpoint - measured;                    //stepen(greska) = koliko hocu - koliko je izmereno

    pid->integral += error * dt;                          //akumulacija greske- P ispod krive, drift, POTENCIJALNO UVESTI GRANICE (RESENO)
    if(pid->integral > 100.0f) pid->integral = 100.0f;
    if(pid->integral < -100.0f) pid->integral = -100.0f;
    float derivative = (error - pid->prev_error) / dt;   //koliko se brzo menja greska, stepen/s

    pid->prev_error = error;    //trenutna greska postaje prethodna za sledecu iteraciju

    //korekcija je zbir doprinosa sve 3 komponente
    float result= pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

    return result;
}


void flight_control_task(void *pvParameter)
{
    int16_t ax, ay, az, gx, gy, gz;         //sirovo iz senzora

    float roll = 0, pitch = 0;
    float alpha = 0.98f;         //double sporiji, drzi se float-a

    pid_controller_t pid_roll;          //rotacija levo-desno
    pid_roll.kp = 1.3f;
    pid_roll.ki = 0.0003f;
    pid_roll.kd = 0.3f;
    pid_roll.integral = 0;
    pid_roll.prev_error = 0;

    pid_controller_t pid_pitch;      //rotacija napred-nazad
    pid_pitch.kp = 1.3f;
    pid_pitch.ki = 0.0003f;
    pid_pitch.kd = 0.3f;
    pid_pitch.integral = 0;
    pid_pitch.prev_error = 0;

    float dt = 0.010f; //vreme izmedju merenja 1/100Hz

    TickType_t last_wake = xTaskGetTickCount();      //sitna vremenska jedinica, broj tikova od pocetka sistema
    //petlja radi na 4ms

    while(1)
    {
        //FAIL-SAFE
        if((xTaskGetTickCount() - last_packet_time) > pdMS_TO_TICKS(1000))
        {
            if(throttle > 1000)
            {
                printf("GUBITAK SIGNALA! Motori se gase.\n");
                throttle -= 20;  //Postepeno postavi throttle na minimum
                if(throttle < 1000)
                    throttle = 1000;
            }
        }

        mpu6050_read(&ax, &ay, &az, &gx, &gy, &gz);

        float accelX = ax / 16384.0f;
        float accelY = ay / 16384.0f;
        float accelZ = az / 16384.0f;

        float gyroX = gx / 131.0f;
        float gyroY = gy / 131.0f;

        float roll_acc = atan2f(accelY, accelZ) * 57.3f;   //radijani u stepene
        float pitch_acc = atan2f(-accelX, sqrtf(accelY*accelY + accelZ*accelZ)) * 57.3f;

        roll = alpha * (roll + gyroX * dt) + (1-alpha) * roll_acc;
        pitch = alpha * (pitch + gyroY * dt) + (1-alpha) * pitch_acc;

        float roll_corr = pid_update(&pid_roll, 0.0f, roll, dt);      //korekcija u zavisnosti od zeljene vrendnosti, izmerenog roll i proteklog vremena
        float pitch_corr = pid_update(&pid_pitch, 0.0f, pitch, dt);

        int base = throttle;  // iz WiFi-a

        int m1 = base + pitch_corr + roll_corr;
        int m2 = base - pitch_corr + roll_corr;
        int m3 = base - pitch_corr - roll_corr;
        int m4 = base + pitch_corr - roll_corr;

        if(m1 < 1000) 
            m1 = 1000;   
        if(m1 > 2000) 
            m1 = 2000;

        if(m2 < 1000) 
            m2 = 1000; 
        if(m2 > 2000) 
            m2 = 2000;

        if(m3 < 1000) 
            m3 = 1000;
        if(m3 > 2000) 
            m3 = 2000;

        if(m4 < 1000) 
            m4 = 1000;
        if(m4 > 2000) 
            m4 = 2000;

        ledc_set_duty(LEDC_LOW_SPEED_MODE, 0, calculate_duty(m1));       //nacin azuriranja hardvera, ESC signal spor pa nema poente HIGH SPEED
        ledc_set_duty(LEDC_LOW_SPEED_MODE, 1, calculate_duty(m2));
        ledc_set_duty(LEDC_LOW_SPEED_MODE, 2, calculate_duty(m3));
        ledc_set_duty(LEDC_LOW_SPEED_MODE, 3, calculate_duty(m4));

        for(int i=0;i<4;i++)
            ledc_update_duty(LEDC_LOW_SPEED_MODE, i);

        printf("roll: %.2f\n", roll);
        printf("pitch: %.2f\n", pitch);

        printf("roll: %.2f\n", roll_corr);
        printf("pitch: %.2f\n", pitch_corr);
        printf("------------------------------------\n");

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
    }
}