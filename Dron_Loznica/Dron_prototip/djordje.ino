

#include <MPU9250_WE.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <NetworkClient.h>
#include <WiFiAP.h>

#define MPU9250_ADDR 0x68
MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);
Servo esc1, esc2, esc3, esc4;


//prednji desni motor 17
//prednji levi motor 18
//zadnji desni motor 16
//zadnje levi motor 19

//************************************************

float pid_p_gain_roll = 0.30;               //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.0;              //Gain setting for the roll I-controller
float pid_d_gain_roll = 4;              //Gain setting for the roll D-controller
int pid_max_roll = 200;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 0.20;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.0;          //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 200;                     //Maximum output of the PID-controller (+/-)

float temp_pitch = 0, temp_yaw = 0, temp_roll = 0;

//**************************************


// --- WIFI ---
const char *ssid = "Dron_leti";
const char *password = "12345678";
NetworkServer server(80);
IPAddress local_IP(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

int escPin1 = 16, escPin2 = 17, escPin3 = 18, escPin4 = 19;
int motor1, motor2, motor3, motor4;
unsigned int throttle = 1000;
bool emergencyStop = false;



float pid_i_mem_roll = 0, pid_last_roll_d_error = 0;
float pid_i_mem_pitch = 0, pid_last_pitch_d_error = 0;
float pid_i_mem_yaw = 0, pid_last_yaw_d_error = 0;

double gyro_roll_read, gyro_pitch_read, gyro_yaw_read;

float gyro_roll_input, gyro_pitch_input, gyro_yaw_input;
float pid_roll_setpoint, pid_pitch_setpoint, pid_yaw_setpoint;
float pid_output_roll, pid_output_pitch, pid_output_yaw;
float yaw_apsolute = 0;


bool sensorReady = false;
float angle_pitch = 0, angle_roll = 0;
float angle_pitch_acc = 0, angle_roll_acc = 0;
float pitch_level_adjust = 0, roll_level_adjust = 0;
unsigned long loop_timer;
float pid_error_temp = 0;


//***************************************************************************
void setup() {

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  esc1.setPeriodHertz(100);
  esc2.setPeriodHertz(100);
  esc3.setPeriodHertz(100);
  esc4.setPeriodHertz(100);
  //prednji desni motor 17
  //prednji levi motor 18
  //zadnji desni motor 16
  //zadnje levi motor 19

  esc1.attach(escPin2, 1000, 2000);//prednji desni
  esc2.attach(escPin3, 1000, 2000);//prednji levi
  esc3.attach(escPin1, 1000, 2000);//zadnji desni
  esc4.attach(escPin4, 1000, 2000);//zadnji levi

  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);

  Serial.begin(115200);
  Wire.begin();
  delay(200);


  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(ssid, password);
  server.begin();

  myMPU9250.init();
  delay(1000);
  Serial.println("Senzor inicijalizovan.");


  Serial.println("Kalibracija...");
  delay(1000);
  myMPU9250.autoOffsets();

  // HARDVERSKI FILTER (Veoma vazno za vibracije)
  myMPU9250.enableGyrDLPF();
  myMPU9250.setGyrDLPF(MPU9250_DLPF_6); // Najaci filter u biblioteci

  Serial.println("Spremno!");
  loop_timer = micros();
}

//***********************************************************************************************
void loop() {
  NetworkClient client = server.accept();

  if (client) {
    String req = "";
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        req += c;
        if (c == '\n') {
          // --- KONTROLE ---
          if (req.indexOf("GET /STOP") >= 0) emergencyStop = true;
          if (req.indexOf("GET /RESET") >= 0) {
            emergencyStop = false; throttle = 1000;
            pid_i_mem_roll = 0; pid_last_roll_d_error = 0;
            pid_i_mem_pitch = 0; pid_last_pitch_d_error = 0;
            pid_i_mem_yaw = 0; pid_last_yaw_d_error = 0;
            temp_roll = 0; temp_pitch = 0; temp_yaw = 0;
            // Reset setpointova na 0

          }

          // --- GAS LOGIKA (+20 / -50) ---
          if (req.indexOf("GET /UP") >= 0) {
            throttle += 20;
            if (throttle > 1500) throttle = 1500;
          }
          if (req.indexOf("GET /DOWN") >= 0) {
            throttle -= 50;
            if (throttle < 1000) throttle = 1000;
          }

          // --- MENJANJE SETPOINTA ---
          // Roll (+/- 1)
          if (req.indexOf("GET /ROLP") >= 0) temp_roll += 5.0;
          if (req.indexOf("GET /ROLM") >= 0) temp_roll -= 5.0;

          // Pitch (+/- 1)
          if (req.indexOf("GET /PITP") >= 0) temp_pitch += 5.0;
          if (req.indexOf("GET /PITM") >= 0) temp_pitch -= 5.0;

          // Yaw (+/- 5)
          if (req.indexOf("GET /YAWP") >= 0) temp_yaw += 10.0;
          if (req.indexOf("GET /YAWM") >= 0) temp_yaw -= 10.0;

          // --- CUVANJE NOVOG PID-a (Sada prima 6 vrednosti) ---
          if (req.indexOf("GET /SETPID") >= 0) {
            int pr_pos = req.indexOf("pr=");
            int ir_pos = req.indexOf("&ir=");
            int dr_pos = req.indexOf("&dr=");
            int py_pos = req.indexOf("&py=");
            int iy_pos = req.indexOf("&iy=");
            int dy_pos = req.indexOf("&dy=");
            int space_pos = req.indexOf(" HTTP");

            if (pr_pos > 0 && dy_pos > 0) {
              pid_p_gain_roll = req.substring(pr_pos + 3, ir_pos).toFloat();
              pid_i_gain_roll = req.substring(ir_pos + 4, dr_pos).toFloat();
              pid_d_gain_roll = req.substring(dr_pos + 4, py_pos).toFloat();

              pid_p_gain_yaw = req.substring(py_pos + 4, iy_pos).toFloat();
              pid_i_gain_yaw = req.substring(iy_pos + 4, dy_pos).toFloat();
              pid_d_gain_yaw = req.substring(dy_pos + 4, space_pos).toFloat();

              pid_p_gain_pitch = pid_p_gain_roll;
              pid_i_gain_pitch = pid_i_gain_roll;
              pid_d_gain_pitch = pid_d_gain_roll;

              Serial.println("NOVI PID SNIMLJEN (UKLJUCUJUCI YAW)!");
            }
          }

          // --- SLANJE PODATAKA (JSON) ---
          if (req.indexOf("GET /data") >= 0) {
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type: application/json");
            client.println();

            // Dodati setpointi u slanje podataka
            client.print("{\"r\":" + String(angle_roll) +
                         ",\"p\":" + String(angle_pitch) +
                         ",\"y\":" + String(yaw_apsolute) +
                         ",\"t\":" + String(throttle) +
                         ",\"spr\":" + String(temp_roll) +
                         ",\"spp\":" + String(temp_pitch) +
                         ",\"spy\":" + String(temp_yaw) +
                         ",\"lt\":" + String(loop_timer) + "}");
            break;
          }

          // --- PRIKAZ STRANICE (HTML) ---
          client.println("HTTP/1.1 200 OK");
          client.println("Content-type:text/html");
          client.println();

          client.print("<html><head><meta name='viewport' content='width=device-width, initial-scale=1.0'>");
          client.print("<style>body{font-family:sans-serif;text-align:center;background:#222;color:#fff;padding:10px;}");
          client.print("button{width:100px;height:50px;margin:5px;font-size:16px;cursor:pointer;}");
          client.print(".spbtn{width:50px;height:40px;}"); // Manje dugme za setpoint
          client.print("input{width:60px;height:35px;font-size:16px;text-align:center;margin:5px;}");
          client.print(".stop{background:red;width:100%;font-weight:bold;color:white;}</style></head><body>");

          client.print("<h1>YMFC TUNING</h1>");
          client.print("<button class='stop' onclick=\"fetch('/STOP')\">STOP</button><br>");
          client.print("<button onclick=\"fetch('/UP')\">GAS +20</button>");
          client.print("<button onclick=\"fetch('/DOWN')\">GAS -50</button><hr>");

          client.print("<h2>Gas: <span id='t'>1000</span></h2>");
          client.print("<p>Roll: <span id='r'>0</span> Pitch: <span id='p'>0</span> Yaw: <span id='y'>0</span></p>");

          // --- HTML ZA SETPOINT UNOS ---
          client.print("<hr><h3>Setpointi (Zeljene Vrednosti)</h3>");
          client.print("Roll SP: <span id='spr' style='color:yellow;'>0</span> <button class='spbtn' onclick=\"fetch('/ROLM')\">-1</button><button class='spbtn' onclick=\"fetch('/ROLP')\">+1</button><br>");
          client.print("Pitch SP: <span id='spp' style='color:yellow;'>0</span> <button class='spbtn' onclick=\"fetch('/PITM')\">-1</button><button class='spbtn' onclick=\"fetch('/PITP')\">+1</button><br>");
          client.print("Yaw SP: <span id='spy' style='color:yellow;'>0</span> <button class='spbtn' onclick=\"fetch('/YAWM')\">-5</button><button class='spbtn' onclick=\"fetch('/YAWP')\">+5</button><br><hr>");

          // --- HTML ZA PID UNOS ---
          client.print("<h3>Roll/Pitch PID</h3>");
          client.print("P: <input type='number' step='0.01' id='valPr' value='" + String(pid_p_gain_roll) + "'> ");
          client.print("I: <input type='number' step='0.001' id='valIr' value='" + String(pid_i_gain_roll) + "'> ");
          client.print("D: <input type='number' step='0.1' id='valDr' value='" + String(pid_d_gain_roll) + "'><br>");

          client.print("<h3>Yaw PID</h3>");
          client.print("P: <input type='number' step='0.1' id='valPy' value='" + String(pid_p_gain_yaw) + "'> ");
          client.print("I: <input type='number' step='0.01' id='valIy' value='" + String(pid_i_gain_yaw) + "'> ");
          client.print("D: <input type='number' step='0.1' id='valDy' value='" + String(pid_d_gain_yaw) + "'><br><br>");

          client.print("<button style='width:250px;background:green;color:white;font-weight:bold;' onclick=\"sendPID()\">SNIMI SVE PID-ove</button><hr>");

          client.print("<p>Loop Timer: <span id='lt'>0</span> us</p>");

          // --- JAVASCRIPT ---
          client.print("<script>");

          client.print("function sendPID() {");
          client.print("  let pr = document.getElementById('valPr').value;");
          client.print("  let ir = document.getElementById('valIr').value;");
          client.print("  let dr = document.getElementById('valDr').value;");
          client.print("  let py = document.getElementById('valPy').value;");
          client.print("  let iy = document.getElementById('valIy').value;");
          client.print("  let dy = document.getElementById('valDy').value;");
          client.print("  fetch('/SETPID?pr=' + pr + '&ir=' + ir + '&dr=' + dr + '&py=' + py + '&iy=' + iy + '&dy=' + dy);");
          client.print("  alert('PID azuriran!');");
          client.print("}");

          client.print("setInterval(function(){fetch('/data').then(r=>r.json()).then(d=>{");
          client.print("document.getElementById('r').innerText=d.r.toFixed(1);");
          client.print("document.getElementById('p').innerText=d.p.toFixed(1);");
          client.print("document.getElementById('y').innerText=d.y;");
          client.print("document.getElementById('t').innerText=d.t;");

          // Update setpointova na ekranu
          client.print("document.getElementById('spr').innerText=d.spr.toFixed(1);");
          client.print("document.getElementById('spp').innerText=d.spp.toFixed(1);");
          client.print("document.getElementById('spy').innerText=d.spy.toFixed(1);");

          client.print("document.getElementById('lt').innerText=d.lt;");
          client.print("});}, 200);</script></body></html>");

          break;
        }
      }
    }
    client.stop();
  }
  if (emergencyStop == true) {
    temp_roll = 0; temp_pitch = 0; //temp_yaw=0;
    throttle=0;
  }

  xyzFloat angles = myMPU9250.getAngles();
  xyzFloat gyr = myMPU9250.getGyrValues();
  gyro_roll_read = gyr.y;
  gyro_pitch_read = gyr.x;
  gyro_yaw_read = gyr.z;
  //low pass filter za ziroskop(svaki put menjamo za samo 30 posto zbog vibracija)

  gyro_roll_input = (gyro_roll_input * 0.7) + (gyro_roll_read * 0.3);
  gyro_pitch_input = (gyro_pitch_input * 0.7) + (gyro_pitch_read * 0.3);
  gyro_yaw_input = (gyro_yaw_input * 0.7) + (gyro_yaw_read * 0.3);

  yaw_apsolute += gyro_yaw_input * 0.01;

  /*roll = roll * 0.9 + angles.x * 0.1;
    pitch = pitch * 0.9 + angles.y * 0.1;*/
  //angle i pitch u stepenima
  angle_pitch += gyro_pitch_read * 0.01;
  angle_roll  += gyro_roll_read * 0.01;


  angle_pitch -= angle_roll * sin(gyr.z * 0.0001745329); //koristimo za apdejtovanje pitch i roll
  angle_roll += angle_pitch * sin(gyr.z * 0.0001745329); //prilikom okretanja

  //preuzimanje vrednosti akcelerometra
  angle_pitch_acc = angles.x;
  angle_roll_acc = angles.y;

  //ispravljanje drift-a
  angle_pitch = angle_pitch * 0.996 + angle_pitch_acc * 0.004;
  angle_roll = angle_roll * 0.996 + angle_roll_acc * 0.004;

  // brzina vracanja drona u ravnotezni polozaj

  //***********
  //konstanta
  //***********
  pid_roll_setpoint = temp_roll;
  pid_pitch_setpoint = temp_pitch;

  pitch_level_adjust = angle_pitch * 5;
  roll_level_adjust = angle_roll * 5;






  pid_roll_setpoint -= roll_level_adjust;
  pid_pitch_setpoint -= pitch_level_adjust;

  //***********
  //konstanta
  //***********
  pid_roll_setpoint /= 1.25;
  pid_pitch_setpoint /= 1.25;


  pid_yaw_setpoint = temp_yaw;

  calculate_pid();

  int esc_1_value = throttle - pid_output_pitch + pid_output_roll + pid_output_yaw;  // Prednji desni
  int esc_2_value = throttle - pid_output_pitch - pid_output_roll - pid_output_yaw;  // Prednji levi
  int esc_3_value = throttle + pid_output_pitch + pid_output_roll - pid_output_yaw;  // Zadnji desni
  int esc_4_value = throttle + pid_output_pitch - pid_output_roll + pid_output_yaw;  // Zadnji levi

  //prednji desni

  // Onda osiguraš da je između 1000 i 1500
  esc_1_value = constrain(esc_1_value, 1000, 1500);
  // I tek onda šalješ motoru
  esc1.writeMicroseconds(esc_1_value);

  //prednji levi

  // Onda osiguraš da je između 1000 i 150
  esc_2_value = constrain(esc_2_value, 1000, 1500);
  // I tek onda šalješ motoru
  esc2.writeMicroseconds(esc_2_value);

  //zadnj desni

  // Onda osiguraš da je između 1000 i 1500
  esc_3_value = constrain(esc_3_value, 1000, 1500);
  // I tek onda šalješ motoru
  esc3.writeMicroseconds(esc_3_value);

  //zadnji levi

  // Onda osiguraš da je između 1000 i 1500
  esc_4_value = constrain(esc_4_value, 1000, 1500);
  // I tek onda šalješ motoru
  esc4.writeMicroseconds(esc_4_value);


  while (micros() - loop_timer < 10000);                                     //We wait until 10000us=10ms are passed.
  loop_timer = micros();                                                    //Set the timer for the next loop.

}



//**************************************************************************

void calculate_pid() {
  //---------------------ROLL--------------------------
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;

  if (pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if (pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;
  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);

  if (pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if (pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;
  //---------------------PITCH-------------------------
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;

  if (pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if (pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if (pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if (pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //---------------------YAW---------------------------
  pid_error_temp = yaw_apsolute - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if (pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if (pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if (pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if (pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;

}
