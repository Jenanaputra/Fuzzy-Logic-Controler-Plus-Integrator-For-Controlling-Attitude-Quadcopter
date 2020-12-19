#include <Wire.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
SoftwareSerial blueSerial(0,1);
float pid_i_gain_roll = 0.01;
int out_max_roll = 350;
float pid_i_gain_pitch = pid_i_gain_roll;
int out_max_pitch = out_max_roll;
/*float pid_p_gain_yaw = 4.0;
float pid_i_gain_yaw = 0.02;
float pid_d_gain_yaw = 0.0;
int pid_max_yaw = 400; */
boolean auto_level = true;
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
byte eeprom_data[36];
byte highByte, lowByte;
volatile
int
receiver_input_channel_1,
receiver_input_channel_2,
receiver_input_channel_3, receiver_input_channel_4;
int
counter_channel_1,
counter_channel_2,
counter_channel_3,
counter_channel_4, loop_counter;
int esc_1, esc_2, esc_3, esc_4;
int throttle, battery_voltage;
int cal_int, start, gyro_address;
68int receiver_input[5];
int temperature;
int acc_axis[4], gyro_axis[4];
float roll_level_adjust, pitch_level_adjust;
long acc_x, acc_y, acc_z, acc_total_vector;
unsigned
long
timer_channel_1,
timer_channel_2,
timer_channel_3,
timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
unsigned long loop_timer;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_axis_cal[4];
float pid_error_temp,x;
float
pid_i_mem_roll,
pid_roll_setpoint,
gyro_roll_input,
pid_output_roll,
pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch,
pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw,
pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
boolean gyro_angles_set;
float angle_roll1,angle_pitch1;
float dataString1;
const byte interruptPin1 = 2;
const byte interruptPin2 = 3;
const byte interruptPin3 = 18;int ez,en,ep,enb,epb,elz,eln,elnb,elp,elpb;
double uez,uen,uenb,uep,uepb,uelz,ueln,uelnb,uelp,uelpb;
double utotal,OP;
int Z,N,NB,P,PB;
void setup(){
//Serial.begin(57600);
blueSerial.begin(115200);
for(start = 0; start <= 35; start++)eeprom_data[start] = EEPROM.read(start);
start = 0;
gyro_address = eeprom_data[32];
Wire.begin();
TWBR = 12;
//Set the I2C clock speed to
400kHz.
DDRA |= B11110000;
DDRB |= B01000000;
while(eeprom_data[33] != 'J' || eeprom_data[34] != 'M' || eeprom_data[35] !=
'B')delay(10);
if(eeprom_data[31] == 2 || eeprom_data[31] == 3)delay(10);
set_gyro_registers();
for (cal_int = 0; cal_int < 1250 ; cal_int ++){
PORTA |= B11110000;
delayMicroseconds(1000);
PORTA &= B00001111;
delayMicroseconds(3000);
}for (cal_int = 0; cal_int < 2000 ; cal_int ++){
if(cal_int % 15 == 0);
gyro_signalen();
gyro_axis_cal[1] += gyro_axis[1];
gyro_axis_cal[2] += gyro_axis[2];
gyro_axis_cal[3] += gyro_axis[3];
PORTA |= B11110000;
delayMicroseconds(1000);
PORTA &= B00001111;
delay(3);
}
gyro_axis_cal[1] /= 2000;
gyro_axis_cal[2] /= 2000;
gyro_axis_cal[3] /= 2000;
pinMode(interruptPin1, INPUT);
pinMode(interruptPin2, INPUT);
pinMode(interruptPin3, INPUT);
attachInterrupt(digitalPinToInterrupt(interruptPin1), blink, CHANGE);
attachInterrupt(digitalPinToInterrupt(interruptPin2), blink, CHANGE);
attachInterrupt(digitalPinToInterrupt(interruptPin3), blink, CHANGE);
receiver_input_channel_3 = convert_receiver_channel(3);receiver_input_channel_1 = convert_receiver_channel(1);
loop_timer = micros();
uez=0.00;
uen=0.00;
uep=0.00;
uenb=0.00;
uepb=0.00;
uelz=0.00;
ueln=0.00;
uelnb=0.00;
uelp=0.00;
uelpb=0.00;
OP=0.00;
utotal=0.0;
Z=0;
N=-115;
NB=-215;
P=115;
PB=215;
}
void loop(){//65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
gyro_roll_input = (gyro_roll_input * 0.7) + ((gyro_roll / 65.5) * 0.3); //Gyro pid
input is deg/sec.
gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_pitch / 65.5) * 0.3);//Gyro
pid input is deg/sec.
gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyro_yaw / 65.5) * 0.3);
//Gyro
pid input is deg/sec.
//0.0000611 = 1 / (250Hz / 65.5)
angle_pitch += gyro_pitch * 0.0000611;
angle_roll += gyro_roll * 0.0000611;
//0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is
in radians
angle_pitch -= angle_roll * sin(gyro_yaw * 0.000001066);
angle_roll += angle_pitch * sin(gyro_yaw * 0.000001066);
//Accelerometer angle calculations
acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));
if(abs(acc_y) < acc_total_vector){
angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;
}
if(abs(acc_x) < acc_total_vector){
angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;
}angle_pitch_acc -= 0.0;
angle_roll_acc -= 0.0;
angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;
angle_roll1=angle_roll-1.5;
angle_pitch1=angle_pitch;
dataString1 = angle_roll1;
blueSerial.println(dataString1,1);
pitch_level_adjust = angle_pitch1 * 15;
roll_level_adjust = angle_roll1 * 15;
if(!auto_level){
pitch_level_adjust = 0;
roll_level_adjust = 0;
}
if(receiver_input_channel_3 < 1080){
start = 2;
angle_pitch = angle_pitch_acc;
angle_roll = angle_roll_acc;gyro_angles_set = true;
pid_i_mem_roll = 0;
pid_last_roll_d_error = 0;
pid_i_mem_pitch = 0;
pid_last_pitch_d_error = 0;
pid_i_mem_yaw = 0;
pid_last_yaw_d_error = 0;
}
pid_roll_setpoint = 0;
if(receiver_input_channel_1 > 1506)pid_roll_setpoint = receiver_input_channel_1
- 1510;
else
if(receiver_input_channel_1
<
1486)pid_roll_setpoint
=
receiver_input_channel_1 - 1490;
pid_roll_setpoint -= roll_level_adjust;
pid_roll_setpoint /= 3.0;
pid_pitch_setpoint = 0;
if(receiver_input_channel_2
>
1510)pid_pitch_setpoint
=receiver_input_channel_2-1510 ;
else
if(receiver_input_channel_2
=receiver_input_channel_2-1490 ;
pid_pitch_setpoint -= pitch_level_adjust;
<
1490)pid_pitch_setpointpid_pitch_setpoint /= 3.0;
pid_yaw_setpoint = 0;
calculate_pid();
throttle = receiver_input_channel_3;
//Serial.println(throttle);
if (start == 2){
if (throttle > 1800) throttle = 1800;
esc_1 = throttle + pid_output_roll+ pid_output_pitch + pid_output_yaw;
esc_2 = throttle - pid_output_roll + pid_output_pitch - pid_output_yaw;
esc_3 = throttle - pid_output_roll- pid_output_pitch + pid_output_yaw;
esc_4 = throttle + pid_output_roll - pid_output_pitch - pid_output_yaw;
if (esc_1 < 1100) esc_1 = 1100;
if (esc_2 < 1100) esc_2 = 1100;
if (esc_3 < 1100) esc_3 = 1100;
if (esc_4 < 1100) esc_4 = 1100;
if(esc_1 > 2000)esc_1 = 2000;
if(esc_2 > 2000)esc_2 = 2000;
if(esc_3 > 2000)esc_3 = 2000;
if(esc_4 > 2000)esc_4 = 2000;
}
else{
esc_1 = 1000;esc_2 = 1000;
esc_3 = 1000;
esc_4 = 1000;
}
if(micros() - loop_timer > 4050)//digitalWrite(12, HIGH);
while(micros() - loop_timer < 4000);
loop_timer = micros();
PORTA |= B11110000;
timer_channel_1 = esc_1 + loop_timer;
timer_channel_2 = esc_2 + loop_timer;
timer_channel_3 = esc_3 + loop_timer;
timer_channel_4 = esc_4 + loop_timer;
gyro_signalen();
while(PORTA >= 16){
esc_loop_timer = micros();
if(timer_channel_1 <= esc_loop_timer)PORTA &= B11101111;
if(timer_channel_2 <= esc_loop_timer)PORTA &= B11011111;
if(timer_channel_3 <= esc_loop_timer)PORTA &= B10111111;
if(timer_channel_4 <= esc_loop_timer)PORTA &= B01111111;
}
}
void blink() {
current_time = micros();
//Channel 1=========================================if(PINE & B00010000){
if(last_channel_1 == 0){
last_channel_1 = 1;
timer_1 = current_time;
}
}
else if(last_channel_1 == 1){
last_channel_1 = 0;
receiver_input[1] = current_time - timer_1;
}
//Channel 2=========================================
if(PINE & B00100000 ){
if(last_channel_2 == 0){
last_channel_2 = 1;
timer_2 = current_time;
}
}
else if(last_channel_2 == 1){
last_channel_2 = 0;
receiver_input[2] = current_time - timer_2;
}
//Channel 3=========================================
if(PIND & B0001000 ){
if(last_channel_3 == 0){
last_channel_3 = 1;timer_3 = current_time;
}
}
else if(last_channel_3 == 1){
last_channel_3 = 0;
receiver_input[3] = current_time - timer_3;
}
}
void gyro_signalen(){
//Read the MPU-6050
if(eeprom_data[31] == 1){
Wire.beginTransmission(gyro_address);
Wire.write(0x3B);
Wire.endTransmission();
Wire.requestFrom(gyro_address,14);
receiver_input_channel_1 = convert_receiver_channel(1);
receiver_input_channel_2 = convert_receiver_channel(2);
receiver_input_channel_3 = convert_receiver_channel(3);
receiver_input_channel_4 = convert_receiver_channel(4);
while(Wire.available() < 14);
acc_axis[1] = Wire.read()<<8|Wire.read();
acc_axis[2] = Wire.read()<<8|Wire.read();
acc_axis[3] = Wire.read()<<8|Wire.read();temperature = Wire.read()<<8|Wire.read();
gyro_axis[1] = Wire.read()<<8|Wire.read();
gyro_axis[2] = Wire.read()<<8|Wire.read();
gyro_axis[3] = Wire.read()<<8|Wire.read();
}
if(cal_int == 2000){
gyro_axis[1] -= gyro_axis_cal[1];
gyro_axis[2] -= gyro_axis_cal[2];
gyro_axis[3] -= gyro_axis_cal[3];
}
gyro_roll = gyro_axis[eeprom_data[28] & 0b00000011];
if(eeprom_data[28] & 0b10000000)gyro_roll *= -1;
gyro_pitch = gyro_axis[eeprom_data[29] & 0b00000011];
if(eeprom_data[29] & 0b10000000)gyro_pitch *= -1;
gyro_yaw = gyro_axis[eeprom_data[30] & 0b00000011];
if(eeprom_data[30] & 0b10000000)gyro_yaw *= -1;
acc_x = acc_axis[eeprom_data[29] & 0b00000011];
if(eeprom_data[29] & 0b10000000)acc_x *= -1;
acc_y = acc_axis[eeprom_data[28] & 0b00000011];
if(eeprom_data[28] & 0b10000000)acc_y *= -1;
acc_z = acc_axis[eeprom_data[30] & 0b00000011];
if(eeprom_data[30] & 0b10000000)acc_z *= -1;
}void calculate_pid(){
pid_error_temp = pid_roll_setpoint-gyro_roll_input;
if( (pid_error_temp) <=-78.5){
enb=1;
if((pid_error_temp<-157))uenb=1; else
if ((pid_error_temp>=-157 && pid_error_temp<=-78.5))
uenb=(-0.01274)*(pid_error_temp+78.5);
}
if ((pid_error_temp>=-157) &&(pid_error_temp<=0) ){
en=1;
if((pid_error_temp>=-157)
&&
(pid_error_temp<=-
60))uen=(0.01031)*(pid_error_temp+157);
else
if
((pid_error_temp)>-60
&&
(pid_error_temp)<=0)uen=(-
0.0167)*(pid_error_temp);
}
if ((pid_error_temp>=-78.5) &&(pid_error_temp<=78.5) ){
ez=1;
if((pid_error_temp>=-78.5)
&&
(pid_error_temp<=0))uez=(0.01274)*(pid_error_temp+78.5);
else
if
((pid_error_temp)>0
0.01274)*(pid_error_temp-78.5);
}
&&
(pid_error_temp)<=78.5)uez=(-if ((pid_error_temp>=0) &&(pid_error_temp<=157) ){
ep=1;
if((pid_error_temp>=0)
&&
(pid_error_temp<=60))uep=(0.0167)*(pid_error_temp);
else
if
((pid_error_temp)>60
&&
(pid_error_temp)<=157)uep=(-
0.01031)*(pid_error_temp-157);
}
if (pid_error_temp>=78.5){
epb=1;
if((pid_error_temp>=78.5)
(pid_error_temp<157))uepb=(0.01274)*(pid_error_temp-78.5);
else if ((pid_error_temp)>=157)uepb=1;
}
x=pid_error_temp-pid_last_roll_d_error;
if( (x) <=-16){
elnb=1;
if((x<-32))uelnb=1; else
if ((x>=-32 && x<=-16))
uelnb=(-0.0625)*(x+16);
}
if ((x>=-32) &&(x<=0) ){
&&eln=1;
if((x>=-32) && (x<=-10))ueln=(0.0455)*(x+32);
else if ((x)>-10 && (x)<=0)ueln=(-0.1)*(x);
}
if ((x>=-16) &&(x<=16) ){
elz=1;
if((x>=-16) && (x<=0))uelz=(0.0625)*(x+16);
else if ((x)>0 && (x)<=16)uelz=(-0.0625)*(x-16);
}
if ((x>=0) &&(x<=32) ){
elp=1;
if((x>=0) && (x<=10))uelp=(0.1)*(x);
else if ((x)>10 && (x)<=32)uelp=(-0.0455)*(x-32);
}
if (x>=16){
elpb=1;
if((x>=16) && (x<32))uelpb=(0.0625)*(x-16);
else if ((x)>=32)uelpb=1;
}
if(enb==1 && elnb==1){
OP+=min(uenb,uelnb)*NB;
utotal+=min(uenb,uelnb);}
if(enb==1 && eln==1){
OP+=min(uenb,ueln)*NB;
utotal+=min(uenb,ueln);
}
if(enb==1 && elz==1){
OP+=min(uenb,uelz)*NB;
utotal+=min(uenb,uelz);
}
if(enb==1 && elp==1){
OP+=min(uenb,uelp)*P;
utotal+=min(uenb,uelp);
}
if(enb==1 && elpb==1){
OP+=min(uenb,uelpb)*PB;
utotal+=min(uenb,uelpb);
}
if(en==1 && elnb==1){
OP+=min(uen,uelnb)*NB;
utotal+=min(uen,uelnb);
}
if(en==1 && eln==1){
OP+=min(uen,ueln)*N;
utotal+=min(uen,ueln);
}if(en==1 && elz==1){
OP+=min(uen,uelz)*N;
utotal+=min(uen,uelz);
}
if(en==1 && elp==1){
OP+=min(uen,uelp)*P;
utotal+=min(uen,uelp);
}
if(en==1 && elpb==1){
OP+=min(uen,uelpb)*PB;
utotal+=min(uen,uelpb);
}
if(ez==1 && elnb==1){
OP+=min(uez,uelnb)*NB;
utotal+=min(uez,uelnb);
}
if(ez==1 && eln==1){
OP+=min(uez,ueln)*N;
utotal+=min(uez,ueln);
}
if(ez==1 && elz==1){
OP+=min(uez,uelz)*Z;
utotal+=min(uez,uelz);
}
if(ez==1 && elp==1){OP+=min(uez,uelp)*P;
utotal+=min(uez,uelp);
}
if(ez==1 && elpb==1){
OP+=min(uez,uelpb)*PB;
utotal+=min(uez,uelpb);
}
if(ep==1 && elnb==1){
OP+=min(uep,uelnb)*NB;
utotal+=min(uep,uelnb);
}
if(ep==1 && eln==1){
OP+=min(uep,ueln)*N;
utotal+=min(uep,ueln);
}
if(ep==1 && elz==1){
OP+=min(uep,uelz)*P;
utotal+=min(uep,uelz);
}
if(ep==1 && elp==1){
OP+=min(uep,uelp)*P;
utotal+=min(uep,uelp);
}
if(ep==1 && elpb==1){
OP+=min(uep,uelpb)*PB;utotal+=min(uep,uelpb);
}
if(epb==1 && elnb==1){
OP+=min(uepb,uelnb)*NB;
utotal+=min(uepb,uelnb);
}
if(epb==1 && eln==1){
OP+=min(uepb,ueln)*NB;
utotal+=min(uepb,ueln);
}
if(epb==1 && elz==1){
OP+=min(uepb,uelz)*PB;
utotal+=min(uepb,uelz);
}
if(epb==1 && elp==1){
OP+=min(uepb,uelp)*PB;
utotal+=min(uepb,uelp);
}
if(epb==1 && elpb==1){
OP+=min(uepb,uelpb)*PB;
utotal+=min(uepb,uelpb);
}
pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
if(pid_i_mem_roll > out_max_roll)pid_i_mem_roll = out_max_roll;else if(pid_i_mem_roll < out_max_roll * -1)pid_i_mem_roll = out_max_roll * -
1;
pid_output_roll=((OP/utotal)+pid_i_mem_roll);
if (pid_output_roll > out_max_roll)pid_output_roll=out_max_roll;
if (pid_output_roll < out_max_roll*-1)pid_output_roll=out_max_roll*-1;
pid_last_roll_d_error = pid_error_temp;
uez=0.00;
uen=0.00;
uenb=0.00;
uep=0.00;
uepb=0.00;
uelz=0.00;
ueln=0.00;
uelnb=0.00;
uelpb=0.00;
uelp=0.00;
OP=0.00;
utotal=0.00;
//..........................
pid_error_temp =pid_pitch_setpoint - gyro_pitch_input;
if( (pid_error_temp) <=-78.5){enb=1;
if((pid_error_temp<-157))uenb=1; else
if ((pid_error_temp>=-157 && pid_error_temp<=-78.5))
uenb=(-0.01274)*(pid_error_temp+78.5);
}
if ((pid_error_temp>=-157) &&(pid_error_temp<=0) ){
en=1;
if((pid_error_temp>=-157)
&&
(pid_error_temp<=-
60))uen=(0.01031)*(pid_error_temp+157);
else
if
((pid_error_temp)>-60
&&
(pid_error_temp)<=0)uen=(-
0.0167)*(pid_error_temp);
}
if ((pid_error_temp>=-78.5) &&(pid_error_temp<=78.5) ){
ez=1;
if((pid_error_temp>=-78.5)
&&
(pid_error_temp<=0))uez=(0.01274)*(pid_error_temp+78.5);
else
if
((pid_error_temp)>0
&&
(pid_error_temp)<=78.5)uez=(-
0.01274)*(pid_error_temp-78.5);
}
if ((pid_error_temp>=0) &&(pid_error_temp<=157) ){
ep=1;
if((pid_error_temp>=0)
(pid_error_temp<=60))uep=(0.0167)*(pid_error_temp);
&&else
if
((pid_error_temp)>60
&&
(pid_error_temp)<=157)uep=(-
0.01031)*(pid_error_temp-157);
}
if (pid_error_temp>=78.5){
epb=1;
if((pid_error_temp>=78.5)
(pid_error_temp<157))uepb=(0.01274)*(pid_error_temp-78.5);
else if ((pid_error_temp)>=157)uepb=1;
}
x=pid_error_temp-pid_last_pitch_d_error;
if( (x) <=-16){
elnb=1;
if((x<-32))uelnb=1; else
if ((x>=-32 && x<=-16))
uelnb=(-0.0625)*(x+16);
}
if ((x>=-32) &&(x<=0) ){
eln=1;
if((x>=-32) && (x<=-10))ueln=(0.0455)*(x+32);
else if ((x)>-10 && (x)<=0)ueln=(-0.1)*(x);
}
&&if ((x>=-16) &&(x<=16) ){
elz=1;
if((x>=-16) && (x<=0))uelz=(0.0625)*(x+16);
else if ((x)>0 && (x)<=16)uelz=(-0.0625)*(x-16);
}
if ((x>=0) &&(x<=32) ){
elp=1;
if((x>=0) && (x<=10))uelp=(0.1)*(x);
else if ((x)>10 && (x)<=32)uelp=(-0.0455)*(x-32);
}
if (x>=16){
elpb=1;
if((x>=16) && (x<32))uelpb=(0.0625)*(x-16);
else if ((x)>=32)uelpb=1;
}
if(enb==1 && elnb==1){
OP+=min(uenb,uelnb)*NB;
utotal+=min(uenb,uelnb);
}
if(enb==1 && eln==1){OP+=min(uenb,ueln)*NB;
utotal+=min(uenb,ueln);
}
if(enb==1 && elz==1){
OP+=min(uenb,uelz)*NB;
utotal+=min(uenb,uelz);
}
if(enb==1 && elp==1){
OP+=min(uenb,uelp)*P;
utotal+=min(uenb,uelp);
}
if(enb==1 && elpb==1){
OP+=min(uenb,uelpb)*PB;
utotal+=min(uenb,uelpb);
}
if(en==1 && elnb==1){
OP+=min(uen,uelnb)*NB;
utotal+=min(uen,uelnb);
}
if(en==1 && eln==1){OP+=min(uen,ueln)*N;
utotal+=min(uen,ueln);
}
if(en==1 && elz==1){
OP+=min(uen,uelz)*N;
utotal+=min(uen,uelz);
}
if(en==1 && elp==1){
OP+=min(uen,uelp)*P;
utotal+=min(uen,uelp);
}
if(en==1 && elpb==1){
OP+=min(uen,uelpb)*PB;
utotal+=min(uen,uelpb);
}
if(ez==1 && elnb==1){
OP+=min(uez,uelnb)*NB;
utotal+=min(uez,uelnb);
}
if(ez==1 && eln==1){
OP+=min(uez,ueln)*N;
utotal+=min(uez,ueln);
}
if(ez==1 && elz==1){
OP+=min(uez,uelz)*Z;utotal+=min(uez,uelz);
}
if(ez==1 && elp==1){
OP+=min(uez,uelp)*P;
utotal+=min(uez,uelp);
}
if(ez==1 && elpb==1){
OP+=min(uez,uelpb)*PB;
utotal+=min(uez,uelpb);
}
if(ep==1 && elnb==1){
OP+=min(uep,uelnb)*NB;
utotal+=min(uep,uelnb);
}
if(ep==1 && eln==1){
OP+=min(uep,ueln)*N;
utotal+=min(uep,ueln);
}
if(ep==1 && elz==1){
OP+=min(uep,uelz)*P;
utotal+=min(uep,uelz);
}
if(ep==1 && elp==1){
OP+=min(uep,uelp)*P;
utotal+=min(uep,uelp);}
if(ep==1 && elpb==1){
OP+=min(uep,uelpb)*PB;
utotal+=min(uep,uelpb);
}
if(epb==1 && elnb==1){
OP+=min(uepb,uelnb)*NB;
utotal+=min(uepb,uelnb);
}
if(epb==1 && eln==1){
OP+=min(uepb,ueln)*NB;
utotal+=min(uepb,ueln);
}
if(epb==1 && elz==1){
OP+=min(uepb,uelz)*PB;
utotal+=min(uepb,uelz);
}
if(epb==1 && elp==1){
OP+=min(uepb,uelp)*PB;
utotal+=min(uepb,uelp);
}
if(epb==1 && elpb==1){
OP+=min(uepb,uelpb)*PB;
utotal+=min(uepb,uelpb);
}pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
if(pid_i_mem_pitch > out_max_pitch)pid_i_mem_pitch = out_max_pitch;
else
if(pid_i_mem_pitch
<
out_max_pitch
*
-1)pid_i_mem_pitch
=
out_max_pitch * -1;
pid_output_pitch=((OP/utotal)+pid_i_mem_pitch);
if (pid_output_pitch > out_max_pitch) pid_output_pitch=out_max_pitch;
if (pid_output_pitch < out_max_pitch*-1) pid_output_pitch=out_max_pitch*-1;
pid_last_pitch_d_error = pid_error_temp;
uez=0.00;
uen=0.00;
uenb=0.00;
uep=0.00;
uepb=0.00;
uelz=0.00;
ueln=0.00;
uelnb=0.00;
uelpb=0.00;
uelp=0.00;
OP=0.00;
utotal=0.00;pid_error_temp = -pid_yaw_setpoint+gyro_yaw_input ;
pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw *
-1;
pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw +
pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -
1;
pid_last_yaw_d_error = pid_error_temp;
}
int convert_receiver_channel(byte function){
byte channel, reverse;
int low, center, high, actual;
int difference;
channel = eeprom_data[function + 23] & 0b00000111;
if(eeprom_data[function + 23] & 0b10000000)reverse = 1;
else reverse = 0;
actual = receiver_input[channel];
low = (eeprom_data[channel * 2 + 15] << 8) | eeprom_data[channel * 2 + 14];
center = (eeprom_data[channel * 2 - 1] << 8) | eeprom_data[channel * 2 - 2];
high = (eeprom_data[channel * 2 + 7] << 8) | eeprom_data[channel * 2 + 6];
if(actual < center){
if(actual < low)actual = low;
difference = ((long)(center - actual) * (long)500) / (center - low);if(reverse == 1)return 1500 + difference;
else return 1500 - difference;
}
else if(actual > center){
if(actual > high)actual = high;
difference = ((long)(actual - center) * (long)500) / (high - center);
if(reverse == 1)return 1500 - difference;
else return 1500 + difference;
}
else return 1500;
}
void set_gyro_registers(){
//Setup the MPU-6050
if(eeprom_data[31] == 1){
Wire.beginTransmission(gyro_address);
Wire.write(0x6B);
Wire.write(0x00);
Wire.endTransmission();
Wire.beginTransmission(gyro_address);
Wire.write(0x1B);
Wire.write(0x08);
Wire.endTransmission();
Wire.beginTransmission(gyro_address);
Wire.write(0x1C);
Wire.write(0x10);Wire.endTransmission();
Wire.beginTransmission(gyro_address);
Wire.write(0x1B);
Wire.endTransmission();
Wire.requestFrom(gyro_address, 1);
while(Wire.available() < 1);
if(Wire.read() != 0x08){
digitalWrite(12,HIGH);
while(1)delay(10);
}
Wire.beginTransmission(gyro_address);
Wire.write(0x1A);
Wire.write(0x03);
Wire.endTransmission();
}
}