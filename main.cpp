#include "mbed.h"
#include "PS3.h"
#include "QEI.h"
#include "PIDcontroller.h"
#include "iostream"
#include "sstream"
#include "iomanip"
#include "BNO055.h"
using namespace std;


#define LEFT_FRONT 0x22
#define RIGHT_FRONT 0x14
#define LEFT_BACK 0x42
#define RIGHT_BACK 0x24

#define FAN 0x62

PS3 ps3(A0,A1);
I2C i2c(D14,D15);
BNO055 bno(D14, D15);
QEI roller_lf(D5,D6,NC,2048,QEI::X2_ENCODING); //b,a
QEI roller_rf(D9,D10,NC,2048,QEI::X2_ENCODING);
QEI roller_lb(D3,D4,NC,2048,QEI::X2_ENCODING);
QEI roller_rb(D7,D8,NC,2048,QEI::X2_ENCODING);
//QEI fan(D11,D12,NC,2048,QEI::X2_ENCODING);

PID PID_lf(5.2,  0.0,  2.5, 0.050);
PID PID_rf(5.2,  0.0,  2.5, 0.050);
PID PID_lb(5.2,  0.0,  2.5, 0.050);
PID PID_rb(5.2,  0.0,  2.5, 0.050);

DigitalOut  sig(D13);
DigitalIn   led(PD_2);
void send(char add, char data);
void getdata(void);
//void motor(int F1, int F2, int B1, int B2); //0:stop, 1:forward, 2:reverse
void motor(char F1, char F2, char B1, char B2);
void get_rpm();
void fan_rpm();

void PID_asi();
void get_angles();

void fan_control();

int Ry, Rx, Ra, Ly, Lx, La;

bool L1, R1, L2, R2, ue, sita, migi, hidari, maru, sankaku, sikaku, batu;

double yaw;

bool corner_mode; //コーナーで90度曲がるときにtrueにする
double corner_yaw; //コーナー前の角度を保存
double corner_minus; //yawと↑の差
double yaw_kizyun = 0.0; //直進する基準
double hosei_yaw; //本当の角度から基準を引いて、直進方向を０とする

double hosei = 3.0; //補正の閾値(度)
bool hosei_mode; //角度を補正する
bool low_magaru_mode;
bool high_magaru_mode;

bool high_mode; //速度アップ

Ticker flip;

int input_pwm[4];
double puls_kotaiti[4] = { 169.45, 169.4, 174.3, 162.75};//4
double minus_kotaiti[4] = { 152.1, 141.0, 147.2, 134.75};//2
int output_pwm[4];

int LF,RF,LB,RB; //目標値
int pulse[4]; // lf:0, rf, lb:2, rb:3
double rpm[4]; // ↑↑

int fan_pulse;
double rpm_fan;

char PID_data[4];
char true_motor[4];

bool fan_mode;
bool fan_kyousei;
char fan_speed = 128;

int main(){ 


    sig = 0;

    char stop = 0x80;
    char forward = 0x99; //正転
    char reverse = 0x67; //逆転

    char low_forward = 0x90;
    char low_reverse = 0x70;

    char high_forward = 0xff;
    char high_reverse = 0x00;

    char high_low_forward = 0xf5;
    char high_low_reverse = 0x0b;

    bno.reset();
    while(!bno.check());
    printf("bno start.\n");
    bno.setmode(OPERATION_MODE_NDOF);

    roller_lf.reset();
    roller_rf.reset();
    roller_lb.reset();
    roller_rb.reset();
    //fan.reset();

    flip.attach(&fan_control,200ms);
    while (true) {
        string joy_state;
        getdata();
        get_angles();

        // PIDなし
        if(high_mode == false && corner_mode == false && hosei_mode == false){
            if(L1 || R1){ //回転モード
                if(L1 && R1){
                    joy_state = "error";
                }else if(L1){
                    joy_state = "left_turn";
                    motor(reverse,reverse,reverse,reverse);
                }else if(R1){
                    joy_state = "right_turn";
                    motor(forward,forward,forward,forward);
                }
            }else{ //通常移動
                if(La >= 135/2 && La < 225/2){
                    joy_state = "mae";
                    motor(forward,reverse,forward,reverse);
                }else if(La >= 225/2 && La < 315/2){
                    joy_state = "hidari_mae";
                    motor(stop,reverse,forward,stop);
                }else if(La >= 315/2 || La < -(315/2)){
                    joy_state = "hidari";
                    motor(reverse,reverse,forward,forward);
                }else if(La >= -(315/2) && La < -(225/2)){
                    joy_state = "hidari_usiro";
                    motor(reverse,stop,stop,reverse);
                }else if(La >= -(225/2) && La < -(135/2)){
                    joy_state = "usiro";
                    motor(reverse,forward,reverse,forward);
                }else if(La >= -(135/2) && La < -(45/2)){
                    joy_state = "migi_usiro";
                    motor(stop,forward,reverse,stop);
                }else if(La >= -(45/2) && La < 45/2 && !(Lx == 0)){
                    joy_state = "migi";
                    motor(forward,forward,reverse,reverse);
                }else if(La >= 45/2 && La < 135/2){
                    joy_state = "migi_mae";
                    motor(forward,stop,stop,reverse);
                }else{
                    joy_state = "none";
                    motor(stop,stop,stop,stop);        
                }
            }
        }

        if(sikaku){
            bno.reset();
            bno.setmode(OPERATION_MODE_NDOF);
        }
        if(maru && corner_mode == false){
            if(corner_mode == false){
                corner_yaw = 0.0;     //角度保存
            }
            corner_mode = true;
            while(ps3.getButtonState(PS3::maru));
        }else if(maru && corner_mode == true){
            corner_mode = false;
            while(ps3.getButtonState(PS3::maru));
        }

        if(corner_mode == true){
            joy_state = "corner";
            motor(reverse,reverse,reverse,reverse);
            corner_minus = corner_yaw - hosei_yaw;
            if(corner_minus >= 80 || (corner_minus < 0 && (corner_minus + 360) >= 80)){ //90度曲がれた時にモード切替
                corner_mode = false;
                joy_state = "corner_finish";
                
                motor(stop,stop,stop,stop);
                yaw_kizyun = yaw_kizyun - 90.0;
                if(yaw_kizyun < 0.0){
                    yaw_kizyun = yaw_kizyun + 360.0;
                }
            }
        }
        
        if(batu && corner_mode == false){
            while(ps3.getButtonState(PS3::batu));
            corner_yaw = 0.0;
            corner_minus = corner_yaw - hosei_yaw;
            while((corner_minus >= 0 && corner_minus < 45) || (corner_minus < 0 && (corner_minus + 360) < 45)){
                get_angles();
                corner_minus = corner_yaw - hosei_yaw;
                motor(0x20,0x20,0x20,0x20);
                if(ps3.getButtonState(PS3::batu)){
                    motor(stop,stop,stop,stop);
                    break;
                }
            }
            motor(stop,stop,stop,stop);
            while(ps3.getButtonState(PS3::batu));
        }
        
        if(sankaku){
            joy_state = "hosei";
            hosei_mode = true;
        }else{
            hosei_mode = false;
        }
        if(ue){
            high_mode = true;
        }else{
            high_mode = false;
        }

        if(corner_mode == false && high_mode == true && hosei_mode == false){
            joy_state = "high_nohosei";
            motor(high_forward,high_reverse,high_forward,high_reverse);
        }else if(corner_mode == false && high_mode == true && hosei_mode == true){
            if((hosei_yaw < hosei || hosei_yaw > (360 - hosei)) && high_magaru_mode == false){
                motor(high_forward,high_reverse,high_forward,high_reverse);
            }else{
                high_magaru_mode = true;
            }
        }


        if(La >= 135/2 && La < 225/2 && hosei_mode == true && high_mode == false){
            if((hosei_yaw < hosei || hosei_yaw > (360 - hosei)) && low_magaru_mode == false){
                motor(forward,reverse,forward,reverse);
            }else{
                low_magaru_mode = true;
            }
        }
        if(corner_mode == false && hosei_mode == true){
            if(low_magaru_mode == true){
                if(hosei_yaw > 1.0 && 180.0 > hosei_yaw){
                    motor(low_forward,reverse,low_forward,reverse);
                }else if(359.0 > hosei_yaw && hosei_yaw >= 180.0){
                    motor(forward,low_reverse,forward,low_reverse);
                }else{
                    low_magaru_mode = false;
                    high_magaru_mode = false;
                }
            }else if(high_magaru_mode == true){
                if(hosei_yaw > 1.0 && 180.0 > hosei_yaw){
                    motor(high_low_forward,high_reverse,high_low_forward,high_reverse);
                }else if(359.0 > hosei_yaw && hosei_yaw >= 180.0){
                    motor(high_forward,high_low_reverse,high_forward,high_low_reverse);
                }else{
                    low_magaru_mode = false;
                    high_magaru_mode = false;
                }
            }
        }

        if(R2 && fan_mode == false){
            fan_kyousei = false;
            fan_mode = true;
            while(ps3.getButtonState(PS3::R2));
        }else if(R2 && fan_mode == true){
            fan_mode = false;
            while(ps3.getButtonState(PS3::R2));
        }
        if(L2){
            fan_kyousei = true;
            fan_mode = false;
            fan_speed = 128;
        }
        //send(FAN,fan_speed);
        printf("yaw: %f, ",yaw);
        printf("hoyaw: %f, ",hosei_yaw);
        /*
        for(int n = 0;n < 4;n++){
            printf("%f, ",rpm[n]);
        }
        */
        //printf("fan: %f ,",rpm_fan);

        cout << ", 足： " << joy_state << endl;
        ThisThread::sleep_for(10ms);
    } 
} 

void send(char add, char data){ //I2C 送信
    wait_us(20);
    i2c.start(); 
    i2c.write(add); 
    i2c.write(data); 
    i2c.stop(); 
} 

void getdata(void){ //PS3　情報受取
    //ジョイスティック
    Ry = ps3.getRightJoystickYaxis();
    Rx = ps3.getRightJoystickXaxis();
    Ra = ps3.getRightJoystickAngle();
    Ly = ps3.getLeftJoystickYaxis();
    Lx = ps3.getLeftJoystickXaxis();
    La = ps3.getLeftJoystickAngle();

    L1 = ps3.getButtonState(PS3::L1);
    R1 = ps3.getButtonState(PS3::R1);
    L2 = ps3.getButtonState(PS3::L2);
    R2 = ps3.getButtonState(PS3::R2);

    ue = ps3.getButtonState(PS3::ue);
    sita = ps3.getButtonState(PS3::sita);
    migi = ps3.getButtonState(PS3::migi);
    hidari = ps3.getButtonState(PS3::hidari);

    maru = ps3.getButtonState(PS3::maru);
    batu = ps3.getButtonState(PS3::batu);
    sankaku = ps3.getButtonState(PS3::sankaku);
    sikaku = ps3.getButtonState(PS3::sikaku);
}


void motor(char F1, char F2, char B1, char B2){ //足回り制御_PIDなし (LEFT_FRONT, RIGHT_FRONT, LEFT_BACK, RIGHT_BACK)
    input_pwm[0] = F1; input_pwm[1] = F2; input_pwm[2] = B1; input_pwm[3] = B2;
    for(int n = 0;n < 4;n++){
        input_pwm[n] = input_pwm[n] - 128;
        if(input_pwm[n] > 0){//基準４
            output_pwm[n] = round(minus_kotaiti[3] / puls_kotaiti[n] * (double)input_pwm[n]);
            output_pwm[n] = output_pwm[n] + 128;
        }else if(input_pwm[n] < 0){//基準２
            output_pwm[n] = round(minus_kotaiti[3] / minus_kotaiti[n] * (double)abs(input_pwm[n]));
            output_pwm[n] = 128 - output_pwm[n];
        }else{
            output_pwm[n] = 128;
        }
    }
    send( LEFT_FRONT, output_pwm[0]);
    send( RIGHT_FRONT, output_pwm[1]);
    send( LEFT_BACK, output_pwm[2]);
    send( RIGHT_BACK, output_pwm[3]);
}


void get_rpm(){
    pulse[0] = roller_lf.getPulses();
    pulse[1] = roller_rf.getPulses();
    pulse[2] = roller_lb.getPulses();
    pulse[3] = roller_rb.getPulses();
    roller_lf.reset();
    roller_rf.reset();
    roller_lb.reset();
    roller_rb.reset();
    for(int i = 0;i < 4;i++){
        rpm[i] = (60 * 5 * (double)pulse[i]) / (2048 * 2);
    }
    //PID_asi();
}

void fan_rpm(){
    //fan_pulse = fan.getPulses();
    //fan.reset();
    //rpm_fan = (60 * 5 * (double)fan_pulse) / (2048 * 2);
}

void get_angles(){
    bno.get_angles();
    yaw = bno.euler.yaw;
    hosei_yaw = yaw - yaw_kizyun;
    if(hosei_yaw < 0.0){
        hosei_yaw = hosei_yaw + 360.0;
    }
}

void fan_control(){
    if(fan_kyousei == false && fan_mode == true && fan_speed > 0){
        fan_speed = fan_speed - 1;
    }else if(fan_kyousei == false && fan_mode == false && fan_speed < 128){
        fan_speed = fan_speed + 1;
    }
}
