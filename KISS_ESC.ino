#include <Arduino.h>
#include "DSHOT.h"
#include "ESCCMD.h"

#define ESCPID_USB_UART_SPEED 115200  //シリアル通信の速度
#define ESCPID_NB_ESC 1               //ESCの数

const float LOGGING_CYCLE = 0.01;     //制御周期[s]

IntervalTimer Timer;                  //10msの割り込み

int16_t val = 0;                      //指令値を格納する変数

typedef struct {
  uint32_t      magic;                       // Magic number
  int8_t        err[ESCPID_NB_ESC];          // Last error number
  uint8_t       deg[ESCPID_NB_ESC];          // ESC temperature (Â°C)
  uint16_t      cmd[ESCPID_NB_ESC];          // Current ESC command value
  uint16_t      volt[ESCPID_NB_ESC];         // Voltage of the ESC power supply (0.01V)
  uint16_t      amp[ESCPID_NB_ESC];          // ESC current (0.01A)
  int16_t       rpm[ESCPID_NB_ESC];          // Motor rpm (10 rpm)
} ESCPIDcomm_struct_t;

ESCPIDcomm_struct_t ESCPID_comm = {
  {},
  {},
  {},
  {},
  {},
  {},
  {}
};

void setup() {
  
  Serial.begin( ESCPID_USB_UART_SPEED );

  // Initialize the CMD subsystem
  ESCCMD_init( ESCPID_NB_ESC );

  // Arming ESCs
  ESCCMD_arm_all( );

  // Switch 3D mode on
  ESCCMD_3D_off( );

  // Arming ESCs
  ESCCMD_arm_all( );

  // Start periodic loop
  ESCCMD_start_timer( );

  // Stop all motors
  for (int i = 0; i < ESCPID_NB_ESC; i++ ) {
    ESCCMD_stop( i );
  }

  Timer.priority(200);
  Timer.begin(interrupt_10ms, LOGGING_CYCLE * 1000000);  //10msでタイマー割り込み開始

}

void interrupt_10ms(void)
{
  //シリアルモニタに表示
  for (int i = 0; i < ESCPID_NB_ESC; i++ ) {
    Serial.print("Channel:\t");
    Serial.println(i);
    Serial.print("Error:\t\t");
    Serial.println(ESCPID_comm.err[i]);
    Serial.print("Command:\t");
    Serial.println(ESCPID_comm.cmd[i]);
    Serial.print("Temperature:\t");
    Serial.println(ESCPID_comm.deg[i]);
    Serial.print("Voltage:\t");
    Serial.println(ESCPID_comm.volt[i]*0.01);
    Serial.print("Current:\t");
    Serial.println(ESCPID_comm.amp[i]*0.01);
    Serial.print("rpm:\t\t");
    Serial.println(ESCPID_comm.rpm[i]);
    Serial.println();
  }

  if (KICK_SERIAL())
  {
    val = SERIAL_VAL();//シリアル通信で送られてきた指令値
  }
}


void loop() {
  static int ret;

  ret = ESCCMD_tic();//テレメトリ情報の更新

  //テレメトリ情報の取得
  for (int i = 0; i < ESCPID_NB_ESC; i++ ) {
    ESCCMD_read_err(i, &ESCPID_comm.err[i]);
    ESCCMD_read_cmd(i, &ESCPID_comm.cmd[i]);
    ESCCMD_read_deg(i, &ESCPID_comm.deg[i]);
    ESCCMD_read_volt(i, &ESCPID_comm.volt[i]);
    ESCCMD_read_amp(i, &ESCPID_comm.amp[i]);
    ESCCMD_read_rpm(i, &ESCPID_comm.rpm[i]);
  }

  //ESCへ指令値送信
  if (ret == ESCCMD_TIC_OCCURED)
  {
    for (int i = 0; i < ESCPID_NB_ESC; i++)
    {
      ret = ESCCMD_throttle(i, val);
    }
  }


}

//シリアル通信の受信処理
bool KICK_SERIAL() {
  bool flag = false;
  if (Serial.available() > 0)
  {
    flag = true;
    //delay(20);
  }
  return flag;
}

//シリアル通信の受信処理
int SERIAL_VAL() {

  byte data_size = Serial.available();

  byte buf[data_size], degree = 1;
  long recv_data = 0, dub = 1;
  bool minus = 0;

  for (byte i = 0 ; i < data_size ; i++)
  {
    buf[i] = Serial.read();

    if (buf[i] >= '0' && buf[i] <= '9') buf[i] -= '0';
    else {
      if (buf[0] == '-') minus = 1;
      else degree = 0;
    }
  }
  if (degree == 1) degree = data_size - minus;

  for (byte i = 0 ; i < degree ; i++)
  {
    recv_data += buf[(data_size - 1) - i] * dub;
    dub *= 10;
  }
  if (minus) recv_data *= -1;

  return recv_data;
}