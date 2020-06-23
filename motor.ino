#include <math.h>
#include <Wire.h>
#include <EEPROM.h>


// Motor/追尾パラメータ
double OneDayTime = 86164.0905308329; //1日の秒数 [sec]
double MotorR = 1.8;                //モーターの回転角(1pulse) [°]
double MotorMicroStep = 16;         //モーターのマイクロステップ設定
double MotorRatio = 488;            //トータル減速比
double MotorCycle = 0;              //1pulseの周期 [us]

double ClockFrequncy = 16;          //クロック周波数[MHz]
double Prescaler = 256;             //分周比


//モーター設定
int RA_Motor_dir = 1;               //RAモーター回転方向反転設定(1正転 or -1逆転)
int DEC_Motor_dir = 1;               //DECモーター回転方向反転設定(1正転 or -1逆転)
unsigned long MotorWait = 80;                //モーター１パルスの最低間隔[us]
unsigned long MotorWait_upper = 4000;        //モーター１パルスの最大間隔[us]

unsigned long MotorWait_val = 80;            //コマンドでの移動時のモーター１パルスの最低間隔[μs];

int DEC_Auto_enable = 1;            //自動でDECのモーターをON/OFFにする(1:する 0:しない)


int RA_Auto_enable = 0;             //RAは恒星追尾以外で動いた後は停止
int RA_DISENABLE_COUNT = 0;         //TMS2208用　恒星追尾以外で動いた後に消費電力が大きいので一度止める

long RA_count = 0;                  //RA側の出力パルス数
long RA_no_touch_count = 0;         //RA側の出力パルス数 恒星追尾/ガイド修正
int now_RA_enable = 0;              //現在のRAモーターイネーブル状態 (0:DISENABLE 1:ENABLE)

long DEC_count = 0;                 //DEC側の出力パルス数
long DEC_no_touch_count = 0;        //DEC側の出力パルス数 恒星追尾/ガイド修正
int now_DEC_enable = 0;             //現在のDECモーターイネーブル状態 (0:DISENABLE 1:ENABLE)

int auto_tracking = 1;               // 恒星自動追尾　1: する -1: しない

const int DIR_FORWARD = HIGH;
const int DIR_REVERS = LOW;


//加減速制御
long ACC_FACTOR;
long RA_acc_wait_now = 0;
long DEC_acc_wait_now = 0;


//I2Cのアドレス
const int I2C_SLAVE_ADDRESS = 0x04;

//GPIO ピン定義
const int RA_DIR = 2;     //PD2
const int RA_STEP = 3;    //PD3
const int RA_ENABLE = 4;  //PD4

const int DEC_DIR = 5;    //PD5
const int DEC_STEP = 6;   //PD6
const int DEC_ENABLE = 7; //PD7

#define RA_DIR_HIGH {PORTD |=  _BV(PD2);}
#define RA_DIR_LOW  {PORTD &= ~_BV(PD2);}
#define RA_STEP_HIGH {PORTD |=  _BV(PD3);}
#define RA_STEP_LOW  {PORTD &= ~_BV(PD3);}
#define RA_ENABLE_HIGH {PORTD |=  _BV(PD4);}
#define RA_ENABLE_LOW  {PORTD &= ~_BV(PD4);}

#define DEC_DIR_HIGH {PORTD |=  _BV(PD5);}
#define DEC_DIR_LOW  {PORTD &= ~_BV(PD5);}
#define DEC_STEP_HIGH {PORTD |=  _BV(PD6);}
#define DEC_STEP_LOW  {PORTD &= ~_BV(PD6);}
#define DEC_ENABLE_HIGH {PORTD |=  _BV(PD7);}
#define DEC_ENABLE_LOW  {PORTD &= ~_BV(PD7);}


///////////
//////////
//EEPROM MAP
#define ADR_CHECK_SUM       0  //2byte EEPROM チェックサム（チェックサムNGの場合は初期化処理を実施)
#define ADR_DEC_DIR         2  //1byte DEC 方向 0(+) or 1(-)
#define ADR_RA_DIR          3  //1byte RA 方向 0(+) or 1(-)
#define ADR_MOTOR_RATIO     4  //4byte
#define ADR_MOTOR_R         8  //4byte
#define ADR_MICRO_STEP      12 //4byte
#define ADR_MOTOR_WAIT      16 //4byte
#define ADR_ACC_FACTOR      20 //4byte



int eeprom_sum() {
  int sum = 0;
  for (int i = 2; i < 100; ++i) {
    sum = sum ^ EEPROM.read(i);
  }

  return sum & 0xffff;
}

void write_eeprom_sum() {
  int sum = eeprom_sum();
  EEPROM.write(0, (byte)(sum & 0xff));
  EEPROM.write(1, (byte)((sum >> 8) & 0xff));

  logout("write eeprom sum\n");
}

int check_eeprom_sum() {
  int sum = eeprom_sum();

  int d0 = EEPROM.read(0);
  int d1 = EEPROM.read(1);

  long val = (d0 & 0xff) + ((d1 << 8) & 0xff00);

  return val - sum;
}

void load_eeprom() {
  logout("load eeprom\n");

  auto_tracking = 1;

  if (check_eeprom_sum() != 0) {
    logout("init eeprom\n");
    init_eeprom();
  }

  eeprom_get_dec_dir();
  eeprom_get_ra_dir();

  MotorRatio = read_eeprom_double(ADR_MOTOR_RATIO);
  MotorR = read_eeprom_double(ADR_MOTOR_R);
  MotorMicroStep = read_eeprom_long(ADR_MICRO_STEP);
  MotorWait = read_eeprom_long(ADR_MOTOR_WAIT);
  ACC_FACTOR = read_eeprom_long(ADR_ACC_FACTOR);

}

void init_eeprom() {
  eeprom_set_dec_dir(1);
  eeprom_set_ra_dir(1);

  write_eeprom_double(ADR_MOTOR_RATIO, 488.0);
  write_eeprom_double(ADR_MOTOR_R, 1.8);
  write_eeprom_long(ADR_MICRO_STEP, 16);
  write_eeprom_long(ADR_MOTOR_WAIT, 80);
  write_eeprom_long(ADR_ACC_FACTOR, 80000);

  write_eeprom_sum();
}

void write_eeprom_long(int adr, long v) {
  EEPROM.write(adr + 0, (byte)(v & 0xff));
  EEPROM.write(adr + 1, (byte)((v >> 8) & 0xff));
  EEPROM.write(adr + 2, (byte)((v >> 16) & 0xff));
  EEPROM.write(adr + 3, (byte)((v >> 24) & 0xff));
}
void write_eeprom_double(int adr, double v) {
  long lv = (long)(v * 1000);
  write_eeprom_long(adr, lv);
}

long read_eeprom_long(int adr) {
  long d0 = EEPROM.read(adr);
  long d1 = EEPROM.read(adr + 1);
  long d2 = EEPROM.read(adr + 2);
  long d3 = EEPROM.read(adr + 3);

  long ret = (d0 & 0xff) + ((d1 << 8) & 0xff00) + ((d2 << 16) & 0xff0000) + ((d3 << 24) & 0xff000000);
  return ret;
}
double read_eeprom_double(int adr) {
  double ret = ((double)read_eeprom_long(adr)) / 1000.0;
  return ret;
}



void eeprom_set_dec_dir(int d) {
  byte old_d = EEPROM.read(ADR_DEC_DIR);
  byte new_d;
  if (d < 0) {
    new_d = 1;
  } else {
    new_d = 0;
  }
  if (new_d != old_d) {
    EEPROM.write(ADR_DEC_DIR, new_d);
    write_eeprom_sum();
    logout("dec dir write:%d %d\n", d, new_d);
  }
  DEC_Motor_dir = d;
}

void eeprom_get_dec_dir() {
  byte d = EEPROM.read(ADR_DEC_DIR);
  if (d == 0) {
    DEC_Motor_dir = 1;
  } else {
    DEC_Motor_dir = -1;
  }
  logout("dec dir read:%d\n", DEC_Motor_dir);

}

void eeprom_set_ra_dir(int d) {
  byte old_d = EEPROM.read(ADR_RA_DIR);
  byte new_d;
  if (d < 0) {
    new_d = 1;
  } else {
    new_d = 0;
  }
  if (new_d != old_d) {
    EEPROM.write(ADR_RA_DIR, new_d);
    write_eeprom_sum();
    logout("ra dir write:%d %d\n", d, new_d);
  }
  RA_Motor_dir = d;
}

void eeprom_get_ra_dir() {
  byte d = EEPROM.read(ADR_RA_DIR);
  if (d == 0) {
    RA_Motor_dir = 1;
  } else {
    RA_Motor_dir = -1;
  }
   logout("ra dir read:%d\n", RA_Motor_dir);

}


//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
void logout(const char *s, ...) {
  return;
  va_list vp;
  char bff[100];
  va_start(vp, s);
  vsprintf(bff, s, vp);
  Serial.println(bff);
  va_end(vp);
}

void setup() {
  //Serial.begin(115200);

  load_eeprom();


  // I2C設定(Raspberry PI 通信用)
  Wire.begin(I2C_SLAVE_ADDRESS);
  Wire.onReceive(onRecv);
  Wire.onRequest(onReq);

  //タイマー割り込み設定(恒星追尾設定)
  noInterrupts ();
  TCCR1A = 0;    // Clear Timer on Compare (OCR1A) Mode
  TCCR1B = 0x0c; // Clear Timer on Compare (OCR1A) Mode  ClockRatio=1/256
  TIMSK1 = 0x02;
  TCNT1 = 0;
  TIFR1 = 0x02;
  TIMSK1 = 0x02;
  MotorCycle = get_motor_cycle();
  set_Interrupt_Trriger(MotorCycle);
  interrupts ();

  //モーター用 DIR/EN/STEPピン設定
  //RA Motor
  pinMode(RA_DIR, OUTPUT);
  pinMode(RA_STEP, OUTPUT);
  pinMode(RA_ENABLE, OUTPUT);
  //pinMode(RA_ENABLE, INPUT);

  //DEC Motor
  pinMode(DEC_DIR, OUTPUT);
  pinMode(DEC_STEP, OUTPUT);
  pinMode(DEC_ENABLE, OUTPUT);

  if (DEC_Auto_enable == 1) {
    digitalWrite(DEC_ENABLE, HIGH); //DECは初期DISENABLE
  }

  set_accel();

  logout("STARTUP -- OUT\n");
}

///////////////////////////////////////
//////////////////////////////////////
////I2C

void onRecv(int inNumOfRecvBytes) {
  //logout("get command:%d", inNumOfRecvBytes);

  if (inNumOfRecvBytes > 30) {
    logout("i2c recive len over error:%d\n", inNumOfRecvBytes);
    inNumOfRecvBytes = 30;
  }

  char str[inNumOfRecvBytes + 1];
  int count = 0;
  unsigned char sum = 0;
  while (Wire.available() > 0) {
    char c = Wire.read();
    str[count] = c;
    sum = sum + c;
    ++count;
    if (count > inNumOfRecvBytes) {
      logout("i2c recive error:%d\n", inNumOfRecvBytes);
      break;
    }
  }
  if (count < 2) {
    logout("i2c command len short error\n");
    return;
  }

  sum = sum - str[count - 1] - str[count - 2];
  unsigned char sum1 = '0' + (sum % 100) / 10;
  unsigned char sum2 = '0' + (sum % 10);

  str[count] = '\0';
  //logout("sum %d %d %d %d %s\n",(int)sum1,(int)(sum2),(int)str[count-2],(int)str[count-1],str);
  if ((sum1 != str[count - 2]) || (sum2 != str[count - 1])) {
    logout("i2c check sum error\n");
    str[0] = '\0';
    return;
  }

  str[count - 2] = '\0';
  //logout("command:%s", str);

  command(str);

}

byte send_data[256];
byte send_len = 0;
void onReq() {
  if (send_len == 0) {
    logout("write i2c:no data");
    return;
  }
  send_data[send_len] = '\0';

  //logout("write i2c:%s", send_data);
  Wire.write(send_data, send_len);
  send_len = 0;
}


/////////////////////////////////////////////
/////////////////////////
///////////////////
///タイマー割り込み処理
ISR(TIMER1_COMPA_vect) {
  if (auto_tracking == -1) return;

  //恒星追尾
  noInterrupts ();
  ++RA_no_touch_count;
  // ++DEC_no_touch_count;
  interrupts ();
}

void loop() {
  if ((RA_no_touch_count != 0) || (RA_count != 0)) {
    step_RA_motor();
  } else {
    DisEnable_RA_motor();
  }
  if (RA_count == 0) {
    RA_acc_wait_now = MotorWait_upper;
  }


  if ((DEC_no_touch_count != 0) || (DEC_count != 0)) {
    step_DEC_motor();
  } else {
    DisEnable_DEC_motor();
  }
  if (DEC_count == 0) {
    DEC_acc_wait_now = MotorWait_upper;
  }

}

//////////////////////////////////////////////////////
//////////////////////////////////////////////////////

//タイマー周期を設定
void set_Interrupt_Trriger(double us) {
  unsigned int t = us * ClockFrequncy / Prescaler;
  OCR1A = t;
}

//1pulseの間隔をμｓで返す
double get_motor_cycle() {
  return (OneDayTime * 1000 * 1000 * MotorR) / (360 * MotorRatio * MotorMicroStep);
}


/////////////////////////////////////////////
////////////////////////////////////////////
//モーター制御


long ACC_max_step;
long fit_acc_step[10][2];
int fit_acc_max = 0;

void set_accel() {
  ACC_max_step = 0;
  long r = MotorWait_upper;
  long next_div = MotorWait_upper * 0.5;

  fit_acc_max = 0;
  fit_acc_step[fit_acc_max][0] = r;
  fit_acc_step[fit_acc_max][1] = ACC_max_step;
  ++fit_acc_max;

  long tr;
  while ((tr = get_next_wait(r, 1000, 0)) > MotorWait) {
    //logout("acc r:%ld tr:%ld\n", r, tr);

    if (tr >= r) {
      r = r - 1;
    } else {
      r = tr;
    }

    ++ACC_max_step;
    if ((next_div > r) && (fit_acc_max < 9)) {
      fit_acc_step[fit_acc_max][0] = r;
      fit_acc_step[fit_acc_max][1] = ACC_max_step;
      ++fit_acc_max;
      next_div = next_div * 0.5;
    }
  }

  fit_acc_step[fit_acc_max][0] = MotorWait;
  fit_acc_step[fit_acc_max][1] = ACC_max_step;
  ++fit_acc_max;

  /*
    for (int i = 0; i < fit_acc_max; ++i) {
      logout("acc table %ld %ld\n", fit_acc_step[i][0], fit_acc_step[i][1]);
    }
    for (long i = MotorWait_upper; i > MotorWait; i = i * 95 / 100) {
      get_acc_step(i);
    }
  */

}


long cacsh_acc_step = -100;
long cacsh_acc_step_val = 0;
long get_acc_step(long r) {
  int s;

  if (r == cacsh_acc_step) return cacsh_acc_step_val;

  cacsh_acc_step = r;

  for (s = 0; s < fit_acc_max; ++s) {
    if (fit_acc_step[s][0] < r) break;
  }
  if (s == 0) {
    cacsh_acc_step_val = 0;
    return 0;
  }
  if (s >= fit_acc_max) {
    cacsh_acc_step_val = ACC_max_step;
    return ACC_max_step;
  }

  long ll = fit_acc_step[s - 1][1] - (fit_acc_step[s][1] - fit_acc_step[s - 1][1]) * (r - fit_acc_step[s - 1][0]) / (fit_acc_step[s - 1][0] - fit_acc_step[s][0]);
  if (ll <= 0) ll = 0;
  if (ll >= ACC_max_step) ll = ACC_max_step;

  //logout("acc calc %d %ld %ld\n",s,r,ll);

  cacsh_acc_step_val = ll;

  return ll;
}

long get_next_wait(long r, long count, long d) {
  long ll;

  if (count < 0) count = -count;

  if (count > d) {
    ll = (long)(r - r * r / ACC_FACTOR);
    if (ll >= r) ll = r - 1;
  } else {
    ll = (long)(r + r * r / ACC_FACTOR);
    if (ll <= r) ll = r + 1;
  }

  if (ll > MotorWait_upper) ll = MotorWait_upper;
  if (ll < MotorWait) ll = MotorWait;

  return ll;
}



unsigned long RA_time = 0;
long now_RA_count = 0;

void step_RA_motor() {
  unsigned long t = micros();
  if (MotorWait_val > RA_acc_wait_now) {
    if ((t - RA_time) < MotorWait_val) {
      return;
    }
  } else {
    if ((t - RA_time) < RA_acc_wait_now) {
      return;
    }
  }

  RA_acc_wait_now = get_next_wait(RA_acc_wait_now, RA_count, get_acc_step(RA_acc_wait_now));


  int do_step_f = 0;

  int dir = DIR_FORWARD;

  long c = RA_no_touch_count;

  if (c < 0) {
    dir = DIR_REVERS;

    noInterrupts ();
    ++RA_no_touch_count;
    if (RA_count > 0) {
      do_step_f = 1;
      --RA_count;
      ++now_RA_count;
    }
    interrupts ();
  } else if (c > 0) {
    noInterrupts ();
    --RA_no_touch_count;
    if (RA_count < 0) {
      do_step_f = 1;
      ++RA_count;
      --now_RA_count;
    }
    interrupts ();
  } else {
    do_step_f = 1;
  }

  if (do_step_f == 1) {
    dir = DIR_FORWARD;
    c = RA_count;

    if (c == 0) return ;
    if (c < 0) {
      dir = DIR_REVERS;
      noInterrupts ();
      ++RA_count;
      --now_RA_count;
      interrupts ();
    } else {
      noInterrupts ();
      --RA_count;
      ++now_RA_count;
      interrupts ();
    }
  }

  RA_ENABLE_LOW;        //digitalWrite(RA_ENABLE, LOW);
  RA_STEP_HIGH;         //digitalWrite(RA_STEP, HIGH);

  if (RA_Motor_dir < 0) {
    if (dir == DIR_FORWARD) {
      dir = DIR_REVERS;
    } else {
      dir = DIR_FORWARD;
    }
  }

  now_RA_enable = 1;
  if (dir == DIR_FORWARD) {
    RA_DIR_LOW;         //digitalWrite(RA_DIR, LOW);
  } else {
    RA_DIR_HIGH;        //digitalWrite(RA_DIR, HIGH);
  }

  RA_STEP_LOW;          //digitalWrite(RA_STEP, LOW);

  RA_time = t;//micros();

  RA_Auto_enable = RA_DISENABLE_COUNT;
}


void DisEnable_RA_motor() {
  if (RA_Auto_enable == 0) return;
  if (now_RA_enable == 0) return;

  if ((RA_count != 0) || (RA_no_touch_count != 0)) return;

  unsigned long t = micros();
  if ((t - RA_time) < MotorWait) {
    return;
  }
  now_RA_enable = 0;
  RA_ENABLE_HIGH;     //digitalWrite(RA_ENABLE, HIGH);
  --RA_Auto_enable;
  if (RA_Auto_enable <= 0)   RA_Auto_enable = 0;
}


long now_DEC_count = 0;
unsigned long DEC_time = 0;
void step_DEC_motor() {
  unsigned long t = micros();

  if (MotorWait_val > DEC_acc_wait_now) {
    if ((t - DEC_time) < MotorWait_val) {
      return;
    }
  } else {
    if ((t - DEC_time) < DEC_acc_wait_now) {
      return;
    }
  }

  DEC_acc_wait_now = get_next_wait(DEC_acc_wait_now, DEC_count, get_acc_step(DEC_acc_wait_now));

  int do_step_f = 0;

  int dir = DIR_FORWARD;
  long  c = DEC_no_touch_count;

  if (c < 0) {
    dir = DIR_REVERS;
    noInterrupts ();
    ++DEC_no_touch_count;
    if (DEC_count > 0) {
      do_step_f = 1;
      --DEC_count;
      ++now_DEC_count;
    }
    interrupts ();
  } else if (c > 0) {
    noInterrupts ();
    --DEC_no_touch_count;
    if (DEC_count < 0) {
      do_step_f = 1;
      ++DEC_count;
      --now_DEC_count;
    }
    interrupts ();
  } else {
    do_step_f = 1;
  }

  if (do_step_f == 1) {
    dir = DIR_FORWARD;
    c = DEC_count;

    if (c == 0) return ;
    if (c < 0) {
      dir = DIR_REVERS;
      noInterrupts ();
      ++DEC_count;
      --now_DEC_count;
      interrupts ();
    } else {
      noInterrupts ();
      --DEC_count;
      ++now_DEC_count;
      interrupts ();
    }
  }

  DEC_ENABLE_LOW;       //digitalWrite(DEC_ENABLE, LOW);
  DEC_STEP_HIGH;        //digitalWrite(DEC_STEP, HIGH);

  if (DEC_Motor_dir < 0) {
    if (dir == DIR_FORWARD) {
      dir = DIR_REVERS;
    } else {
      dir = DIR_FORWARD;
    }
  }

  now_DEC_enable = 1;
  if (dir == DIR_FORWARD) {
    DEC_DIR_LOW;        //digitalWrite(DEC_DIR, LOW);
  } else {
    DEC_DIR_HIGH;       //digitalWrite(DEC_DIR, HIGH);
  }

  DEC_STEP_LOW;         //digitalWrite(DEC_STEP, LOW);

  DEC_time = t;//micros();
}

void DisEnable_DEC_motor() {
  if (DEC_Auto_enable == 0) return;
  if (now_DEC_enable == 0) return;

  if ((DEC_count != 0) || (DEC_no_touch_count != 0)) return;

  unsigned long t = micros();
  if ((t - DEC_time) < MotorWait) {
    return;
  }

  now_DEC_enable = 0;
  DEC_ENABLE_HIGH;      //digitalWrite(DEC_ENABLE, HIGH);
}

//////////////////////////////////////
/////////////////////////////////////
//コマンド処理

//制御コマンド
const int CommandMax = 24;
const char *Command[] = {
  "RA_GO",      //0 RA_GO YYYYY     YYYYY:パルス数(符号付long)   戻り値:なし
  "DEC_GO",     //1 DEC_GO YYYYY    YYYYY:パルス数(符号付long)   戻り値:なし
  "RA_STOP",    //2 戻り値:なし
  "DEC_STOP",   //3 戻り値:なし
  "RA_CNT_GO",  //4 RA_CNT_GO 1 or -1  戻り値:なし
  "DEC_CNT_GO", //5 DEC_CNT_GO 1 or -1 戻り値:なし
  "RA_COUNT",   //6 RAの現在の角度（起動からの累積パルス数）を返す（ガイド修正、恒星追尾分を除く）    戻り値: 'R'+long(4byte)+チェックサム(1byte)
  "DEC_COUNT",  //7 DECの現在の角度（起動からの累積パルス数）を返す（ガイド修正分を除く）            戻り値: 'D'+long(4byte)+チェックサム(1byte)
  "STOP_TR",    //8 恒星追尾停止 戻り値:なし
  "START_TR",    //9 恒星追尾開始（デフォルト) 戻り値:なし
  "SET_RG",     //10 スピード設定
  "SET_RC",     //11 スピード設定
  "SET_RM",     //12 スピード設定
  "SET_RS",     //13 スピード設定
  "MOVING_RA",  //14 移動の残りカウントを返す（ガイド修正、恒星追尾分を除く）戻り値: 'r'+long(4byte)+チェックサム(1byte)
  "MOVING_DEC", //15 移動の残りカウントを返す（ガイド修正分を除く）         戻り値: 'd'+long(4byte)+チェックサム(1byte)
  "SET_DEC_DIR",//16 SET_DEC_DIR 0(+) or 1(-) DECの移動方向設定
  "GET_DEC_DIR",//17 DECの移動方向取得　戻り値:'I'+dir(1byte)+チェックサム(1byte)
  "SET_RA_DIR", //18 SET_RA_DIR 0(+) or 1(-) RAの移動方向設定
  "GET_RA_DIR", //19 DECの移動方向取得　戻り値:'i'+dir(1byte)+チェックサム(1byte)
  "SET_PARAM",  //20 パラメータ設定
  "GET_PARAM",  //21 パラメータ取得 0:減速比 1:モーター回転角 2:マイクロステップ 3:最低パルス間隔 4:加速係数
  "G_STOP",     //22 恒星追尾停止
  "G_START",    //23 恒星追尾開始


};
byte sum_data(byte buf[], int len) {
  byte s = 0;
  for (int i = 0; i < len; ++i) {
    s = s + buf[i];
  }
  return s;
}

void command(char *cmd) {

  //logout("command str:%s\n",cmd);

  int c = 0;
  while (cmd[c] != '\0') {
    if (cmd[c] == ' ') break;
    ++c;
    if (c > 1000) {
      logout("command error(too long)");
      return;   //command error
    }
  }
  if (c == 0) {
    logout("command error(len%d)", c);
    return;
  }

  int cmdno = -1;
  for (int i = 0; i < CommandMax; ++i) {
    if (strncmp(cmd, Command[i], c) == 0) {
      cmdno = i;
      break;
    }
  }
  if (cmdno < 0) {
    logout("command error(undifined command)");
    return;
  }

  long val, val2;
  long tmp_ct;
  switch (cmdno) {
    case 0:   sscanf(&cmd[c + 1], "%ld", &val);
      noInterrupts ();
      RA_count += val;
      interrupts ();
      logout("RA_GO count:%ld %ld", val, RA_count);
      break;
    case 1:   sscanf(&cmd[c + 1], "%ld", &val);
      noInterrupts ();
      DEC_count += val;
      interrupts ();
      logout("DEC_GO count:%ld %ld", val, DEC_count);
      break;
    case 2:
      tmp_ct = RA_count;
      noInterrupts ();
      if (RA_count > 0) {
        RA_count = get_acc_step(RA_acc_wait_now);//0
      } else if (RA_count < 0) {
        RA_count = -get_acc_step(RA_acc_wait_now);//0
      }
      interrupts ();

      if (((RA_count > 0) && (tmp_ct < 0)) || ((RA_count < 0) && (tmp_ct > 0))) {
        RA_acc_wait_now = MotorWait_upper;
      }
      break;
    case 3:
      tmp_ct = DEC_count;
      noInterrupts ();
      if (DEC_count > 0) {
        DEC_count = get_acc_step(DEC_acc_wait_now);//0
      } else if (DEC_count < 0) {
        DEC_count = -get_acc_step(DEC_acc_wait_now);//0
      }
      interrupts ();
      if (((DEC_count > 0) && (tmp_ct < 0)) || ((DEC_count < 0) && (tmp_ct > 0))) {
        DEC_acc_wait_now = MotorWait_upper;
      }
      break;
    case 4:   sscanf(&cmd[c + 1], "%ld", &val);
      if (val > 0) {
        val = 2147483647;
      } else {
        val = -2147483648;
      }
      noInterrupts ();
      RA_count = val;
      interrupts ();
      logout("RA_CT_GO");
      break;
    case 5:   sscanf(&cmd[c + 1], "%ld", &val);
      if (val > 0) {
        val = 2147483647;
      } else {
        val = -2147483648;
      }
      noInterrupts ();
      DEC_count = val;
      interrupts ();
      logout("DEC_CT_GO");
      break;
    case 6:   val = now_RA_count;
      send_data[0] = 'R';
      send_data[1] = (byte)(val & 0xff);
      send_data[2] = (byte)((val >> 8) & 0xff);
      send_data[3] = (byte)((val >> 16) & 0xff);
      send_data[4] = (byte)((val >> 24) & 0xff);
      send_data[5] = sum_data(send_data, 5);
      send_len = 6;
      break;

    case 7:   val = now_DEC_count;
      send_data[0] = 'D';
      send_data[1] = (byte)(val & 0xff);
      send_data[2] = (byte)((val >> 8) & 0xff);
      send_data[3] = (byte)((val >> 16) & 0xff);
      send_data[4] = (byte)((val >> 24) & 0xff);
      send_data[5] = sum_data(send_data, 5);
      send_len = 6;
      break;

    case 10://RG
      MotorWait_val = MotorWait * 3 * 3 * 3;
      break;
    case 11://RC
      MotorWait_val = MotorWait * 3 * 3;
      break;
    case 12://RM
      MotorWait_val = MotorWait * 3;
      break;
    case 13://RS
      MotorWait_val = MotorWait;
      break;

    case 14:   val = RA_count;
      send_data[0] = 'r';
      send_data[1] = (byte)(val & 0xff);
      send_data[2] = (byte)((val >> 8) & 0xff);
      send_data[3] = (byte)((val >> 16) & 0xff);
      send_data[4] = (byte)((val >> 24) & 0xff);
      send_data[5] = sum_data(send_data, 5);
      send_len = 6;
      break;
    case 15:   val = DEC_count;
      send_data[0] = 'd';
      send_data[1] = (byte)(val & 0xff);
      send_data[2] = (byte)((val >> 8) & 0xff);
      send_data[3] = (byte)((val >> 16) & 0xff);
      send_data[4] = (byte)((val >> 24) & 0xff);
      send_data[5] = sum_data(send_data, 5);
      send_len = 6;
      break;

    case 16:   sscanf(&cmd[c + 1], "%ld", &val);
      if (val == 0) {
        eeprom_set_dec_dir(1);
      } else {
        eeprom_set_dec_dir(-1);
      }
      break;

    case 17:
      send_data[0] = 'I';
      if (DEC_Motor_dir > 0) {
        send_data[1] = 0;
      } else {
        send_data[1] = 1;
      }
      send_data[2] = sum_data(send_data, 2);
      send_len = 3;
      logout("send dec dir:%d\n",(int)send_data[1]);
      break;

    case 18:   sscanf(&cmd[c + 1], "%ld", &val);
      if (val == 0) {
        eeprom_set_ra_dir(1);
      } else {
        eeprom_set_ra_dir(-1);
      }
      break;

    case 19:
      send_data[0] = 'i';
      if (RA_Motor_dir > 0) {
        send_data[1] = 0;
      } else {
        send_data[1] = 1;
      }
      send_data[2] = sum_data(send_data, 2);
      send_len = 3;
      logout("send ra dir:%d\n",(int)send_data[1]);
      break;

    case 20:   sscanf(&cmd[c + 1], "%ld %ld", &val2, &val);
      //logout("param %ld %ld\n", val2, val);
      switch (val2) {
        case 0: MotorRatio = (double)val / 1000.0;
          write_eeprom_double(ADR_MOTOR_RATIO, MotorRatio);
          break;
        case 1: MotorR = (double)val / 1000.0;
          write_eeprom_double(ADR_MOTOR_R, MotorR);
          break;
        case 2: MotorMicroStep = (double)val;
          write_eeprom_long(ADR_MICRO_STEP, MotorMicroStep);
          break;
        case 3: MotorWait = (unsigned long)val;
          write_eeprom_long(ADR_MOTOR_WAIT, MotorWait);
          break;
        case 4: ACC_FACTOR = (long)val;
          write_eeprom_long(ADR_ACC_FACTOR, ACC_FACTOR);
          break;
      }
      write_eeprom_sum();

      break;

    case 21:
      sscanf(&cmd[c + 1], "%ld", &val);
      switch (val) {
        case 0: val = (long)(MotorRatio * 1000);
          send_data[0] = 'H';
          break;
        case 1: val = (long)(MotorR * 1000);
          send_data[0] = 'R';
          break;
        case 2: val = (long)(MotorMicroStep);
          send_data[0] = 'S';
          break;
        case 3: val = (long)(MotorWait);
          send_data[0] = 'W';
          break;
        case 4: val = (long)(ACC_FACTOR);
          send_data[0] = 'A';
          break;
        default:
          send_data[0] = 'N';
          val = 0;
          break;
      }

      send_data[1] = (byte)(val & 0xff);
      send_data[2] = (byte)((val >> 8) & 0xff);
      send_data[3] = (byte)((val >> 16) & 0xff);
      send_data[4] = (byte)((val >> 24) & 0xff);
      send_data[5] = sum_data(send_data, 5);
      send_len = 6;
      break;

    case 22://G_STOP
      auto_tracking = -1;
      break;

    case 23://G_START
      auto_tracking = 1;
      break;

  }

}


