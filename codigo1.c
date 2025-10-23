#include <Arduino.h>
#include <math.h>
#include "esp_task_wdt.h"
extern "C" {
  #include "soc/gpio_reg.h"
  #include "esp_rom_sys.h"
}
#include <driver/gpio.h>

// ===== Config =====
#define ENA_ALWAYS_ON 1        // 1: ENA fijo ON (recomendado)
#define SINK_ACTIVE   1        // 1: activo es LOW
#define PULSE_HIGH_US 5        // ancho de pulso HIGH
const uint32_t MIN_HZ=200, MAX_HZ=60000;  //velocidades min7max

// Pines
const int X_STEP=20, X_DIR=21, X_ENA=47, X_H0=3, X_HF=46;
const int Y_STEP=45,  Y_DIR=0,  Y_ENA=35;
const int Z_STEP=37, Z_DIR=48, Z_ENA=39;

// POS fijo (X)
const uint32_t POS_HZ=10000, POS_ACC_MS=100, POS_DEC_MS=100;

// ===== Tipos =====
struct MoveCmd { uint32_t pasos, hz, acc_ms, dec_ms; uint8_t dir; };// numero de pasos, vel max, tiempo de aceleracion, tiempo de desaceleracion; direccion 1 o 0 

// ===== Métrica X =====
volatile int64_t  x_pos_steps=0;
volatile uint32_t x_span_steps=0;
volatile float    x_mm_total=550.0f;
volatile float    x_steps_per_mm=0.0f;

// ===== RTOS =====
QueueHandle_t qX,qY,qZ;
TaskHandle_t  tX,tY,tZ,tUI;
volatile bool x_busy=false,y_busy=false,z_busy=false;

// ===== Nextion (opcional) =====
static const int NX_TX=17, NX_RX=16; 
#define NX_BAUD 921600
HardwareSerial& nx=Serial2;
unsigned long lastLastSeenMs=0;
volatile uint32_t x_lastEmitted=0, x_lastEmitted1=UINT32_MAX;

// ===== GPIO rápidos =====
IRAM_ATTR static inline void _w1ts(int pin){ uint32_t m=1UL<<(pin&31); if(pin<32) REG_WRITE(GPIO_OUT_W1TS_REG,m); else REG_WRITE(GPIO_OUT1_W1TS_REG,m); }
IRAM_ATTR static inline void _w1tc(int pin){ uint32_t m=1UL<<(pin&31); if(pin<32) REG_WRITE(GPIO_OUT_W1TC_REG,m); else REG_WRITE(GPIO_OUT1_W1TC_REG,m); }
IRAM_ATTR static inline void _hi(int p){ _w1ts(p); } //IRAM_ATTR asegura que enten en RAM interna para maxima velocidad
IRAM_ATTR static inline void _lo(int p){ _w1tc(p); }
IRAM_ATTR static inline void _act(int p,bool on){ if(SINK_ACTIVE){ on?_lo(p):_hi(p);} else { on?_hi(p):_lo(p);} }
IRAM_ATTR static inline void _dir_fast(int p,bool fwd){ _act(p,fwd); esp_rom_delay_us(10); } // 10us lead
IRAM_ATTR static inline int  _gin(int p){ if(p<32) return (REG_READ(GPIO_IN_REG)>>p)&1; else return (REG_READ(GPIO_IN1_REG)>>(p-32))&1; }
inline bool x_limH0(){ return _gin(X_H0)==0; }
inline bool x_limHF(){ return _gin(X_HF)==0; }

// ===== Timers =====
hw_timer_t *tx=nullptr,*ty=nullptr,*tz=nullptr;

typedef struct {
  int pinSTEP, pinDIR, pinENA;
  volatile uint32_t period_us;        // periodo completo
  volatile uint32_t steps_remaining;  // decrece cada pulso
  volatile uint32_t steps_done;       // cont hecho
  volatile bool     running;
  volatile bool     dirFwd;
  // sólo X: run until limit
  volatile bool     until_limit;
  volatile bool     towardHF;
} Axis;

volatile Axis AX_X={}, AX_Y={}, AX_Z={};

// ===== ISR: pulso por evento (edge) =====
void IRAM_ATTR onTimerX(){
  if(!AX_X.running) return;
  _act(AX_X.pinSTEP,true);
  esp_rom_delay_us(PULSE_HIGH_US);
  _act(AX_X.pinSTEP,false);

  AX_X.steps_done++;
  if(!AX_X.towardHF && x_limH0()){ AX_X.running=false; return; }
  if( AX_X.towardHF && x_limHF()){ AX_X.running=false; return; }

  if(AX_X.dirFwd){ if(x_pos_steps<(int64_t)x_span_steps) x_pos_steps++; }
  else           { if(x_pos_steps>0)                     x_pos_steps--; }

  if(!AX_X.until_limit){
    if(AX_X.steps_remaining) AX_X.steps_remaining--;
    if(AX_X.steps_remaining==0){ AX_X.running=false; }
  }
}
void IRAM_ATTR onTimerY(){
  if(!AX_Y.running) return;
  _act(AX_Y.pinSTEP,true); esp_rom_delay_us(PULSE_HIGH_US); _act(AX_Y.pinSTEP,false);
  AX_Y.steps_done++; if(AX_Y.steps_remaining) AX_Y.steps_remaining--; if(AX_Y.steps_remaining==0){ AX_Y.running=false; }
}
void IRAM_ATTR onTimerZ(){
  if(!AX_Z.running) return;
  _act(AX_Z.pinSTEP,true); esp_rom_delay_us(PULSE_HIGH_US); _act(AX_Z.pinSTEP,false);
  AX_Z.steps_done++; if(AX_Z.steps_remaining) AX_Z.steps_remaining--; if(AX_Z.steps_remaining==0){ AX_Z.running=false; }
}

static inline void timer_set_period(hw_timer_t* t, uint32_t per_us){ timerAlarmWrite(t, per_us, true); }

// ===== Ejecutores con rampas =====
static void runMoveExactX(const MoveCmd& c){
  if(c.pasos==0) return;
  uint32_t hzC=constrain(c.hz,MIN_HZ,MAX_HZ);
  float acc_s=max(1u,c.acc_ms)/1000.0f, dec_s=max(1u,c.dec_ms)/1000.0f;
  // calcula aceleracio, velocidad constante y desaceleracion
  float S_up=0.5f*hzC*acc_s, S_dn=0.5f*hzC*dec_s;
  if(c.pasos < (uint32_t)(S_up+S_dn)){//ajuste de perfil si el movimiento es muy corto 
    float peak=(2.0f*c.pasos)/(acc_s+dec_s);
    hzC=constrain((uint32_t)peak,MIN_HZ,MAX_HZ);
    S_up=0.5f*hzC*acc_s; S_dn=0.5f*hzC*dec_s;
  }
  uint32_t n_up=(uint32_t)lroundf(S_up);
  uint32_t n_dn=(uint32_t)lroundf(S_dn);
  uint32_t n_cru=(c.pasos>(n_up+n_dn))?(c.pasos-n_up-n_dn):0;
  const float f0=(float)MIN_HZ, fC=(float)hzC;

  if(!ENA_ALWAYS_ON){
    _act(X_ENA,true); vTaskDelay(pdMS_TO_TICKS(200));
  }
  _dir_fast(X_DIR, c.dir!=0);

  AX_X.steps_done=0; AX_X.steps_remaining=c.pasos;
  AX_X.running=true; AX_X.dirFwd=(c.dir!=0);
  AX_X.until_limit=false; AX_X.towardHF=false;

  AX_X.period_us=(uint32_t)(1000000.0f/f0);
  timer_set_period(tx, AX_X.period_us);
  timerAlarmEnable(tx);

  x_busy=true;
  while(AX_X.running){
    uint32_t k=AX_X.steps_done; float f=f0;
    if(k<n_up)                 f = f0 + (fC-f0)*((float)k/(float)max(1u,n_up));
    else if(k<n_up+n_cru)      f = fC;
    else { uint32_t kd=k-n_up-n_cru; f = fC - (fC-f0)*((float)kd/(float)max(1u,n_dn)); }
    if(f<MIN_HZ) f=MIN_HZ; if(f>MAX_HZ) f=MAX_HZ;

    uint32_t per=(uint32_t)(1000000.0f/f);
    if(per!=AX_X.period_us){ AX_X.period_us=per; timer_set_period(tx, per); }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  x_busy=false; x_lastEmitted += AX_X.steps_done;
}

static void runMoveExactY(const MoveCmd& c){
  if(c.pasos==0) return;
  uint32_t hzC=constrain(c.hz,MIN_HZ,MAX_HZ);
  float acc_s=max(1u,c.acc_ms)/1000.0f, dec_s=max(1u,c.dec_ms)/1000.0f;

  float S_up=0.5f*hzC*acc_s, S_dn=0.5f*hzC*dec_s;
  if(c.pasos < (uint32_t)(S_up+S_dn)){
    float peak=(2.0f*c.pasos)/(acc_s+dec_s);
    hzC=constrain((uint32_t)peak,MIN_HZ,MAX_HZ);
    S_up=0.5f*hzC*acc_s; S_dn=0.5f*hzC*dec_s;
  }
  uint32_t n_up=(uint32_t)lroundf(S_up);
  uint32_t n_dn=(uint32_t)lroundf(S_dn);
  uint32_t n_cru=(c.pasos>(n_up+n_dn))?(c.pasos-n_up-n_dn):0;
  const float f0=(float)MIN_HZ, fC=(float)hzC;

  if(!ENA_ALWAYS_ON){ _act(Y_ENA,true); vTaskDelay(pdMS_TO_TICKS(200)); }
  _dir_fast(Y_DIR, c.dir!=0);

  AX_Y.steps_done=0; AX_Y.steps_remaining=c.pasos;
  AX_Y.running=true; AX_Y.dirFwd=(c.dir!=0);

  AX_Y.period_us=(uint32_t)(1000000.0f/f0);
  timer_set_period(ty, AX_Y.period_us);
  timerAlarmEnable(ty);

  y_busy=true;
  while(AX_Y.running){
    uint32_t k=AX_Y.steps_done; float f=f0;
    if(k<n_up) f = f0 + (fC-f0)*(float)k/(float)max(1u,n_up);
    else if(k<n_up+n_cru) f = fC;
    else { uint32_t kd=k-n_up-n_cru; f = fC - (fC-f0)*(float)kd/(float)max(1u,n_dn); }
    if(f<MIN_HZ) f=MIN_HZ; if(f>MAX_HZ) f=MAX_HZ;

    uint32_t per=(uint32_t)(1000000.0f/f);
    if(per!=AX_Y.period_us){ AX_Y.period_us=per; timer_set_period(ty, per); }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  y_busy=false;
}

static void runMoveExactZ(const MoveCmd& c){
  if(c.pasos==0) return;
  uint32_t hzC=constrain(c.hz,MIN_HZ,MAX_HZ);
  float acc_s=max(1u,c.acc_ms)/1000.0f, dec_s=max(1u,c.dec_ms)/1000.0f;

  float S_up=0.5f*hzC*acc_s, S_dn=0.5f*hzC*dec_s;
  if(c.pasos < (uint32_t)(S_up+S_dn)){
    float peak=(2.0f*c.pasos)/(acc_s+dec_s);
    hzC=constrain((uint32_t)peak,MIN_HZ,MAX_HZ);
    S_up=0.5f*hzC*acc_s; S_dn=0.5f*hzC*dec_s;
  }
  uint32_t n_up=(uint32_t)lroundf(S_up);
  uint32_t n_dn=(uint32_t)lroundf(S_dn);
  uint32_t n_cru=(c.pasos>(n_up+n_dn))?(c.pasos-n_up-n_dn):0;
  const float f0=(float)MIN_HZ, fC=(float)hzC;

  if(!ENA_ALWAYS_ON){ _act(Z_ENA,true); vTaskDelay(pdMS_TO_TICKS(200)); }
  _dir_fast(Z_DIR, c.dir!=0);

  AX_Z.steps_done=0; AX_Z.steps_remaining=c.pasos;
  AX_Z.running=true; AX_Z.dirFwd=(c.dir!=0);

  AX_Z.period_us=(uint32_t)(1000000.0f/f0);
  timer_set_period(tz, AX_Z.period_us);
  timerAlarmEnable(tz);

  z_busy=true;
  while(AX_Z.running){
    uint32_t k=AX_Z.steps_done; float f=f0;
    if(k<n_up) f = f0 + (fC-f0)*(float)k/(float)max(1u,n_up);
    else if(k<n_up+n_cru) f = fC;
    else { uint32_t kd=k-n_up-n_cru; f = fC - (fC-f0)*(float)kd/(float)max(1u,n_dn); }
    if(f<MIN_HZ) f=MIN_HZ; if(f>MAX_HZ) f=MAX_HZ;

    uint32_t per=(uint32_t)(1000000.0f/f);
    if(per!=AX_Z.period_us){ AX_Z.period_us=per; timer_set_period(tz, per); }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  z_busy=false;
}

// ===== HOME X con timer (hasta límite) =====
static uint32_t x_runUntilLimit(bool towardHF, uint32_t hz){
  uint32_t per=(uint32_t)(1000000.0f/constrain(hz,MIN_HZ,MAX_HZ));
  if(!ENA_ALWAYS_ON){ _act(X_ENA,true); vTaskDelay(pdMS_TO_TICKS(200)); }
  _dir_fast(X_DIR, towardHF);

  AX_X.steps_done=0; AX_X.steps_remaining=0;
  AX_X.running=true; AX_X.dirFwd=towardHF;
  AX_X.until_limit=true; AX_X.towardHF=towardHF;

  AX_X.period_us=per;
  timer_set_period(tx, per);
  timerAlarmEnable(tx);

  while(AX_X.running) vTaskDelay(1);
  return AX_X.steps_done;
}

inline bool hasHome(){ return (x_span_steps>0 && x_steps_per_mm>0.0f); }
void doHOME(){
  (void)x_runUntilLimit(false,5000); x_pos_steps=0;
  uint32_t span=x_runUntilLimit(true,5000); x_span_steps=span;
  x_steps_per_mm = (x_mm_total>0 && x_span_steps>0)? ((float)x_span_steps/x_mm_total):0.0f;
  MoveCmd back{ x_span_steps, 20000, 1,1, 0 }; runMoveExactX(back);
  x_pos_steps=0;
}

// ===== Encoladores / STOP =====
inline void x_enqueue(const MoveCmd& m){ xQueueSend(qX,&m,0); }
inline void y_enqueue(const MoveCmd& m){ xQueueSend(qY,&m,0); }
inline void z_enqueue(const MoveCmd& m){ xQueueSend(qZ,&m,0); }
inline void x_stopNow(){ AX_X.running=false; timerAlarmDisable(tx); }
inline void y_stopNow(){ AX_Y.running=false; timerAlarmDisable(ty); }
inline void z_stopNow(){ AX_Z.running=false; timerAlarmDisable(tz); }

// ===== Tareas movimiento =====
void TaskX(void*){
  for(;;){
    MoveCmd m; if(xQueueReceive(qX,&m,portMAX_DELAY)!=pdTRUE) continue;
    if(m.pasos==0 && m.hz==0 && m.acc_ms==0 && m.dec_ms==0xFFFFFFFF){ x_stopNow(); continue; }
    runMoveExactX(m);
  }
}
void TaskY(void*){
  for(;;){
    MoveCmd m; if(xQueueReceive(qY,&m,portMAX_DELAY)!=pdTRUE) continue;
    if(m.pasos==0 && m.hz==0 && m.acc_ms==0 && m.dec_ms==0xFFFFFFFF){ y_stopNow(); continue; }
    runMoveExactY(m);
  }
}
void TaskZ(void*){
  for(;;){
    MoveCmd m; if(xQueueReceive(qZ,&m,portMAX_DELAY)!=pdTRUE) continue;
    if(m.pasos==0 && m.hz==0 && m.acc_ms==0 && m.dec_ms==0xFFFFFFFF){ z_stopNow(); continue; }
    runMoveExactZ(m);
  }
}

// ===== Nextion helpers mínimos (opcionales) =====
static inline void nxSend(const String& cmd){ nx.print(cmd); nx.write(0xFF); nx.write(0xFF); nx.write(0xFF); }
static inline void drainUntilIdle(uint32_t quiet_ms=60){
  uint32_t t=millis(); for(;;){ while(nx.available()){ nx.read(); t=millis(); } if(millis()-t>quiet_ms) break; delay(1); }
}
int nxGetPageOnce(uint32_t timeout_ms=600){
  drainUntilIdle(); nxSend("sendme");
  uint32_t t0=millis(); bool saw66=false; int page=-1; int ff=0;
  while(millis()-t0<timeout_ms){
    if(!nx.available()) continue; int b=nx.read(); if(b<0) continue; uint8_t u=b;
    if(!saw66){ if(u==0x66){ saw66=true; continue; } else continue; }
    else if(page<0){ page=u; continue; }
    else{ if(u==0xFF){ if(++ff==3) return page; } else ff=0; }
  } return -1;
}
void nxUpdateButtons(){
  if(!hasHome()){ nxSend("vis Homebutton,1"); nxSend("vis Startbutton,0"); }
  else          { nxSend("vis Homebutton,0"); nxSend("vis Startbutton,1"); }
}

// ===== Parser PC =====
void pcHandle(const String& s){
  if(s.startsWith("MOVEX")){
    uint32_t p,h,a,d,dr; if(sscanf(s.c_str(),"MOVEX %u %u %u %u %u",&p,&h,&a,&d,&dr)==5){ x_enqueue({p,h,a,d,(uint8_t)dr}); Serial.println(F("OK MOVEX")); }
    else Serial.println(F("ERR MOVEX"));
  }else if(s.startsWith("MOVEY")){
    uint32_t p,h,a,d,dr; if(sscanf(s.c_str(),"MOVEY %u %u %u %u %u",&p,&h,&a,&d,&dr)==5){ y_enqueue({p,h,a,d,(uint8_t)dr}); Serial.println(F("OK MOVEY")); }
    else Serial.println(F("ERR MOVEY"));
  }else if(s.startsWith("MOVEZ")){
    uint32_t p,h,a,d,dr; if(sscanf(s.c_str(),"MOVEZ %u %u %u %u %u",&p,&h,&a,&d,&dr)==5){ z_enqueue({p,h,a,d,(uint8_t)dr}); Serial.println(F("OK MOVEZ")); }
    else Serial.println(F("ERR MOVEZ"));
  }else if(s=="STOPX"){ x_stopNow(); Serial.println(F("OK STOPX")); }
  else if(s=="STOPY"){ y_stopNow(); Serial.println(F("OK STOPY")); }
  else if(s=="STOPZ"){ z_stopNow(); Serial.println(F("OK STOPZ")); }
  else if(s=="HOME"){ doHOME(); }
  else if(s.startsWith("POS")){
    float mm; if(sscanf(s.c_str(),"POS %f",&mm)==1){
      if(!hasHome()){ Serial.println(F("ERR: haga HOME primero")); return; }
      if(mm<0) mm=0; if(mm>x_mm_total) mm=x_mm_total;
      int64_t tgt=(int64_t)lroundf(mm*x_steps_per_mm);
      int64_t d=tgt - x_pos_steps; if(d==0){ Serial.println(F("OK POS ya en destino")); return; }
      MoveCmd m{ (uint32_t)llabs(d), POS_HZ, POS_ACC_MS, POS_DEC_MS, (uint8_t)(d>0?1:0) };
      x_enqueue(m);
    } else Serial.println(F("ERR POS"));
  }else if(s.startsWith("MMSET")){
    float mm; if(sscanf(s.c_str(),"MMSET %f",&mm)==1 && mm>0){ x_mm_total=mm; if(x_span_steps) x_steps_per_mm=(float)x_span_steps/x_mm_total; Serial.println(F("OK MMSET")); }
    else Serial.println(F("ERR MMSET"));
  }else{
    Serial.println(F("CMDS: MOVEX|MOVEY|MOVEZ|STOPX|STOPY|STOPZ|HOME|POS <mm>|MMSET <mm_total>|STAT"));
  }
}

// ===== UI task =====
void TaskUI(void*){
  nx.begin(NX_BAUD, SERIAL_8N1, NX_RX, NX_TX);
  drainUntilIdle(); (void)nxGetPageOnce(); nxUpdateButtons();
  for(;;){
    if(millis()-lastLastSeenMs>200 && (x_lastEmitted1!=x_lastEmitted)){
      lastLastSeenMs=millis(); x_lastEmitted1=x_lastEmitted;
      int32_t pos_mm = (x_span_steps && x_steps_per_mm>0.0f)? (int32_t)lroundf((float)x_pos_steps/x_steps_per_mm) : -1;
      nxSend(String("tPos.val=")+pos_mm);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ===== Setup / Loop =====
void timersInit(){
  AX_X.pinSTEP=X_STEP; AX_X.pinDIR=X_DIR; AX_X.pinENA=X_ENA; AX_X.period_us=1000; AX_X.steps_remaining=0; AX_X.steps_done=0; AX_X.running=false; AX_X.dirFwd=false; AX_X.until_limit=false; AX_X.towardHF=false;
  AX_Y.pinSTEP=Y_STEP; AX_Y.pinDIR=Y_DIR; AX_Y.pinENA=Y_ENA; AX_Y.period_us=1000; AX_Y.steps_remaining=0; AX_Y.steps_done=0; AX_Y.running=false; AX_Y.dirFwd=false; AX_Y.until_limit=false; AX_Y.towardHF=false;
  AX_Z.pinSTEP=Z_STEP; AX_Z.pinDIR=Z_DIR; AX_Z.pinENA=Z_ENA; AX_Z.period_us=1000; AX_Z.steps_remaining=0; AX_Z.steps_done=0; AX_Z.running=false; AX_Z.dirFwd=false; AX_Z.until_limit=false; AX_Z.towardHF=false;

  tx=timerBegin(0,80,true); ty=timerBegin(1,80,true); tz=timerBegin(2,80,true);
  timerAttachInterrupt(tx,&onTimerX,true);
  timerAttachInterrupt(ty,&onTimerY,true);
  timerAttachInterrupt(tz,&onTimerZ,true);
  timerAlarmWrite(tx,1000,true); timerAlarmWrite(ty,1000,true); timerAlarmWrite(tz,1000,true);
  timerAlarmDisable(tx); timerAlarmDisable(ty); timerAlarmDisable(tz);
}

void setup(){
  setCpuFrequencyMhz(240);
  Serial.begin(115200);
  esp_task_wdt_init(60,true);

  auto prepOut=[&](int pin){ pinMode(pin,OUTPUT); gpio_set_direction((gpio_num_t)pin,GPIO_MODE_OUTPUT); gpio_set_drive_capability((gpio_num_t)pin,GPIO_DRIVE_CAP_3); };
  prepOut(X_STEP); prepOut(X_DIR); prepOut(X_ENA);
  prepOut(Y_STEP); prepOut(Y_DIR); prepOut(Y_ENA);
  prepOut(Z_STEP); prepOut(Z_DIR); prepOut(Z_ENA);
  pinMode(X_H0,INPUT_PULLUP); pinMode(X_HF,INPUT_PULLUP);

  _act(X_STEP,false); _act(X_DIR,false); _act(X_ENA,false);
  _act(Y_STEP,false); _act(Y_DIR,false); _act(Y_ENA,false);
  _act(Z_STEP,false); _act(Z_DIR,false); _act(Z_ENA,false);

  timersInit();

  qX=xQueueCreate(8,sizeof(MoveCmd));
  qY=xQueueCreate(8,sizeof(MoveCmd));
  qZ=xQueueCreate(8,sizeof(MoveCmd));

  xTaskCreatePinnedToCore(TaskX,"MotionX",4096,NULL, configMAX_PRIORITIES-1, &tX, 0);
  xTaskCreatePinnedToCore(TaskY,"MotionY",4096,NULL, configMAX_PRIORITIES-1, &tY, 0);
  xTaskCreatePinnedToCore(TaskZ,"MotionZ",4096,NULL, configMAX_PRIORITIES-1, &tZ, 0);
  xTaskCreatePinnedToCore(TaskUI,"UI",4096,NULL,1,&tUI,1);

#if ENA_ALWAYS_ON
  _act(X_ENA,true); _act(Y_ENA,true); _act(Z_ENA,true);
#endif

  Serial.println(F("Ready. CMDS: MOVEX|MOVEY|MOVEZ|STOPX|STOPY|STOPZ|HOME|POS <mm>|MMSET <mm_total>|STAT"));
}

void loop(){ // comandos por Serial
  static String buf;
  while(Serial.available()){
    char c=(char)Serial.read();
    if(c=='\n'||c=='\r'){ buf.trim(); if(buf.startsWith("MOVEX")||buf.startsWith("MOVEY")||buf.startsWith("MOVEZ")||buf=="STOPX"||buf=="STOPY"||buf=="STOPZ"||buf=="HOME"||buf.startsWith("POS")||buf.startsWith("MMSET")) pcHandle(buf); buf=""; }
    else buf+=c;
  }
  vTaskDelay(pdMS_TO_TICKS(10));
}