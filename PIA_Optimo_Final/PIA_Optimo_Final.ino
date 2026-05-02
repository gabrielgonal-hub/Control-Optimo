// ================= PINES =================
#define X2_1 33
#define X2_2 32
#define X1_3 35
#define X2_3 34
#define X1_4 39
#define X2_4 36

#define REF_PIN 25

#define PWM1 13
#define PWM2 14
#define PWM3 27
#define PWM4 26

#define BTN_ON 16
#define BTN_OFF 17

// ================= PWM =================
#define PWM_FREQ 5000
#define PWM_RES 8

// ================= PLANTA =================
#define R1 1000000
#define R2 1000000
#define R3 1000000
#define C1 0.000001
#define C2 0.000001

#define mX (3.3/4095.0)
#define mU (255.0/3.3)

// ================= TIEMPO =================
unsigned long TS = 50;
float Tseg = 0.05;
unsigned long TIC = 0, TS_code = 0, TC = 0;

// ================= VARIABLES =================
float R = 0;
bool Habilitado = 0;

// ===== MODELO =====
float Am11, Am12, Am21, Am22, Bm1, Bm2;

// ================= SISTEMA 1 (POLOS) =================
float Xe1_1=0, Xe2_1=0;
float K1_1, K2_1, Kp_1;
float H1_1, H2_1;
float U1=0, Y1=0;

// ================= SISTEMA 2 (INTEGRAL POLOS) =================
float Xe1_2=0, Xe2_2=0, X0_2=0;
float K0_2, K1_2, K2_2;
float H1_2, H2_2;
float U2=0, Y2=0;

// ================= SISTEMA 3 (LQR) =================
float K1_3=0.41421356;
float K2_3=0.41421356;
float Kp_3;
float X13=0, X23=0;
float U3=0, Y3=0;

// ================= SISTEMA 4 (LQI) =================
float K0_4=3.16227766;
float K1_4=0.41495869;
float K2_4=0.41601274;
float X0_4=0;
float X14=0, X24=0;
float U4=0, Y4=0;

// =========================================================
// ======================= SETUP ============================
// =========================================================
void setup(){
  Serial.begin(115200);

  pinMode(BTN_ON, INPUT_PULLUP);
  pinMode(BTN_OFF, INPUT_PULLUP);

  // PWM
  ledcSetup(0, PWM_FREQ, PWM_RES);
  ledcSetup(1, PWM_FREQ, PWM_RES);
  ledcSetup(2, PWM_FREQ, PWM_RES);
  ledcSetup(3, PWM_FREQ, PWM_RES);

  ledcAttachPin(PWM1,0);
  ledcAttachPin(PWM2,1);
  ledcAttachPin(PWM3,2);
  ledcAttachPin(PWM4,3);

  // ===== MODELO =====
  float p11 = R1*C1;
  float p12 = R1*C2;
  float p21 = R2*C1;
  float p22 = R2*C2;
  float p32 = R3*C2;

  Am11 = -1/p11 - 1/p21;
  Am12 = 1/p21;
  Am21 = 1/p22;
  Am22 = -1/p22 - 1/p32;
  Bm1 = 1/p11;
  Bm2 = 0;

  // =====================================================
  // SISTEMA 1 (ASIGNACIÓN DE POLOS)
  // =====================================================
  float a1r=-1.25, a1i=-1.7320508;   //polos controlador
  float a2r=-1.25, a2i=1.7320508;
  float b1r=-10, b2r=-10;            //polos observador

  float sum_a = -(a1r + a2r);
  float prod_a = a1r*a2r - a1i*a2i;
  float sum_b = -(b1r + b2r);
  float prod_b = b1r*b2r;


  //Ganancias de controlador
  K1_1 = p11*(sum_a - 1/p11 - 1/p21 - 1/p22 - 1/p32);
  K2_1 = p12*p22*(prod_a - 1/(p11*p22) - 1/(p11*p32) - 1/(p21*p22) - 1/(p21*p32)
          - K1_1/(p11*p22) - K1_1/(p11*p32) + 1/(p21*p22));
  Kp_1 = K1_1 + K2_1 + 1 + (K1_1*R2+R1+R2)/R3;  //Ganancia de seguimiento mediante cambio de coordenadas


  //Observador de Luenberger
  H2_1 = sum_b - 1/p11 - 1/p21 - 1/p22 - 1/p32;
  H1_1 = p22*(prod_b - 1/(p11*p22) -1/(p11*p32) - 1/(p21*p22)
          - 1/(p21*p32) - H2_1/p11 - H2_1/p21 + 1/(p21*p22));

  // =====================================================
  // SISTEMA 2 (INTEGRAL)
  // =====================================================
  float a3r=-1;  //polo integral

  float sum_ai = -(a1r + a2r + a3r);
  float prod_ai = -(a1r*a2r - a1i*a2i)*a3r;

  //Ganancias de controlador
  K0_2 = p11*p22*prod_ai;
  K1_2 = p11*(sum_ai - 1/p11 - 1/p21 - 1/p22 - 1/p32);
  K2_2 = p11*(p22*(a1r*a3r + a2r*a3r + a1r*a2r)
          - 1/(p21*p32) - (1+K1_2)/(p11*p32)) - 1 - K1_2;

  //Mismo observador de Luenberger
  H2_2 = H2_1;
  H1_2 = H1_1;

  // =====================================================
  // SISTEMA 3 (LQR)
  // =====================================================
  Kp_3 = K1_3 + K2_3 + 1 + (K1_3*R2+R1+R2)/R3;  //Ganancia de seguimiento mediante cambio de coordenadas

}

// =========================================================
// ======================== LOOP ============================
// =========================================================
void loop(){

  if(digitalRead(BTN_ON) == LOW) Habilitado=1;
  else if(digitalRead(BTN_OFF) == LOW) Habilitado=0;

  R = Habilitado*(analogRead(REF_PIN)*mX);  //Referencia

  control_1();
  control_2();
  control_3();
  control_4();

  // ===== PWM =====
  ledcWrite(0, U1*mU);
  ledcWrite(1, U2*mU);
  ledcWrite(2, U3*mU);
  ledcWrite(3, U4*mU);

  // coms_arduino_ide();

  coms_python(&R,
              &Xe1_1,&Y1,&U1,  //sistema 1
              &Xe1_2,&Y2,&U2,  //sistema 2
              &X13,&Y3,&U3,    //sistema 3
              &X14,&Y4,&U4);   //sistema 4

  espera();
}

// ===== Control por Asignación de polos =====
void control_1(){
  Y1 = analogRead(X2_1)*mX;  //estado 2 y salida
  float e1 = Y1 - Xe2_1;

  float f1_1 = Am11*Xe1_1 + Am12*Xe2_1 + Bm1*U1 + H1_1*e1;  //Dinamica estado 1 observador
  float f2_1 = Am21*Xe1_1 + Am22*Xe2_1 + H2_1*e1;           //Dinamica estado 2 observador

  Xe1_1 += Tseg*f1_1;  //Integrador estado 1 mediante Euler
  Xe2_1 += Tseg*f2_1;  //Integrador estado 2 mediante Euler
  
  //Ley de control u = -Kx
  U1 = Kp_1*R - K1_1*Xe1_1 - K2_1*Xe2_1;
  
  U1 = constrain(U1,0,3.3);  //saturacion
}

// ===== Control Integral por Asignación de polos =====
void control_2(){
  Y2 = analogRead(X2_2)*mX;  //estado 2 y salida

  //observador
  float e2 = Y2 - Xe2_2;

  float f1_2 = Am11*Xe1_2 + Am12*Xe2_2 + Bm1*U2 + H1_2*e2;  //Dinamica estado 1 observador
  float f2_2 = Am21*Xe1_2 + Am22*Xe2_2 + H2_2*e2;           //Dinamica estado 2 observador

  Xe1_2 += Tseg*f1_2;      //Integrador estado 1 mediante Euler
  Xe2_2 += Tseg*f2_2;      //Integrador estado 2 mediante Euler
  X0_2 += Tseg*(R - Y2);  //estado 0
  
  //Ley de control u = -Kx
  U2 = K0_2*X0_2 - K1_2*Xe1_2 - K2_2*Xe2_2;
  
  U2 = constrain(U2,0,3.3);  //saturación
}

// ===== Control LQR =====
void control_3(){
  X13 = analogRead(X1_3)*mX;  //estado 1
  X23 = analogRead(X2_3)*mX;  //estado 2
  Y3 = X23;  //salida
  //Ley de control u = -Kx
  U3 = Kp_3*R - K1_3*X13 - K2_3*X23;

  U3 = constrain(U3,0,3.3);  //saturacion
}

// ===== Control LQI =====
void control_4(){
  X14 = analogRead(X1_4)*mX;  //estado 1
  X24 = analogRead(X2_4)*mX;  //estado 2
  Y4 = X24;  //salida

  X0_4 += Tseg*(R - Y4);  //estado 0
  
  //Ley de control u = -Kx
  U4 = K0_4*X0_4 - K1_4*X14 - K2_4*X24;

  U4 = constrain(U4,0,3.3);  //saturacion
}

//Muestreo uniforme
void espera(){   
  TS_code = millis()- TIC;
  TC = TS - TS_code;
  if (TS_code < TS) delay(TC);
  TIC = millis();
}

//Comunicación con serial monitor y serial plotter
void coms_arduino_ide(){

    Serial.print("R:"); Serial.print(R);  //referencia

    // Serial.print(" | Y1:"); Serial.print(Y1); Serial.print(" U1:"); Serial.print(U1);  //sistema 1
    Serial.print(" | Y2:"); Serial.print(Y2); Serial.print(" U2:"); Serial.print(U2);     //sistema 2
    Serial.print(" | Y3:"); Serial.print(Y3); Serial.print(" U3:"); Serial.print(U3);     //sistema 3
    Serial.print(" | Y4:"); Serial.print(Y4); Serial.print(" U4:"); Serial.println(U4);   //sistema 4
}

//Comunicación con monitor de pyhton
void coms_python(float* Rp,
                 float* X11,float* Y1p, float* U1p,
                 float* X12,float* Y2p, float* U2p,
                 float* X13,float* Y3p, float* U3p,
                 float* X14,float* Y4p, float* U4p)
{
  float t = TIC/1000.0;

  byte* tB  = (byte*)(&t);  //tiempo
  byte* RB  = (byte*)(Rp);  //referencia

  byte* X1B1 = (byte*)(X11);  //estado 1 sistema 1 AdP
  byte* Y1B  = (byte*)(Y1p);  //salida sistema 1 AdP
  byte* U1B  = (byte*)(U1p);  //control sistema 1 AdP

  byte* X1B2 = (byte*)(X12);  //estado 1 sistema 2 Integral AdP
  byte* Y2B  = (byte*)(Y2p);  //salida sistema 2 Integral AdP
  byte* U2B  = (byte*)(U2p);  //control sistema 2 Integral AdP

  byte* X1B3 = (byte*)(X13);  //estado 1 sistema 3 LQR
  byte* Y3B  = (byte*)(Y3p);  //salida sistema 3 LQR
  byte* U3B  = (byte*)(U3p);  //control sistema 3 LQR

  byte* X1B4 = (byte*)(X14);  //estado 1 sistema 4 LQR
  byte* Y4B  = (byte*)(Y4p);  //salida sistema 4 LQR
  byte* U4B  = (byte*)(U4p);  //control sistema 4 LQR

  byte buf[56] = {
    tB[0],tB[1],tB[2],tB[3],  //tiempo
    RB[0],RB[1],RB[2],RB[3],  //Referencia

    X1B1[0],X1B1[1],X1B1[2],X1B1[3],  //estado 1 sistema 1 AdP
    Y1B[0],Y1B[1],Y1B[2],Y1B[3],      //salida sistema 1 AdP
    U1B[0],U1B[1],U1B[2],U1B[3],      //control sistema 1 AdP

    X1B2[0],X1B2[1],X1B2[2],X1B2[3],  //estado 1 sistema 2 Integral AdP
    Y2B[0],Y2B[1],Y2B[2],Y2B[3],      //salida sistema 2 Integral AdP
    U2B[0],U2B[1],U2B[2],U2B[3],      //control sistema 2 Integral AdP

    X1B3[0],X1B3[1],X1B3[2],X1B3[3],  //estado 1 sistema 3 LQR
    Y3B[0],Y3B[1],Y3B[2],Y3B[3],      //salida sistema 3 LQR
    U3B[0],U3B[1],U3B[2],U3B[3],      //control sistema 3 LQR

    X1B4[0],X1B4[1],X1B4[2],X1B4[3],  //estado 1 sistema 4 LQR
    Y4B[0],Y4B[1],Y4B[2],Y4B[3],      //salida sistema 4 LQR
    U4B[0],U4B[1],U4B[2],U4B[3]       //control sistema 4 LQR
  };

  Serial.write(buf, 56);
}
