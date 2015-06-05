#include "Wire.h"

#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 mpu;
#define GyroMeasError PI * (40.0f / 180.0f)       // gyroscope measurement error in rads/s (shown as 3 deg/s)
#define GyroMeasDrift PI * (0.0f / 180.0f)

float biasACCX,biasACCY,biasACCZ;
float biasGX,biasGY,biasGZ;
float pitch, yaw, roll;
float deltat = 0.0f;        // integration interval for both filter schemes
uint16_t lastUpdate = 0; // used to calculate integration interval
uint16_t now = 0;        // used to calculate integration interval
float ACC[3];
float w[3]; // corrected spin velocity
float MAG[3]; // north vector

uint16_t count = 0;  
uint16_t delt_t = 0; 
uint16_t mcount = 0;

float p_is[4] = {0.0, 0.0, 0.0, 0.0};
float p_ig[4] = {0.0, 0.0, 1.0, 0.0};
float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
float q[4] = {0.0f, 0.0f, 0.0f, 1.0f};    // vector to hold quaternion


int q_print[4];
int16_t a1, a2, a3, g1, g2, g3, m1, m2, m3;     // raw data arrays reading

uint8_t MagRate;     // read rate for magnetometer data

#define LED_PIN 13
bool blinkState = false;

void setup() {
    
    Wire.begin();
    Serial.begin(9600);

  
    //Serial.println("Initializing I2C devices...");
    mpu.initialize();

    
    //Serial.println("Testing device connections...");
    //Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    mpu.setRate(7); // set gyro rate to 8 kHz/(1 * rate) shows 1 kHz, accelerometer ODR is fixed at 1 KHz

    MagRate = 10; // set magnetometer read rate in Hz; 10 to 100 (max) Hz are reasonable values
    mpu.setDLPFMode(4);
    mpu.setFullScaleGyroRange(0);
    mpu.setFullScaleAccelRange(0);
    mpu.setIntDataReadyEnabled(true);
    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);
}

void loop() {
     if(mpu.getIntDataReadyStatus() == 1) { // wait for data ready status register to update all data registers
            mcount++;
           // read the raw sensor data
            
            mpu.getAcceleration  ( &a1, &a2, &a3  );
            ax = a1*2.0f/32768.0f; // 2 g full range for accelerometer
            ay = a2*2.0f/32768.0f;
            az = a3*2.0f/32768.0f;
           
            
            mpu.getRotation  ( &g1, &g2, &g3  );
            gx = g1*250.0f/32768.0f; // 250 deg/s full range for gyroscope
            gy = g2*250.0f/32768.0f;
            gz = g3*250.0f/32768.0f;
            
            for (int i =0; i++; i<100){
            biasACCX =+ax;
            biasACCY =+ay;
            biasACCZ =+az;
            biasGX =+gx;
            biasGY =+gy;
            biasGZ =+gz;
            }
            biasACCX = biasACCX/100;
            biasACCY = biasACCY/100;
            
            biasGX = biasGX/100;
            biasGY = biasGY/100;
            biasGZ = biasGZ/100;
            ACC[0] = 9.81*(ax-biasACCX+0.07);
            ACC[1] = 9.81*(ay-biasACCY-0.02);
            ACC[2] = 9.81*(az);
            w[0] = (gx-biasGX)*PI/180.0f;
            w[1] = (gy-biasGY-1)*PI/180.0f;
            w[2] = (gz-biasGZ-0.7)*PI/180.0f;

            if (mcount > 1000/MagRate) {  
            mpu.getMag  ( &m1, &m2, &m3 );
            mx = m1*10.0f*1229.0f/4096.0f -135.0f; // milliGauss (1229 microTesla per 2^12 bits, 10 mG per microTesla)
            my = m2*10.0f*1229.0f/4096.0f + 219.0f; // apply calibration offsets in mG that correspond to your environment and magnetometer
            mz = m3*10.0f*1229.0f/4096.0f + 54.0f;
            MAG[0]=my;
            MAG[1]=mx;
            MAG[2]=mz;
            mcount = 0;
             }
           for (int i = 0; i < 100; i += 1) {
            p_is[0] =p_is[0]+ my;
            p_is[1] = p_is[1]+mx;
            p_is[2] = mz; 
          }
          p_is[0]/=100.0;
          p_is[1]/=100.0;
          p_is[2]/=100.0;
          p_is[0] = sqrt(p_is[0]*p_is[0] + p_is[1]*p_is[1])*cos(0/180*PI); // 3.23 - inclination angle
          p_is[1] = -sqrt(p_is[0]*p_is[0] + p_is[1]*p_is[1])*sin(0/180*PI);
          VNormalize(p_is);          
         }
   
  now = micros();
  deltat = ((now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = now;
  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
  // the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
 
    Kalman3D(q);
    // Serial print and/or display at 0.5 s rate independent of data rates
    delt_t = millis() - count;
    if (delt_t > 500) { // update LCD once per half-second independent of read rate
 /*
    Serial.print("ax = "); Serial.print(9.81*(ax-biasACCX));  
    Serial.print(" ay = "); Serial.print(9.81*(ay-biasACCX)); 
    Serial.print(" az = "); Serial.print(9.81*(az+0.04)); Serial.println(" mg");
                      
    Serial.print("gx = "); Serial.print( gx-biasGX); 
    Serial.print(" gy = "); Serial.print( gy-biasGY-1); 
    Serial.print(" gz = "); Serial.print( gz-biasGZ-0.7); Serial.println(" deg/s");

    Serial.print("mx = "); Serial.print( (int)mx ); 
    Serial.print(" my = "); Serial.print( (int)my ); 
    Serial.print(" mz = "); Serial.print( (int)mz ); Serial.println(" mG");
    
    Serial.print("q0 = "); Serial.print(q[0]);
    Serial.print(" qx = "); Serial.print(q[1]); 
    Serial.print(" qy = "); Serial.print(q[2]); 
    Serial.print(" qz = "); Serial.println(q[3]); 
      */             
   
  
  // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
  // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
  
    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI - 3.66; // Declination at Zagreb is 3.66 
    roll  *= 180.0f / PI;
/*
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(yaw, 2);
    Serial.print(", ");
    Serial.print(pitch, 2);
    Serial.print(", ");
    Serial.println(roll, 2);
    
    Serial.print("rate = "); Serial.print((float)1.0f/deltat, 2); Serial.println(" Hz");
*/
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
     count = millis();
     serialPrintFloatArr(q, 4);
     Serial.print("\n");
  }
}
float Norm3(float *x)
{
    // ulazni parametar trodimenzionalni float
    return(sqrt(x[0]*x[0] + x[1]*x[1] + x[2]*x[2]));
}
float Norm4(float *x)
{
    // ulazni parametar ƒçetverodimenzionalni float
    return(sqrt(x[0]*x[0] + x[1]*x[1] + x[2]*x[2] + x[3]*x[3]));
}
void QNormalize(float *q)
{
    // normalizacija kvaterniona
    float norm = Norm4(q);
    for(int i=0; i<4; i++)
        q[i] /= norm;
}
void VNormalize(float *v)
{
    // normalizacija kvaterniona
    float norm = Norm3(v);
    for(int i=0; i<3; i++)
        v[i] /= norm;
}
void Qconjugate(float *q1, float *q)
{
    // q1 - ulazni kvateornion
    // q - konjugirani kvaternion
    q[0] = -q1[0];
    q[1] = -q1[1];
    q[2] = -q1[2];
    q[3] = q1[3];
}
void QMultiply(float *q1, float *q2, float *q)
{
    // q1, q2 - ulazni kvaternioni
    // q = q1*q2
    q[0] = q1[3]*q2[0] - q1[2]*q2[1] + q1[1]*q2[2] + q1[0]*q2[3];
    q[1] = q1[2]*q2[0] + q1[3]*q2[1] - q1[0]*q2[2] + q1[1]*q2[3];
    q[2] = -q1[1]*q2[0] + q1[0]*q2[1] + q1[3]*q2[2] + q1[2]*q2[3];
    q[3] = -q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] + q1[3]*q2[3];
}
void QuatPropagation(float *w, float *q_old, float T, float *q)
{
    //quaternion propagation
    // input params: w - angle velocity
    // q_old - last step quaternion
    // T - period
    // output parameter: q - new quaternion
    float W, c, s, n1, n2, n3;
    W = Norm3(w);
    c = cos(0.5*W*T);
    s = sin(0.5*W*T);
    if (W == 0)
    {
        n1 = 0;
        n2 = 0;
        n3 = 0;
    }
    else
    {
        n1 = w[0]/W;
        n2 = w[1]/W;
        n3 = w[2]/W;
    }
    q[0] = c*q_old[0] + s*(n3*q_old[1] - n2*q_old[2] + n1*q_old[3]);
    q[1] = c*q_old[1] + s*(-n3*q_old[0] + n1*q_old[2] + n2*q_old[3]);
    q[2] = c*q_old[2] + s*(n2*q_old[0] - n1*q_old[1] + n3*q_old[3]);
    q[3] = c*q_old[3] + s*(-n1*q_old[0] - n2*q_old[1] - n3*q_old[2]);
}

void DqMatrix(float *w, float T, float *Th)
{
    //calculation of matrices Th
    // input param: w - brzina vrtnje
    // T - period
    float W = Norm3(w);
    if (W == 0)
    {
        for(int i=0;i<3; i++)
            for(int j=0;j<3; j++)
            {
                if (i == j)
                    Th[i*3 + j] = 1;
                else
                    Th[i*3 + j] = 0;
            }
    }
    else
    {
        float s = sin(W*T)/W;
        float c = (1-cos(W*T))/(W*W);
        Th[0] = 1 - c*(w[1]*w[1] + w[2]*w[2]);
        Th[1] = s*w[2] + c*w[0]*w[1];
        Th[2] = c*w[0]*w[2] - s*w[1];
        Th[3] = c*w[0]*w[1] - s*w[2];
        Th[4] = 1 - c*(w[0]*w[0] + w[2]*w[2]);
        Th[5] = s*w[0] + c*w[1]*w[2];
        Th[6] = s*w[1] + c*w[0]*w[2];
        Th[7] = c*w[1]*w[2] - s*w[0];
        Th[8] = 1 - c*(w[0]*w[0] + w[1]*w[1]);
    }
}
void OutputMatrixSeq(float *q, int i, float *Hi)
{
    // calculation of i-row output matix of sequential Kalman filter matrice sekvencijalnog Kalmanovog filtra
    // input params: q - new quaternion value
    // i - filter iteration i = [1,6],
    // output parameter: Hi - i-row of output matrix
    float qv[4], qc[4], qpom[4];
    qv[3] = 0;
    if (i<3)
    {
        // qv - measured north vector quaternion
        qv[0] = p_is[0];
        qv[1] = p_is[1];
        qv[2] = p_is[2];
        Qconjugate(q, qc);
        QMultiply(qc, qv, qpom);
        QMultiply(qpom, q, qc);
    }
    else
    {
        //qv - measured gravity quaternion
        qv[0] = p_ig[0];
        qv[1] = p_ig[1];
        qv[2] = p_ig[2];
        Qconjugate(q, qc);
        QMultiply(qc, qv, qpom);
        QMultiply(qpom, q, qc);
    }
    
    i = i % 3; // i = [0,5]
    switch(i)
    {
        case 0:
            Hi[0] = 0;
            Hi[1] = -2*qc[2];
            Hi[2] = 2*qc[1];
            break;
        case 1:
            Hi[0] = 2*qc[2];
            Hi[1] = 0;
            Hi[2] = -2*qc[0];
            break;
        case 2:
            Hi[0] = -2*qc[1];
            Hi[1] = 2*qc[0];
            Hi[2] = 0;
            break;
        default:;
    }
}
void Matrix3x3Multiply(float *m1, float *m2, float *m)
{
    //  3x3 matrix multiplying
    // input params: m1, m2 - matrices dim. 3x3 as 1D array
    // output param: m - matrix dim. 3x3 as 1D array
    for(int i = 0; i<3; i++)
        for (int j = 0; j<3; j++)
        {
            m[i*3 + j] = 0;
            for(int k = 0 ; k<3; k++)
                m[i*3 + j] += m1[i*3 + k]*m2[k*3 + j];
        }
}
void Matrix3x3Transpose(float *m1, float *m)
{
    //  3x3 matrix transpose
    // input params: m1 - 3x3 matrix as 1D array
    // output params: m - transpose of m1 as 1D array
    float pom;
    for(int i=0; i<3; i++)
        for(int j=0; j <=i; j++)
        {
            if (i == j)
                m[i*3 +j] = m1[i*3 +j];
            else
            {
                m[i*3 + j] = m1[j*3 + i]; // m(i,j) = m1(j,i)
                m[j*3 + i] = m1[i*3 + j]; // m(j,i) = m1(i,j)
            }
        }
}
void Matrix3x3Copying(float *m1, float *m2)
{
    // copying input matrix m1 dim. 3x3 into m2 matrix 1D array
    for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
            m2[i*3 + j] = m1[i*3 + j];
}
void Vector1xNCopying(float *v1, int n, float *v2)
{
  
    for(int i=0; i<n; i++)
        v2[i] = v1[i];
}
void Kalman3D(float *q)
{   
    int i;
    //Kalman filter algorithm for obtaining 3D orientation in quaternion representation
    // output param: q - calculated quaternion
    static float q_old[4] = {0.0, 0.0, 0.0, 1.0}; // old quaternion value
    static unsigned long t_old = 0; // old time value, [ms]
    static float P_old[9] = {10.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 10.0}; //old covariance matrix
    static int k = 0;
    float rm = 100; // magnetometer noise variance
    float ra = 100; // accelerometer noise variance
    float nw = 1; // gyro noise variance
    float mag[3]; // normailzed north vector
    float akc[3]; // normailzed gravity vector
    float qi[4];
    float Th[9];
    float P[9];
    float P_pom1[9], P_pom2[9];
    float p_se[4], p_ge[4];
    float q_pom1[4], q_pom2[4];
    float T;
    T = (millis() - t_old)/1000.0;
    t_old = millis();
    Vector1xNCopying(ACC, 3, akc);
    VNormalize(akc);
    Vector1xNCopying(MAG, 3, mag);
    VNormalize(mag);
    
   
    // quaternion integration
    QuatPropagation(w, q_old, T, qi);
    QNormalize(qi);
    
    // quaternion error
    DqMatrix(w, T, Th);
    // P = Th*P_old_s*Th' + Q;
    Matrix3x3Transpose(Th, P_pom1);
    Matrix3x3Multiply(Th, P_old, P_pom2);
    Matrix3x3Multiply(P_pom2, P_pom1, P);
    P[0] += nw;
    P[4] += nw;
    P[8] += nw;
    // vectors estimate
    Qconjugate(qi, q_pom2); // q_pom2 = q_i*
    // p_se = qi*.p_is.qi
    
    QMultiply(q_pom2, p_is, q_pom1);
    QMultiply(q_pom1, qi, p_se);
    QNormalize(p_se);
    // p_ge = qi*.p_ig.qi
    QMultiply(q_pom2, p_ig, q_pom1);
    QMultiply(q_pom1, qi, p_ge);
    QNormalize(p_ge);
    //  q_pom2 = qi
    Vector1xNCopying(qi, 4, q_pom2);
     
    
    
    // sequential refreshing equations
    for( i=0; i<6; i++)
    {
        float Hi[3], dyi, Ri, Ki[3], dq[4];
        if (i<3)
        {
            Ri = rm;
            dyi = mag[i] - p_se[i];
        }
        else
        {
            Ri = ra;
            dyi = akc[i%3] - p_ge[i%3];
        }
        // i-row of output matrix
        OutputMatrixSeq(q_pom2, i, Hi);
        // Ki = P*Hi'/(Hi*P*Hi' + Ri)
        // P_pom1 = P*Hi'
        for(int j=0; j<3; j++)
        {
            P_pom1[j] = 0;
            for(int k=0; k<3; k++)
                P_pom1[j] += P[j*3 + k]*Hi[k];
        }
        // P_pom2 = Hi*P*Hi' = Hi*P_pom1
        P_pom2[0] = 0;
        for(int j=0; j<3; j++)
            P_pom2[0] += Hi[j]*P_pom1[j];
        // P_pom2 = Hi*P*Hi' + Ri = P_pom2 + Ri
        P_pom2[0] += Ri;
        // Ki = P_pom1/P_pom2
        // dq = Ki*dyi
        for(int j=0; j<3; j++)
        {
            Ki[j] = P_pom1[j]/P_pom2[0];
            dq[j] = Ki[j]*dyi;
        }
        dq[3] = 1;
        QMultiply(qi, dq, q_pom1);
        QNormalize(q_pom1);
        //qi = q_pom1
    
    
        //Vector1xNCopying(q_pom1, 4, qi);
    
        // P_pom1 = -Ki*Hi
        
        for(int j=0; j<3; j++)
            for(int k=0; k<3; k++)
                P_pom1[j*3+k] = -Ki[j]*Hi[k];
        //P_pom1 = eye(3) - Ki*Hi
        P_pom1[0] = 1 + P_pom1[0];
        P_pom1[4] = 1 + P_pom1[4];
        P_pom1[8] = 1 + P_pom1[8];
        //P_pom2 = (eye(3) - Ki*Hi)*P = P_pom1*P
        Matrix3x3Multiply(P_pom1, P, P_pom2);
        // P = P_pom2;
        //Matrix3x3Copying(P_pom2, P);
    }
    
    // P_old = P;
    
    Matrix3x3Copying(P_pom2, P_old);
    // q_old = q
    // q = qi
    // w_old = w;
    Vector1xNCopying(q_pom1, 4, q_old);
    Vector1xNCopying(q_pom1, 4, q);
    
}
void serialPrintFloatArr(float * arr, int length) {
  for(int i=0; i<length; i++) {
    serialFloatPrint(arr[i]);
    Serial.print(",");
  }
}


void serialFloatPrint(float f) {
  byte * b = (byte *) &f;
  for(int i=0; i<4; i++) {
    
    byte b1 = (b[i] >> 4) & 0x0f;
    byte b2 = (b[i] & 0x0f);
    
    char c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
    char c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;
    
    Serial.print(c1);
    Serial.print(c2);
  }
}  
