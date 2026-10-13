// #pragma once

// class PID{
// public:
//     PID(float ki, float kd, float kp) : ki(ki), kd(kd), kp(kp) {} 

//     double CalcPID(float datain){
//         // PID -> G = KD + KP + KI
//         float PrevError = 0.0f;
//         float i;
//         error = datain - error;

//         float p = kp * error;

//         i += error;

//         float d = (d + ) 
//     }

//     void reset(){
//         error = 0.0f;
//         kd = 0.0f;
//         kp = 0.0f;
//         ki = 0.0f;
//     }

// private:
//     float kd;
//     float ki;
//     float kp;
//     float N;
//     float error;

// };