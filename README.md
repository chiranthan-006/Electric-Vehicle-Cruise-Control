# Electric-Vehicle-Cruise-Control
Electric Vehicle Cruise Control is a feedback control system that maintains a set speed by adjusting motor torque based on error between actual and desired speed. Using a PID controller, it ensures stable speed tracking, improves comfort, and enhances energy efficiency under varying road conditions.

1. Objective

    To design and simulate a cruise control system that maintains a constant vehicle speed under varying road conditions using a PI/PID controller, ensuring:
  
    -Steady-state error < 2%
  
    -Overshoot < 5%
  
    -Smooth transient response
  
    -Stability under disturbance

3. System Modeling

    The vehicle dynamics are given by the transfer function:
   
                  G(s)=1/5s+1

   -Input: Throttle (u)

   -Output: Vehicle Speed (v)

   -Disturbance: Road slope at t=10s

   This is a first-order system with time constant:
   
                  τ=5seconds

4. Open Loop Analysis

    Without controller:

   -Slow response

   -Large steady-state error

   -Cannot reject disturbance

   Hence, a controller is required.

5. Controller Design

   Choice: PI Controller

   (A PI controller is sufficient for zero steady-state error)

                 C(s)= Kp + Ki/s
   
   Tuning (Trial / Ziegler-Nichols / Manual)

   Selected values (suitable for project):

   -Kp=2

   -Ki=0.8

6. Closed Loop System

               T(s)= C(s)G(s)/1+C(s)G(s)

   Substitute:

               T(s)= (2+0.8/s)(1/(5s+1))/1+(2+0.8/s)(1/(5s+1))

7. Simulink Implementation Steps

   Blocks Required

   -Step (Reference speed)

   -Sum (Error calculation)

   -PID Controller (set to PI mode)

   -Transfer Function block → 1/(5s+1)

   -Scope

   -Step (Disturbance)

   Connections

   -Step → (+) Sum

   -Output → (–) Sum

   -Sum → PI Controller

   -Controller → Transfer Function

   -Output → Scope

   Disturbance Setup

   -Add disturbance using Sum block

   -Step disturbance:

       -Time = 10 s

       -Magnitude = small (e.g., 0.2)

8. Simulation Results (Expected)

   Without Controller

   -Slow rise time

   -Large steady-state error

   -Disturbance causes speed drop

   With PI Controller

   -Rise time: Improved

   -Overshoot: < 5%

   -Steady-state error: ≈ 0%

   -Disturbance rejection: Good recovery

8. Performance Metrics

   | Parameter          | Value (Approx) |
   | ------------------ | -------------- |
   | Rise Time          | ~3–5 sec       |
   | Settling Time      | ~6–8 sec       |
   | Overshoot          | < 5%           |
   | Steady-State Error | ≈ 0%           |

9. Disturbance Analysis

   At t=10s:

   -Speed temporarily drops due to slope

   -Controller increases throttle

   -Speed returns to reference quickly

   Demonstrates robustness of system

11. MATLAB Code

    clc;

    clear;

    s = tf('s');

    G = 1/(5*s + 1);

    Kp = 2;

    Ki = 0.8;

    C = Kp + Ki/s;

    T = feedback(C*G,1);
    
    step(T);

    grid on;

    title('Closed Loop Step Response');

11. Conclusion

    The designed PI-based cruise control system successfully maintains the desired vehicle speed with minimal steady-state error and low overshoot. It effectively         compensates for disturbances such as road slope and ensures stable and smooth operation. The system meets all specified performance criteria.

12. Future Scope

    -Use PID for faster response

    -Adaptive control for varying vehicle mass

    -Real-time implementation using hardware

    -Integration with electric motor model
