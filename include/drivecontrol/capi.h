/* C interface used by the Python simulator (ctypes) and other hosts. */
#ifndef DRIVECONTROL_CAPI_H
#define DRIVECONTROL_CAPI_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  double x, y, yaw, v, steer, yaw_rate;
  int32_t valid;
} dc_state_t;

typedef struct {
  double steer, throttle, brake;
} dc_cmd_t;

typedef struct {
  int32_t state;
  int32_t reasons;
  double lateral_error, heading_error, lookahead, s;
  double v_ref, a_des;
  double raw_steer, raw_throttle, raw_brake;
} dc_debug_t;

typedef struct {
  int32_t controller;   /* 0 = drive controller, 1 = naive baseline */
  int32_t lateral;      /* 0 = pure pursuit, 1 = Stanley */
  int32_t anti_windup;  /* 0 none, 1 clamp, 2 back-calculation */
  int32_t use_can;
  double pp_ld0, pp_kv, pp_ld_min, pp_ld_max;
  double st_k, st_k_soft, st_k_yaw_rate;
  double st_k_understeer;
  double v_kp, v_ki, v_kd;
  double steer_max, steer_rate_max;
  double estop_brake_rate;
  double max_lateral_error, controlled_decel;
  double naive_k_e, naive_k_psi, naive_kp_v, naive_ki_v; /* baseline gains */
} dc_config_t;

void dc_default_config(dc_config_t* cfg);
void* dc_create(const dc_config_t* cfg);
void dc_destroy(void* h);
int dc_set_path(void* h, const double* x, const double* y, const double* v, int n);
void dc_can_rx(void* h, uint32_t id, const uint8_t* data, double t);
void dc_step(void* h, const dc_state_t* s, double t, double dt, dc_cmd_t* out);
int dc_can_tx(void* h, uint32_t* id, uint8_t* data);
void dc_get_debug(void* h, dc_debug_t* d);

/* CAN helpers */
void dc_encode_command(double speed, int enable, int reset, uint8_t counter, uint8_t* out8);
void dc_encode_estop(int estop, uint8_t counter, uint8_t* out8);
int dc_decode_actuator(const uint8_t* data8, dc_cmd_t* out);
int dc_decode_status(const uint8_t* data8, int32_t* state, double* speed, int32_t* faults);

/* building blocks exposed for parity tests against the Python reference */
void dc_pid_run(const double* gains /* kp ki kd kff out_min out_max d_tau aw kt */, const double* ref,
                const double* meas, const double* ff, int n, double dt, double* out);
double dc_lateral_once(int mode, const double* cfg4, const double* px, const double* py, const double* pv, int n,
                       const dc_state_t* s, double wheelbase);

#ifdef __cplusplus
}
#endif

#endif
