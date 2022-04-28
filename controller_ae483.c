#include "controller_ae483.h"
#include "stabilizer_types.h"
#include "power_distribution.h"
#include "log.h"
#include "param.h"
#include "num.h"
#include "math3d.h"
#include "radiolink.h"
#include "string.h"
#include "debug.h"
#include "configblock.h"
#include "time.h"

// Sensor measurements
// - tof (from the z ranger on the flow deck)
static uint16_t tof_count = 0;
static float tof_distance = 0.0f;
// - flow (from the optical flow sensor on the flow deck)
static uint16_t flow_count = 0;
static float flow_dpixelx = 0.0f;
static float flow_dpixely = 0.0f;

// Parameters
static uint8_t use_observer = 0;
static bool reset_observer = false;

// State
static float o_x = 0.0f;
static float o_y = 0.0f;
static float o_z = 0.0f;
static float psi = 0.0f;
static float theta = 0.0f;
static float phi = 0.0f;
static float v_x = 0.0f;
static float v_y = 0.0f;
static float v_z = 0.0f;
static float w_x = 0.0f;
static float w_y = 0.0f;
static float w_z = 0.0f;

// Setpoint
static float o_x_des = 0.0f;
static float o_y_des = 0.0f;
static float o_z_des = 0.0f;

// Input
static float tau_x = 0.0f;
static float tau_y = 0.0f;
static float tau_z = 0.0f;
static float f_z = 0.0f;

// Motor power command
static uint16_t m_1 = 0;
static uint16_t m_2 = 0;
static uint16_t m_3 = 0;
static uint16_t m_4 = 0;
// Measurements
static float n_x = 0.0f;
static float n_y = 0.0f;
static float r = 0.0f;
static float a_z = 0.0f;

// Constants
static float k_flow = 4.09255568f;
static float g = 9.81f;
static float dt = 0.002f;
static float o_z_eq = 0.5f; 

// Measurement errors
static float n_x_err = 0.0f;
static float n_y_err = 0.0f;
static float r_err = 0.0f;

// OptiTrack State
static float x = 0.0f; // x position from OptiTrack
static float y = 0.0f; // y position from OptiTrack
static float z = 0.0f; // z position from OptiTrack
static float qx = 0.0f; // x quaternion from OptiTrack
static float qy = 0.0f; // y quaternion from OptiTrack
static float qz = 0.0f; // z quaternion from OptiTrack
static float qw = 0.0f; // w quaternion from OptiTrack
static float old_x = 0.0f; // store old x position from OpiTrack
static float old_y = 0.0f; // store old y position from OptiTrack
static float old_z = 0.0f; // store old z position from OptiTrack
static long old_ts = 0L;
static float dt_pose = 0.0f;


static long get_nanos(void) {
    struct timespec ts;
    timespec_get(&ts, TIME_UTC);
    return (long)ts.tv_sec * 1000000000L + ts.tv_nsec;
}

void ae483UpdateWithTOF(tofMeasurement_t *tof)
{
  tof_distance = tof->distance;
  tof_count++;
}

void ae483UpdateWithFlow(flowMeasurement_t *flow)
{
  flow_dpixelx = flow->dpixelx;
  flow_dpixely = flow->dpixely;
  flow_count++;
}

void ae483UpdateWithDistance(distanceMeasurement_t *meas)
{
  // If you have a loco positioning deck, this function will be called
  // each time a distance measurement is available. You will have to write
  // code to handle these measurements. These data are available:
  //
  //  meas->anchorId  uint8_t   id of anchor with respect to which distance was measured
  //  meas->x         float     x position of this anchor
  //  meas->y         float     y position of this anchor
  //  meas->z         float     z position of this anchor
  //  meas->distance  float     the measured distance
}

void ae483UpdateWithPosition(positionMeasurement_t *meas)
{
  // This function will be called each time you send an external position
  // measurement (x, y, z) from the client, e.g., from a motion capture system.
  // You will have to write code to handle these measurements. These data are
  // available:
  //
  x = meas->x;         //float     x component of external position measurement
  y = meas->y;         //float     y component of external position measurement
  z = meas->z;         //float     z component of external position measurement
  
}

void ae483UpdateWithPose(poseMeasurement_t *meas)
{
  // This function will be called each time you send an external "pose" measurement
  // (position as x, y, z and orientation as quaternion) from the client, e.g., from
  // a motion capture system. You will have to write code to handle these measurements.
  // These data are available:
  //
  old_x = x;
  old_y = y;
  old_z = z;
  long new_ts = get_nanos();
  dt_pose = (float)(new_ts - old_ts) / 1000000000.0f; // In seconds
  old_ts = new_ts;
  
  x = meas->x;
  y = meas->y;
  z = meas->z;
  qx = meas->quat.x;
  qy = meas->quat.y;
  qz = meas->quat.z;
  qw = meas->quat.w;
}

void ae483UpdateWithData(const struct AE483Data* data)
{
  // This function will be called each time AE483-specific data are sent
  // from the client to the drone. You will have to write code to handle
  // these data. For the example AE483Data struct, these data are:
  //
  //  data->x         float
  //  data->y         float
  //  data->z         float
  //
  // Exactly what "x", "y", and "z" mean in this context is up to you.
}


void controllerAE483Init(void)
{
  // Do nothing
}

bool controllerAE483Test(void)
{
  // Do nothing (test is always passed)
  return true;
}

void controllerAE483(control_t *control,
                     setpoint_t *setpoint,
                     const sensorData_t *sensors,
                     const state_t *state,
                     const uint32_t tick)
{
  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    // Everything in here runs at 500 Hz

    // Desired position
    o_x_des = setpoint->position.x;
    o_y_des = setpoint->position.y;
    o_z_des = setpoint->position.z;

    // Measurements
    w_x = radians(sensors->gyro.x);
    w_y = radians(sensors->gyro.y);
    w_z = radians(sensors->gyro.z);
    a_z = g * sensors->acc.z;
    n_x = flow_dpixelx;
    n_y = flow_dpixely;
    r = tof_distance;

    if (reset_observer) {
      o_x = 0.0f;
      o_y = 0.0f;
      o_z = 0.0f;
      psi = 0.0f;
      theta = 0.0f;
      phi = 0.0f;
      v_x = 0.0f;
      v_y = 0.0f;
      v_z = 0.0f;
      reset_observer = false;
    }

    // State estimates
    if (use_observer == 1) {
      // AE483 observer
    
      // Compute each element of:
      // 
      //   C x + D u - y
      // 
      n_x_err = k_flow * ((v_x / o_z_eq) - w_y) - n_x;
      n_y_err = k_flow * ((v_y / o_z_eq) + w_x) - n_y; 
      r_err = o_z - r;
      

     
      // Update estimates
      o_x += dt * (v_x);   
      o_y += dt * (v_y);   
      o_z += dt * (v_z - 19.991665f*r_err);   
      psi += dt * (w_z);  
      theta += dt * (w_y - 0.003731f*n_x_err); 
      phi += dt * (w_x + 0.002154f*n_y_err);  
      v_x += dt * (g*theta - 0.098888f*n_x_err);  
      v_y += dt * (-g*phi - 0.073462f*n_y_err); 
      v_z += dt * (a_z-g - 87.333333f*r_err);
      
    } 
    if (use_observer == 0) {
      // Complementary Filter
      o_x = state->position.x;
      o_y = state->position.y;
      o_z = state->position.z;
      psi = radians(state->attitude.yaw);
      theta = - radians(state->attitude.pitch);
      phi = radians(state->attitude.roll);
      v_x = state->velocity.x;
      v_y = state->velocity.y;
      v_z = state->velocity.z;
    }

    if (use_observer == 2) {
      // OptiTrack Observer

      // Get State position directly from OptiTrack
      o_x = x;
      o_y = y;
      o_z = z;
      // Convert quaternion to (roll, pitch, yaw) Euler angles using Tait-Bryan convention
      // (yaw, then pitch about new pitch axis, then roll about new roll axis)
      psi = atan2f(2.0f * (qw * qz + qx * qy), 1 - 2 * (fsqr(qy) + fsqr(qz))); // yaw
      theta = asinf(2.0f * (qw * qy - qx * qz)); // pitch
      phi = atan2f(2.0f * (qw * qx + qy * qz), 1 - 2 * (fsqr(qx) + fsqr(qy))); // roll
      // Get x, y, z velocities based on current and previous position
      v_x = (x - old_x) / dt_pose;
      v_y = (y - old_y) / dt_pose;
      v_z = (z - old_z) / dt_pose;
    }
   
    // Parse measurements
    n_x = flow_dpixelx;
    n_y = flow_dpixely;
    r = tof_distance;
    a_z = 9.81f * sensors->acc.z;

    if (setpoint->mode.z == modeDisable) {
      // If there is no desired position, then all
      // motor power commands should be zero

      powerSet(0, 0, 0, 0);
    } else {
      // Otherwise, motor power commands should be
      // chosen by the controller

      // FIXME
      tau_x = 0.00198846f * (o_y - o_y_des) -0.00464423f * phi + 0.00150935f * v_y -0.00073568f * w_x;
      tau_y = -0.00198846f * (o_x - o_x_des) -0.00514773f * theta -0.00157552f * v_x -0.00074979f * w_y;
      tau_z = -0.00033122f * psi -0.00015339f * w_z;
      f_z = -0.18494275f * (o_z - o_z_des) -0.21387905f * v_z + 0.30607200f;

      // FIXME
      m_1 = limitUint16( -3569388.9f * tau_x -3569388.9f * tau_y -39123630.7f * tau_z + 121359.2f * f_z );
      m_2 = limitUint16( -3569388.9f * tau_x + 3569388.9f * tau_y + 39123630.7f * tau_z + 121359.2f * f_z );
      m_3 = limitUint16( 3569388.9f * tau_x + 3569388.9f * tau_y -39123630.7f * tau_z + 121359.2f * f_z );
      m_4 = limitUint16( 3569388.9f * tau_x -3569388.9f * tau_y + 39123630.7f * tau_z + 121359.2f * f_z );

      
      // Apply motor power commands
      powerSet(m_1, m_2, m_3, m_4);
    }
  } 
} 

static int MESSAGE_LENGHT = 10;

void p2pcallbackHandler(P2PPacket *p)
{
  // Parse the data from the other crazyflie and print it
  uint8_t other_id = p->data[0];
  char msg[MESSAGE_LENGHT + 1];
  memcpy(&msg, &p->data[1], sizeof(char)*MESSAGE_LENGHT);
  msg[MESSAGE_LENGHT] = 0;
  uint8_t rssi = p->rssi;

  DEBUG_PRINT("[RSSI: -%d dBm] Message from CF nr. %d, %s\n", rssi, other_id, msg);
}

void appMain()
{
  DEBUG_PRINT("Waiting for activation ...\n");

    // Initialize the p2p packet 
    static P2PPacket p_reply;
    p_reply.port=0x00;
    
    // Get the current address of the crazyflie and obtain
    //   the last two digits and send it as the first byte
    //   of the payload
    uint64_t address = configblockGetRadioAddress();
    uint8_t my_id =(uint8_t)((address) & 0x00000000ff);
    p_reply.data[0]=my_id;

    //Put a string in the payload
    char *str="Hello World";
    memcpy(&p_reply.data[1], str, sizeof(char)*MESSAGE_LENGHT);

    // Set the size, which is the amount of bytes the payload with ID and the string 
    p_reply.size=sizeof(char)*MESSAGE_LENGHT+1;

    // Register the callback function so that the CF can receive packets as well.
    p2pRegisterCB(p2pcallbackHandler);
}

//              1234567890123456789012345678 <-- max total length
//              group   .name
LOG_GROUP_START(ae483log)
LOG_ADD(LOG_UINT16,         num_tof,                &tof_count)
LOG_ADD(LOG_UINT16,         num_flow,               &flow_count)
LOG_ADD(LOG_FLOAT,          o_x,                    &o_x)
LOG_ADD(LOG_FLOAT,          o_y,                    &o_y)
LOG_ADD(LOG_FLOAT,          o_z,                    &o_z)
LOG_ADD(LOG_FLOAT,          psi,                    &psi)
LOG_ADD(LOG_FLOAT,          theta,                  &theta)
LOG_ADD(LOG_FLOAT,          phi,                    &phi)
LOG_ADD(LOG_FLOAT,          v_x,                    &v_x)
LOG_ADD(LOG_FLOAT,          v_y,                    &v_y)
LOG_ADD(LOG_FLOAT,          v_z,                    &v_z)
LOG_ADD(LOG_FLOAT,          w_x,                    &w_x)
LOG_ADD(LOG_FLOAT,          w_y,                    &w_y)
LOG_ADD(LOG_FLOAT,          w_z,                    &w_z)
LOG_ADD(LOG_FLOAT,          o_x_des,                &o_x_des)
LOG_ADD(LOG_FLOAT,          o_y_des,                &o_y_des)
LOG_ADD(LOG_FLOAT,          o_z_des,                &o_z_des)
LOG_ADD(LOG_FLOAT,          tau_x,                  &tau_x)
LOG_ADD(LOG_FLOAT,          tau_y,                  &tau_y)
LOG_ADD(LOG_FLOAT,          tau_z,                  &tau_z)
LOG_ADD(LOG_FLOAT,          f_z,                    &f_z)
LOG_ADD(LOG_UINT16,         m_1,                    &m_1)
LOG_ADD(LOG_UINT16,         m_2,                    &m_2)
LOG_ADD(LOG_UINT16,         m_3,                    &m_3)
LOG_ADD(LOG_UINT16,         m_4,                    &m_4)
LOG_ADD(LOG_FLOAT,          n_x,                    &n_x)
LOG_ADD(LOG_FLOAT,          n_y,                    &n_y)
LOG_ADD(LOG_FLOAT,          r,                      &r)
LOG_ADD(LOG_FLOAT,          a_z,                    &a_z)
LOG_GROUP_STOP(ae483log)

//                1234567890123456789012345678 <-- max total length
//                group   .name
PARAM_GROUP_START(ae483par)
PARAM_ADD(PARAM_UINT8,     use_observer,            &use_observer)
PARAM_ADD(PARAM_UINT8,     reset_observer,          &reset_observer)
PARAM_GROUP_STOP(ae483conpar)
