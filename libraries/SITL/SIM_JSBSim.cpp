/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  simulator connector for JSBSim
*/

#include "SIM_JSBSim.h"

#if HAL_SIM_JSBSIM_ENABLED

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

namespace SITL {

// the asprintf() calls are not worth checking for SITL
#pragma GCC diagnostic ignored "-Wunused-result"

#define DEBUG_JSBSIM 1

JSBSim::JSBSim(const char *frame_str) :
    Aircraft(frame_str),
    sock_control(false),
    sock_fgfdm(true),
    initialised(false),
    opened_control_socket(false),
    opened_fdm_socket(false),
    frame(FRAME_NORMAL)
{
    if (strstr(frame_str, "elevon")) {
        frame = FRAME_ELEVON;
    } else if (strstr(frame_str, "vtail")) {
        frame = FRAME_VTAIL;
    } else {
        frame = FRAME_NORMAL;
    }
    const char *model_name = strchr(frame_str, ':');
    if (model_name != nullptr) {
        jsbsim_model = model_name + 1;
    }
}


/*
  open control socket to JSBSim
 */
bool JSBSim::open_control_socket(void)
{
    if (opened_control_socket) {
        return true;
    }

    control_port = 5505 + instance*10;
    fdm_port = 5504 + instance*10;
    printf("JSBSim backend started: control_port=%u fdm_port=%u\n",
           control_port, fdm_port);
    
    if (!sock_control.connect("127.0.0.1", control_port)) {
        return false;
    }
    printf("Opened JSBSim control socket\n");
    sock_control.set_blocking(false);
    opened_control_socket = true;

    char startup[] =
        "info\n"
        "resume\n"
        "set atmosphere/turb-type 4\n";
    sock_control.send(startup, strlen(startup));
    return true;
}

/*
  open fdm socket from JSBSim
 */
bool JSBSim::open_fdm_socket(void)
{
    if (opened_fdm_socket) {
        return true;
    }
    if (!sock_fgfdm.bind("127.0.0.1", fdm_port)) {
        printf("Failed to open JSBSim fdm socket\n");
        return false;
    }
    sock_fgfdm.set_blocking(false);
    sock_fgfdm.reuseaddress();
    opened_fdm_socket = true;
    return true;
}


/*
  decode and send servos
*/
void JSBSim::send_servos(const struct sitl_input &input)
{
    char *buf = nullptr;
    float aileron  = filtered_servo_angle(input, 0);
    float elevator = filtered_servo_angle(input, 1);
    float throttle = filtered_servo_range(input, 2);
    float rudder   = filtered_servo_angle(input, 3);
    if (frame == FRAME_ELEVON) {
        // fake an elevon plane
        float ch1 = aileron;
        float ch2 = elevator;
        aileron  = (ch2-ch1)/2.0f;
        // the minus does away with the need for RC2_REVERSED=-1
        elevator = -(ch2+ch1)/2.0f;
    } else if (frame == FRAME_VTAIL) {
        // fake a vtail plane
        float ch1 = elevator;
        float ch2 = rudder;
        // this matches VTAIL_OUTPUT==2
        elevator = (ch2-ch1)/2.0f;
        rudder   = (ch2+ch1)/2.0f;
    }
    float wind_speed_fps = input.wind.speed / FEET_TO_METERS;
    asprintf(&buf,
             "set fcs/aileron-cmd-norm %f\n"
             "set fcs/elevator-cmd-norm %f\n"
             "set fcs/rudder-cmd-norm %f\n"
             "set fcs/throttle-cmd-norm[0] %f\n"
             "set fcs/throttle-cmd-norm[1] %f\n"
             "set atmosphere/psiw-rad %f\n"
             "set atmosphere/wind-mag-fps %f\n"
             "set atmosphere/turbulence/milspec/windspeed_at_20ft_AGL-fps %f\n"
             "set atmosphere/turbulence/milspec/severity %f\n",
             aileron, elevator, rudder, throttle, throttle,
             radians(input.wind.direction),
             wind_speed_fps,
             wind_speed_fps/3,
             input.wind.turbulence);
    ssize_t buflen = strlen(buf);
    ssize_t sent = sock_control.send(buf, buflen);
    free(buf);
    if (sent < 0) {
        if (errno != EAGAIN) {
            fprintf(stderr, "Fatal: Failed to send on control socket: %s\n",
                    strerror(errno));
            exit(1);
        }
    }
    if (sent < buflen) {
        fprintf(stderr, "Failed to send all bytes on control socket\n");
    }
}

/* nasty hack ....
   JSBSim sends in little-endian
 */
void FGNetFDM::ByteSwap(void)
{
    uint32_t *buf = (uint32_t *)this;
    for (uint16_t i=0; i<sizeof(*this)/4; i++) {
        buf[i] = ntohl(buf[i]);
    }
    // fixup the 3 doubles
    buf = (uint32_t *)&longitude;
    uint32_t tmp;
    for (uint8_t i=0; i<3; i++) {
        tmp = buf[0];
        buf[0] = buf[1];
        buf[1] = tmp;
        buf += 2;
    }
}

/*
  receive an update from the FDM
  This is a blocking function
 */
void JSBSim::recv_fdm(const struct sitl_input &input)
{
    FGNetFDM fdm;
    memset(&fdm, 0, sizeof(fdm));
    time_now_us = fdm.cur_time; 

    do {
        while (sock_fgfdm.recv(&fdm, sizeof(fdm), 100) != sizeof(fdm)) {
            send_servos(input);
        }
        fdm.ByteSwap();
    } while (fdm.cur_time == time_now_us);

    accel_body = Vector3f(fdm.A_X_pilot, fdm.A_Y_pilot, fdm.A_Z_pilot) * FEET_TO_METERS;

    double p, q, r;
    SIM::convert_body_frame(degrees(fdm.phi), degrees(fdm.theta),
                             degrees(fdm.phidot), degrees(fdm.thetadot), degrees(fdm.psidot),
                             &p, &q, &r);
    gyro = Vector3f(p, q, r);

    velocity_ef = Vector3f(fdm.v_north, fdm.v_east, fdm.v_down) * FEET_TO_METERS;
    location.lat = RAD_TO_DEG_DOUBLE * fdm.latitude * 1.0e7;
    location.lng = RAD_TO_DEG_DOUBLE * fdm.longitude * 1.0e7;
    location.alt = fdm.agl*100 + home.alt;
    dcm.from_euler(fdm.phi, fdm.theta, fdm.psi);
    airspeed = fdm.vcas * KNOTS_TO_METERS_PER_SECOND;
    airspeed_pitot = airspeed;

    // update magnetic field
    update_mag_field_bf();
    
    rpm[0] = fdm.rpm[0];
    rpm[1] = fdm.rpm[1];
    
    time_now_us = fdm.cur_time;
}

void JSBSim::drain_control_socket()
{
    const uint16_t buflen = 1024;
    char buf[buflen];
    ssize_t received;
    do {
        received = sock_control.recv(buf, buflen, 0);
    } while (received > 0);
}
/*
  update the JSBSim simulation by one time step
 */
void JSBSim::update(const struct sitl_input &input)
{
    while (!initialised) {
        if (!open_control_socket() ||
            !open_fdm_socket()) {
            time_now_us = 1;
            return;
        }
        initialised = true;
        printf("JSBSim initialised\n");
    }
    send_servos(input);
    recv_fdm(input);
    adjust_frame_time(rate_hz);
    sync_frame_time();
    drain_control_socket();
}

} // namespace SITL

#endif  // HAL_SIM_JSBSIM_ENABLED
