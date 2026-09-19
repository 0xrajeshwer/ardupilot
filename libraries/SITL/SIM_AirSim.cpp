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
	Simulator Connector for AirSim
*/

#include "SIM_AirSim.h"

#if HAL_SIM_AIRSIM_ENABLED

#include <stdio.h>
#include <arpa/inet.h>
#include <errno.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_HAL/utility/replace.h>

#define UDP_TIMEOUT_MS 100

extern const AP_HAL::HAL& hal;

using namespace SITL;

/*
  State-of-charge -> voltage table, ported from SIM_Battery.cpp's discharge
  curve. Values are volt_per_cell at a given state-of-charge percentage;
  scaled at lookup time by (pack_max_voltage / max_cell_voltage) so it works
  for whatever pack voltage this frame simulates (3S, 4S, etc), same as the
  native battery model does.
*/
static const struct {
    float volt_per_cell;
    float soc_pct;
} airsim_soc_table[] = {
    { 4.173, 100 },
    { 4.112, 96.15 },
    { 4.085, 92.31 },
    { 4.071, 88.46 },
    { 4.039, 84.62 },
    { 3.987, 80.77 },
    { 3.943, 76.92 },
    { 3.908, 73.08 },
    { 3.887, 69.23 },
    { 3.854, 65.38 },
    { 3.833, 61.54 },
    { 3.801, 57.69 },
    { 3.783, 53.85 },
    { 3.742, 50 },
    { 3.715, 46.15 },
    { 3.679, 42.31 },
    { 3.636, 38.46 },
    { 3.588, 34.62 },
    { 3.543, 30.77 },
    { 3.503, 26.92 },
    { 3.462, 23.08 },
    { 3.379, 19.23 },
    { 3.296, 15.38 },
    { 3.218, 11.54 },
    { 3.165, 7.69 },
    { 3.091, 3.85 },
    { 2.977, 2.0 },
    { 2.8,   1.5 },
    { 2.7,   1.3 },
    { 2.5,   1.2 },
    { 2.3,   1.1 },
    { 2.1,   1.0 },
    { 1.9,   0.9 },
    { 1.6,   0.8 },
    { 1.3,   0.7 },
    { 1.0,   0.6 },
    { 0.6,   0.4 },
    { 0.3,   0.2 },
    { 0.01,  0.01},
    { 0.001, 0.001 }};

/*
  Look up resting (no-load) pack voltage for a given state-of-charge
  percentage, using the same linear-interpolation approach as
  Battery::get_resting_voltage() in SIM_Battery.cpp.
*/
static float airsim_get_resting_voltage(float charge_pct, float max_voltage)
{
    const float max_cell_voltage = airsim_soc_table[0].volt_per_cell;
    for (uint8_t i = 1; i < ARRAY_SIZE(airsim_soc_table); i++) {
        if (charge_pct >= airsim_soc_table[i].soc_pct) {
            float dv1 = charge_pct - airsim_soc_table[i].soc_pct;
            float dv2 = airsim_soc_table[i-1].soc_pct - airsim_soc_table[i].soc_pct;
            float vpc1 = airsim_soc_table[i].volt_per_cell;
            float vpc2 = airsim_soc_table[i-1].volt_per_cell;
            float cell_volt = vpc1 + (dv1 / dv2) * (vpc2 - vpc1);
            return (cell_volt / max_cell_voltage) * max_voltage;
        }
    }
    // off the bottom of the table, return a small non-zero to prevent math errors
    return 0.001f;
}

AirSim::AirSim(const char *frame_str) :
	Aircraft(frame_str),
	sock(true)
{
    if (strstr(frame_str, "-copter")) {
        output_type = OutputType::Copter;
    } else if (strstr(frame_str, "-rover")) {
        output_type = OutputType::Rover;
    } else {
        // default to copter
        output_type = OutputType::Copter;
    }

	printf("Starting SITL Airsim type %u\n", (unsigned)output_type);
}

/*
	Create & set in/out socket
*/
void AirSim::set_interface_ports(const char* address, const int port_in, const int port_out)
{
	if (!sock.bind("0.0.0.0", port_in)) {
		printf("Unable to bind Airsim sensor_in socket at port %u - Error: %s\n",
				 port_in, strerror(errno));
		return;
	}
	printf("Bind SITL sensor input at %s:%u\n", "127.0.0.1", port_in);
	sock.set_blocking(false);
	sock.reuseaddress();

	airsim_ip = address;
	airsim_control_port = port_out;
	airsim_sensor_port = port_in;

	printf("AirSim control interface set to %s:%u\n", airsim_ip, airsim_control_port);
}

/*
	Decode and send servos
*/
void AirSim::output_copter(const sitl_input& input)
{
    servo_packet pkt;

	for (uint8_t i=0; i<kArduCopterRotorControlCount; i++) {
		pkt.pwm[i] = input.servos[i];
	}

	ssize_t send_ret = sock.sendto(&pkt, sizeof(pkt), airsim_ip, airsim_control_port);
	if (send_ret != sizeof(pkt)) {
		if (send_ret <= 0) {
			printf("Unable to send servo output to %s:%u - Error: %s, Return value: %ld\n",
                   airsim_ip, airsim_control_port, strerror(errno), (long)send_ret);
		} else {
			printf("Sent %ld bytes instead of %lu bytes\n", (long)send_ret, (unsigned long)sizeof(pkt));
		}
	}
}

void AirSim::output_rover(const sitl_input& input)
{
    rover_packet pkt;

    pkt.steering = 2*((input.servos[0]-1000)/1000.0f - 0.5f);
    pkt.throttle = 2*((input.servos[2]-1000)/1000.0f - 0.5f);

    ssize_t send_ret = sock.sendto(&pkt, sizeof(pkt), airsim_ip, airsim_control_port);
    if (send_ret != sizeof(pkt)) {
        if (send_ret <= 0) {
            printf("Unable to send control output to %s:%u - Error: %s, Return value: %ld\n",
                     airsim_ip, airsim_control_port, strerror(errno), (long)send_ret);
        } else {
            printf("Sent %ld bytes instead of %lu bytes\n", (long)send_ret, (unsigned long)sizeof(pkt));
        }
    }
}

/*
    Wrapper function to send servo output
*/
void AirSim::output_servos(const sitl_input& input)
{
    switch (output_type) {
    case OutputType::Copter:
        output_copter(input);
        break;

    case OutputType::Rover:
        output_rover(input);
        break;
    }
}

/*
  very simple JSON parser for sensor data
  called with pointer to one row of sensor data, nul terminated

  This parser does not do any syntax checking, and is not at all
  general purpose
*/
bool AirSim::parse_sensors(const char *json)
{
    // printf("%s\n", json);
    for (uint16_t i=0; i<ARRAY_SIZE(keytable); i++) {
        struct keytable &key = keytable[i];

        /* look for section header */
        const char *p = strstr(json, key.section);
        if (!p) {
            // we don't have this sensor
            continue;
        }
        p += strlen(key.section)+1;

        // find key inside section
        p = strstr(p, key.key);
        if (!p) {
            printf("Failed to find key %s/%s\n", key.section, key.key);
            return false;
        }

        p += strlen(key.key)+3;
        switch (key.type) {
            case DATA_UINT64:
                *((uint64_t *)key.ptr) = strtoul(p, nullptr, 10);
                break;

            case DATA_FLOAT:
                *((float *)key.ptr) = atof(p);
                break;

            case DATA_DOUBLE:
                *((double *)key.ptr) = atof(p);
                break;

            case DATA_VECTOR3F: {
                Vector3f *v = (Vector3f *)key.ptr;
                if (sscanf(p, "[%f, %f, %f]", &v->x, &v->y, &v->z) != 3) {
                    printf("Failed to parse Vector3f for %s/%s\n", key.section, key.key);
                    return false;
                }
                break;
            }

            case DATA_VECTOR3F_ARRAY: {
                // - array of floats that represent [x,y,z] coordinate for each point hit within the range
                //       x0, y0, z0, x1, y1, z1, ..., xn, yn, zn
                // example: [23.1,0.677024,1.4784,-8.97607135772705,-8.976069450378418,-8.642673492431641e-07,]
                if (*p++ != '[') {
                    return false;
                }
                uint16_t n = 0;
                vector3f_array *v = (vector3f_array *)key.ptr;
                while (true) {
                    if (n >= v->length) {
                        Vector3f *d = (Vector3f *)realloc(v->data, sizeof(Vector3f)*(n+1));
                        if (d == nullptr) {
                            return false;
                        }
                        v->data = d;
                        v->length = n+1;
                    }
                    if (sscanf(p, "%f,%f,%f,", &v->data[n].x, &v->data[n].y, &v->data[n].z) != 3) {
                        printf("Failed to parse Vector3f for %s/%s[%u]\n", key.section, key.key, n);
                        return false;
                    }
                    n++;
                    // Goto 3rd occurence of ,
                    p = strchr(p,',');
                    if (!p) {
                        return false;
                    }
                    p++;
                    p = strchr(p,',');
                    if (!p) {
                        return false;
                    }
                    p++;
                    p = strchr(p,',');
                    if (!p) {
                        return false;
                    }
                    p++;
                    // Reached end of point cloud
                    if (p[0] == ']') {
                        break;
                    }
                }
                v->length = n;
                break;
            }

            case DATA_FLOAT_ARRAY: {
                // example: [18.0, 12.694079399108887]
                if (*p++ != '[') {
                    return false;
                }
                uint16_t n = 0;
                float_array *v = (float_array *)key.ptr;
                while (true) {
                    if (n >= v->length) {
                        float *d = (float *)realloc(v->data, sizeof(float)*(n+1));
                        if (d == nullptr) {
                            return false;
                        }
                        v->data = d;
                        v->length = n+1;
                    }
                    v->data[n] = atof(p);
                    n++;
                    p = strchr(p,',');
                    if (!p) {
                        break;
                    }
                    p++;
                }
                v->length = n;
                break;
            }
        }
    }
    return true;
}

/*
	Receive new sensor data from simulator
	This is a blocking function
*/
void AirSim::recv_fdm(const sitl_input& input)
{
    // Receive sensor packet
    ssize_t ret = sock.recv(&sensor_buffer[sensor_buffer_len], sizeof(sensor_buffer)-sensor_buffer_len, UDP_TIMEOUT_MS);
    uint32_t wait_ms = UDP_TIMEOUT_MS;
    while (ret <= 0) {
        // printf("No sensor message received - %s\n", strerror(errno));
        ret = sock.recv(&sensor_buffer[sensor_buffer_len], sizeof(sensor_buffer)-sensor_buffer_len, UDP_TIMEOUT_MS);
        wait_ms += UDP_TIMEOUT_MS;

        // If no sensor message is received after 1 second, resend servos
        // this helps if messages are lost on the way, and both AP & Airsim are waiting for each ther
        if (wait_ms > 1000) {
            wait_ms = 0;
            printf("No sensor message received in last 1s, error - %s, resending servos\n", strerror(errno));
            output_servos(input);
        }
    }

    // convert '\n' into nul
    while (uint8_t *p = (uint8_t *)memchr(&sensor_buffer[sensor_buffer_len], '\n', ret)) {
        *p = 0;
    }
    sensor_buffer_len += ret;

    const uint8_t *p2 = (const uint8_t *)memrchr(sensor_buffer, 0, sensor_buffer_len);
    if (p2 == nullptr || p2 == sensor_buffer) {
        return;
    }
    const uint8_t *p1 = (const uint8_t *)memrchr(sensor_buffer, 0, p2 - sensor_buffer);
    if (p1 == nullptr) {
        return;
    }

    parse_sensors((const char *)(p1+1));

    memmove(sensor_buffer, p2, sensor_buffer_len - (p2 - sensor_buffer));
    sensor_buffer_len = sensor_buffer_len - (p2 - sensor_buffer);

    accel_body = state.imu.linear_acceleration;
    gyro = state.imu.angular_velocity;
    velocity_ef = state.velocity.world_linear_velocity;

    location.lat = state.gps.lat * 1.0e7;
    location.lng = state.gps.lon * 1.0e7;
    location.alt = state.gps.alt * 100.0f;

    position = origin.get_distance_NED_double(location);

    dcm.from_euler(state.pose.roll, state.pose.pitch, state.pose.yaw);

    if (last_timestamp) {
        int deltat = state.timestamp - last_timestamp;
        time_now_us += deltat;

        if (deltat > 0 && deltat < 100000) {
            if (average_frame_time < 1) {
                average_frame_time = deltat;
            }
            average_frame_time = average_frame_time * 0.98 + deltat * 0.02;
        }
    }

    scanner.points = state.lidar.points;

    // Update RC input, max 12 channels
    rcin_chan_count = MIN(state.rc.rc_channels.length, 12);
    for (uint8_t i=0; i < rcin_chan_count; i++) {
        rcin[i] = state.rc.rc_channels.data[i];
    }

    // Update Rangefinder data, max sensors limit as defined
    uint8_t rng_sensor_count = MIN(state.rng.rng_distances.length, ARRAY_SIZE(rangefinder_m));
    for (uint8_t i=0; i<rng_sensor_count; i++) {
        rangefinder_m[i] = state.rng.rng_distances.data[i];
    }

#if 0
// @LoggerMessage: ASM1
// @Description: AirSim simulation data
// @Field: TimeUS: Time since system startup
// @Field: TUS: Simulation's timestamp
// @Field: R: Simulation's roll
// @Field: P: Simulation's pitch
// @Field: Y: Simulation's yaw
// @Field: GX: Simulated gyroscope, X-axis
// @Field: GY: Simulated gyroscope, Y-axis
// @Field: GZ: Simulated gyroscope, Z-axis
    AP::logger().WriteStreaming("ASM1", "TimeUS,TUS,R,P,Y,GX,GY,GZ",
                       "QQffffff",
                       AP_HAL::micros64(),
                       state.timestamp,
                       degrees(state.pose.roll),
                       degrees(state.pose.pitch),
                       degrees(state.pose.yaw),
                       degrees(gyro.x),
                       degrees(gyro.y),
                       degrees(gyro.z));

    Vector3f velocity_bf = dcm.transposed() * velocity_ef;

// @LoggerMessage: ASM2
// @Description: More AirSim simulation data
// @Field: TimeUS: Time since system startup
// @Field: AX: simulation's acceleration, X-axis
// @Field: AY: simulation's acceleration, Y-axis
// @Field: AZ: simulation's acceleration, Z-axis
// @Field: VX: simulation's velocity, X-axis
// @Field: VY: simulation's velocity, Y-axis
// @Field: VZ: simulation's velocity, Z-axis
// @Field: PX: simulation's position, X-axis
// @Field: PY: simulation's position, Y-axis
// @Field: PZ: simulation's position, Z-axis
// @Field: Alt: simulation's gps altitude
// @Field: SD: simulation's earth-frame speed-down
    AP::logger().WriteStreaming("ASM2", "TimeUS,AX,AY,AZ,VX,VY,VZ,PX,PY,PZ,Alt,SD",
                       "Qfffffffffff",
                       AP_HAL::micros64(),
                       accel_body.x,
                       accel_body.y,
                       accel_body.z,
                       velocity_bf.x,
                       velocity_bf.y,
                       velocity_bf.z,
                       position.x,
                       position.y,
                       position.z,
                       state.gps.alt,
                       velocity_ef.z);
#endif

    last_timestamp = state.timestamp;
}

/*
  update the AirSim simulation by one time step
*/
void AirSim::update(const sitl_input& input)
{
    // Send servos to AirSim
    output_servos(input);

    /// Battery simulation: derive current draw from a simplified physical
    /// motor model — Power = Thrust x power_factor, Current = Power / Voltage —
    /// the same relationship ArduPilot's native SIM_Motor.cpp uses for
    /// non-external-physics vehicles. This ties current to actual airframe
    /// weight (via hover-throttle calibration) and motor/prop efficiency
    /// (via power_factor), instead of an arbitrary throttle^2 curve that has
    /// no connection to physical drone properties.
    ///
    /// vehicle_mass_kg / hover_throttle_frac: set these to match the
    /// airframe you're simulating — hover_throttle_frac is the same concept
    /// as ArduPilot's MOT_THST_HOVER param (throttle fraction needed to
    /// hold hover), typically 0.35-0.55 for a well-built multirotor.
    /// power_factor_w_per_n: electrical watts consumed per newton of thrust
    /// produced. This is where motor/prop choice actually matters — a
    /// larger, more efficient prop needs fewer watts per newton than a
    /// small, inefficient one. Typical small-quad hardware sits ~100-150
    /// W/N; larger, more efficient props can be lower.
    const float vehicle_mass_kg = 1.5f;
    const float hover_throttle_frac = 0.5f;
    const float power_factor_w_per_n = 120.0f;

    // Total thrust the airframe needs to hover == its weight in newtons.
    // thrust_scale is the thrust produced at 100% throttle, assuming a
    // roughly linear throttle->thrust relationship (same assumption
    // SIM_Plane.cpp makes for its own thrust_scale calibration).
    const float thrust_scale = (vehicle_mass_kg * GRAVITY_MSS) / hover_throttle_frac;

    const float pwm_active_threshold = 900;
    float total_throttle = 0.0f;
    uint8_t motor_count = 0;

    if (output_type == OutputType::Copter) {
        for (uint8_t i = 0; i < kArduCopterRotorControlCount; i++) {
            float pwm = input.servos[i];
            if (pwm < pwm_active_threshold) {
                continue; // this motor slot isn't driven on the current frame
            }
            float throttle = constrain_float((pwm - 1000.0f) / 1000.0f, 0.0f, 1.0f);
            total_throttle += throttle;
            motor_count++;
        }
    } else { // OutputType::Rover
        // matches the single throttle channel used in output_rover()
        float pwm = input.servos[2];
        float throttle = constrain_float((pwm - 1000.0f) / 1000.0f, 0.0f, 1.0f);
        total_throttle += throttle;
        motor_count = 1;
    }

    // Average throttle across active motors drives total simulated thrust.
    float average_throttle = (motor_count > 0) ? (total_throttle / motor_count) : 0.0f;
    float total_thrust_n = thrust_scale * average_throttle;
    float total_power_w = power_factor_w_per_n * total_thrust_n;

    // Use last tick's battery voltage to convert power->current (matches
    // SIM_Motor.cpp's own current = power / MAX(voltage, 0.1) pattern).
    // battery_voltage still holds the previous frame's computed value here,
    // since we haven't overwritten it yet this tick.
    float total_current_amps = total_power_w / MAX(battery_voltage, 0.1f);

    // Baseline avionics/idle current — FC, ESCs, sensors, RX/telemetry draw
    // real current even at zero throttle while armed.
    const float idle_current_amps = 0.5f;
    if (motor_count > 0) {
        total_current_amps += idle_current_amps;
    }

    // Manually calculate capacity drain in the background
    static float capacity_mah = 5000.0f;
    static uint32_t last_time_ms = 0;
    uint32_t now_ms = AP_HAL::millis();

    if (last_time_ms != 0) {
        float dt_hours = (now_ms - last_time_ms) / 1000.0f / 3600.0f;
        capacity_mah -= (total_current_amps * 1000.0f) * dt_hours;
    }
    last_time_ms = now_ms;

    // Calculate resting (no-load) voltage from remaining capacity using the
    // same SoC->voltage discharge curve as SIM_Battery.cpp, so this tracks
    // the real ArduPilot battery model's knee-curve near empty instead of a
    // crude linear approximation.
    //
    // Pull pack voltage/capacity from the native SIM_BATT_VOLTAGE and
    // SIM_BATT_CAP_AH params instead of hardcoding, so this matches whatever
    // the user has actually configured in Mission Planner without a rebuild.
    const float pack_max_voltage = (sitl->batt_voltage > 0) ? sitl->batt_voltage : 12.6f;
    const float full_mah = (sitl->batt_capacity_ah > 0) ? (sitl->batt_capacity_ah * 1000.0f) : 5000.0f;
    float pct_remaining = constrain_float((capacity_mah / full_mah) * 100.0f, 0.0f, 100.0f);
    float resting_voltage = airsim_get_resting_voltage(pct_remaining, pack_max_voltage);

    // Internal-resistance sag on top of the resting voltage, scaled with
    // current draw (this is the only part the previous version modeled).
    const float internal_resistance = 0.02f; // ohms, tune per simulated pack
    battery_voltage = resting_voltage - (total_current_amps * internal_resistance);

    // Write into the Aircraft base-class members so fill_fdm() picks them up.
    battery_current = total_current_amps;

    // Receive sensor data
    recv_fdm(input);

    // update magnetic field
    update_mag_field_bf();

    report_FPS();
}

/*
  report frame rates
 */
void AirSim::report_FPS(void)
{
    if (frame_counter++ % 1000 == 0) {
        if (last_frame_count != 0) {
            printf("FPS avg=%.2f\n", 1.0e6/average_frame_time);
        }
        last_frame_count = state.timestamp;
    }
}

#endif  // HAL_SIM_AIRSIM_ENABLED