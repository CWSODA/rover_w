#include "lidar_parser.hpp"

#include <cstring>

#include "settings.hpp"
#include "sender.hpp"

constexpr float LIDAR_ANGLE_CONST = 22.5f;

void LidarParser::parse_byte(uint8_t byte) {
    // ignore if checksum bytes
    if (state != State::CHECKSUM) {
        checksum_total += byte;
        ++frame_byte_count;
    }

    // debug state
#if DEBUG_LIDAR_STATE
    print_state();
#endif

    switch (state) {
        case (State::START): {
#if TIMESTAMP_FRAME
            uint8_t msg[] = {'$', 'T'};
            send_bytes(msg, sizeof(msg));
#endif
            // ignore invalid header
            // assert(byte == 0xAA);
            if (byte != 0xAA) {
                reset_state();
                break;
            }
            state = State::FRAME_LENGTH;
            break;
        }
        case (State::FRAME_LENGTH): {
            if (frame_len_buf.insert(byte)) state = State::VERSION;
            break;
        }
        case (State::VERSION): {
            // assert(byte == 0x10); // not true
            state = State::TYPE;
            break;
        }
        case (State::TYPE): {
            // ignore invalid type
            // assert(byte == 0x61);
            if (byte != 0x61) {
                reset_state();
                break;
            }
            state = State::COMMAND;
            break;
        }
        case (State::COMMAND): {
            if (byte == 0xAD) {
                // normal, parse data
                is_valid_data = true;
            } else if (byte == 0xAE) {
                // health status
                is_valid_data = false;
            } else {  // unknown command???
                assert(false);
            }
            state = State::DATA_LENGTH;
            break;
        }
        case (State::DATA_LENGTH): {
            if (!data_len_buf.insert(byte)) break;

            if (is_valid_data) {
                // remember to reset data state to intial rot speed
                data_state = DataState::ROT_SPEED;
                data_byte_count = 0;
                state = State::DATA;
            } else
                state = State::HEALTH;

            // ensures data length is consistent with frame length
            // data length should be 1+2+1+1+1+2 = 8 bytes shorter
            // DBG("Data length: %d, Frame Length: %d\n",
            //        data_len_buf.val(), frame_len_buf.val());
            assert(data_len_buf.val() == frame_len_buf.val() - 8);

            break;
        }
        case (State::DATA): {
            if (parse_data(byte)) state = State::CHECKSUM;
            break;
        }
        case (State::HEALTH): {
            assert(data_len_buf.val() == 1);
            ++data_byte_count;
            rotation_speed = byte * 0.05f;
            state = State::CHECKSUM;
            break;
        }
        case (State::CHECKSUM): {
            // enforce checksum
            if (checksum_buf.insert(byte)) {
                // TODO
#if DEBUG_ROT_SPEED
                DBG("ROT: %f\n", rotation_speed);
#endif
                reset_state();
            }
            break;
        }
    }
}

bool LidarParser::parse_data(uint8_t byte) {
    ++data_byte_count;
    switch (data_state) {
        case (DataState::ROT_SPEED): {
            rotation_speed = byte * 0.05f;
            // assert(rotation_speed != 0);  // shouldnt be 0 in here
            data_state = DataState::OFFSET_ANGLE;
            break;
        }
        case (DataState::OFFSET_ANGLE): {
            if (offset_angle_buf.insert(byte))
                data_state = DataState::START_ANGLE;
            break;
        }
        case (DataState::START_ANGLE): {
            if (!start_angle_buf.insert(byte)) break;

            float offset_angle = offset_angle_buf.val() * 0.01f;
            float start_angle = start_angle_buf.val() * 0.01f;
            // temp_point.angle = start_angle - offset_angle;
            temp_point.angle = start_angle;

            // data length includes 1+2+2 header for rot speed and angles
            // each data point has 1+2 bytes for strength(1) and distance(2)
            uint16_t n_measurements = (data_len_buf.val() - 5) / 3;
            assert(n_measurements != 0);  // should not have zero data
            delta_angle = LIDAR_ANGLE_CONST / (float)n_measurements;

// debug stuff
#if DEBUG_SAMPLE_COUNT
            DBG("N: %d\n", n_measurements);
#endif
#if DEBUG_OFFSET_ANGLE
            DBG("OA: %.3f\n", offset_angle);
#endif
#if DEBUG_ANGLE
            DBG("SA: %.3f\n", start_angle);
            DBG("N: %d\n", n_measurements);
            DBG("dA: %.3f\n", delta_angle);
#endif

            data_state = DataState::SIG_STRENGTH;
            break;
        }
        // parts below loop
        case (DataState::SIG_STRENGTH):
            temp_point.sig_strength = byte;
            data_state = DataState::DIST;
            break;
        case (DataState::DIST): {
            if (!dist_buf.insert(byte)) break;
            // byte finished processing
            // save data to vector
            // increments of 0.25mm = 0.00025 m
            temp_point.distance = dist_buf.val() * 0.25E-3f;

            // STORE DATA
            data_queue_p->push(temp_point);
#if SEND_LIDAR_DATA
            // send opcode L (lidar)
            // send sig_str, dist, angle
            // only send every 5th data point to avoid overloading uart
            static unsigned int count = 0;
            if (count % DATA_THROTTLE_COUNT == 0) {
                // sends 11 bytes total per sample
                // inputs approximately 3 bytes per sample excluding headers
                // output baudrate must be 4x faster
                // at least 460,800
                uint8_t msg[1 + 1 + 1 + 4 + 4];
                msg[0] = '$';
                msg[1] = 'L';
                msg[2] = temp_point.sig_strength;
                memcpy(&msg[3], &temp_point.distance, 4);
                memcpy(&msg[7], &temp_point.angle, 4);
                send_bytes(msg, sizeof(msg));
            }
            count++;
#endif

            // increment angle for next insert
            temp_point.angle += delta_angle;

            // loop back to sig_strength for next measurement
            data_state = DataState::SIG_STRENGTH;
            break;
        }
    }
    if (data_byte_count == data_len_buf.val()) {
        return true;
    }
    return false;
}

void LidarParser::reset_state() {
    state = State::START;

#if DEBUG_LENGTH
    DBG("CS > tot: %d, exp: %d\n", checksum_total, checksum_buf.val());
    DBG("FL > tot: %d, exp: %d\n", frame_byte_count, frame_len_buf.val());
    DBG("DL > tot: %d, exp: %d\n", data_byte_count, data_len_buf.val());
#endif
    // ensure data is correct
    // assert(checksum_total == checksum_buf.val());
    // assert(frame_byte_count == frame_len_buf.val());

    // reset values
    frame_byte_count = 0;
    checksum_total = 0;
    return;
}

void LidarParser::print_state() {
    switch (state) {
        case (State::START):
            DBG("START\n");
            break;
        case (State::FRAME_LENGTH):
            DBG("FRAME LENGTH\n");
            break;
        case (State::VERSION):
            DBG("VERSION\n");
            break;
        case (State::TYPE):
            DBG("TYPE\n");
            break;
        case (State::COMMAND):
            DBG("COMMAND\n");
            break;
        case (State::DATA_LENGTH):
            DBG("DATA LENGTH\n");
            break;
        case (State::DATA):
            switch (data_state) {
                case (DataState::ROT_SPEED):
                    DBG("DATA: ROT SPEED\n");
                    break;
                case (DataState::OFFSET_ANGLE):
                    DBG("DATA: OFFSET ANGLE\n");
                    break;
                case (DataState::START_ANGLE):
                    DBG("DATA: START ANGLE\n");
                    break;
                case (DataState::SIG_STRENGTH):
                    DBG("DATA: SIGNAL STRENGTH\n");
                    break;
                case (DataState::DIST):
                    DBG("DATA: DISTANCE\n");
                    break;
            }
            break;
        case (State::HEALTH):
            DBG("HEALTH\n");
            break;
        case (State::CHECKSUM):
            DBG("CHECKSUM\n");
            break;
    }
}