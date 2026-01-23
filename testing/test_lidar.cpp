#include "test_lidar.hpp"

void LidarParser::parse_byte(uint8_t byte) {
    // ignore if checksum bytes
    if (state != State::CHECKSUM) {
        checksum_total += byte;
        ++frame_byte_count;
    }

    switch (state) {
        case (State::START):
            assert(byte == 0xAA);
            advance_state();
            break;
        case (State::FRAME_LENGTH):
            if (frame_len_buf.insert(byte)) advance_state();
            break;
        case (State::VERSION):
            // assert(byte == 0x10);
            advance_state();
            break;
        case (State::TYPE):
            assert(byte == 0x61);
            advance_state();
            break;
        case (State::COMMAND):
            if (byte == 0xAD) {
                // normal, parse data
                is_valid_data = true;
            } else if (byte == 0xAE) {
                // health status
                is_valid_data = false;
            } else {  // unknown command???
                assert(false);
            }
            advance_state();
            break;
        case (State::DATA_LENGTH):
            if (data_len_buf.insert(byte)) {
                data_state = DataState::ROT_SPEED;
                if (is_valid_data)
                    state = State::DATA;
                else
                    state = State::HEALTH;

                // ensures data length is consistent with frame length
                // data length should be 1+2+1+1+1+2 = 8 bytes shorter
                printf("Data length: %d, Frame Length: %d\n",
                       data_len_buf.val(), frame_len_buf.val());
                // assert(data_len_buf.val() == frame_len_buf.val() - 8);
            }
            break;
        case (State::DATA):
            if (parse_data(byte)) advance_state();
            break;
        case (State::HEALTH):
            assert(data_len_buf.val() == 1);
            rotation_speed = byte * 0.05f;
            advance_state();
            break;
        case (State::CHECKSUM):
            // enforce checksum
            if (checksum_buf.insert(byte)) {
                advance_state();
            }
            break;
    }

    // debug state
    print_state();
}

bool LidarParser::parse_data(uint8_t byte) {
    ++data_byte_count;
    switch (data_state) {
        case (DataState::ROT_SPEED):
            rotation_speed = byte * 0.05f;
            assert(rotation_speed != 0);  // shouldnt be 0 in here
            advance_d_state();
            break;
        case (DataState::ANGLE):
            if (angle_buf.insert(byte)) advance_d_state();
            break;
        case (DataState::START_ANGLE):
            if (start_angle_buf.insert(byte)) {
                start_angle = start_angle_buf.val() * 0.01;
                temp_point.angle = start_angle;
                advance_d_state();
            }
            break;
        case (DataState::END_ANGLE):
            if (end_angle_buf.insert(byte)) {
                end_angle = end_angle_buf.val() * 0.01;
                uint16_t n_measurements = (data_len_buf.val() - 7) / 3;
                assert(n_measurements != 0);
                delta_angle = (end_angle - start_angle) / n_measurements;
                advance_d_state();
            }
            break;
        // parts below loop
        case (DataState::SIG_STRENGTH):
            temp_point.sig_strength = byte;
            advance_d_state();
            break;
        case (DataState::DIST):
            if (dist_buf.insert(byte)) {
                // byte finished processing
                // save data to vector
                temp_point.distance = dist_buf.val();

                data_vec.push_back(temp_point);

                // increment angle for next insert
                temp_point.angle += delta_angle;

                advance_d_state();
            }
            break;
    }
    if (data_byte_count == data_len_buf.val()) return true;
    return false;
}

void LidarParser::advance_state() {
    if (state == State::CHECKSUM) {
        // resets state machine
        state = State::START;

        // ensure data is correct
        printf("CHECKSUM >>> total: %d, expected: %d\n", checksum_total,
               checksum_buf.val());
        printf("FRAME LENGTH >>> total: %d, expected: %d\n", frame_byte_count,
               frame_len_buf.val());
        printf("DATA LENGTH >>> total: %d, expected: %d\n", data_byte_count,
               data_len_buf.val());
        assert(checksum_total == checksum_buf.val());
        assert(frame_byte_count == frame_len_buf.val());
        assert(data_byte_count == data_len_buf.val());

        // reset values
        frame_byte_count = 0;
        data_byte_count = 0;
        checksum_total = 0;
        return;
    } else if (state == State::HEALTH) {
        state = State::CHECKSUM;
        return;
    }

    state = static_cast<State>(static_cast<int>(state) + 1);
}

void LidarParser::advance_d_state() {
    // loops between sig_strength and dist for subsequent measurements
    if (data_state == DataState::DIST) {
        data_state = DataState::SIG_STRENGTH;
        return;
    }

    data_state = static_cast<DataState>(static_cast<int>(data_state) + 1);
}

void LidarParser::print_state() {
    switch (state) {
        case (State::START):
            printf("START\n");
            break;
        case (State::FRAME_LENGTH):
            printf("FRAME LENGTH\n");
            break;
        case (State::VERSION):
            printf("VERSION\n");
            break;
        case (State::TYPE):
            printf("TYPE\n");
            break;
        case (State::COMMAND):
            printf("COMMAND\n");
            break;
        case (State::DATA_LENGTH):
            printf("DATA LENGTH\n");
            break;
        case (State::DATA):
            switch (data_state) {
                case (DataState::ROT_SPEED):
                    printf("DATA: ROT SPEED\n");
                    break;
                case (DataState::ANGLE):
                    printf("DATA: ANGLE\n");
                    break;
                case (DataState::START_ANGLE):
                    printf("DATA: START ANGLE\n");
                    break;
                case (DataState::END_ANGLE):
                    printf("DATA: END ANGLE\n");
                    break;
                // parts below loop
                case (DataState::SIG_STRENGTH):
                    printf("DATA: SIGNAL STRENGTH\n");
                    break;
                case (DataState::DIST):
                    printf("DATA: DISTANCE\n");
                    break;
            }
            break;
        case (State::HEALTH):
            printf("HEALTH\n");
            break;
        case (State::CHECKSUM):
            printf("CHECKSUM\n");
            break;
    }
}