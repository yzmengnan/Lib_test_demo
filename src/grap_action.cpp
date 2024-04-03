//
// Created by Yang on 2024/4/1.
//

#include "grap_action.h"

grap_action::grap_action(gp &m1, gt &m2) : m1(m1), m2(m2) {
}

grap_action::grap_action(TcAds_Grap_Position_Control &ads_p, TcAds_Grap_Torque_Control &ads_t) {
    this->m1 = gp(ads_p);
    this->m2 = gt(ads_t);
}
void grap_action::fast_tool_move(const int &dist) {
    int true_dist;
    if (dist >= 100) {
        true_dist = 100;
    } else if (dist <= 0) {
        true_dist = 0;
    } else
        true_dist = dist;

    int target = dist * this->fast_tool_moving_ratio +
                 fast_tool_moving_offset;
#ifdef graptool_by_position
    if (!isEnabled) {
        cout << "Error! Please enable the servos" << endl;
    } else {
        auto position_now = m1.show();
        m1.Motion({position_now[0], position_now[1],
                   target});
        while (abs(position_now[2] - target) > 200) {
            position_now = m1.show();
            this_thread::sleep_for(chrono::milliseconds(100));
            m1.Motion({position_now[0], position_now[1], target});
            this_thread::sleep_for(chrono::milliseconds(100));
        }
    }
#else
    if (!isEnabled) {
        cout << "Error! Please enable the servos" << endl;
    } else {
        auto position_now = m1.show();
        m1.Motion({target});
        while (abs(position_now[0] - target) > 200) {

            position_now = m1.show();
            this_thread::sleep_for(chrono::milliseconds(100));
            m1.Motion({target});
            this_thread::sleep_for(chrono::milliseconds(100));
        }
    }
#endif
}
void grap_action::grap_tool(bool flag) {
#ifdef graptool_by_position
    if (!isEnabled) {
        cout << "Error! Please enable the servos" << endl;
    } else {
        auto position_now = m1.show();
        if (flag) {
            while (abs(position_now[0] - uptool_open_position) + abs(position_now[1] - downtool_open_position) > 100) {
                m1.Motion({uptool_open_position, downtool_open_position});
                this_thread::sleep_for(chrono::milliseconds(100));
                position_now = m1.show();
                this_thread::sleep_for(chrono::milliseconds(100));
            };
        } else {
            while (abs(position_now[0] - uptool_close_position) + abs(position_now[1] - downtool_close_position) > 100) {
                m1.Motion({uptool_close_position, downtool_close_position});
                this_thread::sleep_for(chrono::milliseconds(100));
                position_now = m1.show();
                this_thread::sleep_for(chrono::milliseconds(100));
            }
        }
    }
#else
    if (!isEnabled) {
        cout << "Error! Please enable the servos" << endl;
    } else {
        if (!flag) {
            //close
            auto torque_now = m2.show();
            int vec1 = 0, vec2 = 0;
            int vec1_last = 0, vec2_last = 0;
            int pos_last1 = 0, pos_last2 = 0;
            int torque1_added = 0, torque2_added = 0;
            auto time_start = clock();
            while (true) {
                auto time_end = clock();
                if ((double) (time_end - time_start) / CLOCKS_PER_SEC > overtime) {
                    cout << "grap quit by overclock" << endl;
                    break;
                }
                // TODO: 添加超时函数，设定超时推出机制
                bool flag_1 = !(abs(torque_now[0]) > grap_torque_threshold), flag_2 = !(abs(torque_now[1]) > grap_torque_threshold);
                m2.Motion({grap_torque * flag_1 + torque1_added, grap_torque * flag_2 + torque2_added});
                if (!flag_1 && !flag_2)
                    break;
                this_thread::sleep_for(chrono::milliseconds(10));
                torque_now = m2.show();
                m2.show_position();
                auto position_now = m2.get_position();
                if (abs(position_now[0] - uptool_close_position) + abs(position_now[1] - downtool_close_position) <= 40000) {
                    cout << "reach limit position" << endl;
                    break;
                }
                vec1 = abs(pos_last1 - position_now[0]);
                vec2 = abs(pos_last2 - position_now[1]);
                pos_last1 = position_now[0];
                pos_last2 = position_now[1];
                if (vec1_last - 2000 > vec1) {
                    cout << "vec1:" << vec1 << " vec1_last" << vec1_last << endl;
                    torque1_added += 2;
                }
                if (vec2_last - 2000 > vec2) {
                    cout << "vec2:" << vec2 << " vec2_last:" << vec2_last << endl;
                    torque2_added += 2;
                }
                vec1_last = vec1;
                vec2_last = vec2;
                this_thread::sleep_for(chrono::milliseconds(10));
            }
        } else {
            //open
            auto torque_now = m2.show();
            int vec1 = 0, vec2 = 0;
            int vec1_last = 0, vec2_last = 0;
            int pos_last1 = 0, pos_last2 = 0;
            int torque1_added = 0, torque2_added = 0;
            auto time_start = clock();
            while (true) {
                auto time_end = clock();
                if ((double) (time_end - time_start) / CLOCKS_PER_SEC > overtime) {
                    cout << "grap quit by overclock" << endl;
                    break;
                }
                bool flag_1 = !(abs(torque_now[0]) > grap_torque_threshold), flag_2 = !(abs(torque_now[1]) > grap_torque_threshold);
                m2.Motion({-grap_torque * flag_1 - torque1_added, -grap_torque * flag_2 - torque2_added});
                if (!flag_1 && !flag_2)
                    break;
                this_thread::sleep_for(chrono::milliseconds(10));
                torque_now = m2.show();
                m2.show_position();
                auto position_now = m2.get_position();
                if (abs(position_now[0] - uptool_open_position) + abs(position_now[1] - downtool_open_position) <= 40000) {
                    cout << "reach limit position" << endl;
                    break;
                }
                vec1 = abs(pos_last1 - position_now[0]);
                vec2 = abs(pos_last2 - position_now[1]);
                cout << "vec1:" << vec1 << " vec2:" << vec2 << " vec1_last" << vec1_last << " vec2_last:" << vec2_last << endl;
                pos_last1 = position_now[0];
                pos_last2 = position_now[1];
                if (vec1_last - 2000 > vec1) {
                    torque1_added += 2;
                }
                if (vec2_last - 2000 > vec2) {
                    torque2_added += 2;
                }
                vec1_last = vec1;
                vec2_last = vec2;
                this_thread::sleep_for(chrono::milliseconds(10));
            }
        }
    }

#endif
}
grap_action::~grap_action() {
}
grap_action::grap_action() {
    auto ads1 = new TcAds_Grap_Position_Control;
    auto ads2 = new TcAds_Grap_Torque_Control;
    m1 = gp(*ads1);
    m2 = gt(*ads2);
}
void grap_action::ftmr(bool f_or_b) {
    if (f_or_b == ROTATE_FORWARD) {
        m2.Motion({0, 0, 200, 200});
        fast_tool_move(20);
        Sleep(200);
        m2.Motion({0, 0, -500, -500});
        fast_tool_move(0);
        Sleep(200);
        m2.Motion({0, 0, 500, 500});
        fast_tool_move(50);
        Sleep(200);
        m2.Motion({0, 0, -500, -500});
        fast_tool_move(0);
        m2.Motion({0, 0, 500, 500});
        fast_tool_move(100);
        Sleep(200);
    } else if (f_or_b == ROTATE_BACKWARD) {
        m2.Motion({0, 0, -200, -200});
        fast_tool_move(5);

    } else {
    }
}
