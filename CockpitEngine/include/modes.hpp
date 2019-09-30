#ifndef MODES_HPP
#define MODES_HPP

#include <iostream>
#include <string>

enum Mode: int
{
    // Robot control modes
    savepos = 0,
    unlock = 1,
    follow = 2,
    show_laser = 3,
    //show_tip = 4,
};

std::string get_mode_string(Mode& m){

    switch (m){

    case Mode::unlock:
        return "Unlock Tool";
    case Mode::follow:
        return "Automatic Tool Tracking";
    case Mode::savepos:
        return "Save View";
    case Mode::show_laser:
        return "Activate Laser";
    /*
        case Mode::show_tip:
        return "show_tip";
        */
    }
}

void operator++(Mode &m){

    m = (m == Mode::show_laser) ? Mode::savepos : static_cast<Mode>(static_cast<int>(m) + 1);
    std::cout << "Mode " << get_mode_string(m) << " activated ! " << std::endl;
}

#endif // MODES_HPP
