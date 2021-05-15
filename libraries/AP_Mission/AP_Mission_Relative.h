/*
 *   Created: 08.05.2020
 *    Author: Dipl.Ing.(FH)WillyZehnder
 */
/// @file    AP_Mission_Relative.h
/// @brief   translates and rotates Missions according to the location where Mode-AUTO has been switched on
/*
 *   The AP_Mission_Relative library:
 *   - memorizes the location where Mode-AUTO has been switched on (Base-Point)
 *   - moves the individual Waypoints according to Base-Point and switch-setting
 */
#pragma once

#include <AP_Vehicle/ModeReason.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

// no translation of the Mission within a radius of XX m around Homepoint
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
  #define NO_TRANSLATION_RADIUS 50
#elif APM_BUILD_TYPE(APM_BUILD_ArduCopter)
  #define NO_TRANSLATION_RADIUS 10
#else
  #define NO_TRANSLATION_RADIUS 5
#endif

    class AP_Mission;

/// @class    AP_Mission_Relative
/// @brief    Object managing movement of Waypoints
class AP_Mission_Relative
{
    friend class AP_Mission;

public:

    AP_Mission_Relative(void);
    /* Do not allow copies */
    AP_Mission_Relative(const AP_Mission_Relative &other) = delete;
    AP_Mission_Relative &operator=(const AP_Mission_Relative&) = delete;
    static AP_Mission_Relative *get_singleton(void) {
        return _singleton;
    }

private:

    static AP_Mission_Relative *_singleton;

    enum class Restart_Behaviour {
        RESTART_NOT_TRANSLATED,
        RESTART_PARALLEL_TRANSLATED,
        RESTART_ROTATED_HEADING
    };

    // for lat/lng translation of a Relative Mission
    struct Translation {
        int32_t alt;            // altitude-displacement [cm]
        int32_t direction;      // direction from HomePoint to Basepoint [10^2deg] North=0 East=9000
        bool    do_translation; // flag for release or blocking of translation
        bool    calculated;     // flag if first Waypoint is still proceeded and displacement is calculated
        };

    // for lat/lng rotation of a Relative Mission
    struct Rotation {
        int32_t lat;            // latitude-displacement of location [10^7deg]
        int32_t lng;            // longitude-displacement of location [10^7deg]
        int32_t direction;      // amount of rotation of a Waypoint location [10^2deg] North=0 East=9000
        };

    // internal variables
    Restart_Behaviour _restart_behaviour;     // behavior at restart of a Mission
    int32_t           _no_translation_radius; // distance in [m] from HomeLocation wherein a translation of a Relative Mission will be ignored

    struct Location         _basepoint_loc; // location where the MODE has been switched to AUTO
    struct Location         _first_wp_loc;  // original location of very first Waypoint of a Mission
    struct Translation      _translation;   // info concerning the translation of a  Relative Mission

    /// memorizes the Location and Attitudes at Base-Point
    void memorize_basepoint(void);

    /// set _translation.do_translation = false
    void set_no_translation();

    /// if the Command id is a Waypoint, the Location will be moved according to switch-setting
    void move_location(Location& loc, const uint16_t id);

    /// translate a Waypoint location
    void translate_location(Location& loc);

    /// rotate a Waypoint location
    void rotate_location(Location& loc);

};

namespace AP {
    AP_Mission_Relative &mission_relative();
};
