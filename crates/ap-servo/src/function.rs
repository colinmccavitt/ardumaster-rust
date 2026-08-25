//! Output functions a servo channel can be assigned to.
//!
//! Generated from `SRV_Channel.h` by `tools/parity/gen_srv_functions.py`.
//! Do not edit by hand.
//!
//! These numbers live in the `SERVOn_FUNCTION` parameters, so a wrong one
//! does not fail to compile -- it silently reassigns a vehicle's outputs.
//! That is why they are generated rather than typed.

/// One past the highest function this build defines, upstream
/// `k_nr_aux_servo_functions`. Also the size of the function registry.
pub const NR_AUX_SERVO_FUNCTIONS: usize = 190;

/// What a channel is for, upstream `SRV_Channel::Function`.
///
/// A newtype rather than an enum, because the value comes from a parameter
/// and a parameter can hold a number this build does not name. Upstream
/// carries such a value and lets [`Self::valid`] judge it; an enum would
/// have to reject it at the boundary instead.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Function(pub u8);

impl Function {
    /// Upstream `k_none`.
    pub const NONE: Self = Self(0);
    /// Upstream `k_manual`.
    pub const MANUAL: Self = Self(1);
    /// Upstream `k_flap`.
    pub const FLAP: Self = Self(2);
    /// Upstream `k_flap_auto`.
    pub const FLAP_AUTO: Self = Self(3);
    /// Upstream `k_aileron`.
    pub const AILERON: Self = Self(4);
    /// Upstream `k_unused1`.
    pub const UNUSED1: Self = Self(5);
    /// Upstream `k_mount_pan`.
    pub const MOUNT_PAN: Self = Self(6);
    /// Upstream `k_mount_tilt`.
    pub const MOUNT_TILT: Self = Self(7);
    /// Upstream `k_mount_roll`.
    pub const MOUNT_ROLL: Self = Self(8);
    /// Upstream `k_mount_open`.
    pub const MOUNT_OPEN: Self = Self(9);
    /// Upstream `k_cam_trigger`.
    pub const CAM_TRIGGER: Self = Self(10);
    /// Upstream `k_egg_drop`.
    pub const EGG_DROP: Self = Self(11);
    /// Upstream `k_mount2_pan`.
    pub const MOUNT2_PAN: Self = Self(12);
    /// Upstream `k_mount2_tilt`.
    pub const MOUNT2_TILT: Self = Self(13);
    /// Upstream `k_mount2_roll`.
    pub const MOUNT2_ROLL: Self = Self(14);
    /// Upstream `k_mount2_open`.
    pub const MOUNT2_OPEN: Self = Self(15);
    /// Upstream `k_dspoilerLeft1`.
    pub const DSPOILERLEFT1: Self = Self(16);
    /// Upstream `k_dspoilerRight1`.
    pub const DSPOILERRIGHT1: Self = Self(17);
    /// Upstream `k_aileron_with_input`.
    pub const AILERON_WITH_INPUT: Self = Self(18);
    /// Upstream `k_elevator`.
    pub const ELEVATOR: Self = Self(19);
    /// Upstream `k_elevator_with_input`.
    pub const ELEVATOR_WITH_INPUT: Self = Self(20);
    /// Upstream `k_rudder`.
    pub const RUDDER: Self = Self(21);
    /// Upstream `k_sprayer_pump`.
    pub const SPRAYER_PUMP: Self = Self(22);
    /// Upstream `k_sprayer_spinner`.
    pub const SPRAYER_SPINNER: Self = Self(23);
    /// Upstream `k_flaperon_left`.
    pub const FLAPERON_LEFT: Self = Self(24);
    /// Upstream `k_flaperon_right`.
    pub const FLAPERON_RIGHT: Self = Self(25);
    /// Upstream `k_steering`.
    pub const STEERING: Self = Self(26);
    /// Upstream `k_parachute_release`.
    pub const PARACHUTE_RELEASE: Self = Self(27);
    /// Upstream `k_gripper`.
    pub const GRIPPER: Self = Self(28);
    /// Upstream `k_landing_gear_control`.
    pub const LANDING_GEAR_CONTROL: Self = Self(29);
    /// Upstream `k_engine_run_enable`.
    pub const ENGINE_RUN_ENABLE: Self = Self(30);
    /// Upstream `k_heli_rsc`.
    pub const HELI_RSC: Self = Self(31);
    /// Upstream `k_heli_tail_rsc`.
    pub const HELI_TAIL_RSC: Self = Self(32);
    /// Upstream `k_motor1`.
    pub const MOTOR1: Self = Self(33);
    /// Upstream `k_motor2`.
    pub const MOTOR2: Self = Self(34);
    /// Upstream `k_motor3`.
    pub const MOTOR3: Self = Self(35);
    /// Upstream `k_motor4`.
    pub const MOTOR4: Self = Self(36);
    /// Upstream `k_motor5`.
    pub const MOTOR5: Self = Self(37);
    /// Upstream `k_motor6`.
    pub const MOTOR6: Self = Self(38);
    /// Upstream `k_motor7`.
    pub const MOTOR7: Self = Self(39);
    /// Upstream `k_motor8`.
    pub const MOTOR8: Self = Self(40);
    /// Upstream `k_motor_tilt`.
    pub const MOTOR_TILT: Self = Self(41);
    /// Upstream `k_generator_control`.
    pub const GENERATOR_CONTROL: Self = Self(42);
    /// Upstream `k_tiltMotorRear`.
    pub const TILTMOTORREAR: Self = Self(45);
    /// Upstream `k_tiltMotorRearLeft`.
    pub const TILTMOTORREARLEFT: Self = Self(46);
    /// Upstream `k_tiltMotorRearRight`.
    pub const TILTMOTORREARRIGHT: Self = Self(47);
    /// Upstream `k_rcin1`.
    pub const RCIN1: Self = Self(51);
    /// Upstream `k_rcin2`.
    pub const RCIN2: Self = Self(52);
    /// Upstream `k_rcin3`.
    pub const RCIN3: Self = Self(53);
    /// Upstream `k_rcin4`.
    pub const RCIN4: Self = Self(54);
    /// Upstream `k_rcin5`.
    pub const RCIN5: Self = Self(55);
    /// Upstream `k_rcin6`.
    pub const RCIN6: Self = Self(56);
    /// Upstream `k_rcin7`.
    pub const RCIN7: Self = Self(57);
    /// Upstream `k_rcin8`.
    pub const RCIN8: Self = Self(58);
    /// Upstream `k_rcin9`.
    pub const RCIN9: Self = Self(59);
    /// Upstream `k_rcin10`.
    pub const RCIN10: Self = Self(60);
    /// Upstream `k_rcin11`.
    pub const RCIN11: Self = Self(61);
    /// Upstream `k_rcin12`.
    pub const RCIN12: Self = Self(62);
    /// Upstream `k_rcin13`.
    pub const RCIN13: Self = Self(63);
    /// Upstream `k_rcin14`.
    pub const RCIN14: Self = Self(64);
    /// Upstream `k_rcin15`.
    pub const RCIN15: Self = Self(65);
    /// Upstream `k_rcin16`.
    pub const RCIN16: Self = Self(66);
    /// Upstream `k_ignition`.
    pub const IGNITION: Self = Self(67);
    /// Upstream `k_choke`.
    pub const CHOKE: Self = Self(68);
    /// Upstream `k_starter`.
    pub const STARTER: Self = Self(69);
    /// Upstream `k_throttle`.
    pub const THROTTLE: Self = Self(70);
    /// Upstream `k_tracker_yaw`.
    pub const TRACKER_YAW: Self = Self(71);
    /// Upstream `k_tracker_pitch`.
    pub const TRACKER_PITCH: Self = Self(72);
    /// Upstream `k_throttleLeft`.
    pub const THROTTLELEFT: Self = Self(73);
    /// Upstream `k_throttleRight`.
    pub const THROTTLERIGHT: Self = Self(74);
    /// Upstream `k_tiltMotorLeft`.
    pub const TILTMOTORLEFT: Self = Self(75);
    /// Upstream `k_tiltMotorRight`.
    pub const TILTMOTORRIGHT: Self = Self(76);
    /// Upstream `k_elevon_left`.
    pub const ELEVON_LEFT: Self = Self(77);
    /// Upstream `k_elevon_right`.
    pub const ELEVON_RIGHT: Self = Self(78);
    /// Upstream `k_vtail_left`.
    pub const VTAIL_LEFT: Self = Self(79);
    /// Upstream `k_vtail_right`.
    pub const VTAIL_RIGHT: Self = Self(80);
    /// Upstream `k_boost_throttle`.
    pub const BOOST_THROTTLE: Self = Self(81);
    /// Upstream `k_motor9`.
    pub const MOTOR9: Self = Self(82);
    /// Upstream `k_motor10`.
    pub const MOTOR10: Self = Self(83);
    /// Upstream `k_motor11`.
    pub const MOTOR11: Self = Self(84);
    /// Upstream `k_motor12`.
    pub const MOTOR12: Self = Self(85);
    /// Upstream `k_dspoilerLeft2`.
    pub const DSPOILERLEFT2: Self = Self(86);
    /// Upstream `k_dspoilerRight2`.
    pub const DSPOILERRIGHT2: Self = Self(87);
    /// Upstream `k_winch`.
    pub const WINCH: Self = Self(88);
    /// Upstream `k_mainsail_sheet`.
    pub const MAINSAIL_SHEET: Self = Self(89);
    /// Upstream `k_cam_iso`.
    pub const CAM_ISO: Self = Self(90);
    /// Upstream `k_cam_aperture`.
    pub const CAM_APERTURE: Self = Self(91);
    /// Upstream `k_cam_focus`.
    pub const CAM_FOCUS: Self = Self(92);
    /// Upstream `k_cam_shutter_speed`.
    pub const CAM_SHUTTER_SPEED: Self = Self(93);
    /// Upstream `k_scripting1`.
    pub const SCRIPTING1: Self = Self(94);
    /// Upstream `k_scripting2`.
    pub const SCRIPTING2: Self = Self(95);
    /// Upstream `k_scripting3`.
    pub const SCRIPTING3: Self = Self(96);
    /// Upstream `k_scripting4`.
    pub const SCRIPTING4: Self = Self(97);
    /// Upstream `k_scripting5`.
    pub const SCRIPTING5: Self = Self(98);
    /// Upstream `k_scripting6`.
    pub const SCRIPTING6: Self = Self(99);
    /// Upstream `k_scripting7`.
    pub const SCRIPTING7: Self = Self(100);
    /// Upstream `k_scripting8`.
    pub const SCRIPTING8: Self = Self(101);
    /// Upstream `k_scripting9`.
    pub const SCRIPTING9: Self = Self(102);
    /// Upstream `k_scripting10`.
    pub const SCRIPTING10: Self = Self(103);
    /// Upstream `k_scripting11`.
    pub const SCRIPTING11: Self = Self(104);
    /// Upstream `k_scripting12`.
    pub const SCRIPTING12: Self = Self(105);
    /// Upstream `k_scripting13`.
    pub const SCRIPTING13: Self = Self(106);
    /// Upstream `k_scripting14`.
    pub const SCRIPTING14: Self = Self(107);
    /// Upstream `k_scripting15`.
    pub const SCRIPTING15: Self = Self(108);
    /// Upstream `k_scripting16`.
    pub const SCRIPTING16: Self = Self(109);
    /// Upstream `k_airbrake`.
    pub const AIRBRAKE: Self = Self(110);
    /// Upstream `k_LED_neopixel1`.
    pub const LED_NEOPIXEL1: Self = Self(120);
    /// Upstream `k_LED_neopixel2`.
    pub const LED_NEOPIXEL2: Self = Self(121);
    /// Upstream `k_LED_neopixel3`.
    pub const LED_NEOPIXEL3: Self = Self(122);
    /// Upstream `k_LED_neopixel4`.
    pub const LED_NEOPIXEL4: Self = Self(123);
    /// Upstream `k_roll_out`.
    pub const ROLL_OUT: Self = Self(124);
    /// Upstream `k_pitch_out`.
    pub const PITCH_OUT: Self = Self(125);
    /// Upstream `k_thrust_out`.
    pub const THRUST_OUT: Self = Self(126);
    /// Upstream `k_yaw_out`.
    pub const YAW_OUT: Self = Self(127);
    /// Upstream `k_wingsail_elevator`.
    pub const WINGSAIL_ELEVATOR: Self = Self(128);
    /// Upstream `k_ProfiLED_1`.
    pub const PROFILED_1: Self = Self(129);
    /// Upstream `k_ProfiLED_2`.
    pub const PROFILED_2: Self = Self(130);
    /// Upstream `k_ProfiLED_3`.
    pub const PROFILED_3: Self = Self(131);
    /// Upstream `k_ProfiLED_Clock`.
    pub const PROFILED_CLOCK: Self = Self(132);
    /// Upstream `k_winch_clutch`.
    pub const WINCH_CLUTCH: Self = Self(133);
    /// Upstream `k_min`.
    pub const MIN: Self = Self(134);
    /// Upstream `k_trim`.
    pub const TRIM: Self = Self(135);
    /// Upstream `k_max`.
    pub const MAX: Self = Self(136);
    /// Upstream `k_mast_rotation`.
    pub const MAST_ROTATION: Self = Self(137);
    /// Upstream `k_alarm`.
    pub const ALARM: Self = Self(138);
    /// Upstream `k_alarm_inverted`.
    pub const ALARM_INVERTED: Self = Self(139);
    /// Upstream `k_rcin1_mapped`.
    pub const RCIN1_MAPPED: Self = Self(140);
    /// Upstream `k_rcin2_mapped`.
    pub const RCIN2_MAPPED: Self = Self(141);
    /// Upstream `k_rcin3_mapped`.
    pub const RCIN3_MAPPED: Self = Self(142);
    /// Upstream `k_rcin4_mapped`.
    pub const RCIN4_MAPPED: Self = Self(143);
    /// Upstream `k_rcin5_mapped`.
    pub const RCIN5_MAPPED: Self = Self(144);
    /// Upstream `k_rcin6_mapped`.
    pub const RCIN6_MAPPED: Self = Self(145);
    /// Upstream `k_rcin7_mapped`.
    pub const RCIN7_MAPPED: Self = Self(146);
    /// Upstream `k_rcin8_mapped`.
    pub const RCIN8_MAPPED: Self = Self(147);
    /// Upstream `k_rcin9_mapped`.
    pub const RCIN9_MAPPED: Self = Self(148);
    /// Upstream `k_rcin10_mapped`.
    pub const RCIN10_MAPPED: Self = Self(149);
    /// Upstream `k_rcin11_mapped`.
    pub const RCIN11_MAPPED: Self = Self(150);
    /// Upstream `k_rcin12_mapped`.
    pub const RCIN12_MAPPED: Self = Self(151);
    /// Upstream `k_rcin13_mapped`.
    pub const RCIN13_MAPPED: Self = Self(152);
    /// Upstream `k_rcin14_mapped`.
    pub const RCIN14_MAPPED: Self = Self(153);
    /// Upstream `k_rcin15_mapped`.
    pub const RCIN15_MAPPED: Self = Self(154);
    /// Upstream `k_rcin16_mapped`.
    pub const RCIN16_MAPPED: Self = Self(155);
    /// Upstream `k_lift_release`.
    pub const LIFT_RELEASE: Self = Self(156);
    /// Upstream `k_motor13`.
    pub const MOTOR13: Self = Self(160);
    /// Upstream `k_motor14`.
    pub const MOTOR14: Self = Self(161);
    /// Upstream `k_motor15`.
    pub const MOTOR15: Self = Self(162);
    /// Upstream `k_motor16`.
    pub const MOTOR16: Self = Self(163);
    /// Upstream `k_motor17`.
    pub const MOTOR17: Self = Self(164);
    /// Upstream `k_motor18`.
    pub const MOTOR18: Self = Self(165);
    /// Upstream `k_motor19`.
    pub const MOTOR19: Self = Self(166);
    /// Upstream `k_motor20`.
    pub const MOTOR20: Self = Self(167);
    /// Upstream `k_motor21`.
    pub const MOTOR21: Self = Self(168);
    /// Upstream `k_motor22`.
    pub const MOTOR22: Self = Self(169);
    /// Upstream `k_motor23`.
    pub const MOTOR23: Self = Self(170);
    /// Upstream `k_motor24`.
    pub const MOTOR24: Self = Self(171);
    /// Upstream `k_motor25`.
    pub const MOTOR25: Self = Self(172);
    /// Upstream `k_motor26`.
    pub const MOTOR26: Self = Self(173);
    /// Upstream `k_motor27`.
    pub const MOTOR27: Self = Self(174);
    /// Upstream `k_motor28`.
    pub const MOTOR28: Self = Self(175);
    /// Upstream `k_motor29`.
    pub const MOTOR29: Self = Self(176);
    /// Upstream `k_motor30`.
    pub const MOTOR30: Self = Self(177);
    /// Upstream `k_motor31`.
    pub const MOTOR31: Self = Self(178);
    /// Upstream `k_motor32`.
    pub const MOTOR32: Self = Self(179);
    /// Upstream `k_cam_zoom`.
    pub const CAM_ZOOM: Self = Self(180);
    /// Upstream `k_lights1`.
    pub const LIGHTS1: Self = Self(181);
    /// Upstream `k_lights2`.
    pub const LIGHTS2: Self = Self(182);
    /// Upstream `k_video_switch`.
    pub const VIDEO_SWITCH: Self = Self(183);
    /// Upstream `k_actuator1`.
    pub const ACTUATOR1: Self = Self(184);
    /// Upstream `k_actuator2`.
    pub const ACTUATOR2: Self = Self(185);
    /// Upstream `k_actuator3`.
    pub const ACTUATOR3: Self = Self(186);
    /// Upstream `k_actuator4`.
    pub const ACTUATOR4: Self = Self(187);
    /// Upstream `k_actuator5`.
    pub const ACTUATOR5: Self = Self(188);
    /// Upstream `k_actuator6`.
    pub const ACTUATOR6: Self = Self(189);

    /// Whether this build defines this function, upstream
    /// `valid_function`.
    ///
    /// Upstream also tests the lower bound against `k_none`. That half is
    /// vacuous here -- `k_none` is zero and the value is unsigned -- so it
    /// is asserted at compile time below instead, which is stronger: if
    /// `k_none` ever stopped being zero this would fail to build rather
    /// than quietly accept every value.
    #[must_use]
    pub const fn valid(self) -> bool {
        (self.0 as usize) < NR_AUX_SERVO_FUNCTIONS
    }

    /// The function driving a zero-based motor channel, upstream
    /// `SRV_Channels::get_motor_function`.
    ///
    /// Three ranges, not one. Motors 1-8 are contiguous, then 9-12 sit
    /// elsewhere in the enum, then 13 onward somewhere else again --
    /// because the later motor functions were added long after the first
    /// eight and had to take whatever numbers were free. A port that
    /// assumed `k_motor1 + channel` throughout would quietly drive the
    /// wrong outputs on anything with more than eight motors.
    #[must_use]
    pub const fn motor(channel: u8) -> Self {
        if channel < 8 {
            Self(Self::MOTOR1.0 + channel)
        } else if channel < 12 {
            Self(Self::MOTOR9.0 + (channel - 8))
        } else {
            Self(Self::MOTOR13.0 + (channel - 12))
        }
    }
}

/// The premise `valid` relies on: `k_none` is the lowest function value,
/// so an unsigned value can only fail the upper bound.
const _: () = assert!(Function::NONE.0 == 0);
