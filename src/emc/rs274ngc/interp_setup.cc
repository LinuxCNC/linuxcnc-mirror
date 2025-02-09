/*
 * Copyright: 2013-4
 * Author: Jeff Epler <jepler@unpythonic.net>
 * Author: Michael Haberler <git@mah.priv.at>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#ifndef BOOST_PYTHON_MAX_ARITY
#define BOOST_PYTHON_MAX_ARITY 4
#endif
#include <string.h>
#include "rs274ngc_interp.hh"
#include <boost/python/object.hpp>

#pragma GCC diagnostic error "-Wmissing-field-initializers"
setup::setup() :
    AA_axis_offset(0.0),
    AA_current(0.0),
    AA_origin_offset(0.0),
    BB_axis_offset(0.0),
    BB_current(0.0),
    BB_origin_offset(0.0),
    CC_axis_offset(0.0),
    CC_current(0.0),
    CC_origin_offset(0.0),

    u_axis_offset(0.0),
    u_current(0.0),
    u_origin_offset(0.0),

    v_axis_offset(0.0),
    v_current(0.0),
    v_origin_offset(0.0),

    w_axis_offset(0.0),
    w_current(0.0),
    w_origin_offset(0.0),

    active_g_codes{},
    active_m_codes{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
    active_settings{},

    arc_not_allowed(0),

    axis_offset_x(0.0),
    axis_offset_y(0.0),
    axis_offset_z(0.0),

    blocks{},
    remap_level(0),
    blocktext{},
    control_mode(CANON_EXACT_STOP),
    tolerance(0.0),
    naivecam_tolerance(0.0),
    current_pocket(0),

    current_x (0.0),
    current_y (0.0),
    current_z (0.0),
    cutter_comp_radius (0.0),
    cutter_comp_orientation(0),
    cutter_comp_side(CUTTER_COMP::OFF),
    cycle_cc (0.0),
    cycle_i (0.0),
    cycle_j (0.0),
    cycle_k (0.0),
    cycle_l(0),
    cycle_p (0.0),
    cycle_q (0.0),
    cycle_r (0.0),
    cycle_il (0.0),
    cycle_il_flag(0),
    distance_mode(DISTANCE_MODE::ABSOLUTE),

    ijk_distance_mode(DISTANCE_MODE::ABSOLUTE),
    feed_mode(FEED_MODE::UNITS_PER_MINUTE),
    feed_override(0),
    feed_rate (0.0),
    filename{},
    file_pointer(NULL),
    flood(0),
    length_units(CANON_UNITS_INCHES),
    center_arc_radius_tolerance_inch(CENTER_ARC_RADIUS_TOLERANCE_INCH),
    center_arc_radius_tolerance_mm(CENTER_ARC_RADIUS_TOLERANCE_MM),
    line_length(0),
    linetext{},
    mist(0),
    motion_mode(0),
    origin_index(0),
    origin_offset_x (0.0),
    origin_offset_y (0.0),
    origin_offset_z (0.0),
    rotation_xy (0.0),

    parameters{0},
    parameter_occurrence(0),
    parameter_numbers{0},
    parameter_values{0},
    named_parameter_occurrence(0),
    named_parameters{nullptr},
    named_parameter_values{0},
    percent_flag(0),
    plane(CANON_PLANE::XY),
    probe_flag(0),
    input_flag(0),
    toolchange_flag(0),
    input_index(0),
    input_digital(0),
    cutter_comp_firstmove(0),
    program_x (0.0),
    program_y (0.0),
    program_z (0.0),
    retract_mode(RETRACT_MODE::R_PLANE),
    random_toolchanger(0),
    selected_pocket(0),
    selected_tool(0),
    sequence_number(0),
    num_spindles(0),
    active_spindle(0),
    speed {0.0},
    spindle_mode{SPINDLE_MODE::CONSTANT_RPM},
    speed_feed_mode{CANON_INDEPENDENT},
    speed_override{false},
    spindle_turning{CANON_STOPPED},
    stack{},
    stack_index(0),
    tool_offset{{0,0,0},0,0,0,0,0,0},
    tool_table{},
    traverse_rate (0.0),
    orient_offset (0.0),

    defining_sub(0),
    sub_name(NULL),
    doing_continue(0),
    doing_break(0),
    executed_if(0),

    skipping_o(NULL),
    skipping_to_sub(NULL),
    skipping_start(0),
    test_value (0.0),
    return_value (0.0),
    value_returned(0),
    call_level(0),
    sub_context{},
    call_state(0),
    adaptive_feed(0),
    feed_hold(0),
    loggingLevel(0),
    debugmask(0),
    log_file{},
    program_prefix{},
    subroutines{},
    use_lazy_close(0),
    lazy_closing(0),
    wizard_root{},
    tool_change_at_g30(0),
    tool_change_quill_up(0),
    tool_change_with_spindle_on(0),
    parameter_g73_peck_clearance(0.0),
    parameter_g83_peck_clearance(0.0),
    a_axis_wrapped(0),
    b_axis_wrapped(0),
    c_axis_wrapped(0),

    a_indexer_jnum(0),
    b_indexer_jnum(0),
    c_indexer_jnum(0),

    lathe_diameter_mode(0),
    mdi_interrupt(0),
    feature_set(0),
    disable_fanuc_style_sub(false),
    loop_on_main_m99(false),
    disable_g92_persistence(0),
    pythis(),
    on_abort_command(NULL),
    init_once(CANON_STOPPED)
{
  std::fill(parameters, parameters + interp_param_global::RS274NGC_MAX_PARAMETERS, 0);
}

setup::~setup() {
    assert(!pythis || Py_IsInitialized());
    if(pythis) delete pythis;
}
