/********************************************************************
* Description: modal_state.cc
*
* State storage class for interpreter
*
* Copyright © 2014 Robert W. Ellenberg
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
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
********************************************************************/
#include "interp_base.hh"
#include "modal_state.hh"
#include <string.h>

StateTag::StateTag(): flags(0)
{
    memset(fields,-1,sizeof(fields));
    packed_flags = 0;
    memset(fields_float,-1,sizeof(fields_float));
    memset(filename, 0, sizeof(filename));
}

StateTag::StateTag(struct state_tag_t const & basetag):
    state_tag_t(basetag), flags(basetag.packed_flags)
{}

/**
 * Return true if the tag is a valid state, and false if not
 */
int StateTag::is_valid(void) const
{

    if (fields[GM_FIELD_LINE_NUMBER] <= 1) {
        return false;
    }

    //TODO magic numbers
    if (fields[GM_FIELD_ORIGIN] < 540) {
        return false;
    }

    if (fields[GM_FIELD_PLANE] < 170 ) {
        return false;
    }

    return true;
}


/**
 * Return the C-equivalent state_tag version of the current state.
 */
state_tag_t StateTag::get_state_tag() const
{
    state_tag_t out = static_cast<state_tag_t> (*this);
    out.packed_flags = flags.to_ulong();
    return out;
}

