/*
   Copyright (C) 2018 Raoul Rubien (github.com/rubienr) Updated for Linuxcnc 2020 by alkabal_free.fr

   This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2 of the License, or (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with the program; if not, write to the Free
   Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
   02111-1307 USA.
 */

#include "pendant.h"

// system includes
#include <assert.h>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <functional>
#include <limits>
#include <cmath>

// local includes
#include "./hal.h"
#include "./xhc-whb04b6.h"
#include "./usb.h"

using std::endl;

namespace XhcWhb04b6 {
// ----------------------------------------------------------------------
const ButtonsCode              KeyCodes::Buttons;
const MetaButtonsCodes         KeyCodes::Meta(Buttons);
const AxisRotaryButtonCodes    KeyCodes::Axis;
const FeedRotaryButtonCodes    KeyCodes::Feed;
// ----------------------------------------------------------------------
KeyEventListener::~KeyEventListener()
{
}
// ----------------------------------------------------------------------
HandwheelStepModeStepSize::HandwheelStepModeStepSize() :
    mSequence{0.001, 0.01, 0.1, 1, 5, 10, 0}
{
}
// ----------------------------------------------------------------------
HandwheelStepModeStepSize::~HandwheelStepModeStepSize()
{
}
// ----------------------------------------------------------------------
float HandwheelStepModeStepSize::getStepSize(PositionNameIndex buttonPosition) const
{
    return mSequence[static_cast<uint8_t>(buttonPosition)];
}
// ----------------------------------------------------------------------
bool HandwheelStepModeStepSize::isPermitted(PositionNameIndex buttonPosition) const
{
    return (getStepSize(buttonPosition) > 0);
}
// ----------------------------------------------------------------------
HandwheelMpgModeStepSize::HandwheelMpgModeStepSize() :
    mSequence{2, 5, 10, 30, 60, 100, 0}
{
}
// ----------------------------------------------------------------------
HandwheelMpgModeStepSize::~HandwheelMpgModeStepSize()
{
}
// ----------------------------------------------------------------------
float HandwheelMpgModeStepSize::getStepSize(PositionNameIndex buttonPosition) const
{
    return mSequence[static_cast<uint8_t>(buttonPosition)];
}
// ----------------------------------------------------------------------
bool HandwheelMpgModeStepSize::isPermitted(PositionNameIndex buttonPosition) const
{
    return (getStepSize(buttonPosition) > 0);
}
// ----------------------------------------------------------------------
HandwheelConModeStepSize::HandwheelConModeStepSize() :
    mSequence{2, 5, 10, 30, 60, 100, 0}
{
}
// ----------------------------------------------------------------------
HandwheelConModeStepSize::~HandwheelConModeStepSize()
{
}
// ----------------------------------------------------------------------
float HandwheelConModeStepSize::getStepSize(PositionNameIndex buttonPosition) const
{
    return mSequence[static_cast<uint8_t>(buttonPosition)];
}
// ----------------------------------------------------------------------
bool HandwheelConModeStepSize::isPermitted(PositionNameIndex buttonPosition) const
{
    return (getStepSize(buttonPosition) > 0);
}
// ----------------------------------------------------------------------
HandwheelLeadModeStepSize::HandwheelLeadModeStepSize() :
    mSequence{0, 0, 0, 0, 0, 0, 0.01}
{
}
// ----------------------------------------------------------------------
HandwheelLeadModeStepSize::~HandwheelLeadModeStepSize()
{
}
// ----------------------------------------------------------------------
float HandwheelLeadModeStepSize::getStepSize(PositionNameIndex buttonPosition) const
{
    return mSequence[static_cast<uint8_t>(buttonPosition)];
}
// ----------------------------------------------------------------------
bool HandwheelLeadModeStepSize::isPermitted(PositionNameIndex buttonPosition) const
{
    return (buttonPosition == PositionNameIndex::LEAD);
}
// ----------------------------------------------------------------------
KeyCode::KeyCode(uint8_t code, const char* text, const char* altText) :
    code(code),
    text(text),
    altText(altText)
{
}
// ----------------------------------------------------------------------
KeyCode::KeyCode(const KeyCode& other) :
    code(other.code),
    text(other.text),
    altText(other.altText)
{
}
// ----------------------------------------------------------------------
bool KeyCode::operator==(const KeyCode& other) const
{
    return ((code == other.code) && (text == other.text) && (altText == other.altText));
}
// ----------------------------------------------------------------------
bool KeyCode::operator!=(const KeyCode& other) const
{
    return !(*this == other);
}
// ----------------------------------------------------------------------
std::ostream& operator<<(std::ostream& os, const KeyCode& data)
{
    std::ios init(NULL);
    init.copyfmt(os);

    os << std::hex << std::setfill('0') << "{code=0x" << std::setw(2) << static_cast<uint16_t>(data.code)
       << " text='" << data.text << "'"
       << " altText='" << data.altText << "'}";

    os.copyfmt(init);
    return os;
}
// ----------------------------------------------------------------------
MetaButtonCodes::MetaButtonCodes(const KeyCode& key, const KeyCode& modifier) :
    key(key),
    modifier(modifier)
{
}
// ----------------------------------------------------------------------
MetaButtonCodes::~MetaButtonCodes()
{
}
// ----------------------------------------------------------------------
std::ostream& operator<<(std::ostream& os, const MetaButtonCodes& data)
{
    std::ios init(NULL);
    init.copyfmt(os);
    os << "{key=" << data.key << " modifier=" << data.modifier << "}";
    os.copyfmt(init);
    return os;
}
// ----------------------------------------------------------------------
bool MetaButtonCodes::containsKeys(const KeyCode& key, const KeyCode& modifier) const
{
    return (this->key.code == key.code) && (this->modifier.code == modifier.code);
}
// ----------------------------------------------------------------------
bool MetaButtonCodes::operator==(const MetaButtonCodes& other) const
{
    return (key == other.key) && (modifier == other.modifier);
}
// ----------------------------------------------------------------------
bool MetaButtonCodes::operator!=(const MetaButtonCodes& other) const
{
    return !(*this == other);
}
// ----------------------------------------------------------------------
MetaButtonsCodes::MetaButtonsCodes(const ButtonsCode& buttons) :
    reset(buttons.reset, buttons.undefined),
    macro11(buttons.reset, buttons.function),
    stop(buttons.stop, buttons.undefined),
    macro12(buttons.stop, buttons.function),
    start(buttons.start, buttons.undefined),
    macro13(buttons.start, buttons.function),
    feed_plus(buttons.feed_plus, buttons.undefined),
    macro1(buttons.feed_plus, buttons.function),
    feed_minus(buttons.feed_minus, buttons.undefined),
    macro2(buttons.feed_minus, buttons.function),
    spindle_plus(buttons.spindle_plus, buttons.undefined),
    macro3(buttons.spindle_plus, buttons.function),
    spindle_minus(buttons.spindle_minus, buttons.undefined),
    macro4(buttons.spindle_minus, buttons.function),
    machine_home(buttons.machine_home, buttons.undefined),
    macro5(buttons.machine_home, buttons.function),
    safe_z(buttons.safe_z, buttons.undefined),
    macro6(buttons.safe_z, buttons.function),
    workpiece_home(buttons.workpiece_home, buttons.undefined),
    macro7(buttons.workpiece_home, buttons.function),
    spindle_on_off(buttons.spindle_on_off, buttons.undefined),
    macro8(buttons.spindle_on_off, buttons.function),
    function(buttons.function, buttons.undefined),
    probe_z(buttons.probe_z, buttons.undefined),
    macro9(buttons.probe_z, buttons.function),
    macro10(buttons.macro10, buttons.undefined),
    macro14(buttons.macro10, buttons.function),
    continuous(buttons.continuous, buttons.undefined),
    macro15(buttons.continuous, buttons.function),
    step(buttons.step, buttons.undefined),
    macro16(buttons.step, buttons.function),
    undefined(buttons.undefined, buttons.undefined),
    buttons{
        &reset,
        &macro11,
        &stop,
        &macro12,
        &start,
        &macro13,
        &feed_plus,
        &macro1,
        &feed_minus,
        &macro2,
        &spindle_plus,
        &macro3,
        &spindle_minus,
        &macro4,
        &machine_home,
        &macro5,
        &safe_z,
        &macro6,
        &workpiece_home,
        &macro7,
        &spindle_on_off,
        &macro8,
        &function,
        &probe_z,
        &macro9,
        &macro10,
        &macro14,
        &continuous,
        &macro15,
        &step,
        &macro16,
        &undefined
    }
{
}
// ----------------------------------------------------------------------
const MetaButtonCodes& MetaButtonsCodes::find(const KeyCode& keyCode, const KeyCode& modifierCode) const
{

    std::function<bool(const MetaButtonCodes*)> comparator = [&keyCode, &modifierCode](
        const MetaButtonCodes* metaButton)
    {
        return metaButton->containsKeys(keyCode, modifierCode);
    };

    std::list<const MetaButtonCodes*>::const_iterator button = std::find_if(buttons.begin(), buttons.end(), comparator);

    if (button == buttons.end())
    {
        std::cerr << "failed to find metaButton={ keyCode={" << keyCode << "} modifierCode={" << modifierCode << "}}"
                  << endl;
    }
    assert(button != buttons.end());

    return **button;
}
// ----------------------------------------------------------------------
MetaButtonsCodes::~MetaButtonsCodes()
{
}
// ----------------------------------------------------------------------
Button::Button(const KeyCode& key) :
    mKey(&key)
{
}
// ----------------------------------------------------------------------
Button::~Button()
{
}
// ----------------------------------------------------------------------
Button& Button::operator=(const Button& other)
{
    mKey = other.mKey;
    return *this;
}
// ----------------------------------------------------------------------
AxisRotaryButtonCodes::AxisRotaryButtonCodes() :
    off(0x06, "OFF", ""),
    x(0x11, "X", ""),
    y(0x12, "Y", ""),
    z(0x13, "Z", ""),
    a(0x14, "A", ""),
    b(0x15, "B", ""),
    c(0x16, "C", ""),
    undefined(0x00, "", ""),
    codeMap{
        {off.code,       &off},
        {x.code,         &x},
        {y.code,         &y},
        {z.code,         &z},
        {a.code,         &a},
        {b.code,         &b},
        {c.code,         &c},
        {undefined.code, &undefined}
    }
{
}
// ----------------------------------------------------------------------
FeedRotaryButtonCodes::FeedRotaryButtonCodes() :
    percent_2(0x0d, "0.001", "2%"),
    percent_5(0x0e, "0.01", "5%"),
    percent_10(0x0f, "0.1", "10%"),
    percent_30(0x10, "1", "30%"),
    percent_60(0x1a, "5", "60%"),
    percent_100(0x1b, "10", "100%"),
    lead(0x1c, "Lead", ""),  // user jasenk2 seem to need 0x9b for xhc-whb06-4 see : https://github.com/LinuxCNC/linuxcnc/pull/987
    // solution added in this file for use both know keycodes (0x1c) + Jasenk (0x9b)
    undefined(0x00, "", ""),
    codeMap{
        {percent_2.code,   &percent_2},
        {percent_5.code,   &percent_5},
        {percent_10.code,  &percent_10},
        {percent_30.code,  &percent_30},
        {percent_60.code,  &percent_60},
        {percent_100.code, &percent_100},
        {lead.code,        &lead},
        {undefined.code,   &undefined}
    }
{
}
// ----------------------------------------------------------------------
ButtonsCode::ButtonsCode() :
    reset(0x01, "reset", "macro-11"),
    stop(0x02, "stop", "macro-12"),
    start(0x03, "start-pause", "macro-13"),
    feed_plus(0x04, "feed-plus", "macro-1"),
    feed_minus(0x05, "feed-minus", "macro-2"),
    spindle_plus(0x06, "spindle-plus", "macro-3"),
    spindle_minus(0x07, "spindle-minus", "macro-4"),
    machine_home(0x08, "m-home", "macro-5"),
    safe_z(0x09, "safe-z", "macro-6"),
    workpiece_home(0x0a, "w-home", "macro-7"),
    spindle_on_off(0x0b, "s-on-off", "macro-8"),
    function(0x0c, "fn", "<unused>"),
    probe_z(0x0d, "probe-z", "macro-9"),
    macro10(0x10, "macro-10", "macro-14"),
    continuous(0x0e, "mode-continuous", "macro-15"),
    step(0x0f, "mode-step", "macro-16"),
    undefined(0x00, "", ""),
    codeMap{
        {reset.code,                  &reset},
        {stop.code,                   &stop},
        {start.code,                  &start},
        {feed_plus.code,              &feed_plus},
        {feed_minus.code,             &feed_minus},
        {spindle_plus.code,           &spindle_plus},
        {spindle_minus.code,          &spindle_minus},
        {machine_home.code,           &machine_home},
        {safe_z.code,                 &safe_z},
        {workpiece_home.code,         &workpiece_home},
        {spindle_on_off.code,         &spindle_on_off},
        {function.code,               &function},
        {probe_z.code,                &probe_z},
        {macro10.code,                &macro10},
        {continuous.code,             &continuous},
        {step.code,                   &step},
        {undefined.code,              &undefined}
    }
{
}
// ----------------------------------------------------------------------
const KeyCode& ButtonsCode::getKeyCode(uint8_t keyCode) const
{
    const KeyCode* buttonKeyCode = reinterpret_cast<const KeyCode*>(this);

    while (buttonKeyCode->code != 0)
    {
        if (buttonKeyCode->code == keyCode)
        {
            break;
        }
        buttonKeyCode++;
    }

    assert(nullptr != buttonKeyCode);

    return *buttonKeyCode;
}
// ----------------------------------------------------------------------
const KeyCode& Button::keyCode() const
{
    return *mKey;
}
// ----------------------------------------------------------------------
bool Button::setKeyCode(const KeyCode& keyCode)
{
    bool isNewButton = *mKey != keyCode;
    mKey = &keyCode;
    return isNewButton;
}
// ----------------------------------------------------------------------
std::ostream& operator<<(std::ostream& os, const Button& data)
{
    std::ios init(NULL);
    init.copyfmt(os);
    os << "{key=" << data.keyCode() << "}";
    os.copyfmt(init);
    return os;
}
// ----------------------------------------------------------------------
ToggleButton::ToggleButton(const KeyCode& key, const KeyCode& modifier) :
    Button(key),
    mModifier(&modifier)
{
}
// ----------------------------------------------------------------------
ToggleButton::~ToggleButton()
{
}
// ----------------------------------------------------------------------
std::ostream& operator<<(std::ostream& os, const ToggleButton& data)
{
    std::ios init(NULL);
    init.copyfmt(os);
    os << "{" << *static_cast<const Button*>(&data) << " modifier=" << data.modifierCode() << "}";
    os.copyfmt(init);
    return os;
}
// ----------------------------------------------------------------------
ToggleButton& ToggleButton::operator=(const ToggleButton& other)
{
    Button::operator=(other);
    mModifier = other.mModifier;
    return *this;
}
// ----------------------------------------------------------------------
const KeyCode& ToggleButton::modifierCode() const
{
    return *mModifier;
}
// ----------------------------------------------------------------------
void ToggleButton::setModifierCode(KeyCode& modifierCode)
{
    mModifier = &modifierCode;
}
// ----------------------------------------------------------------------
bool ToggleButton::containsKeys(const KeyCode& key, const KeyCode& modifier) const
{
    return ((key.code == mKey->code) && (modifier.code == mModifier->code));
}
// ----------------------------------------------------------------------
RotaryButton::RotaryButton(const KeyCode& keyCode) :
    Button(keyCode)
{
}
// ----------------------------------------------------------------------
RotaryButton::~RotaryButton()
{
}
// ----------------------------------------------------------------------
std::ostream& operator<<(std::ostream& os, const RotaryButton& data)
{
    std::ios init(NULL);
    init.copyfmt(os);
    os << "{" << *static_cast<const Button*>(&data) << "}";
    os.copyfmt(init);
    return os;
}
// ----------------------------------------------------------------------
RotaryButton& RotaryButton::operator=(const RotaryButton& other)
{
    Button::operator=(other);
    return *this;
}
// ----------------------------------------------------------------------
const HandwheelStepModeStepSize       FeedRotaryButton::mStepSizeMapper;
const HandwheelMpgModeStepSize        FeedRotaryButton::mMpgSizeMapper;
const HandwheelLeadModeStepSize       FeedRotaryButton::mLeadSizeMapper;
const HandwheelConModeStepSize        FeedRotaryButton::mConSizeMapper;

const std::map<const KeyCode*, HandwheelStepModeStepSize::PositionNameIndex>       FeedRotaryButton::mStepKeycodeLut{
    {&KeyCodes::Feed.percent_2,   HandwheelStepModeStepSize::PositionNameIndex::RotaryButton0001},
    {&KeyCodes::Feed.percent_5,   HandwheelStepModeStepSize::PositionNameIndex::RotaryButton0010},
    {&KeyCodes::Feed.percent_10,  HandwheelStepModeStepSize::PositionNameIndex::RotaryButton0100},
    {&KeyCodes::Feed.percent_30,  HandwheelStepModeStepSize::PositionNameIndex::RotaryButton100},
    {&KeyCodes::Feed.percent_60,  HandwheelStepModeStepSize::PositionNameIndex::RotaryButton500},
    {&KeyCodes::Feed.percent_100, HandwheelStepModeStepSize::PositionNameIndex::RotaryButton1000},
    {&KeyCodes::Feed.lead,        HandwheelStepModeStepSize::PositionNameIndex::RotaryButtonUndefined}
};
const std::map<const KeyCode*, HandwheelMpgModeStepSize::PositionNameIndex> FeedRotaryButton::mMpgKeycodeLut{
    {&KeyCodes::Feed.percent_2,   HandwheelMpgModeStepSize::PositionNameIndex::RotaryButton2percent},
    {&KeyCodes::Feed.percent_5,   HandwheelMpgModeStepSize::PositionNameIndex::RotaryButton5percent},
    {&KeyCodes::Feed.percent_10,  HandwheelMpgModeStepSize::PositionNameIndex::RotaryButton10percent},
    {&KeyCodes::Feed.percent_30,  HandwheelMpgModeStepSize::PositionNameIndex::RotaryButton30percent},
    {&KeyCodes::Feed.percent_60,  HandwheelMpgModeStepSize::PositionNameIndex::RotaryButton60percent},
    {&KeyCodes::Feed.percent_100, HandwheelMpgModeStepSize::PositionNameIndex::RotaryButton100percent},
    {&KeyCodes::Feed.lead,        HandwheelMpgModeStepSize::PositionNameIndex::RotaryButtonUndefined}
};
const std::map<const KeyCode*, HandwheelConModeStepSize::PositionNameIndex> FeedRotaryButton::mConKeycodeLut{
    {&KeyCodes::Feed.percent_2,   HandwheelConModeStepSize::PositionNameIndex::RotaryButton2percent},
    {&KeyCodes::Feed.percent_5,   HandwheelConModeStepSize::PositionNameIndex::RotaryButton5percent},
    {&KeyCodes::Feed.percent_10,  HandwheelConModeStepSize::PositionNameIndex::RotaryButton10percent},
    {&KeyCodes::Feed.percent_30,  HandwheelConModeStepSize::PositionNameIndex::RotaryButton30percent},
    {&KeyCodes::Feed.percent_60,  HandwheelConModeStepSize::PositionNameIndex::RotaryButton60percent},
    {&KeyCodes::Feed.percent_100, HandwheelConModeStepSize::PositionNameIndex::RotaryButton100percent},
    {&KeyCodes::Feed.lead,        HandwheelConModeStepSize::PositionNameIndex::RotaryButtonUndefined}
};
const std::map<const KeyCode*, HandwheelLeadModeStepSize::PositionNameIndex>       FeedRotaryButton::mLeadKeycodeLut{
    {&KeyCodes::Feed.percent_2,   HandwheelLeadModeStepSize::PositionNameIndex::UNDEFINED},
    {&KeyCodes::Feed.percent_5,   HandwheelLeadModeStepSize::PositionNameIndex::UNDEFINED},
    {&KeyCodes::Feed.percent_10,  HandwheelLeadModeStepSize::PositionNameIndex::UNDEFINED},
    {&KeyCodes::Feed.percent_30,  HandwheelLeadModeStepSize::PositionNameIndex::UNDEFINED},
    {&KeyCodes::Feed.percent_60,  HandwheelLeadModeStepSize::PositionNameIndex::UNDEFINED},
    {&KeyCodes::Feed.percent_100, HandwheelLeadModeStepSize::PositionNameIndex::UNDEFINED},
    {&KeyCodes::Feed.lead,        HandwheelLeadModeStepSize::PositionNameIndex::LEAD}
};
// ----------------------------------------------------------------------
FeedRotaryButton::FeedRotaryButton(const KeyCode& keyCode,
    HandwheelStepmodes::Mode stepMode,
    KeyEventListener* listener) :
    RotaryButton(keyCode),
    mStepMode(stepMode),
    mIsPermitted(false),
    mStepSize(0),
    mIsStepMode_5_10(false),
    mEventListener(listener)
{
    (void)mEventListener;
}
// ----------------------------------------------------------------------
FeedRotaryButton::~FeedRotaryButton()
{
}
// ----------------------------------------------------------------------
std::ostream& operator<<(std::ostream& os, const FeedRotaryButton& data)
{
    std::ios init(NULL);
    init.copyfmt(os);
    os << "{" << *static_cast<const RotaryButton*>(&data) << " "
       << "isPermitted=" << ((data.isPermitted()) ? "TRUE" : "FALSE") << " "
       << "stepSize=" << data.stepSize() << " "
       << "stepMode=0x" << std::setfill('0') << std::hex << std::setw(2)
       << static_cast<int16_t>(data.stepMode()) << "}";
    os.copyfmt(init);
    return os;
}
// ----------------------------------------------------------------------
FeedRotaryButton& FeedRotaryButton::operator=(const FeedRotaryButton& other)
{
    RotaryButton::operator=(other);
    mStepMode = other.mStepMode;
    return *this;
}
// ----------------------------------------------------------------------
bool FeedRotaryButton::setKeyCode(const KeyCode& keyCode)
{
    bool hasChanged = Button::setKeyCode(keyCode);
    update();
    return hasChanged;
}
// ----------------------------------------------------------------------
void FeedRotaryButton::setStepMode(HandwheelStepmodes::Mode stepMode)
{
    mStepMode = stepMode;
    update();
}
// ----------------------------------------------------------------------
HandwheelStepmodes::Mode FeedRotaryButton::stepMode() const
{
    return mStepMode;
}
// ----------------------------------------------------------------------
float FeedRotaryButton::stepSize() const
{
    return mStepSize;
}
// ----------------------------------------------------------------------
void FeedRotaryButton::update()
{
    if (*mKey == KeyCodes::Feed.undefined)
    {
        mIsPermitted = false;
        return;
    }

    if (*mKey == KeyCodes::Feed.lead)
    {
        mStepSize    = mLeadSizeMapper.getStepSize(HandwheelLeadModeStepSize::PositionNameIndex::LEAD);
        mIsPermitted = mLeadSizeMapper.isPermitted(HandwheelLeadModeStepSize::PositionNameIndex::LEAD);
    }
    else if (mStepMode == HandwheelStepmodes::Mode::MPG)
    {
        auto enumValue = mMpgKeycodeLut.find(mKey);
        assert(enumValue != mMpgKeycodeLut.end());
        auto second = enumValue->second;
        mStepSize    = mMpgSizeMapper.getStepSize(second);
        mIsPermitted = mMpgSizeMapper.isPermitted(second);
    }
    else if (mStepMode == HandwheelStepmodes::Mode::STEP)
    {
        auto enumValue = mStepKeycodeLut.find(mKey);
        assert(enumValue != mStepKeycodeLut.end());
        auto second = enumValue->second;
        mStepSize    = mStepSizeMapper.getStepSize(second);
        mIsPermitted = mStepSizeMapper.isPermitted(second);
        
        if (mIsStepMode_5_10 && mStepSize > 2) {mStepSize    = 0;}             // TODO DOES NOT WORK bool variable seems to be not synched inside pendant.h
        
    }
    else if (mStepMode == HandwheelStepmodes::Mode::CON)
    {
        auto enumValue = mConKeycodeLut.find(mKey);
        assert(enumValue != mConKeycodeLut.end());
        auto second = enumValue->second;
        mStepSize    = mConSizeMapper.getStepSize(second);
        mIsPermitted = mConSizeMapper.isPermitted(second);
    }
    else
    {
        assert(false);
    }
}
// ----------------------------------------------------------------------
bool FeedRotaryButton::isPermitted() const
{
    return mIsPermitted;
}
// ----------------------------------------------------------------------
AxisRotaryButton::AxisRotaryButton(const KeyCode& keyCode, KeyEventListener* listener) :
    RotaryButton(keyCode),
    mEventListener(listener)
{
    (void)mEventListener;
}
// ----------------------------------------------------------------------
AxisRotaryButton::~AxisRotaryButton()
{
}
// ----------------------------------------------------------------------
std::ostream& operator<<(std::ostream& os, const AxisRotaryButton& data)
{
    std::ios init(NULL);
    init.copyfmt(os);
    os << "{" << *static_cast<const RotaryButton*>(&data)
       << " isPermitted=" << ((data.isPermitted()) ? "TRUE" : "FALSE") << "}";
    os.copyfmt(init);
    return os;
}
// ----------------------------------------------------------------------
AxisRotaryButton& AxisRotaryButton::operator=(const AxisRotaryButton& other)
{
    RotaryButton::operator=(other);
    return *this;
}
// ----------------------------------------------------------------------
bool AxisRotaryButton::isPermitted() const
{
    return (*mKey != KeyCodes::Axis.undefined) && (*mKey != KeyCodes::Axis.off);
}
// ----------------------------------------------------------------------
Handwheel::Handwheel(const FeedRotaryButton& feedButton, KeyEventListener* listener) :
    mCounters(),
    mFeedButton(feedButton),
    mEventListener(listener),
    mWheelCout(&std::cout),
    mPrefix("pndnt ")
{
    (void)mFeedButton;
}
// ----------------------------------------------------------------------
Handwheel::~Handwheel()
{
}
// ----------------------------------------------------------------------
std::ostream& operator<<(std::ostream& os, const Handwheel& data)
{
    std::ios init(NULL);
    init.copyfmt(os);
    os << "{counters=" << data.counters() << "}";
    return os;
}
// ----------------------------------------------------------------------
const HandWheelCounters& Handwheel::counters() const
{
    Handwheel* self = const_cast<Handwheel*>(this);
    return static_cast<const HandWheelCounters&>(self->counters());
}
// ----------------------------------------------------------------------
void Handwheel::setEnabled(bool enabled)
{
    mIsEnabled = enabled;
}
// ----------------------------------------------------------------------
HandWheelCounters& Handwheel::counters()
{
    return mCounters;
}
// ----------------------------------------------------------------------
void Handwheel::enableVerbose(bool enable)
{
    if (enable)
    {
        mWheelCout = &std::cout;
    }
    else
    {
        mWheelCout = &mDevNull;
    }
}
// ----------------------------------------------------------------------
void Handwheel::setMode(HandWheelCounters::CounterNameToIndex activeCounterMode)
{
    mCounters.setActiveCounter(activeCounterMode);
}
// ----------------------------------------------------------------------
void Handwheel::count(int8_t delta)
{
    assert(mEventListener != nullptr);

    if (mIsEnabled)
    {
        mCounters.count(delta);
        mEventListener->onJogDialEvent(mCounters, delta);
    }

    std::ios init(NULL);
    init.copyfmt(*mWheelCout);
    *mWheelCout << mPrefix << "wheel total counts " << std::setfill(' ') << std::setw(5) << mCounters
                << endl;
    mWheelCout->copyfmt(init);
}
// ----------------------------------------------------------------------
ButtonsState::ButtonsState(KeyEventListener* listener, const ButtonsState* previousState) :
    mPressedButtons(),
    mCurrentMetaButton(&KeyCodes::Meta.undefined),
    mAxisButton(KeyCodes::Axis.undefined, listener),
    mFeedButton(KeyCodes::Feed.undefined, HandwheelStepmodes::Mode::MPG, listener),
    mPreviousState(previousState),
    mEventListener(listener)
{
    (void)mPreviousState;
}
// ----------------------------------------------------------------------
ButtonsState::~ButtonsState()
{
}
// ----------------------------------------------------------------------
void ButtonsState::update(const KeyCode& keyCode,
                          const KeyCode& modifierCode,
                          const KeyCode& axisButton,
                          const KeyCode& feedButton)
{
    //! propagate push button events
    const MetaButtonCodes& newButton = KeyCodes::Meta.find(keyCode, modifierCode);
    if (*mCurrentMetaButton != newButton)
    {
        if (*mCurrentMetaButton != KeyCodes::Meta.undefined)
        {
            if (mEventListener != nullptr)
            {
                mEventListener->onButtonReleasedEvent(*mCurrentMetaButton);
            }
        }

        mCurrentMetaButton = &newButton;
        if (*mCurrentMetaButton != KeyCodes::Meta.undefined)
        {
            if (mEventListener != nullptr)
            {
                mEventListener->onButtonPressedEvent(*mCurrentMetaButton);
            }
        }
    }

    //! propagate axis rotary button events
    const KeyCode& oldAxisKeyCode = mAxisButton.keyCode();
    if (mAxisButton.setKeyCode(axisButton))
    {
        mEventListener->onAxisInactiveEvent(oldAxisKeyCode);
        mEventListener->onAxisActiveEvent(mAxisButton.keyCode());
    }

    //! propagate feed rotary button events
    const KeyCode& oldFeedKeyCode = mFeedButton.keyCode();
    if (mFeedButton.setKeyCode(feedButton))
    {
        mEventListener->onFeedInactiveEvent(oldFeedKeyCode);
        mEventListener->onFeedActiveEvent(mFeedButton.keyCode());
    }
}
// ----------------------------------------------------------------------
std::ostream& operator<<(std::ostream& os, const ButtonsState& data)
{
    std::ios init(NULL);
    init.copyfmt(os);

    os << "{pressed buttons=";
    for (const KeyCode* pb : data.pressedButtons())
    {
        assert(pb != nullptr);
        os << *pb << " ";
    }
    os << " metaButton=" << *data.currentMetaButton()
       << " axisButton=" << data.axisButton()
       << " feedButton=" << data.feedButton() << "}";
    os.copyfmt(init);
    return os;
}
// ----------------------------------------------------------------------
ButtonsState& ButtonsState::operator=(const ButtonsState& other)
{
    mPressedButtons    = other.mPressedButtons;
    mCurrentMetaButton = other.mCurrentMetaButton;
    mAxisButton        = other.mAxisButton;
    mFeedButton        = other.mFeedButton;
    return *this;
}
// ----------------------------------------------------------------------
void ButtonsState::clearPressedButtons()
{
    mPressedButtons.clear();
}
// ----------------------------------------------------------------------
const std::list<const KeyCode*>& ButtonsState::pressedButtons() const
{
    return mPressedButtons;
}
// ----------------------------------------------------------------------
const MetaButtonCodes* ButtonsState::currentMetaButton() const
{
    return mCurrentMetaButton;
}
// ----------------------------------------------------------------------
const AxisRotaryButton& ButtonsState::axisButton() const
{
    return mAxisButton;
}
// ----------------------------------------------------------------------
const FeedRotaryButton& ButtonsState::feedButton() const
{
    return mFeedButton;
}
// ----------------------------------------------------------------------
FeedRotaryButton& ButtonsState::feedButton()
{
    return mFeedButton;
}
// ----------------------------------------------------------------------
Pendant::Pendant(Hal& hal, UsbOutPackageData& displayOutData) :
    mHal(hal),
    mPreviousButtonsState(this),
    mCurrentButtonsState(this, &mPreviousButtonsState),
    mHandWheel(mCurrentButtonsState.feedButton(), this),
    mDisplay(mCurrentButtonsState, hal, displayOutData),
    mPrefix("pndnt "),
    mPendantCout(&std::cout)
{
}
// ----------------------------------------------------------------------
Pendant::~Pendant()
{
}
// ----------------------------------------------------------------------
std::ostream& operator<<(std::ostream& os, const Pendant& data)
{
    std::ios init(NULL);
    init.copyfmt(os);

    os << "{currentButtonState=" << data.currentButtonsState() << " "
       << "previousButtonState=" << data.previousButtonsState() << " "
       << "wheel= " << data.handWheel() << "}";
    return os;
}
// ----------------------------------------------------------------------
void Pendant::processEvent(uint8_t keyCode,
                           uint8_t modifierCode,
                           uint8_t rotaryButtonAxisKeyCode,
                           uint8_t rotaryButtonFeedKeyCode,
                           int8_t handWheelStepCount)
{

    shiftButtonState();

    auto key      = KeyCodes::Buttons.codeMap.find(keyCode);
    auto modifier = KeyCodes::Buttons.codeMap.find(modifierCode);
    auto axis     = KeyCodes::Axis.codeMap.find(rotaryButtonAxisKeyCode);
    auto feed     = KeyCodes::Feed.codeMap.find(rotaryButtonFeedKeyCode);

    if (key == KeyCodes::Buttons.codeMap.end())
    {
        *mPendantCout << mPrefix << "failed to interpret key code keyCode={" << keyCode << "}" << endl;
        modifier = KeyCodes::Buttons.codeMap.find(0);
    }
    if (modifier == KeyCodes::Buttons.codeMap.end())
    {
        *mPendantCout << mPrefix << "failed to interpret modifier code keyCode={" << modifierCode << "}" << endl;
        modifier = KeyCodes::Buttons.codeMap.find(0);
    }
    if (axis == KeyCodes::Axis.codeMap.end())
    {
        *mPendantCout << mPrefix << "failed to interpret axis code axisCode={" << rotaryButtonAxisKeyCode << "}" << endl;
        axis = KeyCodes::Axis.codeMap.find(0);
    }
    if (feed == KeyCodes::Feed.codeMap.end())
    {
        *mPendantCout << mPrefix << "failed to interpret axis code feed axisCode={" << rotaryButtonFeedKeyCode << "}" << endl;
        feed = KeyCodes::Feed.codeMap.find(0);
    }

    processEvent(*key->second, *modifier->second, *axis->second, *feed->second, handWheelStepCount);
}
// ----------------------------------------------------------------------
void Pendant::processEvent(const KeyCode& keyCode,
                           const KeyCode& modifierCode,
                           const KeyCode& rotaryButtonAxisKeyCode,
                           const KeyCode& rotaryButtonFeedKeyCode,
                           int8_t handWheelStepCount)
{
    mHandWheel.setEnabled(mHal.getIsMachineOn());
    mCurrentButtonsState.update(keyCode, modifierCode, rotaryButtonAxisKeyCode, rotaryButtonFeedKeyCode);
    mHandWheel.count(handWheelStepCount);
    mDisplay.updateData();
}
// ----------------------------------------------------------------------
void Pendant::updateDisplayData()
{
    mDisplay.updateData();
}
// ----------------------------------------------------------------------
void Pendant::clearDisplayData()
{
    mDisplay.clearData();
}
// ----------------------------------------------------------------------
void Pendant::shiftButtonState()
{
    mPreviousButtonsState = mCurrentButtonsState;
    mCurrentButtonsState.clearPressedButtons();
}
// ----------------------------------------------------------------------
void Pendant::enableVerbose(bool enable)
{
    mHandWheel.enableVerbose(enable);
    if (enable)
    {
        mPendantCout = &std::cout;
    }
    else
    {
        mPendantCout = &mDevNull;
    }
}
// ----------------------------------------------------------------------
const ButtonsState& Pendant::currentButtonsState() const
{
    return mCurrentButtonsState;
}
// ----------------------------------------------------------------------
const ButtonsState& Pendant::previousButtonsState() const
{
    return mPreviousButtonsState;
}
// ----------------------------------------------------------------------
const Handwheel& Pendant::handWheel() const
{
    return mHandWheel;
}
// ----------------------------------------------------------------------
Handwheel& Pendant::handWheel()
{
    return const_cast<Handwheel&>(const_cast<const Pendant*>(this)->handWheel());
}
// ----------------------------------------------------------------------
bool Pendant::onButtonPressedEvent(const MetaButtonCodes& metaButton)
{
    *mPendantCout << mPrefix << "button pressed  event metaButton=" << metaButton << endl;
    bool isHandled = false;
    if (metaButton == KeyCodes::Meta.reset)
    {
        mHal.setReset(true);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.stop)
    {
        mHal.setStop(true);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.start)
    {
        mHal.setStart(true);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.feed_plus)
    {
        mHal.setFeedPlus(true);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.feed_minus)
    {
        mHal.setFeedMinus(true);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.spindle_plus)
    {
        mHal.setSpindleOverridePlus(true);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.spindle_minus)
    {
        mHal.setSpindleOverrideMinus(true);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.function)
    {
        mHal.setFunction(true);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.machine_home)
    {
        mHal.setMachineHomingAll(true);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.safe_z)
    {
        mHal.setSafeZ(true);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.workpiece_home)
    {
        mHal.setWorkpieceHome(true);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.spindle_on_off)
    {
        mHal.toggleSpindleOnOff(true);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.probe_z)
    {
        mHal.setProbeZ(true);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.macro10)
    {
        mHal.setMacro10(true);                         // Hardcoded Absolute/relative Dro
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.continuous)
    {
        mCurrentButtonsState.feedButton().setStepMode(HandwheelStepmodes::Mode::CON);
        mHal.setConMode(true);
        dispatchFeedValueToHal();
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.step)
    {
        mCurrentButtonsState.feedButton().setStepMode(HandwheelStepmodes::Mode::STEP);
        mHal.setStepMode(true);
        dispatchFeedValueToHal();
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.macro11)
    {
        mHal.setMacro11(true);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.macro12)
    {
        mHal.setMacro12(true);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.macro13)
    {
        mHal.setMacro13(true);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.macro1)
    {
        mHal.setMacro1(true);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.macro3)
    {
        mHal.setMacro3(true);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.macro4)
    {
        mHal.setMacro4(true);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.macro5)
    {
        mHal.setMacro5(true);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.macro6)
    {
        mHal.setMacro6(true);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.macro7)
    {
        mHal.setMacro7(true);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.macro8)
    {
        mHal.toggleSpindleDirection(true);
        mHal.setMacro8(true);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.macro9)
    {
        mHal.setMacro9(true);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.macro14)
    {
        mHal.setMacro14(true);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.macro15)
    {
        mHal.toggleFloodOnOff(true);
        mHal.setMacro15(true);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.macro16)
    {
        mHal.toggleMistOnOff(true);
        mHal.setMacro16(true);
        isHandled = true;
    }

    mDisplay.onButtonPressedEvent(metaButton);
    return isHandled;
}
// ----------------------------------------------------------------------
bool Pendant::onButtonReleasedEvent(const MetaButtonCodes& metaButton)
{
    *mPendantCout << mPrefix << "button released event metaButton=" << metaButton << endl;
    bool isHandled = false;
    if (metaButton == KeyCodes::Meta.reset)
    {
        mHal.setReset(false);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.stop)
    {
        mHal.setStop(false);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.start)
    {
        mHal.setStart(false);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.feed_plus)
    {
        mHal.setFeedPlus(false);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.feed_minus)
    {
        mHal.setFeedMinus(false);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.spindle_plus)
    {
        mHal.setSpindleOverridePlus(false);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.spindle_minus)
    {
        mHal.setSpindleOverrideMinus(false);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.function)
    {
        mHal.setFunction(false);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.machine_home)
    {
        mHal.setMachineHomingAll(false);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.safe_z)
    {
        mHal.setSafeZ(false);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.workpiece_home)
    {
        mHal.setWorkpieceHome(false);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.spindle_on_off)
    {
        mHal.toggleSpindleOnOff(false);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.probe_z)
    {
        mHal.setProbeZ(false);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.macro10)
    {
        mHal.setMacro10(false);                        // Hardcoded Absolute/relative Dro
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.continuous)
    {
        mHal.setConMode(false);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.step)
    {
        mHal.setStepMode(false);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.macro11)
    {
        mHal.setMacro11(false);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.macro12)
    {
        mHal.setMacro12(false);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.macro13)
    {
        mHal.setMacro13(false);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.macro1)
    {
        mHal.setMacro1(false);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.macro3)
    {
        mHal.setMacro3(false);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.macro4)
    {
        mHal.setMacro4(false);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.macro5)
    {
        mHal.setMacro5(false);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.macro6)
    {
        mHal.setMacro6(false);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.macro7)
    {
        mHal.setMacro7(false);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.macro8)
    {
        mHal.toggleSpindleDirection(false);
        mHal.setMacro8(false);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.macro9)
    {
        mHal.setMacro9(false);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.macro14)
    {
        mHal.setMacro14(false);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.macro15)
    {
        mHal.toggleFloodOnOff(false);
        mHal.setMacro15(false);
        isHandled = true;
    }
    else if (metaButton == KeyCodes::Meta.macro16)
    {
        mHal.toggleMistOnOff(false);
        mHal.setMacro16(false);
        isHandled = true;
    }

    mDisplay.onButtonReleasedEvent(metaButton);
    return isHandled;
}
// ----------------------------------------------------------------------
void Pendant::onAxisActiveEvent(const KeyCode& axis)
{
    *mPendantCout << mPrefix << "axis   active   event axis=" << axis
                  << " axisButton=" << mCurrentButtonsState.axisButton() << endl;
    dispatchAxisEventToHandwheel(axis, true);
    dispatchAxisEventToHal(axis, true);
    mDisplay.onAxisActiveEvent(axis);
}
// ----------------------------------------------------------------------
void Pendant::onAxisInactiveEvent(const KeyCode& axis)
{
    *mPendantCout << mPrefix << "axis   inactive event axis=" << axis
                  << " axisButton=" << mCurrentButtonsState.axisButton() << endl;
    dispatchAxisEventToHandwheel(axis, false);
    dispatchAxisEventToHal(axis, false);
    mDisplay.onAxisInactiveEvent(axis);
}
// ----------------------------------------------------------------------
void Pendant::onFeedActiveEvent(const KeyCode& feed)
{
    (*mPendantCout) << mPrefix << "feed   active   event feed=" << feed
                    << " feedButton=" << mCurrentButtonsState.feedButton() << endl;

    dispatchFeedEventToHandwheel(feed, true);
    dispatchFeedValueToHal();
    dispatchActiveFeedToHal(feed, true);
    mDisplay.onFeedActiveEvent(feed);
}
// ----------------------------------------------------------------------
void Pendant::dispatchFeedEventToHandwheel(const KeyCode& feed, bool isActive)
{
    if (feed.code == KeyCodes::Feed.lead.code || feed.code == 0x9b) // user jasenk2 seem to need 0x9b for xhc-whb06-4 see : https://github.com/LinuxCNC/linuxcnc/pull/987
    {
        mHandWheel.counters().enableLeadCounter(isActive);
    }
}
// ----------------------------------------------------------------------
void Pendant::dispatchActiveFeedToHal(const KeyCode& feed, bool isActive)
{
    if (feed.code == KeyCodes::Feed.percent_2.code)
    {
        mHal.setFeedValueSelected2(isActive);
    }
    else if (feed.code == KeyCodes::Feed.percent_5.code)
    {
        mHal.setFeedValueSelected5(isActive);
    }
    else if (feed.code == KeyCodes::Feed.percent_10.code)
    {
        mHal.setFeedValueSelected10(isActive);
    }
    else if (feed.code == KeyCodes::Feed.percent_30.code)
    {
        mHal.setFeedValueSelected30(isActive);
    }
    else if (feed.code == KeyCodes::Feed.percent_60.code)
    {
        mHal.setFeedValueSelected60(isActive);
    }
    else if (feed.code == KeyCodes::Feed.percent_100.code)
    {
        mHal.setFeedValueSelected100(isActive);
    }
    else if (feed.code == KeyCodes::Feed.lead.code || feed.code == 0x9b) // user jasenk2 seem to need 0x9b for xhc-whb06-4 see : https://github.com/LinuxCNC/linuxcnc/pull/987
    {
        mHal.setFeedValueSelectedLead(isActive);
        mCurrentButtonsState.feedButton().setStepMode(HandwheelStepmodes::Mode::MPG);
    }
}
// ----------------------------------------------------------------------
void Pendant::dispatchFeedValueToHal()
{
    // on feed rotary button change
    FeedRotaryButton& feedButton = mCurrentButtonsState.feedButton();
    if (feedButton.isPermitted())
    {
        float axisJogStepSize = 0;
        if (feedButton.stepMode() == HandwheelStepmodes::Mode::STEP)
        {
            mHal.setStepMode(true);
            mHal.setMpgMode(false);
            mHal.setConMode(false);
            mHal.setFeedOverrideScale(0);
            axisJogStepSize = feedButton.stepSize();
        }
        else if (feedButton.stepMode() == HandwheelStepmodes::Mode::MPG)
        {
            mHal.setStepMode(false);
            mHal.setMpgMode(true);
            mHal.setConMode(false);
            mHal.setFeedOverrideScale(0);                                                  // NO SEND MOVE IN MPG MODE only used for Feed override
        }
        else if (feedButton.stepMode() == HandwheelStepmodes::Mode::CON)
        {
            mHal.setStepMode(false);
            mHal.setMpgMode(false);
            mHal.setConMode(true);
            mHal.setFeedOverrideScale(0);
            axisJogStepSize = feedButton.stepSize() * 0.0001 * mHal.getFeedOverrideMaxVel();
        }
        else
        {
        }
        mHal.setStepSize(axisJogStepSize);
    }
    else
    {
        mHal.setStepSize(0);
    }
}
// ----------------------------------------------------------------------
void Pendant::onFeedInactiveEvent(const KeyCode& feed)
{
    *mPendantCout << mPrefix << "feed   inactive event feed=" << feed
                  << " feedButton=" << mCurrentButtonsState.feedButton() << endl;
    dispatchFeedEventToHandwheel(feed, false);
    dispatchActiveFeedToHal(feed, false);
    mDisplay.onFeedInactiveEvent(feed);
}
// ----------------------------------------------------------------------
bool Pendant::onJogDialEvent(const HandWheelCounters& counters, int8_t delta)
{

    FeedRotaryButton& feedButton = mCurrentButtonsState.feedButton();

    if (HandWheelCounters::CounterNameToIndex::UNDEFINED != counters.activeCounter()) // && 0 != counters.counts())
    {
        *mPendantCout << mPrefix << "wheel  event " << counters.counts() << endl;

        if (0 != delta)
        {
            if (counters.isLeadCounterActive() && mIsLeadModeSpindle)
            {   // Spindle override mode
                if (delta > 0)
                {
                    mHal.toggleSpindleOverrideIncrease();
                }
                else
                {
                    mHal.toggleSpindleOverrideDecrease();
                }
            }
            else if (!counters.isLeadCounterActive() && mIsLeadModeFeed && feedButton.stepMode() == HandwheelStepmodes::Mode::MPG)
            {      // FeedRate override mode
                   if (delta > 0)
                   {
                       mHal.toggleFeedrateIncrease();
                   }
                   else
                   {
                       mHal.toggleFeedrateDecrease();
                   }
             }
             else if (!counters.isLeadCounterActive() && (feedButton.stepMode() == HandwheelStepmodes::Mode::CON || feedButton.stepMode() == HandwheelStepmodes::Mode::STEP))
             {      // Normal Mode
                    mHal.setJogCounts(counters);
             }
        }
        mDisplay.onJogDialEvent(counters, delta);
        return true;
    }
    return false;
}
// ----------------------------------------------------------------------
void Pendant::dispatchAxisEventToHandwheel(const KeyCode& axis, bool isActive)
{
    if (!isActive)
    {
        mHandWheel.counters().setActiveCounter(HandWheelCounters::CounterNameToIndex::UNDEFINED);
    }
    else if (axis.code == KeyCodes::Axis.off.code)
    {
        mHandWheel.counters().setActiveCounter(HandWheelCounters::CounterNameToIndex::UNDEFINED);
    }
    else if (axis.code == KeyCodes::Axis.x.code)
    {
        mHandWheel.counters().setActiveCounter(HandWheelCounters::CounterNameToIndex::AXIS_X);
    }
    else if (axis.code == KeyCodes::Axis.y.code)
    {
        mHandWheel.counters().setActiveCounter(HandWheelCounters::CounterNameToIndex::AXIS_Y);
    }
    else if (axis.code == KeyCodes::Axis.z.code)
    {
        mHandWheel.counters().setActiveCounter(HandWheelCounters::CounterNameToIndex::AXIS_Z);
    }
    else if (axis.code == KeyCodes::Axis.a.code)
    {
        mHandWheel.counters().setActiveCounter(HandWheelCounters::CounterNameToIndex::AXIS_A);
    }
    else if (axis.code == KeyCodes::Axis.b.code)
    {
        mHandWheel.counters().setActiveCounter(HandWheelCounters::CounterNameToIndex::AXIS_B);
    }
    else if (axis.code == KeyCodes::Axis.c.code)
    {
        mHandWheel.counters().setActiveCounter(HandWheelCounters::CounterNameToIndex::AXIS_C);
    }
    else if (axis.code == KeyCodes::Axis.undefined.code)
    {
        mHandWheel.counters().setActiveCounter(HandWheelCounters::CounterNameToIndex::UNDEFINED);
    }
}
// ----------------------------------------------------------------------
void Pendant::dispatchAxisEventToHal(const KeyCode& axis, bool isActive)
{
    if (axis.code == KeyCodes::Axis.off.code)
    {
        mHal.setNoAxisActive(isActive);
    }
    else if (axis.code == KeyCodes::Axis.x.code)
    {
        mHal.setAxisXActive(isActive);
    }
    else if (axis.code == KeyCodes::Axis.y.code)
    {
        mHal.setAxisYActive(isActive);
    }
    else if (axis.code == KeyCodes::Axis.z.code)
    {
        mHal.setAxisZActive(isActive);
    }
    else if (axis.code == KeyCodes::Axis.a.code)
    {
        mHal.setAxisAActive(isActive);
    }
    else if (axis.code == KeyCodes::Axis.b.code)
    {
        mHal.setAxisBActive(isActive);
    }
    else if (axis.code == KeyCodes::Axis.c.code)
    {
        mHal.setAxisCActive(isActive);
    }
    else if (axis.code == KeyCodes::Axis.undefined.code)
    {
        mHal.setNoAxisActive(isActive);
    }
}
// ----------------------------------------------------------------------
void Pendant::setLeadModeSpindle(bool /*enable*/)
{
    mIsLeadModeSpindle = true;
}
// ----------------------------------------------------------------------
void Pendant::setLeadModeFeed(bool /*enable*/)
{
    mIsLeadModeFeed = true;
}
// ----------------------------------------------------------------------
void Pendant::setStepMode_5_10(bool /*enable*/)
{
    mIsStepMode_5_10 = true;
}
// ----------------------------------------------------------------------
Display::Display(const ButtonsState& currentButtonsState, Hal& hal, UsbOutPackageData& displayData) :
    mCurrentButtonsState(currentButtonsState),
    mHal(hal),
    mDisplayData(displayData),
    mAxisPositionMethod(AxisPositionMethod::RELATIVE),
    mActiveAxisGroup(AxisGroup::XYZ)
{
}
// ----------------------------------------------------------------------
Display::~Display()
{
}
// ----------------------------------------------------------------------
bool Display::onButtonPressedEvent(const MetaButtonCodes& metaButton)
{
    if (metaButton == KeyCodes::Meta.continuous)
    {
        mDisplayData.displayModeFlags.asBitFields.stepMode =
            static_cast<typename std::underlying_type<DisplayIndicatorStepMode::StepMode>::type>(DisplayIndicatorStepMode::StepMode::CON);
        return true;
    }
    else if (metaButton == KeyCodes::Meta.step)
    {
        mDisplayData.displayModeFlags.asBitFields.stepMode =
            static_cast<typename std::underlying_type<DisplayIndicatorStepMode::StepMode>::type>(DisplayIndicatorStepMode::StepMode::STEP);
        return true;
    }
    else if (metaButton == KeyCodes::Meta.macro10)
    {
        if (mAxisPositionMethod == AxisPositionMethod::RELATIVE)
    {
        mAxisPositionMethod = AxisPositionMethod::ABSOLUTE;
        return true;
    }
        else if (mAxisPositionMethod == AxisPositionMethod::ABSOLUTE)
    {
        mAxisPositionMethod = AxisPositionMethod::RELATIVE;
        return true;
    }
    return false;
}
    return false;
}
// ----------------------------------------------------------------------
bool Display::onButtonReleasedEvent(const MetaButtonCodes& /*metaButton*/)
{
    return false;
}
// ----------------------------------------------------------------------
void Display::onAxisActiveEvent(const KeyCode& axis)
{
    if ((axis.code == KeyCodes::Axis.x.code) ||
        (axis.code == KeyCodes::Axis.y.code) ||
        (axis.code == KeyCodes::Axis.z.code))
    {
        mActiveAxisGroup = AxisGroup::XYZ;
    }
    else
    { // a || b || c
        mActiveAxisGroup = AxisGroup::ABC;
    }
}
// ----------------------------------------------------------------------
void Display::onAxisInactiveEvent(const KeyCode& /*axis*/)
{
}
// ----------------------------------------------------------------------
void Display::onFeedActiveEvent(const KeyCode& /*feed*/)
{
    if (mCurrentButtonsState.feedButton().stepMode() == HandwheelStepmodes::Mode::STEP)
    {
        mDisplayData.displayModeFlags.asBitFields.stepMode =
            static_cast<typename std::underlying_type<DisplayIndicatorStepMode::StepMode>::type>(DisplayIndicatorStepMode::StepMode::STEP);
    }
    else if (mCurrentButtonsState.feedButton().stepMode() == HandwheelStepmodes::Mode::MPG)
    {
        mDisplayData.displayModeFlags.asBitFields.stepMode =
            static_cast<typename std::underlying_type<DisplayIndicatorStepMode::StepMode>::type>(DisplayIndicatorStepMode::StepMode::MPG);
    }
    else if (mCurrentButtonsState.feedButton().stepMode() == HandwheelStepmodes::Mode::CON)
    {
        mDisplayData.displayModeFlags.asBitFields.stepMode =
            static_cast<typename std::underlying_type<DisplayIndicatorStepMode::StepMode>::type>(DisplayIndicatorStepMode::StepMode::CON);
    }
}
// ----------------------------------------------------------------------
void Display::onFeedInactiveEvent(const KeyCode& /*feed*/)
{
}
// ----------------------------------------------------------------------
bool Display::onJogDialEvent(const HandWheelCounters& /*counters*/, int8_t /*delta*/)
{
    return false;
}
// ----------------------------------------------------------------------
void Display::updateData()
{
    mDisplayData.displayModeFlags.asBitFields.isReset = !mHal.getIsMachineOn();

    uint32_t spindleSpeedIncrease = static_cast<uint32_t>(mHal.getspindleSpeedChangeIncrease());
    uint32_t spindleSpeedDecrease = static_cast<uint32_t>(mHal.getspindleSpeedChangeDecrease());
    uint32_t spindleFeedRate = static_cast<uint32_t>(mHal.getSpindleOverrideValue() * 100);
    uint32_t spindleSpeedCmd = static_cast<uint32_t>(mHal.getspindleSpeedCmd());
    uint32_t feedRate     = static_cast<uint32_t>(mHal.getFeedOverrideValue() * 100);

    assert(spindleFeedRate <= std::numeric_limits<uint16_t>::max());
    assert(spindleSpeedCmd <= std::numeric_limits<uint16_t>::max());
    assert(feedRate <= std::numeric_limits<uint16_t>::max());

    if (spindleSpeedIncrease || spindleSpeedDecrease)
    {
    	  mDisplayData.spindleFeedRate = spindleSpeedCmd;
    }
    else
    {
        mDisplayData.spindleFeedRate = spindleFeedRate;   
    }
    	
    mDisplayData.feedRate     = feedRate;

    bool isAbsolutePositionRequest = (mAxisPositionMethod == AxisPositionMethod::ABSOLUTE);
    mDisplayData.displayModeFlags.asBitFields.isRelativeCoordinate = !isAbsolutePositionRequest;
    if (mActiveAxisGroup == AxisGroup::XYZ)
    {
        mDisplayData.row1Coordinate.setCoordinate(static_cast<float>(mHal.getAxisXPosition(isAbsolutePositionRequest)));
        mDisplayData.row2Coordinate.setCoordinate(static_cast<float>(mHal.getAxisYPosition(isAbsolutePositionRequest)));
        mDisplayData.row3Coordinate.setCoordinate(static_cast<float>(mHal.getAxisZPosition(isAbsolutePositionRequest)));
    }
    else
    {
        mDisplayData.row1Coordinate.setCoordinate(static_cast<float>(mHal.getAxisAPosition(isAbsolutePositionRequest)));
        mDisplayData.row2Coordinate.setCoordinate(static_cast<float>(mHal.getAxisBPosition(isAbsolutePositionRequest)));
        mDisplayData.row3Coordinate.setCoordinate(static_cast<float>(mHal.getAxisCPosition(isAbsolutePositionRequest)));
    }
}

void Display::clearData()
{
    mDisplayData.feedRate     = 0;
    mDisplayData.spindleFeedRate = 0;
    mDisplayData.displayModeFlags.asBitFields.stepMode =
        static_cast<typename std::underlying_type<DisplayIndicatorStepMode::StepMode>::type>(DisplayIndicatorStepMode::StepMode::MPG);
    mDisplayData.displayModeFlags.asBitFields.isReset              = true;
    mDisplayData.displayModeFlags.asBitFields.isRelativeCoordinate = false;
    mDisplayData.row1Coordinate.clear();
    mDisplayData.row2Coordinate.clear();
    mDisplayData.row3Coordinate.clear();
}
}

