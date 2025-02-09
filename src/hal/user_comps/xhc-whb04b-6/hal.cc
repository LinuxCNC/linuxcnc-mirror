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

#include "./hal.h"

// system includes
#include <iostream>
#include <iomanip>
#include <assert.h>
#include <string.h>
#include <stdarg.h>
#include <unistd.h>
#include <fstream>

// local library includes
#include "./pendant.h"

using std::endl;

namespace XhcWhb04b6 {
// ----------------------------------------------------------------------
HalMemory::HalMemory() :
    in(),
    out()
{
}
// ----------------------------------------------------------------------
HalMemory::~HalMemory()
{
}
// ----------------------------------------------------------------------
HalMemory::In::In()
{
}
// ----------------------------------------------------------------------
HalMemory::Out::Out()
{
}
// ----------------------------------------------------------------------
void Hal::freeSimulatedPin(void** pin)
{
    if (*pin != nullptr)
    {
        free(*pin);
        *pin = nullptr;
    }
}
// ----------------------------------------------------------------------
Hal::Hal(Profiles::HalRequestProfile halRequestProfile) :
    mButtonNameToIdx(),
    mHalCout(&mDevNull),
    mStepMode(HandwheelStepmodes::Mode::MPG),
    mHalRequestProfile(halRequestProfile)
{
    (void)mStepMode;
}
// ----------------------------------------------------------------------
Hal::~Hal()
{
    if (!mIsSimulationMode)
    {
        memory->~HalMemory();
        // documentation tells us to not free hal pins
        return;
    }

    if (memory == nullptr)
    {
        return;
    }

    freeSimulatedPin((void**)(&memory->in.floodIsOn));
    freeSimulatedPin((void**)(&memory->in.mistIsOn));

    freeSimulatedPin((void**)(&memory->in.axisXPosition));
    freeSimulatedPin((void**)(&memory->in.axisYPosition));
    freeSimulatedPin((void**)(&memory->in.axisZPosition));
    freeSimulatedPin((void**)(&memory->in.axisAPosition));
    freeSimulatedPin((void**)(&memory->in.axisBPosition));
    freeSimulatedPin((void**)(&memory->in.axisCPosition));

    freeSimulatedPin((void**)(&memory->in.axisXPositionRelative));
    freeSimulatedPin((void**)(&memory->in.axisYPositionRelative));
    freeSimulatedPin((void**)(&memory->in.axisZPositionRelative));
    freeSimulatedPin((void**)(&memory->in.axisAPositionRelative));
    freeSimulatedPin((void**)(&memory->in.axisBPositionRelative));
    freeSimulatedPin((void**)(&memory->in.axisCPositionRelative));

    freeSimulatedPin((void**)(&memory->in.spindleIsOn));
    freeSimulatedPin((void**)(&memory->in.spindleOverrideValue));
    freeSimulatedPin((void**)(&memory->in.spindleSpeedCmd));

    freeSimulatedPin((void**)(&memory->in.feedOverrideMaxVel));
    freeSimulatedPin((void**)(&memory->in.feedOverrideValue));

    freeSimulatedPin((void**)(&memory->in.isProgramRunning));
    freeSimulatedPin((void**)(&memory->in.isProgramPaused));
    freeSimulatedPin((void**)(&memory->in.isProgramIdle));

    freeSimulatedPin((void**)(&memory->in.isModeAuto));
    freeSimulatedPin((void**)(&memory->in.isModeJoint));
    freeSimulatedPin((void**)(&memory->in.isModeManual));
    freeSimulatedPin((void**)(&memory->in.isModeMdi));
    freeSimulatedPin((void**)(&memory->in.isModeTeleop));

    // If axis is not homed we need to ask Teleop mode but we need to bypass that if machine is homed
    // https://forum.linuxcnc.org/49-basic-configuration/40581-how-to-configure-a-xhc-whb04b-pendant
    freeSimulatedPin((void**)(&memory->in.JointXisHomed));
    freeSimulatedPin((void**)(&memory->in.JointYisHomed));
    freeSimulatedPin((void**)(&memory->in.JointZisHomed));
    freeSimulatedPin((void**)(&memory->in.JointAisHomed));
    freeSimulatedPin((void**)(&memory->in.JointBisHomed));
    freeSimulatedPin((void**)(&memory->in.JointCisHomed));

    freeSimulatedPin((void**)(&memory->in.isMachineOn));

    constexpr size_t pinsCount = sizeof(memory->out.button_pin) / sizeof(hal_bit_t * );
    for (size_t      idx       = 0; idx < pinsCount; idx++)
    {
        freeSimulatedPin((void**)(&memory->out.button_pin[idx]));
    }

    freeSimulatedPin((void**)(&memory->out.floodStop));
    freeSimulatedPin((void**)(&memory->out.floodStart));
    freeSimulatedPin((void**)(&memory->out.mistStop));
    freeSimulatedPin((void**)(&memory->out.mistStart));

    freeSimulatedPin((void**)(&memory->out.axisXJogCounts));
    freeSimulatedPin((void**)(&memory->out.axisYJogCounts));
    freeSimulatedPin((void**)(&memory->out.axisZJogCounts));
    freeSimulatedPin((void**)(&memory->out.axisAJogCounts));
    freeSimulatedPin((void**)(&memory->out.axisBJogCounts));
    freeSimulatedPin((void**)(&memory->out.axisCJogCounts));

    freeSimulatedPin((void**)(&memory->out.axisXJogEnable));
    freeSimulatedPin((void**)(&memory->out.axisYJogEnable));
    freeSimulatedPin((void**)(&memory->out.axisZJogEnable));
    freeSimulatedPin((void**)(&memory->out.axisAJogEnable));
    freeSimulatedPin((void**)(&memory->out.axisBJogEnable));
    freeSimulatedPin((void**)(&memory->out.axisCJogEnable));

    freeSimulatedPin((void**)(&memory->out.axisXJogScale));
    freeSimulatedPin((void**)(&memory->out.axisYJogScale));
    freeSimulatedPin((void**)(&memory->out.axisZJogScale));
    freeSimulatedPin((void**)(&memory->out.axisAJogScale));
    freeSimulatedPin((void**)(&memory->out.axisBJogScale));
    freeSimulatedPin((void**)(&memory->out.axisCJogScale));

    freeSimulatedPin((void**)(&memory->out.axisXSetVelocityMode));
    freeSimulatedPin((void**)(&memory->out.axisYSetVelocityMode));
    freeSimulatedPin((void**)(&memory->out.axisZSetVelocityMode));
    freeSimulatedPin((void**)(&memory->out.axisASetVelocityMode));
    freeSimulatedPin((void**)(&memory->out.axisBSetVelocityMode));
    freeSimulatedPin((void**)(&memory->out.axisCSetVelocityMode));

    freeSimulatedPin((void**)(&memory->out.feedValueSelected_2));
    freeSimulatedPin((void**)(&memory->out.feedValueSelected_5));
    freeSimulatedPin((void**)(&memory->out.feedValueSelected_10));
    freeSimulatedPin((void**)(&memory->out.feedValueSelected_30));
    freeSimulatedPin((void**)(&memory->out.feedValueSelected_60));
    freeSimulatedPin((void**)(&memory->out.feedValueSelected_100));
    freeSimulatedPin((void**)(&memory->out.feedValueSelected_lead));
    freeSimulatedPin((void**)(&memory->out.feedValueSelected_mpg_feed));
    freeSimulatedPin((void**)(&memory->out.feedValueSelected_continuous));
    freeSimulatedPin((void**)(&memory->out.feedValueSelected_step));

    freeSimulatedPin((void**)(&memory->out.feedOverrideScale));
    freeSimulatedPin((void**)(&memory->out.feedOverrideDecrease));
    freeSimulatedPin((void**)(&memory->out.feedOverrideIncrease));

    freeSimulatedPin((void**)(&memory->out.spindleStart));
    freeSimulatedPin((void**)(&memory->out.spindleStop));
    freeSimulatedPin((void**)(&memory->out.spindleDoRunForward));
    freeSimulatedPin((void**)(&memory->out.spindleDoRunReverse));
    freeSimulatedPin((void**)(&memory->out.spindleDoDecrease));
    freeSimulatedPin((void**)(&memory->out.spindleDoIncrease));
    freeSimulatedPin((void**)(&memory->out.spindleOverrideDoDecrease));
    freeSimulatedPin((void**)(&memory->out.spindleOverrideDoIncrease));
    freeSimulatedPin((void**)(&memory->out.spindleOverrideScale));

    freeSimulatedPin((void**)(&memory->out.axisXSelect));
    freeSimulatedPin((void**)(&memory->out.axisYSelect));
    freeSimulatedPin((void**)(&memory->out.axisZSelect));
    freeSimulatedPin((void**)(&memory->out.axisASelect));
    freeSimulatedPin((void**)(&memory->out.axisBSelect));
    freeSimulatedPin((void**)(&memory->out.axisCSelect));

    freeSimulatedPin((void**)(&memory->out.isPendantSleeping));
    freeSimulatedPin((void**)(&memory->out.isPendantConnected));

    freeSimulatedPin((void**)(&memory->out.doRunProgram));
    freeSimulatedPin((void**)(&memory->out.doPauseProgram));
    freeSimulatedPin((void**)(&memory->out.doResumeProgram));
    freeSimulatedPin((void**)(&memory->out.doStopProgram));

    freeSimulatedPin((void**)(&memory->out.doModeAuto));
    freeSimulatedPin((void**)(&memory->out.doModeJoint));
    freeSimulatedPin((void**)(&memory->out.doModeManual));
    freeSimulatedPin((void**)(&memory->out.doModeMdi));
    freeSimulatedPin((void**)(&memory->out.doModeTeleop));

    freeSimulatedPin((void**)(&memory->out.doMachineOn));
    freeSimulatedPin((void**)(&memory->out.doMachineOff));

    delete memory;
}
// ----------------------------------------------------------------------
int Hal::newSimulatedHalPin(char* /*pin_name*/, void** ptr, int s)
{
    *ptr = calloc(s, 1);
    assert(*ptr != nullptr);
    memset(*ptr, 0, s);
    return 0;
}
// ----------------------------------------------------------------------
int Hal::newHalFloat(hal_pin_dir_t direction, hal_float_t** ptr, int componentId, const char* fmt, ...)
{
    char    pin_name[256];
    va_list args;
    va_start(args, fmt);
    vsprintf(pin_name, fmt, args);
    va_end(args);

    assert(strlen(pin_name) < HAL_NAME_LEN);

    *mHalCout << "hal   float ";
    if (direction == HAL_OUT)
    {
        *mHalCout << "out ";
    }
    else
    {
        *mHalCout << "in  ";
    }
    *mHalCout << pin_name << endl;

    assert(ptr != nullptr);
    assert(*ptr == nullptr);

    if (mIsSimulationMode)
    {
        return newSimulatedHalPin(pin_name, (void**)ptr, sizeof(hal_float_t));
    }
    else
    {
        int r = hal_pin_float_new(pin_name, direction, ptr, componentId);
        assert(r == 0);
        return r;
    }
}
// ----------------------------------------------------------------------
int Hal::newHalSigned32(hal_pin_dir_t direction, hal_s32_t** ptr, int componentId, const char* fmt, ...)
{
    char    pin_name[256];
    va_list args;
    va_start(args, fmt);
    vsprintf(pin_name, fmt, args);
    va_end(args);

    assert(strlen(pin_name) < HAL_NAME_LEN);

    *mHalCout << "hal   s32   ";
    if (direction == HAL_OUT)
    {
        *mHalCout << "out ";
    }
    else
    {
        *mHalCout << "in  ";
    }
    *mHalCout << pin_name << endl;

    assert(ptr != nullptr);
    assert(*ptr == nullptr);

    if (mIsSimulationMode)
    {
        return newSimulatedHalPin(pin_name, (void**)ptr, sizeof(hal_s32_t));
    }
    else
    {
        int r = hal_pin_s32_new(pin_name, direction, ptr, componentId);
        assert(r == 0);
        return r;
    }
}
// ----------------------------------------------------------------------
int Hal::newHalUnsigned32(hal_pin_dir_t direction, hal_u32_t** ptr, int componentId, const char* fmt, ...)
{
    char    pin_name[256];
    va_list args;
    va_start(args, fmt);
    vsprintf(pin_name, fmt, args);
    va_end(args);

    assert(strlen(pin_name) < HAL_NAME_LEN);

    *mHalCout << "hal   u32   ";
    if (direction == HAL_OUT)
    {
        *mHalCout << "out ";
    }
    else
    {
        *mHalCout << "in  ";
    }
    *mHalCout << pin_name << endl;

    assert(ptr != nullptr);
    assert(*ptr == nullptr);

    if (mIsSimulationMode)
    {
        return newSimulatedHalPin(pin_name, (void**)ptr, sizeof(hal_u32_t));
    }
    else
    {
        int r = hal_pin_u32_new(pin_name, direction, ptr, componentId);
        assert(r == 0);
        return r;
    }
}
// ----------------------------------------------------------------------
int Hal::newHalBit(hal_pin_dir_t direction, hal_bit_t** ptr, int componentId, const char* fmt, ...)
{
    char    pin_name[256];
    va_list args;
    va_start(args, fmt);
    vsprintf(pin_name, fmt, args);
    va_end(args);

    assert(strlen(pin_name) < HAL_NAME_LEN);

    *mHalCout << "hal   bit   ";
    if (direction == HAL_OUT)
    {
        *mHalCout << "out ";
    }
    else
    {
        *mHalCout << "in  ";
    }
    *mHalCout << pin_name << endl;

    assert(ptr != nullptr);
    assert(*ptr == nullptr);

    if (mIsSimulationMode)
    {
        return newSimulatedHalPin(pin_name, (void**)ptr, sizeof(hal_bit_t));
    }
    else
    {
        int r = hal_pin_bit_new(pin_name, direction, ptr, componentId);
        assert(r == 0);
        return r;
    }
}
// ----------------------------------------------------------------------
bool Hal::isSimulationModeEnabled() const
{
    return mIsSimulationMode;
}
// ----------------------------------------------------------------------
void Hal::setSimulationMode(bool isSimulationMode)
{
    this->mIsSimulationMode = isSimulationMode;
}
// ----------------------------------------------------------------------
int Hal::getHalComponentId() const
{
    return mHalCompId;
}
// ----------------------------------------------------------------------
const char* Hal::getHalComponentName() const
{
    return mName;
}
// ----------------------------------------------------------------------
void Hal::init(const MetaButtonCodes* metaButtons, const KeyCodes& keyCodes)
{
    assert(!mIsInitialized);

    if (!mIsSimulationMode)
    {
        *mHalCout << "hal   initialize HAL component in HAL mode " << mName << " ... ";
        mHalCompId = hal_init(mName);
        if (mHalCompId <= 0)
        {
            std::cerr << endl << "failed to initialize HAL component " << mName << endl;
            exit(EXIT_FAILURE);
        }
        *mHalCout << "ok" << endl;

        *mHalCout << "hal   initialize shared HAL memory for component id  " << mHalCompId << " ... ";
        memory = reinterpret_cast<HalMemory*>(hal_malloc(sizeof(HalMemory)));
        memory = new(memory) HalMemory();
    }
    else
    {
        *mHalCout << "hal   initialize simulated HAL memory " << " ... ";
        memory = new HalMemory();
    }

    if (memory == nullptr)
    {
        std::cerr << "failed to allocate HAL memory" << endl;
        exit(EXIT_FAILURE);
    }
    *mHalCout << "ok" << endl;

    // register all known xhc-whb04b-6 buttons
    for (size_t idx = 0; !((metaButtons[idx].key.code == keyCodes.Buttons.undefined.code) &&
                           (metaButtons[idx].modifier.code == keyCodes.Buttons.undefined.code)); idx++)
    {
        const char* buttonName = nullptr;
        if (&metaButtons[idx].modifier == &keyCodes.Buttons.undefined)
        {
            buttonName = metaButtons[idx].key.text;
        }
        else
        {
            buttonName = metaButtons[idx].key.altText;
        }

        mButtonNameToIdx[std::string(mComponentPrefix) + ".button." + std::string(buttonName)] = idx;
        newHalBit(HAL_OUT, &((memory->out.button_pin)[idx]), mHalCompId, "%s.button.%s", mComponentPrefix,
                  buttonName);
    }

    newHalBit(HAL_IN, &(memory->in.floodIsOn), mHalCompId, "%s.halui.flood.is-on", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.floodStop), mHalCompId, "%s.halui.flood.off", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.floodStart), mHalCompId, "%s.halui.flood.on", mComponentPrefix);

    newHalBit(HAL_IN, &(memory->in.mistIsOn), mHalCompId, "%s.halui.mist.is-on", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.mistStop), mHalCompId, "%s.halui.mist.off", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.mistStart), mHalCompId, "%s.halui.mist.on", mComponentPrefix);

    newHalSigned32(HAL_OUT, &(memory->out.axisXJogCounts), mHalCompId, "%s.axis.x.jog-counts", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.axisXJogEnable), mHalCompId, "%s.axis.x.jog-enable", mComponentPrefix);
    newHalFloat(HAL_OUT, &(memory->out.axisXJogScale), mHalCompId, "%s.axis.x.jog-scale", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.axisXSetVelocityMode), mHalCompId, "%s.axis.x.jog-vel-mode", mComponentPrefix);

    newHalSigned32(HAL_OUT, &(memory->out.axisYJogCounts), mHalCompId, "%s.axis.y.jog-counts", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.axisYJogEnable), mHalCompId, "%s.axis.y.jog-enable", mComponentPrefix);
    newHalFloat(HAL_OUT, &(memory->out.axisYJogScale), mHalCompId, "%s.axis.y.jog-scale", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.axisYSetVelocityMode), mHalCompId, "%s.axis.y.jog-vel-mode", mComponentPrefix);

    newHalSigned32(HAL_OUT, &(memory->out.axisZJogCounts), mHalCompId, "%s.axis.z.jog-counts", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.axisZJogEnable), mHalCompId, "%s.axis.z.jog-enable", mComponentPrefix);
    newHalFloat(HAL_OUT, &(memory->out.axisZJogScale), mHalCompId, "%s.axis.z.jog-scale", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.axisZSetVelocityMode), mHalCompId, "%s.axis.z.jog-vel-mode", mComponentPrefix);

    newHalSigned32(HAL_OUT, &(memory->out.axisAJogCounts), mHalCompId, "%s.axis.a.jog-counts", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.axisAJogEnable), mHalCompId, "%s.axis.a.jog-enable", mComponentPrefix);
    newHalFloat(HAL_OUT, &(memory->out.axisAJogScale), mHalCompId, "%s.axis.a.jog-scale", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.axisASetVelocityMode), mHalCompId, "%s.axis.a.jog-vel-mode", mComponentPrefix);

    newHalSigned32(HAL_OUT, &(memory->out.axisBJogCounts), mHalCompId, "%s.axis.b.jog-counts", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.axisBJogEnable), mHalCompId, "%s.axis.b.jog-enable", mComponentPrefix);
    newHalFloat(HAL_OUT, &(memory->out.axisBJogScale), mHalCompId, "%s.axis.b.jog-scale", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.axisBSetVelocityMode), mHalCompId, "%s.axis.b.jog-vel-mode", mComponentPrefix);

    newHalSigned32(HAL_OUT, &(memory->out.axisCJogCounts), mHalCompId, "%s.axis.c.jog-counts", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.axisCJogEnable), mHalCompId, "%s.axis.c.jog-enable", mComponentPrefix);
    newHalFloat(HAL_OUT, &(memory->out.axisCJogScale), mHalCompId, "%s.axis.c.jog-scale", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.axisCSetVelocityMode), mHalCompId, "%s.axis.c.jog-vel-mode", mComponentPrefix);

    newHalBit(HAL_OUT, &(memory->out.isPendantSleeping), mHalCompId, "%s.pendant.is-sleeping", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.isPendantConnected), mHalCompId, "%s.pendant.is-connected", mComponentPrefix);

    newHalFloat(HAL_IN, &(memory->in.axisXPosition), mHalCompId, "%s.halui.axis.x.pos-feedback", mComponentPrefix);
    newHalFloat(HAL_IN, &(memory->in.axisYPosition), mHalCompId, "%s.halui.axis.y.pos-feedback", mComponentPrefix);
    newHalFloat(HAL_IN, &(memory->in.axisZPosition), mHalCompId, "%s.halui.axis.z.pos-feedback", mComponentPrefix);
    newHalFloat(HAL_IN, &(memory->in.axisAPosition), mHalCompId, "%s.halui.axis.a.pos-feedback", mComponentPrefix);
    newHalFloat(HAL_IN, &(memory->in.axisBPosition), mHalCompId, "%s.halui.axis.b.pos-feedback", mComponentPrefix);
    newHalFloat(HAL_IN, &(memory->in.axisCPosition), mHalCompId, "%s.halui.axis.c.pos-feedback", mComponentPrefix);

    newHalFloat(HAL_IN, &(memory->in.axisXPositionRelative), mHalCompId, "%s.halui.axis.x.pos-relative", mComponentPrefix);
    newHalFloat(HAL_IN, &(memory->in.axisYPositionRelative), mHalCompId, "%s.halui.axis.y.pos-relative", mComponentPrefix);
    newHalFloat(HAL_IN, &(memory->in.axisZPositionRelative), mHalCompId, "%s.halui.axis.z.pos-relative", mComponentPrefix);
    newHalFloat(HAL_IN, &(memory->in.axisAPositionRelative), mHalCompId, "%s.halui.axis.a.pos-relative", mComponentPrefix);
    newHalFloat(HAL_IN, &(memory->in.axisBPositionRelative), mHalCompId, "%s.halui.axis.b.pos-relative", mComponentPrefix);
    newHalFloat(HAL_IN, &(memory->in.axisCPositionRelative), mHalCompId, "%s.halui.axis.c.pos-relative", mComponentPrefix);

    newHalBit(HAL_OUT, &(memory->out.feedValueSelected_2), mHalCompId, "%s.halui.feed.selected-2", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.feedValueSelected_5), mHalCompId, "%s.halui.feed.selected-5", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.feedValueSelected_10), mHalCompId, "%s.halui.feed.selected-10", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.feedValueSelected_30), mHalCompId, "%s.halui.feed.selected-30", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.feedValueSelected_60), mHalCompId, "%s.halui.feed.selected-60", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.feedValueSelected_100), mHalCompId, "%s.halui.feed.selected-100", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.feedValueSelected_lead), mHalCompId, "%s.halui.feed.selected-lead", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.feedValueSelected_mpg_feed), mHalCompId, "%s.halui.feed.selected-mpg-feed", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.feedValueSelected_continuous), mHalCompId, "%s.halui.feed.selected-continuous", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.feedValueSelected_step), mHalCompId, "%s.halui.feed.selected-step", mComponentPrefix);

    newHalFloat(HAL_OUT, &(memory->out.feedOverrideScale), mHalCompId, "%s.halui.feed-override.scale", mComponentPrefix);
    newHalFloat(HAL_IN, &(memory->in.feedOverrideMaxVel), mHalCompId, "%s.halui.max-velocity.value", mComponentPrefix);
    newHalFloat(HAL_IN, &(memory->in.feedOverrideValue), mHalCompId, "%s.halui.feed-override.value", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.feedOverrideDecrease), mHalCompId, "%s.halui.feed-override.decrease", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.feedOverrideIncrease), mHalCompId, "%s.halui.feed-override.increase", mComponentPrefix);

    newHalFloat(HAL_IN, &(memory->in.spindleSpeedCmd), mHalCompId, "%s.halui.spindle-speed-cmd", mComponentPrefix);
    newHalFloat(HAL_IN, &(memory->in.spindleOverrideValue), mHalCompId, "%s.halui.spindle-override.value",mComponentPrefix);
    newHalFloat(HAL_OUT, &(memory->out.spindleOverrideScale), mHalCompId, "%s.halui.spindle-override.scale", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.spindleDoIncrease), mHalCompId, "%s.halui.spindle.increase", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.spindleDoDecrease), mHalCompId, "%s.halui.spindle.decrease", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.spindleOverrideDoIncrease), mHalCompId, "%s.halui.spindle-override.increase", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.spindleOverrideDoDecrease), mHalCompId, "%s.halui.spindle-override.decrease", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.spindleStart), mHalCompId, "%s.halui.spindle.start", mComponentPrefix);
    newHalBit(HAL_IN, &(memory->in.spindleIsOn), mHalCompId, "%s.halui.spindle.is-on", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.spindleStop), mHalCompId, "%s.halui.spindle.stop", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.spindleDoRunForward), mHalCompId, "%s.halui.spindle.forward", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.spindleDoRunReverse), mHalCompId, "%s.halui.spindle.reverse", mComponentPrefix);

    newHalBit(HAL_IN, &(memory->in.isMachineOn), mHalCompId, "%s.halui.machine.is-on", mComponentPrefix);

    newHalBit(HAL_OUT, &(memory->out.doMachineOn), mHalCompId, "%s.halui.machine.on", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.doMachineOff), mHalCompId, "%s.halui.machine.off", mComponentPrefix);

    newHalBit(HAL_IN, &(memory->in.isProgramIdle), mHalCompId, "%s.halui.program.is-idle", mComponentPrefix);
    newHalBit(HAL_IN, &(memory->in.isProgramPaused), mHalCompId, "%s.halui.program.is-paused", mComponentPrefix);
    newHalBit(HAL_IN, &(memory->in.isProgramRunning), mHalCompId, "%s.halui.program.is-running", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.doResumeProgram), mHalCompId, "%s.halui.program.resume", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.doPauseProgram), mHalCompId, "%s.halui.program.pause", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.doRunProgram), mHalCompId, "%s.halui.program.run", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.doStopProgram), mHalCompId, "%s.halui.program.stop", mComponentPrefix);

    newHalBit(HAL_IN, &(memory->in.isModeAuto), mHalCompId, "%s.halui.mode.is-auto", mComponentPrefix);
    newHalBit(HAL_IN, &(memory->in.isModeJoint), mHalCompId, "%s.halui.mode.is-joint", mComponentPrefix);
    newHalBit(HAL_IN, &(memory->in.isModeManual), mHalCompId, "%s.halui.mode.is-manual", mComponentPrefix);
    newHalBit(HAL_IN, &(memory->in.isModeMdi), mHalCompId, "%s.halui.mode.is-mdi", mComponentPrefix);
    newHalBit(HAL_IN, &(memory->in.isModeTeleop), mHalCompId, "%s.halui.mode.is-teleop", mComponentPrefix);


    // If axis is not homed we need to ask Teleop mode but we need to bypass that if machine is homed
    // https://forum.linuxcnc.org/49-basic-configuration/40581-how-to-configure-a-xhc-whb04b-pendant
    newHalBit(HAL_IN, &(memory->in.JointXisHomed), mHalCompId, "%s.halui.joint.x.is-homed", mComponentPrefix);
    newHalBit(HAL_IN, &(memory->in.JointYisHomed), mHalCompId, "%s.halui.joint.y.is-homed", mComponentPrefix);
    newHalBit(HAL_IN, &(memory->in.JointZisHomed), mHalCompId, "%s.halui.joint.z.is-homed", mComponentPrefix);
    newHalBit(HAL_IN, &(memory->in.JointAisHomed), mHalCompId, "%s.halui.joint.a.is-homed", mComponentPrefix);
    newHalBit(HAL_IN, &(memory->in.JointBisHomed), mHalCompId, "%s.halui.joint.b.is-homed", mComponentPrefix);
    newHalBit(HAL_IN, &(memory->in.JointCisHomed), mHalCompId, "%s.halui.joint.c.is-homed", mComponentPrefix);


    newHalBit(HAL_OUT, &(memory->out.doModeAuto), mHalCompId, "%s.halui.mode.auto", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.doModeJoint), mHalCompId, "%s.halui.mode.joint", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.doModeManual), mHalCompId, "%s.halui.mode.manual", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.doModeMdi), mHalCompId, "%s.halui.mode.mdi", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.doModeTeleop), mHalCompId, "%s.halui.mode.teleop", mComponentPrefix);

    newHalBit(HAL_OUT, &(memory->out.axisXSelect), mHalCompId, "%s.halui.axis.x.select", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.axisYSelect), mHalCompId, "%s.halui.axis.y.select", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.axisZSelect), mHalCompId, "%s.halui.axis.z.select", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.axisASelect), mHalCompId, "%s.halui.axis.a.select", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.axisBSelect), mHalCompId, "%s.halui.axis.b.select", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.axisCSelect), mHalCompId, "%s.halui.axis.c.select", mComponentPrefix);

    mIsInitialized = true;
}
// ----------------------------------------------------------------------
real_t Hal::getAxisXPosition(bool absolute) const
{
    if (absolute)
    {
        return *memory->in.axisXPosition;
    }
    return *memory->in.axisXPositionRelative;
}
// ----------------------------------------------------------------------
real_t Hal::getAxisYPosition(bool absolute) const
{
    if (absolute)
    {
        return *memory->in.axisYPosition;
    }
    return *memory->in.axisYPositionRelative;
}
// ----------------------------------------------------------------------
real_t Hal::getAxisZPosition(bool absolute) const
{
    if (absolute)
    {
        return *memory->in.axisZPosition;
    }
    return *memory->in.axisZPositionRelative;
}
// ----------------------------------------------------------------------
real_t Hal::getAxisAPosition(bool absolute) const
{
    if (absolute)
    {
        return *memory->in.axisAPosition;
    }
    return *memory->in.axisAPositionRelative;
}
// ----------------------------------------------------------------------
real_t Hal::getAxisBPosition(bool absolute) const
{
    if (absolute)
    {
        return *memory->in.axisBPosition;
    }
    return *memory->in.axisBPositionRelative;
}
// ----------------------------------------------------------------------
real_t Hal::getAxisCPosition(bool absolute) const
{
    if (absolute)
    {
        return *memory->in.axisCPosition;
    }
    return *memory->in.axisCPositionRelative;
}
// ----------------------------------------------------------------------
void Hal::enableVerbose(bool enable)
{
    if (enable)
    {
        mHalCout = &std::cout;
    }
    else
    {
        mHalCout = &mDevNull;
    }
}
// ----------------------------------------------------------------------
void Hal::setNoAxisActive(bool /*enabled*/)
{
    *mHalCout << "hal   OFF no axis active" << endl;
}
// ----------------------------------------------------------------------
void Hal::setAxisXActive(bool enabled)
{
    *memory->out.axisXSelect   = enabled;
    *memory->out.axisXJogEnable = enabled;
    *mHalCout << "hal   X axis active" << endl;
}
// ----------------------------------------------------------------------
void Hal::setAxisYActive(bool enabled)
{
    *memory->out.axisYSelect   = enabled;
    *memory->out.axisYJogEnable = enabled;
    *mHalCout << "hal   Y axis active" << endl;
}
// ----------------------------------------------------------------------
void Hal::setAxisZActive(bool enabled)
{
    *memory->out.axisZSelect   = enabled;
    *memory->out.axisZJogEnable = enabled;
    *mHalCout << "hal   Z axis active" << endl;
}
// ----------------------------------------------------------------------
void Hal::setAxisAActive(bool enabled)
{
    *memory->out.axisASelect   = enabled;
    *memory->out.axisAJogEnable = enabled;
    *mHalCout << "hal   A axis active" << endl;
}
// ----------------------------------------------------------------------
void Hal::setAxisBActive(bool enabled)
{
    *memory->out.axisBSelect   = enabled;
    *memory->out.axisBJogEnable = enabled;
    *mHalCout << "hal   B axis active" << endl;
}
// ----------------------------------------------------------------------
void Hal::setAxisCActive(bool enabled)
{
    *memory->out.axisCSelect   = enabled;
    *memory->out.axisCJogEnable = enabled;
    *mHalCout << "hal   C axis active" << endl;
}
// ----------------------------------------------------------------------
void Hal::setStepSize(const real_t stepSize)
{
    *memory->out.axisXJogScale = stepSize;
    *memory->out.axisYJogScale = stepSize;
    *memory->out.axisZJogScale = stepSize;
    *memory->out.axisAJogScale = stepSize;
    *memory->out.axisBJogScale = stepSize;
    *memory->out.axisCJogScale = stepSize;
    *mHalCout << "hal   step size " << stepSize << endl;
}
// ----------------------------------------------------------------------
void Hal::setLead()
{
    std::ios init(NULL);
    init.copyfmt(*mHalCout);
    *mHalCout << "hal   feed rate Lead" << endl;
    mHalCout->copyfmt(init);
}
// ----------------------------------------------------------------------
void Hal::setReset(bool enabled)
{
    if (*memory->in.isMachineOn)
    { // disable machine
        clearStartResumeProgramStates();
        *memory->out.doMachineOff = true;
    }
    else
    { // enable machine
        *memory->out.doMachineOn = true;
    }

    if (!enabled)
    {
        *memory->out.doMachineOff = false;
        *memory->out.doMachineOn  = false;
    }
    setPin(enabled, KeyCodes::Buttons.reset.text);
}
// ----------------------------------------------------------------------
void Hal::setStop(bool enabled)
{
    clearStartResumeProgramStates();
    *memory->out.doStopProgram = enabled;
    setPin(enabled, KeyCodes::Buttons.stop.text);
}
// ----------------------------------------------------------------------
void Hal::setStart(bool enabled)
{
    if (requestAutoMode(enabled))
    {
        if (enabled)
    {
        toggleStartResumeProgram();
    }
    setPin(enabled, KeyCodes::Buttons.start.text);
    }

    if (!enabled)
    {
        setPin(enabled, KeyCodes::Buttons.start.text);
    }
}
// ----------------------------------------------------------------------
bool Hal::getIsMachineOn() const
{
    return *memory->in.isMachineOn;
}
// ----------------------------------------------------------------------
void Hal::setIsPendantSleeping(bool isSleeping)
{
    *memory->out.isPendantSleeping = isSleeping;
}
// ----------------------------------------------------------------------
bool Hal::getIsPendantSleeping() const
{
    return *memory->out.isPendantSleeping;
}
// ----------------------------------------------------------------------
void Hal::setIsPendantConnected(bool isSleeping)
{
    *memory->out.isPendantConnected = isSleeping;
}
// ----------------------------------------------------------------------
bool Hal::getIsPendantConnected() const
{
    return *memory->out.isPendantConnected;
}
// ----------------------------------------------------------------------
void Hal::clearStartResumeProgramStates()
{
    *memory->out.doModeTeleop      = false;
    *memory->out.doModeJoint       = false;
    *memory->out.doModeAuto        = false;
    *memory->out.doPauseProgram    = false;
    *memory->out.doRunProgram      = false;
    *memory->out.doResumeProgram   = false;
}

void Hal::checkState(bool state, hal_bit_t *pin)
{
    // 500 milliseconds timeout
    unsigned int timeouts=500;
    unsigned int timeoutMs=1;
    do
    {
        if (state == *pin)
        {
            usleep(timeoutMs * 1000);
        }
        else
        {
            break;
        }
    } while ((state == *pin) && (--timeouts) > 0);
}
// ----------------------------------------------------------------------
void Hal::toggleStartResumeProgram()
{
    if (*memory->in.isProgramPaused)
    {
        *memory->out.doPauseProgram  = false;
        *memory->out.doRunProgram    = false;
        *memory->out.doResumeProgram = true;
        checkState(true, memory->in.isProgramPaused);
        *memory->out.doResumeProgram = false;
    } else if (*memory->in.isProgramRunning)
    {
        *memory->out.doPauseProgram  = true;
        checkState(false, memory->in.isProgramPaused);
        *memory->out.doPauseProgram  = false;
        *memory->out.doRunProgram    = false;
        *memory->out.doResumeProgram = false;
    } else if (*memory->in.isProgramIdle)
    {
        *memory->out.doPauseProgram  = false;
        *memory->out.doRunProgram    = true;
        checkState(false, memory->in.isProgramRunning);
        *memory->out.doRunProgram    = false;
        *memory->out.doResumeProgram = false;
    }
}
// ----------------------------------------------------------------------
void Hal::setFeedPlus(bool enabled)
{
    *memory->out.feedOverrideScale = 0.05;
    *memory->out.feedOverrideIncrease = enabled;
    setPin(enabled, KeyCodes::Buttons.feed_plus.text);
}
// ----------------------------------------------------------------------
void Hal::setFeedMinus(bool enabled)
{
    *memory->out.feedOverrideScale = 0.05;
    *memory->out.feedOverrideDecrease = enabled;
    setPin(enabled, KeyCodes::Buttons.feed_minus.text);
}
// ----------------------------------------------------------------------
real_t Hal::getspindleSpeedCmd() const
{
    return *memory->in.spindleSpeedCmd;
}
// ----------------------------------------------------------------------
real_t Hal::getspindleSpeedChangeIncrease() const
{
    return *memory->out.spindleDoIncrease;
}
// ----------------------------------------------------------------------
real_t Hal::getspindleSpeedChangeDecrease() const
{
    return *memory->out.spindleDoDecrease;
}
// ----------------------------------------------------------------------
real_t Hal::getSpindleOverrideValue() const
{
    return *memory->in.spindleOverrideValue;
}
// ----------------------------------------------------------------------
real_t Hal::getFeedOverrideMaxVel() const
{
    return *memory->in.feedOverrideMaxVel;
}
// ----------------------------------------------------------------------
real_t Hal::getFeedOverrideValue() const
{
    return *memory->in.feedOverrideValue;
}
// ----------------------------------------------------------------------
void Hal::setFeedValueSelected2(bool selected)
{
    *memory->out.feedValueSelected_2 = selected;
}
// ----------------------------------------------------------------------
void Hal::setFeedValueSelected5(bool selected)
{
    *memory->out.feedValueSelected_5 = selected;
}
// ----------------------------------------------------------------------
void Hal::setFeedValueSelected10(bool selected)
{
    *memory->out.feedValueSelected_10 = selected;
}
// ----------------------------------------------------------------------
void Hal::setFeedValueSelected30(bool selected)
{
    *memory->out.feedValueSelected_30 = selected;
}
// ----------------------------------------------------------------------
void Hal::setFeedValueSelected60(bool selected)
{
    *memory->out.feedValueSelected_60 = selected;
}
// ----------------------------------------------------------------------
void Hal::setFeedValueSelected100(bool selected)
{
    *memory->out.feedValueSelected_100 = selected;
}
// ----------------------------------------------------------------------
void Hal::setFeedValueSelectedLead(bool selected)
{
    *memory->out.feedValueSelected_lead = selected;
}
// ----------------------------------------------------------------------
void Hal::setFeedOverrideScale(real_t scale)
{
    *memory->out.feedOverrideScale = scale;
}
// ----------------------------------------------------------------------
void Hal::setSpindleOverridePlus(bool enabled)
{
    if (enabled)
    {
        *memory->out.spindleOverrideScale = 0.05;
        *memory->out.spindleOverrideDoIncrease = true;
    }
    else
    {
        *memory->out.spindleOverrideDoIncrease = false;
    }
    setPin(enabled, KeyCodes::Buttons.spindle_plus.text);
}
// ----------------------------------------------------------------------
void Hal::setSpindleOverrideMinus(bool enabled)
{
    if (enabled)
    {
        *memory->out.spindleOverrideScale = 0.05;
        *memory->out.spindleOverrideDoDecrease = true;
    }
    else
    {
        *memory->out.spindleOverrideDoDecrease = false;
    }
    setPin(enabled, KeyCodes::Buttons.spindle_minus.text);
}
// ----------------------------------------------------------------------
/**
 * Requests machine to do homing.
 * The task is performed via halui command.
 */
void Hal::setMachineHomingAll(bool enabled)
{
//    if (requestManualMode(enabled))
    if (requestJointMode(enabled))
    {
        if (enabled)
        {
            setPin(enabled, KeyCodes::Buttons.machine_home.text);
        }
    }
    if (!enabled)
{
    setPin(enabled, KeyCodes::Buttons.machine_home.text);
}
}
// ----------------------------------------------------------------------
/**
 * Sends machine to safe Z position.
 * The task is performed via MDI code.
 */
void Hal::setSafeZ(bool enabled)
{
    if (requestMdiMode(enabled))
    {
        if (enabled)
        {
            setPin(enabled, KeyCodes::Buttons.safe_z.text);
        }
    }
    if (!enabled)
    {
    setPin(enabled, KeyCodes::Buttons.safe_z.text);
}
}
// ----------------------------------------------------------------------
/**
 * Sends machine to workpiece home position.
 * The task is performed via MDI code.
 */
void Hal::setWorkpieceHome(bool enabled)
{
    if (requestMdiMode(enabled))
    {
        if (enabled)
        {
            setPin(enabled, KeyCodes::Buttons.workpiece_home.text);
        }
    }
    if (!enabled)
    {
    setPin(enabled, KeyCodes::Buttons.workpiece_home.text);
}
}
// ----------------------------------------------------------------------
void Hal::toggleSpindleDirection(bool enabled)
{
    if (enabled)
    {
        mIsSpindleDirectionForward = !mIsSpindleDirectionForward;
    }

    // on running spindle update direction immediately
    if (*memory->in.spindleIsOn)
    {
        if (enabled)
        {
            if (mIsSpindleDirectionForward)
            {
                *memory->out.spindleDoRunForward = true;
                *memory->out.spindleDoIncrease = true;
            }
            else
            {
                *memory->out.spindleDoRunReverse = true;
                *memory->out.spindleDoIncrease = true;
            }
        }
        else
        {
            *memory->out.spindleDoRunForward = false;
            *memory->out.spindleDoRunReverse = false;
            *memory->out.spindleDoIncrease   = false;
        }
    }
}
// ----------------------------------------------------------------------
void Hal::toggleSpindleOnOff(bool enabled)
{
    if (enabled)
    {
        if (*memory->in.spindleIsOn)
        {
            // on spindle stop
            *memory->out.spindleStop = true;
        }
        else
        {
            // on spindle start
            if (mIsSpindleDirectionForward)
            {
                *memory->out.spindleDoRunForward = true;
                *memory->out.spindleDoIncrease = true;
                *memory->out.spindleStart = true;
            }
            else
            {
                *memory->out.spindleDoRunReverse = true;
                *memory->out.spindleDoIncrease = true;
                *memory->out.spindleStart = true;
                
            }
        }
    }
    else
    {
        // on button released
        *memory->out.spindleStop         = false;
        *memory->out.spindleDoRunForward = false;
        *memory->out.spindleDoRunReverse = false;
        *memory->out.spindleDoIncrease   = false;
        *memory->out.spindleStart        = false;
    }
    setPin(enabled, KeyCodes::Buttons.spindle_on_off.text);
}
// ----------------------------------------------------------------------
void Hal::toggleFloodOnOff(bool enabled)
{
    if (enabled)
    {
        if (*memory->in.floodIsOn)
        {
            // on flood stop
            *memory->out.floodStop = true;
        }
        else
        {
            // on flood start
            *memory->out.floodStart = true;
        }
    }
    else
    {
        // on button released
        *memory->out.floodStop         = false;
        *memory->out.floodStart        = false;
    }
}
// ----------------------------------------------------------------------
void Hal::toggleMistOnOff(bool enabled)
{
    if (enabled)
    {
        if (*memory->in.mistIsOn)
        {
            // on mist stop
            *memory->out.mistStop = true;
        }
        else
        {
            // on mist start
            *memory->out.mistStart = true;
        }
    }
    else
    {
        // on button released
        *memory->out.mistStop         = false;
        *memory->out.mistStart        = false;
    }
}
// ----------------------------------------------------------------------
/**
 * Puts machine into probing mode for Z axis.
 * The task is performed via MDI code.
 */
void Hal::setProbeZ(bool enabled)
{
    if (requestMdiMode(enabled))
    {
        if (enabled)
        {
            setPin(enabled, KeyCodes::Buttons.probe_z.text);
        }
    }
    if (!enabled)
    {
    setPin(enabled, KeyCodes::Buttons.probe_z.text);
}
}
// ----------------------------------------------------------------------
void Hal::setConMode(bool enabled)
{
    if (enabled)
    {
        *memory->out.axisXSetVelocityMode = true;
        *memory->out.axisYSetVelocityMode = true;
        *memory->out.axisZSetVelocityMode = true;
        *memory->out.axisASetVelocityMode = true;
        *memory->out.axisBSetVelocityMode = true;
        *memory->out.axisCSetVelocityMode = true;
        *mHalCout << "hal   step mode is con" << endl;
        *memory->out.feedValueSelected_mpg_feed = false;
        *memory->out.feedValueSelected_continuous = true;
        *memory->out.feedValueSelected_step = false;
    }
    setPin(enabled, KeyCodes::Buttons.continuous.text);
}
// ----------------------------------------------------------------------
void Hal::setStepMode(bool enabled)
{
    if (enabled)
    {
        *memory->out.axisXSetVelocityMode = false;
        *memory->out.axisYSetVelocityMode = false;
        *memory->out.axisZSetVelocityMode = false;
        *memory->out.axisASetVelocityMode = false;
        *memory->out.axisBSetVelocityMode = false;
        *memory->out.axisCSetVelocityMode = false;
        *mHalCout << "hal   step mode is step" << endl;
        *memory->out.feedValueSelected_mpg_feed = false;
        *memory->out.feedValueSelected_continuous = false;
        *memory->out.feedValueSelected_step = true;
    }
    setPin(enabled, KeyCodes::Buttons.step.text);
}
// ----------------------------------------------------------------------
void Hal::setMpgMode(bool enabled)
{
    if (enabled)
    {
        *memory->out.axisXSetVelocityMode = false;
        *memory->out.axisYSetVelocityMode = false;
        *memory->out.axisZSetVelocityMode = false;
        *memory->out.axisASetVelocityMode = false;
        *memory->out.axisBSetVelocityMode = false;
        *memory->out.axisCSetVelocityMode = false;
        *mHalCout << "hal   step mode is mpg" << endl;
        *memory->out.feedValueSelected_mpg_feed = true;
        *memory->out.feedValueSelected_continuous = false;
        *memory->out.feedValueSelected_step = false;
    }
}
// ----------------------------------------------------------------------
void Hal::setMacro1(bool enabled)
{
    setPin(enabled, KeyCodes::Buttons.feed_plus.altText);
}
// ----------------------------------------------------------------------
void Hal::setMacro2(bool enabled)
{
    setPin(enabled, KeyCodes::Buttons.feed_minus.altText);
}
// ----------------------------------------------------------------------
void Hal::setMacro3(bool enabled)
{
    if (enabled)
    {
        if (*memory->in.spindleIsOn)
        {
            *memory->out.spindleDoIncrease = true;
        }
    }
    else
    {
        *memory->out.spindleDoIncrease = false;
    }
    setPin(enabled, KeyCodes::Buttons.spindle_plus.altText);
}
// ----------------------------------------------------------------------
void Hal::setMacro4(bool enabled)
{
    if (enabled)
    {
        if (*memory->in.spindleIsOn)
        {
            *memory->out.spindleDoDecrease = true;
        }
    }
    else
    {
        *memory->out.spindleDoDecrease = false;
    }
    setPin(enabled, KeyCodes::Buttons.spindle_minus.altText);
}
// ----------------------------------------------------------------------
void Hal::setMacro5(bool enabled)
{
    setPin(enabled, KeyCodes::Buttons.machine_home.altText);
}
// ----------------------------------------------------------------------
void Hal::setMacro6(bool enabled)
{
    setPin(enabled, KeyCodes::Buttons.safe_z.altText);
}
// ----------------------------------------------------------------------
void Hal::setMacro7(bool enabled)
{
    setPin(enabled, KeyCodes::Buttons.workpiece_home.altText);
}
// ----------------------------------------------------------------------
void Hal::setMacro8(bool enabled)
{
    setPin(enabled, KeyCodes::Buttons.spindle_on_off.altText);              // Hardcoded toggle spindle direction
}
// ----------------------------------------------------------------------
void Hal::setMacro9(bool enabled)
{
    setPin(enabled, KeyCodes::Buttons.probe_z.altText);
}
// ----------------------------------------------------------------------
void Hal::setMacro10(bool enabled)
{
    setPin(enabled, KeyCodes::Buttons.macro10.text);                        // Hardcoded Absolute/relative Dro
}
// ----------------------------------------------------------------------
void Hal::setMacro11(bool enabled)
{
    setPin(enabled, KeyCodes::Buttons.reset.altText);
}
// ----------------------------------------------------------------------
void Hal::setMacro12(bool enabled)
{
    setPin(enabled, KeyCodes::Buttons.stop.altText);
}
// ----------------------------------------------------------------------
void Hal::setMacro13(bool enabled)
{
    setPin(enabled, KeyCodes::Buttons.start.altText);
}
// ----------------------------------------------------------------------
void Hal::setMacro14(bool enabled)
{
    setPin(enabled, KeyCodes::Buttons.macro10.altText);
}
// ----------------------------------------------------------------------
void Hal::setMacro15(bool enabled)
{
    setPin(enabled, KeyCodes::Buttons.continuous.altText);
}
// ----------------------------------------------------------------------
void Hal::setMacro16(bool enabled)
{
    setPin(enabled, KeyCodes::Buttons.step.altText);
}
// ----------------------------------------------------------------------
void Hal::setPin(bool enabled, size_t pinNumber, const char* pinName)
{
    *mHalCout << "hal   " << pinName << ((enabled) ? " enabled" : " disabled") << " (pin # " << pinNumber << ")"
              << endl;
    *(memory->out.button_pin[pinNumber]) = enabled;
}
// ----------------------------------------------------------------------
void Hal::setPin(bool enabled, const char* pinName)
{
    std::string fullyQualifiedPinName = std::string(mComponentPrefix) + ".button." + pinName;
    assert(mButtonNameToIdx.find(fullyQualifiedPinName) != mButtonNameToIdx.end());
    size_t pinNumber = mButtonNameToIdx[fullyQualifiedPinName];
    setPin(enabled, pinNumber, pinName);
}
// ----------------------------------------------------------------------
void Hal::setJogCounts(const HandWheelCounters& counters)
{
    // If axis is not homed we need to ask Teleop mode but we need to bypass that if machine is homed
    // https://forum.linuxcnc.org/49-basic-configuration/40581-how-to-configure-a-xhc-whb04b-pendant
    if      (*memory->out.axisXSelect && false == *memory->in.JointXisHomed) {requestTeleopMode(true);}
    else if (*memory->out.axisYSelect && false == *memory->in.JointYisHomed) {requestTeleopMode(true);}
    else if (*memory->out.axisZSelect && false == *memory->in.JointZisHomed) {requestTeleopMode(true);}
    else if (*memory->out.axisASelect && false == *memory->in.JointAisHomed) {requestTeleopMode(true);}
    else if (*memory->out.axisBSelect && false == *memory->in.JointBisHomed) {requestTeleopMode(true);}
    else if (*memory->out.axisCSelect && false == *memory->in.JointCisHomed) {requestTeleopMode(true);}
    {requestManualMode(true);}

    *memory->out.axisXJogCounts = counters.counts(HandWheelCounters::CounterNameToIndex::AXIS_X);
    *memory->out.axisYJogCounts = counters.counts(HandWheelCounters::CounterNameToIndex::AXIS_Y);
    *memory->out.axisZJogCounts = counters.counts(HandWheelCounters::CounterNameToIndex::AXIS_Z);
    *memory->out.axisAJogCounts = counters.counts(HandWheelCounters::CounterNameToIndex::AXIS_A);
    *memory->out.axisBJogCounts = counters.counts(HandWheelCounters::CounterNameToIndex::AXIS_B);
    *memory->out.axisCJogCounts = counters.counts(HandWheelCounters::CounterNameToIndex::AXIS_C);
        requestManualMode(false);
        requestTeleopMode(false);
}
// ----------------------------------------------------------------------
void Hal::setFunction(bool enabled)
{
    setPin(enabled, KeyCodes::Buttons.function.text);
}
// ----------------------------------------------------------------------
bool Hal::requestAutoMode(bool isRisingEdge)
{
    return requestMode(isRisingEdge, memory->out.doModeAuto, memory->in.isModeAuto);
}
// ----------------------------------------------------------------------
bool Hal::requestManualMode(bool isRisingEdge)
{
    return requestMode(isRisingEdge, memory->out.doModeManual, memory->in.isModeManual);
}
// ----------------------------------------------------------------------
bool Hal::requestMdiMode(bool isRisingEdge)
{
    return requestMode(isRisingEdge, memory->out.doModeMdi, memory->in.isModeMdi);
}
// ----------------------------------------------------------------------
bool Hal::requestTeleopMode(bool isRisingEdge)
{
    return requestMode(isRisingEdge, memory->out.doModeTeleop, memory->in.isModeTeleop);
}
// ----------------------------------------------------------------------
bool Hal::requestJointMode(bool isRisingEdge)
{
    return requestMode(isRisingEdge, memory->out.doModeJoint, memory->in.isModeJoint);
}
// ----------------------------------------------------------------------
bool Hal::requestMode(bool isRisingEdge, hal_bit_t *requestPin, hal_bit_t * modeFeedbackPin)
{
    if (isRisingEdge)
    {
        bool rv;
        if (true == *modeFeedbackPin)
        {
            // shortcut for mode request which is already active
            return true;
        }
        // request mode
        *requestPin = true;
        usleep(mHalRequestProfile.mode.holdMs * 1000);
        rv = waitForRequestedMode(modeFeedbackPin);
        *requestPin = false;
        usleep(mHalRequestProfile.mode.spaceMs * 1000);
        return rv;
    }
    else
    {
      // on button released always clear request
      *requestPin = false;
      return false;
    }
    return false;
}
// ----------------------------------------------------------------------
bool Hal::waitForRequestedMode(volatile hal_bit_t * condition)
{
    if(mIsSimulationMode)
    {
        return true;
    }
    useconds_t   timeoutMs   = mHalRequestProfile.mode.modeCheckLoopTimeoutMs;
    unsigned int maxTimeouts = mHalRequestProfile.mode.modeCheckLoops;
    unsigned int timeouts    = maxTimeouts;

    do
    {
        if (false == *condition)
        {
            usleep(timeoutMs * 1000);
        }
        else
        {
            return true;
        }
    } while ((false == *condition) && (--timeouts) > 0);
    if (false == *condition)
    {
        auto delay = (maxTimeouts - timeouts) * timeoutMs;
        std::cerr << "hal   failed to wait for requested mode. waited " << delay << "ms\n";
        return false;
    }
    else
    {
        return true;
    }
    return false;
}
// ----------------------------------------------------------------------
void Hal::toggleSpindleOverrideIncrease()
{
    if (*memory->out.spindleOverrideDoIncrease)
    {
        *memory->out.spindleOverrideDoIncrease = false;
    }
    else
    {
        *memory->out.spindleOverrideScale = 0.01;
        *memory->out.spindleOverrideDoIncrease = true;
    }
}
// ----------------------------------------------------------------------
void Hal::toggleSpindleOverrideDecrease()
{
    if (*memory->out.spindleOverrideDoDecrease)
    {
        *memory->out.spindleOverrideDoDecrease = false;
    }
    else
    {
        *memory->out.spindleOverrideScale = 0.01;
        *memory->out.spindleOverrideDoDecrease = true;
    }
}
// ----------------------------------------------------------------------
void Hal::toggleFeedrateIncrease()
{
    if (*memory->out.feedOverrideIncrease)
    {
        *memory->out.feedOverrideIncrease = false;
    }
    else
    {
        *memory->out.feedOverrideScale = 0.01;
        *memory->out.feedOverrideIncrease = true;
    }
}
// ----------------------------------------------------------------------
void Hal::toggleFeedrateDecrease()
{
    if (*memory->out.feedOverrideDecrease)
    {
        *memory->out.feedOverrideDecrease = false;
    }
    else
    {
        *memory->out.feedOverrideScale = 0.01;
        *memory->out.feedOverrideDecrease = true;
    }
}
}
