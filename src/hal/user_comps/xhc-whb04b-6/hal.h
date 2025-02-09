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

#ifndef __XHC_WHB04B_6_HAL_H
#define __XHC_WHB04B_6_HAL_H

// local includes
#include "pendant-types.h"

// system includes
#include <stdint.h>
#include <ostream>
#include <string>
#include <map>

// local library includes
#include <hal.h>
#include <sys/types.h>

namespace XhcWhb04b6 {

// forward declarations
class MetaButtonCodes;
class KeyCodes;
namespace Profiles {
struct ModeRequest;
struct SpindleRequest;
struct HalRequestProfile;
}

namespace Profiles {
// ----------------------------------------------------------------------
//! Defines hold and space delays for mode requests when pin is toggled.
struct ModeRequest {
    const useconds_t holdMs;
    const useconds_t spaceMs;
    const uint       modeCheckLoops;
    const useconds_t modeCheckLoopTimeoutMs;
};
// ----------------------------------------------------------------------
//! Defines hold and space delays for spindle increment/decrement requests.
struct SpindleRequest {
    const useconds_t holdMs;
    const useconds_t spaceMs;
};
// ----------------------------------------------------------------------
//! Overall hal request profile.
struct HalRequestProfile
{
    ModeRequest    mode;
    SpindleRequest spindle;
};
// ----------------------------------------------------------------------
//! A slow profile is reasonable for the BeagleBoneBlack especially when using Axis UI.
static constexpr HalRequestProfile halRequestSlowProfile()
{
    //! For BeagleBoneBlack this values seams reasonable, since it is a rather slow hardware.
    return HalRequestProfile
    {
        {
            // ModeRequest
            10, 10, // hold, space
            60,  5  // loops, timeout
        },
        {
            // SpindleRequest
            30, 30,  // hold, space
        }
    };
}
}
// ----------------------------------------------------------------------
//! HAL memory pointers. Each pointer represents an i/o hal pin.

class HalMemory
{
public:
    struct In
    {
    public:

        //! to be connected to \ref halui.flood.is-on
        hal_bit_t  * floodIsOn{nullptr};
        //! to be connected to \ref halui.mist.is-on
        hal_bit_t  * mistIsOn{nullptr};

        //! to be connected to \ref halui.axis.x.pos-feedback
        hal_float_t* axisXPosition{nullptr};
        //! to be connected to \ref halui.axis.y.pos-feedback
        hal_float_t* axisYPosition{nullptr};
        //! to be connected to \ref halui.axis.z.pos-feedback
        hal_float_t* axisZPosition{nullptr};
        //! to be connected to \ref halui.axis.a.pos-feedback
        hal_float_t* axisAPosition{nullptr};
        //! to be connected to \ref halui.axis.b.pos-feedback
        hal_float_t* axisBPosition{nullptr};
        //! to be connected to \ref halui.axis.c.pos-feedback
        hal_float_t* axisCPosition{nullptr};

        //! to be connected to \ref halui.axis.x.pos-relative
        hal_float_t* axisXPositionRelative{nullptr};
        //! to be connected to \ref halui.axis.y.pos-relative
        hal_float_t* axisYPositionRelative{nullptr};
        //! to be connected to \ref halui.axis.z.pos-relative
        hal_float_t* axisZPositionRelative{nullptr};
        //! to be connected to \ref halui.axis.a.pos-relative
        hal_float_t* axisAPositionRelative{nullptr};
        //! to be connected to \ref halui.axis.b.pos-relative
        hal_float_t* axisBPositionRelative{nullptr};
        //! to be connected to \ref halui.axis.c.pos-relative
        hal_float_t* axisCPositionRelative{nullptr};

        //! to be connected to \ref halui.spindle.is-on
        hal_bit_t  * spindleIsOn{nullptr};
        //! to be connected to \ref halui.spindle-override.value
        hal_float_t* spindleOverrideValue{nullptr};
        //! To be connected to an encoded and correctly scaled value of an spindle feedback signal.
        //! See also \ref encoder and \ref scale.
        hal_float_t* spindleSpeedCmd{nullptr};

        //! to be connected to \ref halui.max-velocity.value
        hal_float_t* feedOverrideMaxVel{nullptr};
        //! to be connected to \ref halui.feed-override.value
        hal_float_t* feedOverrideValue{nullptr};

        //! to be connected to \ref halui.program.is-running
        hal_bit_t* isProgramRunning{nullptr};
        //! to be connected to \ref halui.program.is-paused
        hal_bit_t* isProgramPaused{nullptr};
        //! to be connected to \ref halui.program.is-idle
        hal_bit_t* isProgramIdle{nullptr};

        //! to be connected to \ref halui.mode.is-auto
        hal_bit_t* isModeAuto{nullptr};
        //! to be connected to \ref halui.mode.is-joint
        hal_bit_t* isModeJoint{nullptr};
        //! to be connected to \ref halui.mode.is-manual
        hal_bit_t* isModeManual{nullptr};
        //! to be connected to \ref halui.mode.is-mdi
        hal_bit_t* isModeMdi{nullptr};
        //! to be connected to \ref halui.mode.is-teleop
        hal_bit_t* isModeTeleop{nullptr};


        // If axis is not homed we need to ask Teleop mode but we need to bypass that if machine is homed
        // https://forum.linuxcnc.org/49-basic-configuration/40581-how-to-configure-a-xhc-whb04b-pendant
        hal_bit_t* JointXisHomed{nullptr};
        hal_bit_t* JointYisHomed{nullptr};
        hal_bit_t* JointZisHomed{nullptr};
        hal_bit_t* JointAisHomed{nullptr};
        hal_bit_t* JointBisHomed{nullptr};
        hal_bit_t* JointCisHomed{nullptr};




        //! to be connected to \ref halui.machine.is-on
        hal_bit_t* isMachineOn{nullptr};

        In();
    };

    struct Out
    {
    public:
        hal_bit_t* button_pin[64] = {0};

        //! to be connected to \ref halui.flood.off
        hal_bit_t  * floodStop{nullptr};
        //! to be connected to \ref halui.flood.on
        hal_bit_t  * floodStart{nullptr};

        //! to be connected to \ref halui.mist.off
        hal_bit_t  * mistStop{nullptr};
        //! to be connected to \ref halui.mist.on
        hal_bit_t  * mistStart{nullptr};

        //! to be connected to \ref axis.x.jog-counts
        hal_s32_t* axisXJogCounts{nullptr};
        //! to be connected to \ref axis.y.jog-counts
        hal_s32_t* axisYJogCounts{nullptr};
        //! to be connected to \ref axis.z.jog-counts
        hal_s32_t* axisZJogCounts{nullptr};
        //! to be connected to \ref axis.a.jog-counts
        hal_s32_t* axisAJogCounts{nullptr};
        //! to be connected to \ref axis.b.jog-counts
        hal_s32_t* axisBJogCounts{nullptr};
        //! to be connected to \ref axis.c.jog-counts
        hal_s32_t* axisCJogCounts{nullptr};

        //! to be connected to \ref axis.x.jog-enable
        hal_bit_t* axisXJogEnable{nullptr};
        //! to be connected to \ref axis.y.jog-enable
        hal_bit_t* axisYJogEnable{nullptr};
        //! to be connected to \ref axis.z.jog-enable
        hal_bit_t* axisZJogEnable{nullptr};
        //! to be connected to \ref axis.a.jog-enable
        hal_bit_t* axisAJogEnable{nullptr};
        //! to be connected to \ref axis.b.jog-enable
        hal_bit_t* axisBJogEnable{nullptr};
        //! to be connected to \ref axis.c.jog-enable
        hal_bit_t* axisCJogEnable{nullptr};

        //! to be connected to \ref axis.x.jog-scale
        hal_float_t* axisXJogScale{nullptr};
        //! to be connected to \ref axis.y.jog-scale
        hal_float_t* axisYJogScale{nullptr};
        //! to be connected to \ref axis.z.jog-scale
        hal_float_t* axisZJogScale{nullptr};
        //! to be connected to \ref axis.a.jog-scale
        hal_float_t* axisAJogScale{nullptr};
        //! to be connected to \ref axis.b.jog-scale
        hal_float_t* axisBJogScale{nullptr};
        //! to be connected to \ref axis.c.jog-scale
        hal_float_t* axisCJogScale{nullptr};

        //! to be connected to \ref axis.x.jog-vel-mode
        hal_bit_t* axisXSetVelocityMode{nullptr};
        //! to be connected to \ref axis.y.jog-vel-mode
        hal_bit_t* axisYSetVelocityMode{nullptr};
        //! to be connected to \ref axis.z.jog-vel-mode
        hal_bit_t* axisZSetVelocityMode{nullptr};
        //! to be connected to \ref axis.a.jog-vel-mode
        hal_bit_t* axisASetVelocityMode{nullptr};
        //! to be connected to \ref axis.b.jog-vel-mode
        hal_bit_t* axisBSetVelocityMode{nullptr};
        //! to be connected to \ref axis.c.jog-vel-mode
        hal_bit_t* axisCSetVelocityMode{nullptr};

        hal_bit_t* feedValueSelected_2{nullptr};
        hal_bit_t* feedValueSelected_5{nullptr};
        hal_bit_t* feedValueSelected_10{nullptr};
        hal_bit_t* feedValueSelected_30{nullptr};
        hal_bit_t* feedValueSelected_60{nullptr};
        hal_bit_t* feedValueSelected_100{nullptr};
        hal_bit_t* feedValueSelected_lead{nullptr};
        hal_bit_t* feedValueSelected_mpg_feed{nullptr};
        hal_bit_t* feedValueSelected_continuous{nullptr};
        hal_bit_t* feedValueSelected_step{nullptr};

        //! to be connected to \ref  \ref halui.feed-override.scale
        hal_float_t* feedOverrideScale{nullptr};
        //! to be connected to \ref halui.feed-override.decrease
        hal_bit_t  * feedOverrideDecrease{nullptr};
        //! to be connected to \ref halui.feed-override.increase
        hal_bit_t  * feedOverrideIncrease{nullptr};

        //! to be connected to \ref halui.spindle.start
        hal_bit_t* spindleStart{nullptr};
        //! to be connected to \ref halui.spindle.stop
        hal_bit_t* spindleStop{nullptr};
        //! to be connected to \ref halui.spindle.forward
        hal_bit_t* spindleDoRunForward{nullptr};
        //! to be connected to \ref halui.spindle.reverse
        hal_bit_t* spindleDoRunReverse{nullptr};
        //! to be connected to halui.spindle.decrease
        hal_bit_t* spindleDoDecrease{nullptr};
        //! to be connected to halui.spindle.increase
        hal_bit_t* spindleDoIncrease{nullptr};
        //! to be connected to halui.spindle-override.decrease
        hal_bit_t* spindleOverrideDoDecrease{nullptr};
        //! to be connected to halui.spindle-override.increase
        hal_bit_t* spindleOverrideDoIncrease{nullptr};
        //! to be connected to \ref  \ref halui.spindle-override.scale
        hal_float_t* spindleOverrideScale{nullptr};

        //!to be connected to \ref halui.axis.N.select
        hal_bit_t* axisXSelect{nullptr};
        //!to be connected to \ref halui.axis.N.select
        hal_bit_t* axisYSelect{nullptr};
        //!to be connected to \ref halui.axis.N.select
        hal_bit_t* axisZSelect{nullptr};
        //!to be connected to \ref halui.axis.N.select
        hal_bit_t* axisASelect{nullptr};
        //!to be connected to \ref halui.axis.N.select
        hal_bit_t* axisBSelect{nullptr};
        //!to be connected to \ref halui.axis.N.select
        hal_bit_t* axisCSelect{nullptr};

        //! reflects the pendant's idle state
        hal_bit_t* isPendantSleeping{nullptr};
        //! reflects pendant's connectivity
        hal_bit_t* isPendantConnected{nullptr};

        //! to be connected to \ref halui.program.run
        hal_bit_t* doRunProgram{nullptr};
        //! to be connected to \ref halui.program.pause
        hal_bit_t* doPauseProgram{nullptr};
        //! to be connected to \ref halui.program.resume
        hal_bit_t* doResumeProgram{nullptr};
        //! to be connected to \ref halui.program.stop
        hal_bit_t* doStopProgram{nullptr};

        //! to be connected to \ref halui.mode.auto
        hal_bit_t* doModeAuto{nullptr};
        //! to be connected to \ref halui.mode.joint
        hal_bit_t* doModeJoint{nullptr};
        //! to be connected to \ref halui.mode.manual
        hal_bit_t* doModeManual{nullptr};
        //! to be connected to \ref halui.mode.mdi
        hal_bit_t* doModeMdi{nullptr};
        //! to be connected to \ref halui.mode.teleop
        hal_bit_t* doModeTeleop{nullptr};

        //! to be connected to \ref halui.machine.on
        hal_bit_t* doMachineOn{nullptr};
        //! to be connected to \ref halui.machine.off
        hal_bit_t* doMachineOff{nullptr};

        Out();
    };

    In  in;
    Out out;

    HalMemory();
    ~HalMemory();
};
// ----------------------------------------------------------------------
//! HAL and related parameters
class Hal
{
public:
    Hal(Profiles::HalRequestProfile halRequestProfile=Profiles::halRequestSlowProfile());
    ~Hal();
    //! Initializes HAL memory and pins according to simulation mode. Must not be called more than once.
    //! If \ref mIsSimulationMode is true heap memory will be used, shared HAL memory otherwise.
    //! \ref setIsSimulationMode() must be set before accordingly
    void init(const MetaButtonCodes* metaButtons, const KeyCodes& codes);
    //! \return true if void init(const MetaButtonCodes*, const KeyCodes&) has been called beforehand,
    //! false otherwise
    bool isInitialized() const { return mIsInitialized; };
    //! \return true if simulation mode is enabled, false otherwise
    bool isSimulationModeEnabled() const;
    //! indicates the program has been invoked in hal mode or normal
    void setSimulationMode(bool isSimulationMode);
    int getHalComponentId() const;
    const char* getHalComponentName() const;
    //! Enables verbose hal output.
    //! \param enable true to enable hal messages, disable otherwise
    void enableVerbose(bool enable);
    //! If set indicates that no other axis is active.
    //! \param enabled true if no axis is active, false otherwise
    void setNoAxisActive(bool enabled);
    //! Sets the A axis to active or disables the same.
    //! \param enabled true if axis should be the one and only active
    void setAxisXActive(bool enabled);
    //! \sa setAxisXActive(bool)
    void setAxisYActive(bool enabled);
    //! \sa setAxisXActive(bool)
    void setAxisZActive(bool enabled);
    //! \sa setAxisXActive(bool)
    void setAxisAActive(bool enabled);
    //! \sa setAxisXActive(bool)
    void setAxisBActive(bool enabled);
    //! \sa setAxisXActive(bool)
    void setAxisCActive(bool enabled);

    //! Sets the new feed rate. The step mode must be set accordingly.
    //! \param feedRate the new feed rate independent of step mode
    void setStepSize(const real_t feedRate);
    //! If lead is active.
    void setLead();
    //! Sets the hal state of the respective pin (reset). Usually called in case the reset
    //! button is pressed or released. The pin should be connected to \ref halui.estop.activate.
    //! \param enabled the new pin value, (true if the button was pressed, false otherwise)
    //! \ref Hal::halInit(const WhbSoftwareButton* , size_t , const WhbKeyCodes&)
    //! \sa setReset(bool)
    void setReset(bool enabled);
    //! \sa setStop(bool)
    void setStop(bool enabled);
    //! Toggles the start/pause/resume states.
    //! \param enabled true if start/resume is requested, false otherwise
    //! \param pinNumber \sa setStart(bool, size_t)
    void setStart(bool enabled);

    //! Returns the machine status.
    //! \sa HalMemory::In::isMachineOn
    //! \return true if machine is on, false otherwise
    bool getIsMachineOn() const;

    //! Writes the pendant sleeping status to hal.
    //! \sa HalMemory::Out::isPendantSleeping
    //! \param isSleeping true if sleeping, false otherwise
    void setIsPendantSleeping(bool isSleeping);
    //! Returns the pendant sleep state as written to hal.
    //! \sa setIsPendantSleeping(bool)
    bool getIsPendantSleeping() const;
    //! Writes the pendant connection status to hal.
    //! \sa HalMemory::Out::isPendantConnected
    //! \param isConnected true if connected, false otherwise
    void setIsPendantConnected(bool isConnected);
    //! Returns the pendant connection state as written to hal.
    //! \sa setIsPendantConnected(bool)
    //! \return true if connected, false otherwise
    bool getIsPendantConnected() const;

    //! \sa feedOverrideIncrease(bool, size_t)
    void setFeedPlus(bool enabled);
    //! \sa feedOverrideDecrease(bool, size_t)
    void setFeedMinus(bool enabled);
    //! Returns the current Max Velocity value.
    //! \sa Hal::In::feedOverrideMaxVel
    real_t getFeedOverrideMaxVel() const;
    //! Returns the current feed override value.
    //! \sa Hal::In::feedOverrideValue
    //! \return the current feed override value v: 0 <= v <= 1
    real_t getFeedOverrideValue() const;
    //! \xrefitem HalMemory::Out::feedOverrideScale setter
    void setFeedOverrideScale(real_t scale);

    //! Propagates the feed value 0.001 selection state to hal.
    //! \sa Hal::Out::feedValueSelected_2
    //! \param selected true if 0.001 is selected, false otherwise
    void setFeedValueSelected2(bool selected);
    //! Propagates the feed value 0.01 selection state to hal.
    //! \sa Hal::Out::feedValueSelected_5
    //! \param selected true if 0.01 is selected, false otherwise
    void setFeedValueSelected5(bool selected);
    //! Propagates the feed value 0.1 selection state to hal.
    //! \sa Hal::Out::feedValueSelected_10
    //! \param selected true if 0.1 is selected, false otherwise
    void setFeedValueSelected10(bool selected);
    //! Propagates the feed value 1.0 selection state to hal.
    //! \sa Hal::Out::feedValueSelected_30
    //! \param selected true if 1.0 is selected, false otherwise
    void setFeedValueSelected30(bool selected);
    //! Propagates the feed value 60% selection state to hal.
    //! \sa Hal::Out::feedValueSelected_60
    //! \param selected true if 60% is selected, false otherwise
    void setFeedValueSelected60(bool selected);
    //! Propagates the feed value 100% selection state to hal.
    //! \sa Hal::Out::feedValueSelected_100
    //! \param selected true if 100% is selected, false otherwise
    void setFeedValueSelected100(bool selected);
    //! Propagates the feed value Lead selection state to hal.
    //! \sa Hal::Out::feedValueSelected_lead
    //! \param selected true if Lead is selected, false otherwise
    void setFeedValueSelectedLead(bool selected);

    //! Returns the spindle speed.
    //! \return the spindle speed in rounds per second
    real_t getspindleSpeedCmd() const;
    real_t getspindleSpeedChangeIncrease() const;
    real_t getspindleSpeedChangeDecrease() const;
    //! Returns the current spindle override value.
    //! \sa Hal::In::spindleOverrideValue
    //! \return the current spindle override value v: 0 <= v <= 1
    real_t getSpindleOverrideValue() const;
    //! \sa setSpindleOverridePlus(bool, size_t)
    void setSpindleOverridePlus(bool enabled);
    //! \sa setSpindleOverrideMinus(bool, size_t)
    void setSpindleOverrideMinus(bool enabled);
    //! \sa setFunction(bool, size_t)
    void setFunction(bool enabled);
    //! Requests machine to search home for all axis. \ref halui.home-all
    void setMachineHomingAll(bool enabled);
    //! \sa setSafeZ(bool, size_t)
    void setSafeZ(bool enabled);
    //! \sa setWorkpieceHom(bool, size_t)
    void setWorkpieceHome(bool enabled);
    //! \sa toggleSpindleDirection(bool, size_t)
    void toggleSpindleDirection(bool enabled);
    //! \sa toggleSpindleOnOff(bool, size_t)
    void toggleSpindleOnOff(bool enabled);
    //! \sa toggleFloodOnOff(bool, size_t)
    void toggleFloodOnOff(bool enabled);
    //! \sa toggleMistOnOff(bool, size_t)
    void toggleMistOnOff(bool enabled);
    //! \sa setProbeZ(bool, size_t)
    void setProbeZ(bool enabled);
    //! \sa setMpgMode(bool, size_t)
    void setMpgMode(bool enabled);
    //! \sa setStepMode(bool, size_t)
    void setStepMode(bool enabled);
    //! \sa setConMode(bool, size_t)
    void setConMode(bool enabled);
    //! Sets the hal state of the macro pin. Usually called in case the macro
    //! button is pressed or released. A macro button can be any button
    //! when pressed together with the modifier key.
    //! \param enabled the new pin value, (true if the button was pressed, false otherwise)
    //! \param pinNumber The pin number in \ref HalMemory.
    void setMacro1(bool enabled);
    //! \sa setMacro1(bool, size_t)
    void setMacro2(bool enabled);
    //! \sa setMacro2(bool, size_t)
    void setMacro3(bool enabled);
    //! \sa setMacro3(bool, size_t)        // Hardcoded spindle speed
    void setMacro4(bool enabled);
    //! \sa setMacro4(bool, size_t)        // Hardcoded spindle speed
    void setMacro5(bool enabled);
    //! \sa setMacro5(bool, size_t)
    void setMacro6(bool enabled);
    //! \sa setMacro6(bool, size_t)
    void setMacro7(bool enabled);
    //! \sa setMacro7(bool, size_t)
    void setMacro8(bool enabled);
    //! \sa setMacro8(bool, size_t)        // Hardcoded toggle spindle direction
    void setMacro9(bool enabled);
    //! \sa setMacro9(bool, size_t)
    void setMacro10(bool enabled);
    //! \sa setMacro10(bool, size_t)        // Hardcoded Absolute/relative Dro
    void setMacro11(bool enabled);
    //! \sa setMacro11(bool, size_t)
    void setMacro12(bool enabled);
    //! \sa setMacro12(bool, size_t)
    void setMacro13(bool enabled);
    //! \sa setMacro13(bool, size_t)
    void setMacro14(bool enabled);
    //! \sa setMacro14(bool, size_t)
    void setMacro15(bool enabled);
    //! \sa setMacro15(bool, size_t)
    void setMacro16(bool enabled);
    //! \sa setMacro16(bool, size_t)

    void toggleSpindleOverrideIncrease();
    //! Inverts the spindle decrease signal state once.
    void toggleSpindleOverrideDecrease();
    //! Inverts the feedrate increase signal state once.
    //! \sa feedrateIncrease(int8_t)
    void toggleFeedrateIncrease();
    //! Inverts the feedrate decrease signal state once.
    //! \sa feedrateDecrease(int8_t)
    void toggleFeedrateDecrease();
    //! Writes the corresponding counter to to each axis' count.
    //! \param counters values to propagate to each axis
    void setJogCounts(const HandWheelCounters& counters);
    //! waits until a given pin is set to a requested state 
    //! \param state requested state
    //! \param pin requested pin to compare with
    void checkState(bool state, hal_bit_t *pin);

    //! Returns the axis position.
    //! \param absolute true absolute, false relative
    //! \return the absolute or relative position in machine units
    real_t getAxisXPosition(bool absolute) const;
    //! \xrefitem getAxisXPosition(bool)
    real_t getAxisYPosition(bool absolute) const;
    //! \xrefitem getAxisXPosition(bool)
    real_t getAxisZPosition(bool absolute) const;
    //! \xrefitem getAxisXPosition(bool)
    real_t getAxisAPosition(bool absolute) const;
    //! \xrefitem getAxisXPosition(bool)
    real_t getAxisBPosition(bool absolute) const;
    //! \xrefitem getAxisXPosition(bool)
    real_t getAxisCPosition(bool absolute) const;


private:
    HalMemory* memory{nullptr};
    std::map <std::string, size_t> mButtonNameToIdx;
    bool                           mIsSimulationMode{false};
    bool                           mIsInitialized{false};
    const char* mName{"xhc-whb04b-6"};
    const char* mComponentPrefix{"whb"};
    int          mHalCompId{-1};
    std::ostream mDevNull{nullptr};
    std::ostream* mHalCout{nullptr};
    HandwheelStepmodes::Mode mStepMode;
    bool mIsSpindleDirectionForward{true};
    Profiles::HalRequestProfile mHalRequestProfile;

    //! //! Allocates new hal_bit_t pin according to \ref mIsSimulationMode. If \ref mIsSimulationMode then
    //! mallocs memory, hal_pin_bit_new allocation otherwise.
    //! \param pin_name the pin name when registered to hal
    //! \param ptr will point to the allocated memory
    //! \param s size in bytes
    //! \return != 0 on error, 0 otherwise
    int newSimulatedHalPin(char* pin_name, void** ptr, int s);
    //! \sa  newBitHalPin(hal_pin_dir_t, hal_bit_t**, int, const char*, ...)
    int newHalFloat(hal_pin_dir_t direction, hal_float_t** ptr, int componentId, const char* fmt, ...);
    //! \sa  newBitHalPin(hal_pin_dir_t, hal_bit_t**, int, const char*, ...)
    int newHalSigned32(hal_pin_dir_t direction, hal_s32_t** ptr, int componentId, const char* fmt, ...);
    //! \sa  newBitHalPin(hal_pin_dir_t, hal_bit_t**, int, const char*, ...)
    int newHalUnsigned32(hal_pin_dir_t direction, hal_u32_t** ptr, int componentId, const char* fmt, ...);
    //! \param direction module input or output
    //! \param ptr will point to the allocated memory
    //! \param componentId hal id
    //! \param fmt the pin name when registered to hal
    //! \param ... va args
    //! \return != 0 on error, 0 otherwise
    int newHalBit(hal_pin_dir_t direction, hal_bit_t** ptr, int componentId, const char* fmt, ...);
    //! allocates new hal pin according to \ref mIsSimulationMode
    //! \param pin pointer reference to the memory to be fred
    //! \post pin == nullptr
    void freeSimulatedPin(void** pin);
    //! \param enabled the new pin value
    //! \param pinNumber the pin number to set the value
    //! \param pinName arbitrary name for logging
    void setPin(bool enabled, size_t pinNumber, const char* pinName);
    //! \sa setPin(bool, size_t, const char*)
    void setPin(bool enabled, const char* pinName);
    //! Toggles program states; running, paused, resume.
    //! Should be called each time after setStart(true) (\sa setStart(bool)) to stay in sync.
    void toggleStartResumeProgram();

    void clearStartResumeProgramStates();
    //! \sa requestManualMode(bool)
    bool requestAutoMode(bool isRisingEdge);
    //! Requests manual mode if in MDI mode. Skips request if in AUTO mode.
    //! \sa requestMode(bool, hal_bit_t*, hal_bit_t*)
    //! \return true if machine has selected the mode, false otherwise
    bool requestManualMode(bool isRisingEdge);
    //! \sa requestManualMode(bool)
    bool requestMdiMode(bool isRisingEdge);
    //! \sa requestTeleopMode(bool)
    bool requestTeleopMode(bool isRisingEdge);
    //! \sa requestJointMode(bool)
    bool requestJointMode(bool isRisingEdge);

    //! Polls for condition with timeout and max loops count.
    //! Returns if condition is met or number of loops is exhausted.
    //! Experience on BeagleBoneBlack with Axis UI revealed that the delay until a mode is switched is
    //! approximately 80ms to 150ms.
    //! \param condition the condition to be polled
    //! \param timeout_ms delay in [ms] in between condition is checks
    //! \param max_timeouts maximum number of checks
    //! \return true if condition was met, false otherwise
    bool waitForRequestedMode(volatile hal_bit_t* condition);

    //! Requests machine mode such as auto, mdi, manual. When toggling it introduces hold and space delay.
    //! \sa mModesRequestProfile
    //! \param requestPin the (output) pin to toggle for requesting
    //! \param modeFeedbackPin the (input) pin reflecting if the mode is set
    //! \return on rising edge: true if the machine has selected or is in the desired mode, false otherwise;
    //! on falling edge: false
    bool requestMode(bool isRisingEdge, hal_bit_t* requestPin, hal_bit_t* modeFeedbackPin);
};
}
#endif
