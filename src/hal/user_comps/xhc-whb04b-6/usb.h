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

#pragma once

// system includes
#include <stdint.h>
#include <ostream>
#include <sys/time.h>

// forward declarations
struct libusb_device_handle;
struct libusb_context;
struct libusb_transfer;

namespace XhcWhb04b6 {

// forward declarations
class Hal;
class Usb;
// ----------------------------------------------------------------------
//! Axis coordinate structure as sent via usb.
//! Caution: do not reorder fields!
class UsbOutPackageAxisCoordinate
{
public:
    uint16_t integerValue;
    uint16_t fractionValue  :15;
    uint16_t coordinateSign :1;
    void setCoordinate(const float& coordinate);
    void clear();
} __attribute__((packed));
// ----------------------------------------------------------------------
class DisplayIndicatorStepMode
{
public:
    //! \see DisplayIndicatorBitFields::stepMode
    enum class StepMode : uint8_t
    {
        //! Displays "CON: <xxx>%" according to feed rotary button position:
        //! CON:2%, CON:5%, CON:10%, CON:30%, CON:60%, CON:100%.
            CON = 0x00,
        //! Displays "STP: <x.xxxx>" according to feed rotary button position:
        //! STP:0.001, STP:0.01, STP:0.1, STP:1.0.
        //! On 60%, 100% or Lead still displays "STP: 1.0" (firmware bug).
            STEP = 0x01,
        //! Displays "MPG: <xxx>%" according to feed rotary button position:
        //! MPG:2%, MPG:5%, MPG:10%, MPG:30%, MPG:60%, MPG:100%
            MPG = 0x02,
        //! Displays <xxx%> according to feed rotary button position:
        //! 2%, 5%, 10%, 30%, 60%, 100%.
            PERCENT = 0x03
    };
};
// ----------------------------------------------------------------------
//! Caution: do not reorder fields!
class DisplayIndicatorBitFields
{
public:
    //! \see DisplayIndicatorStepMode
    uint8_t stepMode            : 2;
    // TODO: investigate the exact meaning of the yet still unknown bit field
    //! unknown flags
    uint8_t unknown             : 4;
    //! if flag set displays "RESET", \ref stepMode otherwise
    uint8_t isReset             : 1;
    //! if flag set axis names are "X1" "X1" ... "C1", "X" "Y" ... "C" otherwise
    uint8_t isRelativeCoordinate : 1;
} __attribute__((packed));
// ----------------------------------------------------------------------
//! Caution: do not reorder fields!
union DisplayIndicator
{
public:
    uint8_t                   asByte;
    DisplayIndicatorBitFields asBitFields;
    DisplayIndicator();
} __attribute__((packed));
// ----------------------------------------------------------------------
//! Convenience structure for accessing data fields in output package stream.
//! Caution: do not reorder fields!
class UsbOutPackageData
{
public:
    //! constant: 0xfdfe
    uint16_t                    header;
    uint8_t                     seed;
    DisplayIndicator            displayModeFlags;
    UsbOutPackageAxisCoordinate row1Coordinate;
    UsbOutPackageAxisCoordinate row2Coordinate;
    UsbOutPackageAxisCoordinate row3Coordinate;
    //! on feed+/- button pressed shown on display
    uint16_t                    feedRate;
    //! on spindle+/- button pressed shown on display
    uint16_t                    spindleFeedRate;
    UsbOutPackageData();
    void clear();

private:
    // TODO: investigate if this is still needed. it was needed when copying data chunks to blocks to avoid invalid read
    uint8_t padding;
} __attribute__((packed));
// ----------------------------------------------------------------------
//! Convenience structure for accessing data in input package stream.
//! Caution: do not reorder fields!
class UsbInPackage
{
public:
    //! constant 0x04
    const uint8_t header;
    const uint8_t randomByte;
    const uint8_t buttonKeyCode1;
    const uint8_t buttonKeyCode2;
    const uint8_t rotaryButtonFeedKeyCode;
    const uint8_t rotaryButtonAxisKeyCode;
    const int8_t  stepCount;
    const uint8_t crc;
    UsbInPackage();
    UsbInPackage(const uint8_t notAvailable1, const uint8_t notAvailable2, const uint8_t buttonKeyCode1,
                 const uint8_t buttonKeyCode2, const uint8_t rotaryButtonFeedKeyCode,
                 const uint8_t rotaryButtonAxisKeyCode, const int8_t stepCount, const uint8_t crc);
}__attribute__((packed));
// ----------------------------------------------------------------------
//! This package is sent as last but one package before xhc-whb04-6 is powered off,
//! and is meant to be used with operator== for comparison.
class UsbEmptyPackage :
    public UsbInPackage
{
public:

    UsbEmptyPackage();
    //! caution: it is not guaranteed that (this == \p other) == (\p other == this)
    bool operator==(const UsbInPackage& other) const;
    //! \see operator==(const UsbInPackage&)
    bool operator!=(const UsbInPackage& other) const;
} __attribute__((packed));
// ----------------------------------------------------------------------
//! This package is sent as last package before xhc-whb04-6 is powered off,
//! and is meant to be used with operator== for comparison.
class UsbSleepPackage :
    public UsbInPackage
{
public:
    UsbSleepPackage();
    //! caution: it is not guaranteed that (this == \p other) == (\p other == this)
    bool operator==(const UsbInPackage& other) const;
    //! \see operator==(const UsbInPackage&)
    bool operator!=(const UsbInPackage& other) const;
} __attribute__((packed));
// ----------------------------------------------------------------------
//! set of constant usb packages
class ConstantUsbPackages
{
public:
    const UsbSleepPackage sleepPackage;
    const UsbEmptyPackage emptyPackage;
    ConstantUsbPackages();
};
// ----------------------------------------------------------------------
class OnUsbInputPackageListener
{
public:
    //! callback with structured input data
    virtual void onInputDataReceived(const UsbInPackage& inPackage) = 0;

    virtual ~OnUsbInputPackageListener();
};
// ----------------------------------------------------------------------
class UsbRawInputListener
{
public:
    //! callback with raw input data
    virtual void onUsbDataReceived(struct libusb_transfer* transfer) = 0;

    virtual ~UsbRawInputListener();
};
// ----------------------------------------------------------------------
//! Convenience structure for initializing a transmission block.
//! Caution: do not reorder fields!
class UsbOutPackageBlockFields
{
public:
    //! constant 0x06
    uint8_t reportId;
    uint8_t __padding0;
    uint8_t __padding1;
    uint8_t __padding2;
    uint8_t __padding3;
    uint8_t __padding4;
    uint8_t __padding5;
    uint8_t __padding6;
    UsbOutPackageBlockFields();
    void init(const void* data);
} __attribute__((packed));
// ----------------------------------------------------------------------
//! Convenience structure for accessing a block as byte buffer.
class UsbOutPackageBlockBuffer
{
public:
    uint8_t asBytes[sizeof(UsbOutPackageBlockFields)];
} __attribute__((packed));
// ----------------------------------------------------------------------
union UsbOutPackageBlock
{
public:
    UsbOutPackageBlockBuffer asBuffer;
    UsbOutPackageBlockFields asBlock;
    UsbOutPackageBlock();
} __attribute__((packed));
// ----------------------------------------------------------------------
//! Convenience structure for initializing a transmission package's blocks.
//! Caution: do not reorder fields!
class UsbOutPackageBlocks
{
public:
    UsbOutPackageBlockFields block0;
    UsbOutPackageBlockFields block1;
    UsbOutPackageBlockFields block2;
    UsbOutPackageBlocks();
    void init(const UsbOutPackageData* data);
} __attribute__((packed));
// ----------------------------------------------------------------------
//! Convenience structure for casting data in package stream.
//! Caution: do not reorder fields!
union UsbOutPackageBuffer
{
public:
    UsbOutPackageBlock  asBlockArray[sizeof(UsbOutPackageBlocks) / sizeof(UsbOutPackageBlock)];
    UsbOutPackageBlocks asBlocks;
    UsbOutPackageBuffer();
} __attribute__((packed));
// ----------------------------------------------------------------------
//! Convenience structure for casting data in package stream.
//! Caution: do not reorder fields!
union UsbInPackageBuffer
{
public:
    const UsbInPackage asFields;
    uint8_t            asBuffer[sizeof(UsbInPackage)];
    UsbInPackageBuffer();
} __attribute__((packed));
// ----------------------------------------------------------------------
//! pendant sleep/idle state parameters
class SleepDetect
{
    friend Usb;

public:

    SleepDetect();

private:
    bool           mDropNextInPackage;
    struct timeval mLastWakeupTimestamp;
};
// ----------------------------------------------------------------------
//! USB related parameters
class Usb : public UsbRawInputListener
{
public:
    static const ConstantUsbPackages ConstantPackages;
    //! \param name device string used for printing messages
    //! \param onDataReceivedCallback called when received data is ready
    Usb(const char* name, OnUsbInputPackageListener& onDataReceivedCallback, Hal &hal);
    ~Usb();
    uint16_t getUsbVendorId() const;
    uint16_t getUsbProductId() const;
    const bool isDeviceOpen() const;
    libusb_context** getContextReference();
    libusb_context* getContext();
    void setContext(libusb_context* context);
    libusb_device_handle* getDeviceHandle();
    void setDeviceHandle(libusb_device_handle* deviceHandle);
    bool isWaitForPendantBeforeHalEnabled() const;
    bool getDoReconnect() const;
    void setDoReconnect(bool doReconnect);
    void onUsbDataReceived(struct libusb_transfer* transfer) override;
    void setSimulationMode(bool isSimulationMode);
    void setIsRunning(bool enableRunning);
    void requestTermination();
    //! Do offer a HAL memory before calling this method.
    //! \sa takeHalMemoryReference(HalMemory *)
    bool setupAsyncTransfer();
    void sendDisplayData();
    void enableVerboseTx(bool enable);
    void enableVerboseRx(bool enable);
    void enableVerboseInit(bool enable);
    bool init();
    void setWaitWithTimeout(uint8_t waitSecs);

    UsbOutPackageData& getOutputPackageData();

private:
    const uint16_t usbVendorId{0x10ce};
    const uint16_t usbProductId{0xeb93};
    libusb_context      * context{nullptr};
    libusb_device_handle* deviceHandle{nullptr};
    bool                mDoReconnect{false};
    bool                isWaitWithTimeout{false};
    bool                mIsSimulationMode{false};
    SleepDetect         sleepState;
    bool                mIsRunning{false};
    UsbInPackageBuffer  inputPackageBuffer;
    UsbOutPackageBuffer outputPackageBuffer;
    UsbOutPackageData   outputPackageData;
    OnUsbInputPackageListener& mDataHandler;
    void (* const mRawDataCallback)(struct libusb_transfer*);
    Hal                   & mHal;
    struct libusb_transfer* inTransfer{nullptr};
    struct libusb_transfer* outTransfer{nullptr};
    std::ostream devNull{nullptr};
    std::ostream* verboseTxOut{nullptr};
    std::ostream* verboseRxOut{nullptr};
    std::ostream* verboseInitOut{nullptr};
    const char  * mName{nullptr};
    uint8_t mWaitSecs{0};
};
// ----------------------------------------------------------------------
std::ostream& operator<<(std::ostream& os, const UsbOutPackageAxisCoordinate& coordinate);
std::ostream& operator<<(std::ostream& os, const UsbOutPackageData& data);
std::ostream& operator<<(std::ostream& os, const UsbOutPackageBlockFields& block);
std::ostream& operator<<(std::ostream& os, const UsbOutPackageBlocks& blocks);
}
