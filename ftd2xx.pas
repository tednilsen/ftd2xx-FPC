unit ftd2xx;

{-------------------------------------------------------------------------

  ftd2xx.pas

  FTDI device driver translation ftd2xx v1.4.27
  * https://github.com/tednilsen/ftd2xx-FPC

  * Future Technology Devices International Ltd.
    Home Page:      https://www.ftdichip.com
    Guide:          https://www.ftdichip.com/Support/Documents/ProgramGuides/D2XX_Programmer's_Guide(FT_000071).pdf
    Drivers:        https://www.ftdichip.com/Drivers/D2XX.htm

  Changelog:
  ----------

  2022.08.23
  * D2XX v1.4.27

-------------------------------------------------------------------------}

{ Copyright (c) 2001-2021 Future Technology Devices International Limited

  THIS SOFTWARE IS PROVIDED BY FUTURE TECHNOLOGY DEVICES INTERNATIONAL LIMITED "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
  FUTURE TECHNOLOGY DEVICES INTERNATIONAL LIMITED BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
  OF SUBSTITUTE GOODS OR SERVICES LOSS OF USE, DATA, OR PROFITS OR BUSINESS INTERRUPTION)
  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

  FTDI DRIVERS MAY BE USED ONLY IN CONJUNCTION WITH PRODUCTS BASED ON FTDI PARTS.

  FTDI DRIVERS MAY BE DISTRIBUTED IN ANY FORM AS LONG AS LICENSE INFORMATION IS NOT MODIFIED.

  IF A CUSTOM VENDOR ID AND/OR PRODUCT ID OR DESCRIPTION STRING ARE USED, IT IS THE
  RESPONSIBILITY OF THE PRODUCT MANUFACTURER TO MAINTAIN ANY CHANGES AND SUBSEQUENT WHQL
  RE-CERTIFICATION AS A RESULT OF MAKING THESE CHANGES.


  Module Name:

  ftd2xx.h

  Abstract:

  Native USB device driver for FTDI FT232x, FT245x, FT2232x, FT4232x, FT2233H and FT4233H devices
  FTD2XX library definitions

  Environment:

  kernel & user mode

  @file ftd2xx.h

}

{$MODE objfpc}{$H+}
{$I ftd2xx.inc}               // <--- edit to configure

interface

{$IFNDEF FTD2XX_H}            // <--- allready included the headers?
{$DEFINE FTD2XX_H}

uses
  dynlibs;

  function  libFTD_Initialize: boolean;                 // Load and Initialize the FTD library
  procedure libFTD_Finalize;                            // Unloads the library
  function  libFTD_Initialized: boolean; inline;        // Returns True if FTD is loaded and present
  function  libFTD_GetHandle: TLibHandle; inline;       // Returns the handle of FTD library
  procedure libFTD_SetHandle(libHandle: TLibHandle);    // Set the handle of FTD library (careful)
  function  libFTD_GetName: string; inline;             // Returns the library's name (without suffix)

  {........................................................................}

type

  TLPVOID           = pointer;

  TOverlapped       = packed record
    Internal        : DWord;
    InternalHigh    : DWord;
    Event           : THandle;
    case integer of
      0:( Offset      : DWord;
          OffsetHigh  : DWord);
      1:( Ptr         : pointer ) ;
  end;
  POverlapped = ^TOverlapped;

  TSecurityAttributes = packed record
    Length            : DWord;
    SecurityDescriptor: TLPVOID;
    InheritHandle     : LongBool;
  end;
  PSecurityAttributes = ^TSecurityAttributes;

  TFTHandle = pointer;
  PFTHandle = ^TFTHandle;

  TByte7    = packed array [0..6] of Byte;

  {$packEnum 4}

  {* @name FT_STATUS
   * @details Return status values for API calls. }
  TFTStatus = (
    FT_OK = 0,
    FT_INVALID_HANDLE,
    FT_DEVICE_NOT_FOUND,
    FT_DEVICE_NOT_OPENED,
    FT_IO_ERROR,
    FT_INSUFFICIENT_RESOURCES,
    FT_INVALID_PARAMETER,
    FT_INVALID_BAUD_RATE,

    FT_DEVICE_NOT_OPENED_FOR_ERASE,
    FT_DEVICE_NOT_OPENED_FOR_WRITE,
    FT_FAILED_TO_WRITE_DEVICE,
    FT_EEPROM_READ_FAILED,
    FT_EEPROM_WRITE_FAILED,
    FT_EEPROM_ERASE_FAILED,
    FT_EEPROM_NOT_PRESENT,
    FT_EEPROM_NOT_PROGRAMMED,
    FT_INVALID_ARGS,
    FT_NOT_SUPPORTED,
    FT_OTHER_ERROR,
    FT_DEVICE_LIST_NOT_READY
  );
  PFTStatus = ^TFTStatus;

  {$minEnumSize default}

  {** @name FT_SUCCESS Macro
   * Macro to determine success of an API call.
   * @returns Non-zero for successful call. }
  function FT_SUCCESS(const status: TFTStatus): boolean; inline;

const

  // FT_OpenEx Flags
  // @see FT_OpenEx

  FT_OPEN_BY_SERIAL_NUMBER          = 1;
  FT_OPEN_BY_DESCRIPTION            = 2;
  FT_OPEN_BY_LOCATION               = 4;

  FT_OPEN_MASK                      = (FT_OPEN_BY_SERIAL_NUMBER or FT_OPEN_BY_DESCRIPTION or FT_OPEN_BY_LOCATION);

  // FT_ListDevices Flags (used in conjunction with FT_OpenEx Flags)
  // @see FT_ListDevices & FT_OpenEx

  FT_LIST_NUMBER_ONLY               = $80000000;
  FT_LIST_BY_INDEX                  = $40000000;
  FT_LIST_ALL                       = $20000000;

  FT_LIST_MASK                      = (FT_LIST_NUMBER_ONLY or FT_LIST_BY_INDEX or FT_LIST_ALL);

  // Standard baud rates supported by many implementations and applications.
  // @see FT_SetBaudRate

  FT_BAUD_300                       = 300;
  FT_BAUD_600                       = 600;
  FT_BAUD_1200                      = 1200;
  FT_BAUD_2400                      = 2400;
  FT_BAUD_4800                      = 4800;
  FT_BAUD_9600                      = 9600;
  FT_BAUD_14400                     = 14400;
  FT_BAUD_19200                     = 19200;
  FT_BAUD_38400                     = 38400;
  FT_BAUD_57600                     = 57600;
  FT_BAUD_115200                    = 115200;
  FT_BAUD_230400                    = 230400;
  FT_BAUD_460800                    = 460800;
  FT_BAUD_921600                    = 921600;

  // Word Lengths
  // @see FT_SetDataCharacteristics

  FT_BITS_8                         = 8;
  FT_BITS_7                         = 7;

  // Stop Bits
  // @see FT_SetDataCharacteristics

  FT_STOP_BITS_1                    = 0;
  FT_STOP_BITS_2                    = 2;

  // Parity
  // @see FT_SetDataCharacteristics

  FT_PARITY_NONE                    = 0;
  FT_PARITY_ODD                     = 1;
  FT_PARITY_EVEN                    = 2;
  FT_PARITY_MARK                    = 3;
  FT_PARITY_SPACE                   = 4;

  // Flow Control
  // @see FT_SetFlowControl

  FT_FLOW_NONE                      = $0000;
  FT_FLOW_RTS_CTS                   = $0100;
  FT_FLOW_DTR_DSR                   = $0200;
  FT_FLOW_XON_XOFF                  = $0400;

  // Purge rx and tx buffers
  // @see FT_Purge

  FT_PURGE_RX                       = 1;
  FT_PURGE_TX                       = 2;

  // Events
  // @see FT_SetEventNotification

  //typedef void (*PFT_EVENT_HANDLER)(DWORD,DWORD);

  FT_EVENT_RXCHAR                   = 1;
  FT_EVENT_MODEM_STATUS             = 2;
  FT_EVENT_LINE_STATUS              = 4;

  // Timeouts
  // @see FT_SetTimeouts

  FT_DEFAULT_RX_TIMEOUT             = 300;
  FT_DEFAULT_TX_TIMEOUT             = 300;

type

  {$packEnum 4}

  // @name Device Types
  // @details Known supported FTDI device types supported by this library
  TFTDevice = (
    FT_DEVICE_BM = 0,
    FT_DEVICE_AM,
    FT_DEVICE_100AX,
    FT_DEVICE_UNKNOWN,
    FT_DEVICE_2232C,
    FT_DEVICE_232R,
    FT_DEVICE_2232H,
    FT_DEVICE_4232H,
    FT_DEVICE_232H,
    FT_DEVICE_X_SERIES,
    FT_DEVICE_4222H_0,
    FT_DEVICE_4222H_1_2,
    FT_DEVICE_4222H_3,
    FT_DEVICE_4222_PROG,
    FT_DEVICE_900,
    FT_DEVICE_930,
    FT_DEVICE_UMFTPD3A,
    FT_DEVICE_2233HP,
    FT_DEVICE_4233HP,
    FT_DEVICE_2232HP,
    FT_DEVICE_4232HP,
    FT_DEVICE_233HP,
    FT_DEVICE_232HP,
    FT_DEVICE_2232HA,
    FT_DEVICE_4232HA
  );
  PFTDevice = TFTDevice;

  {$minEnumSize default}

const

  // @name Bit Modes
  // @see FT_SetBitMode FT_GetBitMode

  FT_BITMODE_RESET                        = $00;
  FT_BITMODE_ASYNC_BITBANG                = $01;
  FT_BITMODE_MPSSE                        = $02;
  FT_BITMODE_SYNC_BITBANG                 = $04;
  FT_BITMODE_MCU_HOST                     = $08;
  FT_BITMODE_FAST_SERIAL                  = $10;
  FT_BITMODE_CBUS_BITBANG                 = $20;
  FT_BITMODE_SYNC_FIFO                    = $40;

  // @name FT232R CBUS Options EEPROM values

  FT_232R_CBUS_TXDEN                      = $00;  //  Tx Data Enable
  FT_232R_CBUS_PWRON                      = $01;  //  Power On
  FT_232R_CBUS_RXLED                      = $02;  //  Rx LED
  FT_232R_CBUS_TXLED                      = $03;  //  Tx LED
  FT_232R_CBUS_TXRXLED                    = $04;  //  Tx and Rx LED
  FT_232R_CBUS_SLEEP                      = $05;  //  Sleep
  FT_232R_CBUS_CLK48                      = $06;  //  48MHz clock
  FT_232R_CBUS_CLK24                      = $07;  //  24MHz clock
  FT_232R_CBUS_CLK12                      = $08;  //  12MHz clock
  FT_232R_CBUS_CLK6                       = $09;  //  6MHz clock
  FT_232R_CBUS_IOMODE                     = $0A;  //  IO Mode for CBUS bit-bang
  FT_232R_CBUS_BITBANG_WR                 = $0B;  //  Bit-bang write strobe
  FT_232R_CBUS_BITBANG_RD                 = $0C;  //  Bit-bang read strobe
  FT_232R_CBUS0_RXF                       = $0D;  //  CBUS0 RXF#
  FT_232R_CBUS1_TXE                       = $0D;  //  CBUS1 TXE#
  FT_232R_CBUS2_RD                        = $0D;  //  CBUS2 RD#
  FT_232R_CBUS3_WR                        = $0D;  //  CBUS3 WR#

  // @name FT232H CBUS Options EEPROM values

  FT_232H_CBUS_TRISTATE                   = $00;  //  Tristate
  FT_232H_CBUS_TXLED                      = $01;  //  Tx LED
  FT_232H_CBUS_RXLED                      = $02;  //  Rx LED
  FT_232H_CBUS_TXRXLED                    = $03;  //  Tx and Rx LED
  FT_232H_CBUS_PWREN                      = $04;  //  Power Enable
  FT_232H_CBUS_SLEEP                      = $05;  //  Sleep
  FT_232H_CBUS_DRIVE_0                    = $06;  //  Drive pin to logic 0
  FT_232H_CBUS_DRIVE_1                    = $07;  //  Drive pin to logic 1
  FT_232H_CBUS_IOMODE                     = $08;  //  IO Mode for CBUS bit-bang
  FT_232H_CBUS_TXDEN                      = $09;  //  Tx Data Enable
  FT_232H_CBUS_CLK30                      = $0A;  //  30MHz clock
  FT_232H_CBUS_CLK15                      = $0B;  //  15MHz clock
  FT_232H_CBUS_CLK7_5                     = $0C;  //  7.5MHz clock

  // @name FT X Series CBUS Options EEPROM values

  FT_X_SERIES_CBUS_TRISTATE               = $00;  //  Tristate
  FT_X_SERIES_CBUS_TXLED                  = $01;  //  Tx LED
  FT_X_SERIES_CBUS_RXLED                  = $02;  //  Rx LED
  FT_X_SERIES_CBUS_TXRXLED                = $03;  //  Tx and Rx LED
  FT_X_SERIES_CBUS_PWREN                  = $04;  //  Power Enable
  FT_X_SERIES_CBUS_SLEEP                  = $05;  //  Sleep
  FT_X_SERIES_CBUS_DRIVE_0                = $06;  //  Drive pin to logic 0
  FT_X_SERIES_CBUS_DRIVE_1                = $07;  //  Drive pin to logic 1
  FT_X_SERIES_CBUS_IOMODE                 = $08;  //  IO Mode for CBUS bit-bang
  FT_X_SERIES_CBUS_TXDEN                  = $09;  //  Tx Data Enable
  FT_X_SERIES_CBUS_CLK24                  = $0A;  //  24MHz clock
  FT_X_SERIES_CBUS_CLK12                  = $0B;  //  12MHz clock
  FT_X_SERIES_CBUS_CLK6                   = $0C;  //  6MHz clock
  FT_X_SERIES_CBUS_BCD_CHARGER            = $0D;  //  Battery charger detected
  FT_X_SERIES_CBUS_BCD_CHARGER_N          = $0E;  //  Battery charger detected inverted
  FT_X_SERIES_CBUS_I2C_TXE                = $0F;  //  I2C Tx empty
  FT_X_SERIES_CBUS_I2C_RXF                = $10;  //  I2C Rx full
  FT_X_SERIES_CBUS_VBUS_SENSE             = $11;  //  Detect VBUS
  FT_X_SERIES_CBUS_BITBANG_WR             = $12;  //  Bit-bang write strobe
  FT_X_SERIES_CBUS_BITBANG_RD             = $13;  //  Bit-bang read strobe
  FT_X_SERIES_CBUS_TIMESTAMP              = $14;  //  Toggle output when a USB SOF token is received
  FT_X_SERIES_CBUS_KEEP_AWAKE             = $15;  //

  // @name Driver Types

  FT_DRIVER_TYPE_D2XX             = 0;
  FT_DRIVER_TYPE_VCP              = 1;

var

  {$IFNDEF WINDOWS}

  {** @noop FT_SetVIDPID
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * @par Summary
   * A command to include a custom VID and PID combination within the internal device list table.
   * This will allow the driver to load for the specified VID and PID combination.
   * @param dwVID Device Vendor ID (VID)
   * @param dwPID Device Product ID (PID)
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code.
   * @remarks
   * By default, the driver will support a limited set of VID and PID matched devices (VID 0x0403
   * with PIDs for standard FTDI devices only).
   * @n In order to use the driver with other VID and PID combinations the FT_SetVIDPID function
   * must be used prior to calling FT_ListDevices, FT_Open, FT_OpenEx or FT_CreateDeviceInfoList.
   * @note Extra function for non-Windows platforms to compensate for lack of .INF file to specify
   * Vendor and Product IDs. }
  FT_SetVIDPID: function(VID, PID: DWord): TFTStatus; FTD2XX_API;

  {** @noop FT_GetVIDPID
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * @par Summary
   * A command to retrieve the current VID and PID combination from within the internal device list table.
   * @param pdwVID Pointer to DWORD that will contain the internal VID
   * @param pdwPID Pointer to DWORD that will contain the internal PID
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code.
   * @remarks
   * @note Extra function for non-Windows platforms to compensate for lack of .INF file to specify Vendor and Product IDs.
   * @see FT_SetVIDPID. }
  FT_GetVIDPID: function(VID, PID: PDWord): TFTStatus; FTD2XX_API;

  {$ENDIF WINDOWS}

  {** @noop FT_CreateDeviceInfoList
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * This function builds a device information list and returns the number of D2XX devices connected to the
   * system. The list contains information about both unopen and open devices.
   * @param lpdwNumDevs Pointer to unsigned long to store the number of devices connected.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code.
   * @remarks
   * An application can use this function to get the number of devices attached to the system. It can then
   * allocate space for the device information list and retrieve the list using FT_GetDeviceInfoList or
   * FT_GetDeviceInfoDetail.
   * @n If the devices connected to the system change, the device info list will not be updated until
   * FT_CreateDeviceInfoList is called again.
   * @see FT_GetDeviceInfoList
   * @see FT_GetDeviceInfoDetail }
  FT_CreateDeviceInfoList: function(NumDevs: PLongWord): TFTStatus; FTD2XX_API;

type

  {$packEnum 4}

  // @name FT_DEVICE_LIST_INFO_NODE Device Information Flags
  // @par Summary
  // These flags are used in the Flags member of FT_DEVICE_LIST_INFO_NODE
  // to indicated the state of the device and speed of the device.
  TFTDevice_Flags = (
    FT_FLAGS_OPENED = 1,
    FT_FLAGS_HISPEED = 2
  );
  PFTDevice_Flags = TFTDevice_Flags;

  {$minEnumSize default}

  {**  @noop FT_DEVICE_LIST_INFO_NODE
   * @par Summary
   * This structure is used for passing information about a device back from the FT_GetDeviceInfoList function.
   * @see FT_GetDeviceInfoList }
  TFT_device_list_info_node = packed record
    Flags        : TFTDevice_Flags;
    Type_        : LongWord;
    ID           : LongWord;
    LocId        : DWord;
    SerialNumber : packed array [0..15] of Char;
    Description  : packed array [0..63] of Char;
    Handle       : TFTHandle;
  end;
  PFT_device_list_info_node = ^TFT_device_list_info_node;

var

  {** @noop FT_GetDeviceInfoList
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * This function returns a device information list and the number of D2XX devices in the list.
   * @param *pDest Pointer to an array of FT_DEVICE_LIST_INFO_NODE structures.
   * @param lpdwNumDevs Pointer to the number of elements in the array.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code.
   * @remarks
   * This function should only be called after calling FT_CreateDeviceInfoList. If the devices connected to the
   * system change, the device info list will not be updated until FT_CreateDeviceInfoList is called again.
   * Location ID information is not returned for devices that are open when FT_CreateDeviceInfoList is called.
   * Information is not available for devices which are open in other processes. In this case, the Flags
   * parameter of the FT_DEVICE_LIST_INFO_NODE will indicate that the device is open, but other fields will
   * be unpopulated.
   * @n The flag value is a 4-byte bit map containing miscellaneous data as defined Appendix A - Type
   * Definitions. Bit 0 (least significant bit) of this number indicates if the port is open (1) or closed (0). Bit 1
   * indicates if the device is enumerated as a high-speed USB device (2) or a full-speed USB device (0). The
   * remaining bits (2 - 31) are reserved.
   * @n The array of FT_DEVICE_LIST_INFO_NODES contains all available data on each device. The structure of
   * FT_DEVICE_LIST_INFO_NODES is given in the Appendix. The storage for the list must be allocated by
   * the application. The number of devices returned by FT_CreateDeviceInfoList can be used to do this.
   * When programming in Visual Basic, LabVIEW or similar languages, FT_GetDeviceInfoDetail may be
   * required instead of this function.
   * @note Please note that Windows CE does not support location IDs. As such, the Location ID parameter in the
   * structure will be empty.
   * @see FT_CreateDeviceInfoList }
  FT_GetDeviceInfoList: function(Dest: PFT_device_list_info_node; NumDevs: PLongWord): TFTStatus; FTD2XX_API;

  {** @noop FT_GetDeviceInfoDetail
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * This function returns an entry from the device information list.
   * @param dwIndex Index of the entry in the device info list.
   * @param lpdwFlags Pointer to unsigned long to store the flag value.
   * @param lpdwType Pointer to unsigned long to store device type.
   * @param lpdwID Pointer to unsigned long to store device ID.
   * @param lpdwLocId Pointer to unsigned long to store the device location ID.
   * @param lpSerialNumber Pointer to buffer to store device serial number as a nullterminated string.
   * @param lpDescription Pointer to buffer to store device description as a null-terminated string.
   * @param *pftHandle Pointer to a variable of type FT_HANDLE where the handle will be stored.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code.
   * @remarks
   * This function should only be called after calling FT_CreateDeviceInfoList. If the devices connected to the
   * system change, the device info list will not be updated until FT_CreateDeviceInfoList is called again.
   * @n The index value is zero-based.
   * @n The flag value is a 4-byte bit map containing miscellaneous data as defined Appendix A - Type
   * Definitions. Bit 0 (least significant bit) of this number indicates if the port is open (1) or closed (0). Bit 1
   * indicates if the device is enumerated as a high-speed USB device (2) or a full-speed USB device (0). The
   * remaining bits (2 - 31) are reserved.
   * @n Location ID information is not returned for devices that are open when FT_CreateDeviceInfoList is called.
   * Information is not available for devices which are open in other processes. In this case, the lpdwFlags
   * parameter will indicate that the device is open, but other fields will be unpopulated.
   * To return the whole device info list as an array of FT_DEVICE_LIST_INFO_NODE structures, use
   * FT_CreateDeviceInfoList.
   * @note Please note that Windows CE does not support location IDs. As such, the Location ID parameter in the
   * structure will be empty.
   * @see FT_CreateDeviceInfoList }
  FT_GetDeviceInfoDetail: function(Index: DWord; Flags, Type_, ID, LocId: PLongWord; SerialNumber, Description: pointer; Handle: PFTHandle): TFTStatus; FTD2XX_API;

  {** @noop FT_ListDevices
   * @par Summary
   * Gets information concerning the devices currently connected. This function can return information such
   * as the number of devices connected, the device serial number and device description strings, and the
   * location IDs of connected devices.
   * @param pvArg1 Meaning depends on dwFlags.
   * @param pvArg2 Meaning depends on dwFlags.
   * @param dwFlags Determines format of returned information.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code.
   * @remarks
   * This function can be used in a number of ways to return different types of information. A more powerful
   * way to get device information is to use the FT_CreateDeviceInfoList, FT_GetDeviceInfoList and
   * FT_GetDeviceInfoDetail functions as they return all the available information on devices.
   * In its simplest form, it can be used to return the number of devices currently connected. If
   * FT_LIST_NUMBER_ONLY bit is set in dwFlags, the parameter pvArg1 is interpreted as a pointer to a
   * DWORD location to store the number of devices currently connected.
   * @n It can be used to return device information: if FT_OPEN_BY_SERIAL_NUMBER bit is set in dwFlags, the
   * serial number string will be returned; if FT_OPEN_BY_DESCRIPTION bit is set in dwFlags, the product
   * description string will be returned; if FT_OPEN_BY_LOCATION bit is set in dwFlags, the Location ID will
   * be returned; if none of these bits is set, the serial number string will be returned by default.
   * @n It can be used to return device string information for a single device. If FT_LIST_BY_INDEX and
   * FT_OPEN_BY_SERIAL_NUMBER or FT_OPEN_BY_DESCRIPTION bits are set in dwFlags, the parameter
   * pvArg1 is interpreted as the index of the device, and the parameter pvArg2 is interpreted as a pointer to
   * a buffer to contain the appropriate string. Indexes are zero-based, and the error code
   * FT_DEVICE_NOT_FOUND is returned for an invalid index.
   * @n It can be used to return device string information for all connected devices. If FT_LIST_ALL and
   * FT_OPEN_BY_SERIAL_NUMBER or FT_OPEN_BY_DESCRIPTION bits are set in dwFlags, the parameter
   * pvArg1 is interpreted as a pointer to an array of pointers to buffers to contain the appropriate strings and
   * the parameter pvArg2 is interpreted as a pointer to a DWORD location to store the number of devices
   * currently connected. Note that, for pvArg1, the last entry in the array of pointers to buffers should be a
   * NULL pointer so the array will contain one more location than the number of devices connected.
   * @n The location ID of a device is returned if FT_LIST_BY_INDEX and FT_OPEN_BY_LOCATION bits are set in
   * dwFlags. In this case the parameter pvArg1 is interpreted as the index of the device, and the parameter
   * pvArg2 is interpreted as a pointer to a variable of type long to contain the location ID. Indexes are
   * zerobased, and the error code FT_DEVICE_NOT_FOUND is returned for an invalid index. Please note that
   * Windows CE and Linux do not support location IDs.
   * @n The location IDs of all connected devices are returned if FT_LIST_ALL and FT_OPEN_BY_LOCATION bits
   * are set in dwFlags. In this case, the parameter pvArg1 is interpreted as a pointer to an array of variables
   * of type long to contain the location IDs, and the parameter pvArg2 is interpreted as a pointer to a
   * DWORD location to store the number of devices currently connected.
   * @see FT_CreateDeviceInfoList
   * @see FT_GetDeviceInfoList
   * @see FT_GetDeviceInfoDetail }
  FT_ListDevices: function(Arg1, Arg2: pointer; Flags: DWord): TFTStatus; FTD2XX_API;

  {** @noop FT_Open
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * Open the device and return a handle which will be used for subsequent accesses.
   * @param deviceNumber Index of the device to open. Indices are 0 based.
   * @param pHandle Pointer to a variable of type FT_HANDLE where the handle will be stored. This handle must
   * be used to access the device.
   * @return
   * FT_OK if successful, otherwise the return value is an FT error code.
   * @remarks
   * Although this function can be used to open multiple devices by setting iDevice to 0, 1, 2 etc. there is no
   * ability to open a specific device. To open named devices, use the function FT_OpenEx.
   * @see FT_OpenEx. }
  FT_Open: function(deviceNumber: integer; Handle: PFTHandle): TFTStatus; FTD2XX_API;

  {** @noop FT_OpenEx
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * Open the specified device and return a handle that will be used for subsequent accesses. The device can
   * be specified by its serial number, device description or location.
   * @n This function can also be used to open multiple devices simultaneously. Multiple devices can be specified
   * by serial number, device description or location ID (location information derived from the physical
   * location of a device on USB). Location IDs for specific USB ports can be obtained using the utility
   * USBView and are given in hexadecimal format. Location IDs for devices connected to a system can be
   * obtained by calling FT_GetDeviceInfoList or FT_ListDevices with the appropriate flags.
   * @param pvArg1 Pointer to an argument whose type depends on the value of dwFlags.
   * It is normally be interpreted as a pointer to a null terminated string.
   * @param dwFlags FT_OPEN_BY_SERIAL_NUMBER, FT_OPEN_BY_DESCRIPTION or FT_OPEN_BY_LOCATION.
   * @param pHandle Pointer to a variable of type FT_HANDLE where the handle will be
   * stored. This handle must be used to access the device.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code.
   * @remarks
   * The parameter specified in pvArg1 depends on dwFlags: if dwFlags is FT_OPEN_BY_SERIAL_NUMBER,
   * pvArg1 is interpreted as a pointer to a null-terminated string that represents the serial number of the
   * device; if dwFlags is FT_OPEN_BY_DESCRIPTION, pvArg1 is interpreted as a pointer to a nullterminated
   * string that represents the device description; if dwFlags is FT_OPEN_BY_LOCATION, pvArg1
   * is interpreted as a long value that contains the location ID of the device. Please note that Windows CE
   * and Linux do not support location IDs.
   * @n ftHandle is a pointer to a variable of type FT_HANDLE where the handle is to be stored. This handle must
   * be used to access the device. }
  FT_OpenEx: function(Arg1: pointer; Flags: DWord; Handle: PFTHandle): TFTStatus; FTD2XX_API;

  {** @noop FT_Close
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * Close an open device.
   * @param ftHandle Handle of the device.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code. }
  FT_Close: function(Handle: TFTHandle): TFTStatus; FTD2XX_API;

  {** @noop FT_Read
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * Read data from the device.
   * @param ftHandle Handle of the device.
   * @param lpBuffer Pointer to the buffer that receives the data from the device.
   * @param dwBytesToRead Number of bytes to be read from the device.
   * @param lpdwBytesReturned Pointer to a variable of type DWORD which receives the number of
   * bytes read from the device.
   * @returns
   * FT_OK if successful, FT_IO_ERROR otherwise. $see FT_STATUS
   * @remarks
   * FT_Read always returns the number of bytes read in lpdwBytesReturned.
   * @n This function does not return until dwBytesToRead bytes have been read into the buffer. The number of
   * bytes in the receive queue can be determined by calling FT_GetStatus or FT_GetQueueStatus, and
   * passed to FT_Read as dwBytesToRead so that the function reads the device and returns immediately.
   * When a read timeout value has been specified in a previous call to FT_SetTimeouts, FT_Read returns
   * when the timer expires or dwBytesToRead have been read, whichever occurs first. If the timeout
   * occurred, FT_Read reads available data into the buffer and returns FT_OK.
   * @n An application should use the function return value and lpdwBytesReturned when processing the buffer.
   * If the return value is FT_OK, and lpdwBytesReturned is equal to dwBytesToRead then FT_Read has
   * completed normally. If the return value is FT_OK, and lpdwBytesReturned is less then dwBytesToRead
   * then a timeout has occurred and the read has been partially completed. Note that if a timeout occurred
   * and no data was read, the return value is still FT_OK.
   * @n A return value of FT_IO_ERROR suggests an error in the parameters of the function, or a fatal error like a
   * USB disconnect has occurred. }
  FT_Read: function(Handle: TFTHandle; Buffer: pointer; BytesToRead: DWord; BytesReturned: PDWord): TFTStatus; FTD2XX_API;

  {** @noop FT_Write
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * Write data to the device.
   * @param ftHandle Handle of the device.
   * @param lpBuffer Pointer to the buffer that contains the data to be written to the device.
   * @param dwBytesToWrite Number of bytes to write to the device.
   * @param lpdwBytesWritten Pointer to a variable of type DWORD which receives the number of
   * bytes written to the device.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code. }
  FT_Write: function(Handle: TFTHandle; Buffer: pointer; BytesToWrite: DWord; BytesWritten: PDWord): TFTStatus; FTD2XX_API;

  {** @noop FT_SetBaudRate
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * This function sets the baud rate for the device.
   * @param ftHandle Handle of the device.
   * @param dwBaudRate Baud rate.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code }
  FT_SetBaudRate: function(Handle: TFTHandle; BaudRate: LongWord): TFTStatus; FTD2XX_API;

  {** @noop FT_SetDivisor
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * This function sets the baud rate for the device. It is used to set non-standard baud rates.
   * @param ftHandle Handle of the device.
   * @param usDivisor Divisor.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code.
   * @remarks
   * This function is no longer required as FT_SetBaudRate will now automatically calculate the required
   * divisor for a requested baud rate. The application note "Setting baud rates for the FT8U232AM" is
   * available from the Application Notes section of the FTDI website describes how to calculate the divisor for
   * a non-standard baud rate. }
  FT_SetDivisor: function(Handle: TFTHandle; Divisor: Word): TFTStatus; FTD2XX_API;

  {** @noop FT_SetDataCharacteristics
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * This function sets the data characteristics for the device.
   * @param ftHandle Handle of the device.
   * @param uWordLength Number of bits per word - must be FT_BITS_8 or FT_BITS_7.
   * @param uStopBits Number of stop bits - must be FT_STOP_BITS_1 or FT_STOP_BITS_2.
   * @param uParity Parity - must be FT_PARITY_NONE, FT_PARITY_ODD, FT_PARITY_EVEN,
   * FT_PARITY_MARK or FT_PARITY SPACE.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code. }
  FT_SetDataCharacteristics: function(Handle: TFTHandle; WordLength, StopBits, Parity: Byte): TFTStatus; FTD2XX_API;

  {** @noop FT_SetTimeouts
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * This function sets the read and write timeouts for the device.
   * @param ftHandle Handle of the device.
   * @param dwReadTimeout Read timeout in milliseconds.
   * @param dwWriteTimeout Write timeout in milliseconds.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code. }
  FT_SetTimeouts: function(Handle: TFTHandle; ReadTimeout, WriteTimeout: LongWord): TFTStatus; FTD2XX_API;

  {** @noop FT_SetFlowControl
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * This function sets the flow control for the device.
   * @param ftHandle Handle of the device.
   * @param usFlowControl Must be one of FT_FLOW_NONE, FT_FLOW_RTS_CTS, FT_FLOW_DTR_DSR or
   * FT_FLOW_XON_XOFF.
   * @param uXonChar Character used to signal Xon. Only used if flow control is FT_FLOW_XON_XOFF.
   * @param uXoffChar Character used to signal Xoff. Only used if flow control is FT_FLOW_XON_XOFF.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code. }
  FT_SetFlowControl: function(Handle: TFTHandle; FlowControl: Word; XonChar, XoffChar: Byte): TFTStatus; FTD2XX_API;

  {** @noop FT_SetDtr
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * This function sets the Data Terminal Ready (DTR) control signal.
   * @param ftHandle Handle of the device.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code.
   * @remarks
   * This function asserts the Data Terminal Ready (DTR) line of the device. }
  FT_SetDtr: function(Handle: TFTHandle): TFTStatus; FTD2XX_API;

  {** @noop FT_ClrDtr
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * This function clears the Data Terminal Ready (DTR) control signal.
   * @param ftHandle Handle of the device.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code.
   * @remarks
   * This function de-asserts the Data Terminal Ready (DTR) line of the device. }
  FT_ClrDtr: function(Handle: TFTHandle): TFTStatus; FTD2XX_API;

  {** @noop FT_SetRts
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * This function sets the Request To Send (RTS) control signal.
   * @param ftHandle Handle of the device.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code.
   * @remarks
   * This function asserts the Request To Send (RTS) line of the device. }
  FT_SetRts: function(Handle: TFTHandle): TFTStatus; FTD2XX_API;

  {** @noop FT_ClrRts
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * This function clears the Request To Send (RTS) control signal.
   * @param ftHandle Handle of the device.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code.
   * @remarks
   * This function de-asserts the Request To Send (RTS) line of the device. }
  FT_ClrRts: function(Handle: TFTHandle): TFTStatus; FTD2XX_API;

  {** @noop FT_GetModemStatus
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * Gets the modem status and line status from the device.
   * @param ftHandle Handle of the device.
   * @param lpdwModemStatus Pointer to a variable of type DWORD which receives the modem
   * status and line status from the device.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code.
   * @remarks
   * The least significant byte of the lpdwModemStatus value holds the modem status. On Windows and
   * Windows CE, the line status is held in the second least significant byte of the lpdwModemStatus value.
   * @n The modem status is bit-mapped as follows: Clear To Send (CTS) = 0x10, Data Set Ready (DSR) = 0x20,
   * Ring Indicator (RI) = 0x40, Data Carrier Detect (DCD) = 0x80.
   * @n The line status is bit-mapped as follows: Overrun Error (OE) = 0x02, Parity Error (PE) = 0x04, Framing
   * Error (FE) = 0x08, Break Interrupt (BI) = 0x10. }
  FT_GetModemStatus: function(Handle: TFTHandle; ModemStatus: PLongWord): TFTStatus; FTD2XX_API;

  {** @noop FT_GetQueueStatus
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * Gets the number of bytes in the receive queue.
   * @param ftHandle Handle of the device.
   * @param lpdwAmountInRxQueue Pointer to a variable of type DWORD which receives the number of
   * bytes in the receive queue.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code. }
  FT_GetQueueStatus: function(Handle: TFTHandle; RxBytes: PDWord): TFTStatus; FTD2XX_API;

  {** @noop FT_GetDeviceInfo
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * Get device information for an open device.
   * @param ftHandle Handle of the device.
   * @param lpftDevice Pointer to unsigned long to store device type.
   * @param lpdwID Pointer to unsigned long to store device ID.
   * @param pcSerialNumber Pointer to buffer to store device serial number as a nullterminated string.
   * @param pcDescription Pointer to buffer to store device description as a null-terminated string.
   * @param pvDummy Reserved for future use - should be set to NULL.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code.
   * @remarks
   * This function is used to return the device type, device ID, device description and serial number.
   * The device ID is encoded in a DWORD - the most significant word contains the vendor ID, and the least
   * significant word contains the product ID. So the returned ID 0x04036001 corresponds to the device ID
   * VID_0403&PID_6001. }
  FT_GetDeviceInfo: function(Handle: TFTHandle; Device: PFTDevice; ID: PDWord; SerialNumber, Description: PChar; Dummy: pointer): TFTStatus; FTD2XX_API;

  {$IFNDEF WINDOWS}

  {** @noop FT_GetDeviceLocId
   * @note Extra function for non-Windows platforms to compensate for lack of .INF file to specify Vendor and Product IDs. }
  FT_GetDeviceLocId: function(Handle: TFTHandle; LocId: PDWord): TFTStatus; FTD2XX_API;

  {$ENDIF WINDOWS}

  {$IFDEF WINDOWS}

  {** @noop FT_GetDriverVersion
   * @par Supported Operating Systems
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * This function returns the D2XX driver version number.
   * @param ftHandle Handle of the device.
   * @param lpdwDriverVersion Pointer to the driver version number.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code.
   * @remarks
   * A version number consists of major, minor and build version numbers contained in a 4-byte field
   * (unsigned long). Byte0 (least significant) holds the build version, Byte1 holds the minor version, and
   * Byte2 holds the major version. Byte3 is currently set to zero.
   * @n For example, driver version "2.04.06" is represented as 0x00020406. Note that a device has to be
   * opened before this function can be called. }
  FT_GetDriverVersion: function(Handle: TFTHandle; Version: PLongWord): TFTStatus; FTD2XX_API;

  {** @noop FT_GetComPortNumber
   * @par Supported Operating Systems
   * Windows (2000 and later)
   * @par Summary
   * Retrieves the COM port associated with a device.
   * @param ftHandle Handle of the device.
   * @param lplComPortNumber Pointer to a variable of type LONG which receives the COM port number
   * associated with the device.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code.
   * @remarks
   * This function is only available when using the Windows CDM driver as both the D2XX and VCP drivers can
   * be installed at the same time.
   * @n If no COM port is associated with the device, lplComPortNumber will have a value of -1 }
  FT_GetComPortNumber: function(Handle: TFTHandle; ComPortNumber: PLongWord): TFTStatus; FTD2XX_API;

  {$ENDIF WINDOWS}

  {** @noop FT_GetLibraryVersion
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @n
   * This function returns D2XX DLL or library version number.
   * @param lpdwDLLVersion Pointer to the DLL or library version number.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code.
   * @remarks
   * A version number consists of major, minor and build version numbers contained in a 4-byte field
   * (unsigned long). Byte0 (least significant) holds the build version, Byte1 holds the minor version, and
   * Byte2 holds the major version. Byte3 is currently set to zero.
   * @n For example, D2XX DLL version "3.01.15" is represented as 0x00030115. Note that this function does
   * not take a handle, and so it can be called without opening a device. }
  FT_GetLibraryVersion: function(Version: PLongWord): TFTStatus; FTD2XX_API;

  {** @noop FT_GetStatus
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * Gets the device status including number of characters in the receive queue, number of characters in the
   * transmit queue, and the current event status.
   * @param ftHandle Handle of the device.
   * @param lpdwAmountInRxQueue Pointer to a variable of type DWORD which receives the number of characters in
   * the receive queue.
   * @param lpdwAmountInTxQueue Pointer to a variable of type DWORD which receives the number of characters in
   * the transmit queue.
   * @param lpdwEventStatus Pointer to a variable of type DWORD which receives the current state of
   * the event status.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code.
   * @remarks
   * For an example of how to use this function, see the sample code in FT_SetEventNotification. }
  FT_GetStatus: function(Handle: TFTHandle; InRxQueue, InTxQueue, EventStatus: PDWord): TFTStatus; FTD2XX_API;

  {** @noop FT_SetEventNotification
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * Sets conditions for event notification.
   * @param ftHandle Handle of the device.
   * @param dwEventMask Conditions that cause the event to be set.
   * @param pvArg Interpreted as the handle of an event.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code.
   * @remarks
   * An application can use this function to setup conditions which allow a thread to block until one of the
   * conditions is met. Typically, an application will create an event, call this function, then block on the
   * event. When the conditions are met, the event is set, and the application thread unblocked.
   * dwEventMask is a bit-map that describes the events the application is interested in. pvArg is interpreted
   * as the handle of an event which has been created by the application. If one of the event conditions is
   * met, the event is set.
   * @n If FT_EVENT_RXCHAR is set in dwEventMask, the event will be set when a character has been received
   * by the device.
   * @n If FT_EVENT_MODEM_STATUS is set in dwEventMask, the event will be set when a change in the modem
   * signals has been detected by the device.
   * @n If FT_EVENT_LINE_STATUS is set in dwEventMask, the event will be set when a change in the line status
   * has been detected by the device. }
  FT_SetEventNotification: function(Handle: TFTHandle; Mask: DWord; Param: pointer): TFTStatus; FTD2XX_API;

  {** @noop FT_SetChars
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * This function sets the special characters for the device.
   * @param ftHandle Handle of the device.
   * @param uEventChar Event character.
   * @param uEventCharEnabled 0 if event character disabled, non-zero otherwise.
   * @param uErrorChar Error character.
   * @param uErrorCharEnabled 0 if error character disabled, non-zero otherwise.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code.
   * @remarks
   * This function allows for inserting specified characters in the data stream to represent events firing or
   * errors occurring. }
  FT_SetChars: function(Handle: TFTHandle; EventChar, EventCharEnabled, ErrorChar, ErrorCharEnabled: Byte): TFTStatus; FTD2XX_API;

  {** @noop FT_SetBreakOn
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * Sets the BREAK condition for the device.
   * @param ftHandle Handle of the device.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code }
  FT_SetBreakOn: function(Handle: TFTHandle): TFTStatus; FTD2XX_API;

{** @noop FT_SetBreakOff
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * Resets the BREAK condition for the device.
   * @param ftHandle Handle of the device.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code. }
  FT_SetBreakOff: function(Handle: TFTHandle): TFTStatus; FTD2XX_API;

  {** @noop FT_Purge
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * This function purges receive and transmit buffers in the device.
   * @param ftHandle Handle of the device.
   * @param ulMask Combination of FT_PURGE_RX and FT_PURGE_TX.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code. }
  FT_Purge: function(Handle: TFTHandle; Mask: LongWord): TFTStatus; FTD2XX_API;

  {** @noop FT_ResetDevice
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * This function sends a reset command to the device.
   * @param ftHandle Handle of the device.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code. }
  FT_ResetDevice: function(Handle: TFTHandle): TFTStatus; FTD2XX_API;

  {$IFDEF WINDOWS}

  {** @noop FT_ResetPort
   * @par Supported Operating Systems
   * Windows (2000 and later)
   * @par Summary
   * Send a reset command to the port.
   * @param ftHandle Handle of the device.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code.
   * @remarks
   * This function is used to attempt to recover the port after a failure. It is not equivalent
   * to an unplug-replug event. For the equivalent of an unplug-replug event, use FT_CyclePort.
   * @see FT_CyclePort }
  FT_ResetPort: function(Handle: TFTHandle): TFTStatus; FTD2XX_API;

  {** @noop FT_CyclePort
   * @par Supported Operating Systems
   * Windows (2000 and later)
   * @par Summary
   * Send a cycle command to the USB port.
   * @param ftHandle Handle of the device.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code.
   * @remarks
   * The effect of this function is the same as disconnecting then reconnecting the device from
   * USB. Possible use of this function is situations where a fatal error has occurred and it is
   * difficult, or not possible, to recover without unplugging and replugging the USB cable.
   * This function can also be used after reprogramming the EEPROM to force the FTDI device to
   * read the new EEPROM contents which would otherwise require a physical disconnect-reconnect.
   * @n As the current session is not restored when the driver is reloaded, the application must
   * be able to recover after calling this function. It is ithe responisbility of the application
   * to close the handle after successfully calling FT_CyclePort.
   * @n For FT4232H, FT2232H and FT2232 devices, FT_CyclePort will only work under Windows XP and later. }
  FT_CyclePort: function(Handle: TFTHandle): TFTStatus; FTD2XX_API;

  {** @noop FT_Rescan
   * @par Supported Operating Systems
   * Windows (2000 and later)
   * @par Summary
   * This function can be of use when trying to recover devices programatically.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code.
   * @remarks
   * Calling FT_Rescan is equivalent to clicking the "Scan for hardware changes" button in the Device
   * Manager. Only USB hardware is checked for new devices. All USB devices are scanned, not just FTDI
   * devices. }
  FT_Rescan: function(): TFTStatus; FTD2XX_API;

  {** @noop FT_Reload
   * @par Supported Operating Systems
   * Windows (2000 and later)
   * @par Summary
   * This function forces a reload of the driver for devices with a specific VID and PID combination.
   * @param wVID Vendor ID of the devices to reload the driver for.
   * @param wPID Product ID of the devices to reload the driver for.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code.
   * @remarks
   * Calling FT_Reload forces the operating system to unload and reload the driver for the specified device
   * IDs. If the VID and PID parameters are null, the drivers for USB root hubs will be reloaded, causing all
   * USB devices connected to reload their drivers. Please note that this function will not work correctly on
   * 64-bit Windows when called from a 32-bit application. }
  FT_Reload: function(Vid, Pid: Word): TFTStatus; FTD2XX_API;

  {** @noop FT_SetResetPipeRetryCount
   * @par Supported Operating Systems
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * Set the ResetPipeRetryCount value.
   * @param ftHandle Handle of the device.
   * @param dwCount Unsigned long containing required ResetPipeRetryCount.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code.
   * @remarks
   * This function is used to set the ResetPipeRetryCount. ResetPipeRetryCount controls the maximum
   * number of times that the driver tries to reset a pipe on which an error has occurred.
   * ResetPipeRequestRetryCount defaults to 50. It may be necessary to increase this value in noisy
   * environments where a lot of USB errors occur. }
  FT_SetResetPipeRetryCount: function(Handle: TFTHandle; Count: DWord): TFTStatus; FTD2XX_API;

  {$ENDIF WINDOWS}

  {** @noop FT_StopInTask
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * Stops the driver's IN task.
   * @param ftHandle Handle of the device.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code.
   * @remarks
   * This function is used to put the driver's IN task (read) into a wait state. It can be used in situations
   * where data is being received continuously, so that the device can be purged without more data being
   * received. It is used together with FT_RestartInTask which sets the IN task running again.
   * @see FT_RestartInTask }
  FT_StopInTask: function(Handle: TFTHandle): TFTStatus; FTD2XX_API;

  {** @noop FT_RestartInTask
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * Restart the driver's IN task.
   * @param ftHandle Handle of the device.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code.
   * @remarks
   * This function is used to restart the driver's IN task (read) after it has been stopped by a call to
   * FT_StopInTask.
   * @see FT_StopInTask }
  FT_RestartInTask: function(Handle: TFTHandle): TFTStatus; FTD2XX_API;

  {** @noop FT_SetDeadmanTimeout
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * This function allows the maximum time in milliseconds that a USB request can remain outstanding to
   * be set.
   * @param ftHandle Handle of the device.
   * @param ulDeadmanTimeout Deadman timeout value in milliseconds. Default value is 5000.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code.
   * @remarks
   * The deadman timeout is referred to in application note AN232B-10 Advanced Driver Options from the
   * FTDI web site as the USB timeout. It is unlikely that this function will be required by most users. }
  FT_SetDeadmanTimeout: function(Handle: TFTHandle; DeadmanTimeout: LongWord): TFTStatus; FTD2XX_API;

  {** @noop FT_IoCtl
   * Undocumented function. }
  FT_IoCtl: function(Handle: TFTHandle; IoControlCode: DWord; InBuf: pointer; InBufSize: DWord; OutBuf: pointer; OutBufSize: DWord; BytesReturned: PDWord; Overlapped: POverlapped): DWord; FTD2XX_API;

  {** @noop FT_SetWaitMask
   * Undocumented function. }
  FT_SetWaitMask: function(Handle: TFTHandle; Mask: DWord): TFTStatus; FTD2XX_API;

  {** @noop FT_WaitOnMask
   * Undocumented function. }
  FT_WaitOnMask: function(Handle: TFTHandle; Mask: PDWord): TFTStatus; FTD2XX_API;

  {** @noop FT_GetEventStatus
   * Undocumented function. }
  FT_GetEventStatus: function(Handle: TFTHandle; EventDWord: PDWord): TFTStatus; FTD2XX_API;

  {........................................................................}

  {* @name EEPROM Programming Interface Functions
   * FTDI device EEPROMs can be both read and programmed using the functions listed in this section.
   * @n Please note the following information:
   * @li The Maximum length of the Manufacturer, ManufacturerId, Description and SerialNumber strings
   * is 48 words (1 word = 2 bytes).
   * @li The first two characters of the serial number are the manufacturer ID.
   * @li The Manufacturer string length plus the Description string length is less than or equal to 40
   * characters with the following functions: FT_EE_Read, FT_EE_Program, FT_EE_ProgramEx,
   * FT_EEPROM_Read, FT_EEPROM_Program.
   * @li The serial number should be maximum 15 characters long on single port devices (eg FT232R, FT-X)
   * and 14 characters on multi port devices (eg FT2232H, FT4232H). If it is longer then it may be
   * truncated and will not have a null terminator.

   * For instance a serial number which is 15 characters long on a multi-port device will have an
   * effective serial number which is 16 characters long since the serial number is appended with the
   * channel identifier (A,B,etc). The buffer used to return the string from the API is only 16
   * characters in size so the NULL termination will be lost.
   * @n If the serial number or description are too long in the EEPROM or configuration of a device then the
   * strings returned by FT_GetDeviceInfo and FT_ListDevices may not be NULL terminated }

   {........................................................................}

  {** @noop FT_ReadEE
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * Read a value from an EEPROM location.
   * @param ftHandle Handle of the device.
   * @param dwWordOffset EEPROM location to read from.
   * @param lpwValue Pointer to the WORD value read from the EEPROM.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code.
   * @remarks
   * EEPROMs for FTDI devices are organised by WORD, so each value returned is 16-bits wide. }
  FT_ReadEE: function(Handle: TFTHandle; WordOffset: DWord; Value: PWord): TFTStatus; FTD2XX_API;

  {** @noop FT_WriteEE
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * Write a value to an EEPROM location.
   * @param ftHandle Handle of the device.
   * @param dwWordOffset EEPROM location to read from.
   * @param wValue The WORD value write to the EEPROM.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code.
   * @remarks
   * EEPROMs for FTDI devices are organised by WORD, so each value written to the EEPROM is
   * 16-bits wide. }
  FT_WriteEE: function(Handle: TFTHandle; WordOffset: DWord; Value: Word): TFTStatus; FTD2XX_API;

  {** @noop FT_EraseEE
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * Erases the device EEPROM.
   * @param ftHandle Handle of the device.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code.
   * @remarks
   * This function will erase the entire contents of an EEPROM, including the user area.
   * Note that the FT232R and FT245R devices have an internal EEPROM that cannot be erased. }
  FT_EraseEE: function(Handle: TFTHandle): TFTStatus; FTD2XX_API;

type

  {**
   * Structure to hold program data for FT_EE_Program, FT_EE_ProgramEx, FT_EE_Read
   * and FT_EE_ReadEx functions.
   * @see FT_EE_Read
   * @see FT_EE_ReadEx
   * @see FT_EE_Program
   * @see FT_EE_ProgramEx }
  TFT_Program_Data = packed record
    Signature1        : DWord;          // Header - must be 0x00000000
    Signature2        : DWord;          // Header - must be 0xffffffff

    Version           : DWord;          // Header - FT_PROGRAM_DATA version
                                        //          0 = original
                                        //          1 = FT2232 extensions
                                        //          2 = FT232R extensions
                                        //          3 = FT2232H extensions
                                        //          4 = FT4232H extensions
                                        //          5 = FT232H extensions

    VendorId          : Word;           // 0x0403
    ProductId         : Word;           // 0x6001
    Manufacturer      : PChar;          // "FTDI"
    ManufacturerId    : PChar;          // "FT"
    Description       : PChar;          // "USB HS Serial Converter"
    SerialNumber      : PChar;          // "FT000001" if fixed, or NULL
    MaxPower          : Word;           // 0 < MaxPower <= 500
    PnP               : Word;           // 0 = disabled, 1 = enabled
    SelfPowered       : Word;           // 0 = bus powered, 1 = self powered
    RemoteWakeup      : Word;           // 0 = not capable, 1 = capable

    // Rev4 (FT232B) extensions

    Rev4              : Byte;           // non-zero if Rev4 chip, zero otherwise
    IsoIn             : Byte;           // non-zero if in endpoint is isochronous
    IsoOut            : Byte;           // non-zero if out endpoint is isochronous
    PullDownEnable    : Byte;           // non-zero if pull down enabled
    SerNumEnable      : Byte;           // non-zero if serial number to be used
    USBVersionEnable  : Byte;           // non-zero if chip uses USBVersion
    USBVersion        : Word;           // BCD (0x0200 => USB2)

    // Rev 5 (FT2232) extensions

    Rev5              : Byte;           // non-zero if Rev5 chip, zero otherwise
    IsoInA            : Byte;           // non-zero if in endpoint is isochronous
    IsoInB            : Byte;           // non-zero if in endpoint is isochronous
    IsoOutA           : Byte;           // non-zero if out endpoint is isochronous
    IsoOutB           : Byte;           // non-zero if out endpoint is isochronous
    PullDownEnable5   : Byte;           // non-zero if pull down enabled
    SerNumEnable5     : Byte;           // non-zero if serial number to be used
    USBVersionEnable5 : Byte;           // non-zero if chip uses USBVersion
    USBVersion5       : Word;           // BCD (0x0200 => USB2)
    AIsHighCurrent    : Byte;           // non-zero if interface is high current
    BIsHighCurrent    : Byte;           // non-zero if interface is high current
    IFAIsFifo         : Byte;           // non-zero if interface is 245 FIFO
    IFAIsFifoTar      : Byte;           // non-zero if interface is 245 FIFO CPU target
    IFAIsFastSer      : Byte;           // non-zero if interface is Fast serial
    AIsVCP            : Byte;           // non-zero if interface is to use VCP drivers
    IFBIsFifo         : Byte;           // non-zero if interface is 245 FIFO
    IFBIsFifoTar      : Byte;           // non-zero if interface is 245 FIFO CPU target
    IFBIsFastSer      : Byte;           // non-zero if interface is Fast serial
    BIsVCP            : Byte;           // non-zero if interface is to use VCP drivers

    // Rev 6 (FT232R) extensions

    UseExtOsc         : Byte;           // Use External Oscillator
    HighDriveIOs      : Byte;           // High Drive I/Os
    EndpointSize      : Byte;           // Endpoint size
    PullDownEnableR   : Byte;           // non-zero if pull down enabled
    SerNumEnableR     : Byte;           // non-zero if serial number to be used
    InvertTXD         : Byte;           // non-zero if invert TXD
    InvertRXD         : Byte;           // non-zero if invert RXD
    InvertRTS         : Byte;           // non-zero if invert RTS
    InvertCTS         : Byte;           // non-zero if invert CTS
    InvertDTR         : Byte;           // non-zero if invert DTR
    InvertDSR         : Byte;           // non-zero if invert DSR
    InvertDCD         : Byte;           // non-zero if invert DCD
    InvertRI          : Byte;           // non-zero if invert RI
    Cbus0             : Byte;           // Cbus Mux control
    Cbus1             : Byte;           // Cbus Mux control
    Cbus2             : Byte;           // Cbus Mux control
    Cbus3             : Byte;           // Cbus Mux control
    Cbus4             : Byte;           // Cbus Mux control
    RIsD2XX           : Byte;           // non-zero if using D2XX driver

    // Rev 8 (FT4232H) Extensions

    PullDownEnable8   : Byte;           // non-zero if pull down enabled
    SerNumEnable8     : Byte;           // non-zero if serial number to be used
    ASlowSlew         : Byte;           // non-zero if A pins have slow slew
    ASchmittInput     : Byte;           // non-zero if A pins are Schmitt input
    ADriveCurrent     : Byte;           // valid values are 4mA, 8mA, 12mA, 16mA
    BSlowSlew         : Byte;           // non-zero if B pins have slow slew
    BSchmittInput     : Byte;           // non-zero if B pins are Schmitt input
    BDriveCurrent     : Byte;           // valid values are 4mA, 8mA, 12mA, 16mA
    CSlowSlew         : Byte;           // non-zero if C pins have slow slew
    CSchmittInput     : Byte;           // non-zero if C pins are Schmitt input
    CDriveCurrent     : Byte;           // valid values are 4mA, 8mA, 12mA, 16mA
    DSlowSlew         : Byte;           // non-zero if D pins have slow slew
    DSchmittInput     : Byte;           // non-zero if D pins are Schmitt input
    DDriveCurrent     : Byte;           // valid values are 4mA, 8mA, 12mA, 16mA
    ARIIsTXDEN        : Byte;           // non-zero if port A uses RI as RS485 TXDEN
    BRIIsTXDEN        : Byte;           // non-zero if port B uses RI as RS485 TXDEN
    CRIIsTXDEN        : Byte;           // non-zero if port C uses RI as RS485 TXDEN
    DRIIsTXDEN        : Byte;           // non-zero if port D uses RI as RS485 TXDEN
    AIsVCP8           : Byte;           // non-zero if interface is to use VCP drivers
    BIsVCP8           : Byte;           // non-zero if interface is to use VCP drivers
    CIsVCP8           : Byte;           // non-zero if interface is to use VCP drivers
    DIsVCP8           : Byte;           // non-zero if interface is to use VCP drivers

    // Rev 9 (FT232H) Extensions

    PullDownEnableH   : Byte;           // non-zero if pull down enabled
    SerNumEnableH     : Byte;           // non-zero if serial number to be used
    ACSlowSlewH       : Byte;           // non-zero if AC pins have slow slew
    ACSchmittInputH   : Byte;           // non-zero if AC pins are Schmitt input
    ACDriveCurrentH   : Byte;           // valid values are 4mA, 8mA, 12mA, 16mA
    ADSlowSlewH       : Byte;           // non-zero if AD pins have slow slew
    ADSchmittInputH   : Byte;           // non-zero if AD pins are Schmitt input
    ADDriveCurrentH   : Byte;           // valid values are 4mA, 8mA, 12mA, 16mA
    Cbus0H            : Byte;           // Cbus Mux control
    Cbus1H            : Byte;           // Cbus Mux control
    Cbus2H            : Byte;           // Cbus Mux control
    Cbus3H            : Byte;           // Cbus Mux control
    Cbus4H            : Byte;           // Cbus Mux control
    Cbus5H            : Byte;           // Cbus Mux control
    Cbus6H            : Byte;           // Cbus Mux control
    Cbus7H            : Byte;           // Cbus Mux control
    Cbus8H            : Byte;           // Cbus Mux control
    Cbus9H            : Byte;           // Cbus Mux control
    IsFifoH           : Byte;           // non-zero if interface is 245 FIFO
    IsFifoTarH        : Byte;           // non-zero if interface is 245 FIFO CPU target
    IsFastSerH        : Byte;           // non-zero if interface is Fast serial
    IsFT1248H         : Byte;           // non-zero if interface is FT1248
    FT1248CpolH       : Byte;           // FT1248 clock polarity - clock idle high (1) or clock idle low (0)
    FT1248LsbH        : Byte;           // FT1248 data is LSB (1) or MSB (0)
    FT1248FlowControlH: Byte;           // FT1248 flow control enable
    IsVCPH            : Byte;           // non-zero if interface is to use VCP drivers
    PowerSaveEnableH  : Byte;           // non-zero if using ACBUS7 to save power for self-powered designs
  end;
  PFT_Program_Data = ^TFT_Program_Data;

var

  {** @noop FT_EE_Read
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * Read the contents of the EEPROM.
   * @param ftHandle Handle of the device.
   * @param pData Pointer to structure of type FT_PROGRAM_DATA.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code.
   * @remarks
   * This function interprets the parameter pData as a pointer to a structure of type FT_PROGRAM_DATA
   * that contains storage for the data to be read from the EEPROM.
   * @n The function does not perform any checks on buffer sizes, so the buffers passed in the
   * FT_PROGRAM_DATA structure must be big enough to accommodate their respective strings (including
   * null terminators). The sizes shown in the following example are more than adequate and can be rounded
   * down if necessary. The restriction is that the Manufacturer string length plus the Description string
   * length is less than or equal to 40 characters.
   * @note Note that the DLL must be informed which version of the FT_PROGRAM_DATA structure is being used.
   * This is done through the Signature1, Signature2 and Version elements of the structure. Signature1
   * should always be 0x00000000, Signature2 should always be 0xFFFFFFFF and Version can be set to use
   * whichever version is required. For compatibility with all current devices Version should be set to the
   * latest version of the FT_PROGRAM_DATA structure which is defined in FTD2XX.h.
   * @see FT_PROGRAM_DATA }
  FT_EE_Read: function(Handle: TFTHandle; Data: PFT_Program_Data): TFTStatus; FTD2XX_API;

  {** @noop FT_EE_ReadEx
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * Read the contents of the EEPROM and pass strings separately.
   * @param ftHandle Handle of the device.
   * @param pData Pointer to structure of type FT_PROGRAM_DATA.
   * @param *Manufacturer Pointer to a null-terminated string containing the manufacturer name.
   * @param *ManufacturerId Pointer to a null-terminated string containing the manufacturer ID.
   * @param *Description Pointer to a null-terminated string containing the device description.
   * @param *SerialNumber Pointer to a null-terminated string containing the device serial number.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code.
   * @remarks
   * This variation of the standard FT_EE_Read function was included to provide support for languages such
   * as LabVIEW where problems can occur when string pointers are contained in a structure.
   * @n This function interprets the parameter pData as a pointer to a structure of type FT_PROGRAM_DATA that
   * contains storage for the data to be read from the EEPROM.
   * @n The function does not perform any checks on buffer sizes, so the buffers passed in the
   * FT_PROGRAM_DATA structure must be big enough to accommodate their respective strings (including
   * null terminators).
   * @note Note that the DLL must be informed which version of the FT_PROGRAM_DATA structure is being used.
   * This is done through the Signature1, Signature2 and Version elements of the structure. Signature1
   * should always be 0x00000000, Signature2 should always be 0xFFFFFFFF and Version can be set to use
   * whichever version is required. For compatibility with all current devices Version should be set to the
   * latest version of the FT_PROGRAM_DATA structure which is defined in FTD2XX.h.
   * @n The string parameters in the FT_PROGRAM_DATA structure should be passed as DWORDs to avoid
   * overlapping of parameters. All string pointers are passed out separately from the FT_PROGRAM_DATA
   * structure.
   * @see FT_PROGRAM_DATA }
  FT_EE_ReadEx: function(Handle: TFTHandle; Data: PFT_Program_Data; Manufacturer, ManufacturerId, Description, SerialNumber: PChar): TFTStatus; FTD2XX_API;

  {** @noop FT_EE_Program
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * Program the EEPROM.
   * @param ftHandle Handle of the device.
   * @param pData Pointer to structure of type FT_PROGRAM_DATA.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code.
   * @remarks
   * This function interprets the parameter pData as a pointer to a structure of type FT_PROGRAM_DATA
   * that contains the data to write to the EEPROM. The data is written to EEPROM, then read back and
   * verified.
   * @n If the SerialNumber field in FT_PROGRAM_DATA is NULL, or SerialNumber points to a NULL string,
   * a serial number based on the ManufacturerId and the current date and time will be generated. The
   * Manufacturer string length plus the Description string length must be less than or equal to 40
   * characters.
   * @note Note that the DLL must be informed which version of the FT_PROGRAM_DATA structure is being
   * used. This is done through the Signature1, Signature2 and Version elements of the structure.
   * Signature1 should always be 0x00000000, Signature2 should always be 0xFFFFFFFF and Version can be
   * set to use whichever version is required. For compatibility with all current devices Version
   * should be set to the latest version of the FT_PROGRAM_DATA structure which is defined in FTD2XX.h.
   * If pData is NULL, the structure version will default to 0 (original BM series) and the device will
   * be programmed with the default data.
   * @see FT_PROGRAM_DATA }
  FT_EE_Program: function(Handle: TFTHandle; Data: PFT_Program_Data): TFTStatus; FTD2XX_API;

  {** @noop FT_EE_ProgramEx
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * Program the EEPROM and pass strings separately.
   * @param ftHandle Handle of the device.
   * @param pData Pointer to structure of type FT_PROGRAM_DATA.
   * @param *Manufacturer Pointer to a null-terminated string containing the manufacturer name.
   * @param *ManufacturerId Pointer to a null-terminated string containing the manufacturer ID.
   * @param *Description Pointer to a null-terminated string containing the device description.
   * @param *SerialNumber Pointer to a null-terminated string containing the device serial number.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code.
   * @remarks
   * This variation of the FT_EE_Program function was included to provide support for languages such
   * as LabVIEW where problems can occur when string pointers are contained in a structure.
   * @n This function interprets the parameter pData as a pointer to a structure of type FT_PROGRAM_DATA
   * that contains the data to write to the EEPROM. The data is written to EEPROM, then read back and
   * verified. The string pointer parameters in the FT_PROGRAM_DATA structure should be allocated as
   * DWORDs to avoid overlapping of parameters. The string parameters are then passed in separately.
   * @n If the SerialNumber field is NULL, or SerialNumber points to a NULL string, a serial number based
   * on the ManufacturerId and the current date and time will be generated. The Manufacturer string
   * length plus the Description string length must be less than or equal to 40 characters.
   * @note Note that the DLL must be informed which version of the FT_PROGRAM_DATA structure is being used.
   * This is done through the Signature1, Signature2 and Version elements of the structure. Signature1
   * should always be 0x00000000, Signature2 should always be 0xFFFFFFFF and Version can be set to use
   * whichever version is required. For compatibility with all current devices Version should be set to the
   * latest version of the FT_PROGRAM_DATA structure which is defined in FTD2XX.h.
   * If pData is NULL, the structure version will default to 0 (original BM series) and the device will be
   * programmed with the default data.
   * @see FT_PROGRAM_DATA }
  FT_EE_ProgramEx: function(Handle: TFTHandle; Data: PFT_Program_Data; Manufacturer, ManufacturerId, Description, SerialNumber: PChar): TFTStatus; FTD2XX_API;

  {** @noop FT_EE_UASize
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * Get the available size of the EEPROM user area.
   * @param ftHandle Handle of the device.
   * @param lpdwSize Pointer to a DWORD that receives the available size, in bytes, of the EEPROM
   * user area.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code.
   * @remarks
   * The user area of an FTDI device EEPROM is the total area of the EEPROM that is unused by device
   * configuration information and descriptors. This area is available to the user to store information
   * specific to their application. The size of the user area depends on the length of the Manufacturer,
   * ManufacturerId, Description and SerialNumber strings programmed into the EEPROM. }
  FT_EE_UASize: function(Handle: TFTHandle; Size: PDWord): TFTStatus; FTD2XX_API;

  {** @noop FT_EE_UARead
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * Read the contents of the EEPROM user area.
   * @param ftHandle Handle of the device.
   * @param pucData Pointer to a buffer that contains storage for data to be read.
   * @param dwDataLen Size, in bytes, of buffer that contains storage for the data to be read.
   * @param lpdwBytesRead Pointer to a DWORD that receives the number of bytes read.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code.
   * @remarks
   * This function interprets the parameter pucData as a pointer to an array of bytes of size
   * dwDataLen that contains storage for the data to be read from the EEPROM user area. The actual
   * number of bytes read is stored in the DWORD referenced by lpdwBytesRead.
   * @n If dwDataLen is less than the size of the EEPROM user area, then dwDataLen bytes are read
   * into the buffer. Otherwise, the whole of the EEPROM user area is read into the buffer. The
   * available user area size can be determined by calling FT_EE_UASize.
   * @n An application should check the function return value and lpdwBytesRead when FT_EE_UARead
   * returns. }
  FT_EE_UARead: function(Handle: TFTHandle; Data: PByte; DataLen: DWord; BytesRead: PDWord): TFTStatus; FTD2XX_API;

  {** @noop FT_EE_UAWrite
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * Write data into the EEPROM user area.
   * @param ftHandle Handle of the device.
   * @param pucData Pointer to a buffer that contains the data to be written.
   * @param dwDataLen Size, in bytes, of buffer that contains storage for the data to be read.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code.
   * @remarks
   * This function interprets the parameter pucData as a pointer to an array of bytes of size
   * dwDataLen that contains the data to be written to the EEPROM user area. It is a programming
   * error for dwDataLen to be greater than the size of the EEPROM user area. The available user
   * area size can be determined by calling FT_EE_UASize. }
  FT_EE_UAWrite: function(Handle: TFTHandle; Data: PByte; DataLen: DWord): TFTStatus; FTD2XX_API;

type

  {........................................................................}

  {**  @noop FT_EEPROM_HEADER
   * @par Summary
   * Structure to hold data for FT_EEPROM_Program and FT_EEPROM_Read functions.
   * The structure for the command includes one FT_EEPROM_HEADER with a device-specific
   * structure appended.
   * @see FT_EEPROM_Read
   * @see FT_EEPROM_Program }
  TFT_eeprom_header = packed record
    deviceType        : TFTDevice;   // FTxxxx device type to be programmed

    // Device descriptor options

    VendorId          : Word;       // 0x0403
    ProductId         : Word;       // 0x6001
    SerNumEnable      : Byte;       // non-zero if serial number to be used

    // Config descriptor options

    MaxPower          : Word;       // 0 < MaxPower <= 500
    SelfPowered       : Byte;       // 0 = bus powered, 1 = self powered
    RemoteWakeup      : Byte;       // 0 = not capable, 1 = capable

    // Hardware options
    PullDownEnable    : Byte;       // non-zero if pull down in suspend enabled
  end;
  PFT_eeprom_header = ^TFT_eeprom_header;

  {........................................................................}

    {**  @noop FT_EEPROM_232B
     * @par Summary
     * Structure to hold data for the FT232B data in the FT_EEPROM_Program and FT_EEPROM_Read functions.
     * This is appended to an FT_EEPROM_HEADER structure.
     * @see FT_EEPROM_HEADER }
    TFT_eeprom_232b   = packed record
      // Common header
      common            : TFT_eeprom_header;  // common elements for all device EEPROMs
    end;
  PFT_eeprom_232b   = ^TFT_eeprom_232b;

  {........................................................................}

  {**  @noop FT_EEPROM_2232
   * @par Summary
   * Structure to hold data for the FT2232C, FT2232D and FT2232L data in the FT_EEPROM_Program
   * and FT_EEPROM_Read functions.
   * This is appended to an FT_EEPROM_HEADER structure.
   * @see FT_EEPROM_HEADER }
  TFT_eeprom_2232 = packed record
    // Common header
    common            : TFT_eeprom_header;  // common elements for all device EEPROMs

    // Drive options

    AIsHighCurrent    : Byte;       // non-zero if interface is high current
    BIsHighCurrent    : Byte;       // non-zero if interface is high current

    // Hardware options

    AIsFifo           : Byte;       // non-zero if interface is 245 FIFO
    AIsFifoTar        : Byte;       // non-zero if interface is 245 FIFO CPU target
    AIsFastSer        : Byte;       // non-zero if interface is Fast serial
    BIsFifo           : Byte;       // non-zero if interface is 245 FIFO
    BIsFifoTar        : Byte;       // non-zero if interface is 245 FIFO CPU target
    BIsFastSer        : Byte;       // non-zero if interface is Fast serial

    // Driver option

    ADriverType       : Byte;       // non-zero if interface is to use VCP drivers
    BDriverType       : Byte;       // non-zero if interface is to use VCP drivers
  end;
  PFT_eeprom_2232 = ^TFT_eeprom_2232;

  {........................................................................}

  {**  @noop FT_EEPROM_232R
   * @par Summary
   * Structure to hold data for the FT232R data in the FT_EEPROM_Program and FT_EEPROM_Read functions.
   * This is appended to an FT_EEPROM_HEADER structure.
   * @see FT_EEPROM_HEADER }
  TFT_eeprom_232r = packed record
    // Common header
    common            : TFT_eeprom_header;  // common elements for all device EEPROMs

    // Drive options
    IsHighCurrent     : Byte;       // non-zero if interface is high current

    // Hardware options

    UseExtOsc         : Byte;       // Use External Oscillator
    InvertTXD         : Byte;       // non-zero if invert TXD
    InvertRXD         : Byte;       // non-zero if invert RXD
    InvertRTS         : Byte;       // non-zero if invert RTS
    InvertCTS         : Byte;       // non-zero if invert CTS
    InvertDTR         : Byte;       // non-zero if invert DTR
    InvertDSR         : Byte;       // non-zero if invert DSR
    InvertDCD         : Byte;       // non-zero if invert DCD
    InvertRI          : Byte;       // non-zero if invert RI
    Cbus0             : Byte;       // Cbus Mux control
    Cbus1             : Byte;       // Cbus Mux control
    Cbus2             : Byte;       // Cbus Mux control
    Cbus3             : Byte;       // Cbus Mux control
    Cbus4             : Byte;       // Cbus Mux control
    // Driver option
    DriverType        : Byte;       // non-zero if using D2XX driver
  end;
  PFT_eeprom_232r = ^TFT_eeprom_232r;

  {........................................................................}

  {**  @noop FT_EEPROM_2232H
   * @par Summary
   * Structure to hold data for the FT2232H data in the FT_EEPROM_Program and FT_EEPROM_Read functions.
   * This is appended to an FT_EEPROM_HEADER structure.
   * @see FT_EEPROM_HEADER }
  TFT_eeprom_2232h = packed record
    // Common header
    common            : TFT_eeprom_header;  // common elements for all device EEPROMs

    // Drive options

    ALSlowSlew        : Byte;       // non-zero if AL pins have slow slew
    ALSchmittInput    : Byte;       // non-zero if AL pins are Schmitt input
    ALDriveCurrent    : Byte;       // valid values are 4mA, 8mA, 12mA, 16mA
    AHSlowSlew        : Byte;       // non-zero if AH pins have slow slew
    AHSchmittInput    : Byte;       // non-zero if AH pins are Schmitt input
    AHDriveCurrent    : Byte;       // valid values are 4mA, 8mA, 12mA, 16mA
    BLSlowSlew        : Byte;       // non-zero if BL pins have slow slew
    BLSchmittInput    : Byte;       // non-zero if BL pins are Schmitt input
    BLDriveCurrent    : Byte;       // valid values are 4mA, 8mA, 12mA, 16mA
    BHSlowSlew        : Byte;       // non-zero if BH pins have slow slew
    BHSchmittInput    : Byte;       // non-zero if BH pins are Schmitt input
    BHDriveCurrent    : Byte;       // valid values are 4mA, 8mA, 12mA, 16mA

    // Hardware options

    AIsFifo           : Byte;       // non-zero if interface is 245 FIFO
    AIsFifoTar        : Byte;       // non-zero if interface is 245 FIFO CPU target
    AIsFastSer        : Byte;       // non-zero if interface is Fast serial
    BIsFifo           : Byte;       // non-zero if interface is 245 FIFO
    BIsFifoTar        : Byte;       // non-zero if interface is 245 FIFO CPU target
    BIsFastSer        : Byte;       // non-zero if interface is Fast serial
    PowerSaveEnable   : Byte;       // non-zero if using BCBUS7 to save power for self-powered designs

    // Driver option

    ADriverType       : Byte;       // non-zero if interface is to use VCP drivers
    BDriverType       : Byte;       // non-zero if interface is to use VCP drivers
  end;
  PFT_eeprom_2232h = ^TFT_eeprom_2232h;

  {........................................................................}

  {**  @noop FT_EEPROM_4232H
   * @par Summary
   * Structure to hold data for the FT4232H data in the FT_EEPROM_Program and FT_EEPROM_Read functions.
   * This is appended to an FT_EEPROM_HEADER structure.
   * @see FT_EEPROM_HEADER }
  TFT_eeprom_4232h = packed record
    // Common header
    common            : TFT_eeprom_header;  // common elements for all device EEPROMs

    // Drive options

    ASlowSlew         : Byte;       // non-zero if A pins have slow slew
    ASchmittInput     : Byte;       // non-zero if A pins are Schmitt input
    ADriveCurrent     : Byte;       // valid values are 4mA, 8mA, 12mA, 16mA
    BSlowSlew         : Byte;       // non-zero if B pins have slow slew
    BSchmittInput     : Byte;       // non-zero if B pins are Schmitt input
    BDriveCurrent     : Byte;       // valid values are 4mA, 8mA, 12mA, 16mA
    CSlowSlew         : Byte;       // non-zero if C pins have slow slew
    CSchmittInput     : Byte;       // non-zero if C pins are Schmitt input
    CDriveCurrent     : Byte;       // valid values are 4mA, 8mA, 12mA, 16mA
    DSlowSlew         : Byte;       // non-zero if D pins have slow slew
    DSchmittInput     : Byte;       // non-zero if D pins are Schmitt input
    DDriveCurrent     : Byte;       // valid values are 4mA, 8mA, 12mA, 16mA

    // Hardware options

    ARIIsTXDEN        : Byte;       // non-zero if port A uses RI as RS485 TXDEN
    BRIIsTXDEN        : Byte;       // non-zero if port B uses RI as RS485 TXDEN
    CRIIsTXDEN        : Byte;       // non-zero if port C uses RI as RS485 TXDEN
    DRIIsTXDEN        : Byte;       // non-zero if port D uses RI as RS485 TXDEN

    // Driver option

    ADriverType       : Byte;       // non-zero if interface is to use VCP drivers
    BDriverType       : Byte;       // non-zero if interface is to use VCP drivers
    CDriverType       : Byte;       // non-zero if interface is to use VCP drivers
    DDriverType       : Byte;       // non-zero if interface is to use VCP drivers
  end;
  PFT_eeprom_4232h = ^TFT_eeprom_4232h;

  {........................................................................}

  {**  @noop FT_EEPROM_232H
   * @par Summary
   * Structure to hold data for the FT232H data in the FT_EEPROM_Program and FT_EEPROM_Read functions.
   * This is appended to an FT_EEPROM_HEADER structure.
   * @see FT_EEPROM_HEADER }
  TFT_eeprom_232h = packed record
    // Common header
    common            : TFT_eeprom_header;  // common elements for all device EEPROMs

    // Drive options
    ACSlowSlew        : Byte;       // non-zero if AC bus pins have slow slew
    ACSchmittInput    : Byte;       // non-zero if AC bus pins are Schmitt input
    ACDriveCurrent    : Byte;       // valid values are 4mA, 8mA, 12mA, 16mA
    ADSlowSlew        : Byte;       // non-zero if AD bus pins have slow slew
    ADSchmittInput    : Byte;       // non-zero if AD bus pins are Schmitt input
    ADDriveCurrent    : Byte;       // valid values are 4mA, 8mA, 12mA, 16mA

    // CBUS options

    Cbus0             : Byte;       // Cbus Mux control
    Cbus1             : Byte;       // Cbus Mux control
    Cbus2             : Byte;       // Cbus Mux control
    Cbus3             : Byte;       // Cbus Mux control
    Cbus4             : Byte;       // Cbus Mux control
    Cbus5             : Byte;       // Cbus Mux control
    Cbus6             : Byte;       // Cbus Mux control
    Cbus7             : Byte;       // Cbus Mux control
    Cbus8             : Byte;       // Cbus Mux control
    Cbus9             : Byte;       // Cbus Mux control

    // FT1248 options

    FT1248Cpol        : Byte;       // FT1248 clock polarity - clock idle high (1) or clock idle low (0)
    FT1248Lsb         : Byte;       // FT1248 data is LSB (1) or MSB (0)
    FT1248FlowControl : Byte;       // FT1248 flow control enable

    // Hardware options

    IsFifo            : Byte;       // non-zero if interface is 245 FIFO
    IsFifoTar         : Byte;       // non-zero if interface is 245 FIFO CPU target
    IsFastSer         : Byte;       // non-zero if interface is Fast serial
    IsFT1248          : Byte;       // non-zero if interface is FT1248
    PowerSaveEnable   : Byte;       //

    // Driver option
    DriverType        : Byte;       // non-zero if interface is to use VCP drivers
  end;
  PFT_eeprom_232h = ^TFT_eeprom_232h;

  {........................................................................}

  {**  @noop FT_EEPROM_X_SERIES
   * @par Summary
   * Structure to hold data for the FT-X series data in the FT_EEPROM_Program and FT_EEPROM_Read functions.
   * This is appended to an FT_EEPROM_HEADER structure.
   * @see FT_EEPROM_HEADER }
  TFT_eeprom_x_series = packed record
      // Common header
      common            : TFT_eeprom_header;  // common elements for all device EEPROMs

    // Drive options
    ACSlowSlew        : Byte;       // non-zero if AC bus pins have slow slew
    ACSchmittInput    : Byte;       // non-zero if AC bus pins are Schmitt input
    ACDriveCurrent    : Byte;       // valid values are 4mA, 8mA, 12mA, 16mA
    ADSlowSlew        : Byte;       // non-zero if AD bus pins have slow slew
    ADSchmittInput    : Byte;       // non-zero if AD bus pins are Schmitt input
    ADDriveCurrent    : Byte;       // valid values are 4mA, 8mA, 12mA, 16mA

    // CBUS options

    Cbus0             : Byte;       // Cbus Mux control
    Cbus1             : Byte;       // Cbus Mux control
    Cbus2             : Byte;       // Cbus Mux control
    Cbus3             : Byte;       // Cbus Mux control
    Cbus4             : Byte;       // Cbus Mux control
    Cbus5             : Byte;       // Cbus Mux control
    Cbus6             : Byte;       // Cbus Mux control

    // UART signal options

    InvertTXD         : Byte;       // non-zero if invert TXD
    InvertRXD         : Byte;       // non-zero if invert RXD
    InvertRTS         : Byte;       // non-zero if invert RTS
    InvertCTS         : Byte;       // non-zero if invert CTS
    InvertDTR         : Byte;       // non-zero if invert DTR
    InvertDSR         : Byte;       // non-zero if invert DSR
    InvertDCD         : Byte;       // non-zero if invert DCD
    InvertRI          : Byte;       // non-zero if invert RI

    // Battery Charge Detect options

    BCDEnable         : Byte;       // Enable Battery Charger Detection
    BCDForceCbusPWREN : Byte;       // asserts the power enable signal on CBUS when charging port detected
    BCDDisableSleep   : Byte;       // forces the device never to go into sleep mode

    // I2C options

    I2CSlaveAddress   : Word;       // I2C slave device address
    I2CDeviceId       : DWord;      // I2C device ID
    I2CDisableSchmitt : Byte;       // Disable I2C Schmitt trigger

    // FT1248 options

    FT1248Cpol        : Byte;       // FT1248 clock polarity - clock idle high (1) or clock idle low (0)
    FT1248Lsb         : Byte;       // FT1248 data is LSB (1) or MSB (0)
    FT1248FlowControl : Byte;       // FT1248 flow control enable

    // Hardware options

    RS485EchoSuppress : Byte;       //
    PowerSaveEnable   : Byte;       //

    // Driver option
    DriverType        : Byte;       // non-zero if interface is to use VCP drivers
  end;
  PFT_eeprom_x_series = ^TFT_eeprom_x_series;

  {**  @noop FT_EEPROM_4222H
   * @par Summary
   * Structure to hold data for the FT4222H data in the FT_EEPROM_Program and FT_EEPROM_Read functions.
   * This is appended to an FT_EEPROM_HEADER structure.
   * @see FT_EEPROM_HEADER }
  TFT_eeprom_4222h = packed record
    // Common header
    common              : TFT_EEPROM_HEADER;  // common elements for all device EEPROMs
    Revision            : Char;     // 'A', 'B', 'C', or 'D'.
    I2C_Slave_Address   : Byte;
    // Suspend
    SPISuspend          : Byte;     // 0 for "Disable SPI, tristate pins", 2 for "Keep SPI pin status", 3 for "Enable SPI pin control"
    SuspendOutPol       : Byte;     // 0 for negative, 1 for positive (not implemented on Rev A)
    EnableSuspendOut    : Byte;     // non-zero to enable (not implemented on Rev A)
    // QSPI
    Clock_SlowSlew      : Byte;     // non-zero if clock pin has slow slew
    Clock_Drive         : Byte;     // valid values are 4mA, 8mA, 12mA, 16mA
    IO0_SlowSlew        : Byte;     // non-zero if IO0 pin has slow slew
    IO1_SlowSlew        : Byte;     // non-zero if IO1 pin has slow slew
    IO2_SlowSlew        : Byte;     // non-zero if IO2 pin has slow slew
    IO3_SlowSlew        : Byte;     // non-zero if IO3 pin has slow slew
    IO_Drive            : Byte;     // valid values are 4mA, 8mA, 12mA, 16mA
    SlaveSelect_PullUp  : Byte;     // non-zero to enable pull up
    SlaveSelect_PullDown: Byte;     // non-zero to enable pull down
    SlaveSelect_Drive   : Byte;     // valid values are 4mA, 8mA, 12mA, 16mA
    SlaveSelect_SlowSlew: Byte;     // non-zero if slave select pin has slow slew
    MISO_Suspend        : Byte;     // 2 for push-low, 3 for push high, 0 and 1 reserved
    SIMO_Suspend        : Byte;     // 2 for push-low, 3 for push high, 0 and 1 reserved
    IO2_IO3_Suspend     : Byte;     // 2 for push-low, 3 for push high, 0 and 1 reserved
    SlaveSelect_Suspend : Byte;     // 0 for no-change (not implemented on Rev A), 2 for push-low, 3 for push high, 1 reserved
    // GPIO
    GPIO0_Drive         : Byte;     // valid values are 4mA, 8mA, 12mA, 16mA
    GPIO1_Drive         : Byte;     // valid values are 4mA, 8mA, 12mA, 16mA
    GPIO2_Drive         : Byte;     // valid values are 4mA, 8mA, 12mA, 16mA
    GPIO3_Drive         : Byte;     // valid values are 4mA, 8mA, 12mA, 16mA
    GPIO0_SlowSlew      : Byte;     // non-zero if IO0 pin has slow slew
    GPIO1_SlowSlew      : Byte;     // non-zero if IO0 pin has slow slew
    GPIO2_SlowSlew      : Byte;     // non-zero if IO0 pin has slow slew
    GPIO3_SlowSlew      : Byte;     // non-zero if IO0 pin has slow slew
    GPIO0_PullDown      : Byte;     // non-zero to enable pull down
    GPIO1_PullDown      : Byte;     // non-zero to enable pull down
    GPIO2_PullDown      : Byte;     // non-zero to enable pull down
    GPIO3_PullDown      : Byte;     // non-zero to enable pull down
    GPIO0_PullUp        : Byte;     // non-zero to enable pull up
    GPIO1_PullUp        : Byte;     // non-zero to enable pull up
    GPIO2_PullUp        : Byte;     // non-zero to enable pull up
    GPIO3_PullUp        : Byte;     // non-zero to enable pull up
    GPIO0_OpenDrain     : Byte;     // non-zero to enable open drain
    GPIO1_OpenDrain     : Byte;     // non-zero to enable open drain
    GPIO2_OpenDrain     : Byte;     // non-zero to enable open drain
    GPIO3_OpenDrain     : Byte;     // non-zero to enable open drain
    GPIO0_Suspend       : Byte;     // 0 for no-change, 1 for input (not implemented on Rev A), 2 for push-low, 3 for push high
    GPIO1_Suspend       : Byte;     // 0 for no-change, 1 for input (not implemented on Rev A), 2 for push-low, 3 for push high
    GPIO2_Suspend       : Byte;     // 0 for no-change, 1 for input (not implemented on Rev A), 2 for push-low, 3 for push high
    GPIO3_Suspend       : Byte;     // 0 for no-change, 1 for input (not implemented on Rev A), 2 for push-low, 3 for push high
    FallingEdge         : Byte;     // non-zero to change GPIO on falling edge
    // BCD
    BCD_Disable         : Byte;     // non-zero to disable BCD
    BCD_OutputActiveLow : Byte;     // non-zero to set BCD output active low
    BCD_Drive           : Byte;     // valid values are 4mA, 8mA, 12mA, 16mA
  end;
  PFT_eeprom_4222h = ^TFT_eeprom_4222h;

  {**  @noop FT_EEPROM_PD_PDO_mv_ma
   * @par Summary
   * Structure to hold PDO Configuration structure, mA supported values 0 to 10230mA, mV supported
   * values 0 to 51100mV. This is part of the FT_EEPROM_PD structure.
   * @see FT_EEPROM_PD }
  TFT_eeprom_PD_PDO_mv_ma = packed record
    PDO1ma              : Word;   // PDO1 mA
    PDO1mv              : Word;   // PDO1 mV
    PDO2ma              : Word;   // PDO2 mA
    PDO2mv              : Word;   // PDO2 mV
    PDO3ma              : Word;   // PDO3 mA
    PDO3mv              : Word;   // PDO3 mV
    PDO4ma              : Word;   // PDO4 mA
    PDO4mv              : Word;   // PDO4 mV
    PDO5ma              : Word;   // PDO5 mA (FTx233HP only)
    PDO5mv              : Word;   // PDO5 mV (FTx233HP only)
    PDO6ma              : Word;   // PDO6 mA (FTx233HP only)
    PDO6mv              : Word;   // PDO6 mV (FTx233HP only)
    PDO7ma              : Word;   // PDO7 mA (FTx233HP only)
    PDO7mv              : Word;   // PDO7 mV (FTx233HP only)
  end;
  PFT_eeprom_PD_PDO_mv_ma = ^TFT_eeprom_PD_PDO_mv_ma;

  {**  @noop FT_EEPROM_PD
   * @par Summary
   * Structure to hold power delivery configuration data for the FT4233PD, FT2233PD, FT4232PD,
   * FT2232PD, FT233PD and FT232PD in the FT_EEPROM_Program and FT_EEPROM_Read functions.
   * This is appended to an FT_EEPROM_HEADER and a base device structure.
   * e_g. @verbatim
   *      struct {
   *          FT_EEPROM_xxx base;
   *          FT_EEPROM_PD pd;
   *      };
   * @endverbatim
   * @remarks
   * Device GPIO values are:
   * @li  FTx233HP - 0 to 7, 15 for N/A
   * @li FTx232HP - 0 to 3, 15 for N/A
   * @see FT_EEPROM_HEADER
   * @see FT_EEPROM_PD_PDO_mv_ma }
  TFT_eeprom_pd = packed record

    // Configuration

    srprs               : Byte;     // non-zero to enable Sink Request Power Role Swap
    sraprs              : Byte;     // non-zero to enable Sink Accept PR Swap
    srrprs              : Byte;     // non-zero to enable Source Request PR SWAP
    saprs               : Byte;     // non-zero to enable Source Accept PR SWAP
    vconns              : Byte;     // non-zero to enable vConn Swap
    passthru            : Byte;     // non-zero to enable Pass Through (FTx233HP only)
    extmcu              : Byte;     // non-zero to enable External MCU
    pd2en               : Byte;     // non-zero to enable PD2 (FTx233HP only)
    pd1autoclk          : Byte;     // non-zero to enable PD1 Auto Clock
    pd2autoclk          : Byte;     // non-zero to enable PD2 Auto Clock (FTx233HP only)
    useefuse            : Byte;     // non-zero to Use EFUSE
    extvconn            : Byte;     // non-zero to enable External vConn

    // GPIO Configuration

    count               : Byte;     // GPIO Count, supported values are 0 to 7
    gpio1               : Byte;     // GPIO Number 1, supports device GPIO values
    gpio2               : Byte;     // GPIO Number 2, supports device GPIO values
    gpio3               : Byte;     // GPIO Number 3, supports device GPIO values
    gpio4               : Byte;     // GPIO Number 4, supports device GPIO values
    gpio5               : Byte;     // GPIO Number 5, supports device GPIO values (FTx233HP only)
    gpio6               : Byte;     // GPIO Number 6, supports device GPIO values (FTx233HP only)
    gpio7               : Byte;     // GPIO Number 7, supports device GPIO values (FTx233HP only)
    pd1lden             : Byte;     // PD1 Load Enable, supports device GPIO values
    pd2lden             : Byte;     // PD2 Load Enable, supports device GPIO values (FTx233HP only)
    dispin              : Byte;     // Discharge Pin, supports device GPIO values
    disenbm             : Byte;     // Discharge Enable BM, 0 for "Drive Hi", 1 for "Drive Low", 2 for "Input Mode", 3 for "Don't Care"
    disdisbm            : Byte;     // Discharge Disable BM, 0 for "Drive Hi", 1 for "Drive Low", 2 for "Input Mode", 3 for "Don't Care"
    ccselect            : Byte;     // CC Select Indicator, supports device GPIO values

    // ISET Configuration

    iset1               : Byte;     // ISET1, supports device GPIO values
    iset2               : Byte;     // ISET2, supports device GPIO values
    iset3               : Byte;     // ISET3, supports device GPIO values
    extiset             : Byte;     // non-zero to enable EXTEND_ISET
    isetpd2             : Byte;     // non-zero to enable ISET_PD2
    iseten              : Byte;     // non-zero to set ISET_ENABLED

    // BM Configuration, 0 for "Drive Hi", 1 for "Drive Low", 2 for "Input Mode", 3 for "Don't Care"

    PDO1_GPIO           : TByte7;   // PDO1 GPIO1 to GPIO7
    PDO2_GPIO           : TByte7;   // PDO2 GPIO1 to GPIO7
    PDO3_GPIO           : TByte7;   // PDO3 GPIO1 to GPIO7
    PDO4_GPIO           : TByte7;   // PDO4 GPIO1 to GPIO7
    PDO5_GPIO           : TByte7;   // PDO5 GPIO1 to GPIO7 (FTx233HP only)
    PDO6_GPIO           : TByte7;   // PDO6 GPIO1 to GPIO7 (FTx233HP only)
    PDO7_GPIO           : TByte7;   // PDO7 GPIO1 to GPIO7 (FTx233HP only)
    VSET0V_GPIO         : TByte7;   // PDO7 GPIO1 to GPIO7
    VSAFE5V_GPIO        : TByte7;   // PDO7 GPIO1 to GPIO7

    BM_PDO_Sink         : TFT_EEPROM_PD_PDO_mv_ma;
    BM_PDO_Source       : TFT_EEPROM_PD_PDO_mv_ma;
    BM_PDO_Sink_2       : TFT_EEPROM_PD_PDO_mv_ma; // (FTx233HP only)

    // PD Timers

    srt                 : Byte;     // Sender Response Timer
    hrt                 : Byte;     // Hard Reset Timer
    sct                 : Byte;     // Source Capability Timer
    dit                 : Byte;     // Discover Identity Timer
    srcrt               : Word;     // Source Recover Timer
    trt                 : Word;     // Transition Timer
    sofft               : Word;     // Source off timer
    nrt                 : Word;     // No Response Timer
    swct                : Word;     // Sink Wait Capability Timer
    snkrt               : Word;     // Sink Request Timer
    dt                  : Byte;     // Discharge Timer
    cnst                : Byte;     // Chunk not supported timer
    it                  : Word;     // Idle Timer

    // PD Control

    i2caddr             : Byte;     // I2C Address (hex)
    prou                : cardinal; // Power Reserved for OWN use
    trim1               : cardinal; // TRIM1
    trim2               : cardinal; // TRIM2
    extdc               : Byte;     // non-zero to enable ETERNAL_DC_POWER
  end;
  PFT_eeprom_pd = ^TFT_eeprom_pd;

  // FT2233HP EEPROM structure for use with FT_EEPROM_Read and FT_EEPROM_Program
  // FT2232H with power delivery
  TFT_eeprom_2233hp = packed record
    ft2232h             : TFT_eeprom_2232h;
    pd                  : TFT_eeprom_pd;
  end;
  PFT_eeprom_2233hp = ^TFT_eeprom_2233hp;

  // FT4233HP EEPROM structure for use with FT_EEPROM_Read and FT_EEPROM_Program
  // FT4232H with power delivery
  TFT_eeprom_4233hp = packed record
    ft4232h             : TFT_eeprom_4232h;
    pd                  : TFT_eeprom_pd;
  end;
  PFT_eeprom_4233hp = ^TFT_eeprom_4233hp;

  // FT2232HP EEPROM structure for use with FT_EEPROM_Read and FT_EEPROM_Program
  // FT2232H with power delivery
  TFT_eeprom_2232hp = packed record
    ft2232h             : TFT_eeprom_2232h;
    pd                  : TFT_eeprom_pd;
  end;
  PFT_eeprom_2232hp = ^TFT_eeprom_2232hp;

  // FT4232HP EEPROM structure for use with FT_EEPROM_Read and FT_EEPROM_Program
  // FT4232H with power delivery
  TFT_eeprom_4232hp = packed record
    ft4232h             : TFT_eeprom_4232h;
    pd                  : TFT_eeprom_pd;
  end;
  PFT_eeprom_4232hp = ^TFT_eeprom_4232hp;

  // FT233HP EEPROM structure for use with FT_EEPROM_Read and FT_EEPROM_Program
  // FT233H with power delivery
  TFT_eeprom_233hp = packed record
    ft232h              : TFT_eeprom_232h;
    pd                  : TFT_eeprom_pd;
  end;
  PFT_eeprom_233hp = ^TFT_eeprom_233hp;

  // FT232HP EEPROM structure for use with FT_EEPROM_Read and FT_EEPROM_Program
  // FT232H with power delivery
  TFT_eeprom_232hp = packed record
    ft232h              : TFT_eeprom_232h;
    pd                  : TFT_eeprom_pd;
  end;
  PFT_eeprom_232hp = ^TFT_eeprom_232hp;

var

  {** @noop FT_EEPROM_Read
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (XP and later)
   * @par Summary
   * Read data from the EEPROM, this command will work for all existing FTDI chipset, and must be
   * used for the FT-X series.
   * @param ftHandle Handle of the device.
   * @param *eepromData Pointer to a buffer that contains the data to be read.
   * Note: This structure is different for each device type.
   * @param eepromDataSize Size of the eepromData buffer that contains storage for the data to be read.
   * @param *Manufacturer Pointer to a null-terminated string containing the manufacturer name.
   * @param *ManufacturerId Pointer to a null-terminated string containing the manufacturer ID.
   * @param *Description Pointer to a null-terminated string containing the device description.
   * @param *SerialNumber Pointer to a null-terminated string containing the device serial number.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code.
   * @remarks
   * This function interprets the parameter *eepromDATA as a pointer to a structure matching the device
   * type being accessed e.g.
   * @li PFT_EEPROM_232B is the structure for FT2xxB devices.
   * @li PFT_EEPROM_2232 is the structure for FT2232D devices.
   * @li PFT_EEPROM_232R is the structure for FT232R devices.
   * @li PFT_EEPROM_2232H is the structure for FT2232H devices.
   * @li PFT_EEPROM_4232H is the structure for FT4232H devices.
   * @li PFT_EEPROM_232H is the structure for FT232H devices.
   * @li PFT_EEPROM_X_SERIES is the structure for FT2xxX devices.
   *
   * The function does not perform any checks on buffer sizes, so the buffers passed in the eepromDATA
   * structure must be big enough to accommodate their respective strings (including null terminators).
   * The sizes shown in the following example are more than adequate and can be rounded down if necessary.
   * The restriction is that the Manufacturer string length plus the Description string length is less than or
   * equal to 40 characters.
   * @note Note that the DLL must be informed which version of the eepromDATA structure is being used. This is
   * done through the PFT_EEPROM_HEADER structure. The first element of this structure is deviceType and
   * may be FT_DEVICE_BM, FT_DEVICE_AM, FT_DEVICE_2232C, FT_DEVICE_232R, FT_DEVICE_2232H,
   * FT_DEVICE_4232H, FT_DEVICE_232H, or FT_DEVICE_X_SERIES as defined in FTD2XX.h. }
  FT_EEPROM_Read: function(Handle: TFTHandle; eepromData: pointer; eepromDataSize: DWord; Manufacturer, ManufacturerId, Description, SerialNumber: PChar): TFTStatus; FTD2XX_API;

  {** @noop FT_EEPROM_Program
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (XP and later)
   * @par Summary
   * Write data into the EEPROM, this command will work for all existing FTDI chipset, and must be used for
   * the FT-X series.
   * @param ftHandle Handle of the device.
   * @param *eepromData Pointer to a buffer that contains the data to be written.
   * Note: This structure is different for each device type.
   * @param eepromDataSize Size of the eepromData buffer that contains storage for the data to be written.
   * @param *Manufacturer Pointer to a null-terminated string containing the manufacturer name.
   * @param *ManufacturerId Pointer to a null-terminated string containing the manufacturer ID.
   * @param *Description Pointer to a null-terminated string containing the device description.
   * @param *SerialNumber Pointer to a null-terminated string containing the device serial number.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code.
   * @remarks
   * This function interprets the parameter *eepromDATA as a pointer to a structure matching the device
   * type being accessed e.g.
   * @li PFT_EEPROM_232B is the structure for FT2xxB devices.
   * @li PFT_EEPROM_2232 is the structure for FT2232D devices.
   * @li PFT_EEPROM_232R is the structure for FT232R devices.
   * @li PFT_EEPROM_2232H is the structure for FT2232H devices.
   * @li PFT_EEPROM_4232H is the structure for FT4232H devices.
   * @li PFT_EEPROM_232H is the structure for FT232H devices.
   * @li PFT_EEPROM_X_SERIES is the structure for FT2xxX devices.
   *
   * The function does not perform any checks on buffer sizes, so the buffers passed in the eepromDATA
   * structure must be big enough to accommodate their respective strings (including null terminators).
   * @n The sizes shown in the following example are more than adequate and can be rounded down if necessary.
   * The restriction is that the Manufacturer string length plus the Description string length is less than or
   * equal to 40 characters.
   * @note Note that the DLL must be informed which version of the eepromDATA structure is being used. This is
   * done through the PFT_EEPROM_HEADER structure. The first element of this structure is deviceType and
   * may be FT_DEVICE_BM, FT_DEVICE_AM, FT_DEVICE_2232C, FT_DEVICE_232R, FT_DEVICE_2232H,
   * FT_DEVICE_4232H, FT_DEVICE_232H, or FT_DEVICE_X_SERIES as defined in FTD2XX.h. }
  FT_EEPROM_Program: function(Handle: TFTHandle; eepromData: pointer; eepromDataSize: DWord; Manufacturer, ManufacturerId, Description, SerialNumber: PChar): TFTStatus; FTD2XX_API;

  {........................................................................}

  {** @name Extended API Functions
   * The extended API functions do not apply to FT8U232AM or FT8U245AM devices. FTDI’s other USB-UART
   * and USB-FIFO ICs (the FT2232H, FT4232H, FT232R, FT245R, FT2232, FT232B and FT245B) do support
   * these functions. Note that there is device dependence in some of these functions. }

  {........................................................................}

  {** @noop FT_SetLatencyTimer
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * Set the latency timer value.
   * @param ftHandle Handle of the device.
   * @param ucLatency Required value, in milliseconds, of latency timer. Valid range is 2 - 255.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code.
   * @remarks
   * In the FT8U232AM and FT8U245AM devices, the receive buffer timeout that is used to flush remaining
   * data from the receive buffer was fixed at 16 ms. In all other FTDI devices, this timeout is
   * programmable and can be set at 1 ms intervals between 2ms and 255 ms. This allows the device to
   * be better optimize for protocols requiring faster response times from short data packets. }
  FT_SetLatencyTimer: function(Handle: TFTHandle; Latency: Byte): TFTStatus; FTD2XX_API;

  {** @noop FT_GetLatencyTimer
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * Get the current value of the latency timer.
   * @param ftHandle Handle of the device.
   * @param pucLatency Pointer to unsigned char to store latency timer value.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code.
   * @remarks
   * In the FT8U232AM and FT8U245AM devices, the receive buffer timeout that is used to flush remaining
   * data from the receive buffer was fixed at 16 ms. In all other FTDI devices, this timeout is
   * programmable and can be set at 1 ms intervals between 2ms and 255 ms. This allows the device to
   * be better optimized for protocols requiring faster response times from short data packets. }
  FT_GetLatencyTimer: function(Handle: TFTHandle; Latency: PByte): TFTStatus; FTD2XX_API;

  {** @noop FT_SetBitMode
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * Enables different chip modes.
   * @param ftHandle Handle of the device.
   * @param ucMask Required value for bit mode mask. This sets up which bits are inputs and outputs.
   * A bit value of 0 sets the corresponding pin to an input, a bit value of 1 sets the corresponding
   * pin to an output.
   * @n In the case of CBUS Bit Bang, the upper nibble of this value controls which pins are inputs
   * and outputs, while the lower nibble controls which of the outputs are high and low.
   * @param ucEnable Mode value. Can be one of the following:
   * @li 0x0 = Reset
   * @li 0x1 = Asynchronous Bit Bang
   * @li 0x2 = MPSSE (FT2232, FT2232H, FT4232H and FT232H devices only)
   * @li 0x4 = Synchronous Bit Bang (FT232R, FT245R, FT2232, FT2232H, FT4232H and FT232H devices only)
   * @li 0x8 = MCU Host Bus Emulation Mode (FT2232, FT2232H, FT4232H and FT232H devices only)
   * @li 0x10 = Fast Opto-Isolated Serial Mode (FT2232, FT2232H, FT4232H and FT232H devices only)
   * @li 0x20 = CBUS Bit Bang Mode (FT232R and FT232H devices only)
   * @li 0x40 = Single Channel Synchronous 245 FIFO Mode (FT2232H and FT232H devices only)
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code.
   * @remarks
   * For a description of available bit modes for the FT232R, see the application note "Bit Bang Modes
   * for the FT232R and FT245R".
   * @n For a description of available bit modes for the FT2232, see the application note "Bit Mode
   * Functions for the FT2232".
   * @n For a description of Bit Bang Mode for the FT232B and FT245B, see the application note
   * "FT232B/FT245B Bit Bang Mode".
   * @n Application notes are available for download from the FTDI website.
   * Note that to use CBUS Bit Bang for the FT232R, the CBUS must be configured for CBUS Bit Bang in the
   * EEPROM.
   * @note Note that to use Single Channel Synchronous 245 FIFO mode for the FT2232H, channel A must be
   * configured for FT245 FIFO mode in the EEPROM. }
  FT_SetBitMode: function(Handle: TFTHandle; Mask, Enable: Byte): TFTStatus; FTD2XX_API;

  {** @noop FT_GetBitMode
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * Gets the instantaneous value of the data bus.
   * @param ftHandle Handle of the device.
   * @param pucMode Pointer to unsigned char to store the instantaneous data bus value.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code.
   * @remarks
   * For a description of available bit modes for the FT232R, see the application note "Bit Bang Modes
   * for the FT232R and FT245R".
   * @n For a description of available bit modes for the FT2232, see the application note "Bit Mode
   * Functions for the FT2232".
   * @n For a description of bit bang modes for the FT232B and FT245B, see the application note
   * "FT232B/FT245B Bit Bang Mode".
   * @n For a description of bit modes supported by the FT4232H and FT2232H devices, please see the
   * IC data sheets.
   * @n These application notes are available for download from the FTDI website. }
  FT_GetBitMode: function(Handle: TFTHandle; Mode: PByte): TFTStatus; FTD2XX_API;

  {** @noop FT_SetUSBParameters
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * Set the USB request transfer size.
   * @param ftHandle Handle of the device.
   * @param ulInTransferSize Transfer size for USB IN request.
   * @param ulOutTransferSize Transfer size for USB OUT request.
   * @returns
   * FT_OK if successful, otherwise the return value is an FT error code.
   * @remarks
   * This function can be used to change the transfer sizes from the default transfer size of 4096
   * bytes to better suit the application requirements. Transfer sizes must be set to a multiple
   * of 64 bytes between 64 bytes and 64k bytes.
   * @n When FT_SetUSBParameters is called, the change comes into effect immediately and any data
   * that was held in the driver at the time of the change is lost.
   * @note Note that, at present, only ulInTransferSize is supported. }
  FT_SetUSBParameters: function(Handle: TFTHandle; InTransferSize, OutTransferSize: LongWord): TFTStatus; FTD2XX_API;

  {........................................................................}

  {** @name FT-Win32 API Functions
   * The functions in this section are supplied to ease porting from a Win32 serial port application. These
   * functions are supported under non-Windows platforms to assist with porting existing applications from
   * Windows. Note that classic D2XX functions and the Win32 D2XX functions should not be mixed unless
   * stated. }

  {........................................................................}

  {** @noop FT_W32_CreateFile
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * Opens the specified device and return a handle which will be used for subsequent accesses.
   * The device can be specified by its serial number, device description, or location.
   * @n This function must be used if overlapped I/O is required.
   * @param lpszName Meaning depends on the value of dwAttrsAndFlags. Can be a pointer to a null
   * terminated string that contains the description or serial number of the device, or can be
   * the location of the device. These values can be obtained from the FT_CreateDeviceInfoList,
   * FT_GetDeviceInfoDetail or FT_ListDevices functions.
   * @param dwAccess Type of access to the device. Access can be GENERIC_READ,
   * GENERIC_WRITE or both. Ignored in Linux.
   * @param dwShareMode How the device is shared. This value must be set to 0.
   * @param lpSecurityAttributes This parameter has no effect and should be set to NULL.
   * @param dwCreate This parameter must be set to OPEN_EXISTING. Ignored in Linux.
   * @param dwAttrsAndFlags File attributes and flags. This parameter is a combination of
   * FILE_ATTRIBUTE_NORMAL, FILE_FLAG_OVERLAPPED if overlapped I/O is used,
   * FT_OPEN_BY_SERIAL_NUMBER if lpszName is the device’s serial number, and
   * FT_OPEN_BY_DESCRIPTION if lpszName is the device’s description.
   * @param hTemplate This parameter must be NULL.
   * @returns
   * If the function is successful, the return value is a handle.
   * If the function is unsuccessful, the return value is the Win32 error code INVALID_HANDLE_VALUE.
   * @remarks
   * The meaning of lpszName depends on dwAttrsAndFlags: if FT_OPEN_BY_SERIAL_NUMBER or
   * FT_OPEN_BY_DESCRIPTION is set in dwAttrsAndFlags, lpszName contains a pointer to a null terminated
   * string that contains the device's serial number or description; if FT_OPEN_BY_LOCATION is set in
   * dwAttrsAndFlags, lpszName is interpreted as a value of type long that contains the location ID of
   * the device. dwAccess can be GENERIC_READ, GENERIC_WRITE or both; dwShareMode must be set to 0;
   * lpSecurityAttributes must be set to NULL; dwCreate must be set to OPEN_EXISTING; dwAttrsAndFlags
   * is a combination of FILE_ATTRIBUTE_NORMAL, FILE_FLAG_OVERLAPPED if overlapped I/O is used,
   * FT_OPEN_BY_SERIAL_NUMBER or FT_OPEN_BY_DESCRIPTION or FT_OPEN_BY_LOCATION; hTemplate
   * must be NULL.
   * @note Note that Linux, Mac OS X and Windows CE do not support overlapped IO. Windows CE
   * does not support location IDs. }
  FT_W32_CreateFile: function(const Name: PChar; Access, ShareMode: DWord; SecurityAttributes: PSecurityAttributes; Create, AttrsAndFlags: DWord; Template: THandle): TFTHandle; FTD2XX_API;

  {** @noop FT_W32_CloseHandle
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * Close the specified device handle.
   * @param ftHandle Handle of the device.
   * @returns
   * If the function is successful, the return value is nonzero.
   * If the function is unsuccessful, the return value is zero. }
  FT_W32_CloseHandle: function(Handle: TFTHandle): LongBool; FTD2XX_API;

  {** @noop FT_W32_ReadFile
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * Read data from the device.
   * @param ftHandle Handle of the device.
   * @param lpBuffer Pointer to a buffer that receives the data from the device.
   * @param nBufferSize Number of bytes to read from the device.
   * @param lpdwBytesReturned Pointer to a variable that receives the number of bytes read from
   * the device.
   * @param lpOverlapped Pointer to an overlapped structure.
   * @returns
   * If the function is successful, the return value is nonzero.
   * If the function is unsuccessful, the return value is zero.
   * @remarks
   * This function supports both non-overlapped and overlapped I/O, except under Linux, Mac OS X
   * and Windows CE where only non-overlapped IO is supported.
   * @n @b Non-overlapped @b I/O
   * @n The parameter, lpOverlapped, must be NULL for non-overlapped I/O.
   * @n This function always returns the number of bytes read in lpdwBytesReturned.
   * This function does not return until dwBytesToRead have been read into the buffer. The number of
   * bytes in the receive queue can be determined by calling FT_GetStatus or FT_GetQueueStatus, and
   * passed as dwBytesToRead so that the function reads the device and returns immediately.
   * @n When a read timeout has been setup in a previous call to FT_W32_SetCommTimeouts, this function
   * returns when the timer expires or dwBytesToRead have been read, whichever occurs first. If a
   * timeout occurred, any available data is read into lpBuffer and the function returns a non-zero value.
   * @n An application should use the function return value and lpdwBytesReturned when processing the
   * buffer. If the return value is non-zero and lpdwBytesReturned is equal to dwBytesToRead then the
   * function has completed normally. If the return value is non-zero and lpdwBytesReturned is less
   * then dwBytesToRead then a timeout has occurred, and the read request has been partially completed.
   * @note Note that if a timeout occurred and no data was read, the return value is still non-zero.
   * @n @b Overlapped @b I/O
   * @n When the device has been opened for overlapped I/O, an application can issue a request and
   * perform some additional work while the request is pending. This contrasts with the case
   * of non-overlapped I/O in which the application issues a request and receives control again only
   * after the request has been completed.
   * @n The parameter, lpOverlapped, must point to an initialized OVERLAPPED structure. If there is
   * enough data in the receive queue to satisfy the request, the request completes immediately and
   * the return code is non-zero. The number of bytes read is returned in lpdwBytesReturned.
   * @n If there is not enough data in the receive queue to satisfy the request, the request completes
   * immediately, and the return code is zero, signifying an error. An application should call
   * FT_W32_GetLastError to get the cause of the error. If the error code is ERROR_IO_PENDING, the
   * overlapped operation is still in progress, and the application can perform other processing.
   * Eventually, the application checks the result of the overlapped request by calling
   * FT_W32_GetOverlappedResult.
   * @n If successful, the number of bytes read is returned in lpdwBytesReturned. }
  FT_W32_ReadFile: function(Handle: TFTHandle; Buffer: pointer; BufferSize: DWord; BytesReturns: PDWord; Overlapped: POverlapped): LongBool; FTD2XX_API;

  {** @noop FT_W32_WriteFile
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * Write data to the device.
   * @param ftHandle Handle of the device.
   * @param lpBuffer Pointer to the buffer that contains the data to write to the device.
   * @param nBufferSize Number of bytes to be written to the device.
   * @param lpdwBytesWritten Pointer to a variable that receives the number of bytes written to the device.
   * @param lpOverlapped Pointer to an overlapped structure.
   * @returns
   * If the function is successful, the return value is nonzero.
   * If the function is unsuccessful, the return value is zero.
   * @remarks
   * This function supports both non-overlapped and overlapped I/O, except under Linux, Mac OS X and
   * Windows CE where only non-overlapped IO is supported.
   * @n @b Non-overlapped @b I/O
   * @n The parameter, lpOverlapped, must be NULL for non-overlapped I/O.
   * @n This function always returns the number of bytes written in lpdwBytesWritten.
   * This function does not return until dwBytesToWrite have been written to the device.
   * When a write timeout has been setup in a previous call to FT_W32_SetCommTimeouts, this function
   * returns when the timer expires or dwBytesToWrite have been written, whichever occurs first. If a
   * timeout occurred, lpdwBytesWritten contains the number of bytes actually written, and the function
   * returns a non-zero value.
   * @n An application should always use the function return value and lpdwBytesWritten. If the return
   * value is non-zero and lpdwBytesWritten is equal to dwBytesToWrite then the function has completed
   * normally. If the return value is non-zero and lpdwBytesWritten is less then dwBytesToWrite then a
   * timeout has occurred, and the write request has been partially completed.
   * @note Note that if a timeout occurred and no data was written, the return value is still non-zero.
   * @n @b Overlapped @b I/O
   * @n When the device has been opened for overlapped I/O, an application can issue a request and perform
   * some additional work while the request is pending. This contrasts with the case of non-overlapped
   * I/O in which the application issues a request and receives control again only after the request has
   * been completed.
   * @n The parameter, lpOverlapped, must point to an initialized OVERLAPPED structure.
   * This function completes immediately, and the return code is zero, signifying an error. An application
   * should call FT_W32_GetLastError to get the cause of the error. If the error code is ERROR_IO_PENDING,
   * the overlapped operation is still in progress, and the application can perform other processing.
   * Eventually, the application checks the result of the overlapped request by calling
   * FT_W32_GetOverlappedResult.
   * @n If successful, the number of bytes written is returned in lpdwBytesWritten. }
  FT_W32_WriteFile: function(Handle: TFTHandle; Buffer: pointer; BufferSize: DWord; Written: PDWord; Overlapped: POverlapped): LongBool; FTD2XX_API;

  {** @noop FT_W32_GetOverlappedResult
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * Gets the result of an overlapped operation.
   * @param ftHandle Handle of the device.
   * @param lpOverlapped Pointer to an overlapped structure.
   * @param lpdwBytesTransferred Pointer to a variable that receives the number of bytes transferred
   * during the overlapped operation.
   * @param bWait Set to TRUE if the function does not return until the operation has been completed.
   * @returns
   * If the function is successful, the return value is nonzero.
   * If the function is unsuccessful, the return value is zero.
   * @remarks
   * This function is used with overlapped I/O and so is not supported in Linux, Mac OS X or Windows CE. For
   * a description of its use, see FT_W32_ReadFile and FT_W32_WriteFile. }
  FT_W32_GetOverlappedResult: function(Handle: TFTHandle; Overlapped: POverlapped; BytesTransferred: PDWord; Wait: LongBool): LongBool; FTD2XX_API;

  {** @noop FT_W32_EscapeCommFunction
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * Perform an extended function.
   * @param ftHandle Handle of the device.
   * @param dwFunc The extended function to perform can be one of the following values:
   * @li CLRDTR - Clear the DTR signal
   * @li CLRRTS - Clear the RTS signal
   * @li SETDTR - Set the DTR signal
   * @li SETRTS - Set the RTS signal
   * @li SETBREAK - Set the BREAK condition
   * @li CLRBREAK - Clear the BREAK condition
   * @returns
   * If the function is successful, the return value is nonzero.
   * If the function is unsuccessful, the return value is zero. }
  FT_W32_EscapeCommFunction: function(Handle: TFTHandle; Func: DWord): LongBool; FTD2XX_API;

  {** @noop FT_W32_GetCommModemStatus
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * This function gets the current modem control value.
   * @param ftHandle Handle of the device.
   * @param lpdwModemStatus Pointer to a variable to contain modem control value. The modem
   * control value can be a combination of the following:
   * @li MS_CTS_ON - Clear To Send (CTS) is on
   * @li MS_DSR_ON - Data Set Ready (DSR) is on
   * @li MS_RING_ON - Ring Indicator (RI) is on
   * @li MS_RLSD_ON - Receive Line Signal Detect (RLSD) is on
   * @returns
   * If the function is successful, the return value is nonzero.
   * If the function is unsuccessful, the return value is zero. }
  FT_W32_GetCommModemStatus: function(Handle: TFTHandle; ModemStatus: PDWord): LongBool; FTD2XX_API;

  {** @noop FT_W32_SetupComm
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * This function sets the read and write buffers.
   * @param ftHandle Handle of the device.
   * @param dwReadBufferSize Length, in bytes, of the read buffer.
   * @param dwWriteBufferSize Length, in bytes, of the write buffer.
   * @returns
   * If the function is successful, the return value is nonzero.
   * If the function is unsuccessful, the return value is zero.
   * @remarks
   * This function has no effect. It is the responsibility of the driver to allocate sufficient
   * storage for I/O requests. }
  FT_W32_SetupComm: function(Handle: TFTHandle; ReadBufferSize, WriteBufferSize: DWord): LongBool; FTD2XX_API;

type

  {** Structure for FT_W32_ClearCommError lpftComstat parameter.
   * @see FT_W32_ClearCommError }
  TFTComStat = packed record
    fCtsHold  : DWord;
    fDsrHold  : DWord;
    fRlsdHold : DWord;
    fXoffHold : DWord;
    fXoffSent : DWord;
    fEof      : DWord;
    fTxim     : DWord;
    fReserved : DWord;
    cbInQue   : DWord;
    cbOutQue  : DWord;
  end;
  PFTComStat = ^TFTComStat;

  {** Structure for FT_W32_SetCommState and FT_W32_GetCommState lpftDcb parameter.
   * @see FT_W32_SetCommState
   * @see FT_W32_GetCommState }
  TFTDCB = packed record
    DCBlength         : DWord;      // * sizeof(FTDCB)
    BaudRate          : DWord;      // * Baudrate at which running
    fBinary           : DWord;      // * Binary Mode (skip EOF check
    fParity           : DWord;      // * Enable parity checking
    fOutxCtsFlow      : DWord;      // * CTS handshaking on output
    fOutxDsrFlow      : DWord;      // * DSR handshaking on output
    fDtrControl       : DWord;      // * DTR Flow control
    fDsrSensitivity   : DWord;      // * DSR Sensitivity
    fTXContinueOnXoff : DWord;      // * Continue TX when Xoff sent
    fOutX             : DWord;      // * Enable output X-ON/X-OFF
    fInX              : DWord;      // * Enable input X-ON/X-OFF
    fErrorChar        : DWord;      // * Enable Err Replacement
    fNull             : DWord;      // * Enable Null stripping
    fRtsControl       : DWord;      // * Rts Flow control
    fAbortOnError     : DWord;      // * Abort all reads and writes on Error
    fDummy2           : DWord;      // * Reserved
    wReserved         : Word;       // * Not currently used
    XonLim            : Word;       // * Transmit X-ON threshold
    XoffLim           : Word;       // * Transmit X-OFF threshold
    ByteSize          : Byte;       // * Number of bits/byte, 4-8
    Parity            : Byte;       // * 0-4=None,Odd,Even,Mark,Space
    StopBits          : Byte;       // * FT_STOP_BITS_1 or FT_STOP_BITS_2
    XonChar           : Char;       // * Tx and Rx X-ON character
    XoffChar          : Char;       // * Tx and Rx X-OFF character
    ErrorChar         : Char;       // * Error replacement char
    EofChar           : Char;       // * End of Input character
    EvtChar           : Char;       // * Received Event character
    wReserved1        : Word;       // * Fill for now.
  end;
  PFTDCB = ^TFTDCB;

  {** Structure for FT_W32_SetCommTimeouts and FT_W32_GetCommTimeouts lpftTimeouts parameter.
   * @see FT_W32_SetCommTimeouts
   * @see FT_W32_GetCommTimeouts }
  TFTTimeouts = packed record
    ReadIntervalTimeout         : DWord;  // * Maximum time between read chars.
    ReadTotalTimeoutMultiplier  : DWord;  // * Multiplier of characters.
    ReadTotalTimeoutConstant    : DWord;  // * Constant in milliseconds.
    WriteTotalTimeoutMultiplier : DWord;  // * Multiplier of characters.
    WriteTotalTimeoutConstant   : DWord;  // * Constant in milliseconds.
  end;
  PFTTimeouts = ^TFTTimeouts;

var

  {** @noop FT_W32_SetCommState
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * This function sets the state of the device according to the contents of a device control block (DCB).
   * @param ftHandle Handle of the device.
   * @param lpftDcb Pointer to an FTDCB structure.
   * @returns
   * If the function is successful, the return value is nonzero.
   * If the function is unsuccessful, the return value is zero. }
  FT_W32_SetCommState: function(Handle: TFTHandle; DCB: PFTDCB): LongBool; FTD2XX_API;

  {** @noop FT_W32_GetCommState
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * This function gets the current device state.
   * @param ftHandle Handle of the device.
   * @param lpftDcb Pointer to an FTDCB structure.
   * @returns
   * If the function is successful, the return value is nonzero.
   * If the function is unsuccessful, the return value is zero.
   * @remarks
   * The current state of the device is returned in a device control block. }
  FT_W32_GetCommState: function(Handle: TFTHandle; DCB: PFTDCB): LongBool; FTD2XX_API;

  {** @noop FT_W32_SetCommTimeouts
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * This function sets the timeout parameters for I/O requests.
   * @param ftHandle Handle of the device.
   * @param pftTimeouts Pointer to an FTTIMEOUTS structure to store timeout information.
   * @returns
   * If the function is unsuccessful, the return value is zero.
   * @remarks
   * Timeouts are calculated using the information in the FTTIMEOUTS structure.
   * @n For read requests, the number of bytes to be read is multiplied by the total timeout
   * multiplier, and added to the total timeout constant. So, if TS is an FTTIMEOUTS structure
   * and the number of bytes to read is dwToRead, the read timeout, rdTO, is calculated as follows.
   * @n rdTO = (dwToRead * TS.ReadTotalTimeoutMultiplier) + TS.ReadTotalTimeoutConstant
   * @n For write requests, the number of bytes to be written is multiplied by the total timeout
   * multiplier, and added to the total timeout constant. So, if TS is an FTTIMEOUTS structure
   * and the number of bytes to write is dwToWrite, the write timeout, wrTO, is calculated as follows.
   * @n wrTO = (dwToWrite * TS.WriteTotalTimeoutMultiplier) + TS.WriteTotalTimeoutConstant
   * @n Linux and Mac OS X currently ignore the ReadIntervalTimeout, ReadTotalTimeoutMultiplier and
   * WriteTotalTimeoutMultiplier. }
  FT_W32_SetCommTimeouts: function(Handle: TFTHandle; Timeouts: PFTTimeouts): LongBool; FTD2XX_API;

  {** @noop FT_W32_GetCommTimeouts
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * This function gets the current read and write request timeout parameters for the specified device.
   * @param ftHandle Handle of the device.
   * @param pftTimeouts Pointer to an FTTIMEOUTS structure to store timeout information.
   * @returns
   * If the function is successful, the return value is nonzero.
   * If the function is unsuccessful, the return value is zero.
   * @remarks
   * For an explanation of how timeouts are used, see FT_W32_SetCommTimeouts. }
  FT_W32_GetCommTimeouts: function(Handle: TFTHandle; Timeouts: PFTTimeouts): LongBool; FTD2XX_API;

  {** @noop FT_W32_SetCommBreak
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * Puts the communications line in the BREAK state.
   * @param ftHandle Handle of the device.
   * @returns
   * If the function is successful, the return value is nonzero.
   * If the function is unsuccessful, the return value is zero. }
  FT_W32_SetCommBreak: function(Handle: TFTHandle): LongBool; FTD2XX_API;

  {** @noop FT_W32_ClearCommBreak
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * Puts the communications line in the non-BREAK state.
   * @param ftHandle Handle of the device.
   * @returns
   * If the function is successful, the return value is nonzero.
   * If the function is unsuccessful, the return value is zero. }
  FT_W32_ClearCommBreak: function(Handle: TFTHandle): LongBool; FTD2XX_API;

  {** @noop FT_W32_SetCommMask
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * This function specifies events that the device has to monitor.
   * @param ftHandle Handle of the device.
   * @param ulEventMask Mask containing events that the device has to monitor. This can be a combination of
   * the following:
   * @li EV_BREAK - BREAK condition detected
   * @li EV_CTS - Change in Clear To Send (CTS)
   * @li EV_DSR - Change in Data Set Ready (DSR)
   * @li EV_ERR - Error in line status
   * @li EV_RING - Change in Ring Indicator (RI)
   * @li EV_RLSD - Change in Receive Line Signal Detect (RLSD)
   * @li EV_RXCHAR - Character received
   * @li EV_RXFLAG - Event character received
   * @li EV_TXEMPTY - Transmitter empty
   * @returns
   * If the function is successful, the return value is nonzero.
   * If the function is unsuccessful, the return value is zero.
   * @remarks
   * This function specifies the events that the device should monitor. An application can call the
   * function FT_W32_WaitCommEvent to wait for an event to occur. }
  FT_W32_SetCommMask: function(Handle: TFTHandle; EventMask: DWord): LongBool; FTD2XX_API;

  {** @noop 6 FT_W32_GetCommMask
   * @par Supported Operating Systems
   * Windows (2000 and later)
   * @par Summary
   * Retrieves the events that are currently being monitored by a device.
   * @param ftHandle Handle of the device.
   * @param lpdwEventMask Pointer to a location that receives a mask that contains the events that are
   * currently enabled. This parameter can be one or more of the following values:
   * @li EV_BREAK - BREAK condition detected
   * @li EV_CTS - Change in Clear To Send (CTS)
   * @li EV_DSR - Change in Data Set Ready (DSR)
   * @li EV_ERR - Error in line status
   * @li EV_RING - Change in Ring Indicator (RI)
   * @li EV_RLSD - Change in Receive Line Signal Detect (RLSD)
   * @li EV_RXCHAR - Character received
   * @li EV_RXFLAG - Event character received
   * @li EV_TXEMPTY - Transmitter empty
   * @returns
   * If the function is successful, the return value is nonzero.
   * If the function is unsuccessful, the return value is zero.
   * @remarks
   * This function returns events currently being monitored by the device. Event monitoring for these
   * events is enabled by the FT_W32_SetCommMask function. }
  FT_W32_GetCommMask: function(Handle: TFTHandle; EventMask: PDWord): LongBool; FTD2XX_API;

  {** @noop FT_W32_WaitCommEvent
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * This function waits for an event to occur.
   * @param ftHandle Handle of the device.
   * @param pulEvent Pointer to a location that receives a mask that contains the events that occurred.
   * @param lpOverlapped Pointer to an overlapped structure.
   * @returns
   * If the function is successful, the return value is nonzero.
   * If the function is unsuccessful, the return value is zero.
   * @remarks
   * This function supports both non-overlapped and overlapped I/O, except under Windows CE and Linux
   * where only non-overlapped IO is supported.
   * @n @b Non-overlapped @b I/O
   * The parameter, lpOverlapped, must be NULL for non-overlapped I/O.
   * @n This function does not return until an event that has been specified in a call to
   * FT_W32_SetCommMask has occurred. The events that occurred and resulted in this function returning
   * are stored in lpdwEvent.
   * @n @b Overlapped @b I/O
   * @n When the device has been opened for overlapped I/O, an application can issue a request and perform
   * some additional work while the request is pending. This contrasts with the case of non-overlapped
   * I/O in which the application issues a request and receives control again only after the request has
   * been completed.
   * @n The parameter, lpOverlapped, must point to an initialized OVERLAPPED structure.
   * This function does not return until an event that has been specified in a call to FT_W32_SetCommMask
   * has occurred.
   * @n If an event has already occurred, the request completes immediately, and the return code is non-zero.
   * @n The events that occurred are stored in lpdwEvent.
   * @n If an event has not yet occurred, the request completes immediately, and the return code is zero,
   * signifying an error. An application should call FT_W32_GetLastError to get the cause of the error. If
   * the error code is ERROR_IO_PENDING, the overlapped operation is still in progress, and the application
   * can perform other processing. Eventually, the application checks the result of the overlapped request
   * by calling FT_W32_GetOverlappedResult. The events that occurred and resulted in this function
   * returning are stored in lpdwEvent. }
  FT_W32_WaitCommEvent: function(Handle: TFTHandle; Event: PLongWord; Overlapped: POverlapped): LongBool; FTD2XX_API;

  {** @noop FT_W32_PurgeComm
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * This function purges the device.
   * @param ftHandle Handle of the device.
   * @param dwMask Specifies the action to take. The action can be a combination of the following:
   * @li PURGE_TXABORT - Terminate outstanding overlapped writes
   * @li PURGE_RXABORT - Terminate outstanding overlapped reads
   * @li PURGE_TXCLEAR - Clear the transmit buffer
   * @li PURGE_RXCLEAR - Clear the receive buffer
   * @returns
   * If the function is successful, the return value is nonzero.
   * If the function is unsuccessful, the return value is zero. }
  FT_W32_PurgeComm: function(Handle: TFTHandle; Mask: DWord): LongBool; FTD2XX_API;

  {** @noop FT_W32_GetLastError
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * Gets the last error that occurred on the device.
   * @param ftHandle Handle of the device.
   * @returns
   * If the function is successful, the return value is nonzero.
   * If the function is unsuccessful, the return value is zero.
   * @remarks
   * This function is normally used with overlapped I/O and so is not supported in Windows CE. For a
   * description of its use, see FT_W32_ReadFile and FT_W32_WriteFile.
   * @n In Linux and Mac OS X, this function returns a DWORD that directly maps to the FT Errors (for
   * example the FT_INVALID_HANDLE error number). }
  FT_W32_GetLastError: function(Handle: TFTHandle): DWord; FTD2XX_API;

  {** @noop FT_W32_ClearCommError
   * @par Supported Operating Systems
   * Linux
   * Mac OS X (10.4 and later)
   * Windows (2000 and later)
   * Windows CE (4.2 and later)
   * @par Summary
   * Gets information about a communications error and get current status of the device.
   * @param ftHandle Handle of the device.
   * @param lpdwErrors Variable that contains the error mask.
   * @param lpftComstat Pointer to FTCOMSTAT structure.
   * @returns
   * If the function is successful, the return value is nonzero.
   * If the function is unsuccessful, the return value is zero. }
  FT_W32_ClearCommError: function(Handle: TFTHandle; Errors: PDWord; Comstat: PFTComStat): LongBool; FTD2XX_API;

  {** @noop FT_W32_CancelIo
   * Undocumented function. }
  FT_W32_CancelIo: function(Handle: TFTHandle): LongBool; FTD2XX_API;

  {........................................................................

   * FT232H Additional EEPROM Functions
   * Undocumented functions.

  .........................................................................}

  FT_EE_ReadConfig: function(Handle: TFTHandle; Address: Byte; Value: PByte): TFTStatus; FTD2XX_API;
  FT_EE_WriteConfig: function(Handle: TFTHandle; Address, Value: Byte): TFTStatus; FTD2XX_API;
  FT_EE_ReadECC: function(Handle: TFTHandle; Option: Byte; Value: PWord): TFTStatus; FTD2XX_API;
  FT_GetQueueStatusEx: function(Handle: TFTHandle; RxBytes: PDWord): TFTStatus; FTD2XX_API;
  FT_ComPortIdle: function(Handle: TFTHandle): TFTStatus; FTD2XX_API;
  FT_ComPortCancelIdle: function(Handle: TFTHandle): TFTStatus; FTD2XX_API;
  FT_VendorCmdGet: function(Handle: TFTHandle; Request: Byte; Buf: PByte; Len: Word): TFTStatus; FTD2XX_API;
  FT_VendorCmdSet: function(Handle: TFTHandle; Request: Byte; Buf: PByte; Len: Word): TFTStatus; FTD2XX_API;
  FT_VendorCmdGetEx: function(Handle: TFTHandle; Value: Word; Buf: PByte; Len: Word): TFTStatus; FTD2XX_API;
  FT_VendorCmdSetEx: function(Handle: TFTHandle; Value: Word; Buf: PByte; Len: Word): TFTStatus; FTD2XX_API;


implementation


const

  // library default name
  LIB_NAME_DEFAULT = 'ftd2xx.' + SharedSuffix;

var
  libFTD:   TLibHandle = NilHandle;           // library handle
  libName:  string     = LIB_NAME_DEFAULT;    // name of the library


//-------------------------------------------------------------------------
//  FT_SUCCESS
//-------------------------------------------------------------------------
function FT_SUCCESS(const status: TFTStatus): boolean;
begin
  Result := status = FT_OK;
end;

//-------------------------------------------------------------------------
//  libFTD_Initialized
//  * Returns True if FTD is loaded and present
//-------------------------------------------------------------------------
function libFTD_Initialized: boolean;
begin
  Result := libFTD <> NilHandle;
end;

//-------------------------------------------------------------------------
//  libFTD_GetHandle
//  * Returns the handle of FTD library.
//-------------------------------------------------------------------------
function libFTD_GetHandle: TLibHandle;
begin
  Result := libFTD;
end;

//-------------------------------------------------------------------------
//  libFTD_GetName
//  - Returns the library's name (without suffix).
//-------------------------------------------------------------------------
function libFTD_GetName: string;
begin
  Result := libName;
end;

//-------------------------------------------------------------------------
//  libFTD_SetHandle
//  - Set the handle of FTD library.
//-------------------------------------------------------------------------
procedure libFTD_SetHandle(libHandle: TLibHandle);
begin
  libFTD := libHandle;
end;

//-------------------------------------------------------------------------
//  FTD_NilLib
//  - Nil all the function variables
//-------------------------------------------------------------------------
procedure FTD_NilLib;
begin

  FT_Open := nil;
  FT_OpenEx := nil;
  FT_ListDevices := nil;
  FT_Close := nil;
  FT_Read := nil;
  FT_Write := nil;
  FT_IoCtl := nil;
  FT_SetBaudRate := nil;
  FT_SetDivisor := nil;
  FT_SetDataCharacteristics := nil;
  FT_SetFlowControl := nil;
  FT_ResetDevice := nil;
  FT_SetDtr := nil;
  FT_ClrDtr := nil;
  FT_SetRts := nil;
  FT_ClrRts := nil;
  FT_GetModemStatus := nil;
  FT_SetChars := nil;
  FT_Purge := nil;
  FT_SetTimeouts := nil;
  FT_GetQueueStatus := nil;
  FT_SetEventNotification := nil;
  FT_GetStatus := nil;
  FT_SetBreakOn := nil;
  FT_SetBreakOff := nil;
  FT_SetWaitMask := nil;
  FT_WaitOnMask := nil;
  FT_GetEventStatus := nil;
  FT_ReadEE := nil;
  FT_WriteEE := nil;
  FT_EraseEE := nil;

  FT_EE_Program := nil;
  FT_EE_ProgramEx := nil;
  FT_EE_Read := nil;
  FT_EE_ReadEx := nil;
  FT_EE_UASize := nil;
  FT_EE_UAWrite := nil;
  FT_EE_UARead := nil;

  FT_EEPROM_Read := nil;
  FT_EEPROM_Program := nil;
  FT_SetLatencyTimer := nil;
  FT_GetLatencyTimer := nil;
  FT_SetBitMode := nil;
  FT_GetBitMode := nil;
  FT_SetUSBParameters := nil;
  FT_SetDeadmanTimeout := nil;

  FT_GetDeviceInfo := nil;
  FT_StopInTask := nil;
  FT_RestartInTask := nil;

  FT_W32_CreateFile := nil;
  FT_W32_CloseHandle := nil;
  FT_W32_ReadFile := nil;
  FT_W32_WriteFile := nil;
  FT_W32_GetLastError := nil;
  FT_W32_GetOverlappedResult := nil;
  FT_W32_CancelIo := nil;

  FT_W32_ClearCommBreak := nil;
  FT_W32_ClearCommError := nil;
  FT_W32_EscapeCommFunction := nil;
  FT_W32_GetCommModemStatus := nil;
  FT_W32_GetCommState := nil;
  FT_W32_GetCommTimeouts := nil;
  FT_W32_PurgeComm := nil;
  FT_W32_SetCommBreak := nil;
  FT_W32_SetCommMask := nil;
  FT_W32_GetCommMask := nil;
  FT_W32_SetCommState := nil;
  FT_W32_SetCommTimeouts := nil;
  FT_W32_SetupComm := nil;
  FT_W32_WaitCommEvent := nil;

  FT_CreateDeviceInfoList := nil;
  FT_GetDeviceInfoList := nil;
  FT_GetDeviceInfoDetail := nil;

  FT_GetLibraryVersion := nil;

  FT_EE_ReadConfig := nil;
  FT_EE_WriteConfig := nil;
  FT_EE_ReadECC := nil;
  FT_GetQueueStatusEx := nil;
  FT_ComPortIdle := nil;
  FT_ComPortCancelIdle := nil;
  FT_VendorCmdGet := nil;
  FT_VendorCmdSet := nil;
  FT_VendorCmdGetEx := nil;
  FT_VendorCmdSetEx := nil;

  {$IFDEF WINDOWS}
  FT_GetDriverVersion := nil;
  FT_GetComPortNumber := nil;
  FT_ResetPort := nil;
  FT_CyclePort := nil;
  FT_Rescan := nil;
  FT_Reload := nil;
  FT_SetResetPipeRetryCount := nil;
  {$ENDIF WINDOWS}

  {$IFNDEF WINDOWS}
  FT_SetVIDPID := nil;
  FT_GetVIDPID := nil;
  FT_GetDeviceLocId := nil;
  {$ENDIF WINDOWS}

end;

//-------------------------------------------------------------------------
//  FTD_LoadLib
//  - Set the function variables (if available)
//-------------------------------------------------------------------------
procedure FTD_LoadLib;

  procedure _initProc(var procPointer; procName: PChar);
  begin
    pointer(procPointer) := dynlibs.GetProcAddress(libFTD, procName);
  end;

begin

  _initProc(FT_Open, 'FT_Open');
  _initProc(FT_OpenEx, 'FT_OpenEx');
  _initProc(FT_ListDevices, 'FT_ListDevices');
  _initProc(FT_Close, 'FT_Close');
  _initProc(FT_Read, 'FT_Read');
  _initProc(FT_Write, 'FT_Write');
  _initProc(FT_IoCtl, 'FT_IoCtl');
  _initProc(FT_SetBaudRate, 'FT_SetBaudRate');
  _initProc(FT_SetDivisor, 'FT_SetDivisor');
  _initProc(FT_SetDataCharacteristics, 'FT_SetDataCharacteristics');
  _initProc(FT_SetFlowControl, 'FT_SetFlowControl');
  _initProc(FT_ResetDevice, 'FT_ResetDevice');
  _initProc(FT_SetDtr, 'FT_SetDtr');
  _initProc(FT_ClrDtr, 'FT_ClrDtr');
  _initProc(FT_SetRts, 'FT_SetRts');
  _initProc(FT_ClrRts, 'FT_ClrRts');
  _initProc(FT_GetModemStatus, 'FT_GetModemStatus');
  _initProc(FT_SetChars, 'FT_SetChars');
  _initProc(FT_Purge, 'FT_Purge');
  _initProc(FT_SetTimeouts, 'FT_SetTimeouts');
  _initProc(FT_GetQueueStatus, 'FT_GetQueueStatus');
  _initProc(FT_SetEventNotification, 'FT_SetEventNotification');
  _initProc(FT_GetStatus, 'FT_GetStatus');
  _initProc(FT_SetBreakOn, 'FT_SetBreakOn');
  _initProc(FT_SetBreakOff, 'FT_SetBreakOff');
  _initProc(FT_SetWaitMask, 'FT_SetWaitMask');
  _initProc(FT_WaitOnMask, 'FT_WaitOnMask');
  _initProc(FT_GetEventStatus, 'FT_GetEventStatus');
  _initProc(FT_ReadEE, 'FT_ReadEE');
  _initProc(FT_WriteEE, 'FT_WriteEE');
  _initProc(FT_EraseEE, 'FT_EraseEE');

  _initProc(FT_EE_Program, 'FT_EE_Program');
  _initProc(FT_EE_ProgramEx, 'FT_EE_ProgramEx');
  _initProc(FT_EE_Read, 'FT_EE_Read');
  _initProc(FT_EE_ReadEx, 'FT_EE_ReadEx');
  _initProc(FT_EE_UASize, 'FT_EE_UASize');
  _initProc(FT_EE_UAWrite, 'FT_EE_UAWrite');
  _initProc(FT_EE_UARead, 'FT_EE_UARead');

  _initProc(FT_EEPROM_Read, 'FT_EEPROM_Read');
  _initProc(FT_EEPROM_Program, 'FT_EEPROM_Program');
  _initProc(FT_SetLatencyTimer, 'FT_SetLatencyTimer');
  _initProc(FT_GetLatencyTimer, 'FT_GetLatencyTimer');
  _initProc(FT_SetBitMode, 'FT_SetBitMode');
  _initProc(FT_GetBitMode, 'FT_GetBitMode');
  _initProc(FT_SetUSBParameters, 'FT_SetUSBParameters');
  _initProc(FT_SetDeadmanTimeout, 'FT_SetDeadmanTimeout');

  _initProc(FT_GetDeviceInfo, 'FT_GetDeviceInfo');
  _initProc(FT_StopInTask, 'FT_StopInTask');
  _initProc(FT_RestartInTask, 'FT_RestartInTask');

  _initProc(FT_W32_CreateFile, 'FT_W32_CreateFile');
  _initProc(FT_W32_CloseHandle, 'FT_W32_CloseHandle');
  _initProc(FT_W32_ReadFile, 'FT_W32_ReadFile');
  _initProc(FT_W32_WriteFile, 'FT_W32_WriteFile');
  _initProc(FT_W32_GetLastError, 'FT_W32_GetLastError');
  _initProc(FT_W32_GetOverlappedResult, 'FT_W32_GetOverlappedResult');
  _initProc(FT_W32_CancelIo, 'FT_W32_CancelIo');

  _initProc(FT_W32_ClearCommBreak, 'FT_W32_ClearCommBreak');
  _initProc(FT_W32_ClearCommError, 'FT_W32_ClearCommError');
  _initProc(FT_W32_EscapeCommFunction, 'FT_W32_EscapeCommFunction');
  _initProc(FT_W32_GetCommModemStatus, 'FT_W32_GetCommModemStatus');
  _initProc(FT_W32_GetCommState, 'FT_W32_GetCommState');
  _initProc(FT_W32_GetCommTimeouts, 'FT_W32_GetCommTimeouts');
  _initProc(FT_W32_PurgeComm, 'FT_W32_PurgeComm');
  _initProc(FT_W32_SetCommBreak, 'FT_W32_SetCommBreak');
  _initProc(FT_W32_SetCommMask, 'FT_W32_SetCommMask');
  _initProc(FT_W32_GetCommMask, 'FT_W32_GetCommMask');
  _initProc(FT_W32_SetCommState, 'FT_W32_SetCommState');
  _initProc(FT_W32_SetCommTimeouts, 'FT_W32_SetCommTimeouts');
  _initProc(FT_W32_SetupComm, 'FT_W32_SetupComm');
  _initProc(FT_W32_WaitCommEvent, 'FT_W32_WaitCommEvent');

  _initProc(FT_CreateDeviceInfoList, 'FT_CreateDeviceInfoList');
  _initProc(FT_GetDeviceInfoList, 'FT_GetDeviceInfoList');
  _initProc(FT_GetDeviceInfoDetail, 'FT_GetDeviceInfoDetail');

  _initProc(FT_GetLibraryVersion, 'FT_GetLibraryVersion');

  _initProc(FT_EE_ReadConfig, 'FT_EE_ReadConfig');
  _initProc(FT_EE_WriteConfig, 'FT_EE_WriteConfig');
  _initProc(FT_EE_ReadECC, 'FT_EE_ReadECC');
  _initProc(FT_GetQueueStatusEx, 'FT_GetQueueStatusEx');
  _initProc(FT_ComPortIdle, 'FT_ComPortIdle');
  _initProc(FT_ComPortCancelIdle, 'FT_ComPortCancelIdle');
  _initProc(FT_VendorCmdGet, 'FT_VendorCmdGet');
  _initProc(FT_VendorCmdSet, 'FT_VendorCmdSet');
  _initProc(FT_VendorCmdGetEx, 'FT_VendorCmdGetEx');
  _initProc(FT_VendorCmdSetEx, 'FT_VendorCmdSetEx');

  {$IFDEF WINDOWS}
  _initProc(FT_GetDriverVersion, 'FT_GetDriverVersion');
  _initProc(FT_GetComPortNumber, 'FT_GetComPortNumber');
  _initProc(FT_ResetPort, 'FT_ResetPort');
  _initProc(FT_CyclePort, 'FT_CyclePort');
  _initProc(FT_Rescan, 'FT_Rescan');
  _initProc(FT_Reload, 'FT_Reload');
  _initProc(FT_SetResetPipeRetryCount, 'FT_SetResetPipeRetryCount');
  {$ENDIF WINDOWS}

  {$IFNDEF WINDOWS}
  _initProc(FT_SetVIDPID, 'FT_SetVIDPID');
  _initProc(FT_GetVIDPID, 'FT_GetVIDPID');
  _initProc(FT_GetDeviceLocId, 'FT_GetDeviceLocId');
  {$ENDIF WINDOWS}

end;

//-------------------------------------------------------------------------
//  libFTD_Initialize
//  - Load and Initialize the FTD library.
//  - Called automatically from Initialization section (if defined).
//  - Returns True if success
//-------------------------------------------------------------------------
function libFTD_Initialize: boolean;
begin

  try

    if libFTD = NilHandle then begin
      FTD_NilLib;
      libFTD := dynlibs.LoadLibrary(libName);
    end;

    if libFTD <> NilHandle
      then FTD_LoadLib;

  finally
    Result := libFTD <> NilHandle;
  end;

end;

//-------------------------------------------------------------------------
//  libFTD_Finalize
//  - Unloads the library
//  - Called automatically from unit's Finalization section (if defined).
//-------------------------------------------------------------------------
procedure libFTD_Finalize;
begin

  if libFTD <> NilHandle then begin
    dynlibs.FreeLibrary(libFTD);
    libFTD := NilHandle;
    FTD_NilLib;
  end;

end;

{$IFDEF FT_AutoInitialize}  // <--- see FTD.inc

//===========================================================================
//  initialization
//===========================================================================
initialization
begin
  libFTD_Initialize;
end;

//===========================================================================
//  finalization
//===========================================================================
finalization
begin
  libFTD_Finalize;
end;

{$ENDIF FT_AutoInitialize}

{$ELSE FTD2XX_H}

  {$WARNING FTD2XX_H allready defined}

implementation

{$ENDIF FTD2XX_H}


end.



