unit ftd2xx_helpers;

{-------------------------------------------------------------------------

  ftd2xx_helpers.pas
  - Part of ftd2xx-FPC package.

  * https://github.com/tednilsen/ftd2xx-FPC

-------------------------------------------------------------------------}

{$mode objfpc}{$H+}
{$INLINE ON}
{$MACRO ON}
{$MODESWITCH AdvancedRecords}
{$MODESWITCH TypeHelpers}

interface

uses
  ftd2xx;

type

  //==========================================================
  //  TFTStatus_Helper
  //==========================================================
  TFTStatus_Helper = type helper for TFTStatus
    function    ToString: string; inline;
	end;

  //==========================================================
  //  TFTDevice_Helper
  //==========================================================
  TFTDevice_Helper = type helper for TFTDevice
    function    ToString: string; inline;
	end;

implementation


//==========================================================
//  TFTDeviceStatus_Helper
//==========================================================
function TFTStatus_Helper.ToString: string;
begin
  case Self of
    FT_OK:                          Result := 'Ok.';
    FT_INVALID_HANDLE:              Result := 'Invalid handle.';
    FT_DEVICE_NOT_FOUND:            Result := 'Device not found.';
    FT_DEVICE_NOT_OPENED:           Result := 'Device not opened.';
    FT_IO_ERROR:                    Result := 'General IO error.';
    FT_INSUFFICIENT_RESOURCES:      Result := 'Insufficient resources.';
    FT_INVALID_PARAMETER:           Result := 'Invalid parameter.';
    FT_INVALID_BAUD_RATE:           Result := 'Invalid baud rate.';
  	FT_DEVICE_NOT_OPENED_FOR_ERASE: Result := 'Device not opened for erase.';
    FT_DEVICE_NOT_OPENED_FOR_WRITE: Result := 'Device not opened for write.';
    FT_FAILED_TO_WRITE_DEVICE:      Result := 'Failed to write to device.';
    FT_EEPROM_READ_FAILED:          Result := 'EEPROM read failed.';
    FT_EEPROM_WRITE_FAILED:         Result := 'EEPROM write failed.';
    FT_EEPROM_ERASE_FAILED:         Result := 'EEPROM erase failed.';
    FT_EEPROM_NOT_PRESENT:          Result := 'EEPROM not present.';
    FT_EEPROM_NOT_PROGRAMMED:       Result := 'EEPROM not programmed.';
    FT_INVALID_ARGS:                Result := 'Invalid arguments.';
    FT_NOT_SUPPORTED:               Result := 'Not supported.';
    FT_OTHER_ERROR:                 Result := 'Other error.';
    FT_DEVICE_LIST_NOT_READY:       Result := 'Device list not ready.';
    else                            Result := 'Unknown.';
  end;
end;

//==========================================================
//  TFTDevice_Helper
//==========================================================
function TFTDevice_Helper.ToString: string;
begin
  case Self of
    FT_DEVICE_BM:        Result := 'BM';
    FT_DEVICE_AM:        Result := 'AM';
    FT_DEVICE_100AX:     Result := '100AX';
    FT_DEVICE_UNKNOWN:   Result := 'UNKNOWN';
    FT_DEVICE_2232C:     Result := '2232C';
    FT_DEVICE_232R:      Result := '232R';
    FT_DEVICE_2232H:     Result := '2232H';
    FT_DEVICE_4232H:     Result := '4232H';
    FT_DEVICE_232H:      Result := '232H';
    FT_DEVICE_X_SERIES:  Result := 'X-Series';
    FT_DEVICE_4222H_0:   Result := '4222H 0';
    FT_DEVICE_4222H_1_2: Result := '4222H 1 2';
    FT_DEVICE_4222H_3:   Result := '4222H 3';
    FT_DEVICE_4222_PROG: Result := '4222 PROG';
    FT_DEVICE_900:       Result := '900';
    FT_DEVICE_930:       Result := '930';
    FT_DEVICE_UMFTPD3A:  Result := 'UMFTPD3A';
    FT_DEVICE_2233HP:    Result := '2233HP';
    FT_DEVICE_4233HP:    Result := '4233HP';
    FT_DEVICE_2232HP:    Result := '2232HP';
    FT_DEVICE_4232HP:    Result := '4232HP';
    FT_DEVICE_233HP:     Result := '233HP';
    FT_DEVICE_232HP:     Result := '232HP';
    FT_DEVICE_2232HA:    Result := '2232HA';
    FT_DEVICE_4232HA:    Result := '4232HA';
    else                 Result := 'Unknown';
  end;
end;


end.

