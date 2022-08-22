//-------------------------------------------------------------------------
//
//  ftd2xx.inc
//
//-------------------------------------------------------------------------

{$INLINE ON}
{$MACRO ON}
{$IFDEF FPC}
  {$PACKRECORDS C}
{$ENDIF}

// define to automatic initialize and finalize the unit
{$DEFINE FT_AutoInitialize}

//-------------------------------------------------------------------------
// ...automated settings, no need to edit!
//-------------------------------------------------------------------------

{$IFDEF Windows}
  {$DEFINE FTD2XX_API := stdcall}
{$ELSE}
  {$DEFINE FTD2XX_API := cdecl}
{$ENDIF}

