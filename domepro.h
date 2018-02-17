//
//  DomePro.h
//
//  Created by Rodolphe Pineau on 6/11/2016.
//  Rigel rotation drive unit for Pulsar Dome X2 plugin

#ifndef __RIGEL_DOME__
#define __RIGEL_DOME__
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <ctype.h>
#include <memory.h>
#include <string.h>
#include <time.h>
#ifdef SB_MAC_BUILD
#include <unistd.h>
#endif


#include <string>
#include <vector>
#include <sstream>
#include <iostream>

#include "../../licensedinterfaces/sberrorx.h"
#include "../../licensedinterfaces/serxinterface.h"
#include "../../licensedinterfaces/loggerinterface.h"

#define FILE_DEBUG 1   // define this to have log files


#ifdef FILE_DEBUG
#if defined(SB_WIN_BUILD)
#define LOGFILENAME "C:\\DomePro_X2.log"
#elif defined(SB_LINUX_BUILD)
#define LOGFILENAME "/tmp/DomePro_X2.log"
#elif defined(SB_MAC_BUILD)
#define LOGFILENAME "/tmp/DomePro_X2.log"
#endif
#endif


#define SERIAL_BUFFER_SIZE 256
#define MAX_TIMEOUT 5000
#define DP2_LOG_BUFFER_SIZE 256

/// ATCL response code
#define ATCL_ACK	0x8F
#define ATCL_NACK	0xA5

// some DomePro2 value definitions
#define CLASSIC_DOME 0x0D
#define CLAMSHELL    0x0E
#define ROR          0x0F

enum DomePro2_Module {MODULE_AZ = 0, MODULE_SHUT, MODULE_UKNOWN};
enum DomePro2_Motor {ON_OFF = 0, STEP_DIR, MOTOR_UNKNOWN};
enum DomePro2_Polarity {POSITIVE = 0, NEGATIVE, POLARITY_UKNOWN};
enum DomeAzMoveMode {FIXED = 0, LEFT, RIGHT, GOTO, HOMING, AZ_TO};

enum DomeProErrors {DP2_OK=0, NOT_CONNECTED, DP2_CANT_CONNECT, DP2_BAD_CMD_RESPONSE, COMMAND_FAILED};

enum DomeProShutterState {OPEN=0, CLOSED, OPENING, CLOSING, SHUTTER_ERROR, NO_COM,
                        SHUT1_OPEN_TO, SHUT1_CLOSE_TO, SHUT2_OPEN_TO, SHUT2_CLOSE_TO,
                        SHUT1_OPEN_COMPL_TO, SHUT1_CLOSE_COMPL_TO,
                        SHUT2_OPEN_COMPL_TO, SHUT2_CLOSE_COMPL_TO,
                        NOT_FITTED, INTERMEDIATE, SHUT_GOTO
                        };

enum    SwitchState { INNACTIVE = 0, ACTIVE};

class CDomePro
{
public:
    CDomePro();
    ~CDomePro();

    int     Connect(const char *szPort);
    void    Disconnect(void);
    bool    IsConnected(void) { return m_bIsConnected; }

    void    SetSerxPointer(SerXInterface *p) { m_pSerx = p; }
    void    setLogger(LoggerInterface *pLogger) { m_pLogger = pLogger; };

    // Dome movement commands
    int syncDome(double dAz, double dEl);

    int gotoDomePark(void);

    int unparkDome(void);
    int gotoAzimuth(double newAz);
    int openDomeShutters();
    int CloseDomeShutters();
    int abortCurrentCommand();
    int goHome();
    int calibrate();

    // Dome informations
    int getFirmwareVersion(char *version, int strMaxLen);
    int getModel(char *model, int strMaxLen);
    int getModuleType(int &nModuleType);
    int getDomeAzMotorType(int &nMotorType);

    int setDomeAzMotorPolarity(int nPolarity);
    int getDomeAzMotorPolarity(int &nPolarity);

    int setDomeAzEncoderPolarity(int nPolarity);
    int getDomeAzEncoderPolarity(int &nPolarity);
    
    bool hasShutterUnit();

    // command complete functions
    int isGoToComplete(bool &complete);
    int isOpenComplete(bool &complete);
    int isCloseComplete(bool &complete);
    int isParkComplete(bool &complete);
    int isUnparkComplete(bool &complete);
    int isFindHomeComplete(bool &complete);
    int isCalibratingComplete(bool &complete);


    // getter/setter
    int getDomeHomeAz(double &dAz);
    int setHomeAz(double dAz);

    int getDomeAzCoast(double &dAz);

    int setParkAz(double dAz);

    double getCurrentAz();
    double getCurrentEl();

    int getCurrentShutterState();

    void setDebugLog(bool enable);

    // dome states
    int getDomeAzPosition(double &dDomeAz);
    int getDomeEl(double &dDomeEl);


    int getDomeParkAz(double &dAz);

    int             getDomeShutterStatus(int &nState);

    int             setDomeAzCPR(int nValue);
    int             getDomeAzCPR(int &nValue);

    // controller low level data
    int             getDomeSupplyVoltageAzimuthL(double &dVolts);
    int             getDomeSupplyVoltageShutterL(double &dVolts);
    int             getDomeSupplyVoltageAzimuthM(double &dVolts);
    int             getDomeSupplyVoltageShutterM(double &dVolts);
    // not yet implemented in the firmware
    int             getDomeRotationSenseAnalog(double &dVolts);
    //
    int             setDomeShutter1_OpTimeOut(int nTimeout);
    int             getDomeShutter1_OpTimeOut(int &nTimeout);
    int             setDomeShutter2_OpTimeOut(int nTimeout);
    int             getDomeShutter2_OpTimeOut(int &nTimeout);

    int             setDomeShutODirTimeOut(int nTimeout);
    int             getDomeShutODirTimeOut(int &nTimeout);

    int             setDomeAzimuthTimeOutEnabled(bool bEnable);
    int             getDomeAzimuthTimeOutEnabled(bool &bEnable);
    int             setDomeAzimuthTimeOut(int nTimeout);
    int             getDomeAzimuthTimeOut(int &nTimeout);
    int             setDomeShutCloseOnLinkTimeOut(bool bEnable);
    int             getDomeShutCloseOnLinkTimeOut(bool &bEnable);
    int             setDomeShutCloseOnClientTimeOut(bool bEnable);
    int             getDomeShutCloseOnClientTimeOut(bool &bEnable);
    int             setDomeShutCloseClientTimeOut(int nTimeout);
    int             getDomeShutCloseClientTimeOut(int &nTimeout);

    // not yet implemented in the firmware
    int             setDomeShutOpAtHome(bool bEnable);
    int             getDomeShutOpAtHome(bool &bEnable);
    //

    int             getDomeShutdownInputState(bool &bEnable);
    int             getDomePowerGoodInputState(bool &bEnable);

    // not yet implemented in the firmware
    int             getLastDomeShutdownEvent(void);
    //
    int             setDomeSingleShutterMode(bool bEnable);
    int             getDomeSingleShutterMode(bool &bEnable);

    int             getDomeLinkErrCnt(int &nErrCnt);
    int             clearDomeLinkErrCnt(void);

    // not yet implemented in the firmware
    int             getDomeComErr(void);
    int             clearDomeComErr(void);
    //
    int             setDomeShutterOpenFirst(int nShutter);
    int             getDomeShutterOpenFirst(int &nShutter);
    int             setDomeShutterCloseFirst(int nShutter);
    int             getDomeShutterCloseFirst(int &nShutter);

    int             getDomeShutterMotorADC(double &dVolts);
    int             getDomeAzimuthMotorADC(double &dVolts);
    int             getDomeShutterTempADC(double &dTemp);
    int             getDomeAzimuthTempADC(double &dTemp);

protected:

    int             domeCommand(const char *pszCmd, char *pszResult, int nResultMaxLen);
    int             readResponse(unsigned char *pszRespBuffer, int bufferLen);

    // conversion functions
    void AzToTicks(double pdAz, int &ticks);
    void TicksToAz(int ticks, double &pdAz);

    // movements
    int             setDomeLeftOn(void);
    int             setDomeRightOn(void);
    int             killDomeAzimuthMovement(void);

    // command completion/state
    int             isDomeMoving(bool &bIsMoving);
    int             isDomeAtHome(bool &bAtHome);

    // DomePro getter / setter

    // not yet implemented in the firmware
    int             setDomeMaxVel(int nValue);
    int             getDomeMaxVel(int &nValue);
    int             setDomeAccel(int nValue);
    int             getDomeAccel(int &nValue);
    //

    int             setDomeAzCoast(int nValue);
    int             getDomeAzCoast(int &nValue);
    int             getDomeAzDiagPosition(int &nValue);
    int             clearDomeAzDiagPosition(void);
    int             getDomeAzMoveMode(int &mode);

    int             getDomeLimits(void);

    int             setDomeHomeAzimuth(int nPos);
    int             getDomeHomeAzimuth(int &nPos);
    int             homeDomeAzimuth(void);
    int             goToDomeAzimuth(int nPos);

    int             setDomeParkAzimuth(int nPos);
    int             getDomeParkAzimuth(int &nPos);
    int             calibrateDomeAzimuth(int nPos);

    int             killDomeShutterMovement(void);

    int             getDomeDebug(char *pszDebugStrBuff, int nStrMaxLen);

    int             openDomeShutter1(void);
    int             openDomeShutter2(void);
    int             closeDomeShutter1(void);
    int             closeDomeShutter2(void);
    int             stopDomeShutter1(void);
    int             stopDomeShutter2(void);
    int             goToDomeShutter1_ADC(int nPos);
    int             goToDomeShutter2_ADC(int nPos);
    int             getDomeShutter1_AltitudeADC(int &nPos);
    int             getDomeShutter2_AltitudeADC(int &nPos);

    void            hexdump(const char *inputData, char *outBuffer, int size);
    
    // protected variables
    LoggerInterface *m_pLogger;
    bool            m_bDebugLog;

    bool            m_bIsConnected;
    bool            m_bHomed;
    bool            m_bParked;
    bool            m_bCalibrating;

    int             m_nNbStepPerRev;
    double          m_dHomeAz;

    double          m_dParkAz;

    double          m_dCurrentAzPosition;
    double          m_dCurrentElPosition;

    double          m_dGotoAz;

    SerXInterface   *m_pSerx;

    char            m_szFirmwareVersion[SERIAL_BUFFER_SIZE];
    int             m_nShutterState;
    bool            m_bHasShutter;
    bool            m_bShutterOpened;

    char            m_szLogBuffer[DP2_LOG_BUFFER_SIZE];
    int             m_nModel;
    int             m_nModuleType;
    int             m_nMotorType;
    int             m_nMotorPolarity;
    int             m_nAzEncoderPolarity;

    int             m_nShutter1OpenedSwitchState;
    int             m_nShutter1ClosedSwitchState;
    int             m_nShutter2OpenedSwitchState;
    int             m_nShutter2ClosedSwitchState;
    int             m_nAtHomeState;
    int             m_nAtHomeSwitchState;
    int             m_nAtParkSate;

    char            m_hexdumpBuffer[(SERIAL_BUFFER_SIZE*3)+1];

#ifdef FILE_DEBUG
    // timestamp for logs
    char *timestamp;
    time_t ltime;
    FILE *Logfile;	  // LogFile
#endif

};

#endif
