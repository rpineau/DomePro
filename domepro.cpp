//
//  DomePro.cpp
//  Rigel rotation drive unit for Pulsar Dome X2 plugin
//
//  Created by Rodolphe Pineau on 6/11/2016.


#include "domepro.h"
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <memory.h>
#ifdef SB_MAC_BUILD
#include <unistd.h>
#endif

CDomePro::CDomePro()
{
    // set some sane values
    m_bDebugLog = true;

    m_pSerx = NULL;
    m_bIsConnected = false;

    m_nNbStepPerRev = 0;
    m_dShutterBatteryVolts = 0.0;

    m_dHomeAz = 180;
    m_dParkAz = 180;

    m_dCurrentAzPosition = 0.0;
    m_dCurrentElPosition = 0.0;

    m_bCalibrating = false;

    m_bHasShutter = false;
    m_bShutterOpened = false;

    m_bParked = true;
    m_bHomed = false;
    memset(m_szFirmwareVersion,0,SERIAL_BUFFER_SIZE);
    memset(m_szLogBuffer,0,ND_LOG_BUFFER_SIZE);
}

CDomePro::~CDomePro()
{

}

int CDomePro::Connect(const char *pszPort)
{
    int nErr;
    int nState;

    // 9600 8N1
    if(m_pSerx->open(pszPort, 19200, SerXInterface::B_NOPARITY, "-DTR_CONTROL 1") == 0)
        m_bIsConnected = true;
    else
        m_bIsConnected = false;

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    if (m_bDebugLog) {
        snprintf(m_szLogBuffer,ND_LOG_BUFFER_SIZE,"[CDomePro::Connect] Connected.\n");
        m_pLogger->out(m_szLogBuffer);

        snprintf(m_szLogBuffer,ND_LOG_BUFFER_SIZE,"[CDomePro::Connect] Getting Firmware.\n");
        m_pLogger->out(m_szLogBuffer);
    }
    // if this fails we're not properly connected.
    nErr = getFirmwareVersion(m_szFirmwareVersion, SERIAL_BUFFER_SIZE);
    if(nErr) {
        if (m_bDebugLog) {
            snprintf(m_szLogBuffer,ND_LOG_BUFFER_SIZE,"[CDomePro::Connect] Error Getting Firmware.\n");
            m_pLogger->out(m_szLogBuffer);
        }
        m_bIsConnected = false;
        m_pSerx->close();
        return FIRMWARE_NOT_SUPPORTED;
    }

    if (m_bDebugLog) {
        snprintf(m_szLogBuffer,ND_LOG_BUFFER_SIZE,"[CDomePro::Connect] Got Firmware.\n");
        m_pLogger->out(m_szLogBuffer);
    }
    // assume the dome was parked
    getDomeParkAz(m_dCurrentAzPosition);

    syncDome(m_dCurrentAzPosition,m_dCurrentElPosition);
    nErr = getShutterState(nState);

    if(nState != NOT_FITTED && nState != UNKNOWN )
        m_bHasShutter = true;

    return SB_OK;
}


void CDomePro::Disconnect()
{
    if(m_bIsConnected) {
        m_pSerx->purgeTxRx();
        m_pSerx->close();
    }
    m_bIsConnected = false;
}


int CDomePro::readResponse(unsigned char *pszRespBuffer, int nBufferLen)
{
    int nErr = PD_OK;
    unsigned long ulBytesRead = 0;
    unsigned long ulTotalBytesRead = 0;
    unsigned char *pszBufPtr;

    memset(pszRespBuffer, 0, (size_t) nBufferLen);
    pszBufPtr = pszRespBuffer;

    do {
        nErr = m_pSerx->readFile(pszBufPtr, 1, ulBytesRead, MAX_TIMEOUT);
        if(nErr) {
            if (m_bDebugLog) {
                snprintf(m_szLogBuffer,ND_LOG_BUFFER_SIZE,"[CDomePro::readResponse] readFile error.\n");
                m_pLogger->out(m_szLogBuffer);
            }
            return nErr;
        }

        if (m_bDebugLog) {
            snprintf(m_szLogBuffer,ND_LOG_BUFFER_SIZE,"[CDomePro::readResponse] respBuffer = %s\n",pszRespBuffer);
            m_pLogger->out(m_szLogBuffer);
        }

        if (ulBytesRead !=1) {// timeout
            if (m_bDebugLog) {
                snprintf(m_szLogBuffer,ND_LOG_BUFFER_SIZE,"[CDomePro::readResponse] readFile Timeout.\n");
                m_pLogger->out(m_szLogBuffer);
            }
            nErr = PD_BAD_CMD_RESPONSE;
            break;
        }
        ulTotalBytesRead += ulBytesRead;
        if (m_bDebugLog) {
            snprintf(m_szLogBuffer,ND_LOG_BUFFER_SIZE,"[CDomePro::readResponse] nBytesRead = %lu\n",ulBytesRead);
            m_pLogger->out(m_szLogBuffer);
        }
        // check for  errors or single ACK
        if(*pszBufPtr == ATCL_NACK) {
            nErr = PD_BAD_CMD_RESPONSE;
            break;
        }

        if(*pszBufPtr == ATCL_ACK) {
            nErr = PD_OK;
        	break;
        }


    } while (*pszBufPtr++ != ';' && ulTotalBytesRead < nBufferLen );

    if(ulTotalBytesRead && *(pszBufPtr-1) == ';')
        *(pszBufPtr-1) = 0; //remove the ; to zero terminate the string

    return nErr;
}


int CDomePro::domeCommand(const char *pszCmd, char *pszResult, int nResultMaxLen)
{
    int nErr = PD_OK;
    unsigned char szResp[SERIAL_BUFFER_SIZE];
    unsigned long ulBytesWrite;

    m_pSerx->purgeTxRx();
    if (m_bDebugLog) {
        snprintf(m_szLogBuffer,ND_LOG_BUFFER_SIZE,"[CDomePro::domeCommand] Sending %s\n",pszCmd);
        m_pLogger->out(m_szLogBuffer);
    }
    nErr = m_pSerx->writeFile((void *)pszCmd, strlen(pszCmd), ulBytesWrite);
    m_pSerx->flushTx();
    if(nErr)
        return nErr;
    // read response
    if (m_bDebugLog) {
        snprintf(m_szLogBuffer,ND_LOG_BUFFER_SIZE,"[CDomePro::domeCommand] Getting response.\n");
        m_pLogger->out(m_szLogBuffer);
    }
    nErr = readResponse(szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    if(pszResult)
        strncpy(pszResult, (const char *)szResp, nResultMaxLen);

    return nErr;

}

int CDomePro::getDomeAz(double &dDomeAz)
{
    int nErr = PD_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    unsigned long nTmp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return nErr;

    nErr = domeCommand("!DGap;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // convert Az hex string to long
    nTmp = strtol(szResp, NULL, 16);

    dDomeAz = nTmp/(m_nNbStepPerRev * 360.0);

    m_dCurrentAzPosition = dDomeAz;

    return nErr;
}

int CDomePro::getDomeEl(double &dDomeEl)
{
    int nErr = PD_OK;
    int nShutterState;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    getShutterState(nShutterState);

    if(!m_bShutterOpened || !m_bHasShutter)
    {
        dDomeEl = 0.0;
    }
    else {
        dDomeEl = 90.0;
    }

    m_dCurrentElPosition = dDomeEl;

    return nErr;
}


int CDomePro::getDomeHomeAz(double &dAz)
{
    int nErr = PD_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    unsigned long nTmp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return nErr;

    nErr = domeCommand("!DGha;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // convert Az hex string to long
    nTmp = strtol(szResp, NULL, 16);

    dAz = nTmp/(m_nNbStepPerRev * 360.0);

    m_dHomeAz = dAz;
    return nErr;
}

int CDomePro::getDomeParkAz(double &dAz)
{
    int nErr;
    nErr = getDomeAz(dAz);
    return nErr;
}


int CDomePro::getShutterState(int &nState)
{
    int nErr = PD_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    int nShutterState;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return nErr;

    nErr = domeCommand("SHUTTER\r", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    nErr = domeCommand("SHUTTER\r", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

	nShutterState = atoi(szResp);
    switch(nShutterState) {
        case OPEN:
            m_bShutterOpened = true;
            break;

        case CLOSED:
            m_bShutterOpened = false;
            break;

        case NOT_FITTED:
            m_bShutterOpened = false;
            m_bHasShutter = false;
            break;
        default:
            m_bShutterOpened = false;

    }

    nState = atoi(szResp);

    return nErr;
}


int CDomePro::getDomeStepPerRev(int &nStepPerRev)
{
    int nErr = PD_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = domeCommand("ENCREV\r", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    nStepPerRev = atoi(szResp);
    m_nNbStepPerRev = nStepPerRev;
    return nErr;
}

int CDomePro::getBatteryLevels(double &dShutterVolts, int &nPercent)
{
    int nErr = PD_OK;
    int rc = 0;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return nErr;

    nErr = domeCommand("BAT\r", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    rc = sscanf(szResp, "%d %lf", &nPercent, &dShutterVolts);
    if(rc == 0) {
        return COMMAND_FAILED;
    }

    dShutterVolts = dShutterVolts / 1000.0;
    return nErr;
}

bool CDomePro::hasShutterUnit() {
    return m_bHasShutter;
}

void CDomePro::setDebugLog(bool bEnable)
{
    m_bDebugLog = bEnable;
}

int CDomePro::isDomeMoving(bool &bIsMoving)
{
    int tmp;
    int nErr = PD_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = domeCommand("MSTATE\r", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    bIsMoving = false;
    tmp = atoi(szResp);
    if(tmp != 0 || tmp != 3)
        bIsMoving = true;

    return nErr;
}

int CDomePro::isDomeAtHome(bool &bAtHome)
{
    int tmp;
    int nErr = PD_OK;
    char resp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = domeCommand("HOME ?\r", resp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return false;

    bAtHome = false;
    tmp = atoi(resp);
    if(tmp)
        bAtHome = true;

    return nErr;

}

int CDomePro::syncDome(double dAz, double dEl)
{
    int nErr = PD_OK;
    char szBuf[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    m_dCurrentAzPosition = dAz;
    snprintf(szBuf, SERIAL_BUFFER_SIZE, "ANGLE K %3.1f\r", dAz);
    nErr = domeCommand(szBuf, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    if(strncmp(szResp,"A",1) == 0) {
        nErr = PD_OK;
    }
    else {
        nErr = PD_BAD_CMD_RESPONSE;
    }
    return nErr;
}

int CDomePro::parkDome()
{
    int nErr = PD_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return SB_OK;

    nErr = domeCommand("GO P\r", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    if(strncmp(szResp,"A",1) == 0) {
        nErr = PD_OK;
    }
    else {
        nErr = PD_BAD_CMD_RESPONSE;
    }
    return nErr;
}

int CDomePro::unparkDome()
{
    m_bParked = false;
    m_dCurrentAzPosition = m_dParkAz;
    syncDome(m_dCurrentAzPosition,m_dCurrentElPosition);
    return 0;
}

int CDomePro::gotoAzimuth(double dNewAz)
{

    int nErr = PD_OK;
    char szBuf[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    snprintf(szBuf, SERIAL_BUFFER_SIZE, "GO %3.1f\r", dNewAz);
    nErr = domeCommand(szBuf, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    if(strncmp(szResp,"A",1) == 0) {
        nErr = PD_OK;
    }
    else {
        nErr = PD_BAD_CMD_RESPONSE;
    }

    m_dGotoAz = dNewAz;

    return nErr;
}

int CDomePro::openShutter()
{
    int nErr = PD_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return SB_OK;

    nErr = domeCommand("OPEN\r", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    if(strncmp(szResp,"A",1) == 0) {
        nErr = PD_OK;
    }
    else {
        nErr = PD_BAD_CMD_RESPONSE;
    }
    return nErr;
}

int CDomePro::closeShutter()
{
    int nErr = PD_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return SB_OK;

    nErr = domeCommand("CLOSE\r", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    if(strncmp(szResp,"A",1) == 0) {
        nErr = PD_OK;
    }
    else {
        nErr = PD_BAD_CMD_RESPONSE;
    }
    return nErr;
}

int CDomePro::getFirmwareVersion(char *pszVersion, int nStrMaxLen)
{
    int nErr = PD_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    unsigned long nFirmwareVersion;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return SB_OK;

    nErr = domeCommand("!DGfv;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    nFirmwareVersion = strtol(szResp, NULL, 16);
    snprintf(pszVersion, nStrMaxLen, "%lu", nFirmwareVersion);
    return nErr;
}

int CDomePro::getModel(char *pszModel, int nStrMaxLen)
{
    int nErr = PD_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return SB_OK;

    nErr = domeCommand("PULSAR\r", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    strncpy(pszModel, szResp, nStrMaxLen);
    return nErr;
}

int CDomePro::goHome()
{
    int nErr = PD_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return SB_OK;

    nErr = domeCommand("GO H\r", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    if(strncmp(szResp,"A",1) == 0) {
        nErr = PD_OK;
    }
    else {
        nErr = PD_BAD_CMD_RESPONSE;
    }
    return nErr;
}

int CDomePro::calibrate()
{
    int nErr = PD_OK;
    char resp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return SB_OK;

    nErr = domeCommand("CALIBRATE\r", resp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    if(strncmp(resp,"A",1) == 0) {
        nErr = PD_OK;
    }
    else {
        nErr = PD_BAD_CMD_RESPONSE;
        return nErr;
    }

    m_bCalibrating = true;

    return nErr;
}

int CDomePro::isGoToComplete(bool &bComplete)
{
    int nErr = 0;
    double dDomeAz = 0;
    bool bIsMoving = false;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = isDomeMoving(bIsMoving);
    if(nErr) {
        return nErr;
        }

    if(bIsMoving) {
        bComplete = false;
        getDomeAz(dDomeAz);
        return nErr;
    }

    getDomeAz(dDomeAz);

    if (ceil(m_dGotoAz) == ceil(dDomeAz))
        bComplete = true;
    else {
        // we're not moving and we're not at the final destination !!!
        if (m_bDebugLog) {
            snprintf(m_szLogBuffer,ND_LOG_BUFFER_SIZE,"[CDomePro::isGoToComplete] domeAz = %f, mGotoAz = %f\n", ceil(dDomeAz), ceil(m_dGotoAz));
            m_pLogger->out(m_szLogBuffer);
        }
        bComplete = false;
        nErr = ERR_CMDFAILED;
    }

    return nErr;
}

int CDomePro::isOpenComplete(bool &bComplete)
{
    int nErr = 0;
    int nState;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = getShutterState(nState);
    if(nErr)
        return ERR_CMDFAILED;
    if(nState == OPEN){
        m_bShutterOpened = true;
        bComplete = true;
        m_dCurrentElPosition = 90.0;
    }
    else {
        m_bShutterOpened = false;
        bComplete = false;
        m_dCurrentElPosition = 0.0;
    }

    return nErr;
}

int CDomePro::isCloseComplete(bool &bComplete)
{
    int err=0;
    int nState;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    err = getShutterState(nState);
    if(err)
        return ERR_CMDFAILED;
    if(nState == CLOSED){
        m_bShutterOpened = false;
        bComplete = true;
        m_dCurrentElPosition = 0.0;
    }
    else {
        m_bShutterOpened = true;
        bComplete = false;
        m_dCurrentElPosition = 90.0;
    }

    return err;
}


int CDomePro::isParkComplete(bool &bComplete)
{
    int nErr = 0;
    double dDomeAz=0;
    bool bIsMoving = false;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    getDomeAz(dDomeAz);
    nErr = isDomeMoving(bIsMoving);
    if(nErr)
        return nErr;

    if(bIsMoving) {
        bComplete = false;
        return nErr;
    }

    if (ceil(m_dParkAz) == ceil(dDomeAz))
    {
        m_bParked = true;
        bComplete = true;
    }
    else {
        // we're not moving and we're not at the final destination !!!
        bComplete = false;
        m_bHomed = false;
        m_bParked = false;
        nErr = ERR_CMDFAILED;
    }

    return nErr;
}

int CDomePro::isUnparkComplete(bool &bComplete)
{
    int nErr = 0;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    m_bParked = false;
    bComplete = true;

    return nErr;
}

int CDomePro::isFindHomeComplete(bool &bComplete)
{
    int nErr = 0;
    bool bIsMoving = false;
    bool bIsAtHome = false;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = isDomeMoving(bIsMoving);
    if(nErr)
        return nErr;

    if(bIsMoving) {
        m_bHomed = false;
        bComplete = false;
        return nErr;
    }

    nErr = isDomeAtHome(bIsAtHome);
    if(nErr)
        return nErr;

    if(bIsAtHome){
        m_bHomed = true;
        bComplete = true;
    }
    else {
        // we're not moving and we're not at the home position !!!
        if (m_bDebugLog) {
            snprintf(m_szLogBuffer,ND_LOG_BUFFER_SIZE,"[CDomePro::isFindHomeComplete] Not moving and not at home !!!\n");
            m_pLogger->out(m_szLogBuffer);
        }
        bComplete = false;
        m_bHomed = false;
        m_bParked = false;
        nErr = ERR_CMDFAILED;
    }

    return nErr;
}


int CDomePro::isCalibratingComplete(bool &bComplete)
{
    int nErr = 0;
    double dDomeAz = 0;
    bool bIsMoving = false;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = isDomeMoving(bIsMoving);
    if(nErr)
        return nErr;

    if(bIsMoving) {
        getDomeAz(dDomeAz);
        m_bHomed = false;
        bComplete = false;
        return nErr;
    }

    nErr = getDomeAz(dDomeAz);

    if (ceil(m_dHomeAz) != ceil(dDomeAz)) {
        // We need to resync the current position to the home position.
        m_dCurrentAzPosition = m_dHomeAz;
        syncDome(m_dCurrentAzPosition,m_dCurrentElPosition);
        m_bHomed = true;
        bComplete = true;
    }

    nErr = getDomeStepPerRev(m_nNbStepPerRev);
    m_bHomed = true;
    bComplete = true;
    m_bCalibrating = false;
    return nErr;
}


int CDomePro::abortCurrentCommand()
{
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    m_bCalibrating = false;

    return (domeCommand("STOP\r", NULL, SERIAL_BUFFER_SIZE));
}

#pragma mark - Getter / Setter

int CDomePro::getNbTicksPerRev()
{
    if(m_bIsConnected)
        getDomeStepPerRev(m_nNbStepPerRev);
    return m_nNbStepPerRev;
}


double CDomePro::getHomeAz()
{
    if(m_bIsConnected)
        getDomeHomeAz(m_dHomeAz);

    return m_dHomeAz;
}

int CDomePro::setHomeAz(double dAz)
{
    int nErr = PD_OK;
    char szBuf[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    snprintf(szBuf, SERIAL_BUFFER_SIZE, "HOME %3.1f\r", dAz);
    nErr = domeCommand(szBuf, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    if(strncmp(szResp,"A",1) == 0) {
        nErr = PD_OK;
    }
    else {
        nErr = PD_BAD_CMD_RESPONSE;
    }
    m_dHomeAz = dAz;
    return nErr;
}


double CDomePro::getParkAz()
{
    if(m_bIsConnected)
        getDomeParkAz(m_dParkAz);

    return m_dParkAz;

}

int CDomePro::setParkAz(double dAz)
{
    int nErr = PD_OK;
    char szBuf[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    snprintf(szBuf, SERIAL_BUFFER_SIZE, "PARK %3.1f\r", dAz);
    nErr = domeCommand(szBuf, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    if(strncmp(szResp,"A",1) == 0) {
        nErr = PD_OK;
    }
    else {
        nErr = PD_BAD_CMD_RESPONSE;
    }

    m_dParkAz = dAz;
    return nErr;
}


double CDomePro::getCurrentAz()
{
    if(m_bIsConnected)
        getDomeAz(m_dCurrentAzPosition);

    return m_dCurrentAzPosition;
}

double CDomePro::getCurrentEl()
{
    if(m_bIsConnected)
        getDomeEl(m_dCurrentElPosition);

    return m_dCurrentElPosition;
}

int CDomePro::getCurrentShutterState()
{
    if(m_bIsConnected)
        getShutterState(m_nShutterState);

    return m_nShutterState;
}

