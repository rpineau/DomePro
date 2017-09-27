//
//  DomePro.cpp
//  Rigel rotation drive unit for Pulsar Dome X2 plugin
//
//  Created by Rodolphe Pineau on 6/11/2016.


#include "domepro.h"

CDomePro::CDomePro()
{
    // set some sane values
    m_bDebugLog = true;

    m_pSerx = NULL;
    m_bIsConnected = false;

    m_nNbStepPerRev = 0;

    m_dHomeAz = 0;
    m_dParkAz = 0;

    m_dCurrentAzPosition = 0.0;
    m_dCurrentElPosition = 0.0;

    m_bCalibrating = false;

    m_bHasShutter = false;
    m_bShutterOpened = false;

    m_bParked = true;   // assume we were parked.
    m_bHomed = false;
    
    memset(m_szFirmwareVersion,0,SERIAL_BUFFER_SIZE);
    memset(m_szLogBuffer,0,DP2_LOG_BUFFER_SIZE);

#ifdef	FILE_DEBUG
    Logfile = fopen(LOGFILENAME, "w");
    ltime = time(NULL);
    char *timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CDomePro Constructor Called.\n", timestamp);
    fflush(Logfile);
#endif

}

CDomePro::~CDomePro()
{
#ifdef	FILE_DEBUG
    if (Logfile)
        fclose(Logfile);
#endif


}

#pragma mark - Dome Communication

int CDomePro::Connect(const char *pszPort)
{
    int nErr;
    int nState;

    if(!m_pSerx)
        return ERR_COMMNOLINK;

#ifdef FILE_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CDomePro::Connect] Called\n", timestamp);
    fflush(Logfile);
#endif

    // 19200 8N1
    if(m_pSerx->open(pszPort, 19200, SerXInterface::B_NOPARITY, "-DTR_CONTROL 1") == 0)
        m_bIsConnected = true;
    else
        m_bIsConnected = false;

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

#ifdef FILE_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CDomePro::Connect] connected to %s\n", timestamp, pszPort);
    fflush(Logfile);
#endif

    if (m_bDebugLog) {
        snprintf(m_szLogBuffer,DP2_LOG_BUFFER_SIZE,"[CDomePro::Connect] Connected.\n");
        m_pLogger->out(m_szLogBuffer);

        snprintf(m_szLogBuffer,DP2_LOG_BUFFER_SIZE,"[CDomePro::Connect] Getting Firmware.\n");
        m_pLogger->out(m_szLogBuffer);
    }

#ifdef FILE_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CDomePro::Connect] getting Firmware.\n", timestamp);
    fflush(Logfile);
#endif

    // if this fails we're not properly connected.
    nErr = getFirmwareVersion(m_szFirmwareVersion, SERIAL_BUFFER_SIZE);
    if(nErr) {
        if (m_bDebugLog) {
            snprintf(m_szLogBuffer,DP2_LOG_BUFFER_SIZE,"[CDomePro::Connect] Error Getting Firmware.\n");
            m_pLogger->out(m_szLogBuffer);
        }
#ifdef FILE_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CDomePro::Connect] Error %d Getting Firmware : %s\n", timestamp, nErr, m_szFirmwareVersion);
        fflush(Logfile);
#endif

        m_bIsConnected = false;
        m_pSerx->close();
        return ERR_COMMNOLINK;
    }

    if (m_bDebugLog) {
        snprintf(m_szLogBuffer,DP2_LOG_BUFFER_SIZE,"[CDomePro::Connect] Got Firmware.\n");
        m_pLogger->out(m_szLogBuffer);
    }

#ifdef FILE_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CDomePro::Connect] firmware  %s\n", timestamp, m_szFirmwareVersion);
    fflush(Logfile);
#endif


    // assume the dome was parked
    getDomeParkAz(m_dCurrentAzPosition);

    syncDome(m_dCurrentAzPosition, m_dCurrentElPosition);
    nErr = getDomeShutterStatus(nState);

#ifdef FILE_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CDomePro::Connect] m_dCurrentAzPosition  %f\n", timestamp, m_dCurrentAzPosition);
    fflush(Logfile);
#endif

    if(nState != NOT_FITTED )
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


#pragma mark - Dome API call

int CDomePro::syncDome(double dAz, double dEl)
{
    int nErr = DP2_OK;
    int nPos;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    m_dCurrentAzPosition = dAz;
    AzToTicks(dAz, nPos);
    nErr = calibrateDomeAzimuth(nPos);
    return nErr;
}

int CDomePro::gotoDomePark(void)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return nErr;

    nErr = domeCommand("!DSgp;", szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

int CDomePro::unparkDome()
{
    m_bParked = false;
    m_dCurrentAzPosition = m_dParkAz;

    syncDome(m_dCurrentAzPosition, m_dCurrentElPosition);
    return 0;
}

int CDomePro::gotoAzimuth(double dNewAz)
{

    int nErr = DP2_OK;
    int nPos;
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    AzToTicks(dNewAz, nPos);
    nErr = goToDomeAzimuth(nPos);

    m_dGotoAz = dNewAz;

    return nErr;
}

int CDomePro::openDomeShutters()
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return SB_OK;

    nErr = domeCommand("!DSso;", szResp, SERIAL_BUFFER_SIZE);
    return nErr;
}

int CDomePro::CloseDomeShutters()
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return SB_OK;

    nErr = domeCommand("!DSsc;", szResp, SERIAL_BUFFER_SIZE);
    return nErr;
}

int CDomePro::abortCurrentCommand()
{
    int nErr;
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    m_bCalibrating = false;

    nErr = killDomeAzimuthMovement();
    if(m_bHasShutter)
        nErr |= killDomeShutterMovement();
    return nErr;
}

int CDomePro::goHome()
{
    int nErr = DP2_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return SB_OK;

    nErr = homeDomeAzimuth();
    return nErr;
}

#pragma mark TODO : Calibrate needs rewriting
int CDomePro::calibrate()
{
    int nErr = DP2_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return SB_OK;

    m_bCalibrating = true;
    
    return nErr;
}

#pragma mark - dome controller informations

int CDomePro::getFirmwareVersion(char *pszVersion, int nStrMaxLen)
{
    int nErr = DP2_OK;
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
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return SB_OK;

    nErr = domeCommand("!DGhc;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    m_nModel = atoi(szResp);
    switch(m_nModel) {
        case CLASSIC_DOME :
            strncpy(pszModel, "DomePro2-d", SERIAL_BUFFER_SIZE);
            break;

        case CLAMSHELL :
            strncpy(pszModel, "DomePro2-c", SERIAL_BUFFER_SIZE);
            break;

        case ROR :
            strncpy(pszModel, "DomePro2-r", SERIAL_BUFFER_SIZE);
            break;
    }
    return nErr;
}


int CDomePro::getModuleType(int &nModuleType)
{
    int nErr;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return SB_OK;

    nErr = domeCommand("!DGmy;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    if(strstr(szResp,"Az")) {
        m_nModuleType = MODULE_AZ;
    }
    else if(strstr(szResp,"Az")) {
        m_nModuleType = MODULE_SHUT;
    }
    else {
        m_nModuleType = MODULE_UKNOWN;
    }

    return nErr;
}

int CDomePro::setDomeAzMotorPolarity(int nPolarity)
{
    int nErr;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return SB_OK;

    m_nMotorPolarity = nPolarity;

    switch(m_nNbStepPerRev) {
        case POSITIVE :
            snprintf(szCmd, SERIAL_BUFFER_SIZE, "!DSmpPositive;");
            break;
        case NEGATIVE :
            snprintf(szCmd, SERIAL_BUFFER_SIZE, "!DSmpNegative;");
            break;
        default:
            break;
    }
    nErr = domeCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;


    return nErr;
    
}


int CDomePro::getDomeAzMotorPolarity(int &nPolarity)
{
    int nErr;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return SB_OK;

    nErr = domeCommand("!DGmp;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    if(strstr(szResp,"Positive")) {
        m_nMotorPolarity = POSITIVE;
    }
    else if(strstr(szResp,"Negative")) {
        m_nMotorPolarity = NEGATIVE;
    }

    else {
        m_nMotorPolarity = POLARITY_UKNOWN;
    }

    nPolarity = m_nMotorPolarity;
    return nErr;
}


int CDomePro::setDomeAzEncoderPolarity(int nPolarity)
{
    int nErr;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return SB_OK;

    m_nAzEncoderPolarity = nPolarity;

    switch(m_nAzEncoderPolarity) {
        case POSITIVE :
            snprintf(szCmd, SERIAL_BUFFER_SIZE, "!DSepPositive;");
            break;
        case NEGATIVE :
            snprintf(szCmd, SERIAL_BUFFER_SIZE, "!DSepNegative;");
            break;
        default:
            break;
    }
    nErr = domeCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    
    return nErr;
}

int CDomePro::getDomeAzEncoderPolarity(int &nPolarity)
{
    int nErr;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return SB_OK;

    nErr = domeCommand("!DGep;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    if(strstr(szResp,"Positive")) {
        m_nAzEncoderPolarity = POSITIVE;
    }
    else if(strstr(szResp,"Negative")) {
        m_nAzEncoderPolarity = NEGATIVE;
    }

    else {
        m_nAzEncoderPolarity = POLARITY_UKNOWN;
    }

    nPolarity = m_nAzEncoderPolarity;
    return nErr;
}


bool CDomePro::hasShutterUnit() {
    return m_bHasShutter;
}


#pragma mark - command complete functions

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
        getDomeAzPosition(dDomeAz);
        return nErr;
    }

    getDomeAzPosition(dDomeAz);

    if (ceil(m_dGotoAz) == ceil(dDomeAz))
        bComplete = true;
    else {
        // we're not moving and we're not at the final destination !!!
        if (m_bDebugLog) {
            snprintf(m_szLogBuffer,DP2_LOG_BUFFER_SIZE,"[CDomePro::isGoToComplete] domeAz = %f, mGotoAz = %f\n", ceil(dDomeAz), ceil(m_dGotoAz));
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

    nErr = getDomeShutterStatus(nState);
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

    err = getDomeShutterStatus(nState);
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

    getDomeAzPosition(dDomeAz);
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
            snprintf(m_szLogBuffer,DP2_LOG_BUFFER_SIZE,"[CDomePro::isFindHomeComplete] Not moving and not at home !!!\n");
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
        getDomeAzPosition(dDomeAz);
        m_bHomed = false;
        bComplete = false;
        return nErr;
    }

    nErr = getDomeAzPosition(dDomeAz);

    if (ceil(m_dHomeAz) != ceil(dDomeAz)) {
        // We need to resync the current position to the home position.
        m_dCurrentAzPosition = m_dHomeAz;
        syncDome(m_dCurrentAzPosition, m_dCurrentElPosition);
        m_bHomed = true;
        bComplete = true;
    }

    nErr = getDomeAzCPR(m_nNbStepPerRev);
    m_bHomed = true;
    bComplete = true;
    m_bCalibrating = false;
    return nErr;
}



#pragma mark - Getter / Setter

int CDomePro::setHomeAz(double dAz)
{
    int nErr = DP2_OK;
    int nPos;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    AzToTicks(dAz, nPos);
    setDomeHomeAzimuth(nPos);
    m_dHomeAz = dAz;
    return nErr;
}

int CDomePro::getDomeAzCoast(double dAz)
{
    int nErr = DP2_OK;
    int nPos;

    nErr = getDomeAzCoast(nPos);
    if(nErr)
        return nErr;

    TicksToAz(nPos, dAz);

    return nErr;
}


int CDomePro::setParkAz(double dAz)
{
    int nErr = DP2_OK;
    int nPos;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    AzToTicks(dAz, nPos);
    setDomeParkAzimuth(nPos);

    m_dParkAz = dAz;
    return nErr;
}


double CDomePro::getCurrentAz()
{
    if(m_bIsConnected)
        getDomeAzPosition(m_dCurrentAzPosition);

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
        getDomeShutterStatus(m_nShutterState);

    return m_nShutterState;
}

void CDomePro::setDebugLog(bool bEnable)
{
    m_bDebugLog = bEnable;
}


#pragma mark - protected methods

#pragma mark - dome communication

int CDomePro::domeCommand(const char *pszCmd, char *pszResult, int nResultMaxLen)
{
    int nErr = DP2_OK;
    unsigned char szResp[SERIAL_BUFFER_SIZE];
    unsigned long ulBytesWrite;

    m_pSerx->purgeTxRx();
    if (m_bDebugLog) {
        snprintf(m_szLogBuffer,DP2_LOG_BUFFER_SIZE,"[CDomePro::domeCommand] Sending %s\n",pszCmd);
        m_pLogger->out(m_szLogBuffer);
    }

#ifdef FILE_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CDomePro::domeCommand] Sending %s\n", timestamp, pszCmd);
    fflush(Logfile);
#endif

    nErr = m_pSerx->writeFile((void *)pszCmd, strlen(pszCmd), ulBytesWrite);
    m_pSerx->flushTx();
    if(nErr)
        return nErr;
    // read response
    if (m_bDebugLog) {
        snprintf(m_szLogBuffer,DP2_LOG_BUFFER_SIZE,"[CDomePro::domeCommand] Getting response.\n");
        m_pLogger->out(m_szLogBuffer);
    }
    nErr = readResponse(szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {

#ifdef FILE_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CDomePro::domeCommand] error %d reading response : %s\n", timestamp, nErr, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }
    if(pszResult)
        strncpy(pszResult, (const char *)szResp, nResultMaxLen);

#ifdef FILE_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CDomePro::domeCommand] got response : '%s'\n", timestamp, szResp);
    fflush(Logfile);
#endif

    return nErr;

}


int CDomePro::readResponse(unsigned char *pszRespBuffer, int nBufferLen)
{
    int nErr = DP2_OK;
    unsigned long ulBytesRead = 0;
    unsigned long ulTotalBytesRead = 0;
    unsigned char *pszBufPtr;

    memset(pszRespBuffer, 0, (size_t) nBufferLen);
    pszBufPtr = pszRespBuffer;

    do {
        nErr = m_pSerx->readFile(pszBufPtr, 1, ulBytesRead, MAX_TIMEOUT);
        if(nErr) {
            if (m_bDebugLog) {
                snprintf(m_szLogBuffer,DP2_LOG_BUFFER_SIZE,"[CDomePro::readResponse] readFile error.\n");
                m_pLogger->out(m_szLogBuffer);
            }
            return nErr;
        }

#ifdef FILE_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CDomePro::readResponse] *pszBufPtr = %02X\n", timestamp, *pszBufPtr);
        fflush(Logfile);
#endif

        if (ulBytesRead !=1) {// timeout
            if (m_bDebugLog) {
                snprintf(m_szLogBuffer,DP2_LOG_BUFFER_SIZE,"[CDomePro::readResponse] readFile Timeout.\n");
                m_pLogger->out(m_szLogBuffer);
            }
            nErr = DP2_BAD_CMD_RESPONSE;
            break;
        }
        ulTotalBytesRead += ulBytesRead;
        if (m_bDebugLog) {
            snprintf(m_szLogBuffer,DP2_LOG_BUFFER_SIZE,"[CDomePro::readResponse] nBytesRead = %lu\n",ulBytesRead);
            m_pLogger->out(m_szLogBuffer);
        }
        // check for  errors or single ACK
        if(*pszBufPtr == ATCL_NACK) {
            nErr = DP2_BAD_CMD_RESPONSE;
            break;
        }

        if(*pszBufPtr == ATCL_ACK) {
            nErr = DP2_OK;
            break;
        }


    } while (*pszBufPtr++ != ';' && ulTotalBytesRead < nBufferLen );

    if(ulTotalBytesRead && *(pszBufPtr-1) == ';')
        *(pszBufPtr-1) = 0; //remove the ; to zero terminate the string

    return nErr;
}

#pragma mark - conversion functions

//	Convert pdAz to number of ticks from home.
void CDomePro::AzToTicks(double pdAz, int &ticks)
{
    if(!m_nNbStepPerRev)
        getDomeAzCPR(m_nNbStepPerRev);

    ticks = (int) floor(0.5 + (pdAz - m_dHomeAz) * m_nNbStepPerRev / 360.0);
    while (ticks > m_nNbStepPerRev) ticks -= m_nNbStepPerRev;
    while (ticks < 0) ticks += m_nNbStepPerRev;
}


// Convert ticks from home to Az
void CDomePro::TicksToAz(int ticks, double &pdAz)
{
    if(!m_nNbStepPerRev)
        getDomeAzCPR(m_nNbStepPerRev);

    pdAz = m_dHomeAz + (ticks * 360.0 / m_nNbStepPerRev);
    while (pdAz < 0) pdAz += 360;
    while (pdAz >= 360) pdAz -= 360;
}


#pragma mark - Dome movements

int CDomePro::setDomeLeftOn(void)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return nErr;

    nErr = domeCommand("!DSol;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    return nErr;
}

int CDomePro::setDomeRightOn(void)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return nErr;

    nErr = domeCommand("!DSor;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    return nErr;
}

int CDomePro::killDomeAzimuthMovement(void)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return nErr;

    nErr = domeCommand("!DXxa;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    return nErr;
}

#pragma mark - getter / setter

int CDomePro::getDomeAzPosition(double &dDomeAz)
{
    int nErr = DP2_OK;
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

    TicksToAz(nTmp, dDomeAz);

    m_dCurrentAzPosition = dDomeAz;

    return nErr;
}

int CDomePro::getDomeEl(double &dDomeEl)
{
    int nErr = DP2_OK;
    int nShutterState;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    getDomeShutterStatus(nShutterState);

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
    int nErr = DP2_OK;
    int nPos;

    nErr = getDomeHomeAzimuth(nPos);
    if(nErr)
        return nErr;

    TicksToAz(nPos, dAz);

    return nErr;
}

int CDomePro::getDomeParkAz(double &dAz)
{
    int nErr = DP2_OK;
    int nPos;

    nErr = getDomeParkAzimuth(nPos);
    if(nErr)
        return nErr;

    TicksToAz(nPos, dAz);

    return nErr;
}


int CDomePro::getDomeShutterStatus(int &nState)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    int nShutterState;

    nErr = domeCommand("!DGsx;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    nShutterState = strtol(szResp, NULL, 16);

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

    nState = nShutterState;

    return nErr;
}


#pragma mark - command completion/state

int CDomePro::isDomeMoving(bool &bIsMoving)
{
    int nErr = DP2_OK;
    int nMode;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    bIsMoving = false;

    nErr = getDomeAzMoveMode(nMode);
    if(nErr)
        return nErr;

    if(nMode != FIXED && nMode != AZ_TO)
        bIsMoving = true;

    return nErr;
}

int CDomePro::isDomeAtHome(bool &bAtHome)
{
    int nErr = DP2_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    bAtHome = false;

    nErr = getDomeLimits();
    if(nErr)
        return nErr;

    if(m_nAtHomeState == ACTIVE)
        bAtHome = true;

    return nErr;
}

#pragma mark - DomePro getter/setter

int CDomePro::setDomeAzCPR(int nValue)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];

    // nCpr must be betweem 0x20 and 0x40000000 and be even
    if(nValue < 0x20 )
        nValue = 0x20;
    if(nValue>0x40000000)
        nValue = 0x40000000;
    nValue &= 0XFFFFFFFE; // makes it an even number

    snprintf(szCmd, SERIAL_BUFFER_SIZE, "!DScp0x%08X;", nValue);
    nErr = domeCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

int CDomePro::getDomeAzCPR(int &nValue)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = domeCommand("!DGcp;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // convert result hex string to long
    nValue = strtol(szResp, NULL, 16);
    return nErr;
}

#pragma mark not yet implemented in the firmware
int CDomePro::setDomeMaxVel(int nValue)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];

    // nValue must be betweem 0x01 and 0x7C (124)
    if(nValue < 0x1 )
        nValue = 0x1;
    if(nValue>0x7C)
        nValue = 0x7C;

    snprintf(szCmd, SERIAL_BUFFER_SIZE, "!DSmv0x%08X;", nValue);
    nErr = domeCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

#pragma mark not yet implemented in the firmware
int CDomePro::getDomeMaxVel(int &nValue)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = domeCommand("!DGmv;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // convert result hex string to long
    nValue = strtol(szResp, NULL, 16);
    return nErr;
}

#pragma mark not yet implemented in the firmware
int CDomePro::setDomeAccel(int nValue)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];

    // nValue must be betweem 0x01 and 0xFF (255)
    if(nValue < 0x1 )
        nValue = 0x1;
    if(nValue>0xFF)
        nValue = 0xFF;

    snprintf(szCmd, SERIAL_BUFFER_SIZE, "!DSma0x%08X;", nValue);
    nErr = domeCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

#pragma mark not yet implemented in the firmware
int CDomePro::getDomeAccel(int &nValue)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = domeCommand("!DGma;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // convert result hex string to long
    nValue = strtol(szResp, NULL, 16);
    return nErr;
}



int CDomePro::setDomeAzCoast(int nValue)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];

    // nCpr must be betweem 0x20 and 0x40000000 and be even
    if(nValue < 0x1 )
        nValue = 0x1;
    if(nValue>0x7C)
        nValue = 0x7C;

    snprintf(szCmd, SERIAL_BUFFER_SIZE, "!DSco0x%08X;", nValue);
    nErr = domeCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

int CDomePro::getDomeAzCoast(int &nValue)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = domeCommand("!DGco;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // convert result hex string to long
    nValue = strtol(szResp, NULL, 16);
    return nErr;
}

int CDomePro::getDomeAzDiagPosition(int &nValue)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = domeCommand("!DGdp;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // convert result hex string to long
    nValue = strtol(szResp, NULL, 16);
    return nErr;
}

int CDomePro::clearDomeAzDiagPosition(void)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = domeCommand("!DCdp;", szResp, SERIAL_BUFFER_SIZE);
    return nErr;
}

int CDomePro::getDomeAzMoveMode(int &mode)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = domeCommand("!DGam;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    if(strstr(szResp, "Fixed")) {
        mode = FIXED;
    }
    else if(strstr(szResp, "Left")) {
        mode = LEFT;
    }
    else if(strstr(szResp, "Right")) {
        mode = RIGHT;
    }
    else if(strstr(szResp, "GoTo")) {
        mode = GOTO;
    }
    else if(strstr(szResp, "Homing")) {
        mode = HOMING;
    }
    else if(strstr(szResp, "AzimuthTO")) {
        mode = AZ_TO;
    }

    return nErr;
}

int CDomePro::getDomeLimits(void)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = domeCommand("!DGdl;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    m_nShutter1OpenedSwitchState = (szResp[0] == '0' ? INNACTIVE : ACTIVE);
    m_nShutter1ClosedSwitchState = (szResp[1] == '0' ? INNACTIVE : ACTIVE);

    m_nShutter2OpenedSwitchState = (szResp[2] == '0' ? INNACTIVE : ACTIVE);
    m_nShutter2ClosedSwitchState = (szResp[3] == '0' ? INNACTIVE : ACTIVE);

    m_nAtHomeState = (szResp[4] == '0' ? INNACTIVE : ACTIVE);
    m_nAtHomeSwitchState = (szResp[5] == '0' ? INNACTIVE : ACTIVE);
    m_nAtParkSate = (szResp[6] == '0' ? INNACTIVE : ACTIVE);

    return nErr;
}

int CDomePro::setDomeHomeAzimuth(int nPos)
{

    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];

    if(nPos < 0 && nPos> m_nNbStepPerRev)
        return COMMAND_FAILED;
    
    snprintf(szCmd, SERIAL_BUFFER_SIZE, "!DSha0x%08X;", nPos);
    nErr = domeCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}


int CDomePro::getDomeHomeAzimuth(int &nPos)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = domeCommand("!DGha;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // convert result hex string to long
    nPos = strtol(szResp, NULL, 16);

    return nErr;
}

int CDomePro::homeDomeAzimuth(void)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = domeCommand("!DSah;", szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}


int CDomePro::goToDomeAzimuth(int nPos)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];

    if(nPos < 0 && nPos> m_nNbStepPerRev)
        return COMMAND_FAILED;

    snprintf(szCmd, SERIAL_BUFFER_SIZE, "!DSgo0x%08X;", nPos);
    nErr = domeCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

int CDomePro::setDomeParkAzimuth(int nPos)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];

    if(nPos < 0 && nPos> m_nNbStepPerRev)
        return COMMAND_FAILED;

    snprintf(szCmd, SERIAL_BUFFER_SIZE, "!DSpa0x%08X;", nPos);
    nErr = domeCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

int CDomePro::getDomeParkAzimuth(int &nPos)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = domeCommand("!DGpa;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // convert result hex string to long
    nPos = strtol(szResp, NULL, 16);

    return nErr;
}

int CDomePro::calibrateDomeAzimuth(int nPos)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];

    if(nPos < 0 && nPos> m_nNbStepPerRev)
        return COMMAND_FAILED;

    snprintf(szCmd, SERIAL_BUFFER_SIZE, "!DSca0x%08X;", nPos);
    nErr = domeCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}


int CDomePro::killDomeShutterMovement(void)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = domeCommand("!DXxs;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    return nErr;
}

int CDomePro::getDomeDebug(char *pszDebugStrBuff, int nStrMaxLen)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = domeCommand("!DGdg;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    strncpy(pszDebugStrBuff, szResp, nStrMaxLen);

    return nErr;
}

#pragma mark - low level dome data getter/setter

int CDomePro::getDomeSupplyVoltageAzimuthL(double &dVolts)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    unsigned int ulTmp;

    nErr = domeCommand("!DGva;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // convert result hex string
    ulTmp = strtol(szResp, NULL, 16);

    dVolts = (double)ulTmp * 0.00812763;

    return nErr;
}

int CDomePro::getDomeSupplyVoltageShutterL(double &dVolts)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    unsigned int ulTmp;

    nErr = domeCommand("!DGvs;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // convert result hex string
    ulTmp = strtol(szResp, NULL, 16);

    dVolts = (double)ulTmp * 0.00812763;
    
    return nErr;
}

#pragma mark FIX VOLTAGE MULTIPLIER
int CDomePro::getDomeSupplyVoltageAzimuthM(double &dVolts)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    unsigned int ulTmp;

    nErr = domeCommand("!DGoa;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // convert result hex string
    ulTmp = strtol(szResp, NULL, 16);

    dVolts = (double)ulTmp * 1; // TBD

    return nErr;
}


#pragma mark FIX VOLTAGE MULTIPLIER
int CDomePro::getDomeSupplyVoltageShutterM(double &dVolts)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    unsigned int ulTmp;

    nErr = domeCommand("!DGos;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // convert result hex string
    ulTmp = strtol(szResp, NULL, 16);

    dVolts = (double)ulTmp * 1; // TBD

    return nErr;
}

#pragma mark not yet implemented in the firmware
int CDomePro::getDomeRotationSenseAnalog(double &dVolts)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    unsigned int ulTmp;

    nErr = domeCommand("!DGra;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // convert result hex string
    ulTmp = strtol(szResp, NULL, 16);

    dVolts = (double)ulTmp / 255 * 5; // FF = 5v, 0 = 0v
    
    return nErr;

}

int CDomePro::setDomeShutter1_OpTimeOut(int nTimeout)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];

    if(nTimeout < 10 && nTimeout > 500)
        return COMMAND_FAILED;

    snprintf(szCmd, SERIAL_BUFFER_SIZE, "!DSt10x%08X;", nTimeout);
    nErr = domeCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

int CDomePro::getDomeShutter1_OpTimeOut(int &nTimeout)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = domeCommand("!DGt1;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // convert result hex string to long
    nTimeout = strtol(szResp, NULL, 16);

    return nErr;
}

int CDomePro::setDomeShutter2_OpTimeOut(int nTimeout)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];

    if(nTimeout < 10 && nTimeout > 500)
        return COMMAND_FAILED;

    snprintf(szCmd, SERIAL_BUFFER_SIZE, "!DSt20x%08X;", nTimeout);
    nErr = domeCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;

}

int CDomePro::getDomeShutter2_OpTimeOut(int &nTimeout)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = domeCommand("!DGt1;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // convert result hex string to long
    nTimeout = strtol(szResp, NULL, 16);

    return nErr;
}

int CDomePro::setDomeShutODirTimeOut(int nTimeout)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];

    if(nTimeout < 10 && nTimeout > 500)
        return COMMAND_FAILED;

    snprintf(szCmd, SERIAL_BUFFER_SIZE, "!DSto0x%08X;", nTimeout);
    nErr = domeCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

int CDomePro::getDomeShutODirTimeOut(int &nTimeout)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = domeCommand("!DGto;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // convert result hex string to long
    nTimeout = strtol(szResp, NULL, 16);

    return nErr;
}

int CDomePro::setDomeAzimuthTimeOutEnabled(bool bEnable)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];

    if(bEnable)
        snprintf(szCmd, SERIAL_BUFFER_SIZE, "!DSaeYes;");
    else
        snprintf(szCmd, SERIAL_BUFFER_SIZE, "!DSaeNo;");

    nErr = domeCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;

}

int CDomePro::getDomeAzimuthTimeOutEnabled(bool &bEnable)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    bEnable = false;

    nErr = domeCommand("!DGae;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    if(strstr(szResp,"Yes"))
        bEnable = true;

    return nErr;
}

int CDomePro::setDomeAzimuthTimeOut(int nTimeout)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];

    if(nTimeout < 10 && nTimeout > 500)
        return COMMAND_FAILED;

    snprintf(szCmd, SERIAL_BUFFER_SIZE, "!DSta0x%08X;", nTimeout);
    nErr = domeCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

int CDomePro::getDomeAzimuthTimeOut(int &nTimeout)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = domeCommand("!DGta;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // convert result hex string to long
    nTimeout = strtol(szResp, NULL, 16);

    return nErr;
}

int CDomePro::setDomeShutCloseOnLinkTimeOut(bool bEnable)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];

    if(bEnable)
        snprintf(szCmd, SERIAL_BUFFER_SIZE, "!DStsYes;");
    else
        snprintf(szCmd, SERIAL_BUFFER_SIZE, "!DStsNo;");

    nErr = domeCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
    
}

int CDomePro::getDomeShutCloseOnLinkTimeOut(bool &bEnable)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    bEnable = false;

    nErr = domeCommand("!DGts;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    if(strstr(szResp,"Yes"))
        bEnable = true;

    return nErr;
}

int CDomePro::setDomeShutCloseOnClientTimeOut(bool bEnable)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];

    if(bEnable)
        snprintf(szCmd, SERIAL_BUFFER_SIZE, "!DSteYes;");
    else
        snprintf(szCmd, SERIAL_BUFFER_SIZE, "!DSteNo;");

    nErr = domeCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

int CDomePro::getDomeShutCloseOnClientTimeOut(bool &bEnable)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    bEnable = false;

    nErr = domeCommand("!DGte;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    if(strstr(szResp,"Yes"))
        bEnable = true;
    
    return nErr;
}

int CDomePro::setDomeShutCloseClientTimeOut(int nTimeout)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];

    if(nTimeout < 10 && nTimeout > 500)
        return COMMAND_FAILED;

    snprintf(szCmd, SERIAL_BUFFER_SIZE, "!DStc0x%08X;", nTimeout);
    nErr = domeCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

int CDomePro::getDomeShutCloseClientTimeOut(int &nTimeout)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = domeCommand("!DGtc;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // convert result hex string to long
    nTimeout = strtol(szResp, NULL, 16);
    
    return nErr;
}

#pragma mark not yet implemented in the firmware
int CDomePro::setDomeShutOpAtHome(bool bEnable)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];

    if(bEnable)
        snprintf(szCmd, SERIAL_BUFFER_SIZE, "!DSshYes;");
    else
        snprintf(szCmd, SERIAL_BUFFER_SIZE, "!DSshNo;");

    nErr = domeCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

#pragma mark not yet implemented in the firmware
int CDomePro::getDomeShutOpAtHome(bool &bEnable)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    bEnable = false;

    nErr = domeCommand("!DGsh;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    if(strstr(szResp,"Yes"))
        bEnable = true;

    return nErr;
}

int CDomePro::getDomeShutdownInputState(bool &bEnable)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    bEnable = false;

    nErr = domeCommand("!DGsi;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    if(strstr(szResp,"Yes"))
        bEnable = true;

    return nErr;
}

int CDomePro::getDomePowerGoodInputState(bool &bEnable)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    bEnable = false;

    nErr = domeCommand("!DGpi;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    if(strstr(szResp,"Yes"))
        bEnable = true;

    return nErr;
}

#pragma mark not yet implemented in the firmware
int CDomePro::getLastDomeShutdownEvent(void)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = domeCommand("!DGlv;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // need to parse output and set some varaible/structure representing the event

    return nErr;
}

int CDomePro::setDomeSingleShutterMode(bool bEnable)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];

    if(bEnable)
        snprintf(szCmd, SERIAL_BUFFER_SIZE, "!DSssYes;");
    else
        snprintf(szCmd, SERIAL_BUFFER_SIZE, "!DSssNo;");

    nErr = domeCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

int CDomePro::getDomeSingleShutterMode(bool &bEnable)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    bEnable = false;

    nErr = domeCommand("!DGss;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    if(strstr(szResp,"Yes"))
        bEnable = true;

    return nErr;
}

int CDomePro::getDomeLinkErrCnt(int &nErrCnt)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = domeCommand("!DGle;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // convert result hex string to long
    nErrCnt = strtol(szResp, NULL, 16);

    return nErr;
}

int CDomePro::clearDomeLinkErrCnt(void)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = domeCommand("!DCle;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    return nErr;
}

#pragma mark not yet implemented in the firmware
int CDomePro::getDomeComErr(void)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = domeCommand("!DGce;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // need to parse output and set some varaible/structure representing the comms errors
    
    return nErr;
}

#pragma mark not yet implemented in the firmware
int CDomePro::clearDomeComErr(void)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = domeCommand("!DCce;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    return nErr;
}

int CDomePro::openDomeShutter1(void)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = domeCommand("!DSo1;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    return nErr;
}

int CDomePro::openDomeShutter2(void)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = domeCommand("!DSo2;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    return nErr;
}

int CDomePro::closeDomeShutter1(void)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = domeCommand("!DSc1;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    return nErr;
}

int CDomePro::closeDomeShutter2(void)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = domeCommand("!DSc2;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    return nErr;
}

int CDomePro::stopDomeShutter1(void)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = domeCommand("!DSs1;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    return nErr;
}

int CDomePro::stopDomeShutter2(void)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = domeCommand("!DSs2;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    return nErr;
}

int CDomePro::goToDomeShutter1_ADC(int nPos)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];

    snprintf(szCmd, SERIAL_BUFFER_SIZE, "!DSg10x%08X;", nPos);
    nErr = domeCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

int CDomePro::goToDomeShutter2_ADC(int nPos)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];

    snprintf(szCmd, SERIAL_BUFFER_SIZE, "!DSg20x%08X;", nPos);
    nErr = domeCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

int CDomePro::getDomeShutter1_AltitudeADC(int &nPos)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = domeCommand("!DGa1;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // convert result hex string to long
    nPos = strtol(szResp, NULL, 16);

    return nErr;
}

int CDomePro::getDomeShutter2_AltitudeADC(int &nPos)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = domeCommand("!DGa2;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // convert result hex string to long
    nPos = strtol(szResp, NULL, 16);

    return nErr;
}

int CDomePro::setDomeShutterOpenFirst(int nShutter)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];

    snprintf(szCmd, SERIAL_BUFFER_SIZE, "!DSof0x%02X;", nShutter);
    nErr = domeCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;

}

int CDomePro::getDomeShutterOpenFirst(int &nShutter)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = domeCommand("!DGof;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // convert result hex string to long
    nShutter = strtol(szResp, NULL, 16);

    return nErr;
}

int CDomePro::setDomeShutterCloseFirst(int nShutter)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];

    snprintf(szCmd, SERIAL_BUFFER_SIZE, "!DScf0x%02X;", nShutter);
    nErr = domeCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;

}

int CDomePro::getDomeShutterCloseFirst(int &nShutter)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = domeCommand("!DGcf;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // convert result hex string to long
    nShutter = strtol(szResp, NULL, 16);
    
    return nErr;
}

int CDomePro::getDomeShutterMotorADC(double &dVolts)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    unsigned int ulTmp;

    nErr = domeCommand("!DGsc;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // convert result hex string
    ulTmp = strtol(szResp, NULL, 16);

    dVolts = (double)ulTmp / 1023.0 * 3.3;
    dVolts = (dVolts - 1.721) / 0.068847;
    if (dVolts < 0.0)
        dVolts = 0.0;

    return nErr;
}

int CDomePro::getDomeAzimuthMotorADC(double &dVolts)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    unsigned int ulTmp;

    nErr = domeCommand("!DGac;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // convert result hex string
    ulTmp = strtol(szResp, NULL, 16);

    dVolts = (double)ulTmp / 1023.0 * 3.3;
    dVolts = (dVolts - 1.721) / 0.068847;
    if (dVolts < 0.0)
        dVolts = 0.0;

    return nErr;
}

int CDomePro::getDomeShutterTempADC(double &dTemp)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    unsigned int ulTmp;

    nErr = domeCommand("!DGst;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // convert result hex string
    ulTmp = strtol(szResp, NULL, 16);

    dTemp = (double)ulTmp / 1023.0 * 3.3 - 0.5;
    dTemp = dTemp / 0.01;

    return nErr;
}

int CDomePro::getDomeAzimuthTempADC(double &dTemp)
{
    int nErr = DP2_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    unsigned int ulTmp;

    nErr = domeCommand("!DGat;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // convert result hex string
    ulTmp = strtol(szResp, NULL, 16);

    dTemp = (double)ulTmp / 1023.0 * 3.3 - 0.5;
    dTemp = dTemp / 0.01;

    return nErr;
}


void  CDomePro::hexdump(const char* inputData, char *outBuffer, int size)
{
    char *buf = outBuffer;
    int idx=0;
    for(idx=0; idx<size; idx++){
        snprintf((char *)buf,4,"%02X ", inputData[idx]);
        buf+=3;
    }
    *buf = 0;
}

