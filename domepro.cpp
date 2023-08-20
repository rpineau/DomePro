//
//  DomePro.cpp
//  ATCL Dome X2 plugin
//
//  Created by Rodolphe Pineau on 6/11/2017.


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

    m_dAzCoast = 0.0;
    
    m_bCalibrating = false;

    m_bHasShutter = false;
    m_bShutterOpened = false;

    m_bParked = true;   // assume we were parked.
    m_bHomed = false;

    m_nLearning = 0;
    m_nLeftCPR = 0;
    m_nRightCPR = 0;

    m_bShutterGotoEnabled = false;

    m_sFirmware.clear();
    m_sLogBuffer.clear();

#ifdef PLUGIN_DEBUG
#if defined(SB_WIN_BUILD)
    m_sLogfilePath = getenv("HOMEDRIVE");
    m_sLogfilePath += getenv("HOMEPATH");
    m_sLogfilePath += "\\DomeProLog.txt";
#elif defined(SB_LINUX_BUILD)
    m_sLogfilePath = getenv("HOME");
    m_sLogfilePath += "/DomeProLog.txt";
#elif defined(SB_MAC_BUILD)
    m_sLogfilePath = getenv("HOME");
    m_sLogfilePath += "/DomeProLog.txt";
#endif
    m_sLogFile.open(m_sLogfilePath, std::ios::out |std::ios::trunc);
#endif

#if defined PLUGIN_DEBUG
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [CRTIDome] Version " << std::fixed << std::setprecision(2) << PLUGIN_VERSION << " build " << __DATE__ << " " << __TIME__ << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [CRTIDome] Constructor Called." << std::endl;
    m_sLogFile.flush();
#endif

}

CDomePro::~CDomePro()
{
#ifdef	PLUGIN_DEBUG
    if(m_sLogFile.is_open())
        m_sLogFile.close();
#endif
}

#pragma mark - dome communication
int CDomePro::domeCommand(const std::string sCmd, std::string &sResp, int nTimeout, char cEndOfResponse)
{
    int nErr = PLUGIN_OK;
    unsigned long  ulBytesWrite;
    std::string localResp;

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    m_pSerx->purgeTxRx();
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [domeCommand] Sending : '" << sCmd <<  "'" << std::endl;
    m_sLogFile.flush();
#endif
    nErr = m_pSerx->writeFile((void *)(sCmd.c_str()), sCmd.size(), ulBytesWrite);
    m_pSerx->flushTx();

    if(nErr){
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [domeCommand] writeFile error : " << nErr << std::endl;
        m_sLogFile.flush();
#endif
        return nErr;
    }

    // read response
    nErr = readResponse(localResp, nTimeout, cEndOfResponse);
    if(nErr)
        return nErr;

    if(!localResp.size())
        sResp.assign(localResp);
    else
        sResp.clear();

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [domeCommand] response : " << sResp << std::endl;
    m_sLogFile.flush();
#endif

    return nErr;
}

int CDomePro::readResponse(std::string &sResp, int nTimeout, char cEndOfResponse)
{
    int nErr = PLUGIN_OK;
    unsigned char pszBuf[SERIAL_BUFFER_SIZE];
    unsigned long ulBytesRead = 0;
    unsigned long ulTotalBytesRead = 0;
    unsigned char *pszBufPtr;
    int nBytesWaiting = 0 ;
    int nbTimeouts = 0;

    sResp.clear();
    memset(pszBuf, 0, SERIAL_BUFFER_SIZE);
    pszBufPtr = pszBuf;

    do {
        nErr = m_pSerx->bytesWaitingRx(nBytesWaiting);
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse] nBytesWaiting = " << nBytesWaiting << std::endl;
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse] nBytesWaiting nErr = " << nErr << std::endl;
        m_sLogFile.flush();
#endif
        if(!nBytesWaiting) {
            nbTimeouts += MAX_READ_WAIT_TIMEOUT;
            if(nbTimeouts >= nTimeout) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
                m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse] bytesWaitingRx timeout, no data for" << nbTimeouts <<" ms" << std::endl;
                m_sLogFile.flush();
#endif
                nErr = COMMAND_TIMEOUT;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(MAX_READ_WAIT_TIMEOUT));
            continue;
        }
        nbTimeouts = 0;
        if(ulTotalBytesRead + nBytesWaiting <= SERIAL_BUFFER_SIZE)
            nErr = m_pSerx->readFile(pszBufPtr, nBytesWaiting, ulBytesRead, nTimeout);
        else {
            nErr = ERR_RXTIMEOUT;
            break; // buffer is full.. there is a problem !!
        }
        if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse] readFile error." << std::endl;
            m_sLogFile.flush();
#endif
            return nErr;
        }

        if (ulBytesRead != nBytesWaiting) { // timeout
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse] readFile Timeout Error." << std::endl;
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse] readFile nBytesWaiting = " << nBytesWaiting << std::endl;
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse] readFile ulBytesRead =" << ulBytesRead << std::endl;
            m_sLogFile.flush();
#endif
        }

        ulTotalBytesRead += ulBytesRead;
        // check for  errors or single ACK
        if(*pszBufPtr == ATCL_NACK) {
            nErr = BAD_CMD_RESPONSE;
            break;
        }

        if(*pszBufPtr == ATCL_ACK) {
            nErr = PLUGIN_OK;
            break;
        }

        pszBufPtr+=ulBytesRead;

    } while (ulTotalBytesRead < SERIAL_BUFFER_SIZE  && *(pszBufPtr-1) != cEndOfResponse);


#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse] pszBuf = '" << pszBuf << "'" << std::endl;
    m_sLogFile.flush();
#endif


    if(!ulTotalBytesRead)
        nErr = COMMAND_TIMEOUT; // we didn't get an answer.. so timeout
    else  if(*(pszBufPtr-1) == cEndOfResponse)
        *(pszBufPtr-1) = 0; //remove the cEndOfResponse

    sResp.assign((char *)pszBuf);
    return nErr;
}



int CDomePro::Connect(const char *pszPort)
{
    int nErr;
    int nState;

    if(!m_pSerx)
        return ERR_COMMNOLINK;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Connect] Called." << std::endl;
    m_sLogFile.flush();
#endif

    // 19200 8N1
    nErr = m_pSerx->open(pszPort, 19200, SerXInterface::B_NOPARITY, "-DTR_CONTROL 1");
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Connect] Connection failed, nErr = " << nErr <<  std::endl;
        m_sLogFile.flush();
#endif
        m_bIsConnected = false;
        return nErr;
    }
    m_bIsConnected = true;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Connect] connected to " << pszPort << std::endl;
    m_sLogFile.flush();
#endif

    if (m_bDebugLog) {
        m_sLogBuffer.assign("[Connect] Connected.");
        m_pLogger->out(m_sLogBuffer.c_str());

        m_sLogBuffer.assign("[Connect] Getting Firmware.");
        m_pLogger->out(m_sLogBuffer.c_str());
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Connect] Getting Firmware." << std::endl;
    m_sLogFile.flush();
#endif

    // if this fails we're not properly connected.
    nErr = getFirmwareVersion(m_sFirmware);
    if(nErr) {
        if (m_bDebugLog) {
            m_sLogBuffer.assign("[Connect] Error Getting Firmware.");
            m_pLogger->out(m_sLogBuffer.c_str());
        }
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Connect] Error Getting Firmware : " << nErr << std::endl;
        m_sLogFile.flush();
#endif
        m_bIsConnected = false;
        m_pSerx->close();
        return ERR_COMMNOLINK;
    }

    if (m_bDebugLog) {
        m_sLogBuffer.assign("[Connect] Got Firmware.");
        m_pLogger->out(m_sLogBuffer.c_str());
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Connect]Got Firmware : "<<  m_sFirmware << std::endl;
    m_sLogFile.flush();
#endif

    // get dome home az and park az
    getDomeHomeAz(m_dHomeAz);
    getDomeParkAz(m_dParkAz);
    // get dome CPR and coast
    getDomeAzCPR(m_nNbStepPerRev);
    getDomeAzCoast(m_dAzCoast);

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Connect] m_nNbStepPerRev = " << m_nNbStepPerRev << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Connect] m_dHomeAz       = " << std::fixed << std::setprecision(2) << m_dHomeAz << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Connect] m_dParkAz       = " << std::fixed << std::setprecision(2) << m_dParkAz << std::endl;
    m_sLogFile.flush();
#endif

    // Check if the dome is at park
    getDomeLimits();
    if(m_nAtParkSate == ACTIVE) {
        nErr = getDomeParkAz(m_dCurrentAzPosition);
        if(!nErr)
            syncDome(m_dCurrentAzPosition, m_dCurrentElPosition);
    }

    nErr = getDomeShutterStatus(nState);
    nErr = getDomeLimits();

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Connect] m_dCurrentAzPositionr = " << std::fixed << std::setprecision(2) << m_dCurrentAzPosition << std::endl;
    m_sLogFile.flush();
#endif


    if(nState != NOT_FITTED )
        m_bHasShutter = true;

    return SB_OK;
}


void CDomePro::Disconnect()
{
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Disconnect] Called." << std::endl;
    m_sLogFile.flush();
#endif

    if(m_bIsConnected) {
        m_pSerx->purgeTxRx();
        m_pSerx->close();
    }
    m_bIsConnected = false;
}


#pragma mark - Dome API call

int CDomePro::syncDome(double dAz, double dEl)
{
    int nErr = PLUGIN_OK;
    int nPos;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [syncDome] Called." << std::endl;
    m_sLogFile.flush();
#endif

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    m_dCurrentAzPosition = dAz;
    AzToTicks(dAz, nPos);
    nErr = calibrateDomeAzimuth(nPos);
    return nErr;
}

int CDomePro::gotoDomePark(void)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [gotoDomePark] Called." << std::endl;
    m_sLogFile.flush();
#endif

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return nErr;

    nErr = domeCommand("!DSgp;", sResp);

    return nErr;
}

int CDomePro::unparkDome()
{
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [unparkDome] Called." << std::endl;
    m_sLogFile.flush();
#endif

    m_bParked = false;
    m_dCurrentAzPosition = m_dParkAz;

    syncDome(m_dCurrentAzPosition, m_dCurrentElPosition);
    return 0;
}

int CDomePro::gotoAzimuth(double dNewAz)
{

    int nErr = PLUGIN_OK;
    int nPos;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [gotoAzimuth] Called." << std::endl;
    m_sLogFile.flush();
#endif

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    AzToTicks(dNewAz, nPos);

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Connect] dNewAz = " << std::fixed << std::setprecision(2) << dNewAz << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Connect] nPos = " << nPos << std::endl;
    m_sLogFile.flush();
#endif

    nErr = goToDomeAzimuth(nPos);
    m_dGotoAz = dNewAz;
    m_nGotoTries = 0;
    return nErr;
}

int CDomePro::gotoElevation(double dNewEl)
{

    int nErr = PLUGIN_OK;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [gotoElevation] Called." << std::endl;
    m_sLogFile.flush();
#endif

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    m_nTargetAdc = (int) floor(0.5 + ((m_Shutter1CloseAngle - dNewEl) * m_ADC_Ratio1));

    nErr = goToDomeElevation(m_nTargetAdc, 0);

    m_dGotoEl = dNewEl;

    return nErr;
}


int CDomePro::openDomeShutters()
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [openDomeShutters] Called." << std::endl;
    m_sLogFile.flush();
#endif

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return SB_OK;

    nErr = domeCommand("!DSso;", sResp);
    return nErr;
}

int CDomePro::CloseDomeShutters()
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [CloseDomeShutters] Called." << std::endl;
    m_sLogFile.flush();
#endif

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return SB_OK;

    nErr = domeCommand("!DSsc;", sResp);
    return nErr;
}

int CDomePro::abortCurrentCommand()
{
    int nErr;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [abortCurrentCommand] Called." << std::endl;
    m_sLogFile.flush();
#endif

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
    int nErr = PLUGIN_OK;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [goHome] Called." << std::endl;
    m_sLogFile.flush();
#endif

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = homeDomeAzimuth();
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [goHome] ERROR : " << nErr << std::endl;
        m_sLogFile.flush();
#endif
    }

    return nErr;
}

#pragma mark TODO : Calibrate test
int CDomePro::learnAzimuthCprRight()
{
    int nErr = PLUGIN_OK;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [learnAzimuthCprRight] Called." << std::endl;
    m_sLogFile.flush();
#endif

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    // get the number of CPR going right.
    nErr = startDomeAzGaugeRight();

    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [learnAzimuthCprRight] ERROR : " << nErr << std::endl;
        m_sLogFile.flush();
#endif
    }
    m_bCalibrating = true;
    m_nLearning = RIGHT;
    return nErr;
}

int CDomePro::learnAzimuthCprLeft()
{
    int nErr = PLUGIN_OK;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [learnAzimuthCprLeft] Called." << std::endl;
    m_sLogFile.flush();
#endif

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    // get the number of CPR going right.
    nErr = startDomeAzGaugeLeft();
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [learnAzimuthCprLeft] ERROR : " << nErr << std::endl;
        m_sLogFile.flush();
#endif
    }

    m_bCalibrating = true;
    m_nLearning = LEFT;

    return nErr;
}

#pragma mark - dome controller informations

int CDomePro::getFirmwareVersion(std::string &sFirmware)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

    unsigned long nFirmwareVersion;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getFirmwareVersion] Called." << std::endl;
    m_sLogFile.flush();
#endif

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return SB_OK;

    nErr = domeCommand("!DGfv;", sResp);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getFirmwareVersion] ERROR : " << nErr << std::endl;
        m_sLogFile.flush();
#endif
        return nErr;
    }
    nFirmwareVersion =  std::stoul(sResp, nullptr, 16);
    sFirmware = std::to_string(nFirmwareVersion);
    return nErr;
}

int CDomePro::getModel(std::string &sModel)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getModel] Called." << std::endl;
    m_sLogFile.flush();
#endif

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return SB_OK;

    nErr = domeCommand("!DGhc;", sResp);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getModel] ERROR : " << nErr << std::endl;
        m_sLogFile.flush();
#endif
        return nErr;
    }

    m_nModel = std::stoi(sResp, nullptr, 16);
    switch(m_nModel) {
        case CLASSIC_DOME :
            sModel.assign("DomePro2-d");
            break;

        case CLAMSHELL :
            sModel.assign("DomePro2-c");
            break;

        case ROR :
            sModel.assign("DomePro2-r");
            break;

        default:
            sModel.assign("Unknown");
            break;
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getModel] Model : " << sModel << std::endl;
    m_sLogFile.flush();
#endif

    return nErr;
}

int CDomePro::getModelType()
{
    return m_nModel;
}

int CDomePro::getModuleType(int &nModuleType)
{
    int nErr;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getModuleType] Called." << std::endl;
    m_sLogFile.flush();
#endif

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return SB_OK;

    nErr = domeCommand("!DGmy;", sResp);
    if(nErr)
        return nErr;
    if(sResp.find("Az")!= std::string::npos) {
        m_nModuleType = MODULE_AZ;
    }
    if(sResp.find("Shut")!= std::string::npos) {
        m_nModuleType = MODULE_SHUT;
    }
    else {
        m_nModuleType = MODULE_UKNOWN;
    }

    return nErr;
}

int CDomePro::setDomeAzMotorPolarity(int nPolarity)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setDomeAzMotorPolarity] Called." << std::endl;
    m_sLogFile.flush();
#endif

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return SB_OK;

    m_nMotorPolarity = nPolarity;

    switch(m_nMotorPolarity) {
        case POSITIVE :
            nErr = domeCommand("!DSmpPositive;", sResp);
            break;
        case NEGATIVE :
            nErr = domeCommand("!DSmpNegative;", sResp);
            break;
        default:
            nErr = ERR_CMDFAILED;
            break;
    }

    return nErr;
}


int CDomePro::getDomeAzMotorPolarity(int &nPolarity)
{
    int nErr;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeAzMotorPolarity] Called." << std::endl;
    m_sLogFile.flush();
#endif

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return SB_OK;

    nErr = domeCommand("!DGmp;", sResp);
    if(nErr)
        return nErr;
    if(sResp.find("Positive")!= std::string::npos) {
        m_nMotorPolarity = POSITIVE;
    }
    else if(sResp.find("Negative")!= std::string::npos) {
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
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setDomeAzEncoderPolarity] Called." << std::endl;
    m_sLogFile.flush();
#endif

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return SB_OK;

    m_nAzEncoderPolarity = nPolarity;

    switch(m_nAzEncoderPolarity) {
        case POSITIVE :
            nErr = domeCommand("!DSepPositive;", sResp);
            break;

        case NEGATIVE :
            nErr = domeCommand("!DSepNegative;", sResp);
            break;

        default:
            nErr = ERR_CMDFAILED;
            break;
    }
    return nErr;
}

int CDomePro::getDomeAzEncoderPolarity(int &nPolarity)
{
    int nErr;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeAzEncoderPolarity] Called." << std::endl;
    m_sLogFile.flush();
#endif

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return SB_OK;

    nErr = domeCommand("!DGep;", sResp);
    if(nErr)
        return nErr;
    if(sResp.find("Positive")!= std::string::npos) {
        m_nAzEncoderPolarity = POSITIVE;
    }
    else if(sResp.find("Negative")!= std::string::npos) {
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
    std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isGoToComplete] Called." << std::endl;
    m_sLogFile.flush();
#endif

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = isDomeMoving(bIsMoving);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isGoToComplete] Error checking if dome is moving : "  << nErr << std::endl;
        m_sLogFile.flush();
#endif
        return nErr;
        }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isGoToComplete] Dome is moving : "  << (bIsMoving?"Yes":"No") << std::endl;
    m_sLogFile.flush();
#endif

    if(bIsMoving) {
        bComplete = false;
        return nErr;
    }

    getDomeAzPosition(dDomeAz);

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isGoToComplete] dDomeAz = " << std::fixed << std::setprecision(2) << dDomeAz << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isGoToComplete] m_dGotoAz = " << std::fixed << std::setprecision(2) << m_dGotoAz << std::endl;
    m_sLogFile.flush();
#endif

    if(checkBoundaries(m_dGotoAz, dDomeAz, m_dAzCoast+1)) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isGoToComplete] Goto finished" << std::endl;
        m_sLogFile.flush();
#endif
        bComplete = true;
        m_nGotoTries = 0;
    }
    else {
        // we're not moving and we're not at the final destination !!!
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isGoToComplete] we're not moving and we're not at the final destination !!!" << std::endl;
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isGoToComplete] dDomeAz = " << std::fixed << std::setprecision(2) << dDomeAz << std::endl;
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isGoToComplete] m_dGotoAz = " << std::fixed << std::setprecision(2) << m_dGotoAz << std::endl;
        m_sLogFile.flush();
#endif
        if (m_bDebugLog) {
            m_sLogBuffer.assign("[Connect] Got Firmware.");
            m_pLogger->out(m_sLogBuffer.c_str());
            ssTmp << "[isGoToComplete] dDomeAz = " << std::fixed << std::setprecision(2) << dDomeAz << ", m_dGotoAz = " << std::fixed << std::setprecision(2) << m_dGotoAz << std::endl;
            m_pLogger->out(ssTmp.str().c_str());
        }
        if(m_nGotoTries == 0) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isGoToComplete] Retrying Goto" << std::endl;
            m_sLogFile.flush();
#endif
            bComplete = false;
            m_nGotoTries = 1;
            gotoAzimuth(m_dGotoAz);
        }
        else {
            m_nGotoTries = 0;
            bComplete = false;
            nErr = ERR_CMDFAILED;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isGoToComplete] Goto error" << std::endl;
            m_sLogFile.flush();
#endif
        }
    }


#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isGoToComplete] bComplete : " << ( bComplete?"Yes":"No" ) << std::endl;
    m_sLogFile.flush();
#endif
    return nErr;
}


bool CDomePro::checkBoundaries(double dTargetAz, double dDomeAz, double nMargin)
{
    double highMark;
    double lowMark;
    double roundedGotoAz;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [checkBoundaries] Called." << std::endl;
    m_sLogFile.flush();
#endif

    // we need to test "large" depending on the heading error and movement coasting
    highMark = ceil(dDomeAz)+nMargin;
    lowMark = ceil(dDomeAz)-nMargin;
    roundedGotoAz = ceil(dTargetAz);

    if(lowMark < 0 && highMark > 0) { // we're close to 0 degre but above 0
        if((roundedGotoAz+2) >= 360)
            roundedGotoAz = (roundedGotoAz+2)-360;
        if ( (roundedGotoAz > lowMark) && (roundedGotoAz <= highMark)) {
            return true;
        }
    }
    if ( lowMark > 0 && highMark>360 ) { // we're close to 0 but from the other side
        if( (roundedGotoAz+360) > lowMark && (roundedGotoAz+360) <= highMark) {
            return true;
        }
    }
    if (roundedGotoAz > lowMark && roundedGotoAz <= highMark) {
        return true;
    }

    return false;
}


int CDomePro::isGoToElComplete(bool &bComplete)
{
    int nErr = 0;
    int nADC;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isGoToElComplete] Called." << std::endl;
    m_sLogFile.flush();
#endif

    bComplete = false;
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = getDomeShutter1_ADC(nADC);
    if(nErr)
        return nErr;

    if(m_nTargetAdc == nADC) {
        bComplete = true;
    }

    return nErr;
}

int CDomePro::isOpenComplete(bool &bComplete)
{
    int nErr = 0;
    int nState;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isOpenComplete] Called." << std::endl;
    m_sLogFile.flush();
#endif

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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isCloseComplete] Called." << std::endl;
    m_sLogFile.flush();
#endif

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    err = getDomeShutterStatus(nState);
    if(err) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isParkComplete] error." << std::endl;
        m_sLogFile.flush();
#endif
        return ERR_CMDFAILED;
    }
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
    int nMode;
    double dDomeAz=0;
    bool bIsMoving = false;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isParkComplete] Called." << std::endl;
    m_sLogFile.flush();
#endif

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = getDomeAzMoveMode(nMode);
    if(nMode == PARKING)
    {
        bComplete = false;
        return nErr;
    }

    getDomeAzPosition(dDomeAz);
    nErr = isDomeMoving(bIsMoving);
    if(nErr)
        return nErr;

    if(bIsMoving) { // this should not happen
        bComplete = false;
        return nErr;
    }

    if(checkBoundaries(m_dParkAz, dDomeAz, m_dAzCoast+1)) {
        m_bParked = true;
        bComplete = true;
    }
    else {
        // we're not moving and we're not at the final destination !!!
        bComplete = false;
        m_bHomed = false;
        m_bParked = false;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isParkComplete] error." << std::endl;
        m_sLogFile.flush();
#endif
        nErr = ERR_CMDFAILED;
    }

    return nErr;
}

int CDomePro::isUnparkComplete(bool &bComplete)
{
    int nErr = 0;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isUnparkComplete] Called." << std::endl;
    m_sLogFile.flush();
#endif

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
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isUnparkComplete] error checking if dome is moving : " << nErr << std::endl;
        m_sLogFile.flush();
#endif
        return nErr;
    }

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
        // did we just pass home
        if(!checkBoundaries(m_dHomeAz, m_dCurrentAzPosition, m_dAzCoast+1)) {
            m_nHomingTries = 0;
            gotoAzimuth(m_dHomeAz); // back out a bit
            bComplete = true;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isUnparkComplete] Close to home, backing out to : " << std::fixed << std::setprecision(2) << m_dHomeAz << std::endl;
            m_sLogFile.flush();
#endif
        }
        else {
            // we're not moving and we're not at the home position !!!
            if (m_bDebugLog) {
                m_pLogger->out("[isFindHomeComplete] Not moving and not at home !!!\n");
            }
            if(m_nHomingTries == 0) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
                m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isUnparkComplete] error, not moving and not at home. Retrying" << std::endl;
                m_sLogFile.flush();
#endif
                bComplete = false;
                m_nHomingTries = 1;
                gotoAzimuth(m_dHomeAz);
            }
            else {
                bComplete = false;
                m_bHomed = false;
                m_bParked = false;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
                m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isUnparkComplete] error, not moving and not at home" << std::endl;
                m_sLogFile.flush();
#endif
                nErr = ERR_CMDFAILED;
            }
        }
    }

    return nErr;
}


int CDomePro::isLearningCPRComplete(bool &bComplete)
{
    int nErr = PLUGIN_OK;
    int nMode;
    int nSteps;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isLearningCPRComplete] Called." << std::endl;
    m_sLogFile.flush();
#endif

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = getDomeAzMoveMode(nMode);
    if(nErr) {
        killDomeAzimuthMovement();
        m_bCalibrating = false;
        // restore previous value as there was an error
        m_nNbStepPerRev = m_nNbStepPerRev_save;
    }

    if(nMode == GAUGING)
    {
        bComplete = false;
        return nErr;
    }

    // Gauging is done. let's read the value
    if(m_nLearning == RIGHT) {
        nErr = getDomeAzGaugeRight(nSteps);
        m_nRightCPR = nSteps;
        }
    else {
        nErr = getDomeAzGaugeLeft(nSteps);
        m_nLeftCPR = nSteps;
    }
    if(nErr) {
        killDomeAzimuthMovement();
        m_bCalibrating = false;
        m_nLearning = 0;
        return nErr;
    }
    bComplete = true;
    return nErr;
}

int CDomePro::isPassingHomeComplete(bool &bComplete)
{
    int nErr = PLUGIN_OK;
    bComplete = false;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isPassingHomeComplete] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = getDomeLimits();
    if(nErr) {
        return nErr;
    }
    if(m_nAtHomeSwitchState != ACTIVE)
        bComplete = true;

    return nErr;
}


#pragma mark - Getter / Setter

int CDomePro::setHomeAz(double dAz)
{
    int nErr = PLUGIN_OK;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setHomeAz] Called." << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setHomeAz] Setting home Az to : " << std::fixed << std::setprecision(2) << dAz << std::endl;
    m_sLogFile.flush();
#endif

    m_dHomeAz = dAz;
    return nErr;
}

int CDomePro::setDomeAzCoast(double dAz)
{
    int nErr = PLUGIN_OK;
    int nPos;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setDomeAzCoast] Called." << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setDomeAzCoast] Setting Az coast to : " << std::fixed << std::setprecision(2) << dAz << std::endl;
    m_sLogFile.flush();
#endif

    m_dAzCoast = dAz;
    nPos = (int) ((16385/360) * dAz);
    nErr = setDomeAzCoast(nPos);
    return nErr;

}

int CDomePro::getDomeAzCoast(double &dAz)
{
    int nErr = PLUGIN_OK;
    int nPos;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeAzCoast] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = getDomeAzCoast(nPos);
    if(nErr)
        return nErr;

    dAz = (nPos/16385.0) * 360.0;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeAzCoast] Az coast : " << std::fixed << std::setprecision(2) << dAz << std::endl;
    m_sLogFile.flush();
#endif
    return nErr;
}


int CDomePro::setParkAz(double dAz)
{
    int nErr = PLUGIN_OK;
    int nPos;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setParkAz] Called." << std::endl;
    m_sLogFile.flush();
#endif

    if(!m_bIsConnected)
        return NOT_CONNECTED;


    m_dParkAz = dAz;

    AzToTicks(dAz, nPos);

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setParkAz] Park Az   : " << std::fixed << std::setprecision(2) << dAz << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setParkAz] Park nPos : " << nPos << std::endl;
    m_sLogFile.flush();
#endif

    setDomeParkAzimuth(nPos);
    return nErr;
}


double CDomePro::getCurrentAz()
{
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getCurrentAz] Called." << std::endl;
    m_sLogFile.flush();
#endif

    if(m_bIsConnected)
        getDomeAzPosition(m_dCurrentAzPosition);

    return m_dCurrentAzPosition;
}

double CDomePro::getCurrentEl()
{
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getCurrentEl] Called." << std::endl;
    m_sLogFile.flush();
#endif

    if(m_bIsConnected)
        getDomeEl(m_dCurrentElPosition);

    return m_dCurrentElPosition;
}

int CDomePro::getCurrentShutterState()
{
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getCurrentShutterState] Called." << std::endl;
    m_sLogFile.flush();
#endif
    if(m_bIsConnected)
        getDomeShutterStatus(m_nShutterState);

    return m_nShutterState;
}

void CDomePro::setShutterAngleCalibration(int nShutter1OpenAngle, int nShutter1rOpenAngleADC,
                                int nShutter1CloseAngle, int nShutter1CloseAngleADC,
                                int nShutter2OpenAngle, int nShutter2rOpenAngleADC,
                                int nShutter2CloseAngle, int nShutter2CloseAngleADC,
                                bool bShutterGotoEnabled)
{

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setShutterAngleCalibration] Called." << std::endl;
    m_sLogFile.flush();
#endif

    m_Shutter1OpenAngle = nShutter1OpenAngle;
    m_Shutter1OpenAngle_ADC = nShutter1rOpenAngleADC;
    m_Shutter1CloseAngle = nShutter1CloseAngle;
    m_Shutter1CloseAngle_ADC = nShutter1CloseAngleADC;
    m_ADC_Ratio1 = (m_Shutter1OpenAngle_ADC - m_Shutter1CloseAngle_ADC) / (m_Shutter1OpenAngle - m_Shutter1CloseAngle);

    m_Shutter2OpenAngle = nShutter2OpenAngle;
    m_Shutter2OpenAngle_ADC = nShutter2rOpenAngleADC;
    m_Shutter2CloseAngle = nShutter2CloseAngle;
    m_Shutter2CloseAngle_ADC = nShutter2CloseAngleADC;
    m_ADC_Ratio2 = (m_Shutter2OpenAngle_ADC - m_Shutter2CloseAngle_ADC) / (m_Shutter2OpenAngle - m_Shutter2CloseAngle);

    m_bShutterGotoEnabled = bShutterGotoEnabled;

}


void CDomePro::setDebugLog(bool bEnable)
{
    m_bDebugLog = bEnable;
}


#pragma mark - conversion functions

//	Convert pdAz to number of ticks from home.
void CDomePro::AzToTicks(double pdAz, int &ticks)
{
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [AzToTicks] Called." << std::endl;
    m_sLogFile.flush();
#endif

    if(!m_nNbStepPerRev)
        getDomeAzCPR(m_nNbStepPerRev);

    ticks = (int) floor(0.5 + (pdAz - m_dHomeAz) * m_nNbStepPerRev / 360.0);
    while (ticks > m_nNbStepPerRev) ticks -= m_nNbStepPerRev;
    while (ticks < 0) ticks += m_nNbStepPerRev;
}


// Convert ticks from home to Az
void CDomePro::TicksToAz(unsigned long ticks, double &pdAz)
{
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [TicksToAz] Called." << std::endl;
    m_sLogFile.flush();
#endif
    if(!m_nNbStepPerRev)
        getDomeAzCPR(m_nNbStepPerRev);

    pdAz = m_dHomeAz + (ticks * 360.0 / m_nNbStepPerRev);
    while (pdAz < 0) pdAz += 360;
    while (pdAz >= 360) pdAz -= 360;
}


#pragma mark - Dome movements

int CDomePro::setDomeLeftOn(void)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setDomeLeftOn] Called." << std::endl;
    m_sLogFile.flush();
#endif
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = domeCommand("!DSol;", sResp);
    if(nErr)
        return nErr;

    return nErr;
}

int CDomePro::setDomeRightOn(void)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setDomeRightOn] Called." << std::endl;
    m_sLogFile.flush();
#endif
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = domeCommand("!DSor;", sResp);
    if(nErr)
        return nErr;

    return nErr;
}

int CDomePro::killDomeAzimuthMovement(void)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [killDomeAzimuthMovement] Called." << std::endl;
    m_sLogFile.flush();
#endif
    if(!m_bIsConnected)
        return NOT_CONNECTED;


    nErr = domeCommand("!DXxa;", sResp);
    if(nErr)
        return nErr;

    return nErr;
}

#pragma mark - getter / setter

int CDomePro::getDomeAzPosition(double &dDomeAz)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    unsigned long nTmp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return nErr;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeAzPosition] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DGap;", sResp);
    if(nErr)
        return nErr;

    // convert Az hex string to long
    nTmp = std::stoul(sResp, nullptr, 16);

    TicksToAz(nTmp, dDomeAz);

    m_dCurrentAzPosition = dDomeAz;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeAzPosition] sResp  :" << sResp << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeAzPosition] nTmp    :" << nTmp << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeAzPosition] Park Az : " << std::fixed << std::setprecision(2) << dDomeAz << std::endl;
    m_sLogFile.flush();
#endif

    return nErr;
}

int CDomePro::getDomeEl(double &dDomeEl)
{
    int nErr = PLUGIN_OK;
    int nShutterState;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeEl] Called." << std::endl;
    m_sLogFile.flush();
#endif

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
    int nErr = PLUGIN_OK;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeHomeAz] Called." << std::endl;
    m_sLogFile.flush();
#endif

    dAz = m_dHomeAz;

    return nErr;
}

int CDomePro::getDomeParkAz(double &dAz)
{
    int nErr = PLUGIN_OK;
    int nPos;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeParkAz] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = getDomeParkAzimuth(nPos);
    if(nErr)
        return nErr;

    TicksToAz(nPos, dAz);

    return nErr;
}

int CDomePro::getDomeShutterStatus(int &nState)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    int nShutterState;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeShutterStatus] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DGsx;", sResp);
    if(nErr)
        return nErr;

    nShutterState = std::stoi(sResp, nullptr, 16);

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
    int nErr = PLUGIN_OK;
    int nMode;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isDomeMoving] Called." << std::endl;
    m_sLogFile.flush();
#endif

    bIsMoving = false;

    nErr = getDomeAzMoveMode(nMode);
    if(nErr)
        return nErr;

    if(nMode != FIXED && nMode != AZ_TO)
        bIsMoving = true;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isDomeMoving] moving :" << (bIsMoving?"Yes":"No") << std::endl;
    m_sLogFile.flush();
#endif

    return nErr;
}

int CDomePro::isDomeAtHome(bool &bAtHome)
{
    int nErr = PLUGIN_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isDomeAtHome] Called." << std::endl;
    m_sLogFile.flush();
#endif

    bAtHome = false;

    nErr = getDomeLimits();
    if(nErr) {
        return nErr;
    }
    if(m_nAtHomeState == ACTIVE)
        bAtHome = true;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isDomeAtHome] at home :" << (bAtHome?"Yes":"No") << std::endl;
    m_sLogFile.flush();
#endif

    return nErr;
}

#pragma mark - DomePro getter/setter

int CDomePro::setDomeAzCPR(int nValue)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setDomeAzCPR] Called." << std::endl;
    m_sLogFile.flush();
#endif

    // nCpr must be betweem 0x20 and 0x40000000 and be even
    if(nValue < 0x20 )
        nValue = 0x20;
    if(nValue>0x40000000)
        nValue = 0x40000000;
    nValue &= 0XFFFFFFFE; // makes it an even number

    ssTmp << "!DScp0x"<< std::uppercase << std::setfill('0') << std::setw(8) << std::hex << nValue <<";";

    nErr = domeCommand(ssTmp.str(), sResp);

    return nErr;
}

int CDomePro::getDomeAzCPR(int &nValue)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeAzCPR] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DGcp;", sResp);
    if(nErr)
        return nErr;

    // convert result hex string to long
    nValue = std::stoi(sResp, nullptr, 16);
    return nErr;
}

int CDomePro::getLeftCPR()
{
    return m_nLeftCPR;
}

int CDomePro::getRightCPR()
{
    return m_nRightCPR;

}


#pragma mark not yet implemented in the firmware
int CDomePro::setDomeMaxVel(int nValue)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setDomeMaxVel] Called." << std::endl;
    m_sLogFile.flush();
#endif

    // nValue must be betweem 0x01 and 0x7C (124)
    if(nValue < 0x1 )
        nValue = 0x1;
    if(nValue>0x7C)
        nValue = 0x7C;

    ssTmp << "!DSmv0x"<< std::uppercase << std::setfill('0') << std::setw(8) << std::hex << nValue <<";";
    nErr = domeCommand(ssTmp.str(), sResp);

    return nErr;
}

#pragma mark not yet implemented in the firmware
int CDomePro::getDomeMaxVel(int &nValue)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeMaxVel] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DGmv;", sResp);
    if(nErr)
        return nErr;

    // convert result hex string to long
    nValue = std::stoi(sResp, NULL, 16);
    return nErr;
}

#pragma mark not yet implemented in the firmware
int CDomePro::setDomeAccel(int nValue)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setDomeAccel] Called." << std::endl;
    m_sLogFile.flush();
#endif

    // nValue must be betweem 0x01 and 0xFF (255)
    if(nValue < 0x1 )
        nValue = 0x1;
    if(nValue>0xFF)
        nValue = 0xFF;

    ssTmp << "!DSma0x" << std::uppercase << std::setfill('0') << std::setw(8) << std::hex  << nValue <<";";
    nErr = domeCommand(ssTmp.str(), sResp);

    return nErr;
}

#pragma mark not yet implemented in the firmware
int CDomePro::getDomeAccel(int &nValue)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeAccel] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DGma;", sResp);
    if(nErr)
        return nErr;

    // convert result hex string to int
    nValue = std::stoi(sResp, NULL, 16);
    return nErr;
}



int CDomePro::setDomeAzCoast(int nValue)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;


#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setDomeAzCoast] Called." << std::endl;
    m_sLogFile.flush();
#endif

    // nCpr must be betweem 0x20 and 0x40000000 and be even
    if(nValue < 0x1 )
        nValue = 0x1;
    if(nValue>0x7C)
        nValue = 0x7C;

    ssTmp << "!DSco0x" << std::uppercase << std::setfill('0') << std::setw(8) << std::hex  << nValue <<";";
    nErr = domeCommand(ssTmp.str(), sResp);
    return nErr;
}

int CDomePro::getDomeAzCoast(int &nValue)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeAzCoast] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DGco;", sResp);
    if(nErr)
        return nErr;

    // convert result hex string to int
    nValue = std::stoi(sResp, NULL, 16);
    return nErr;
}

int CDomePro::getDomeAzDiagPosition(int &nValue)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeAzDiagPosition] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DGdp;", sResp);
    if(nErr)
        return nErr;

    // convert result hex string to int
    nValue = std::stoi(sResp, NULL, 16);
    return nErr;
}

int CDomePro::clearDomeAzDiagPosition(void)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [clearDomeAzDiagPosition] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DCdp;", sResp);
    return nErr;
}

int CDomePro::getDomeAzMoveMode(int &mode)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeAzMoveMode] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DGam;", sResp);
    if(nErr)
        return nErr;

    if(sResp.find("Fixed")!= std::string::npos) {
        mode = FIXED;
    }
    else if(sResp.find("Left")!= std::string::npos) {
        mode = LEFT;
    }
    else if(sResp.find("Right")!= std::string::npos) {
        mode = RIGHT;
    }
    else if(sResp.find("GoTo")!= std::string::npos) {
        mode = GOTO;
    }
    else if(sResp.find("Homing")!= std::string::npos ) {
        mode = HOMING;
    }
    else if(sResp.find("AzimuthTO")!= std::string::npos) {
        mode = AZ_TO;
    }
    else if(sResp.find("Gauging")!= std::string::npos) {
        mode = GAUGING;
    }
    else if(sResp.find("Parking")!= std::string::npos) {
        mode = PARKING;
    }
    return nErr;
}

int CDomePro::getDomeLimits(void)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    uint16_t nLimits;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeLimits] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DGdl;", sResp);
    if(nErr)
        return nErr;

    nLimits = (uint16_t) std::stoi(sResp, NULL, 16);

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeLimits] nLimits : " << nLimits << std::endl;
    m_sLogFile.flush();
#endif

    m_nShutter1OpenedSwitchState = (nLimits & BitShutter1_Opened ? ACTIVE : INNACTIVE);
    m_nShutter1ClosedSwitchState = (nLimits & BitShutter1_Closed ? ACTIVE : INNACTIVE);

    m_nShutter2OpenedSwitchState = (nLimits & BitShutter2_Opened ? ACTIVE : INNACTIVE);
    m_nShutter2ClosedSwitchState = (nLimits & BitShutter2_Closed ? ACTIVE : INNACTIVE);

    m_nAtHomeState = (nLimits & BitAtHome ? ACTIVE : INNACTIVE);
    m_nAtHomeSwitchState = (nLimits & BitHomeSwitchState ? ACTIVE : INNACTIVE);
    m_nAtParkSate = (nLimits & BitAtPark ? ACTIVE : INNACTIVE);


#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeLimits] m_nShutter1OpenedSwitchState : " << m_nShutter1OpenedSwitchState << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeLimits] m_nShutter1ClosedSwitchState : " << m_nShutter1ClosedSwitchState << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeLimits] m_nShutter2OpenedSwitchState : " << m_nShutter2OpenedSwitchState << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeLimits] m_nShutter2ClosedSwitchState : " << m_nShutter2ClosedSwitchState << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeLimits] m_nAtHomeState               : " << m_nAtHomeState << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeLimits] m_nAtHomeSwitchState         : " << m_nAtHomeSwitchState << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeLimits] m_nAtParkSate                : " << m_nAtParkSate << std::endl;
    m_sLogFile.flush();
#endif

    return nErr;
}


int CDomePro::setDomeHomeDirection(int nDir)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setDomeHomeDirection] Called." << std::endl;
    m_sLogFile.flush();
#endif

    if(nDir == LEFT) {
        nErr = domeCommand("!DShdLeft;", sResp);
    }
    else if (nDir == RIGHT) {
        nErr = domeCommand("!DShdRight;", sResp);
    }
    else {
        return INVALID_COMMAND;
    }

    return nErr;
}

int CDomePro::getDomeHomeDirection(int &nDir)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeHomeDirection] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DGhd;", sResp);
    if(nErr)
        return nErr;

    if(sResp.find("Left")!= std::string::npos) {
        nDir = LEFT;
    }
    else if(sResp.find("Right")!= std::string::npos) {
        nDir = RIGHT;
    }
    return nErr;
}


int CDomePro::setDomeHomeAzimuth(int nPos)
{

    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setDomeHomeAzimuth] Called." << std::endl;
    m_sLogFile.flush();
#endif

    if(nPos < 0 && nPos> m_nNbStepPerRev)
        return COMMAND_FAILED;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setDomeHomeAzimuth] nPos : " << nPos << std::endl;
    m_sLogFile.flush();
#endif

    ssTmp << "!DSha0x" << std::uppercase << std::setfill('0') << std::setw(8) << std::hex  << nPos <<";";
    nErr = domeCommand(ssTmp.str(), sResp);

    return nErr;
}

int CDomePro::setDomeAzimuthOCP_Limit(double dLimit)
{

    int nErr = PLUGIN_OK;
    int ulTmp;
    std::string sResp;
    std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setDomeAzimuthOCP_Limit] Called." << std::endl;
    m_sLogFile.flush();
#endif

    ulTmp = (int)floor((dLimit/0.0468f)+0.5);

    ssTmp << "!DSxa0x" << std::uppercase << std::setfill('0') << std::setw(8) << std::hex  << ulTmp <<";";
    nErr = domeCommand(ssTmp.str(), sResp);
    return nErr;
}

int CDomePro::getDomeAzimuthOCP_Limit(double &dLimit)
{
    int nErr = PLUGIN_OK;
    int ulTmp;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeAzimuthOCP_Limit] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DGxa;", sResp);
    if(nErr)
        return nErr;

    // convert result hex string to long
    ulTmp = std::stoi(sResp, NULL, 16);

    dLimit = (double)ulTmp * 0.0468f;

    return nErr;
}


int CDomePro::getDomeHomeAzimuth(int &nPos)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeHomeAzimuth] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DGha;", sResp);
    if(nErr)
        return nErr;

    // convert result hex string to long
    nPos = std::stoi(sResp, NULL, 16);

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeAzPosition] sResp : " << sResp << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeAzPosition] nPos   : " << nPos << std::endl;
    m_sLogFile.flush();
#endif

    return nErr;
}

int CDomePro::homeDomeAzimuth(void)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [homeDomeAzimuth] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DSah;", sResp);

    return nErr;
}


int CDomePro::goToDomeAzimuth(int nPos)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;

    if(nPos < 0 && nPos> m_nNbStepPerRev)
        return COMMAND_FAILED;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [goToDomeAzimuth] Called." << std::endl;
    m_sLogFile.flush();
#endif

    ssTmp << "!DSgo0x" << std::uppercase << std::setfill('0') << std::setw(8) << std::hex  << nPos <<";";
    nErr = domeCommand(ssTmp.str(), sResp);
    return nErr;
}

int CDomePro::goToDomeElevation(int nADC1, int nADC2)
{
    int nErr = PLUGIN_OK;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [goToDomeElevation] Called." << std::endl;
    m_sLogFile.flush();
#endif

    if(nADC1 < 0 && nADC2> 4095)
        return COMMAND_FAILED;

    if(nADC2 < 0 && nADC2> 4095)
        return COMMAND_FAILED;

    nErr = GoToDomeShutter1_ADC(nADC1);
    if(nErr)
        return nErr;
    nErr = GoToDomeShutter1_ADC(nADC2);
    if(nErr)
        return nErr;

    return nErr;
}

int CDomePro::GoToDomeShutter1_ADC(int nADC)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [GoToDomeShutter1_ADC] Called." << std::endl;
    m_sLogFile.flush();
#endif

    if(nADC < 0 && nADC> 4095)
        return COMMAND_FAILED;

    ssTmp << "!DSg10x" << std::uppercase << std::setfill('0') << std::setw(8) << std::hex  << nADC <<";";
    nErr = domeCommand(ssTmp.str(), sResp);
    return nErr;
}

int CDomePro::GoToDomeShutter2_ADC(int nADC)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [GoToDomeShutter2_ADC] Called." << std::endl;
    m_sLogFile.flush();
#endif

    if(nADC < 0 && nADC> 4095)
        return COMMAND_FAILED;

    ssTmp << "!DSg20x" << std::uppercase << std::setfill('0') << std::setw(8) << std::hex  << nADC <<";";
    nErr = domeCommand(ssTmp.str(), sResp);
    return nErr;
}


int CDomePro::setDomeParkAzimuth(int nPos)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setDomeParkAzimuth] Called." << std::endl;
    m_sLogFile.flush();
#endif

    if(nPos < 0 && nPos> m_nNbStepPerRev)
        return COMMAND_FAILED;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setDomeParkAzimuth] nPos   : " << nPos << std::endl;
    m_sLogFile.flush();
#endif

    ssTmp << "!DSpa0x" << std::uppercase << std::setfill('0') << std::setw(8) << std::hex  << nPos <<";";
    nErr = domeCommand(ssTmp.str(), sResp);
    return nErr;
}

int CDomePro::getDomeParkAzimuth(int &nPos)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeParkAzimuth] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DGpa;", sResp);
    if(nErr)
        return nErr;

    // convert result hex string to long
    nPos = std::stoi(sResp, NULL, 16);

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeParkAzimuth] sResp : " << sResp << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeParkAzimuth] nPos   : " << nPos << std::endl;
    m_sLogFile.flush();
#endif

    return nErr;
}

int CDomePro::calibrateDomeAzimuth(int nPos)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [calibrateDomeAzimuth] Called." << std::endl;
    m_sLogFile.flush();
#endif

    if(nPos < 0 && nPos> m_nNbStepPerRev)
        return COMMAND_FAILED;

    ssTmp << "!DSca0x" << std::uppercase << std::setfill('0') << std::setw(8) << std::hex  << nPos <<";";
    nErr = domeCommand(ssTmp.str(), sResp);
    return nErr;
}

int CDomePro::startDomeAzGaugeRight()
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [startDomeAzGaugeRight] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DSgr;", sResp);

    return nErr;
}

int CDomePro::getDomeAzGaugeRight(int &nSteps)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeAzGaugeRight] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DGgr;", sResp);
    if(nErr)
        return nErr;

    // convert result hex string to long
    nSteps = std::stoi(sResp, NULL, 16);
    if(!nSteps) { // if we get 0x00000000 there was an error
        // restore old value
        m_nNbStepPerRev = m_nNbStepPerRev_save;
        return ERR_CMDFAILED;
    }
    m_nNbStepPerRev = nSteps;

    return nErr;
}

int CDomePro::startDomeAzGaugeLeft()
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [startDomeAzGaugeLeft] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DSgl;", sResp);

    return nErr;
}

int CDomePro::getDomeAzGaugeLeft(int &nSteps)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeAzGaugeLeft] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DGgl;", sResp);
    if(nErr)
        return nErr;

    // convert result hex string to long
    nSteps = std::stoi(sResp, NULL, 16);
    if(!nSteps) { // if we get 0x00000000 there was an error
        // restore old value
        m_nNbStepPerRev = m_nNbStepPerRev_save;
        return ERR_CMDFAILED;
    }
    m_nNbStepPerRev = nSteps;

    return nErr;
}



int CDomePro::killDomeShutterMovement(void)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [killDomeShutterMovement] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DXxs;", sResp);
    if(nErr)
        return nErr;

    return nErr;
}

int CDomePro::getDomeDebug(std::string &sDebugStrBuff)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeDebug] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DGdg;", sResp);
    if(nErr)
        return nErr;

    sDebugStrBuff.assign(sResp);

    return nErr;
}

#pragma mark - low level dome data getter/setter

int CDomePro::getDomeSupplyVoltageAzimuthL(double &dVolts)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    unsigned int ulTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeSupplyVoltageAzimuthL] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DGva;", sResp);
    if(nErr)
        return nErr;

    // convert result hex string
    ulTmp = std::stoi(sResp, NULL, 16);

    dVolts = (double)ulTmp * 0.00812763;

    return nErr;
}

int CDomePro::getDomeSupplyVoltageShutterL(double &dVolts)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    unsigned int ulTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeSupplyVoltageShutterL] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DGvs;", sResp);
    if(nErr)
        return nErr;

    // convert result hex string
    ulTmp = std::stoi(sResp, NULL, 16);

    dVolts = (double)ulTmp * 0.00812763;
    
    return nErr;
}

#pragma mark FIX VOLTAGE MULTIPLIER
int CDomePro::getDomeSupplyVoltageAzimuthM(double &dVolts)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    unsigned int ulTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeSupplyVoltageAzimuthM] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DGoa;", sResp);
    if(nErr)
        return nErr;

    // convert result hex string
    ulTmp = std::stoi(sResp, NULL, 16);

    dVolts = (double)ulTmp * 0.00812763;

    return nErr;
}


#pragma mark FIX VOLTAGE MULTIPLIER
int CDomePro::getDomeSupplyVoltageShutterM(double &dVolts)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    unsigned int ulTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeSupplyVoltageShutterM] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DGos;", sResp);
    if(nErr)
        return nErr;

    // convert result hex string
    ulTmp = std::stoi(sResp, NULL, 16);

    dVolts = (double)ulTmp * 0.00812763;

    return nErr;
}

#pragma mark not yet implemented in the firmware
int CDomePro::getDomeRotationSenseAnalog(double &dVolts)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    unsigned int ulTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeRotationSenseAnalog] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DGra;", sResp);
    if(nErr)
        return nErr;

    // convert result hex string
    ulTmp = std::stoi(sResp, NULL, 16);

    dVolts = (double)ulTmp / 255 * 5; // FF = 5v, 0 = 0v
    
    return nErr;

}

int CDomePro::setDomeShutter1_OpTimeOut(int nTimeout)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;

    if(nTimeout < 10 && nTimeout > 500)
        return COMMAND_FAILED;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setDomeShutter1_OpTimeOut] Called." << std::endl;
    m_sLogFile.flush();
#endif

    ssTmp << "!DSt10x" << std::uppercase << std::setfill('0') << std::setw(8) << std::hex  << nTimeout <<";";
    nErr = domeCommand(ssTmp.str(), sResp);
    return nErr;
}

int CDomePro::getDomeShutter1_OpTimeOut(int &nTimeout)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeShutter1_OpTimeOut] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DGt1;", sResp);
    if(nErr)
        return nErr;

    // convert result hex string to long
    nTimeout = std::stoi(sResp, NULL, 16);

    return nErr;
}

int CDomePro::setDomeShutter2_OpTimeOut(int nTimeout)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setDomeShutter2_OpTimeOut] Called." << std::endl;
    m_sLogFile.flush();
#endif

    if(nTimeout < 10 && nTimeout > 500)
        return COMMAND_FAILED;

    ssTmp << "!DSt20x" << std::uppercase << std::setfill('0') << std::setw(8) << std::hex  << nTimeout <<";";
    nErr = domeCommand(ssTmp.str(), sResp);
    return nErr;

}

int CDomePro::getDomeShutter2_OpTimeOut(int &nTimeout)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

    nErr = domeCommand("!DGt2;", sResp);
    if(nErr)
        return nErr;

    // convert result hex string to long
    nTimeout = std::stoi(sResp, NULL, 16);

    return nErr;
}

int CDomePro::setDomeShutODirTimeOut(int nTimeout)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setDomeShutODirTimeOut] Called." << std::endl;
    m_sLogFile.flush();
#endif

    if(nTimeout < 10 && nTimeout > 500)
        return COMMAND_FAILED;

    ssTmp << "!DSto0x" << std::uppercase << std::setfill('0') << std::setw(8) << std::hex  << nTimeout <<";";
    nErr = domeCommand(ssTmp.str(), sResp);
    return nErr;
}

int CDomePro::getDomeShutODirTimeOut(int &nTimeout)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeShutODirTimeOut] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DGto;", sResp);
    if(nErr)
        return nErr;

    // convert result hex string to long
    nTimeout = std::stoi(sResp, NULL, 16);

    return nErr;
}

int CDomePro::setDomeAzimuthTimeOutEnabled(bool bEnable)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setDomeAzimuthTimeOutEnabled] Called." << std::endl;
    m_sLogFile.flush();
#endif

    if(bEnable)
        nErr = domeCommand("!DSaeYes;", sResp);
    else
        nErr = domeCommand("!DSaeNo;", sResp);

    return nErr;

}

int CDomePro::getDomeAzimuthTimeOutEnabled(bool &bEnable)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

    bEnable = false;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeAzimuthTimeOutEnabled] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DGae;", sResp);
    if(nErr)
        return nErr;
    if(sResp.find("Yes")!= std::string::npos)
        bEnable = true;

    return nErr;
}

int CDomePro::setDomeAzimuthTimeOut(int nTimeout)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setDomeAzimuthTimeOut] Called." << std::endl;
    m_sLogFile.flush();
#endif

    if(nTimeout < 10 && nTimeout > 500)
        return COMMAND_FAILED;

    ssTmp << "!DSta0x" << std::uppercase << std::setfill('0') << std::setw(8) << std::hex  << nTimeout <<";";
    nErr = domeCommand(ssTmp.str(), sResp);
    return nErr;
}

int CDomePro::getDomeAzimuthTimeOut(int &nTimeout)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeAzimuthTimeOut] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DGta;", sResp);
    if(nErr)
        return nErr;

    // convert result hex string to long
    nTimeout = std::stoi(sResp, NULL, 16);

    return nErr;
}

int CDomePro::setDomeShutCloseOnLinkTimeOut(bool bEnable)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setDomeShutCloseOnLinkTimeOut] Called." << std::endl;
    m_sLogFile.flush();
#endif


    if(bEnable)
        nErr = domeCommand("!DStsYes;", sResp);
    else
        nErr = domeCommand("!DStsNo;", sResp);

    return nErr;
    
}

int CDomePro::getDomeShutCloseOnLinkTimeOut(bool &bEnable)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

    bEnable = false;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeShutCloseOnLinkTimeOut] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DGts;", sResp);
    if(nErr)
        return nErr;
    if(sResp.find("Yes")!= std::string::npos)
        bEnable = true;

    return nErr;
}

int CDomePro::setDomeShutCloseOnClientTimeOut(bool bEnable)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setDomeShutCloseOnClientTimeOut] Called." << std::endl;
    m_sLogFile.flush();
#endif

    if(bEnable)
        nErr = domeCommand("!DSteYes;", sResp);
    else
        nErr = domeCommand("!DSteNo;", sResp);

    return nErr;
}

int CDomePro::getDomeShutCloseOnClientTimeOut(bool &bEnable)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

    bEnable = false;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeShutCloseOnClientTimeOut] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DGte;", sResp);
    if(nErr)
        return nErr;
    if(sResp.find("Yes")!= std::string::npos)
        bEnable = true;
    
    return nErr;
}

int CDomePro::setDomeShutCloseClientTimeOut(int nTimeout)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setDomeShutCloseClientTimeOut] Called." << std::endl;
    m_sLogFile.flush();
#endif

    if(nTimeout < 10 && nTimeout > 500)
        return COMMAND_FAILED;

    ssTmp << "!DStc0x" << std::uppercase << std::setfill('0') << std::setw(8) << std::hex  << nTimeout <<";";
    nErr = domeCommand(ssTmp.str(), sResp);
    return nErr;
}

int CDomePro::getDomeShutCloseClientTimeOut(int &nTimeout)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeShutCloseClientTimeOut] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DGtc;", sResp);
    if(nErr)
        return nErr;

    // convert result hex string to long
    nTimeout = std::stoi(sResp, NULL, 16);
    
    return nErr;
}

int CDomePro::setShutterAutoCloseEnabled(bool bEnable)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setShutterAutoCloseEnabled] Called." << std::endl;
    m_sLogFile.flush();
#endif

    if(bEnable)
        nErr = domeCommand("!DSanYes;", sResp);
    else
        nErr = domeCommand("!DSanNo;", sResp);

    return nErr;
}

int CDomePro::getShutterAutoCloseEnabled(bool &bEnable)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getShutterAutoCloseEnabled] Called." << std::endl;
    m_sLogFile.flush();
#endif

    bEnable = false;

    nErr = domeCommand("!DGan;", sResp);
    if(nErr)
        return nErr;
    if(sResp.find("Yes")!= std::string::npos)
        bEnable = true;

    return nErr;
}


#pragma mark not yet implemented in the firmware
int CDomePro::setDomeShutOpAtHome(bool bEnable)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setDomeShutOpAtHome] Called." << std::endl;
    m_sLogFile.flush();
#endif

    if(bEnable)
        nErr = domeCommand("!DSshYes;", sResp);
    else
        nErr = domeCommand("!DSshNo;", sResp);

    return nErr;
}

#pragma mark not yet implemented in the firmware
int CDomePro::getDomeShutOpAtHome(bool &bEnable)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeShutOpAtHome] Called." << std::endl;
    m_sLogFile.flush();
#endif

    bEnable = false;

    nErr = domeCommand("!DGsh;", sResp);
    if(nErr)
        return nErr;
    if(sResp.find("Yes")!= std::string::npos)
        bEnable = true;

    return nErr;
}

int CDomePro::getDomeShutdownInputState(bool &bEnable)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeShutdownInputState] Called." << std::endl;
    m_sLogFile.flush();
#endif

    bEnable = false;

    nErr = domeCommand("!DGsi;", sResp);
    if(nErr)
        return nErr;
    if(sResp.find("Yes")!= std::string::npos)
        bEnable = true;

    return nErr;
}

int CDomePro::getDomePowerGoodInputState(bool &bEnable)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomePowerGoodInputState] Called." << std::endl;
    m_sLogFile.flush();
#endif

    bEnable = false;

    nErr = domeCommand("!DGpi;", sResp);
    if(nErr)
        return nErr;
    if(sResp.find("Yes")!= std::string::npos)
        bEnable = true;

    return nErr;
}

#pragma mark not yet implemented in the firmware
int CDomePro::getLastDomeShutdownEvent(void)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getLastDomeShutdownEvent] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DGlv;", sResp);
    if(nErr)
        return nErr;

    // need to parse output and set some varaible/structure representing the event

    return nErr;
}

int CDomePro::setDomeSingleShutterMode(bool bEnable)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setDomeSingleShutterMode] Called." << std::endl;
    m_sLogFile.flush();
#endif

    if(bEnable)
        nErr = domeCommand("!DSssYes;", sResp);
    else
        nErr = domeCommand("!DSssNo;", sResp);

    return nErr;
}

int CDomePro::getDomeSingleShutterMode(bool &bEnable)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeSingleShutterMode] Called." << std::endl;
    m_sLogFile.flush();
#endif

    bEnable = false;

    nErr = domeCommand("!DGss;", sResp);
    if(nErr)
        return nErr;
    if(sResp.find("Yes")!= std::string::npos)
        bEnable = true;

    return nErr;
}

int CDomePro::getDomeLinkErrCnt(int &nErrCnt)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeLinkErrCnt] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DGle;", sResp);
    if(nErr)
        return nErr;

    // convert result hex string to long
    nErrCnt = std::stoi(sResp, NULL, 16);

    return nErr;
}

int CDomePro::clearDomeLinkErrCnt(void)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [clearDomeLinkErrCnt] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DCle;", sResp);
    if(nErr)
        return nErr;

    return nErr;
}

#pragma mark not yet implemented in the firmware
int CDomePro::getDomeComErr(void)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeComErr] Called." << std::endl;
    m_sLogFile.flush();
#endif


    nErr = domeCommand("!DGce;", sResp);
    if(nErr)
        return nErr;

    // need to parse output and set some varaible/structure representing the comms errors
    
    return nErr;
}

#pragma mark not yet implemented in the firmware
int CDomePro::clearDomeComErr(void)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [clearDomeComErr] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DCce;", sResp);
    if(nErr)
        return nErr;

    return nErr;
}

int CDomePro::openDomeShutter1(void)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [openDomeShutter1] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DSo1;", sResp);
    if(nErr)
        return nErr;

    return nErr;
}

int CDomePro::openDomeShutter2(void)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [openDomeShutter2] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DSo2;", sResp);
    if(nErr)
        return nErr;

    return nErr;
}

int CDomePro::closeDomeShutter1(void)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [closeDomeShutter1] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DSc1;", sResp);
    if(nErr)
        return nErr;

    return nErr;
}

int CDomePro::closeDomeShutter2(void)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [closeDomeShutter2] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DSc2;", sResp);
    if(nErr)
        return nErr;

    return nErr;
}

int CDomePro::stopDomeShutter1(void)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [stopDomeShutter1] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DSs1;", sResp);
    if(nErr)
        return nErr;

    return nErr;
}

int CDomePro::stopDomeShutter2(void)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

    nErr = domeCommand("!DSs2;", sResp);
    if(nErr)
        return nErr;

    return nErr;
}


int CDomePro::getDomeShutter1_ADC(int &nPos)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeShutter1_ADC] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DGa1;", sResp);
    if(nErr)
        return nErr;

    // convert result hex string to long
    nPos = std::stoi(sResp, NULL, 16);

    return nErr;
}

int CDomePro::getDomeShutter2_ADC(int &nPos)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeShutter2_ADC] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DGa2;", sResp);
    if(nErr)
        return nErr;

    // convert result hex string to long
    nPos = std::stoi(sResp, NULL, 16);

    return nErr;
}

int CDomePro::setDomeShutterOpenFirst(int nShutter)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setDomeShutterOpenFirst] Called." << std::endl;
    m_sLogFile.flush();
#endif
    ssTmp << "!DSof0x" << std::uppercase << std::setfill('0') << std::setw(2) << std::hex  << nShutter <<";";
    nErr = domeCommand(ssTmp.str(), sResp);
    return nErr;

}

int CDomePro::getDomeShutterOpenFirst(int &nShutter)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeShutterOpenFirst] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DGof;", sResp);
    if(nErr)
        return nErr;

    // convert result hex string to long
    nShutter = std::stoi(sResp, NULL, 16);

    return nErr;
}

int CDomePro::setDomeShutterCloseFirst(int nShutter)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setDomeShutterCloseFirst] Called." << std::endl;
    m_sLogFile.flush();
#endif

    ssTmp << "!DScf0x" << std::uppercase << std::setfill('0') << std::setw(2) << std::hex  << nShutter <<";";
    nErr = domeCommand(ssTmp.str(), sResp);

    return nErr;

}

int CDomePro::getDomeShutterCloseFirst(int &nShutter)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

    nErr = domeCommand("!DGcf;", sResp);
    if(nErr)
        return nErr;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeShutterCloseFirst] Called." << std::endl;
    m_sLogFile.flush();
#endif

    // convert result hex string to long
    nShutter = std::stoi(sResp, NULL, 16);
    
    return nErr;
}

int CDomePro::getDomeShutterMotorADC(double &dVolts)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    unsigned int ulTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeShutterMotorADC] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DGsc;", sResp);
    if(nErr)
        return nErr;

    // convert result hex string
    ulTmp = std::stoi(sResp, NULL, 16);

    dVolts = (double)ulTmp / 1023.0 * 3.3;
    dVolts = (dVolts - 1.721) / 0.068847;
    if (dVolts < 0.0)
        dVolts = 0.0;

    return nErr;
}

int CDomePro::getDomeAzimuthMotorADC(double &dVolts)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    unsigned int ulTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeAzimuthMotorADC] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DGac;", sResp);
    if(nErr)
        return nErr;

    // convert result hex string
    ulTmp = std::stoi(sResp, NULL, 16);

    dVolts = (double)ulTmp / 1023.0 * 3.3;
    dVolts = (dVolts - 1.721) / 0.068847;
    if (dVolts < 0.0)
        dVolts = 0.0;

    return nErr;
}

int CDomePro::getDomeShutterTempADC(double &dTemp)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    unsigned int ulTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeShutterTempADC] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DGst;", sResp);
    if(nErr)
        return nErr;

    // convert result hex string
    ulTmp = std::stoi(sResp, NULL, 16);

    dTemp = (double)ulTmp / 1023.0 * 3.3 - 0.5;
    dTemp = dTemp / 0.01;

    return nErr;
}

int CDomePro::getDomeAzimuthTempADC(double &dTemp)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    unsigned int ulTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeAzimuthTempADC] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DGat;", sResp);
    if(nErr)
        return nErr;

    // convert result hex string
    ulTmp = std::stoi(sResp, NULL, 16);

    dTemp = (double)ulTmp / 1023.0 * 3.3 - 0.5;
    dTemp = dTemp / 0.01;

    return nErr;
}

int CDomePro::setDomeShutOpOnHome(bool bEnabled)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setDomeShutOpOnHome] Called." << std::endl;
    m_sLogFile.flush();
#endif

    if(bEnabled)
        nErr = domeCommand("!DSshYes;", sResp);
    else
        nErr = domeCommand("!DSshNo;", sResp);

    return nErr;
}

int CDomePro::getDomeShutOpOnHome(bool &bEnabled)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeShutOpOnHome] Called." << std::endl;
    m_sLogFile.flush();
#endif

    bEnabled = false;

    nErr = domeCommand("!DGsh;", sResp);
    if(nErr)
        return nErr;

    if(sResp.find("Yes")!= std::string::npos)
        bEnabled = true;

    return nErr;
}


int CDomePro::setHomeWithShutterClose(bool bEnabled)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setHomeWithShutterClose] Called." << std::endl;
    m_sLogFile.flush();
#endif

    if(bEnabled)
        nErr = domeCommand("!DSchYes;", sResp);
    else
        nErr = domeCommand("!DSchNo;", sResp);

    return nErr;
}

int CDomePro::getHomeWithShutterClose(bool &bEnabled)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getHomeWithShutterClose] Called." << std::endl;
    m_sLogFile.flush();
#endif

    bEnabled = false;

    nErr = domeCommand("!DGch;", sResp);
    if(nErr)
        return nErr;

    if(sResp.find("Yes")!= std::string::npos)
        bEnabled = true;

    return nErr;
}

int CDomePro::setShutter1_LimitFaultCheckEnabled(bool bEnabled)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setShutter1_LimitFaultCheckEnabled] Called." << std::endl;
    m_sLogFile.flush();
#endif

    if(bEnabled)
        nErr = domeCommand("!DSl1Yes;", sResp);
    else
        nErr = domeCommand("!DSl1No;", sResp);

    return nErr;
}

int CDomePro::getShutter1_LimitFaultCheckEnabled(bool &bEnabled)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getShutter1_LimitFaultCheckEnabled] Called." << std::endl;
    m_sLogFile.flush();
#endif

    bEnabled = false;

    nErr = domeCommand("!DGl1;", sResp);
    if(nErr)
        return nErr;

    if(sResp.find("Yes")!= std::string::npos)
        bEnabled = true;

    return nErr;
}

int CDomePro::setShutter2_LimitFaultCheckEnabled(bool bEnabled)
{    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setShutter2_LimitFaultCheckEnabled] Called." << std::endl;
    m_sLogFile.flush();
#endif

    if(bEnabled)
        nErr = domeCommand("!DSl2Yes;", sResp);
    else
        nErr = domeCommand("!DSl2No;", sResp);

    return nErr;
}

int CDomePro::getShutter2_LimitFaultCheckEnabled(bool &bEnabled)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getShutter2_LimitFaultCheckEnabled] Called." << std::endl;
    m_sLogFile.flush();
#endif

    bEnabled = false;

    nErr = domeCommand("!DGl2;", sResp);
    if(nErr)
        return nErr;

    if(sResp.find("Yes")!= std::string::npos)
        bEnabled = true;

    return nErr;
}

int CDomePro::setDomeShutter1_OCP_Limit(double dLimit)
{
    int nErr = PLUGIN_OK;
    int ulTmp;
    std::string sResp;
    std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setDomeShutter1_OCP_Limit] Called." << std::endl;
    m_sLogFile.flush();
#endif

    ulTmp = (int)floor((dLimit/0.0468f)+0.5);

    ssTmp << "!DSx10x"<< std::uppercase << std::setfill('0') << std::setw(8) << std::hex  << ulTmp <<";";
    nErr = domeCommand(ssTmp.str(), sResp);

    return nErr;
}

int CDomePro::getDomeShutter1_OCP_Limit(double &dLimit)
{
    int nErr = PLUGIN_OK;
    int ulTmp;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeShutter1_OCP_Limit] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DGx1;", sResp);
    if(nErr)
        return nErr;

    // convert result hex string to long
    ulTmp = std::stoi(sResp, NULL, 16);

    dLimit = (double)ulTmp * 0.0468f;

    return nErr;

}

int CDomePro::setDomeShutter2_OCP_Limit(double dLimit)
{
    int nErr = PLUGIN_OK;
    int ulTmp;
    std::string sResp;
    std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setDomeShutter2_OCP_Limit] Called." << std::endl;
    m_sLogFile.flush();
#endif

    ulTmp = (int)floor((dLimit/0.0468f)+0.5);

    ssTmp << "!DSx20x"<< std::uppercase << std::setfill('0') << std::setw(8) << std::hex  << ulTmp <<";";
    nErr = domeCommand(ssTmp.str(), sResp);

    return nErr;
}

int CDomePro::getDomeShutter2_OCP_Limit(double &dLimit)
{
    int nErr = PLUGIN_OK;
    int ulTmp;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDomeShutter2_OCP_Limit] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = domeCommand("!DGx2;", sResp);
    if(nErr)
        return nErr;

    // convert result hex string to long
    ulTmp = std::stoi(sResp, NULL, 16);

    dLimit = (double)ulTmp * 0.0468f;

    return nErr;

}

int CDomePro::clearDomeLimitFault()
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [clearDomeLimitFault] Called." << std::endl;
    m_sLogFile.flush();
#endif


    nErr = domeCommand("!DClf;", sResp);
    return nErr;
}


#ifdef PLUGIN_DEBUG
const std::string CDomePro::getTimeStamp()
{
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    std::strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

    return buf;
}
#endif
