#include <iostream>
#include <winsock2.h>
#include <thread>
#include <conio.h>
#include <mutex>
#include <chrono>
#include <atomic>
#include <vector>
#include <cmath>
#include "HapticControl.h"
#include <fstream>
#include <filesystem>
#include "Eigen/Eigen"

#define USE_FOOT 1
#if USE_FOOT == 5
const int sina = 1;
const int sinb = -1;
#else
const int sina = -1;
const int sinb = 1;
#endif

//#define SAVE_SENSOR_DATA
// #define SEND_LINK_2_3
// #define USE_MASTER_PLANE

#pragma comment(lib, "ws2_32.lib")

const float PI = 3.14159265358979323846f;
#define RadToDeg 180/PI
#define DegToRad PI/180

struct sendStruct {
    int intValue;            //button state
    float floatValues[6];    //Position and Angular
};

struct recvStruct {
    int intValue;
    float floatValues[18];
};

const char* slave_addr = "192.168.14.255";

static hduVector3Dd prePos(0.0, 0.0, 0.0);
static std::chrono::high_resolution_clock::time_point previousTime;

HHD hHD;
hduVector3Dd fixedPosition;
hduVector3Dd GlobalMasterPos;
HDdouble GlobalAng[3];
HDint GlobalBtn;
std::vector<float> GlobalSlavePos(3);
std::vector<float> GlobalSlaveAng(3);
std::vector<float> GlobalSlaveForce(3);
std::vector<float> GlobalSlavePlane(3);// = {-1.0, 0.0, 0.0};
std::atomic<int> ContactFlag;
std::mutex recv_mutex, send_mutex;

//for callback
hduVector3Dd MasPos;
hduVector3Dd MasPos0;
hduVector3Dd MasVel;
HDdouble jointAngles[3];
HDint ButtonState;
HDint LastButtonState;
hduVector3Dd jointGimbal;
hduVector3Dd force;
std::vector<float> LocalSlavePos(3, 0.0f);
std::vector<float> LocalSlavePos0(3, 0.0f);
std::vector<float> LocalSlaveAng(3, 0.0f);
std::vector<float> LocalSlaveForce(3, 0.0f);
std::vector<float> LocalSlavePlane(3, 0.0f);
int LastContactFlag = 0;

std::vector<std::vector<float>> PointMatrix = { {0.0f, 0.0f, 0.0f},
                                                {0.0f, 0.0f, 0.0f},
                                                {0.0f, 0.0f, 0.0f} };
std::vector<float> n_plane = { 0.0f, 0.0f, 0.0f };
std::vector<float> TargetMasPos = { 0.0f, 0.0f, 0.0f };
std::vector<float> MasPosFromSlAng = { 0.0f, 0.0f, 0.0f };
float MasPosError = 0.0f;
int GetPointNum = 0;
bool PlaneGenerated = true;
bool resetPos = false;


void DToR(std::vector<float>& Vec);
void RToD(std::vector<float>& Vec);
void setValue(std::vector<float>& Vec, const std::vector<float>& setVec);
void setValue(std::vector<float>& Vec, const hduVector3Dd& setVec);
void setValue(hduVector3Dd& Vec, const hduVector3Dd& setVec);
void setValue(hduVector3Dd& Vec, const std::vector<float>& setVec);
void setValue(std::vector<float>& Vec, const std::vector<HDdouble>& setVec);
void setZero(hduVector3Dd& Vec);
std::vector<float> MasAngToMasPos(const std::vector<float>& Ang);
std::vector<float> MasPosToMasAng(const std::vector<float>& Ang);
std::vector<float> SlAngToMasAng(const std::vector<float>& Ang);
std::vector<float> MasAngToSlAng(const std::vector<float>& Ang);
void MasToSl(hduVector3Dd& Vec);//Change Coordinate system
void MasToSl(std::vector<float>& Vec);
void SlToMas(hduVector3Dd& Vec);//Change Coordinate system
void SlToMas(std::vector<float>& Vec);
float VecError(const std::vector<float>& vec1, const std::vector<float>& vec2);
float norm(const std::vector<float>& vec);
float norm(const hduVector3Dd& vec);
void setPlane(const std::vector<std::vector<float>>& Matrix);
hduVector3Dd setPlaneForce(const hduVector3Dd& MP, const std::vector<float>& SP, const int& CF, const int& LCF, const HDint& BS, const HDint& LBS, const hduVector3Dd& Vel);
float runtime;
auto start_time = std::chrono::high_resolution_clock::now();
/*#region haptic*/
HDCallbackCode HDCALLBACK phantomCallback(void* pUserData)
{
    hdBeginFrame(hHD);
    hdGetDoublev(HD_CURRENT_POSITION, MasPos);
    // hdGetDoublev(HD_CURRENT_VELOCITY, MasVel);
    hdGetDoublev(HD_CURRENT_JOINT_ANGLES, jointAngles);
    hdGetIntegerv(HD_CURRENT_BUTTONS, &ButtonState);
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, jointGimbal);
    if (HD_DEVICE_ERROR(hdGetError())) {
        std::cerr << "Failed to initialize haptic device." << std::endl;
    }

    auto currentTime = std::chrono::high_resolution_clock::now();
    runtime = std::chrono::duration<float>(currentTime - start_time).count();
    double dt = std::chrono::duration<double>(currentTime - previousTime).count();
    if (previousTime.time_since_epoch().count() > 0 && dt > 0) {
        MasVel = (MasPos - prePos) / dt;
    }
    previousTime = currentTime;
    prePos = MasPos;

    // std::cout << MasPos[0] << ", " << MasPos[1] << ", " << MasPos[2] << std::endl;
    // std::cout << MasVel[0] << ", " << MasVel[1] << ", " << MasVel[2] << std::endl;
    std::cout << jointAngles[0] * RadToDeg << ", " << jointAngles[1] * RadToDeg << ", " << jointAngles[2] * RadToDeg << std::endl; // for calibration
    {
        std::lock_guard<std::mutex> lock(recv_mutex);
        LocalSlaveForce = GlobalSlaveForce;
        LocalSlavePos = GlobalSlavePos;
        LocalSlaveAng = GlobalSlaveAng;
        LocalSlavePlane = GlobalSlavePlane;
    }

    SlToMas(LocalSlavePos);
    SlToMas(LocalSlavePlane);
    SlToMas(LocalSlaveForce);

    //angle control
    LocalSlaveAng = SlAngToMasAng(LocalSlaveAng); //Slave Angular in Master Coordinate
    TargetMasPos = MasAngToMasPos(LocalSlaveAng); //Manipulator Position in Master Coordinate
    MasPosFromSlAng = MasAngToMasPos(LocalSlaveAng);

    if (ButtonState != 0 && LastButtonState == 0) {
        setValue(LocalSlavePos0, LocalSlavePos);
        setValue(MasPos0, MasPos);
    }

    MasPosError = 0.0f;
    for (int i = 0; i < 3; ++i) {
        MasPosError += (TargetMasPos[i] - MasPos[i]) * (TargetMasPos[i] - MasPos[i]);
    }
    if (MasPosError < 4) {
        setValue(TargetMasPos, MasPos);
    }

    if (PlaneGenerated) {
        setValue(n_plane, LocalSlavePlane);
        // std::cout << "n_plane in master coordinate: " << n_plane[0] << ", " << n_plane[1] << ", " << n_plane[2] << std::endl;
        force = setPlaneForce(MasPos, LocalSlavePos, ContactFlag, LastContactFlag, ButtonState, LastButtonState, MasVel);
    }
    else {
        if (ButtonState == 0x00) {
            setZero(force);
        }
        else {
            if (ContactFlag == 1 && LastContactFlag == 0) {
                resetPos = true;
                std::cout << "start returning base position" << std::endl;
                for (int i = 0; i < 3; ++i) {
                    PointMatrix[GetPointNum][i] = MasPosFromSlAng[i]; //get Contact Point                        
                }
                GetPointNum++;
                if (GetPointNum == 3) {
                    PlaneGenerated = true;
                }
            }
            else if (ContactFlag == 0) {
                for (int i = 0; i < 3; ++i) {
                    if (ButtonState == 0x01) {
                        force[i] = ((LocalSlavePos[i] - LocalSlavePos0[i]) - (MasPos[i] - MasPos0[i]) * 0.001) * 1.0 - MasVel[i] * 0.003;
                    }
                    else if (ButtonState == 0x02) {
                        force[i] = -(MasPos[i] - TargetMasPos[i]) * 0 - MasVel[i] * 0.003;
                    }
                }
            }
            else if (ContactFlag == 1 && LastContactFlag == 1) {
                if (ButtonState == 0x01) {
                    for (int i = 0; i < 3; ++i) {
                        force[i] = LocalSlaveForce[i] * 10 - MasVel[i] * 0.003;
                    }
                }
                else if (ButtonState == 0x02) {
                    for (int i = 0; i < 3; ++i) {
                        force[i] = -(MasPos[i] - TargetMasPos[i]) * 0.5 - MasVel[i] * 0.003;
                    }
                }
            }
        }
    }


    if (resetPos) {
        if (ButtonState == 0x01) {
            for (int i = 0; i < 3; ++i) {
                force[i] = -((LocalSlavePos[i] - LocalSlavePos0[i]) - (MasPos[i] - MasPos0[i]) * 0.001) * 1.5;
            }
        }
        else if (ButtonState == 0x02) {
            for (int i = 0; i < 3; ++i) {
                force[i] = -(MasPos[i] - TargetMasPos[i]) * 0.5 - MasVel[i] * 0.003;
            }
        }

        if (norm(force) < 0.3) resetPos = false;
        ButtonState = 0x04;
    }



    hdSetDoublev(HD_CURRENT_FORCE, force);

    {
        std::lock_guard<std::mutex> lock(send_mutex);
        for (int i = 0; i < 3; ++i) {
            GlobalMasterPos[i] = MasPos[i];
            GlobalAng[i] = jointAngles[i];
            GlobalBtn = ButtonState;
        }
    }

    hdEndFrame(hHD);

    LastContactFlag = ContactFlag;
    LastButtonState = ButtonState;
    Sleep(10);
    return HD_CALLBACK_CONTINUE;
}
/*#endregion*/

/*#region Send*/
void sendData(SOCKET udpSocket, sockaddr_in& otherAddr, HHD hHD) {

    std::vector<float> LocalMasterPos(3, 0.0f);
    std::vector<float> LocalAng(3, 0.0f);
    std::vector<float> CalAng;
    HDint LocalBtn;
    int iBtnState = 0;
    int LastiBtnState = 0;
    float link2_3 = 0.0;
    float x0, y0, z0;
    std::vector<float> p0;
    float dx = 0.0;
    float dy = 0.0;
    float dz = 0.0;
    std::vector<float> dp0;
    std::vector<float> lastdp;

    while (true) {
        if (_kbhit() && _getch() == 'q') {
            std::cout << "Send thread quitting..." << std::endl;
            break;
        }

        {
            std::lock_guard<std::mutex> lock(send_mutex);
            for (int i = 0; i < 3; ++i) {
                LocalMasterPos[i] = GlobalMasterPos[i];
                LocalAng[i] = GlobalAng[i];
            }
            LocalBtn = GlobalBtn;
        }
        MasToSl(LocalMasterPos);
        switch (LocalBtn) {
        case 0x00:
            iBtnState = 0;
            break;
        case 0x01:
            iBtnState = 1;
            break;
        case 0x02:
            iBtnState = 2;
            break;
        case 0x03:
            iBtnState = 3;
            break;
        case 0x04:
            iBtnState = 4;
            break;
        }

        if (iBtnState != LastiBtnState && LastiBtnState == 0) {
            x0 = LocalMasterPos[0];
            y0 = LocalMasterPos[1];
            z0 = LocalMasterPos[2];
        }
        else if (iBtnState == 0) {
            x0 = LocalMasterPos[0];
            y0 = LocalMasterPos[1];
            z0 = LocalMasterPos[2];
        }
        dx = LocalMasterPos[0] - x0;
        dy = LocalMasterPos[1] - y0;
        dz = LocalMasterPos[2] - z0;

        //normalize
        LocalAng = MasAngToSlAng(LocalAng);
        dx = 0.001 * dx;
        dy = 0.001 * dy;
        dz = 0.001 * dz;
        LastiBtnState = iBtnState;
        sendStruct SendStruct = { iBtnState, {dx, dy, dz, LocalAng[0], LocalAng[1], LocalAng[2]} };

        int sendResult = sendto(udpSocket, reinterpret_cast<char*>(&SendStruct), sizeof(SendStruct), 0, (sockaddr*)&otherAddr, sizeof(otherAddr));
        if (sendResult == SOCKET_ERROR) {
            std::cerr << "Send failed. Error Code: " << WSAGetLastError() << std::endl;
        }
        else {
            // std::cout << "Sent: " 
                    //   << SendStruct.intValue 
                    //   << " " << SendStruct.floatValues[0] 
                    //   << " " << SendStruct.floatValues[1] 
                    //   << " " << SendStruct.floatValues[2]
                    //   << "\n " << SendStruct.floatValues[3]
                    //   << "\n " << SendStruct.floatValues[4]
                    //   << "\n " << SendStruct.floatValues[5]
                    //   << std::endl;
        }
        Sleep(10);
    }
}
/*#endregion*/

/*#region Receive*/
void receiveData(SOCKET udpSocket) {
#ifdef SAVE_SENSOR_DATA
    std::string filename = "experiment3.csv";
    std::ofstream outfile(filename);
    if (!outfile.is_open()) {
        std::cerr << "Failed to open file" << std::endl;
        return;
    }
    outfile << "t,x_mani,y_mani,z_mani,x_no,y_no,z_no,theta0,theta1,theta2,CF,force0,force1,force2,f_leg1,f_leg2,f_leg3,nx,ny,nz" << std::endl;
#endif
    sockaddr_in senderAddr;
    int senderAddrSize = sizeof(senderAddr);
    recvStruct recvData;
    std::vector<float> xyz(3);
    std::vector<float> xyz_disire(3);
    std::vector<float> xyz_no(3);
    std::vector<float> theta(3); //Rad
    std::vector<float> joint(3); //Deg
    std::vector<float> force(3); //fx(Mx), fy(My), fz
    std::vector<float> force_leg(3);
    std::vector<float> plane(3);

    while (true) {
        memset(&recvData, 0, sizeof(recvData));

        int recvLen = recvfrom(udpSocket, (char*)&recvData, sizeof(recvData), 0, (sockaddr*)&senderAddr, &senderAddrSize);
        if (recvLen == SOCKET_ERROR) {
            continue;
        }
        else {
            ContactFlag = recvData.intValue;
            for (int i = 0; i < 3; ++i) {
                xyz[i] = recvData.floatValues[i];
                theta[i] = recvData.floatValues[i + 3];
                force[i] = recvData.floatValues[i + 6];
                force_leg[i] = recvData.floatValues[i + 9];
                // xyz_disire[i] = recvData.floatValues[i + 9];
                plane[i] = recvData.floatValues[i + 12];
                xyz_no[i] = recvData.floatValues[i + 15];
            }
        }

        for (int i = 0; i < 3; ++i) {
            joint[i] = theta[i] * RadToDeg;
        }

        {
            std::lock_guard<std::mutex> lock(recv_mutex);
            for (int i = 0; i < 3; ++i) {
                GlobalSlavePos[i] = xyz[i];
                GlobalSlaveAng[i] = theta[i];
                GlobalSlaveForce[i] = force[i];
                GlobalSlavePlane[i] = plane[i];
            }
        }

        // std::cout << "Received Data:"
                //   << "Contact Flag: " << static_cast<int>(recvData.intValue)
                //   << "\njoint 0: " << joint_0
                //   << "\nManipulator Position is"
                //   << "\nx: " << xyz[0]
                //   << "\ny: " << xyz[1]
                //   << "\nz: " << xyz[2]
                //   << "\nrobot joint 1: " << joint[0]
                //   << "\nrobot joint 2: " << joint[1]
                //   << "\nrobot joint 3: " << joint[2]
                //   << "\noperation force x: " << force[0]
                //   << "\noperation force y: " << force[1]
                //   << "\noperation force z: " << force[2]
                //   << std::endl;


#ifdef SAVE_SENSOR_DATA
        outfile << runtime << "," << xyz[0] << "," << xyz[1] << "," << xyz[2] << "," << xyz_no[0] << "," << xyz_no[1] << "," << xyz_no[2] << "," << theta[0] << "," << theta[1] << "," << theta[2] << "," << ContactFlag << "," << force[0] << "," << force[1] << "," << force[2] << "," << force_leg[0] << "," << force_leg[1] << "," << force_leg[2] << "," << plane[0] << "," << plane[1] << "," << plane[2] << std::endl;
#endif
        // Sleep(1);
    }
#ifdef SAVE_SENSOR_DATA
    outfile.close();
#endif
}
/*#endregion*/

int main() {
    ContactFlag = 0;
    hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(hdGetError())) {
        std::cerr << "Failed to initialize haptic device." << std::endl;
        return 0;
    }

    hdEnable(HD_FORCE_OUTPUT);

    hdStartScheduler();
    if (HD_DEVICE_ERROR(hdGetError())) {
        std::cerr << "Failed to start scheduler" << std::endl;
        return 0;
    }

    hdScheduleAsynchronous(phantomCallback, 0, HD_DEFAULT_SCHEDULER_PRIORITY);

    /*#region UDP CONNECTION SETTING*/
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        std::cerr << "Winsock initialization failed. Error Code: " << WSAGetLastError() << std::endl;
        return 1;
    }

    SOCKET recvSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (recvSocket == INVALID_SOCKET) {
        std::cerr << "Receive socket creation failed. Error Code: " << WSAGetLastError() << std::endl;
        WSACleanup();
        return 1;
    }

    sockaddr_in recvAddr;
    recvAddr.sin_family = AF_INET;
    recvAddr.sin_port = htons(12345);
    recvAddr.sin_addr.s_addr = INADDR_ANY;

    if (bind(recvSocket, (sockaddr*)&recvAddr, sizeof(recvAddr)) == SOCKET_ERROR) {
        std::cerr << "Bind failed for receiving socket. Error Code: " << WSAGetLastError() << std::endl;
        closesocket(recvSocket);
        WSACleanup();
        return 1;
    }

    SOCKET sendSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sendSocket == INVALID_SOCKET) {
        std::cerr << "Send socket creation failed. Error Code: " << WSAGetLastError() << std::endl;
        closesocket(recvSocket);
        WSACleanup();
        return 1;
    }

    sockaddr_in otherAddr;
    otherAddr.sin_family = AF_INET;
    otherAddr.sin_port = htons(54321);
    otherAddr.sin_addr.s_addr = inet_addr(slave_addr);

    std::thread receiveThread(receiveData, recvSocket);
    std::thread sendThread(sendData, sendSocket, std::ref(otherAddr), hHD);

    sendThread.join();
    receiveThread.detach();

    closesocket(recvSocket);
    closesocket(sendSocket);
    WSACleanup();
    /*#endregion*/

    return 0;
}

void DToR(std::vector<float>& Vec) {
    for (int i = 0; i < Vec.size(); ++i) {
        Vec[i] = Vec[i] * DegToRad;
    }
}

void RToD(std::vector<float>& Vec) {
    for (int i = 0; i < Vec.size(); ++i) {
        Vec[i] = Vec[i] * RadToDeg;
    }
}


void setValue(std::vector<float>& Vec, const std::vector<float>& setVec) {
    for (int i = 0; i < setVec.size(); ++i) {
        Vec[i] = setVec[i];
    }
    return;
}

void setValue(std::vector<float>& Vec, const hduVector3Dd& setVec) {
    for (int i = 0; i < 3; ++i) {
        Vec[i] = setVec[i];
    }
    return;
}

void setValue(hduVector3Dd& Vec, const hduVector3Dd& setVec) {
    for (int i = 0; i < 3; ++i) {
        Vec[i] = setVec[i];
    }
    return;
}

void setValue(hduVector3Dd& Vec, const std::vector<float>& setVec) {
    for (int i = 0; i < 3; ++i) {
        Vec[i] = setVec[i];
    }
    return;
}

void setValue(std::vector<float>& Vec, const std::vector<HDdouble>& setVec) {
    for (int i = 0; i < 3; ++i) {
        Vec[i] = setVec[i];
    }
    return;
}

void setZero(hduVector3Dd& Vec) {
    for (int i = 0; i < 3; ++i) {
        Vec[i] = 0.0;
    }
    return;
}

std::vector<float> MasAngToMasPos(const std::vector<float>& Ang) {
    std::vector<float> Pos(3); //Body Coordinate
    float l3, l2, r_theta, x, y, z;
    l3 = 133.35;
    l2 = 133.35;
    r_theta = l2 * std::cos(Ang[1]) + l3 * std::sin(Ang[2]);
    x = -r_theta * std::sin(Ang[0]);
    y = l2 * std::sin(Ang[1]) - l3 * std::cos(Ang[2]) + 23.3501;
    z = r_theta * std::cos(Ang[0]) - 168.35;

    Pos[0] = x;
    Pos[1] = y;
    Pos[2] = z;

    return Pos;
}

std::vector<float> MasPosToMasAng(const std::vector<float>& Pos) {
    //Haptic Coordinate
    std::vector<float> Ang(3);
    float A, B, C;
    float phi;
    float y0 = 23.3501;
    float z0 = -168.35;
    float l3 = 133.35;
    float l2 = 133.35;

    Ang[0] = atan(-Pos[0] / (Pos[2] - z0));
    A = asin((pow((Pos[2] - z0) / cos(Ang[0]), 2) + pow((Pos[1] - y0), 2) - l2 * l2 - l3 * l3) / (2 * l2 * l3));
    B = l2 + l3 * sin(A);
    C = -l3 * cos(A);
    phi = atan(C / B);
    Ang[1] = asin((Pos[1] - y0) / std::sqrt(B * B + C * C)) - phi;
    Ang[2] = Ang[1] + A;

    return Ang;
}

std::vector<float> SlAngToMasAng(const std::vector<float>& Ang) {
    std::vector<float> MasAng(3);
    float a1, a2, b1, b2, x10, x20;
    a1 = -1;
    b1 = -0.357967; // -20.51 [deg];
    x10 = 1.570796; //  90    [deg];
    a2 = -1.211;
    b2 = 0.6876597; //  39.4  [deg];
    x20 = 2.844887; // 163;   [deg];
    MasAng[0] = Ang[0];
#ifdef SEND_LINK_2_3
    MasAng[1] = (Ang[1] - b1) / a1 + x10; //70 - Ang[1];
    float link2_3 = (Ang[2] - b2) / a2 + x20;
    MasAng[2] = link2_3 - 90 + Ang[1];
#else
    MasAng[1] = 1.134464 - Ang[1]; //65 - Ang[1]; [deg]
    MasAng[2] = 2.3561945 - Ang[1] - Ang[2]; //75 - (Ang[1] - 20) - (Ang[2] - 40);
#endif

    return MasAng;
}

std::vector<float> MasAngToSlAng(const std::vector<float>& Ang) {
    std::vector<float> SlAng(3);
    float a1, a2, b1, b2, x10, x20;
    a1 = -1;
    b1 = -0.357967; // -20.51 [deg];
    x10 = 1.570796; //  90    [deg];
    a2 = -1.211;
    b2 = 0.6876597; //  39.4  [deg];
    x20 = 2.844887; // 163;   [deg];
    float link2_3 = 1.570796 - Ang[1] + Ang[2]; //90 - Ang[1] + Ang[2];//theta(link2-link3) [deg]
    SlAng[0] = Ang[0];
#ifdef SEND_LINK_2_3
    SlAng[1] = a1 * (Ang[1] - x10) + b1;
    SlAng[2] = a2 * (link2_3 - x20) + b2;
#else
    SlAng[1] = 1.134464 - Ang[1]; //45 - Ang[1] + 20; [deg]
    SlAng[2] = 2.3561945 - Ang[2] - SlAng[1]; //75 - Ang[1] - (SlAng[1] - 20) + 40;
#endif
    return SlAng;
}

float VecError(const std::vector<float>& vec1, const std::vector<float>& vec2) {
    float error = 0.0;
    for (int i = 0; i < 3; ++i) {
        error += (vec1[i] - vec2[i]) * (vec1[i] - vec2[i]);
    }
    return error;
}

float norm(const std::vector<float>& vec) {
    float norm = 0.0f;
    for (int i = 0; i < 3; ++i) {
        norm += vec[i] * vec[i];
    }
    return std::sqrt(norm);
}

float norm(const hduVector3Dd& vec) {
    float norm = 0.0f;
    for (int i = 0; i < 3; ++i) {
        norm += vec[i] * vec[i];
    }
    return std::sqrt(norm);
}

void MasToSl(hduVector3Dd& Vec) {
    //from Haptic Coordinate to Body Coordinate
    float x = Vec[0];
    float y = Vec[1];
    float z = Vec[2];
    Vec[0] = -z;
    Vec[1] = -x;
    Vec[2] = y;

    Vec[0] = 0.7071 * Vec[0] + sinb * 0.7071 * Vec[1];
    Vec[1] = sina * 0.7071 * Vec[0] + 0.7071 * Vec[1];

    return;
}

void MasToSl(std::vector<float>& Vec) {
    float x = Vec[0];
    float y = Vec[1];
    float z = Vec[2];
    Vec[0] = -z;
    Vec[1] = -x;
    Vec[2] = y;

    Vec[0] = 0.7071 * Vec[0] + sinb * 0.7071 * Vec[1];
    Vec[1] = sina * 0.7071 * Vec[0] + 0.7071 * Vec[1];

    return;
}

void SlToMas(hduVector3Dd& Vec) {
    //from Body Coordinate to Haptic Coordinate
    float x = Vec[0];
    float y = Vec[1];
    float z = Vec[2];

    x = 0.7071 * x + sina * 0.7071 * y;
    y = sinb * 0.7071 * x + 0.7071 * y;

    Vec[0] = -y;
    Vec[1] = z;
    Vec[2] = -x;

    return;
}

void SlToMas(std::vector<float>& Vec) {
    //from Body Coordinate to Haptic Coordinate
    float x = Vec[0];
    float y = Vec[1];
    float z = Vec[2];

    x = 0.7071 * x + sina * 0.7071 * y;
    y = sinb * 0.7071 * x + 0.7071 * y;

    Vec[0] = -y;
    Vec[1] = z;
    Vec[2] = -x;

    return;
}

hduVector3Dd setPlaneForce(const hduVector3Dd& MP, const std::vector<float>& SP, const int& CF, const int& LCF, const HDint& BS, const HDint& LBS, const hduVector3Dd& Vel) {
    hduVector3Dd setForce;
    const float f_threshold = 0.3;
    static bool isPressing = true;
    float f = 0;

    if (BS == 0x00) {
        setZero(setForce);
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    }
    else if (BS == 0x01) {
        if (CF == 0) {
            for (int i = 0; i < 3; ++i) {
                setForce[i] = ((SP[i] - LocalSlavePos0[i]) - (MP[i] - MasPos0[i]) * 0.001) * 1.0 - Vel[i] * 0.003;
            }

        }
        else if (CF == 1) {
            for (int i = 0; i < 3; ++i) {
                f -= (MP[i] - MasPos0[i]) * n_plane[i];
            }
            if (f > f_threshold) {
                isPressing = true;
            }
            else if (f < -f_threshold) {
                isPressing = false;
            }
            for (int i = 0; i < 3; ++i) {
                if (isPressing) {
                    setForce[i] = f * n_plane[i];
                }
                else {
                    setZero(setForce);
                }
            }
        }
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////    
    }
    else if (BS == 0x02) {
        for (int i = 0; i < 3; ++i) {
            setForce[i] = -(MP[i] - TargetMasPos[i]) * 0.5;
        }
        setZero(setForce);
    }

    return setForce;
}