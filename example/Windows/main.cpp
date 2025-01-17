// DRFTWin32.cpp : �ܼ� ���� ���α׷��� ���� �������� �����մϴ�.
//

#include <Windows.h>
#include <iostream>
#include <conio.h>
#include <process.h>
// Select your controller version v2(v2~) or v3(v3~)
#ifndef DRCF_VERSION
    #define DRCF_VERSION 2
#endif
#include "../../include/DRFLEx.h"
using namespace DRAFramework;

#undef NDEBUG
#include <assert.h>

CDRFLEx Drfl;
bool g_bHasControlAuthority = FALSE;
bool g_TpInitailizingComplted = FALSE;
bool g_mStat = FALSE;
bool g_Stop = FALSE;
string strDrl = "\r\n\
loop = 0\r\n\
while loop < 1003:\r\n\
 movej(posj(10,10.10,10,10.10), vel=60, acc=60)\r\n\
 movej(posj(00,00.00,00,00.00), vel=60, acc=60)\r\n\
 loop+=1\r\n";
DWORD WINAPI ThreadFunc(void *arg);

bool bAlterFlag = FALSE;
int mCnt = 0;

void OnTpInitializingCompleted()
{
    // Tp �ʱ�ȭ ���� ����� ��û.
    g_TpInitailizingComplted = TRUE;
    Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
}

void OnHommingCompleted()
{
    // 50msec �̳� �۾��� ������ ��.
    cout << "homming completed" << endl;
}

void OnProgramStopped(const PROGRAM_STOP_CAUSE)
{
    assert(Drfl.PlayDrlStop(STOP_TYPE_SLOW));
    // 50msec �̳� �۾��� ������ ��.
    //assert(Drfl.SetRobotMode(ROBOT_MODE_MANUAL));
    cout << "program stopped" << endl;
}

void OnMonitoringDataCB(const LPMONITORING_DATA pData)
{
    // 50msec �̳� �۾��� ������ ��.

    return;
    cout << "# monitoring 0 data "
        << pData->_tCtrl._tTask._fActualPos[0][0]
        << pData->_tCtrl._tTask._fActualPos[0][1]
        << pData->_tCtrl._tTask._fActualPos[0][2]
        << pData->_tCtrl._tTask._fActualPos[0][3]
        << pData->_tCtrl._tTask._fActualPos[0][4]
        << pData->_tCtrl._tTask._fActualPos[0][5] << endl;
}

void OnMonitoringDataExCB(const LPMONITORING_DATA_EX pData)
{
    return;
    cout << "# monitoring 1 data "
        << pData->_tCtrl._tJoint._fActualPos[1]
        << pData->_tCtrl._tJoint._fActualPos[1]
        << pData->_tCtrl._tJoint._fActualPos[2]
        << pData->_tCtrl._tJoint._fActualPos[3]
        << pData->_tCtrl._tJoint._fActualPos[4]
        << pData->_tCtrl._tJoint._fActualPos[5] << endl;
}

void OnMonitoringCtrlIOCB(const LPMONITORING_CTRLIO pData)
{
    return;
    cout << "# monitoring ctrl 0 data" << endl;
    for (int i = 0; i<16; i++)
    {
        cout << (int)pData->_tInput._iActualDI[i] << endl;
    }
}

void OnMonitoringCtrlIOExCB(const LPMONITORING_CTRLIO_EX pData)
{
    return;
    cout << "# monitoring ctrl 1 data" << endl;
    for (int i = 0; i<16; i++)
    {
        cout << (int)pData->_tInput._iActualDI[i] << endl;
    }
    for (int i = 0; i<16; i++)
    {
        cout << (int)pData->_tOutput._iTargetDO[i] << endl;
    }
}


void OnMonitoringStateCB(const ROBOT_STATE eState)
{
    // 50msec �̳� �۾��� ������ ��.
    switch ((unsigned char)eState)
    {
#if 0  // TP �ʱ�ȭ�� ����ϴ� ���������� API ���������� ������� ����.(TP���� �ܵ� ����� ���, ���)
    case STATE_NOT_READY:
        if (g_bHasControlAuthority) Drfl.SetRobotControl(CONTROL_INIT_CONFIG);
        break;
    case STATE_INITIALIZING:
        // add initalizing logic
        if (g_bHasControlAuthority) Drfl.SetRobotControl(CONTROL_ENABLE_OPERATION);
        break;
#endif
    case STATE_EMERGENCY_STOP:
        // popup
        break;
    case STATE_STANDBY:
    case STATE_MOVING:
    case STATE_TEACHING:
        break;
    case STATE_SAFE_STOP:
        if (g_bHasControlAuthority) {
            Drfl.SetSafeStopResetType(SAFE_STOP_RESET_TYPE_DEFAULT);
            Drfl.SetRobotControl(CONTROL_RESET_SAFET_STOP);
        }
        break;
    case STATE_SAFE_OFF:
        //cout << "STATE_SAFE_OFF1" << endl;
        if (g_bHasControlAuthority) {
            //cout << "STATE_SAFE_OFF2" << endl;
            Drfl.SetRobotControl(CONTROL_SERVO_ON);
        }
        break;
    case STATE_SAFE_STOP2:
        if (g_bHasControlAuthority) Drfl.SetRobotControl(CONTROL_RECOVERY_SAFE_STOP);
        break;
    case STATE_SAFE_OFF2:
        if (g_bHasControlAuthority) {
            Drfl.SetRobotControl(CONTROL_RECOVERY_SAFE_OFF);
        }
        break;
    case STATE_RECOVERY:
        //Drfl.SetRobotControl(CONTROL_RESET_RECOVERY);
        break;
    default:
        break;
    }
    return;
    cout << "current state: " << (int)eState << endl;
}

void OnMonitroingAccessControlCB(const MONITORING_ACCESS_CONTROL eTrasnsitControl)
{
    // 50msec �̳� �۾��� ������ ��.

    switch (eTrasnsitControl)
    {
    case MONITORING_ACCESS_CONTROL_REQUEST:
        assert(Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_RESPONSE_NO));
        //Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_RESPONSE_YES);
        break;
    case MONITORING_ACCESS_CONTROL_GRANT:
        g_bHasControlAuthority = TRUE;
        //cout << "GRANT1" << endl;
        //cout << "MONITORINGCB : " << (int)Drfl.GetRobotState() << endl;
        OnMonitoringStateCB(Drfl.GetRobotState());
        //cout << "GRANT2" << endl;
        break;
    case MONITORING_ACCESS_CONTROL_DENY:
    case MONITORING_ACCESS_CONTROL_LOSS:
        g_bHasControlAuthority = FALSE;
        if (g_TpInitailizingComplted) {
            //assert(Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_REQUEST));
            Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
        }
        break;
    default:
        break;
    }
}

void OnLogAlarm(LPLOG_ALARM tLog)
{
    g_mStat = true;
    cout << "Alarm Info: " << "group(" << (unsigned int)tLog->_iGroup << "), index(" << tLog->_iIndex
        << "), param(" << tLog->_szParam[0] << "), param(" << tLog->_szParam[1] << "), param(" << tLog->_szParam[2] << ")" << endl;
}

void OnTpPopup(LPMESSAGE_POPUP tPopup)
{
    cout << "Popup Message: " << tPopup->_szText << endl;
    cout << "Message Level: " << tPopup->_iLevel << endl;
    cout << "Button Type: " << tPopup->_iBtnType << endl;
}

void OnTpLog(const char* strLog)
{
    cout << "Log Message: " << strLog << endl;
}

void OnTpProgress(LPMESSAGE_PROGRESS tProgress)
{
    cout << "Progress cnt : " << (int)tProgress->_iTotalCount << endl;
    cout << "Current cnt : " << (int)tProgress->_iCurrentCount << endl;
}

void OnTpGetuserInput(LPMESSAGE_INPUT tInput)
{
    cout << "User Input : " << tInput->_szText << endl;
    cout << "Data Type : " << (int)tInput->_iType << endl;
}

void OnRTMonitoringData(LPRT_OUTPUT_DATA_LIST tData)
{
    return;
    if (mCnt == 1000)
    {

        printf("timestamp : %.3f\n", tData->time_stamp);
        printf("joint : %f %f %f %f %f %f\n", tData->actual_joint_position[0], tData->actual_joint_position[1], tData->actual_joint_position[2], tData->actual_joint_position[3], tData->actual_joint_position[4], tData->actual_joint_position[5]);
        mCnt = 0;
    }
    else {
        mCnt++;
    }
}

void OnMonitoringSafetyStateCB(SAFETY_STATE iState)
{
    return;
    cout << iState << endl;
}

void OnMonitoringRobotSystemCB(ROBOT_SYSTEM iSystem)
{
    return;
    cout << iSystem << endl;
}

void OnMonitoringSafetyStopTypeCB(const unsigned char iSafetyStopType)
{
    return;
}

DWORD WINAPI ThreadFunc(void *arg)
{
    while (true) {
        if (_kbhit())
        {
            char ch = _getch();
            switch (ch) {
            case 's':
            {
                printf("Stop!\n");
                g_Stop = true;
                Drfl.MoveStop(STOP_TYPE_SLOW);
            }
            break;
            case 'p':
            {
                printf("Pause!\n");
                Drfl.MovePause();
            }
            break;
            case 'r':
            {
                printf("Resume!\n");
                Drfl.MoveResume();
            }
            break;

            case 'y':
            {
                while (bAlterFlag) {
                    float pos[6] = { 10,0,0,10,0,0 };
                    cout << "Alter Flag On !!!!!!" << endl;
                    Drfl.alter_motion(pos);
                }
            }
            }

        }
        Sleep(100);
    }

    return 0;
}

void OnDisConnected()
{
    while (!Drfl.open_connection("192.168.137.100")) {
        Sleep(1000);
    }
}

int main()
{
    // �ݹ� ���(// �ݹ� �Լ� �������� 50msec �̳� �۾��� ������ ��)
    //Drfl.set_on_homming_completed(OnHommingCompleted);
    //Drfl.set_on_monitoring_data(OnMonitoringDataCB);
    //Drfl.set_on_monitoring_data_ex(OnMonitoringDataExCB);
    //Drfl.set_on_monitoring_ctrl_io(OnMonitoringCtrlIOCB);
    //Drfl.set_on_monitoring_ctrl_io_ex(OnMonitoringCtrlIOExCB);
    //Drfl.set_on_monitoring_state(OnMonitoringStateCB);
    //Drfl.set_on_monitoring_access_control(OnMonitroingAccessControlCB);
    //Drfl.set_on_tp_initializing_completed(OnTpInitializingCompleted);
    //Drfl.set_on_log_alarm(OnLogAlarm);
    //Drfl.set_on_tp_popup(OnTpPopup);
    //Drfl.set_on_tp_log(OnTpLog);
    //Drfl.set_on_tp_progress(OnTpProgress);
    //Drfl.set_on_tp_get_user_input(OnTpGetuserInput);
    //   Drfl.set_on_rt_monitoring_data(OnRTMonitoringData);

    //Drfl.set_on_monitoring_robot_system(OnMonitoringRobotSystemCB);
    //Drfl.set_on_monitoring_safety_state(OnMonitoringSafetyStateCB);
    //Drfl.set_on_monitoring_safety_stop_type(OnMonitoringSafetyStopTypeCB);

    //Drfl.set_on_program_stopped(OnProgramStopped);
    //Drfl.set_on_disconnected(OnDisConnected);

    // ���� ����
    assert(Drfl.open_connection("192.168.137.100"));

    // ���� ���� ȹ��
    SYSTEM_VERSION tSysVerion = { '\0', };
    Drfl.get_system_version(&tSysVerion);
    // ����͸� ������ ���� ����
    assert(Drfl.setup_monitoring_version(1));
    Drfl.set_robot_control(CONTROL_SERVO_ON);
    Drfl.set_digital_output(GPIO_CTRLBOX_DIGITAL_INDEX_10, TRUE);
    cout << "System version: " << tSysVerion._szController << endl;
    cout << "Library version: " << Drfl.get_library_version() << endl;

    // ���� ��� ����


    //Drfl.ConfigCreateModbus("mr1", "192.168.137.70", 552, MODBUS_REGISTER_TYPE_HOLDING_REGISTER, 3, 5);

    typedef enum {
        EXAMPLE_JOG,
        EXAMPLE_HOME,
        EXAMPLE_MOVEJ_ASYNC,
        EXAMPLE_MOVEL_SYNC,
        EXAMPLE_MOVEJ_SYNC,
        EXAMPLE_DRL_PROGRAM,
        EXAMPLE_GPIO,
        EXAMPLE_MODBUS,
        EXAMPLE_LAST,
        EXAMPLE_SERVO_OFF
    } EXAMPLE;

    EXAMPLE eExample = EXAMPLE_LAST;

    HANDLE hThread;
    DWORD dwThreadID;
    hThread = CreateThread(NULL, 0, ThreadFunc, NULL, 0, &dwThreadID);
    if (hThread == 0) {
        printf("Thread Error\n");
        return 0;
    }

    bool bLoop = TRUE;
    while (bLoop) {
        g_mStat = false;
        g_Stop = false;
#if 0
        static char ch = '0';
        if (ch == '7') ch = '0';
        else if (ch == '0') ch = '7';
#else
        cout << "\ninput key : ";
        char ch = _getch();
        cout << ch << endl;
#endif    
        switch (ch)
        {
        case 'q':
            bLoop = FALSE;
            break;
        case '0':
        {
            switch ((int)eExample) {
            case EXAMPLE_JOG:
                assert(Drfl.Jog(JOG_AXIS_JOINT_1, MOVE_REFERENCE_BASE, 0.f));
                cout << "jog stop" << endl;
                break;
            case EXAMPLE_HOME:
                assert(Drfl.Home((unsigned char)0));
                cout << "home stop" << endl;
                break;
            case EXAMPLE_MOVEJ_ASYNC:
                assert(Drfl.MoveStop(STOP_TYPE_SLOW));
                cout << "movej async stop" << endl;
                break;
            case EXAMPLE_MOVEL_SYNC:
            case EXAMPLE_MOVEJ_SYNC:
                break;
            case EXAMPLE_DRL_PROGRAM:
                assert(Drfl.PlayDrlStop(STOP_TYPE_SLOW));
                //assert(Drfl.SetRobotMode(ROBOT_MODE_MANUAL));
                //assert(Drfl.SetRobotSystem(ROBOT_SYSTEM_REAL));
                cout << "drl player stop" << endl;
                break;
            case EXAMPLE_GPIO:
                cout << "reset gpio" << endl;
                for (int i = 0; i < NUM_DIGITAL; i++) {
                    assert(Drfl.set_digital_output((GPIO_CTRLBOX_DIGITAL_INDEX)i, FALSE));
                }
                break;
            case EXAMPLE_MODBUS:
                cout << "reset modbus" << endl;
                assert(Drfl.SetModbusValue("mr1", 0));
                break;
            default:
                break;
            }
        }
        break;
        case '1':
        {
            //Drfl.connect_rt_control("127.0.0.1", 12348);
            Drfl.set_robot_control(CONTROL_SERVO_ON);
            //Drfl.change_collision_sensitivity(20);
        }
        break;
        case '2':
        {
            Drfl.set_robot_mode(ROBOT_MODE_MANUAL);
        }
        break;
        case '3':
        {
            float pos[6] = { 0, 0, 90, 0, 90, 0 };
            Drfl.movej(pos, 60, 30);
        }
        break;
        case '4':
        {
            cout << "Input Version : " << Drfl.get_rt_control_input_version_list() << endl;
            cout << "Output Version : " << Drfl.get_rt_control_output_version_list() << endl;
        }
        break;
        case '5':
        {
            cout << "Input Data List : " << Drfl.get_rt_control_input_data_list("v1.0") << endl;
            cout << "Output Data List : " << Drfl.get_rt_control_output_data_list("v1.0") << endl;
        }

        break;
        case '6':
        {
            float vel[6] = { 10, 10, 10, 10, 10, 10 };
            float acc[6] = { 100, 100, 100, 100, 100, 100 };
            float fTargetTime = 4;
            while (1)
            {
                Drfl.speedj_rt(vel, acc, fTargetTime);
                Sleep(1);
            }
        }
        break;
        case '7':
        {
            float tParam[6] = { 1, 2, 3, 4, 5, 6 };

            Drfl.torque_rt(tParam, 2);
        }
        break;
        case '9':
        {
            printf("timestamp : %.3f\n", Drfl.read_data_rt()->time_stamp);
        }

        break;
        case 'a':
        {
            Drfl.set_tool("tool2");
            Sleep(1000);
            cout << Drfl.get_tool() << endl;
        }
        break;
        default:
            break;
        }
        Sleep(100);
    }

    Drfl.CloseConnection();

    return 0;
}

