// InsCHVSControlDemo.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include "ins_chvs_ctrl.hpp"

#define MB_OPERATOR 0x494e5330
#define MB_INIT     0x00000000

#define PIXELWIDTH 8588  //363G 600dpi有效像素宽度8588

int main()
{
    std::cout << "Welcome to use INS_CHVS!\n";
    INS_Result ret, tmpRtVal;
    char m_ip[200];              
    // step 1: 枚举设备
    InsGigE_DeviceInfoList device_info_list;
    InsCHVSCamera::InsCHVS_FindDevice(&device_info_list);

    for (int i = 0; i < device_info_list.device_num; ++i)
    {
        std::cout << "[" << i << "]" << " ip = " << device_info_list.device_infos[i].current_ip << " mac = " << device_info_list.device_infos[i].mac_addr << std::endl;
    }
    std::cout << std::endl;

    // step 2: 选择设备
    std::cout << "Select the index you want to open:" << std::endl;
    system("pause");
    int device_index = 0;
    std::cout << ">> ";
    //std::cin >> device_index;
    device_index = 0;
    std::cout << "Selected IP: " << device_info_list.device_infos[device_index].current_ip << std::endl << std::endl;
    InsCHVSCamera camera;
    ret = camera.OpenCHVS((const char*)(char*)device_info_list.device_infos[device_index].current_ip);
    if (ret != INS_Result::INS_OK)
    {
        std::cout << "Open cmera failed ... " << std::endl;
        return -1;
    }
    else
    {
        camera.GetIP(m_ip);
        std::cout << "Open cmera successfully " << m_ip << "..." << std::endl;
    }

    cout << "Selected IP = " << m_ip << endl;

    //ret =camera.Save_InsCHVS_ConfigFile("C:/Users/INSNEX/Desktop/test0202.icf",5000);
    //ret =camera.Load_InsCHVS_ConfigFile("C:/Users/INSNEX/Desktop/test0202.icf",5000);
    //return 0;

    // 加载配置文件
    std::cout << "Select configuration file:" << std::endl;
    std::cout << "[0] camera1.xml" << std::endl;
    std::cout << "[1] camera2.xml" << std::endl;
    std::cout << ">> ";
    //std::cin >> device_index;
    //camera.Load_InsCHVS_ConfigFile("C:\\Users\\INSNEX\\Desktop\\1234.xml",5000);

    camera.InsCHVS_Set_Img_PixFormat(InsCHVS_PixelFormat::Mono8, 1000); //这个测试工程仅仅适用于mono8格式图像

    int lines = 120;
    cv::Mat mat(lines, PIXELWIDTH, CV_8UC1);
    InsBuffer buffer;
    buffer.p_data = nullptr;
    //测试寄存器，测试通信
    uint32_t MVALUE;
    camera.InsCHVS_Get_Dev_TestReg(&MVALUE, 1000);
    cout << "test:" << MVALUE << endl;
    uint32_t m_value = 0;

    camera.InsCHVS_Get_MBCheck(&m_value, 1000);//读取设备信息时，需要使能位使能
    if (m_value != MB_OPERATOR)
    {
        camera.InsCHVS_Set_MBCheck(MB_OPERATOR, 1000);
    }
    camera.InsCHVS_Get_MBCheck(&m_value, 1000);

    char retVal[200];
    camera.InsCHVS_Get_Info_companyName(retVal, 1000);
    cout << "InsCHVS_Get_Info_companyName = " << retVal << endl;
    camera.InsCHVS_Get_Info_dataTime(retVal, 1000);
    cout << "InsCHVS_Get_Info_dataTime = " << retVal << endl;
    camera.InsCHVS_Get_Info_deviceType(retVal, 1000);
    cout << "InsCHVS_Get_Info_deviceType = " << retVal << endl;
    camera.InsCHVS_Get_Info_FPGAVersion(retVal, 1000);
    cout << "InsCHVS_Get_Info_FPGAVersion = " << retVal << endl;
    camera.InsCHVS_Get_Info_MicroBlazeVersion(retVal, 1000);
    cout << "InsCHVS_Get_Info_MicroBlazeVersion = " << retVal << endl;
    camera.InsCHVS_Set_MBCheck(MB_INIT, 1000);

    double exposure = 1000;

    camera.InsCHVS_Set_Acq_TrigSource(InsCHVS_LineTriggerSource::Internal_Clock, 1000);
    camera.InsCHVS_Set_Acq2_TrigSource(InsCHVS_LineTriggerSource::External_Encoder, 1000);

    camera.StartCHVS();
    camera.InsCHVS_Start_DeviceAcquisition(1000);
    bool stop = false;

    while (!stop)
    {
        tmpRtVal = camera.AcquireImage(&buffer, lines);
        cout << "AcquireImage Once = " << (uint32_t)tmpRtVal << endl;
        memcpy(mat.data, buffer.p_data, PIXELWIDTH * lines);
        cv::namedWindow("CIS", 2);
        cv::imshow("CIS", mat);
        char ch = ' ';
        if (ch = cv::waitKey(30))
        {
            switch (ch)
            {
            case 'A':
            case 'a':
                tmpRtVal = camera.InsCHVS_Set_LED_ExposureTime1(1000, 1000);
                cout << "LED: 5000ns = " << (uint32_t)tmpRtVal << endl;
                break;
            case 'S':
            case 's':
                tmpRtVal = camera.InsCHVS_Set_LED_ExposureTime1(3000, 1000);
                cout << "LED: 50000ns = " << (uint32_t)tmpRtVal << endl;
                break;

            case 'W':
            case 'w':
                cv::imwrite("D:\\image.bmp", mat);
                cout << "Save D:\\image.bmp " << (uint32_t)tmpRtVal << endl;
                break;
            case 'q':
                stop = true;
                break;
            case 'R':
            case 'r':
                tmpRtVal = camera.InsCHVS_Stop_DeviceAcquisition(1000);
                tmpRtVal = camera.InsCHVS_Start_DeviceAcquisition(1000);
                cout << "Restart_DeviceAcquisition = " << (uint32_t)tmpRtVal << endl;
            default:
                break;
            }
        }
    }
    tmpRtVal = camera.InsCHVS_Stop_DeviceAcquisition(1000);
    cout << "InsCHVS_Stop_DeviceAcquisition = " << (uint32_t)tmpRtVal << endl;
    tmpRtVal = camera.StopCHVS();
    cout << "StopCHVS = " << (uint32_t)tmpRtVal << endl;
    cv::destroyWindow("CIS");
}
