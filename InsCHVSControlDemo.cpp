/* GrabContinuous.cpp
 * @brief：本示例向用户演示如何运用InsCHVSControl库，通过CHVS相机连续采集图像。多线程处理图像缺陷检测
 */
#include <conio.h>
#include <unordered_map>
#include <bitset>
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>

using namespace cv;
#include "ins_chvs_ctrl.hpp"
#if _DEBUG
#pragma comment(lib,"InsCHVSControl_d.lib")
#pragma comment(lib,"opencv_world411d.lib")
#endif // _DEBUG
#ifdef NDEBUG
#pragma comment(lib,"InsCHVSControl.lib")
#pragma comment(lib,"opencv_world411.lib")
#endif // NDEBUG


 //设置DPI
#if 0 
#define _DPI_300
#else
#define _DPI_600
#endif

// 超时时间阈值。
#define TIMEOUT 1000

// 图像行数
#define LINES   1024

 /* @brief：打印 错误信息后退出程序。
  * @param[in] info：其他文字信息
  * @param[in] ret ：函数返回值。
  */
#define _CHECK_CHVS(info, ret) printCHVSResultAndErrorExit(info, ret)
#define CHECK_CHVS(ret) printCHVSResultAndErrorExit("", ret)
void printCHVSResultAndErrorExit(const char* info, INS_Result ret);


/* @brief：INS_Result转为String。
 * @param[in] ret ：函数返回值。
 */
std::string ins_ResultToString(INS_Result ret);

// 定义图像队列
std::queue<cv::Mat> imageQueue;
std::mutex queueMutex;
std::condition_variable queueNotEmpty;

// 定义线程数量
const int numThreads = 8;

// 控制线程运行的标志
bool isRunning = true;

// 存储处理后的图像
cv::Mat processedFrame;

// 后台线程函数
void workerThread() {
    while (isRunning) {
        cv::Mat currentFrame;

        // 从队列中取图像
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            queueNotEmpty.wait(lock, [] { return !imageQueue.empty(); });
            currentFrame = imageQueue.front();
            imageQueue.pop();
        }
        //-----------------以下这种方法利用灰度阈值分割适用于特征灰度值低于灰度背景--------------------------------------
        // 图像处理
        cv::Mat grayImage;
        // 将单通道灰度图像转换为三通道 BGR 彩色图像
        cv::Mat colorImage;
        cv::cvtColor(currentFrame, colorImage, cv::COLOR_GRAY2BGR);
        
        cv::cvtColor(colorImage, grayImage, cv::COLOR_BGR2GRAY);

        // 图像预处理
        cv::Mat blurImg;
        cv::GaussianBlur(grayImage, blurImg, cv::Size(5, 5), 0);  // 高斯模糊

        // 阈值分割，将灰度值低于thresholdValue的区域置零
        cv::Mat thresholded;
        cv::threshold(blurImg, thresholded, 165, 255, cv::THRESH_BINARY);

        // 对二值图像进行膨胀和腐蚀操作，以进一步处理噪声或连接断开的轮廓(图像形态学处理，可选择用或不用）
        //cv::Mat dilatedImg, erodedImg;
        //cv::dilate(thresholded, dilatedImg, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
        //cv::erode(dilatedImg, erodedImg, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));


        // 反转图像
        cv::Mat blackContours;
        std::vector<std::vector<cv::Point>> contourss;

        cv::bitwise_not(thresholded, blackContours);  // 反转 the eroded image

        // 对反转图像进行形态学滤波，以消除噪声和平滑图像
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
        cv::morphologyEx(blackContours, blackContours, cv::MORPH_CLOSE, kernel);

        cv::findContours(blackContours, contourss, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 绘制面积小于3000的轮廓（用面积筛掉背景干扰）
        for (const auto& contour : contourss) {
            if (cv::contourArea(contour) < 3000) {
                std::vector<std::vector<cv::Point>> smallContours{ contour };
                cv::drawContours(colorImage, smallContours, -1, cv::Scalar(0, 0, 255), 10);
            }
        }

        //------------------下面注释的部分用的灰度梯度法筛选缺陷特征，理论上可以适用于特征与背景灰度差相差4（可调）的情形------------
        //cv::Mat grayImg;
        //grayImg = currentFrame.clone();

        //cv::Mat blurImg;
        //// 图像预处理
        //GaussianBlur(grayImg, blurImg, Size(5, 5), 0);  // 高斯模糊

        //// 计算灰度差图像
        //cv::Mat diffImg;
        //diffImg = blurImg.clone();

        //// 对灰度差图像进行形态学滤波，以消除噪声和平滑图像
        //cv::Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
        //morphologyEx(diffImg, diffImg, MORPH_CLOSE, kernel);

        //int threshValue = 4;  // 灰度差阈值
        //for (int i = 0; i < blurImg.rows; i++) {
        //    for (int j = 0; j < blurImg.cols; j++) {
        //        uchar pixel1 = blurImg.at<uchar>(i, j);
        //        uchar pixel2 = 0;
        //        if (j > 0) {
        //            pixel2 = blurImg.at<uchar>(i, j - 1);
        //        }
        //        if (abs(pixel1 - pixel2) < threshValue) {
        //            diffImg.at<uchar>(i, j) = 0;
        //        }
        //    }
        //}

        //// 对灰度差图像进行形态学滤波，以消除噪声和平滑图像
        //cv::Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
        //morphologyEx(diffImg, diffImg, MORPH_CLOSE, kernel);

        //// 生成掩码图像
        //cv::Mat maskImg;
        //threshold(diffImg, maskImg, 0, 250, THRESH_BINARY);

        //// 绘制缺陷
        //vector<vector<Point>> contours;
        //findContours(maskImg, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        //for (size_t i = 0; i < contours.size(); i++) {
        //    double area = contourArea(contours[i]);
        //    if (area > 0.1) {
        //        Rect rect = boundingRect(contours[i]);
        //        rectangle(grayImg, rect, Scalar(0, 0, 255), 5);
        //    }
        //}

        // 将处理后的图像存储到全局变量
        {
            std::lock_guard<std::mutex> lock(queueMutex);
            processedFrame = currentFrame.clone();
            //cv::cvtColor(processedFrame, currentFrame, cv::COLOR_GRAY2BGR);
         
        }

        //创建窗口
        cv::namedWindow("CIS1", 2);
        
        cv::imshow("CIS1", colorImage);//在CIS1窗口显示处理后图像
        cv::waitKey(30);  // 添加适当的等待时间

    }
}

// 滑动条回调函数
void onThresholdChange(int, void*) {
    // 通知后台线程重新处理图像
    queueNotEmpty.notify_all();
}

int main()
{
    std::cout << "Welcome to use INS_CHVS!\n";
    char version[20];
    InsCHVSCamera::InsCHVS_SoftWare_Version(version);
    std::cout << "SDK version: " << version << endl;
    //
    //************************************************/
    //
    InsCHVS_DeviceType DEVICE_TYPE;
    int PIXELWIDTH;
    InsCHVS_DPI DPI;

    std::cout << "请输入相机型号的序号：\n1. 363G\t\t2. 550BB" << std::endl;
    char device = _getch();
    std::cout << "您选择的序号是 " << device << std::endl << std::endl;
    if (device == '1') {
        DEVICE_TYPE = InsCHVS_DeviceType::CHVS_363G;
#ifdef _DPI_600
        PIXELWIDTH = 8588;
        DPI = InsCHVS_DPI::DPI600;
#elif defined(_DPI_300)
        PIXELWIDTH = 4304;
        DPI = InsCHVS_DPI::DPI300;
#endif
    }
    else if (device == '2') {
        DEVICE_TYPE = InsCHVS_DeviceType::CHVS_550BB;
        int PIXELWIDTH;
        InsCHVS_DPI DPI;
#ifdef _DPI_600
        PIXELWIDTH = 13088;
        DPI = InsCHVS_DPI::DPI600;
#elif defined(_DPI_300)
        PIXELWIDTH = 6560;
        DPI = InsCHVS_DPI::DPI300;
#endif
    }
    else {
        std::cout << "无效输入" << std::endl;
        system("pause");
        return 0;
    }
    //
    //************************************************/
    // 
    // 
    //************************************************/
    //
    // 函数返回值
    INS_Result ret = INS_Result::INS_OK;
    //
    // CHVS相机对象
    InsCHVSCamera camera;
    //
    // CHVS设备列表
    InsCHVS_DeviceInfoList device_info_list;
    //
    // 图像Buffer
    InsCHVS_Buffer buffer;
    buffer.p_data = nullptr;
    //
    // 用于接收图像
    cv::Mat mat(LINES, PIXELWIDTH, CV_8UC1);
    // 
    //************************************************/

    // step 1: 更新设备列表
    std::cout << "设备列表：" << std::endl;
    InsCHVSCamera::InsCHVS_FindDevice(&device_info_list);

    if (device_info_list.device_num == 0) {
        std::cout << "未找到设备!" << std::endl;
        system("pause");
        return 0;
    }
    for (int i = 0; i < device_info_list.device_num; ++i) {
        std::cout << "[" << i << "]";
        std::cout << " ip = " << device_info_list.device_infos[i].current_ip;
        std::cout << " mac = " << device_info_list.device_infos[i].mac_addr << std::endl;
    }
    std::cout << std::endl;

    // step 2: 选择设备并连接
    system("pause");
    int device_index = 0;
    // std::cin >> device_index;
    std::cout << "Selected IP: " << device_info_list.device_infos[device_index].current_ip << std::endl;
    ret = camera.OpenCHVS(device_info_list.device_infos[device_index].current_ip);
    _CHECK_CHVS("OpenCHVS, ", ret);

    // 连接成功后，获取IP
    char m_ip[16];
    camera.GetIP(m_ip);
    std::cout << "当前已连接设备IP: " << m_ip << endl;
    system("pause");

    // 测试寄存器，测试通信
    uint32_t sValue = 0xff00ff00, tValue;
    camera.InsCHVS_Set_Dev_TestReg(sValue, TIMEOUT);
    camera.InsCHVS_Get_Dev_TestReg(&tValue, TIMEOUT);
    _CHECK_CHVS("Get_Dev_TestReg, ", ret);
    std::cout << "Test Register : " << std::hex << tValue << std::dec << std::endl;

    // 读取设备信息时，需要使能位使能
    std::cout << "-------------------------------------------------------------" << std::endl;
    camera.InsCHVS_Enable_MBCheck(TIMEOUT);
    char retVal[200];
    camera.InsCHVS_Get_Info_companyName(retVal, TIMEOUT);
    std::cout << "InsCHVS_Get_Info_companyName \t= " << retVal << std::endl;
    camera.InsCHVS_Get_Info_deviceType(retVal, TIMEOUT);
    std::cout << "InsCHVS_Get_Info_deviceType \t= " << retVal << std::endl;
    camera.InsCHVS_Get_Info_dataTime(retVal, TIMEOUT);
    std::cout << "InsCHVS_Get_Info_dataTime \t= " << retVal << std::endl;
    camera.InsCHVS_Get_Info_FPGAVersion(retVal, TIMEOUT);
    std::cout << "InsCHVS_Get_Info_FPGAVersion \t= " << retVal << std::endl;
    camera.InsCHVS_Get_Info_MicroBlazeVersion(retVal, TIMEOUT);
    std::cout << "InsCHVS_Get_Info_MicroBlazeVersion = " << retVal << std::endl;
    // 读取结束时，复位使能位
    camera.InsCHVS_Disable_MBCheck(TIMEOUT);
    std::cout << "-------------------------------------------------------------" << std::endl;

    //*******************************************************************************/
    // step 3: 取图参数设置
    // 
    // 相机类型
    camera.InsCHVS_Set_DeviceType(DEVICE_TYPE);
    // 
    // 一个数据包的传输高度，建议设置为单张图片的高度
    camera.InsCHVS_Set_Img_TransHeight(LINES, TIMEOUT);
    //
    // 行节拍数,无需更改其他值
    camera.InsCHVS_Set_Img_FrameBitsCycle(408, TIMEOUT);
    // 
    // 目前硬件仅适用于mono8格式图像
    camera.InsCHVS_Set_Img_PixFormat(InsCHVS_PixelFormat::Mono8, TIMEOUT);
    //
    // 图像DPI：600DPI
    camera.InsCHVS_Set_Img_DPI(DPI, TIMEOUT);//dpi
    //
    // 触发源一：内部时钟触发。
    camera.InsCHVS_Set_Acq_TrigSource(InsCHVS_LineTriggerSource::Internal_Clock, TIMEOUT);
    // 
    // 触发源一：触发源为设备内部时钟时，设定采集频率（最大66，单位khz）
    camera.InsCHVS_Set_Acq_Intern_TrigPeriod(30, TIMEOUT);
    //
    // 触发源一：触发源为设备内部时钟时，设定单次捕获的图像行数，行数设为0表示持续取图
    camera.InsCHVS_Set_Acq_Intern_TrigNums(0, TIMEOUT);
    // 
    // 触发源二：打开触发源2
    camera.InsCHVS_SetEnable_Acq2(InsCHVS_FuncEnable::Ins_Enable, TIMEOUT);
    //
    // 触发源二：内部时钟触发
    camera.InsCHVS_Set_Acq2_TrigSource(InsCHVS_LineTriggerSource::Internal_Clock, TIMEOUT);
    // 
    // 触发源二：触发源为设备内部时钟时，设定采集频率（最大66，单位khz），应当比不大于触发源一的采集频率
    camera.InsCHVS_Set_Acq2_Intern_TrigPeriod(30, TIMEOUT);
    //
    // 触发源二：触发源为设备内部时钟时，设定单次捕获的图像行数，行数设为0表示持续取图
    camera.InsCHVS_Set_Acq2_Intern_TrigNums(0, TIMEOUT);
    //
    // 光源模式：AB光源连续模式
    camera.InsCHVS_SetEnable_Multi_Trig(InsCHVS_LED_TriggerMode::LED_SimultaneousDualBrightness, TIMEOUT);
    //
    // 获取最大曝光时间
    double_t TimeLine_Max = 0;
    camera.InsCHVS_Get_TimeLine_Max(&TimeLine_Max, TIMEOUT);
    std::cout << "TimeLine_Max: " << TimeLine_Max << std::endl;
    //
    // 曝光时间: 单位us, 最大曝光时长可由InsCHVS_Get_TimeLine_Max()获取
    // 设置值大于最大值时，自动设置为最大值。
    camera.InsCHVS_Set_LED_ExposureTime1(10, TIMEOUT);
    //
    void printf_info();
    printf_info();
    system("pause");
    //*******************************************************************************/
    // 
    // step 4: 开始取流
        // 程序开始时间点
    auto start_time = std::chrono::steady_clock::now();
    camera.StartCHVS();
    //
    // 控制设备开始捕获图像并往上传输
    camera.InsCHVS_Start_DeviceAcquisition(TIMEOUT);
    //*******************************************************************************/

    // 创建后台线程
    std::vector<std::thread> threads;
    for (int i = 0; i < numThreads; ++i) {
        threads.emplace_back(workerThread);
    }

    bool stop = false;
    while (!stop)
    {
        // 采集单张图片
        ret = camera.AcquireImage(&buffer, LINES, false);
        //std::cout << "\nAcquireImage Once = " << ins_ResultToString(ret) << std::endl;

        // 转存图片数据,将相机采集图片类型转成opencv可以处理的mat类型
        memcpy(mat.data, buffer.p_data, PIXELWIDTH * LINES);

        // 将帧放入队列
        {
            std::lock_guard<std::mutex> lock(queueMutex);
            imageQueue.push(mat.clone());  // 使用 clone 创建图像的副本
        }
        queueNotEmpty.notify_one();

        // 显示原始视频帧
        cv::namedWindow("CIS", 2);
        cv::imshow("CIS", mat);

        // 检测按键，如果按下ESC键则退出循环
        char key = cv::waitKey(30);
        if (key == 27) // ASCII码值27代表ESC键
            break;

        // 程序结束时间点
        auto end_time = std::chrono::steady_clock::now();

        // 计算时间差并输出
        auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        std::cout << "程序流程总时间为: " << elapsed_time.count() << " 毫秒." << std::endl;
       
        // 等待用户操作
        char ch = ' ';
        if (ch = cv::waitKey(3))
        {
            switch (ch)
            {
            case 'A':
            case 'a':
                // 曝光时间: 3us
                ret = camera.InsCHVS_Set_LED_ExposureTime1(3, TIMEOUT);
                _CHECK_CHVS("LED: 3us = ", ret);
                break;
            case 'W':
            case 'w':
                // 曝光时间: 10us
                ret = camera.InsCHVS_Set_LED_ExposureTime1(20, TIMEOUT);
                _CHECK_CHVS("LED: 10us = ", ret);
                break;
            case 'S':
            case 's':
                // 保存图片
                cv::imwrite(".\\image.bmp", mat);
                _CHECK_CHVS("Save .\image.bmp  = ", ret);
                break;
            case 'Q':
            case 'q':
                // 退出循环，结束取图
                stop = true;
                break;
            case 'R':
            case 'r':
                // 重启取图
                ret = camera.InsCHVS_Stop_DeviceAcquisition(TIMEOUT);
                ret = camera.InsCHVS_Start_DeviceAcquisition(TIMEOUT);
                _CHECK_CHVS("Restart_DeviceAcquisition = ", ret);
                break;
            default:
                break;
            }
        }
    }

    // 结束取图
    ret = camera.InsCHVS_Stop_DeviceAcquisition(TIMEOUT);
    _CHECK_CHVS("InsCHVS_Stop_DeviceAcquisition = ", ret);

    // 停止取流
    ret = camera.StopCHVS();
    _CHECK_CHVS("StopCHVS = ", ret);

    // 关闭图像窗口
    //cv::destroyWindow("CIS");

    // 关闭窗口
    cv::destroyAllWindows();

    // 等待后台线程结束
    isRunning = false;
    queueNotEmpty.notify_all();
    for (auto& thread : threads) {
        thread.join();
    }

    return 0;

    // 关闭相机
    ret = camera.CloseCHVS();
    _CHECK_CHVS("CloseCHVS = ", ret);

}
