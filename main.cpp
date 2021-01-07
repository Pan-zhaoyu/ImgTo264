#include <iostream>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <queue>
#include <memory>
#include <thread>

#include <unistd.h>
#include <sys/time.h>
#include <time.h>
#include <stdio.h>
#include <string.h>
#include <chrono>

extern "C"
{
#include "x264/x264.h"
#include "x264/x264_config.h"
}

using namespace cv;
using namespace std;

const int iWidth  = 640;
const int iHeigth = 360;

mutex g_i_mutex;
queue <shared_ptr<unsigned char>> YUVFrame;
FILE* fp_out = nullptr;

bool bStop = false;

int i_pts = 0;

x264_t* m_x264Encoder = nullptr;

shared_ptr<x264_picture_t> m_pictureIn = make_shared<x264_picture_t>();

x264_param_t m_x264Param;

shared_ptr<unsigned char> sps = nullptr;
shared_ptr<unsigned char> pps = nullptr;
int spslen = 0;
int ppslen = 0;
const unsigned char pHeader[4] = { '\0', '\0', '\0', '\1' };
int iSize = 0;
bool bSave = false; //保存当前图片，进行编码，为了保证一秒有30帧的数据

shared_ptr<unsigned char> GetFrame()
{
    shared_ptr<unsigned char> yuvFrame(nullptr);
    {
        lock_guard<mutex> lock(g_i_mutex);
        if (!YUVFrame.empty())
        {
            yuvFrame = YUVFrame.front();
            YUVFrame.pop();
        }
    }
    return yuvFrame;
}

void EncodeImage()
{
    cout << "Starting encoding......." << endl;
    int pos = 0;
    while(!bStop)
    {
        shared_ptr<unsigned char> yuvFrame = GetFrame();

        if(yuvFrame)
        {
            //编码
            m_pictureIn.get()->i_pts  = i_pts++;
            m_pictureIn.get()->img.plane[0] = yuvFrame.get();//Y
            m_pictureIn.get()->img.plane[1] = yuvFrame.get() + m_x264Param.i_height * m_x264Param.i_width;//U
            m_pictureIn.get()->img.plane[2] = yuvFrame.get() + m_x264Param.i_height * m_x264Param.i_width * 5 / 4;//N
            m_pictureIn.get()->img.i_stride[0] = m_x264Param.i_width;
            m_pictureIn.get()->img.i_stride[1] = m_x264Param.i_width >> 1;
            m_pictureIn.get()->img.i_stride[2] = m_x264Param.i_width >> 1;

            x264_picture_t picOut;
            x264_nal_t* nalOut;
            int nalNum;

            int len = x264_encoder_encode(m_x264Encoder, &nalOut, &nalNum, m_pictureIn.get(), &picOut);
            cout << "nalNum: " << nalNum << endl;
            if (len < 0)
            {
                cout << "x264 encode failed" << endl;
                return;
            }
            if (IS_X264_TYPE_I(picOut.i_type))
            {
                cout << "Important frames" << endl;
                //sps pps
                fseek(fp_out, 0, SEEK_SET);
                fwrite(sps.get(), 1, spslen, fp_out);
                fwrite(pps.get(), 1, ppslen, fp_out);
            }
            if(nalNum <= 0)
            {
                cout << "frame delayed in encoder." << endl;
                //return;
            }

            int firstNalFlag = 0;
            if (nalNum != 0)
            {
                //fseek(fp_out, 0, SEEK_SET);
                fwrite(pHeader,  1, 4, fp_out);
            }
            for (int j = 0; j < nalNum; ++j)
            {
               if (NAL_SEI == nalOut[j].i_type)
               {
                   continue;
               }

               if (nalOut[j].p_payload[0] == 0x00 && nalOut[j].p_payload[1] == 0x00)
               {
                   if (nalOut[j].p_payload[2] == 0x01)
                   {
                       pos = 3;
                   }
                   else if (nalOut[j].p_payload[2] == 0x00 && nalOut[j].p_payload[3] == 0x01)
                   {
                       pos = 4;
                   }
               }
               if (firstNalFlag == 0)
               {
                   //fseek(fp_out, 4, SEEK_SET);
                   fwrite(nalOut[j].p_payload + pos, 1, nalOut[j].i_payload - pos, fp_out);
                   firstNalFlag = 1;
               }
               else
               {
                   //fseek(fp_out, 4, SEEK_SET);
                   fwrite(nalOut[j].p_payload + pos + 1, 1, nalOut[j].i_payload - pos - 1, fp_out);
               }
             }
        }
        else
        {
            this_thread::sleep_for(chrono::milliseconds(2));
        }
    }
}

void ClearFrame()
{
    lock_guard<mutex> lock(g_i_mutex);

    while (!YUVFrame.empty())
    {
        YUVFrame.pop();
    }
}

int main(int argc, char* argv[])
{
    VideoCapture cap(0); // open the default camera

    if (!cap.isOpened())  // check if we succeeded
        return -1;

    cap.set(CV_CAP_PROP_FRAME_WIDTH, iWidth);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, iHeigth);
    cap.set(CV_CAP_PROP_FPS, 15.0);//没有效果
    double count = cap.get(CV_CAP_PROP_FPS);

    Mat edges;
    namedWindow("edges", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);

    int yuvLen = iWidth * iHeigth * 3 / 2;

    clock_t start_time, end_time;
    double delay_time;//延时
    start_time = clock();

    fp_out = fopen("/home/pz1_ad_16/PAN/live/mediaServer/live.264", "wb");

    if (!fp_out)
    {
        cout << "Could not open output 264 file." << endl;
        return -1;
    }
    //初始化编码器

    //使用默认参数，在这里因为我的是实时网络传输，所以我使用了zerolatency的选项
    //使用这个选项之后就不会有delayed_frames
    //如果你使用的不是这样的话，还需要在编码完成之后得到缓存的编码帧
    x264_param_default_preset(&m_x264Param, "ultrafast", "zerolatency");
    x264_param_apply_profile(&m_x264Param, x264_profile_names[0]);

    int maxBitRate = 600;//暂且固定
    m_x264Param.rc.i_bitrate = maxBitRate;//* 码率(比特率,单位Kbps)
    m_x264Param.rc.i_vbv_max_bitrate = maxBitRate; // vbv-maxrate
    m_x264Param.rc.i_vbv_buffer_size = maxBitRate; // vbv-bufsize
    m_x264Param.i_nal_hrd = X264_NAL_HRD_CBR;
    m_x264Param.rc.i_rc_method = X264_RC_ABR;
    m_x264Param.rc.f_rf_constant = 0.0f;
    m_x264Param.b_vfr_input = 1;
    m_x264Param.i_keyint_max = count;
    m_x264Param.i_width = iWidth;//编码的图像宽度
    m_x264Param.i_height = iHeigth;//编码的图像高度
    m_x264Param.b_repeat_headers = 0;
    /*m_x264Param.b_annexb = 0;*/
    m_x264Param.vui.b_fullrange = 0;
    m_x264Param.i_threads = 1;

    int ticks_per_frame = 1;
    m_x264Param.i_timebase_num = 1;
    m_x264Param.i_timebase_den = count;

    m_x264Param.i_fps_num = m_x264Param.i_timebase_den;//帧率分子
    m_x264Param.i_fps_den = m_x264Param.i_timebase_num * ticks_per_frame;//帧率分母

    //打开编码器
    m_x264Encoder = x264_encoder_open(&m_x264Param);

    x264_nal_t *nalOut = NULL;
    int nalNum = 0;
    x264_encoder_headers(m_x264Encoder, &nalOut, &nalNum);

    for (int i = 0; i < nalNum; ++i) {
        x264_nal_t &nal = nalOut[i];
        if (nal.i_type == NAL_SPS) {
            //fwrite(nal.p_payload, 1, nal.i_payload, fp_out);
            sps = shared_ptr<unsigned char>(new unsigned char[nal.i_payload + 1], [](unsigned char* p){delete[] p; });
            memcpy(sps.get(), nal.p_payload, nal.i_payload);
            sps.get()[nal.i_payload] = '\0';
            spslen = nal.i_payload;
            //the PPS always comes after the SPS
            x264_nal_t &ppsnal = nalOut[++i];
            //fwrite(ppsnal.p_payload, 1, ppsnal.i_payload, fp_out);
            pps = shared_ptr<unsigned char>(new unsigned char[ppsnal.i_payload + 1], [](unsigned char* p){delete[] p; });
            memcpy(pps.get(), ppsnal.p_payload, ppsnal.i_payload);
            pps.get()[ppsnal.i_payload] = '\0';
            ppslen = ppsnal.i_payload;
        }
        else if (nal.i_type == NAL_SEI) {
            //fwrite(nal.p_payload, 1, nal.i_payload, fp_out);
        }
    }

    //x264_picture_init(m_pictureIn.get());
    x264_picture_alloc(m_pictureIn.get(), X264_CSP_I420, m_x264Param.i_width, m_x264Param.i_height);
    m_pictureIn.get()->img.i_csp = X264_CSP_I420;
    m_pictureIn.get()->img.i_plane = 3;
    m_pictureIn.get()->i_type = X264_TYPE_AUTO;
    m_pictureIn.get()->i_qpplus1 = 0;
    m_pictureIn.get()->i_pts = 0;

    //启动编码线程
    thread t2(EncodeImage);
    t2.detach();

    for (;;)
    {
        Mat frame;
        cap >> frame; // get a new frame from camera
        //cap.read(frame);
        if(frame.empty())
            break;
        //加上字符的起始点
        time_t now;
        char ch[64];
        time(&now);      //获取系统日期和时间
        localtime(&now);//获取当地日期和时间
        int len = strftime(ch, sizeof(ch), "%Y-%m-%d %H:%M:%S",  localtime(&now)); //年-月-日 时-分-秒
        ch[len] = '\0';
        cv::Point p =  cv::Point(iWidth - 340, 20);//位置
        putText(frame, ch, p, FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 200, 200), 1, CV_AA);

        //计算延时
        end_time = start_time;
        start_time = clock(); //ms
        delay_time = (double)(start_time - end_time) / CLOCKS_PER_SEC * 1000;
        cout << "delay time: " << delay_time << "ms" <<  "    " << iSize++ << endl;

        bSave = true;

        cv::Mat yuvImg;
        cv::cvtColor(frame, yuvImg, CV_BGR2YUV_I420);    //YUV转RGB
        //放入队列        
        if (bSave)
        {
            shared_ptr<unsigned char> yuvBuffer = shared_ptr<unsigned char>(new unsigned char[yuvLen], [](unsigned char* p){delete[] p; });
            memcpy(yuvBuffer.get(), yuvImg.data, yuvLen * sizeof(unsigned char));    //YUV数据复制到yuv_buffer中

            lock_guard<mutex> lock(g_i_mutex);
            YUVFrame.push(yuvBuffer);
            //cout << "YUVFrame.size " << YUVFrame.size() << endl;
            if (YUVFrame.size() > 30)
            {
                cout << "frame size has over 30, size is " << YUVFrame.size() << endl;
            }
        }

        imshow("edges", frame);
        if (waitKey(1) >= 0) break;
    }

    bStop = true;
    this_thread::sleep_for(chrono::milliseconds(100));//等待线程结束
    fclose(fp_out);
    ClearFrame();

    //关闭编码器
    x264_encoder_close(m_x264Encoder);
    bSave = false;
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
