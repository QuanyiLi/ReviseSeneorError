#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "api/RpcLibClientBase.hpp"


#include "opencv2/opencv.hpp"
#include <iostream>
#include "WINDOWS.h"
#include "timer.h" 
#include <pthread.h>
#include<cmath>

#include<fstream>


#include "PIDController.h"
#include "PID.h"

//msr::airlib::MultirotorRpcLibClient client ;//连接localhost:41451
#include"client.h"
//传感器误差修正
#include"ReviseSensorError.h"
//姿态估计
#include "attitude.h"


#define OPENCV

//using namespace std;
using namespace cv;
using namespace msr::airlib;

 
//相机ID 0至4分别对应于中央前方，左前方，右前方，中央下方，中央后方。
//比赛时只能用底部摄像头场景图，前置摄像头的场景图和深度图。
 
//定义相机ID
#define CAMERA_FRONT 0
#define CAMERA_FRONT_LEFT 1
#define CAMERA_FRONT_RIGHT 2
#define CAMERA_BELOW 3
#define CAMERA_BEHIND 4


//airsim 相关
typedef ImageCaptureBase::ImageRequest ImageRequest;
typedef ImageCaptureBase::ImageResponse ImageResponse;
typedef ImageCaptureBase::ImageType ImageType;

msr::airlib::MultirotorRpcLibClient client("localhost", 41451);//连接localhost:41451 


//msr::airlib::MultirotorRpcLibClient client("192.168.0.1", 41451);//用于client与server交互

 

DrivetrainType driveTrain = DrivetrainType::MaxDegreeOfFreedom;

 

float roll = 0.1f, roll_temp;//绕x轴逆时针 //单位是弧度
float pitch = 0.1f, pitch_temp;//绕y轴逆时针  
float yaw = 0.0f; //绕z轴逆时针
float duration = 0.2f;//持续时间
float throttle = 0.575f;
float yaw_rate = 0.1f;
float altitude_last;//上一次的GPS高度，用来测试油门
float altitude_cur;//当前的GPS高度，用来测试油门

				   //传感器数据
GpsData		     GPS_data;
BarometerData	 Barometer_data;
MagnetometerData Magnetometer_data;
ImuData			 Imu_data;



//线程
DWORD WINAPI Key_Scan(LPVOID pVoid); 
DWORD WINAPI get_img(LPVOID pVoid);
DWORD WINAPI Control_Z_Thread(LPVOID pVoid);//控制四轴高度
 

HANDLE hTimer1;
pthread_mutex_t mutex1 = PTHREAD_MUTEX_INITIALIZER;//互斥锁									  
												    

volatile int key_value_cv = -1;
static int key_control(int key);//按键控制

								 
std::vector<ImageRequest> request = { ImageRequest(CAMERA_BELOW, ImageType::Scene) };//创建获取正下方的场景图(默认压缩)的请求
//std::vector<ImageRequest> request = { ImageRequest(CAMERA_FRONT, ImageType::Scene) };//创建获取的场景图(默认压缩)的请求
std::vector<ImageResponse> response;//响应
cv::Mat rgb_image; 

uint8_t Congtrol_Z_flag = 0;//1表示要控制高度
uint8_t Congtrol_Centre_flag = 0;//1表示要控制停机牌中心
								  

double Control_Z = -600.0f;//控制的高度
double Control_Z_tolerance = 0.5f;// 0.01f;//控制高度的容忍
PID my_PID;

//四元数更新姿态
#define Kp 100.0f  //加速度权重，越大则加速度测量值收敛越快
#define Ki  0.001f //误差积分增益
#define halfT 0.001f; //采样周期的一半

float Yaw=0,Pitch=0,Roll=0;

//四元数
float q0=1,q1=0, q2=0, q3=0;
float exInt = 0, eyInt = 0, ezInt = 0;//按比例缩小积分误差

#define PI 3.1415925

int main()
{
	while (RpcLibClientBase::ConnectionState::Connected != client.getConnectionState())
		client.confirmConnection();//连接场景
	while (!client.isApiControlEnabled())
		client.enableApiControl(true);//获取api控制
	client.armDisarm(true);//解锁飞控

	client.hover();//hover模式

    //首先创建可等候定时器
	hTimer1 = CreateWaitableTimer(NULL, FALSE, NULL);
	//设置定时器时间
	INT64 nDueTime = -0 * _SECOND;//定时器生效时间，立即
	SetWaitableTimer(hTimer1, (PLARGE_INTEGER)&nDueTime, 50, NULL, NULL, FALSE);//50表示定时器周期50ms
																				 

	HANDLE hThread2 = CreateThread(NULL, 0, get_img, NULL, 0, NULL);// &nThreadID);
	HANDLE hThread1 = CreateThread(NULL, 0, Key_Scan, NULL, 0, NULL);//第五个 0 表示创建后立即激活线程
	HANDLE hThread3 = CreateThread(NULL, 0, Control_Z_Thread, NULL, 0, NULL); 
 
 

	while (true)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(20));//阻塞当前进程
		if (-1 != key_value_cv)//如果有按键按下
		{
			if (-1 == key_control(key_value_cv))
			{
				//摧毁所有OpenCV窗口
				cv::destroyAllWindows();
				//关闭线程
				CloseHandle(hThread1);
				CloseHandle(hThread2);
				CloseHandle(hThread3);
		 
				//关闭定时器
				CloseHandle(hTimer1);
				return 0;
			}
		}
	}

	//等待线程返回
	WaitForSingleObject(hThread1, INFINITE);
	WaitForSingleObject(hThread2, INFINITE);
	CloseHandle(hThread1);
	CloseHandle(hThread2);
	CloseHandle(hThread3);
 
	//关闭定时器
	CloseHandle(hTimer1);
	return 0;
}

//处理气压计数据
void  ReviseBarometerData(int i) {
	Barometer_data = client.getBarometerdata();
	ofstream out;
	out.open("Bar.txt", ios::app);
	if (!out.is_open())
    {
		cout << "未成功打开文件" << endl;
	}
	out<< Barometer_data.altitude<< "  "<< Barometer_data.pressure<<" "<<i<<endl;
	out.close();

	std::cout <<"************气压计数据*************"<< std::endl;
	std::cout << "altitude：" << Barometer_data.altitude << std::endl;
	std::cout << "pressure: " <<Barometer_data.pressure<< std::endl;
	std::cout << "time_stamp: "<< Barometer_data.time_stamp << std::endl;
}

//处理磁力计数据,
void ReviseMagagnetometerData() {
	Magnetometer_data = client.getMagnetometerdata();
	std::cout << "************磁力计数据*************" << std::endl;
	std::cout << "magnetic_field_body：" << Magnetometer_data.magnetic_field_body<<std::endl;
	std::cout << "yaw:" <<atan2(Magnetometer_data.magnetic_field_body(1,0), Magnetometer_data.magnetic_field_body(0,0))*57.3 << std::endl;
	std::cout << "time_stamp: " << Magnetometer_data.time_stamp << std::endl;
	 
}

//处理IMU数据
void ReviseImuData(){
	Imu_data = client.getImudata();
	std::cout << "************IMU数据*************" << std::endl;
	//角速度
	std::cout << "angular_velocity：" << Imu_data.angular_velocity<< std::endl;
	//线加速度
	std::cout << "linear_acceleration: " << Imu_data.linear_acceleration<< std::endl;
	std::cout << "time_stamp:" <<Imu_data.time_stamp << std::endl;
}

void getAttitude(){
	AttitudeCalculation   att;
	Magnetometer_data = client.getMagnetometerdata();
	Imu_data = client.getImudata();
	Vector3f acc = Imu_data.linear_acceleration;
	Vector3f vel = Imu_data.angular_velocity;
	Vector3f mag = Magnetometer_data.magnetic_field_body;
	Vector3f euler=att.GetAngle(acc, vel, mag, 0.001);
	//client.moveByAngleThrottle(euler(1,0), euler(2, 0), throttle, euler(0, 0), 0.2f);
}


 


DWORD WINAPI Control_Z_Thread(LPVOID pVoid){
	int i = 0;
	while (true){
		//等待定时器时间到达
		WaitForSingleObject(hTimer1, INFINITE);
		//如果需要控制高度
		if (1 == Congtrol_Z_flag){
			//P 控制油门
			double Throttle_Z = my_PID.PIDZ(Control_Z, Control_Z_tolerance);//设定高度，容忍
			if (Throttle_Z > Control_Z_tolerance*0.1 || Throttle_Z < -Control_Z_tolerance*0.1)
			{
				client.moveByAngleThrottle(0.0f, 0.0f, Throttle_Z + 0.575, 0.0f, 0.05f);
			}
		}
	}
}




DWORD WINAPI Key_Scan(LPVOID pVoid)
{
	clock_t time_1;
	while (true)
	{
		//等待定时器时间到达
		WaitForSingleObject(hTimer1, INFINITE);

		pthread_mutex_lock(&mutex1);//获得锁

									//显示图像
		if (!rgb_image.empty()) {
			cv::imshow("original_img", rgb_image);
			cvWaitKey(0);
		}
		else
		{
			cvWaitKey(1);
			std::cout << "failed" << std::endl;
		}


		//按键扫描
		if (-1 == key_value_cv)
		{
			//等待按键按下，并且返回完整键值
			key_value_cv = cv::waitKeyEx(0);
		}
		else
		{
			cv::waitKey(1);
		}
		//while (-1 != cv::waitKey(1));//把缓冲区读完后，才会显示1ms图像
		pthread_mutex_unlock(&mutex1);//释放锁
	}
	return 0;
}

DWORD WINAPI get_img(LPVOID pVoid)
{
	int i = 0;
	clock_t time_1;// = clock();//get time


	while (true)
	{
		//等待定时器时间到达
		WaitForSingleObject(hTimer1, INFINITE);

		pthread_mutex_lock(&mutex1);//获得锁
		if (++i >= 6)//50*6=300ms执行一次
		{
			i = 0;
			time_1 = clock();//get time

			response = client.simGetImages(request);//用这个请求获取一个图像，得到响应

			if (response.size() > 0)//如果接收到的图像数大于0
			{
				//std::cout << response.size() << std::endl;

				for (const ImageResponse& image_info : response)//对每个图像处理
				{
					rgb_image = cv::imdecode(image_info.image_data_uint8, cv::IMREAD_COLOR);                 
				}
			}
		}
		pthread_mutex_unlock(&mutex1);//释放锁
	}//while
	return 0;
}

static int key_control(int key)//按键控制
{
	clock_t time_1;// = clock();//get time
	pthread_mutex_lock(&mutex1);//获得锁
	switch (key)
	{
	//F1
	case 7340032:
		std::cout << "press 'F1' for help" << std::endl;
		std::cout << "press 'ESC' exit" << std::endl;
		std::cout << "press 'T' Drone takeoff" << std::endl;
		std::cout << "press 'W' Drone rise" << std::endl;
		std::cout << "press 'S' Drone descend" << std::endl;
		std::cout << "press Arrow keys: Drone forward,retreat,left,right" << std::endl;
		std::cout << "press 'I' or 'K' or 'J' or 'L': Drone forward,retreat,left,right" << std::endl;
		break;
	case 32://空格
		client.moveByAngleThrottle(pitch, roll, throttle, yaw_rate, duration);
		break;
	case 27://ESC
		std::cout << "disconnection" << std::endl;
		pthread_mutex_unlock(&mutex1);//释放锁
		return -1;
		break;
	case 't'://take off
		std::cout << "takeoff,wait 1s " << std::endl;
		client.takeoff(1.0f);//最大起飞等待1s
		std::this_thread::sleep_for(std::chrono::duration<double>(1));
		client.moveByAngleThrottle(-0.01f, 0.0f, throttle, 0.0f, 0.05f);//执行一次后，GPS高度会变成负数，指导环境复位
		std::cout << "takeoff OK" << std::endl;
		break;
	case 'w':
		client.moveByAngleThrottle(0.0f, 0.0f, 0.7f, 0.0f, 0.2f);
		std::cout << "rise " << std::endl;
		break;//roll += 0.1f;//绕x轴逆时针 //单位是弧度
	case 's':
		client.moveByAngleThrottle(0.0f, 0.0f, 0.5f, 0.0f, 0.2f);
		std::cout << "fall " << std::endl;
		break;
	case 'a'://旋转时会下降...
		client.moveByAngleThrottle(0.0f, 0.0f, throttle, -1.0f, 0.2f);
		break;
	case 'd':
		client.moveByAngleThrottle(0.0f, 0.0f, throttle, 1.0f, 0.2f);
		break;

	case 2490368://高度控制增加，向上箭头
		Control_Z += 1.0f;
		printf("Control_Z=%f\n", Control_Z);
		break;
	case 2621440:////高度控制减少
		Control_Z -= 1.0f;
		printf("Control_Z=%f\n", Control_Z);
		break;
 
	//下面是以机头方向前后左右
	// bool moveByAngleThrottle(float pitch, float roll, float throttle, float yaw_rate, float duration);
	case 'i'://pitch y轴逆时针角度
		client.moveByAngleThrottle(-0.1f, 0.0f, throttle, 0.0f, 0.2f);
		break;
	case 'k'://pitch y轴逆时针角度
		client.moveByAngleThrottle(0.1f, 0.0f, throttle, 0.0f, 0.2f);
		break;
	case 'j'://roll x轴逆时针角度
		client.moveByAngleThrottle(0.0f, -0.1f, throttle, 0.0f, 0.2f);
		break;
	case 'l'://roll x轴逆时针角度
		client.moveByAngleThrottle(0.0f, 0.1f, throttle, 0.0f, 0.2f);
		break;
	case '0'://切换控制高度
		if (1 == Congtrol_Z_flag){
			Congtrol_Z_flag = 0;
		}
		else{
			Congtrol_Z_flag = 1;
		}
		printf("Congtrol_Z_flag=%d\n", Congtrol_Z_flag);
		break;
	case '1'://降落
		client.land();
		break;
	case '2':
		for (int i = 0; i < 100; ++i)
			ReviseBarometerData(i);
		break;
		
	case '3':
		ReviseMagagnetometerData();
		break;
	case '4':
		ReviseImuData();
		break;
	case '5':
		getAttitude();
		break;
	default:
		break;
	}
	key_value_cv = -1;
	pthread_mutex_unlock(&mutex1);//释放锁
	return 0;
}

