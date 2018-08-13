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

//msr::airlib::MultirotorRpcLibClient client ;//����localhost:41451
#include"client.h"
//�������������
#include"ReviseSensorError.h"
//��̬����
#include "attitude.h"


#define OPENCV

//using namespace std;
using namespace cv;
using namespace msr::airlib;

 
//���ID 0��4�ֱ��Ӧ������ǰ������ǰ������ǰ���������·�������󷽡�
//����ʱֻ���õײ�����ͷ����ͼ��ǰ������ͷ�ĳ���ͼ�����ͼ��
 
//�������ID
#define CAMERA_FRONT 0
#define CAMERA_FRONT_LEFT 1
#define CAMERA_FRONT_RIGHT 2
#define CAMERA_BELOW 3
#define CAMERA_BEHIND 4


//airsim ���
typedef ImageCaptureBase::ImageRequest ImageRequest;
typedef ImageCaptureBase::ImageResponse ImageResponse;
typedef ImageCaptureBase::ImageType ImageType;

msr::airlib::MultirotorRpcLibClient client("localhost", 41451);//����localhost:41451 


//msr::airlib::MultirotorRpcLibClient client("192.168.0.1", 41451);//����client��server����

 

DrivetrainType driveTrain = DrivetrainType::MaxDegreeOfFreedom;

 

float roll = 0.1f, roll_temp;//��x����ʱ�� //��λ�ǻ���
float pitch = 0.1f, pitch_temp;//��y����ʱ��  
float yaw = 0.0f; //��z����ʱ��
float duration = 0.2f;//����ʱ��
float throttle = 0.575f;
float yaw_rate = 0.1f;
float altitude_last;//��һ�ε�GPS�߶ȣ�������������
float altitude_cur;//��ǰ��GPS�߶ȣ�������������

				   //����������
GpsData		     GPS_data;
BarometerData	 Barometer_data;
MagnetometerData Magnetometer_data;
ImuData			 Imu_data;



//�߳�
DWORD WINAPI Key_Scan(LPVOID pVoid); 
DWORD WINAPI get_img(LPVOID pVoid);
DWORD WINAPI Control_Z_Thread(LPVOID pVoid);//��������߶�
 

HANDLE hTimer1;
pthread_mutex_t mutex1 = PTHREAD_MUTEX_INITIALIZER;//������									  
												    

volatile int key_value_cv = -1;
static int key_control(int key);//��������

								 
std::vector<ImageRequest> request = { ImageRequest(CAMERA_BELOW, ImageType::Scene) };//������ȡ���·��ĳ���ͼ(Ĭ��ѹ��)������
//std::vector<ImageRequest> request = { ImageRequest(CAMERA_FRONT, ImageType::Scene) };//������ȡ�ĳ���ͼ(Ĭ��ѹ��)������
std::vector<ImageResponse> response;//��Ӧ
cv::Mat rgb_image; 

uint8_t Congtrol_Z_flag = 0;//1��ʾҪ���Ƹ߶�
uint8_t Congtrol_Centre_flag = 0;//1��ʾҪ����ͣ��������
								  

double Control_Z = -600.0f;//���Ƶĸ߶�
double Control_Z_tolerance = 0.5f;// 0.01f;//���Ƹ߶ȵ�����
PID my_PID;

//��Ԫ��������̬
#define Kp 100.0f  //���ٶ�Ȩ�أ�Խ������ٶȲ���ֵ����Խ��
#define Ki  0.001f //����������
#define halfT 0.001f; //�������ڵ�һ��

float Yaw=0,Pitch=0,Roll=0;

//��Ԫ��
float q0=1,q1=0, q2=0, q3=0;
float exInt = 0, eyInt = 0, ezInt = 0;//��������С�������

#define PI 3.1415925

int main()
{
	while (RpcLibClientBase::ConnectionState::Connected != client.getConnectionState())
		client.confirmConnection();//���ӳ���
	while (!client.isApiControlEnabled())
		client.enableApiControl(true);//��ȡapi����
	client.armDisarm(true);//�����ɿ�

	client.hover();//hoverģʽ

    //���ȴ����ɵȺ�ʱ��
	hTimer1 = CreateWaitableTimer(NULL, FALSE, NULL);
	//���ö�ʱ��ʱ��
	INT64 nDueTime = -0 * _SECOND;//��ʱ����Чʱ�䣬����
	SetWaitableTimer(hTimer1, (PLARGE_INTEGER)&nDueTime, 50, NULL, NULL, FALSE);//50��ʾ��ʱ������50ms
																				 

	HANDLE hThread2 = CreateThread(NULL, 0, get_img, NULL, 0, NULL);// &nThreadID);
	HANDLE hThread1 = CreateThread(NULL, 0, Key_Scan, NULL, 0, NULL);//����� 0 ��ʾ���������������߳�
	HANDLE hThread3 = CreateThread(NULL, 0, Control_Z_Thread, NULL, 0, NULL); 
 
 

	while (true)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(20));//������ǰ����
		if (-1 != key_value_cv)//����а�������
		{
			if (-1 == key_control(key_value_cv))
			{
				//�ݻ�����OpenCV����
				cv::destroyAllWindows();
				//�ر��߳�
				CloseHandle(hThread1);
				CloseHandle(hThread2);
				CloseHandle(hThread3);
		 
				//�رն�ʱ��
				CloseHandle(hTimer1);
				return 0;
			}
		}
	}

	//�ȴ��̷߳���
	WaitForSingleObject(hThread1, INFINITE);
	WaitForSingleObject(hThread2, INFINITE);
	CloseHandle(hThread1);
	CloseHandle(hThread2);
	CloseHandle(hThread3);
 
	//�رն�ʱ��
	CloseHandle(hTimer1);
	return 0;
}

//������ѹ������
void  ReviseBarometerData(int i) {
	Barometer_data = client.getBarometerdata();
	ofstream out;
	out.open("Bar.txt", ios::app);
	if (!out.is_open())
    {
		cout << "δ�ɹ����ļ�" << endl;
	}
	out<< Barometer_data.altitude<< "  "<< Barometer_data.pressure<<" "<<i<<endl;
	out.close();

	std::cout <<"************��ѹ������*************"<< std::endl;
	std::cout << "altitude��" << Barometer_data.altitude << std::endl;
	std::cout << "pressure: " <<Barometer_data.pressure<< std::endl;
	std::cout << "time_stamp: "<< Barometer_data.time_stamp << std::endl;
}

//�������������,
void ReviseMagagnetometerData() {
	Magnetometer_data = client.getMagnetometerdata();
	std::cout << "************����������*************" << std::endl;
	std::cout << "magnetic_field_body��" << Magnetometer_data.magnetic_field_body<<std::endl;
	std::cout << "yaw:" <<atan2(Magnetometer_data.magnetic_field_body(1,0), Magnetometer_data.magnetic_field_body(0,0))*57.3 << std::endl;
	std::cout << "time_stamp: " << Magnetometer_data.time_stamp << std::endl;
	 
}

//����IMU����
void ReviseImuData(){
	Imu_data = client.getImudata();
	std::cout << "************IMU����*************" << std::endl;
	//���ٶ�
	std::cout << "angular_velocity��" << Imu_data.angular_velocity<< std::endl;
	//�߼��ٶ�
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
		//�ȴ���ʱ��ʱ�䵽��
		WaitForSingleObject(hTimer1, INFINITE);
		//�����Ҫ���Ƹ߶�
		if (1 == Congtrol_Z_flag){
			//P ��������
			double Throttle_Z = my_PID.PIDZ(Control_Z, Control_Z_tolerance);//�趨�߶ȣ�����
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
		//�ȴ���ʱ��ʱ�䵽��
		WaitForSingleObject(hTimer1, INFINITE);

		pthread_mutex_lock(&mutex1);//�����

									//��ʾͼ��
		if (!rgb_image.empty()) {
			cv::imshow("original_img", rgb_image);
			cvWaitKey(0);
		}
		else
		{
			cvWaitKey(1);
			std::cout << "failed" << std::endl;
		}


		//����ɨ��
		if (-1 == key_value_cv)
		{
			//�ȴ��������£����ҷ���������ֵ
			key_value_cv = cv::waitKeyEx(0);
		}
		else
		{
			cv::waitKey(1);
		}
		//while (-1 != cv::waitKey(1));//�ѻ���������󣬲Ż���ʾ1msͼ��
		pthread_mutex_unlock(&mutex1);//�ͷ���
	}
	return 0;
}

DWORD WINAPI get_img(LPVOID pVoid)
{
	int i = 0;
	clock_t time_1;// = clock();//get time


	while (true)
	{
		//�ȴ���ʱ��ʱ�䵽��
		WaitForSingleObject(hTimer1, INFINITE);

		pthread_mutex_lock(&mutex1);//�����
		if (++i >= 6)//50*6=300msִ��һ��
		{
			i = 0;
			time_1 = clock();//get time

			response = client.simGetImages(request);//����������ȡһ��ͼ�񣬵õ���Ӧ

			if (response.size() > 0)//������յ���ͼ��������0
			{
				//std::cout << response.size() << std::endl;

				for (const ImageResponse& image_info : response)//��ÿ��ͼ����
				{
					rgb_image = cv::imdecode(image_info.image_data_uint8, cv::IMREAD_COLOR);                 
				}
			}
		}
		pthread_mutex_unlock(&mutex1);//�ͷ���
	}//while
	return 0;
}

static int key_control(int key)//��������
{
	clock_t time_1;// = clock();//get time
	pthread_mutex_lock(&mutex1);//�����
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
	case 32://�ո�
		client.moveByAngleThrottle(pitch, roll, throttle, yaw_rate, duration);
		break;
	case 27://ESC
		std::cout << "disconnection" << std::endl;
		pthread_mutex_unlock(&mutex1);//�ͷ���
		return -1;
		break;
	case 't'://take off
		std::cout << "takeoff,wait 1s " << std::endl;
		client.takeoff(1.0f);//�����ɵȴ�1s
		std::this_thread::sleep_for(std::chrono::duration<double>(1));
		client.moveByAngleThrottle(-0.01f, 0.0f, throttle, 0.0f, 0.05f);//ִ��һ�κ�GPS�߶Ȼ��ɸ�����ָ��������λ
		std::cout << "takeoff OK" << std::endl;
		break;
	case 'w':
		client.moveByAngleThrottle(0.0f, 0.0f, 0.7f, 0.0f, 0.2f);
		std::cout << "rise " << std::endl;
		break;//roll += 0.1f;//��x����ʱ�� //��λ�ǻ���
	case 's':
		client.moveByAngleThrottle(0.0f, 0.0f, 0.5f, 0.0f, 0.2f);
		std::cout << "fall " << std::endl;
		break;
	case 'a'://��תʱ���½�...
		client.moveByAngleThrottle(0.0f, 0.0f, throttle, -1.0f, 0.2f);
		break;
	case 'd':
		client.moveByAngleThrottle(0.0f, 0.0f, throttle, 1.0f, 0.2f);
		break;

	case 2490368://�߶ȿ������ӣ����ϼ�ͷ
		Control_Z += 1.0f;
		printf("Control_Z=%f\n", Control_Z);
		break;
	case 2621440:////�߶ȿ��Ƽ���
		Control_Z -= 1.0f;
		printf("Control_Z=%f\n", Control_Z);
		break;
 
	//�������Ի�ͷ����ǰ������
	// bool moveByAngleThrottle(float pitch, float roll, float throttle, float yaw_rate, float duration);
	case 'i'://pitch y����ʱ��Ƕ�
		client.moveByAngleThrottle(-0.1f, 0.0f, throttle, 0.0f, 0.2f);
		break;
	case 'k'://pitch y����ʱ��Ƕ�
		client.moveByAngleThrottle(0.1f, 0.0f, throttle, 0.0f, 0.2f);
		break;
	case 'j'://roll x����ʱ��Ƕ�
		client.moveByAngleThrottle(0.0f, -0.1f, throttle, 0.0f, 0.2f);
		break;
	case 'l'://roll x����ʱ��Ƕ�
		client.moveByAngleThrottle(0.0f, 0.1f, throttle, 0.0f, 0.2f);
		break;
	case '0'://�л����Ƹ߶�
		if (1 == Congtrol_Z_flag){
			Congtrol_Z_flag = 0;
		}
		else{
			Congtrol_Z_flag = 1;
		}
		printf("Congtrol_Z_flag=%d\n", Congtrol_Z_flag);
		break;
	case '1'://����
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
	pthread_mutex_unlock(&mutex1);//�ͷ���
	return 0;
}

