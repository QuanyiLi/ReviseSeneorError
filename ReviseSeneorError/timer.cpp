//WaitableTimer.cpp : 定义控制台应用程序的入口点。
#include "timer.h" 

/************************************************************************/
/* 使用可等候定时器进行线程同步,可等候定时器进行线程同步的原理定时触发,或间隔触发的方式.
在指定间隔的时间通知另一个线程,从而实现同步,精度高,约为100纳秒*/
/************************************************************************/


using namespace std;

DWORD WINAPI ThreadFun(LPVOID pVoid);//__stdcall 函数在返回到调用者之前将参数从栈中删除
HANDLE g_hTimer;
/*int main()
{
int i = 0;
//首先创建可等候定时器
g_hTimer = CreateWaitableTimer(NULL, FALSE, NULL);
//设置定时器时间
INT64 nDueTime = -2 * _SECOND;
SetWaitableTimer(g_hTimer, (PLARGE_INTEGER)&nDueTime, 1000, NULL, NULL, FALSE);
DWORD nThreadID = 0;
HANDLE hThread = CreateThread(NULL, 0, ThreadFun, NULL, 0, &nThreadID);

//主线程的执行路径
for (i = 0; ; ++i)
{
//cout << "主线程:i = " << i << endl;

//Sleep(100);
//cv::waitKey(100);
}
//等待线程返回
WaitForSingleObject(hThread, INFINITE);
CloseHandle(hThread);
//关闭定时器
CloseHandle(g_hTimer);
return 0;
}*/

DWORD WINAPI ThreadFun(LPVOID pVoid)
{
	LONG i = 0;
	while (1)
	{
		//等待定时器时间到达
		WaitForSingleObject(g_hTimer, INFINITE);
		printf("%d开始执行\n", i++);
		//printf("%d开始执行\n", *(int*)pVoid);
	}
	return 0;
}



//#include <windows.h>
//#include <iostream>
//
//using namespace std;
//
////线程函数
//DWORD WINAPI ThreadProc(LPVOID lpParameter)
//{
//	for (int i = 0; i < 50; ++i)
//	{
//		cout << "子线程:i = " << i << endl;
//		//Sleep(100);
//	}
//	return 0L;
//}
//
//int main()
//{
//	//创建一个线程
//	HANDLE thread = CreateThread(NULL, 0, ThreadProc, NULL, 0, NULL);
//	//关闭线程
//	CloseHandle(thread);
//
//	//主线程的执行路径
//	for (int i = 0; i < 50; ++i)
//	{
//		cout << "主线程:i = " << i << endl;
//		//Sleep(100);
//	}
//
//	return 0;
//}