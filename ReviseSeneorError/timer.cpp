//WaitableTimer.cpp : �������̨Ӧ�ó������ڵ㡣
#include "timer.h" 

/************************************************************************/
/* ʹ�ÿɵȺ�ʱ�������߳�ͬ��,�ɵȺ�ʱ�������߳�ͬ����ԭ��ʱ����,���������ķ�ʽ.
��ָ�������ʱ��֪ͨ��һ���߳�,�Ӷ�ʵ��ͬ��,���ȸ�,ԼΪ100����*/
/************************************************************************/


using namespace std;

DWORD WINAPI ThreadFun(LPVOID pVoid);//__stdcall �����ڷ��ص�������֮ǰ��������ջ��ɾ��
HANDLE g_hTimer;
/*int main()
{
int i = 0;
//���ȴ����ɵȺ�ʱ��
g_hTimer = CreateWaitableTimer(NULL, FALSE, NULL);
//���ö�ʱ��ʱ��
INT64 nDueTime = -2 * _SECOND;
SetWaitableTimer(g_hTimer, (PLARGE_INTEGER)&nDueTime, 1000, NULL, NULL, FALSE);
DWORD nThreadID = 0;
HANDLE hThread = CreateThread(NULL, 0, ThreadFun, NULL, 0, &nThreadID);

//���̵߳�ִ��·��
for (i = 0; ; ++i)
{
//cout << "���߳�:i = " << i << endl;

//Sleep(100);
//cv::waitKey(100);
}
//�ȴ��̷߳���
WaitForSingleObject(hThread, INFINITE);
CloseHandle(hThread);
//�رն�ʱ��
CloseHandle(g_hTimer);
return 0;
}*/

DWORD WINAPI ThreadFun(LPVOID pVoid)
{
	LONG i = 0;
	while (1)
	{
		//�ȴ���ʱ��ʱ�䵽��
		WaitForSingleObject(g_hTimer, INFINITE);
		printf("%d��ʼִ��\n", i++);
		//printf("%d��ʼִ��\n", *(int*)pVoid);
	}
	return 0;
}



//#include <windows.h>
//#include <iostream>
//
//using namespace std;
//
////�̺߳���
//DWORD WINAPI ThreadProc(LPVOID lpParameter)
//{
//	for (int i = 0; i < 50; ++i)
//	{
//		cout << "���߳�:i = " << i << endl;
//		//Sleep(100);
//	}
//	return 0L;
//}
//
//int main()
//{
//	//����һ���߳�
//	HANDLE thread = CreateThread(NULL, 0, ThreadProc, NULL, 0, NULL);
//	//�ر��߳�
//	CloseHandle(thread);
//
//	//���̵߳�ִ��·��
//	for (int i = 0; i < 50; ++i)
//	{
//		cout << "���߳�:i = " << i << endl;
//		//Sleep(100);
//	}
//
//	return 0;
//}