// stdafx.cpp : source file that includes just the standard includes
//	MicroClient.pch will be the pre-compiled header
//	stdafx.obj will contain the pre-compiled type information

#include "stdafx.h"
#include "StrategySystem.h"

//�ѽ��յ��Ķ�Ա������ͽǶȷ���r[0][i]���棬�Է���Ա������
//����op����
//ȡÿ����Ա�����٣���Velocity�������������Ѿ��������
//����Ա������
int nKick;
CStrategySystem* thePlannerR=new CStrategySystem(0);
CStrategySystem* thePlannerL=new CStrategySystem(1);
Robot2 r[2][11];
Robot3 op;//��ŶԷ���Ա����������
Robot1 theball;//��������������.