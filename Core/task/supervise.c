#include "supervise.h"

int lost_err =  0x0000;                   //�����ش����־���ñ�����12λ��ÿһλ����Ϊһ�������Ƿ�ʧ�ı�־λ��һ���ɼ��12�����衣0����δ��ʧ��1����ʧ
int lost_counter[DETECT_NUM] = {0};       //���������Ӧ�ĵ�����������������������˵��

void SuperviseTaskHandle(void)
{
  for(int i = 0; i < DETECT_NUM; i++)     //����ɨ���������ĵ����������Ƿ����
	{
		if(lost_counter[i] < 4)              //���û����������������������ʧ��־λ��0
		{
			lost_counter[i]++;
			lost_err &= ~(1<<i);
		}else{                                //��������������������ʧ��־λ��1
			lost_err |= (1<<i);
		}
	}
}

/**
* @brief ���ػ����˸�������ĵ�����������ι����
* @param None
* @retval None
*/
void LostCounterFeed(int index)
{
	lost_counter[index] = 0;
}

/**
* @brief �жϻ�����ָ�������Ƿ�ʧ
* @param ������
* @retval �����Ƿ�ʧ��0����δ��ʧ����0����ʧ
*/
int Is_Error(int index)
{
	return (index&lost_err);
}

/**
* @brief �жϻ������Ƿ�ʧ��Ҫ����
* @param None
* @retval 0����δ��ʧ��1����ʧ������ֵ��
*/
int Is_Serious_Error(void)
{
  return Is_Error(1<<0);
}

int Is_Any_Error(void)
{
  return lost_err;
}
