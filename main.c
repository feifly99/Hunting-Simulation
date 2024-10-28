#include "HuntingSimulation.h"

int main()
{
	srand((UINT)time(NULL));
	FILE* fp = fopen("output.txt", "w");
	PTUSV target = NULL;
	Point targetPos = { 20, 9 };
	PHUSV hunter[huntersNum] = { NULL };
	Point hunterPos[huntersNum] = { {7, 18}, {3, 2},{1, 8} };
	double hunterHeadDirection[3] = { 30, 45, 60 };
	EI huntersEI[3] = { { 0.0 } }; 
	initializeTargetUSV(
		&target, 
		targetPos, 
		targetNormalVelocity, 
		targetEscapingVelocity, 
		targetAlertingRadius, 
		targetSafetyRadius, 
		targetMovableAngleRange, 
		30
	);
	fors(
		huntersNum,
		initializeHunterUsv(
			&hunter[j], 
			hunterPos[j], 
			huntersVelocity, 
			huntersMovableAngleRange,
			hunterHeadDirection[j]
		);
	);
	buildCycleList(hunter); //��ȫ��hunter�ṹ��LIST_ENTRY��
	fors(
		huntersNum,
		getHuntersFourEIByHuntersLocAndHeadDirection(hunter[j], target); //��ȫ��hunter�ṹ��EI��
		getRewardPointsForHunterByHuntersEI(hunter[j]);
		QAQ;
	);
	size_t cnt = 0x0;
	while(1)	
	{
		if(!isSurroundingSuccess(hunter, target))
		{
			printf("δ����. ��������: %zu -> [��ǰ������Ϣ] 0: %lf, 1: %lf, 2: %lf\n", cnt, calculate_P$P_distance(hunter[0]->pos, target->pos), calculate_P$P_distance(hunter[1]->pos, target->pos), calculate_P$P_distance(hunter[2]->pos, target->pos));
			changingHunterInfomationByRewardFunction(&hunter[0], target);
			changingHunterInfomationByRewardFunction(&hunter[1], target);
			changingHunterInfomationByRewardFunction(&hunter[2], target);
			movingNormalTargetRandomly(&target);
			makePythonFile(fp, hunter, target);
			QAQ;
			cnt++;
			continue;
		}
		else
		{
			printf("�Ѿ���������������: %zu -> [��ǰ������Ϣ] 0: %lf, 1: %lf, 2: %lf\n", cnt, calculate_P$P_distance(hunter[0]->pos, target->pos), calculate_P$P_distance(hunter[1]->pos, target->pos), calculate_P$P_distance(hunter[2]->pos, target->pos));
			makePythonFile(fp, hunter, target);
			QAQ;
			break;
		}
	}
	//�Ѿ���Χ�ɹ�����ʼ���л���


	fclose(fp);
	return 0;
}