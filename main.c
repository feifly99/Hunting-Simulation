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
	buildCycleList(hunter); //补全了hunter结构的LIST_ENTRY项
	fors(
		huntersNum,
		getHuntersFourEIByHuntersLocAndHeadDirection(hunter[j], target); //补全了hunter结构的EI项
		getRewardPointsForHunterByHuntersEI(hunter[j]);
		QAQ;
	);
	size_t cnt = 0x0;
	while(1)	
	{
		if(!isSurroundingSuccess(hunter, target))
		{
			printf("未收敛. 迭代次数: %zu -> [当前距离信息] 0: %lf, 1: %lf, 2: %lf\n", cnt, calculate_P$P_distance(hunter[0]->pos, target->pos), calculate_P$P_distance(hunter[1]->pos, target->pos), calculate_P$P_distance(hunter[2]->pos, target->pos));
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
			printf("已经收敛！迭代次数: %zu -> [当前距离信息] 0: %lf, 1: %lf, 2: %lf\n", cnt, calculate_P$P_distance(hunter[0]->pos, target->pos), calculate_P$P_distance(hunter[1]->pos, target->pos), calculate_P$P_distance(hunter[2]->pos, target->pos));
			makePythonFile(fp, hunter, target);
			QAQ;
			break;
		}
	}
	//已经合围成功，开始进行回收


	fclose(fp);
	return 0;
}