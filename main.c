#include "HuntingSimulation.h"

int main()
{
	srand((UINT)time(NULL));
	FILE* fp = fopen("output.txt", "w");
	PTUSV target = NULL;
	//如果更改目标的坐标，请注意捕食者的链表链接顺序.
	Point targetPos = { -16, 9 };
	PHUSV hunter[huntersNum] = { NULL };
	//从目标坐标的右侧射线方向逆时针旋转，第一个碰到的捕食者为链表头，最后一个碰到的为链表尾.
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
		_SURROUNDING_getHuntersFourEIByHuntersLocAndHeadDirection(hunter[j], target); //补全了hunter结构的EI项
	_SURROUNDING_getRewardPointsForHunterByHuntersEI(hunter[j]);
	QAQ;
		);

	size_t cnt = 0x0;
	while (cnt < 250)
	{
		//由于捕食者速度在目标未发现之前快于目标，所以合围100%能实现.
		if (!isSurroundingSuccess(hunter, target)) //!isSurroundingSuccess(hunter, target)
		{
			//如果出现三个捕食者都不动弹，那么：
			//1. (20%) 一定要注意三个捕食者的链接顺序！比如目标在16 9，那么如果3 2和1 8的位置反了，就可能会出现所有捕食者都固定不动的异常现象；
			//2. (70%) 如果链接顺序正确但是捕食者追捕过程中仍然出现全部停滞不动的状况，尝试调整J中w_idealAngleDiffer的权重值：-16 9时不收敛，但是把w_idealAngleDiffer从5改为25之后就收敛了；
			//3. (10%) 放宽一下收敛条件，目前收敛条件算是比较严格的！（调整maxDistanceTolerance和maxAngleTolerance）
			//注1：调试时，只需要把while(1)改为cnt < 200即可.
			//注2：如果发现某个捕食者离目标的距离越来越大，或者某一个距离反复横跳两个值，那么通常是角度权重太大了导致捕食者不择手段来提高角度精度而忽略了距离权重！
			_SURROUNDING_changingHunterInfomationByRewardFunction(&hunter[0], target);
			_SURROUNDING_changingHunterInfomationByRewardFunction(&hunter[1], target);
			_SURROUNDING_changingHunterInfomationByRewardFunction(&hunter[2], target);
			//注意printf必须在这里打印才能确保距离大于safetyRadius + mostSafetyRadius！
			//只有在调整完位置后，距离条件大于safetyRadius + mostSafetyRadius才能满足；
			//否则打印的是前一次的位置信息，造成了视觉上错误，但本质上是对的。
			printf("未收敛. 迭代次数: %zu -> [当前距离信息] 0: %lf, 1: %lf, 2: %lf\n", cnt, calculate_P$P_distance(hunter[0]->pos, target->pos), calculate_P$P_distance(hunter[1]->pos, target->pos), calculate_P$P_distance(hunter[2]->pos, target->pos));
			//printf也不能放在movingNormalTargetRandomly之下，原因同上。
			//movingNormalTargetRandomly(&target);
			makePythonFile(fp, hunter, target);
			QAQ;
			cnt++;
			continue;
		}
		else
		{
			printf("已经收敛！迭代次数: %zu -> [当前距离信息] 0: %lf, 1: %lf, 2: %lf\n", cnt, calculate_P$P_distance(hunter[0]->pos, target->pos), calculate_P$P_distance(hunter[1]->pos, target->pos), calculate_P$P_distance(hunter[2]->pos, target->pos));
			printf("\n最后一次显示的距离信息可能不符合条件<所有捕食者到目标的距离全都大于mostSafetyRadius + 1 * RtN + 1 * Rh>\n\n这是正常的，因为打印此消息时目标已经进行了一次移动！\n");
			makePythonFile(fp, hunter, target);
			QAQ;
			break;
		}
	}


	//流程结束，释放使用的内存，退出程序
	fors(
		huntersNum,
		free(hunter[j]);
	hunter[j] = NULL;
		);
	free(target);
	target = NULL;
	fclose(fp);
	return 0;
}
