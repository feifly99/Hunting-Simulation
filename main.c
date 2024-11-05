#include "HuntingSimulation.h"

int main()
{
	srand((UINT)time(NULL));
	FILE* fp = fopen("output.txt", "w");
	PTUSV target = NULL;
	//如果更改目标的坐标，请注意捕食者的链表链接顺序.
	Point targetPos = { 16 - rand() % 4, 9 - rand() % 4 }; // - rand() % 4
	ckFloatValue(targetPos.x); ckFloatValue(targetPos.y);
	PHUSV hunter[huntersNum] = { NULL };
	//从目标坐标的右侧射线方向逆时针旋转，第一个碰到的捕食者为链表头，最后一个碰到的为链表尾.
	//一定一定要保证链表顺序！只要围捕的时候撞上绝对是链表顺序问题！！！
	Point hunterPos[huntersNum] = { {0, 0} };
	double beginAngle = 160.0;
	double beginRadius = 2.0 * targetSafetyRadius;
	for (size_t j = 0; j < huntersNum; j++)
	{
		hunterPos[j].x = targetPos.x + beginRadius * cosa(beginAngle + 14.0 * (double)j);
		hunterPos[j].y = targetPos.y + beginRadius * sina(beginAngle + 14.0 * (double)j);
	}
	/*double Xx = targetPos.x + beginRadius;
	double Yy = targetPos.y;
	hunterPos[0].x = Xx;
	hunterPos[0].y = Yy + 6;
	hunterPos[1].x = Xx;
	hunterPos[1].y = Yy + 12;
	hunterPos[2].x = Xx;
	hunterPos[2].y = Yy - 12;
	hunterPos[3].x = Xx;
	hunterPos[3].y = Yy - 6;*/
	UCHAR typeHuntingCase = 0;
	UCHAR typeEscapingCase = 1;
	double hunterHeadDirection[huntersNum] = { 0.0 };
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
			hunterHeadDirection[j],
			j
		);
	);
	buildCycleListByHuntersOriginalIndex(hunter); //补全了hunter结构的LIST_ENTRY项
	fors(
		huntersNum,
		_SURROUNDING_getHuntersFourEIByHuntersLocAndHeadDirection(&hunter[j], target); //补全了hunter结构的EI项
		_SURROUNDING_getRewardPointsForHunterByHuntersEI(hunter[j]);
		QAQ;
	);

	size_t surroundingAttempCount = 0x0;

	while (surroundingAttempCount < 200)
	{
		if (!isSurroundingSuccess(hunter, target)) //!isSurroundingSuccess(hunter, target)
		{
			//如果出现三个捕食者都不动弹，那么：
			//1. (20%) 一定要注意三个捕食者的链表顺序！比如目标在16 9，那么如果3 2和1 8的位置反了，就可能会出现所有捕食者都固定不动的异常现象；
			//2. (70%) 如果链接顺序正确但是捕食者追捕过程中仍然出现全部停滞不动的状况，尝试调整J中w_idealAngleDiffer的权重值：-16 9时不收敛，但是把w_idealAngleDiffer从5改为25之后就收敛了；
			//3. (10%) 放宽一下收敛条件，目前收敛条件算是比较严格的！（调整maxDistanceTolerance和maxAngleTolerance）
			//注1：调试时，只需要把while(1)改为surroundingAttempCount < 200即可.
			//注2：如果发现某个捕食者离目标的距离越来越大，或者某一个距离反复横跳两个值，那么通常是角度权重太大了导致捕食者不择手段来提高角度精度而忽略了距离权重！
			fors(
				huntersNum,
				_SURROUNDING_changingHunterInfomationByRewardFunction(&hunter[j], target);
			);
			//_SURROUNDING_remakeHuntersCycleListByRelativeLocCompareToTarget(&hunter, target);
			//注意printf必须在这里打印才能确保距离大于safetyRadius + mostSafetyRadius！
			//只有在调整完位置后，距离条件大于safetyRadius + mostSafetyRadius才能满足；
			//否则打印的是前一次的位置信息，造成了视觉上错误，但本质上是对的。
			printf("未收敛. surroundingAttempCount: %zu\n", surroundingAttempCount);
			fors(
				huntersNum,
				printf("[当前距离信息] %zu -> %lf\n", j, calculate_P$P_distance(hunter[j]->pos, target->pos));
			);
			//printf也不能放在movingNormalTargetRandomly之下，原因同上。
			movingNormalTargetStably(&target);
			makePythonFile(fp, hunter, target);
			QAQ;
			surroundingAttempCount++;
			if (surroundingAttempCount == 100)
			{
				system("pause");
			}
			continue;
		}
		else
		{
			printf("已经收敛！迭代次数: %zu -> [当前距离信息] 0: %lf, 1: %lf, 2: %lf\n", surroundingAttempCount, calculate_P$P_distance(hunter[0]->pos, target->pos), calculate_P$P_distance(hunter[1]->pos, target->pos), calculate_P$P_distance(hunter[2]->pos, target->pos));
			printf("\n最后一次显示的距离信息可能不符合条件<所有捕食者到目标的距离全都大于mostSafetyRadius + 1 * RtN + 1 * Rh>\n\n这是正常的，因为打印此消息时目标已经进行了一次移动！\n");
			makePythonFile(fp, hunter, target);
			QAQ;
			break;
		}
	}
	//合围已经成功，现在开始进行回收过程.
	system("pause");
	//从现在开始根据猎人和目标的行为策略进行模拟追捕过程：
	
	if(typeHuntingCase == 0 && typeEscapingCase == 0)
	{
		/*
			测试例程1：捕食者直接追捕[0]&目标以向量方法逃逸[0]：
				追捕策略：直接追击
				逃逸策略：向量逃逸
				期望追捕结果：失败
				仿真测试结果：失败
				是否符合预期：是
		*/
		size_t testRoutine_type1_totalSteps = 0;
		while (!isRecyclingSuccess(hunter, target))
		{
			fors(
				huntersNum,
				_HUNTER_goTowardsTargetDirectly(&hunter[j], target);
			);
			printf("测试例程:1 --- 当前模拟次数: %zu\n", ++testRoutine_type1_totalSteps);
			fors(
				huntersNum,
				printf("[距离信息] hunter[%zu] <-> target: %lf\n", j, calculate_P$P_distance(hunter[j]->pos, target->pos));
			);
			_TARGET_escapingAngleByVectorMethod(hunter, &target);
			makePythonFile(fp, hunter, target);
			if (isTerminatingRecycling(hunter, target))
			{
				printf("合围失败!\n");
				break;
			}
			if (testRoutine_type1_totalSteps >= 20)
			{
				break;
			}
			else
			{
				continue;
			}
			
		}
	}
	else if(typeHuntingCase == 0 && typeEscapingCase == 1)
	{
		/*
			测试例程2：捕食者直接追捕[0]&目标以学习方法逃逸[1]：
				追捕策略：直接追击
				逃逸策略：学习逃逸
				期望追捕结果：失败
				仿真测试结果：失败
				是否符合预期：是
		*/
		size_t testRoutine_type2_totalSteps = 0;
		while (!isRecyclingSuccess(hunter, target))
		{
			fors(
				huntersNum,
				_HUNTER_goTowardsTargetDirectly(&hunter[j], target);
			);
			printf("测试例程:2 --- 当前模拟次数: %zu\n", ++testRoutine_type2_totalSteps);
			fors(
				huntersNum,
				printf("[距离信息] hunter[%zu] <-> target: %lf\n", j, calculate_P$P_distance(hunter[j]->pos, target->pos));
			);
			_TARGET_escapingAngleByLearningMethod(hunter, &target);
			makePythonFile(fp, hunter, target);
			if (isTerminatingRecycling(hunter, target))
			{
				printf("合围失败!\n");
				break;
			}
			if (testRoutine_type2_totalSteps >= 20)
			{
				break;
			}
		}
	}
	else if (typeHuntingCase == 1 && typeEscapingCase == 0)
	{
		/*
			测试例程3：捕食者以狼群捕猎模式追捕[1]&目标以向量方法逃逸[0]：
				追捕策略：狼群模式
				逃逸策略：向量逃逸
				期望追捕结果：成功
				仿真测试结果：未知
				是否符合预期：未知
		*/
		size_t testRoutine_type2_totalSteps = 200;
		while (testRoutine_type2_totalSteps--)
		{
			_HUNTER_goTowardsTargetDirectly(hunter, target);
			//_TARGET_escapingAngleByLearningMethod(hunter, &target);
			printf("测试例程:1 --- 当前模拟次数: %zu", testRoutine_type2_totalSteps);
			fors(
				huntersNum,
				printf("[距离信息] hunter[%zu] <-> target: %lf\n", j, calculate_P$P_distance(hunter[j]->pos, target->pos));
			);
			makePythonFile(fp, hunter, target);
		}
	}
	else if (typeHuntingCase == 1 && typeEscapingCase == 1)
	{
		/*
			测试例程4：捕食者以狼群捕猎模式追捕[1]&目标以学习方法逃逸[1]：
				追捕策略：狼群模式
				逃逸策略：学习逃逸
				期望追捕结果：成功
				仿真测试结果：未知
				是否符合预期：未知
		*/
		size_t testRoutine_type2_totalSteps = 200;
		while (testRoutine_type2_totalSteps--)
		{
			_HUNTER_goTowardsTargetDirectly(hunter, target);
			//_TARGET_escapingAngleByLearningMethod(hunter, &target);
			printf("测试例程:1 --- 当前模拟次数: %zu", testRoutine_type2_totalSteps);
			fors(
				huntersNum,
				printf("[距离信息] hunter[%zu] <-> target: %lf\n", j, calculate_P$P_distance(hunter[j]->pos, target->pos));
			);
			makePythonFile(fp, hunter, target);
		}
	}
	else
	{
		printf("非法测试用例编号，已驳回.\n");
		//流程异常结束，释放申请的内存，强制退出程序
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

	//流程正常结束，释放申请的内存，退出程序
	fors(
		huntersNum,
		free(hunter[j]);
		hunter[j] = NULL;
	);
	free(target);
	target = NULL;
	fclose(fp);
	return 0xFF;
}
