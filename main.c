#include "HuntingSimulation.h"

static size_t huntersNumRealTime = (size_t)huntersNum;

#define getFlink(j) CONTAINING_RECORD(hunter[j]->hunterListEntry.Flink, HUSV, hunterListEntry)
#define getBlink(j) CONTAINING_RECORD(hunter[j]->hunterListEntry.Blink, HUSV, hunterListEntry)

#pragma warning(disable: 6387 6011)

int main()
{
	srand((UINT)time(NULL));
	FILE* fp = fopen("simulationData.txt", "w");
	PTUSV target = NULL;
	//如果更改目标的坐标，请注意捕食者的链表链接顺序.
	Point targetPos = { 16 , 9 }; // - rand() % 4
	ckFloatValue(targetPos.x); ckFloatValue(targetPos.y);
	PHUSV hunter[huntersNum] = { NULL };
	//从目标坐标的右侧射线方向逆时针旋转，第一个碰到的捕食者为链表头，最后一个碰到的为链表尾.
	//一定一定要保证链表顺序！只要围捕的时候撞上绝对是链表顺序问题！！！
	Point hunterPos[huntersNum] = { {0, 0} };
	double beginAngle = 30.0;
	double beginRadius = 3.5 * targetSafetyRadius;
	for (size_t j = 0; j < huntersNum; j++)
	{
		hunterPos[j].x = targetPos.x + beginRadius * cosa(beginAngle + 2.00 * (double)j);
		hunterPos[j].y = targetPos.y + beginRadius * sina(beginAngle + 2.00 * (double)j);
	}
	initializeTargetUSV(
		&target,
		targetPos,
		targetNormalVelocity,
		targetSafetyRadius
	);
	fors(
		huntersNum,
		initializeHunterUsv(
			&hunter[j],
			hunterPos[j],
			huntersVelocity,
			j
		);
	);
	buildCycleListByHuntersOriginalIndex(hunter); //补全了hunter结构的LIST_ENTRY项

	double obstacleSize = 9.0;
	double obstacleVelocity = 1.20;
	Point* obstaclesCenterPos = (Point*)malloc(obstaclesNum * sizeof(Point));
	RtlZeroMemory(obstaclesCenterPos, obstaclesNum * sizeof(Point));
	for (int i = 0; i < obstaclesNum; i++) 
	{
		double angle = cvtAngle2Rad((double)(rand() % 360));
		double radius = ((double)rand() / RAND_MAX) * 2.0 * targetSafetyRadius;

		obstaclesCenterPos[i].x = targetPos.x + radius * cos(angle);
		obstaclesCenterPos[i].y = targetPos.y + radius * sin(angle);
	}
	double* obstaclesMovingDirection = (double*)malloc(obstaclesNum * sizeof(double));
	RtlZeroMemory(obstaclesMovingDirection, obstaclesNum * sizeof(double));
	for (int i = 0; i < obstaclesNum; i++) 
	{
		obstaclesMovingDirection[i] = ((double)(rand() % 35800) + 100.0) / 100.0;
	}
	POBSTACLE* obstacles = (POBSTACLE*)malloc(obstaclesNum * sizeof(POBSTACLE));
	RtlZeroMemory(obstacles, obstaclesNum * sizeof(POBSTACLE));
	fors(
		obstaclesNum,
		initializeObstacles(&obstacles[j], obstaclesCenterPos[j], obstacleSize, obstacleVelocity, obstaclesMovingDirection[j]);
	);

	size_t surroundingAttempCount = 0x0;
	size_t patience = huntersNum * 20000;

	ExFreeMemoryToNULL((PVOID*)&obstaclesMovingDirection);
	ExFreeMemoryToNULL((PVOID*)&obstaclesCenterPos);

	while (1)
	{
#ifdef _DEBUG
		printf_cyan("******************************************************\n");
#endif
		if (!isSurroundingSuccess(hunter, target)) // !isSurroundingSuccess(hunter, target)
		{
			movingObstaclesRandomly((POBSTACLE (*)[obstaclesNum])&obstacles);
			fors(
				huntersNum,
				_SURROUNDING_changingHunterInfomationByRewardFunctionAndEnvironment(&hunter[j], target, obstacles);
			);
			if (surroundingAttempCount % 250 == 0)
			{
				printf("未收敛. surroundingAttempCount: %zu\n", surroundingAttempCount);
				fors(
					huntersNum,
					printf("[当前距离/角度信息] %zu -> %lf\t%lf\n", j, calculate_P$P_distance(hunter[j]->pos, target->pos), calculate_P$P$P_unsignedAngle(hunter[j]->pos, target->pos, CONTAINING_RECORD(hunter[j]->hunterListEntry.Flink, HUSV, hunterListEntry)->pos) - PHASE_ANGLE);
				);
			}
			movingNormalTargetRandomly(&target);
			makePythonFile(fp, hunter, target, obstacles);
#ifdef _DEBUG
			for (size_t j = 0; j < huntersNum; j++)
			{
				size_t cnt = 0;
				for (size_t i = 0; i < obstaclesNum; i++)
				{
					if (calculate_P$P_distance(hunter[j]->pos, obstacles[i]->center) <= 6.0 * obstacles[0]->size + 5.0 * Rh)
					{
						cnt++;
					}
				}
				//printf("[当前捕食者的视野内障碍数目] %zu -> %zu\n", j, cnt);
			}
			fors(
				obstaclesNum,
				showObstacle(obstacles[j]);
			);
			double neighborDistance[huntersNum] = { 0.0 };
			fors(
				huntersNum,
				neighborDistance[j] = calculate_P$P_distance(hunter[j]->pos, getFlink(j)->pos);
				printf("%zu -> %lf with safety neighbor distance -> %lf\n", j, neighborDistance[j], 1.1 * Rh);
				if (neighborDistance[j] <= 1.1 * Rh)
				{
					printf_green("寄了\n");
				}
			);
#endif
			surroundingAttempCount++;
			if (surroundingAttempCount % patience == 0)
			{
				system("pause");
			}
			continue;
		}
		else
		{
			printf_pink("已经收敛！\n");
			printf("迭代次数: %zu\n", surroundingAttempCount);
			fors(
				huntersNum,
				printf("[当前距离/角度信息] %zu -> %lf\t%lf\n", j, calculate_P$P_distance(hunter[j]->pos, target->pos), calculate_P$P$P_unsignedAngle(hunter[j]->pos, target->pos, CONTAINING_RECORD(hunter[j]->hunterListEntry.Flink, HUSV, hunterListEntry)->pos) - PHASE_ANGLE);
			);
			makePythonFile(fp, hunter, target, obstacles);
#ifdef _DEBUG
			printf_yellow("\n最后一次显示的距离信息可能不符合条件<所有捕食者到目标的距离全都大于mostSafetyRadius + 1 * RtN + 1 * Rh>\n\n这是正常的，因为打印此消息时目标已经进行了一次移动！\n");
#endif
			QAQ;
			break;
		}
	}
	//流程正常结束，释放申请的内存，退出程序
	fors(
		obstaclesNum,
		ExFreeMemoryToNULL((PVOID*)&obstacles[j]);
	);
	ExFreeMemoryToNULL((PVOID*)&obstacles);
	fors(
		huntersNum,
		ExFreeMemoryToNULL((PVOID*)&hunter[j]);
	);
	ExFreeMemoryToNULL((PVOID*)&target);
	fclose(fp);
#ifdef _DEBUG
	printf("Curr file name: %s\n", (const char*)__FILE__);
#endif
	return 0;
}
