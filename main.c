#include "HuntingSimulation.h"

#pragma warning(disable: 6387 6011)

#define huntersBeginFactor 3.0
#define obstaclesBeginFactor 1.2

int main()
{
	srand((UINT)time(NULL));

	GA globalAttributes = { 0 };
	globalAttributes._huntersNum = 10;
	globalAttributes._obstaclesNum = 1;
	globalAttributes._targetNormalVelocity = 5.0;
	globalAttributes._targetSafetyRadius = 800.0;
	globalAttributes._ANGLE_REGION_SIZE = 360;
	globalAttributes._DIS_REGION_SIZE = 20;
	setGlobalAttributes(globalAttributes);

	FILE* fp = fopen("simulationData.txt", "w");
	PTUSV target = NULL;
	//如果更改目标的坐标，请注意捕食者的链表链接顺序.
	Point targetPos = { 16 , 9 }; // - rand() % 4
	initializeTargetUSV(
		&target,
		targetPos,
		g_targetNormalVelocity,
		g_targetSafetyRadius
	);
	PHUSV* hunter = (PHUSV*)malloc(g_huntersNum * sizeof(PHUSV)); RtlZeroMemory(hunter, g_huntersNum * sizeof(PHUSV));
	//从目标坐标的右侧射线方向逆时针旋转，第一个碰到的捕食者为链表头，最后一个碰到的为链表尾.
	//一定一定要保证链表顺序！只要围捕的时候撞上绝对是链表顺序问题！！！
	Point* hunterPos = (Point*)malloc(g_huntersNum * sizeof(Point)); RtlZeroMemory(hunterPos, g_huntersNum * sizeof(Point));
	double beginAngle = 20.0;
	double beginRadius = huntersBeginFactor * g_targetSafetyRadius;
	for (size_t j = 0; j < g_huntersNum; j++)
	{
		hunterPos[j].x = targetPos.x + beginRadius * cosa(beginAngle + 0.2 * (double)j);
		hunterPos[j].y = targetPos.y + beginRadius * sina(beginAngle + 0.2 * (double)j);
	}
	fors(
		g_huntersNum,
		initializeHunterUsv(
			&hunter[j],
			hunterPos[j],
			g_huntersVelocity,
			j
		);
	);
	buildCycleListByHuntersOriginalIndex(hunter); //补全了hunter结构的LIST_ENTRY项

	double obstacleSize = 400;
	double obstacleVelocity = 0.01;
	Point* obstaclesCenterPos = (Point*)malloc(g_obstaclesNum * sizeof(Point));
	RtlZeroMemory(obstaclesCenterPos, g_obstaclesNum * sizeof(Point));
	for (int i = 0; i < g_obstaclesNum; i++)
	{
		double angle = cvtAngle2Rad((double)(rand() % 360));
		double radius = ((double)rand() / RAND_MAX) * obstaclesBeginFactor * g_targetSafetyRadius;

		obstaclesCenterPos[i].x = targetPos.x + radius * cos(angle);
		obstaclesCenterPos[i].y = targetPos.y + radius * sin(angle);
	}
	double* obstaclesMovingDirection = (double*)malloc(g_obstaclesNum * sizeof(double));
	RtlZeroMemory(obstaclesMovingDirection, g_obstaclesNum * sizeof(double));
	for (int i = 0; i < g_obstaclesNum; i++)
	{
		obstaclesMovingDirection[i] = ((double)(rand() % 35800) + 100.0) / 100.0;
	}
	POBSTACLE* obstacles = (POBSTACLE*)malloc(g_obstaclesNum * sizeof(POBSTACLE));
	RtlZeroMemory(obstacles, g_obstaclesNum * sizeof(POBSTACLE));
	fors(
		g_obstaclesNum,
		initializeObstacles(&obstacles[j], obstaclesCenterPos[j], obstacleSize, obstacleVelocity, obstaclesMovingDirection[j]);
	);

	size_t surroundingAttempCount = 0x0;
	size_t patience = g_huntersNum * 20000;

	ExFreeMemoryToNULL((PVOID*)&obstaclesMovingDirection);
	ExFreeMemoryToNULL((PVOID*)&obstaclesCenterPos);

	while (1)
	{
#ifdef _DEBUG
		printf_cyan("******************************************************\n");
#endif
		if (!isSurroundingSuccess(hunter, target, obstacles)) // !isSurroundingSuccess(hunter, target)
		{
			makePythonFile(fp, hunter, target, obstacles);
			fors(
				g_huntersNum,
				_SURROUNDING_changingHunterInfomationByRewardFunctionAndEnvironment(&hunter[j], target, obstacles);
			);
			if (surroundingAttempCount % 50 == 0)
			{
				fors(
					g_huntersNum,
					printf("[当前距离/角度信息] %zu -> %lf\t%lf\n", j, calculate_P$P_distance(hunter[j]->pos, target->pos), calculate_P$P$P_unsignedAngle(hunter[j]->pos, target->pos, CONTAINING_RECORD(hunter[j]->hunterListEntry.Flink, HUSV, hunterListEntry)->pos) - g_PHASE_ANGLE);
				);
				printf("未收敛. surroundingAttempCount: %zu\n", surroundingAttempCount);
			}
			movingNormalTargetRandomly(&target);
			movingObstaclesRandomly(&obstacles);
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
				g_huntersNum,
				printf("[当前距离/角度信息] %zu -> %lf\t%lf\n", j, calculate_P$P_distance(hunter[j]->pos, target->pos), calculate_P$P$P_unsignedAngle(hunter[j]->pos, target->pos, CONTAINING_RECORD(hunter[j]->hunterListEntry.Flink, HUSV, hunterListEntry)->pos) - g_PHASE_ANGLE);
			);
			makePythonFile(fp, hunter, target, obstacles);
			printf_yellow("\n最后一次显示的距离信息可能不符合收敛性条件\n这是正常的，因为打印此消息时目标已经进行了一次移动！\n");
			QAQ;
			break;
		}
	}
	//流程正常结束，释放申请的内存，退出程序
	fors(
		g_obstaclesNum,
		ExFreeMemoryToNULL((PVOID*)&obstacles[j]);
	);
	ExFreeMemoryToNULL((PVOID*)&obstacles);
	ExFreeMemoryToNULL((PVOID*)&hunterPos);
	fors(
		g_huntersNum,
		ExFreeMemoryToNULL((PVOID*)&hunter[j]);
	);
	ExFreeMemoryToNULL((PVOID*)&hunter);
	ExFreeMemoryToNULL((PVOID*)&target);
	fclose(fp);
#ifdef _DEBUG
	printf("Curr file name: %s\n", (const char*)__FILE__);
#endif
	return 0;
}
