#include "HuntingSimulation.h"

static size_t huntersNumRealTime = (size_t)huntersNum;

#define getFlink(j) CONTAINING_RECORD(hunter[j]->hunterListEntry.Flink, HUSV, hunterListEntry)
#define getBlink(j) CONTAINING_RECORD(hunter[j]->hunterListEntry.Blink, HUSV, hunterListEntry)

SIZE_T huntersNums = 6;

typedef struct _OBSTACLE
{
	Point center;
	double size;
	double v;
	V movingDirection;
}REC_OBSTACLE, *PREC_OBSTACLE;

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
	double beginAngle = 20.0;
	double beginRadius = 15.0 * targetSafetyRadius;
	for (size_t j = 0; j < huntersNum; j++)
	{
		hunterPos[j].x = targetPos.x + beginRadius * cosa(beginAngle + 20.0 * (double)j);
		hunterPos[j].y = targetPos.y + beginRadius * sina(beginAngle + 20.0 * (double)j);
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
	fors(
		huntersNum,
		_SURROUNDING_getHuntersTwoEIByHuntersLocAndHeadDirection(&hunter[j], target); //补全了hunter结构的EI项
		_SURROUNDING_getRewardPointsForHunterByHuntersEI(hunter[j]);
		QAQ;
	);

	size_t surroundingAttempCount = 0x0;

	BOOLEAN fullyQueueMark = 1;

	while (1)
	{
#ifdef _DEBUG
		printf_cyan("******************************************************\n");
#endif
		if (!isSurroundingSuccess(hunter, target)) //!isSurroundingSuccess(hunter, target)
		{
			fors(
				huntersNum,
				_SURROUNDING_changingHunterInfomationByRewardFunction(&hunter[j], target);
			);
			printf("未收敛. surroundingAttempCount: %zu\n", surroundingAttempCount);
			fors(
				huntersNum,
				printf("[当前距离信息] %zu -> %lf\n", j, calculate_P$P_distance(hunter[j]->pos, target->pos));
			);
			movingNormalTargetRandomly(&target);
#ifdef _DEBUG
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
			makePythonFile(fp, hunter, target);
			QAQ;
			surroundingAttempCount++;
			if (surroundingAttempCount == 1000)
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
				printf("[当前距离信息] %zu -> %lf\n", j, calculate_P$P_distance(hunter[j]->pos, target->pos));
			);
#ifdef _DEBUG
			printf_yellow("\n最后一次显示的距离信息可能不符合条件<所有捕食者到目标的距离全都大于mostSafetyRadius + 1 * RtN + 1 * Rh>\n\n这是正常的，因为打印此消息时目标已经进行了一次移动！\n");
#endif
			makePythonFile(fp, hunter, target);
			QAQ;
			break;
		}
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
#ifdef _DEBUG
	printf("Curr file name: %s\n", (const char*)__FILE__);
#endif
	return 0;
}
